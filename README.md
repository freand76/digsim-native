# digsim-native

The native simulation engine of [DigSim](https://github.com/freand76/digsim), in Rust
(PyO3). DigSim uses it when it can import `digsim_native`, and simulates in Python as
before when it can't.

There are two engines, Python and native, and they give the same results: the same order
of events, the same output and byte-identical VCD files. The native engine is only faster,
about 10x on Yosys netlists.

## Install

The wheels are on the [GitHub releases](https://github.com/freand76/digsim-native/releases),
one per platform for every Python from 3.11 (abi3): Linux x86_64 and aarch64 (glibc and
musl), Windows x64 and arm64, macOS arm64 and x86_64.

```sh
pip install digsim-native==0.1.0 \
    --find-links https://github.com/freand76/digsim-native/releases/expanded_assets/v0.1.0
DIGSIM_ENGINE=python python ...     # the Python engine, also with the package installed
```

Free-threaded Python doesn't load abi3 modules, DigSim then simulates in Python.

## Build

```sh
pip install maturin
maturin build --release              # the wheel, in target/wheels/
pip install target/wheels/*.whl      # into the environment of DigSim
cargo fmt --check && cargo clippy --release -- -D warnings
```

The tests are in DigSim: its test suite and the engine comparisons in its
`tools/native/` run both engines and compare them byte for byte.

## Releases

1. Bump `version` in `Cargo.toml` and `pyproject.toml` (the same version), and the pin
   in DigSim.
2. Push a tag `vX.Y.Z`. The `Wheels` workflow builds and tests the wheels and attaches
   them to a GitHub release of the tag.

The interface between DigSim and the engine (`Engine` and its arguments) isn't stable:
DigSim pins the version it was written for.

## The two trials



**Trial 1, only the engine** (removed): the event queue and the check for stale events in
Rust, the loop, the ports and all components in Python. The queue is CPython's heapq
algorithm step by step, so events at the same time keep their order.

**Trial 2, the circuit handling too** (`Engine`): Rust runs the event loop, and simulates
every Yosys gate cell (the gates, muxes, tristate buffers, flip-flops and latches) and
functional cell (`src/word.rs`: arithmetic, comparisons, shifts, muxes, the
decoders, flip-flops, latches, `$alu`, `$macc_v2`, `$lut`, `$tribuf` ...), GND and VDD,
and the bus ports (`PortMultiBitWire` up to 64 bits and their `PortWireBit` bits). A
one-bit port with several drivers (a tristate bus) gets its value from them in Rust, as
`resolve()` does. Every other component and port stays in Python: the engine calls `set_value()` and `delta_cycle()` of
Python ports from the same queue, and Python calls back into the engine through
`Circuit.add_event()` and through the ports the engine took over. Those ports get a
subclass while the engine runs, their `_value` is read from the engine, so Python code
that reads them (the VCD writer, a port with several drivers, the GUI) sees the native
values without copying them.

When the wires change during a simulation (the GUI connects a wire), the engine hands its
events, values and flip-flop states back to Python, which goes on from there until the
next `init()`.

## Results

Python 3.15.0, x86_64. Cycles per second, more is better:

| Design | Python | Native queue | Native engine |
| --- | ---: | ---: | ---: |
| 6502, gates (`examples/yosys_6502`) | 977 | 1,135 | 9,643 (10x) |
| 6502, functional | 1,326 | | 13,529 (10x) |
| rv32i, gates (riscv32/, 3000 cycles) | 117 | | 1,215 (10x) |
| rv32i, functional (self test) | 603 | | 5,907 (10x) |
| rv32imc, functional (self test) | 445 | | 4,380 (10x) |

With Python the functional netlist is a little faster than the gates (fewer events), and
natively it stays faster: rv32i functional is 5x its gates. Steps on the way:

- Native gate cells, the bus ports in Python, and every changed value written back to its
  Python port after each run: 3.9x on the 6502 and 2.2x on rv32i gates.
- Native buses, and port values read from the engine instead of copied: 10x on the gates.
- Native functional cells with `num-bigint` values: 4x on the 6502 functional, slower
  than the gates. Every operation allocated, also for 8-bit words, and the parameters
  were looked up by name in each update.
- A number type that is a `u64` while it fits (`src/bits.rs`, `BigUint` only for
  wider words like the 1024-bit register file), arithmetic up to 64 bits in an `i128`,
  and the parameters read once: 10x.

Exactness, on the native engine:

- The VCD files of the 6502 example (gates, functional and `--signals`, which has every
  internal port of the netlist) are byte-identical to the Python engine's.
- All examples give the same output and VCD files.
- The rv32i instruction traces are identical.
- A 6502 that gets a second driver on RDY halfway, which stops the native engine, gives
  an identical VCD file.
- The functional 6502 VCD files (also `--signals`) are byte-identical.
- The rv32 self tests pass on the functional netlist for all four ISAs, with the same
  instruction traces, and the riscv32 test suite (489 tests, against a reference model,
  functional and gates) passes in 2 minutes 50 seconds.
- `pytest` (997 tests), `tools/gui_editor_check.py` and `tools/gui_fuzz.py` pass.

### Tuning the engine

On rv32i gates, the engine itself took most of the time, about 90 ns per event. Measured
with the native engine, cycles per second:

| Step | rv32i gates | rv32i functional | 6502 gates |
| --- | ---: | ---: | ---: |
| Before | 1,215 | 5,907 | 9,643 |
| A lock without atomics instead of a mutex (`lock.rs`, the GIL serializes) | 1,470 | 6,310 | 9,400 |
| 24-byte events, values in a slab, flat fan-out lists | 1,710 | 6,470 | 10,650 |
| Moving a hole in the heap instead of swapping | 1,690 | 6,470 | |
| The heap times in an array of their own (reverted, slower) | 1,550 | | |
| One array for the latest events, flat cell inputs | 1,700 | 6,320 | 10,430 |

1.4x on gate netlists, little on functional ones (they have few events). What is left of
the engine time on rv32i gates: the heap about 40%, Python components 20% (the memory of
the test bench), the gates 15%, and the rest the fan-out, the stale events and the
dispatch. The heap must be CPython's heapq, step by step, to keep the order of events at
the same time, and its cost is in the algorithm (a pop goes down to a leaf, through about
12 levels of a heap of 5,000 events), not in the memory layout. A queue that keeps events
at the same time in another order (buckets per time) would be faster, but gate-level
designs with races show that order, so it would change their VCD files and could change
their behavior.

## Not exact modes (tried and removed)

Two modes that give up the exact order were tried, as `DIGSIM_ENGINE=fast` and `cycle`:

- `fast`: a first in first out queue for each time instead of heapq, and no event for an
  output that keeps its value. 1.5x to 2x the exact engine on gate netlists.
- `cycle`: the gate cells of a netlist sorted in levels and evaluated once per time step
  with no delay, the flip-flops sampling together. 2.4x on the 6502 gates, 13x on the
  rv32imc gates (their dividers ripple and glitch with gate delays).
- Neither helps a functional netlist, it has few events.

They were left out:

- Only the native engine has them, so a circuit would give other results without the
  native package.
- A design with races behaves differently: the 74193 counter counts wrong in `fast`, and
  its functional netlist also in `cycle`.
- `cycle` has no gate delays or glitches, which DigSim shows, and the ports of the gate
  cells are not in the VCD file.

## Findings

- **Moving only the engine is not enough.** The queue was already C (heapq on float
  events), and each event still runs five to six Python calls: `delta_cycle()`,
  `update_wires()`, `set_value()`, `update()`, `set_value()` and `add_event()`. Trial 1
  gains 1.2x.
- **The gain comes from keeping the hot path out of Python**: gates, flip-flops, wires and
  buses. With those native, the rv32i gate netlist does 5,500 events and 7,000 cell
  evaluations per cycle in Rust and calls Python about 80 times, for the memory and the
  console of the test bench.
- **What is left in Python sets the limit.** With only the gate cells native, the
  functional netlist gained 1.5x, since its cells were Python components. With them
  native too it gains 10x.
- **Native code needs native data.** Unbounded numbers that allocate (`BigUint`) made
  the functional cells slower than the gates. Words that fit in 64 bits without
  allocating gave the 10x.
- **Not everything has to move.** Components in Python (the GUI components, the memory,
  user components) work unchanged and keep the event order exact, so they can move one
  at a time, when they matter for speed.

## Not done

- A bus port with several drivers (two tristate outputs of components on one bus) is
  resolved in Python. Inside a netlist a tristate bus is resolved per bit, natively.
- After the wires change, the engine stays off until the next `init()`. It could build a
  new engine from the current state instead.
- The engine state is behind a mutex (the GUI simulates in a thread). Without the GIL
  (free-threaded Python) the Python ports would need more care.
