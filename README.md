# Native engine trial

A trial of moving the DigSim simulation to Rust (PyO3), as an optional package,
`digsim_native`, with the Python engine as the fallback. DigSim uses it when it can import
it, and simulates in Python as before when it can't.

```sh
native/build.sh                     # build the wheel and install it into ../.venv
DIGSIM_ENGINE=python uv run ...     # the Python engine (also without the package)
DIGSIM_ENGINE=queue uv run ...      # trial 1: only the event queue is native
DIGSIM_ENGINE=native uv run ...     # trial 2: the native engine (the default)
```

## The two trials

**Trial 1, only the engine** (`EventQueue`): the event queue and the check for stale
events in Rust, the loop, the ports and all components in Python. The queue is CPython's
heapq algorithm step by step, so events at the same time keep their order.

**Trial 2, the circuit handling too** (`Engine`): Rust runs the event loop, and simulates
the Yosys gate cells (the gates, muxes and the D flip-flops with reset and enable), the
functional cells (`native/src/word.rs`: arithmetic, comparisons, shifts, muxes, the
decoders, flip-flops and latches) and the bus ports (`PortMultiBitWire` up to 64 bits and
their `PortWireBit` bits). Every other
component and port stays in Python: the engine calls `set_value()` and `delta_cycle()` of
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
- A number type that is a `u64` while it fits (`native/src/bits.rs`, `BigUint` only for
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

## Not exact: the fast and cycle modes

Opt-in with `DIGSIM_ENGINE`, the exact mode stays the default. They are native only: the
Python fallback simulates exactly, so a circuit could give other results without the
native package.

- **`fast`**: the same simulation, with gate delays and glitches, but not the same order
  of events at the same time. The queue is a bucket for each time, first in first out
  (`queue.rs`), instead of heapq. An output that keeps its value gets no event, and bus
  ports get events only while a VCD file is written.
- **`cycle`**: `fast`, and the gate cells are cycle-based. The combinational gate cells are
  sorted in levels once. A changed input marks its cell, and one settle event in the time
  step evaluates the marked cells level by level, with no delay. The flip-flops that a
  clock edge triggers sample their inputs together before their outputs change, so
  synchronous designs have no races. Gate cells with a loop (a latch of gates) keep the
  `fast` mode. Functional cells stay event-driven.

Cycles per second (rv32 self test traces as with the exact mode):

| Design | Python | Exact | Fast | Cycle |
| --- | ---: | ---: | ---: | ---: |
| 6502, gates | 977 | 10,208 | 14,996 | 24,322 |
| 6502, functional | 1,326 | 13,448 | 13,652 | 13,615 |
| rv32i, gates | 117 | 1,626 | 3,147 | 5,199 |
| rv32imc, gates | about 30 | 306 | 495 | 3,909 |
| rv32i, functional | 603 | 6,300 | 6,133 | 6,100 |

- `cycle` gives the most for deep logic: the dividers of rv32imc (1,211 gates deep) ripple
  and glitch with gate delays, cycle-based each cell is evaluated once. rv32imc gates are
  13x the exact mode, 130x Python.
- The functional netlists gain nothing: they have few events and no gate cells.
- The rv32 self tests (all four ISAs, gates and functional) give the same instruction
  traces, and the riscv32 suite (489 tests) passes in both modes.
- What differs, from the DigSim tests: in `fast` the 74193 counter is wrong. Its direction
  flip-flop is clocked by `Up & Down` and samples `~Up`, and the AND and the NOT change at
  the same time after `Up`: which one comes first decides what it samples, and only the
  order of heapq gives the intended count. In `cycle` its gate netlist is right again (the
  flip-flops sample first), the functional one is still wrong (functional cells are not
  cycle-based), and a gate output changes at once, not after its delay.
- In the cycle mode the ports of the gate cells are not in a VCD file, they change without
  events.

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

## Packaging, with the Python fallback

The Rust part fits as a separate optional package, `digsim-native`, built with maturin:

- The abi3 build (`abi3-py311`) is one wheel per platform for every Python from 3.11, so
  a release is about 5 wheels (Linux x86_64 and aarch64, macOS x86_64 and arm64, Windows
  x86_64), built in CI with `maturin-action`.
- `digsim-logic-simulator` stays a pure Python wheel. It could depend on the native
  package as an extra, `pip install digsim-logic-simulator[native]`, or depend on it
  on the platforms with wheels (environment markers), and fall back to Python elsewhere.
- One package with both, where the Rust extension is optional at build time, is possible
  too (setuptools-rust with `optional=True`), but then pip builds from the sdist on
  platforms without a wheel, which needs a Rust compiler or silently falls back.

## Not done in the trial

- Some cells are Python: the gate tristate buffer, latches and the other gate flip-flops
  (`_ALDFF_`, `_DFFSR_`), and the functional `$alu`, `$lcu`, `$fa`, `$macc_v2`, `$lut`,
  `$sop`, `$bweqx`, `$concat`, `$slice` and `$tribuf`. A netlist with them works, those
  cells are slower, and functional cells linked as words to one of them stay Python too.
- After the wires change, the engine stays off until the next `init()`. It could build a
  new engine from the current state instead.
- The engine state is behind a mutex (the GUI simulates in a thread). Without the GIL
  (free-threaded Python) the Python ports would need more care.
