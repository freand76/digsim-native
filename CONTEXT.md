# Context: the native engine (branch rust-engine)

Where this work stands, for picking it up later. `README.md` next to this file has the
measurements and findings in full.

## The goal

Make DigSim simulate faster by moving parts of it to Rust, as an optional package, with
the Python engine as the fallback on hosts where the package isn't installed.

## Status

- A working native engine, `digsim_native` (PyO3), that gives **the same results as
  Python**: the same order of events, the same output and byte-identical VCD files.
  About 10x Python on Yosys netlists, gates and functional.
- Two engines only: native (the default when the package can be imported) and Python
  (`DIGSIM_ENGINE=python`, or without the package).
- The branch is made from `main` at 3b30fbc (release 1.8.0), and not merged. The
  package isn't built in CI or published yet, see "Delivering the wheels".

## Commits

1. 97956b6: the trial. Trial 1 moved only the event queue to Rust (1.2x, since removed).
   Trial 2 is the engine: the event loop, the Yosys gate cells and the bus ports in Rust.
2. bd99817: the functional Yosys cells (`$add`, `$mux`, `$dff` ...) in Rust too.
3. 83e0ce2: a faster event loop (lock without atomics, 24-byte events, values in a slab,
   flat fan-out lists): 1.4x on gate netlists.
4. cdba95e: the `fast` and `cycle` modes, which give up the exact order of events.
5. c2570f8: `fast`, `cycle` and the queue-only mode removed again. Only exact results.
6. This file and `tools/`.
7. 060e317, 89a800a: the RISC-V processor in `riscv32/`, cherry-picked from the
   `synth-parameters` branch (f907aa3, f428a59).

## How it works

- `src/digsim/circuit/_native.py` imports `digsim_native` if it can. `Circuit.init()`
  builds a `NativeEngine` from the circuit: the ports, the wires, and the cells it can
  simulate (`_GATES`, `_FLIP_FLOPS`, `_FUNCTIONAL`, buses up to 64 bits).
- Everything else stays a Python component (the GUI components, memories, user
  components, a few Yosys cells listed in the README). The engine calls their ports from
  the same queue, and they call back through `Circuit.add_event()`.
- The ports the engine simulates get a subclass while it runs: their `_value` property
  reads the engine and `set_value` sets the value in the engine. Python code that reads
  them (the VCD writer, the GUI) works unchanged.
- The queue is CPython's heapq algorithm step by step (`src/queue.rs`), comparing only the
  times, so events at the same time come out in the same order as in Python. Gate-level
  designs have races where that order shows. Stale events are skipped like in Python.
- When the wires change during a simulation (the GUI connects a wire), the engine stops
  and hands its events, values and flip-flop states back to Python, which goes on until
  the next `init()`.
- `src/bits.rs`: a number that is a `u64` while it fits and `BigUint` above it. With
  `BigUint` everywhere the functional cells were slower than the gates.
- `src/lock.rs`: the engine state is guarded by a flag, not a mutex, and relies on the
  GIL. abi3 modules aren't loaded by free-threaded Python, which then uses Python.

## Files

- `native/`: the Rust package (`Cargo.toml`, `Cargo.lock`, `pyproject.toml` for
  maturin, `src/`), `build.sh`, `README.md`, this file
- `native/tools/compare_engines.sh`: runs the 6502 example (gates, functional, with and
  without `--signals`) and `rewire_6502.py` in both engines and compares the output and
  VCD files byte for byte
- `native/tools/rewire_6502.py`: a 6502 that gets a second driver on RDY halfway, to
  check the handover from the native engine to Python
- `native/tools/rv32_bench.py`: times the rv32 self test, `python native/tools/rv32_bench.py
  riscv32 rv32i gates 3000`
- `riscv32/`: the RISC-V processor (from the `synth-parameters` branch), designed and
  verified with DigSim, the biggest benchmark of the engine. Its tests:
  `uv run --with pytest pytest riscv32/tests` (minutes)
- `src/digsim/circuit/_native.py` (new) and `_circuit.py` (uses the engine in `init()`,
  `run()`, `process_single_event()`, `add_event()`)

## Build and check

```sh
native/build.sh                       # build the wheel, install it into .venv (offline)
uv run --with pytest pytest           # 997 tests, native engine
DIGSIM_ENGINE=python uv run --with pytest pytest
sh native/tools/compare_engines.sh    # must print "The engines agree"
cd native && cargo clippy --release
```

Also run before merging: every example in both engines against a `git archive` copy of
`main` (the CLAUDE.md rule), `tools/gui_fuzz.py --seeds 3,11,21 --steps 300`,
`tools/gui_editor_check.py` and the riscv32 suite. All of them passed at 83e0ce2, the
engine code is the same now.

`uv run` from the repo root reinstalls the project but keeps the wheel in `.venv`. A new
virtual environment needs `native/build.sh` again, or it silently runs Python.

## Results

Cycles per second, Python 3.15, x86_64:

| Design | Python | Native |
| --- | ---: | ---: |
| 6502, gates | 977 | 10,208 |
| 6502, functional | 1,326 | 13,448 |
| rv32i, gates | 117 | 1,626 |
| rv32imc, gates | about 30 | 306 |
| rv32i, functional | 603 | 6,300 |

Writing a VCD file of every port (`--signals`) is Python and takes most of the time, the
gain is then much smaller.

## Decisions

- **The engine and the cells both had to move.** Only the queue in Rust gained 1.2x: each
  event still ran five to six Python calls.
- **Exact only.** `fast` (a FIFO queue per time) gave up to 2x more and `cycle` (gate
  cells evaluated in levels without delay) up to 13x more on rv32imc gates. They were
  removed: results would depend on whether the native package is installed, designs with
  races (the 74193 counter) behaved differently, and `cycle` hides the gate delays that
  DigSim shows. The numbers are kept in the README.
- **Not everything moves.** Python components work unchanged, so more of them can move
  later, when they matter for speed.

## Not done

- Some cells are still Python (the gate tristate buffer, latches, `_ALDFF_`, `_DFFSR_`,
  `$alu`, `$lcu`, `$fa`, `$macc_v2`, `$lut`, `$sop`, `$bweqx`, `$concat`, `$slice`,
  `$tribuf`). They work, they are slower.
- After the wires change, the engine stays off until the next `init()`. It could build a
  new engine from the current state.
- GUI circuits of single gates use the Python gate components, which the engine calls
  back. Native versions of those would speed up the GUI's own gate circuits.
- No CHANGELOG line, no docs, no CLAUDE.md update yet.

## Delivering the wheels

The build is abi3 (`abi3-py311`), one wheel per OS and CPU for every Python from 3.11.

- A workflow with `PyO3/maturin-action`, one job per target: Linux x86_64 and aarch64
  (`manylinux: "2014"`, the local build is `manylinux_2_34`, too new for older distros),
  Windows x64, macOS arm64 and x86_64 (or `universal2`). The wheels are uploaded as
  artifacts, the maintainer uploads them with twine.
- Test each wheel in the matrix: pytest in both engines, and `compare_engines.sh`.
- A separate package, `digsim-native`. `digsim-logic-simulator` stays pure Python and
  depends on `digsim-native==x.y.z` with environment markers for the platforms with
  wheels. No sdist for `digsim-native` (pip would try to compile it, which needs Rust).
- Release: bump `native/pyproject.toml`, `native/Cargo.toml` and the pin together, upload
  `digsim-native` before the main package.
- Add license and readme metadata to `native/pyproject.toml`, and document the two
  engines (CLAUDE.md, contributing docs, CHANGELOG).
