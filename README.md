# digsim-native

The native simulation engine of [DigSim](https://github.com/freand76/digsim), in Rust
(PyO3). DigSim uses it when it can import `digsim_native`, and simulates in Python when
it can't. Both engines give the same results (the settled values, the native engine has no
delays inside a Yosys component), the native one 45 to 100 times faster on Yosys netlists.

## Install

The wheels are on [PyPI](https://pypi.org/project/digsim-native/) and the
[releases](https://github.com/freand76/digsim-native/releases), one per platform for every
Python from 3.11: Linux x86_64 and aarch64 (glibc and musl), Windows x64 and arm64, macOS
arm64 and x86_64.

```sh
pip install digsim-native
DIGSIM_ENGINE=python python ...     # the Python engine, with the package installed
```

## What it does

Rust runs the event loop and simulates every Yosys gate and functional cell, GND, VDD,
the bus ports up to 64 bits and one-bit ports with several drivers. Every other component
stays in Python and is called from the same event queue. When the wires change during a
simulation, the engine hands its state back to Python until the next `init()`.

A Yosys component is one compiled block (`src/block.rs`), simulated like a compiled
simulator (Verilator) runs its logic: the cell outputs are slots of words (the bits and
the undefined bits), the combinational cells are evaluated in the order of their ranks,
once each when a bit they read changed, and the flip-flops update their outputs in the
next round, like non-blocking assignments. The common cells are computed on 64-bit
words (the others by the cells of `src/word.rs`, so the values are the same), the gates by
a table, and a block of gates evaluates all its gates in a sweep when most of them
changed. Inside a block there are no delays: its outputs change 1 ns after an input. A
netlist with a combinational loop, tristate buffers or inout ports keeps one native cell
for each of its cells, with events, and so do the native cells outside blocks (with
levels instead of delays between them, see `src/levels.rs`). With `levels=False`
(`DIGSIM_BLOCKS=0` in DigSim) every cell has its delay and events, exactly like in Python.

`cargo build --release --features prof` adds time counters (`Engine.profile()`) and
`Engine.block_replay()`, which times the calls of a block again without Python.

## Build

```sh
pip install maturin
maturin build --release              # the wheel, in target/wheels/
pip install target/wheels/*.whl      # into the environment of DigSim
cargo fmt --check && cargo clippy --release -- -D warnings
```

The tests are in DigSim (its pytest, and `tools/native/`).

## Releases

1. Bump `version` in `Cargo.toml` and `pyproject.toml`, and the pin in DigSim.
2. Push a tag `vX.Y.Z`. The `Wheels` workflow builds and tests the wheels and attaches
   them to a release.
3. To upload them to PyPI: `gh workflow run pypi.yaml -f tag=vX.Y.Z`. PyPI trusts the
   workflow (Trusted Publishing: publisher `freand76/digsim-native`, workflow
   `pypi.yaml`, environment `pypi`), so it needs no token.

The interface to DigSim isn't stable, so DigSim pins the version it was written for.
