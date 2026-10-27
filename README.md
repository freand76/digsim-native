# digsim-native

The native simulation engine of [DigSim](https://github.com/freand76/digsim), in Rust
(PyO3). DigSim uses it when it can import `digsim_native`, and simulates in Python when
it can't. Both engines give the same results, the native one about 10x faster on Yosys
netlists.

## Install

The wheels are on the [releases](https://github.com/freand76/digsim-native/releases), one
per platform for every Python from 3.11: Linux x86_64 and aarch64 (glibc and musl),
Windows x64 and arm64, macOS arm64 and x86_64.

```sh
pip install digsim-native==0.1.0 \
    --find-links https://github.com/freand76/digsim-native/releases/expanded_assets/v0.1.0
DIGSIM_ENGINE=python python ...     # the Python engine, with the package installed
```

## What it does

Rust runs the event loop and simulates every Yosys gate and functional cell, GND, VDD,
the bus ports up to 64 bits and one-bit ports with several drivers. Every other component
stays in Python and is called from the same event queue. When the wires change during a
simulation, the engine hands its state back to Python until the next `init()`.

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
