#!/bin/sh
# Build the native engine and install it into the project venv (a trial)
set -e
cd "$(dirname "$0")"
rm -rf dist
CARGO_NET_OFFLINE=true uv tool run maturin build --release --out dist
uv pip install --python ../.venv/bin/python --reinstall dist/*.whl
