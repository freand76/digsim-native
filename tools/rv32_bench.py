# Copyright (c) Fredrik Andersson, 2023-2026
# All rights reserved

"""
Time the rv32 self test with the engine of DIGSIM_ENGINE. The first argument is the
riscv32/ folder (the RISC-V processor).

Usage: python native/tools/rv32_bench.py <riscv32 folder> rv32i|rv32im|rv32ic|rv32imc
       gates|functional <cycles>

The trace digest must be the same with both engines.
"""

import os
import sys
from pathlib import Path


riscv32 = Path(sys.argv[1])
sys.path.insert(0, str(riscv32))

from rv32sim.bench import Bench
from rv32sim.build import Config


isa = sys.argv[2]
functional = sys.argv[3] == "functional"
cycles = int(sys.argv[4])
config = Config(m="m" in isa[5:], c="c" in isa[5:])
image = (riscv32 / "programs" / "bin" / f"selftest_{isa}.bin").read_bytes()
bench = Bench(config, image, functional=functional)
result = bench.run(max_cycles=cycles, trace=True)
digest = hash(tuple(str(r) for r in result.trace)) & 0xFFFFFFFF
print(
    f"{os.environ.get('DIGSIM_ENGINE', 'native')} {isa} {sys.argv[3]}: {result.cycles} cycles"
    f" in {result.seconds:.2f} s = {result.cycles / result.seconds:,.0f} cycles/s,"
    f" trace {len(result.trace)} {digest:08x} (run with PYTHONHASHSEED=0)"
)
engine = getattr(getattr(bench, "circuit", None), "_engine", None)
if engine is not None:
    native, python, stale, calls, evals = engine.engine.stats()
    print(
        f"per cycle: native events {native / result.cycles:.0f}, python events"
        f" {python / result.cycles:.0f}, stale {stale / result.cycles:.0f}, python calls"
        f" {calls / result.cycles:.0f}, cell evaluations {evals / result.cycles:.0f}"
    )
