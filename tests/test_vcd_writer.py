# Copyright (c) Fredrik Andersson, 2023-2026
# All rights reserved

"""
The VcdWriter writes the same files as pyvcd, the VCD writer it replaces in DigSim. Runs
without DigSim: pip install <wheel> pyvcd==0.5.0 pytest, then pytest tests
"""

import pytest
from vcd import VCDWriter

from digsim_native import VcdWriter


# The variables and values DigSim writes: one-bit wires, buses (also wider than 64 bits)
# and strings (the names of states), with 0, 1, "X", "Z" and ints
CHANGES = [
    (0, "bit", "X"),
    (0, "bus", "X"),
    (0, "state", "IDLE"),
    (0, "bit", 0),
    (5, "bit", 1),
    (5, "bus", 0xA5),
    (5, "wide", 2**79 + 1),
    (5, "other", "Z"),
    (6, "bus", 0xA5),
    (7, "bus", "X"),
    (7, "state", "RUN state\t2 \\ end"),
    (9, "bit", 1),
    (12, "wide", "X"),
    (12, "bit", 0),
    (12, "bus", "Z"),
    (20, "bus", 3),
]


def _write(writer_class, path, changes=CHANGES):
    """A VCD file written by a writer class"""
    with open(path, "w", encoding="utf-8") as file:
        writer = writer_class(file, timescale="1 ns", date="today")
        variables = {
            "bit": writer.register_var("top.cell", "A", "wire", size=1),
            "bus": writer.register_var("top", "bus", "wire", size=8),
            "wide": writer.register_var("top", "wide", "wire", size=80),
            "state": writer.register_var("top", "state", "string", init=""),
            "other": writer.register_var("other", "B", "wire", size=1),
        }
        for time_ns, name, value in changes:
            writer.change(variables[name], time_ns, value)
        writer.close()
    return path.read_bytes()


def test_like_pyvcd(tmp_path):
    """The same file as pyvcd"""
    assert _write(VcdWriter, tmp_path / "native.vcd") == _write(VCDWriter, tmp_path / "py.vcd")


def test_only_time_zero(tmp_path):
    """Closed at time 0: the header and the values"""
    changes = [(0, "bit", 1), (0, "bus", 7)]
    native = _write(VcdWriter, tmp_path / "native.vcd", changes)
    assert native == _write(VCDWriter, tmp_path / "py.vcd", changes)


@pytest.mark.parametrize("value", [256, -200, "2"])
def test_invalid_value(tmp_path, value):
    """A value that doesn't fit is a ValueError, like in pyvcd, and nothing is written"""
    for writer_class in (VcdWriter, VCDWriter):
        with open(tmp_path / "file.vcd", "w", encoding="utf-8") as file:
            writer = writer_class(file, timescale="1 ns", date="today")
            bus = writer.register_var("top", "bus", "wire", size=8)
            writer.change(bus, 1, 3)
            with pytest.raises(ValueError):
                writer.change(bus, 2, value)
            writer.change(bus, 3, 4)
            writer.close()
        text = (tmp_path / "file.vcd").read_text()
        assert "#2" not in text
        assert text.endswith("#3\nb100 !\n")


def test_duplicate_variable(tmp_path):
    """A variable registered twice in a scope is a KeyError, like in pyvcd"""
    with open(tmp_path / "file.vcd", "w", encoding="utf-8") as file:
        writer = VcdWriter(file, timescale="1 ns", date="today")
        writer.register_var("top", "A", "wire", size=1)
        with pytest.raises(KeyError):
            writer.register_var("top", "A", "wire", size=1)
