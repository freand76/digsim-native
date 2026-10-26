// Copyright (c) Fredrik Andersson, 2023-2026
// All rights reserved

//! Optional native simulation engine for DigSim (a trial).
//!
//! `Engine` runs the whole event loop: the Yosys gate cells and the bus ports it knows
//! are simulated here, every other component stays in Python and is called back from the
//! same queue. It keeps the exact order of the Python engine: the queue is CPython's
//! heapq algorithm, comparing only the times, so events at the same time come out in the
//! same order.

use pyo3::exceptions::PyValueError;
use pyo3::intern;
use pyo3::prelude::*;
use pyo3::types::{PyInt, PyString};
use std::collections::HashMap;

mod bits;
mod lock;
mod queue;
mod word;
use num_bigint::{BigInt, BigUint};

// The whole event loop, with native gate cells and bus ports

// Values of one-bit native ports: 0, 1, undefined ("X") and not driven ("Z")
const X: u8 = 2;
const Z: u8 = 3;

// Port kinds
const PY: u8 = 0; // A Python port, called back
const IN: u8 = 1; // Input of a native cell that updates the cell (PortIn)
const WIRE: u8 = 2; // Input of a native cell that doesn't (PortWire)
const BIT: u8 = 4; // A bit of a native bus (PortWireBit)
const BUS: u8 = 5; // A bus of bits, up to 64 (PortMultiBitWire)
const CELL_INPUT: u8 = 6; // A bit of an input of a functional cell (_CellInput)
const WORD: u8 = 7; // The word output of a functional cell (_WordOut)
                    // 3 is the output of a native cell (PortOutDelta), never driven

/// The value of a bus: a number, or all bits the same code (X or Z)
#[derive(Clone, Copy, PartialEq)]
enum BusValue {
    Int(u64),
    Code(u8),
}

enum Value {
    Bit(u8),
    Bus(BusValue),
    // A word of a functional cell output: the bits and, above them, the undefined bits
    Word(bits::Bits),
    Python(Py<PyAny>),
}

impl Value {
    fn clone_ref(&self, py: Python<'_>) -> Value {
        match self {
            Value::Bit(v) => Value::Bit(*v),
            Value::Bus(v) => Value::Bus(*v),
            Value::Word(v) => Value::Word(v.clone()),
            Value::Python(v) => Value::Python(v.clone_ref(py)),
        }
    }
}

/// The latest event of a port: seq (0 for none), time and value slot
#[derive(Clone, Copy, Default)]
struct Latest {
    seq: u64,
    time: i64,
    slot: u32,
}

// The value of an event is a code of a one-bit value (below SLOTS), or a slot in
// Inner.payloads for other values
const SLOTS: u32 = 4;

#[derive(Clone, Copy, PartialEq)]
enum Kind {
    Buf,
    Not,
    And,
    Nand,
    Or,
    Nor,
    Xor,
    Xnor,
    AndNot,
    OrNot,
    Mux,
    NMux,
    Mux4,
    Mux8,
    Mux16,
    Aoi3,
    Oai3,
    Aoi4,
    Oai4,
    Dff,
    Dffe,
    DffAsyncReset,
    DffeAsyncReset,
    Sdff,
    Sdffe,
    Sdffce,
    Tbuf,
    Const,
    Sr,
    Dlatch,
    DlatchReset,
    DlatchSr,
    Aldff,
    Aldffe,
    Dffsr,
    Dffsre,
}

const KINDS: [Kind; 36] = {
    use Kind::*;
    [
        Buf,
        Not,
        And,
        Nand,
        Or,
        Nor,
        Xor,
        Xnor,
        AndNot,
        OrNot,
        Mux,
        NMux,
        Mux4,
        Mux8,
        Mux16,
        Aoi3,
        Oai3,
        Aoi4,
        Oai4,
        Dff,
        Dffe,
        DffAsyncReset,
        DffeAsyncReset,
        Sdff,
        Sdffe,
        Sdffce,
        Tbuf,
        Const,
        Sr,
        Dlatch,
        DlatchReset,
        DlatchSr,
        Aldff,
        Aldffe,
        Dffsr,
        Dffsre,
    ]
};

// The inputs of flip-flops and latches, the ones a kind doesn't have aren't read
const C: usize = 0;
const D: usize = 1;
const E: usize = 2;
const R: usize = 3;
const S: usize = 4;
const L: usize = 5;
const AD: usize = 6;

struct Cell {
    kind: Kind,
    // The input ports, in the order of the kind; flip-flops and latches: C, D, E, R, S,
    // L, AD. The first of the input ports in Inner.cell_inputs.
    first: u32,
    output: u32,
    clock_edge: u8,
    reset_level: u8,
    reset_value: u8,
    enable_level: u8,
    set_level: u8,
    load_level: u8,
    old_clock: u8,
}

struct Bus {
    bits: Vec<u32>,
    value: BusValue,
}

/// resolve() in _port.py: the value of the drivers that drive, Z if none does, X if they
/// drive different values
fn resolve(values: impl Iterator<Item = u8>) -> u8 {
    let mut result = Z;
    for value in values {
        if value == Z {
            continue;
        }
        if result == Z {
            result = value;
        } else if result != value {
            return X;
        }
    }
    result
}

// Three-valued logic, the same as _yosys_atoms.py
#[inline]
fn not(a: u8) -> u8 {
    match a {
        0 => 1,
        1 => 0,
        _ => X,
    }
}

#[inline]
fn and(a: u8, b: u8) -> u8 {
    if a == 0 || b == 0 {
        0
    } else if a == 1 && b == 1 {
        1
    } else {
        X
    }
}

#[inline]
fn or(a: u8, b: u8) -> u8 {
    if a == 1 || b == 1 {
        1
    } else if a == 0 && b == 0 {
        0
    } else {
        X
    }
}

#[inline]
fn xor(a: u8, b: u8) -> u8 {
    if a > 1 || b > 1 {
        X
    } else {
        a ^ b
    }
}

#[inline]
fn mux(s: u8, a: u8, b: u8) -> u8 {
    match s {
        0 => a,
        1 => b,
        _ => {
            if a == b {
                a
            } else {
                X
            }
        }
    }
}

#[inline]
fn mux4(s: u8, t: u8, a: u8, b: u8, c: u8, d: u8) -> u8 {
    mux(t, mux(s, a, b), mux(s, c, d))
}

struct Inner {
    time: i64,
    seq: u64,
    kind: Vec<u8>,
    // The cell of a cell port, the bus port of a bit, the bus of a bus port, the
    // functional cell of a port of one
    owner: Vec<u32>,
    // Functional cells: the input and bit of an input bit (input << 20 | bit), the output
    // of a word port
    aux: Vec<u32>,
    value: Vec<u8>,
    // The ports each port drives: wired_targets[wired_start[p]..wired_start[p + 1]]
    wired_start: Vec<u32>,
    wired_targets: Vec<u32>,
    // The drivers of a port with several (resolve()): drivers[driver_start[p]..[p + 1]]
    driver_start: Vec<u32>,
    drivers: Vec<u32>,
    delay: Vec<i64>,
    toplevel: Vec<bool>,
    cells: Vec<Cell>,
    // The input ports of the cells, in the order of each kind
    cell_inputs: Vec<u32>,
    buses: Vec<Bus>,
    words: Vec<word::Cell>,
    heap: queue::Queue,
    // The latest event of each port: seq (0 for none), time and value slot
    latest: Vec<Latest>,
    // The values that aren't one bit, with the number of events and latest entries that
    // have them, and the free slots
    payloads: Vec<(Value, u32)>,
    free: Vec<u32>,
    // Statistics: events (native, Python), stale events, Python calls, cell evaluations
    stats: [u64; 5],
}

impl Inner {
    fn push(&mut self, port: u32, value: Value, delay: i64) {
        let time = self.time + delay;
        self.seq += 1;
        let seq = self.seq;
        let slot = match value {
            Value::Bit(code) => code as u32,
            // The event and the latest entry
            other => self.allocate(other, 2),
        };
        let p = port as usize;
        let old = self.latest[p];
        if old.seq != 0 {
            self.release(old.slot);
        }
        self.latest[p] = Latest { seq, time, slot };
        self.heap.push(time, queue::Entry { seq, port, slot });
    }

    fn allocate(&mut self, value: Value, count: u32) -> u32 {
        match self.free.pop() {
            Some(index) => {
                self.payloads[index as usize] = (value, count);
                index + SLOTS
            }
            None => {
                self.payloads.push((value, count));
                self.payloads.len() as u32 - 1 + SLOTS
            }
        }
    }

    fn release(&mut self, slot: u32) {
        if slot < SLOTS {
            return;
        }
        let index = slot - SLOTS;
        let entry = &mut self.payloads[index as usize];
        entry.1 -= 1;
        if entry.1 == 0 {
            entry.0 = Value::Bit(0);
            self.free.push(index);
        }
    }

    fn slot_value(&self, py: Python<'_>, slot: u32) -> Value {
        if slot < SLOTS {
            Value::Bit(slot as u8)
        } else {
            self.payloads[(slot - SLOTS) as usize].0.clone_ref(py)
        }
    }

    /// Do two slots have the same value
    fn same(&self, py: Python<'_>, a: u32, b: u32) -> PyResult<bool> {
        if a == b {
            return Ok(true);
        }
        if a < SLOTS || b < SLOTS {
            return Ok(false);
        }
        let a = &self.payloads[(a - SLOTS) as usize].0;
        let b = &self.payloads[(b - SLOTS) as usize].0;
        Ok(match (a, b) {
            (Value::Bus(a), Value::Bus(b)) => a == b,
            (Value::Word(a), Value::Word(b)) => a == b,
            (Value::Python(a), Value::Python(b)) => a.bind(py).eq(b.bind(py))?,
            _ => false,
        })
    }

    /// The new output of a cell, None if it doesn't set it (a flip-flop without an edge)
    fn evaluate(&mut self, c: usize) -> Option<u8> {
        let cell = &mut self.cells[c];
        let inputs = &self.cell_inputs[cell.first as usize..];
        let v = |i: usize| self.value[inputs[i] as usize];
        let enabled = |cell: &Cell| v(E) == cell.enable_level;
        let reset = |cell: &Cell| v(R) == cell.reset_level;
        let set = |cell: &Cell| v(S) == cell.set_level;
        use Kind::*;
        let out = match cell.kind {
            Buf => v(0),
            Not => not(v(0)),
            And => and(v(0), v(1)),
            Nand => not(and(v(0), v(1))),
            Or => or(v(0), v(1)),
            Nor => not(or(v(0), v(1))),
            Xor => xor(v(0), v(1)),
            Xnor => not(xor(v(0), v(1))),
            AndNot => and(v(0), not(v(1))),
            OrNot => or(v(0), not(v(1))),
            Mux => mux(v(2), v(0), v(1)),
            NMux => not(mux(v(2), v(0), v(1))),
            Mux4 => mux4(v(4), v(5), v(0), v(1), v(2), v(3)),
            Mux8 => {
                let (s, t) = (v(8), v(9));
                mux(
                    v(10),
                    mux4(s, t, v(0), v(1), v(2), v(3)),
                    mux4(s, t, v(4), v(5), v(6), v(7)),
                )
            }
            Mux16 => {
                let (s, t, u) = (v(16), v(17), v(18));
                mux(
                    v(19),
                    mux(
                        u,
                        mux4(s, t, v(0), v(1), v(2), v(3)),
                        mux4(s, t, v(4), v(5), v(6), v(7)),
                    ),
                    mux(
                        u,
                        mux4(s, t, v(8), v(9), v(10), v(11)),
                        mux4(s, t, v(12), v(13), v(14), v(15)),
                    ),
                )
            }
            Aoi3 => not(or(and(v(0), v(1)), v(2))),
            Oai3 => not(and(or(v(0), v(1)), v(2))),
            Aoi4 => not(or(and(v(0), v(1)), and(v(2), v(3)))),
            Oai4 => not(and(or(v(0), v(1)), or(v(2), v(3)))),
            Tbuf => match v(1) {
                1 => v(0),
                0 => Z,
                _ => X,
            },
            // GND and VDD, their outputs are set once by their default state
            Const => return None,
            Sr => {
                return if reset(cell) {
                    Some(0)
                } else if set(cell) {
                    Some(1)
                } else {
                    None
                };
            }
            Dlatch => return enabled(cell).then(|| v(D)),
            DlatchReset => {
                return if reset(cell) {
                    Some(cell.reset_value)
                } else {
                    enabled(cell).then(|| v(D))
                };
            }
            DlatchSr => {
                return if reset(cell) {
                    Some(0)
                } else if set(cell) {
                    Some(1)
                } else {
                    enabled(cell).then(|| v(D))
                };
            }
            Dff | Dffe | DffAsyncReset | DffeAsyncReset | Sdff | Sdffe | Sdffce | Aldff
            | Aldffe | Dffsr | Dffsre => {
                let clock = v(C);
                let edge = clock != cell.old_clock && clock == cell.clock_edge;
                let load = |cell: &Cell| v(L) == cell.load_level;
                let out = match cell.kind {
                    Dff => edge.then(|| v(1)),
                    Dffe => (edge && enabled(cell)).then(|| v(1)),
                    DffAsyncReset => {
                        if reset(cell) {
                            Some(cell.reset_value)
                        } else {
                            edge.then(|| v(1))
                        }
                    }
                    DffeAsyncReset => {
                        if reset(cell) {
                            Some(cell.reset_value)
                        } else {
                            (edge && enabled(cell)).then(|| v(1))
                        }
                    }
                    Sdff => edge.then(|| if reset(cell) { cell.reset_value } else { v(1) }),
                    Sdffe => {
                        if !edge {
                            None
                        } else if reset(cell) {
                            Some(cell.reset_value)
                        } else if enabled(cell) {
                            Some(v(1))
                        } else {
                            None
                        }
                    }
                    Sdffce => (edge && enabled(cell)).then(|| {
                        if reset(cell) {
                            cell.reset_value
                        } else {
                            v(1)
                        }
                    }),
                    Aldff => {
                        if load(cell) {
                            Some(v(AD))
                        } else {
                            edge.then(|| v(D))
                        }
                    }
                    Aldffe => {
                        if load(cell) {
                            Some(v(AD))
                        } else {
                            (edge && enabled(cell)).then(|| v(D))
                        }
                    }
                    Dffsr => {
                        if reset(cell) {
                            Some(0)
                        } else if set(cell) {
                            Some(1)
                        } else {
                            edge.then(|| v(D))
                        }
                    }
                    _ => {
                        if reset(cell) {
                            Some(0)
                        } else if set(cell) {
                            Some(1)
                        } else {
                            (edge && enabled(cell)).then(|| v(D))
                        }
                    }
                };
                cell.old_clock = clock;
                return out;
            }
        };
        Some(out)
    }
}

// A functional cell from Python: kind, inputs (width, update, bits, undefined bits),
// outputs (width, bit ports, bits with ports that drive something, word port, links, last
// value), parameters, Q, the clock level and if a tristate buffer drives
type WordCellArgs = (
    u8,
    Vec<(usize, bool, BigUint, BigUint)>,
    Vec<(
        usize,
        Vec<u32>,
        BigUint,
        Option<u32>,
        Vec<word::Link>,
        Option<(BigUint, BigUint)>,
    )>,
    HashMap<String, BigInt>,
    (BigUint, BigUint),
    u8,
    bool,
);

/// The simulation of a circuit, see digsim/circuit/_native.py
#[pyclass(frozen)]
struct Engine {
    inner: lock::Lock<Inner>,
    ports: lock::Lock<Vec<Py<PyAny>>>,
    // The Python objects of the values 0, 1, "X" and "Z"
    values: [Py<PyAny>; 4],
}

impl Engine {
    fn port<'py>(&self, py: Python<'py>, p: usize) -> Bound<'py, PyAny> {
        self.ports.lock()[p].bind(py).clone()
    }

    fn bus_object<'py>(&self, py: Python<'py>, value: BusValue) -> PyResult<Bound<'py, PyAny>> {
        Ok(match value {
            BusValue::Int(v) => v.into_pyobject(py)?.into_any(),
            BusValue::Code(c) => self.values[c as usize].bind(py).clone(),
        })
    }

    fn object<'py>(&self, py: Python<'py>, value: &Value) -> PyResult<Bound<'py, PyAny>> {
        Ok(match value {
            Value::Bit(v) => self.values[*v as usize].bind(py).clone(),
            Value::Bus(v) => self.bus_object(py, *v)?,
            Value::Word(v) => BigUint::from(v).into_pyobject(py)?.into_any(),
            Value::Python(v) => v.bind(py).clone(),
        })
    }
}

fn bit_code(value: &Bound<'_, PyAny>) -> PyResult<u8> {
    if let Ok(text) = value.cast::<PyString>() {
        return match text.to_str()? {
            "X" => Ok(X),
            "Z" => Ok(Z),
            other => Err(PyValueError::new_err(format!("Not a value: '{other}'"))),
        };
    }
    Ok(match value.extract::<i64>()? {
        0 => 0,
        1 => 1,
        _ => X,
    })
}

fn bus_value(value: &Bound<'_, PyAny>) -> PyResult<BusValue> {
    if value.is_instance_of::<PyInt>() {
        return Ok(BusValue::Int(value.extract::<u64>()?));
    }
    Ok(BusValue::Code(bit_code(value)?))
}

fn bus_to_bit(value: BusValue) -> u8 {
    match value {
        BusValue::Int(0) => 0,
        BusValue::Int(1) => 1,
        BusValue::Int(_) => X,
        BusValue::Code(c) => c,
    }
}

// A guard on the engine state that is released while Python code runs, Python can add
// events, set native inputs or read native values meanwhile
struct Ctx<'a, 'py> {
    py: Python<'py>,
    engine: &'a Engine,
    guard: Option<lock::Guard<'a, Inner>>,
}

impl<'a, 'py> Ctx<'a, 'py> {
    fn new(py: Python<'py>, engine: &'a Engine) -> Self {
        Ctx {
            py,
            engine,
            guard: Some(engine.inner.lock()),
        }
    }

    #[inline]
    fn st(&mut self) -> &mut Inner {
        self.guard.as_mut().unwrap()
    }

    // Port.set_value() of a Python port, with a value object
    fn call_set_value(&mut self, port: u32, value: Bound<'py, PyAny>) -> PyResult<()> {
        self.st().stats[3] += 1;
        let engine = self.engine;
        self.guard = None;
        let result = engine
            .port(self.py, port as usize)
            .call_method1(intern!(self.py, "set_value"), (value,));
        self.guard = Some(engine.inner.lock());
        result.map(|_| ())
    }

    // Port.update_wires() of a one-bit port
    fn update_wires(&mut self, port: u32, value: u8) -> PyResult<()> {
        let st = self.st();
        let p = port as usize;
        if st.value[p] == value {
            return Ok(());
        }
        st.value[p] = value;
        let (start, end) = (st.wired_start[p] as usize, st.wired_start[p + 1] as usize);
        for i in start..end {
            let target = self.st().wired_targets[i];
            self.set_value(target, value)?;
        }
        Ok(())
    }

    // The value of the drivers of a port with several, _Drivers.set_value()
    fn resolved(&mut self, port: u32) -> PyResult<u8> {
        let p = port as usize;
        let (start, end) = {
            let st = self.st();
            (st.driver_start[p] as usize, st.driver_start[p + 1] as usize)
        };
        let mut values = Vec::with_capacity(end - start);
        for i in start..end {
            let st = self.st();
            let driver = st.drivers[i] as usize;
            let value = match st.kind[driver] {
                BUS => bus_to_bit(st.buses[st.owner[driver] as usize].value),
                // The value is in Python
                PY | WORD => {
                    let engine = self.engine;
                    self.guard = None;
                    let result = engine
                        .port(self.py, driver)
                        .getattr(intern!(self.py, "value"));
                    self.guard = Some(engine.inner.lock());
                    bit_code(&result?)?
                }
                _ => st.value[driver],
            };
            values.push(value);
        }
        Ok(resolve(values.into_iter()))
    }

    // Port.set_value() with a one-bit value
    fn set_value(&mut self, port: u32, value: u8) -> PyResult<()> {
        let p = port as usize;
        let st = self.st();
        let value = if st.driver_start[p] != st.driver_start[p + 1] {
            self.resolved(port)?
        } else {
            value
        };
        match self.st().kind[p] {
            IN => {
                if value != self.st().value[p] {
                    self.update_wires(port, value)?;
                }
                let cell = self.st().owner[p] as usize;
                self.evaluate(cell);
            }
            WIRE => {
                if value != self.st().value[p] {
                    self.update_wires(port, value)?;
                }
            }
            BIT => {
                // PortWireBit.set_value()
                if value != self.st().value[p] {
                    self.update_wires(port, value)?;
                }
                let bus = self.st().owner[p];
                self.update_value_from_bits(bus)?;
            }
            BUS => {
                let value = match value {
                    0 | 1 => BusValue::Int(value as u64),
                    code => BusValue::Code(code),
                };
                self.set_bus(port, value)?;
            }
            CELL_INPUT => {
                // _CellInput.set_value()
                if value == self.st().value[p] {
                    return Ok(());
                }
                self.update_wires(port, value)?;
                let st = self.st();
                let cell = st.owner[p] as usize;
                let aux = st.aux[p] as usize;
                if st.words[cell].set_bit(aux >> 20, aux & 0xFFFFF, value) {
                    self.evaluate_word(cell);
                }
            }
            _ => {
                let object = self.engine.values[value as usize].bind(self.py).clone();
                self.call_set_value(port, object)?;
            }
        }
        Ok(())
    }

    // PortMultiBitWire.set_value()
    fn set_bus(&mut self, port: u32, value: BusValue) -> PyResult<()> {
        let bus = self.st().owner[port as usize] as usize;
        let count = self.st().buses[bus].bits.len();
        for i in 0..count {
            let bit = self.st().buses[bus].bits[i];
            let bit_value = match value {
                BusValue::Int(v) => ((v >> i) & 1) as u8,
                BusValue::Code(c) => c,
            };
            self.set_value(bit, bit_value)?;
        }
        Ok(())
    }

    // Port.update_wires() of a bus port
    fn update_bus_wires(&mut self, port: u32, value: BusValue) -> PyResult<()> {
        let p = port as usize;
        let st = self.st();
        let bus = st.owner[p] as usize;
        if st.buses[bus].value == value {
            return Ok(());
        }
        st.buses[bus].value = value;
        let (start, end) = (st.wired_start[p] as usize, st.wired_start[p + 1] as usize);
        for i in start..end {
            let target = self.st().wired_targets[i];
            match self.st().kind[target as usize] {
                BUS => self.set_bus(target, value)?,
                PY => {
                    let object = self.engine.bus_object(self.py, value)?;
                    self.call_set_value(target, object)?;
                }
                _ => self.set_value(target, bus_to_bit(value))?,
            }
        }
        Ok(())
    }

    // PortMultiBitWire.update_value_from_bits()
    fn update_value_from_bits(&mut self, port: u32) -> PyResult<()> {
        let st = self.st();
        let bus = st.owner[port as usize] as usize;
        let mut value = 0u64;
        let mut not_driven = 0;
        let count = st.buses[bus].bits.len();
        for i in 0..count {
            let bit = st.value[st.buses[bus].bits[i] as usize];
            if bit == X {
                if st.buses[bus].value != BusValue::Code(X) {
                    self.update_bus_wires(port, BusValue::Code(X))?;
                    self.st().push(port, Value::Bus(BusValue::Code(X)), 0);
                }
                return Ok(());
            }
            if bit == Z {
                not_driven += 1;
                continue;
            }
            value |= (bit as u64) << i;
        }
        if not_driven > 0 {
            let bus_value = BusValue::Code(if not_driven == count { Z } else { X });
            if self.st().buses[bus].value != bus_value {
                self.update_bus_wires(port, bus_value)?;
                self.st().push(port, Value::Bus(bus_value), 0);
            }
            return Ok(());
        }
        self.update_bus_wires(port, BusValue::Int(value))?;
        self.st().push(port, Value::Bus(BusValue::Int(value)), 0);
        Ok(())
    }

    fn evaluate(&mut self, cell: usize) {
        let st = self.st();
        st.stats[4] += 1;
        if let Some(out) = st.evaluate(cell) {
            let output = st.cells[cell].output;
            let delay = st.delay[output as usize];
            st.push(output, Value::Bit(out), delay);
        }
    }

    // The events of a functional cell
    fn push_writes(&mut self, writes: Vec<word::Write>) {
        let st = self.st();
        for write in writes {
            match write {
                word::Write::Bit(port, code) => {
                    let delay = st.delay[port as usize];
                    st.push(port, Value::Bit(code), delay);
                }
                word::Write::Word(port, value) => {
                    let delay = st.delay[port as usize];
                    st.push(port, Value::Word(value), delay);
                }
            }
        }
    }

    fn evaluate_word(&mut self, cell: usize) {
        let st = self.st();
        st.stats[4] += 1;
        let mut writes = Vec::new();
        st.words[cell].update(&mut writes);
        self.push_writes(writes);
    }

    // _WordOut.delta_cycle(): set the bits of the cells it drives, then update them
    fn word_event(&mut self, port: u32, value: &bits::Bits) {
        let st = self.st();
        let p = port as usize;
        let cell = st.owner[p] as usize;
        let output = st.aux[p] as usize;
        let width = st.words[cell].outputs[output].width;
        let bits = value & word::mask(width);
        let undefined = value >> width;
        let count = st.words[cell].outputs[output].links.len();
        for index in 0..count {
            let st = self.st();
            let (target, links) = st.words[cell].outputs[output].links[index].clone();
            let mut update = false;
            for (input, offset, start, length) in links {
                let m = word::mask(length);
                let part = word::Word {
                    v: (&bits >> start) & &m,
                    u: (&undefined >> start) & &m,
                };
                if st.words[target].set_bits(input, offset, length, &part) {
                    update = true;
                }
            }
            if update {
                self.evaluate_word(target);
            }
        }
    }

    /// Process the next event, see Circuit.process_single_event(). Returns None if there
    /// is none before the stop time, otherwise the port.
    fn process(
        &mut self,
        stop_time: Option<i64>,
        vcd: &Option<Bound<'py, PyAny>>,
    ) -> PyResult<Option<u32>> {
        loop {
            let py = self.py;
            let st = self.st();
            let Some((time, event)) = st.heap.pop() else {
                return Ok(None);
            };
            let p = event.port as usize;
            let latest = st.latest[p];
            let valid = latest.seq != 0
                && (latest.seq == event.seq
                    || (latest.time == time && st.same(py, latest.slot, event.slot)?));
            if !valid {
                st.stats[2] += 1;
                st.release(event.slot);
                continue;
            }
            if stop_time.is_some_and(|stop| time > stop) {
                st.heap.push(time, event);
                return Ok(None);
            }
            st.latest[p].seq = 0;
            st.release(latest.slot);
            st.time = time;
            let value = st.slot_value(py, event.slot);
            st.release(event.slot);
            match value {
                Value::Bit(value) => {
                    st.stats[0] += 1;
                    self.update_wires(event.port, value)?
                }
                // PortMultiBitWire.delta_cycle() does nothing
                Value::Bus(_) => st.stats[0] += 1,
                Value::Word(value) => {
                    st.stats[0] += 1;
                    self.word_event(event.port, &value);
                }
                Value::Python(value) => {
                    st.stats[1] += 1;
                    let engine = self.engine;
                    self.guard = None;
                    let result = engine
                        .port(py, p)
                        .call_method1(intern!(py, "delta_cycle"), (value.bind(py),));
                    self.guard = Some(engine.inner.lock());
                    result?;
                }
            }
            if let Some(vcd) = vcd {
                let time = self.st().time;
                let engine = self.engine;
                self.guard = None;
                let result = vcd.call_method1(intern!(py, "write"), (engine.port(py, p), time));
                self.guard = Some(engine.inner.lock());
                result?;
            }
            return Ok(Some(event.port));
        }
    }
}

#[pymethods]
impl Engine {
    #[new]
    #[allow(clippy::too_many_arguments, clippy::type_complexity)]
    fn new(
        py: Python<'_>,
        ports: Vec<Py<PyAny>>,
        kinds: Vec<u8>,
        owners: Vec<u32>,
        values: Vec<u8>,
        wired: Vec<Vec<u32>>,
        drivers: Vec<Vec<u32>>,
        delays: Vec<i64>,
        toplevel: Vec<bool>,
        cells: Vec<(u8, Vec<u32>, u32, [u8; 6], u8)>,
        buses: Vec<(Vec<u32>, Bound<'_, PyAny>)>,
        aux: Vec<u32>,
        words: Vec<WordCellArgs>,
        time: i64,
    ) -> PyResult<Self> {
        let mut wired_start = Vec::with_capacity(wired.len() + 1);
        let mut wired_targets = Vec::new();
        for targets in &wired {
            wired_start.push(wired_targets.len() as u32);
            wired_targets.extend_from_slice(targets);
        }
        wired_start.push(wired_targets.len() as u32);
        let mut driver_start = Vec::with_capacity(drivers.len() + 1);
        let mut all_drivers = Vec::new();
        for port_drivers in &drivers {
            driver_start.push(all_drivers.len() as u32);
            all_drivers.extend_from_slice(port_drivers);
        }
        driver_start.push(all_drivers.len() as u32);
        let mut word_cells = Vec::with_capacity(words.len());
        for (kind, inputs, outputs, params, q, old_clock, driving) in words {
            let kind = *word::KINDS.get(kind as usize).ok_or_else(|| {
                PyValueError::new_err(format!("Unknown functional cell kind {kind}"))
            })?;
            let inputs = inputs
                .into_iter()
                .map(|(width, update, v, u)| word::Input {
                    word: word::Word {
                        v: v.into(),
                        u: u.into(),
                    },
                    width,
                    update,
                })
                .collect();
            let outputs = outputs
                .into_iter()
                .map(
                    |(width, bit_ports, bit_outputs, word_port, links, last)| word::Output {
                        mask: word::mask(width),
                        width,
                        last: last.map(|(v, u)| word::Word {
                            v: v.into(),
                            u: u.into(),
                        }),
                        bit_ports,
                        bit_outputs: bit_outputs.into(),
                        word_port,
                        links,
                    },
                )
                .collect();
            word_cells.push(word::Cell::new(
                kind,
                inputs,
                outputs,
                params,
                word::Word {
                    v: q.0.into(),
                    u: q.1.into(),
                },
                old_clock,
                driving,
            ));
        }
        let count = ports.len();
        let mut native_cells = Vec::with_capacity(cells.len());
        let mut cell_inputs = Vec::new();
        for (kind, inputs, output, levels, old_clock) in cells {
            native_cells.push(Cell {
                kind: *KINDS
                    .get(kind as usize)
                    .ok_or_else(|| PyValueError::new_err(format!("Unknown cell kind {kind}")))?,
                first: cell_inputs.len() as u32,
                output,
                clock_edge: levels[0],
                reset_level: levels[1],
                reset_value: levels[2],
                enable_level: levels[3],
                set_level: levels[4],
                load_level: levels[5],
                old_clock,
            });
            cell_inputs.extend_from_slice(&inputs);
        }
        let mut native_buses = Vec::with_capacity(buses.len());
        for (bits, value) in buses {
            native_buses.push(Bus {
                bits,
                value: bus_value(&value)?,
            });
        }
        let objects = [
            0i64.into_pyobject(py)?.into_any().unbind(),
            1i64.into_pyobject(py)?.into_any().unbind(),
            PyString::intern(py, "X").into_any().unbind(),
            PyString::intern(py, "Z").into_any().unbind(),
        ];
        Ok(Engine {
            inner: lock::Lock::new(Inner {
                time,
                seq: 0,
                kind: kinds,
                owner: owners,
                aux,
                value: values,
                wired_start,
                wired_targets,
                driver_start,
                drivers: all_drivers,
                delay: delays,
                toplevel,
                cells: native_cells,
                cell_inputs,
                buses: native_buses,
                words: word_cells,
                heap: queue::Queue::default(),
                latest: vec![Latest::default(); count],
                payloads: Vec::new(),
                free: Vec::new(),
                stats: [0; 5],
            }),
            ports: lock::Lock::new(ports),
            values: objects,
        })
    }

    #[getter]
    fn time(&self) -> i64 {
        self.inner.lock().time
    }

    #[setter]
    fn set_time(&self, time: i64) {
        self.inner.lock().time = time;
    }

    /// The value of a native port, as a Python object
    fn value<'py>(&self, py: Python<'py>, nid: u32) -> PyResult<Bound<'py, PyAny>> {
        let st = self.inner.lock();
        let p = nid as usize;
        if st.kind[p] == BUS {
            self.bus_object(py, st.buses[st.owner[p] as usize].value)
        } else {
            Ok(self.values[st.value[p] as usize].bind(py).clone())
        }
    }

    /// Set the value of a native port, without updating anything (Port._value = value)
    fn set_raw(&self, nid: u32, value: Bound<'_, PyAny>) -> PyResult<()> {
        let mut st = self.inner.lock();
        let p = nid as usize;
        if st.kind[p] == BUS {
            let bus = st.owner[p] as usize;
            st.buses[bus].value = bus_value(&value)?;
        } else {
            st.value[p] = bit_code(&value)?;
        }
        Ok(())
    }

    /// Circuit.add_event() for a port of the engine
    fn add_event(&self, nid: u32, value: Bound<'_, PyAny>, delay: i64) -> PyResult<()> {
        let mut st = self.inner.lock();
        let value = match st.kind[nid as usize] {
            PY => Value::Python(value.unbind()),
            BUS => Value::Bus(bus_value(&value)?),
            WORD => Value::Word(bits::Bits::from(value.extract::<BigUint>()?)),
            _ => Value::Bit(bit_code(&value)?),
        };
        st.push(nid, value, delay);
        Ok(())
    }

    /// Port.set_value() of a native port that Python code calls
    fn set_input(&self, py: Python<'_>, nid: u32, value: Bound<'_, PyAny>) -> PyResult<()> {
        let mut ctx = Ctx::new(py, self);
        if ctx.st().kind[nid as usize] == BUS {
            ctx.set_bus(nid, bus_value(&value)?)
        } else {
            ctx.set_value(nid, bit_code(&value)?)
        }
    }

    /// Circuit.run() up to the stop time. Returns if it stopped for a single step.
    #[pyo3(signature = (stop_time, single_step, vcd=None))]
    fn run(
        &self,
        py: Python<'_>,
        stop_time: i64,
        single_step: bool,
        vcd: Option<Bound<'_, PyAny>>,
    ) -> PyResult<bool> {
        let mut ctx = Ctx::new(py, self);
        let mut stop_time = stop_time;
        let mut single_step_stop = false;
        while ctx.st().time <= stop_time {
            let Some(port) = ctx.process(Some(stop_time), &vcd)? else {
                break;
            };
            if single_step && ctx.st().toplevel[port as usize] {
                stop_time = ctx.st().time;
                single_step_stop = true;
            }
        }
        Ok(single_step_stop)
    }

    /// Circuit.process_single_event()
    #[pyo3(signature = (stop_time=None, vcd=None))]
    fn process_single_event(
        &self,
        py: Python<'_>,
        stop_time: Option<i64>,
        vcd: Option<Bound<'_, PyAny>>,
    ) -> PyResult<(bool, bool)> {
        let mut ctx = Ctx::new(py, self);
        Ok(match ctx.process(stop_time, &vcd)? {
            None => (false, false),
            Some(port) => (true, ctx.st().toplevel[port as usize]),
        })
    }

    /// Add a Python port that the engine didn't get, like an internal port of a
    /// component. Returns its index.
    fn add_port(&self, port: Py<PyAny>, wired: Vec<u32>, toplevel: bool) -> u32 {
        let mut st = self.inner.lock();
        let nid = st.kind.len() as u32;
        st.kind.push(PY);
        st.owner.push(u32::MAX);
        st.aux.push(0);
        st.value.push(X);
        // Ports are added after the others, so their targets go at the end
        st.wired_targets.extend_from_slice(&wired);
        let end = st.wired_targets.len() as u32;
        st.wired_start.push(end);
        let end = st.drivers.len() as u32;
        st.driver_start.push(end);
        st.delay.push(0);
        st.toplevel.push(toplevel);
        st.latest.push(Latest::default());
        self.ports.lock().push(port);
        nid
    }

    /// The state for going back to the Python engine: the events in the order of the
    /// heap as (time, seq, nid, value), the seq of the latest event of each port (None
    /// if it has none), and the clock level of each cell
    #[allow(clippy::type_complexity)]
    fn export<'py>(
        &self,
        py: Python<'py>,
    ) -> PyResult<(
        Vec<(i64, u64, u32, Bound<'py, PyAny>)>,
        Vec<Option<u64>>,
        Vec<Bound<'py, PyAny>>,
    )> {
        let st = self.inner.lock();
        let mut events = Vec::with_capacity(st.heap.len());
        for (time, e) in st.heap.iter() {
            let value = st.slot_value(py, e.slot);
            events.push((time, e.seq, e.port, self.object(py, &value)?));
        }
        let latest = st
            .latest
            .iter()
            .map(|l| (l.seq != 0).then_some(l.seq))
            .collect();
        let clocks = st
            .cells
            .iter()
            .map(|c| self.values[c.old_clock as usize].bind(py).clone())
            .collect();
        Ok((events, latest, clocks))
    }

    /// The default state of a functional cell, a register starts at 0
    fn word_default_state(&self, py: Python<'_>, cell: usize) {
        let mut ctx = Ctx::new(py, self);
        let mut writes = Vec::new();
        if ctx.st().words[cell].is_register() {
            ctx.st().words[cell].default_state(&mut writes);
        }
        ctx.push_writes(writes);
    }

    /// The state of each functional cell for going back to Python: the values of its
    /// inputs, the last values of its outputs, Q, the clock level and if it drives
    #[allow(clippy::type_complexity)]
    fn export_words<'py>(
        &self,
        py: Python<'py>,
    ) -> Vec<(
        Vec<(BigUint, BigUint)>,
        Vec<Option<(BigUint, BigUint)>>,
        (BigUint, BigUint),
        Bound<'py, PyAny>,
        bool,
    )> {
        let st = self.inner.lock();
        st.words
            .iter()
            .map(|cell| {
                (
                    cell.inputs
                        .iter()
                        .map(|i| ((&i.word.v).into(), (&i.word.u).into()))
                        .collect(),
                    cell.outputs
                        .iter()
                        .map(|o| o.last.as_ref().map(|w| ((&w.v).into(), (&w.u).into())))
                        .collect(),
                    ((&cell.q.v).into(), (&cell.q.u).into()),
                    self.values[cell.old_clock as usize].bind(py).clone(),
                    cell.driving,
                )
            })
            .collect()
    }

    fn __len__(&self) -> usize {
        self.inner.lock().heap.len()
    }

    /// Events (native, Python), stale events, Python calls and cell evaluations so far
    fn stats(&self) -> [u64; 5] {
        self.inner.lock().stats
    }
}

#[pymodule]
fn digsim_native(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Engine>()?;
    Ok(())
}
