// Copyright (c) Fredrik Andersson, 2023-2026
// All rights reserved

//! Optional native simulation engine for DigSim (a trial).
//!
//! `EventQueue` is only the event queue of `Circuit`, the Python loop still handles
//! every event. `Engine` runs the whole event loop: the Yosys gate cells and the bus
//! ports it knows are simulated here, every other component stays in Python and is
//! called back from the same queue. Both keep the exact order of the Python engine: the
//! queue is CPython's heapq algorithm, comparing only the times, so events at the same
//! time come out in the same order.

use pyo3::exceptions::PyValueError;
use pyo3::intern;
use pyo3::prelude::*;
use pyo3::types::{PyInt, PyString};
use std::collections::HashMap;
use std::sync::{Mutex, MutexGuard};

trait Timed {
    fn time(&self) -> i64;
}

// CPython's heapq (Modules/_heapqmodule.c), step by step, so that events at the same
// time are popped in the same order as with heapq
fn siftdown<T: Timed>(heap: &mut [T], start: usize, mut pos: usize) {
    while pos > start {
        let parent = (pos - 1) >> 1;
        if heap[pos].time() < heap[parent].time() {
            heap.swap(pos, parent);
            pos = parent;
        } else {
            break;
        }
    }
}

fn siftup<T: Timed>(heap: &mut [T], mut pos: usize) {
    let end = heap.len();
    let start = pos;
    let limit = end >> 1;
    while pos < limit {
        let mut child = 2 * pos + 1;
        // heapq: "not heap[child] < heap[child + 1]"
        if child + 1 < end && heap[child].time() >= heap[child + 1].time() {
            child += 1;
        }
        heap.swap(pos, child);
        pos = child;
    }
    siftdown(heap, start, pos);
}

fn heappush<T: Timed>(heap: &mut Vec<T>, item: T) {
    heap.push(item);
    let last = heap.len() - 1;
    siftdown(heap, 0, last);
}

fn heappop<T: Timed>(heap: &mut Vec<T>) -> Option<T> {
    let last = heap.pop()?;
    if heap.is_empty() {
        return Some(last);
    }
    let first = std::mem::replace(&mut heap[0], last);
    siftup(heap, 0);
    Some(first)
}

// Trial 1: only the event queue

struct QueueEvent {
    time: i64,
    seq: u64,
    port: Py<PyAny>,
    value: Py<PyAny>,
}

impl Timed for QueueEvent {
    fn time(&self) -> i64 {
        self.time
    }
}

struct QueueInner {
    heap: Vec<QueueEvent>,
    seq: u64,
    // The latest event of each port: (seq, time, value), by the address of the port
    latest: HashMap<usize, (u64, i64, Py<PyAny>)>,
}

/// The event queue of a circuit, the same as its heapq list and dict of latest events
#[pyclass(frozen)]
struct EventQueue {
    inner: Mutex<QueueInner>,
}

#[pymethods]
impl EventQueue {
    #[new]
    fn new() -> Self {
        EventQueue {
            inner: Mutex::new(QueueInner {
                heap: Vec::new(),
                seq: 0,
                latest: HashMap::new(),
            }),
        }
    }

    fn add(&self, port: Bound<'_, PyAny>, value: Bound<'_, PyAny>, time: i64) {
        let mut st = self.inner.lock().unwrap();
        st.seq += 1;
        let seq = st.seq;
        st.latest
            .insert(port.as_ptr() as usize, (seq, time, value.clone().unbind()));
        heappush(
            &mut st.heap,
            QueueEvent {
                time,
                seq,
                port: port.unbind(),
                value: value.unbind(),
            },
        );
    }

    /// The next event that isn't stale, as (port, value, time), None if there is none or
    /// it is after the stop time
    #[allow(clippy::type_complexity)]
    fn pop(&self, py: Python<'_>, stop_time: i64) -> PyResult<Option<(Py<PyAny>, Py<PyAny>, i64)>> {
        let mut st = self.inner.lock().unwrap();
        while let Some(event) = heappop(&mut st.heap) {
            let key = event.port.as_ptr() as usize;
            let valid = match st.latest.get(&key) {
                None => false,
                Some((seq, time, value)) => {
                    *seq == event.seq
                        || (*time == event.time && value.bind(py).eq(event.value.bind(py))?)
                }
            };
            if !valid {
                continue;
            }
            if event.time > stop_time {
                heappush(&mut st.heap, event);
                return Ok(None);
            }
            st.latest.remove(&key);
            return Ok(Some((event.port, event.value, event.time)));
        }
        Ok(None)
    }

    fn __len__(&self) -> usize {
        self.inner.lock().unwrap().heap.len()
    }
}

// Trial 2: the whole event loop, with native gate cells and bus ports

// Values of one-bit native ports: 0, 1, undefined ("X") and not driven ("Z")
const X: u8 = 2;
const Z: u8 = 3;

// Port kinds
const PY: u8 = 0; // A Python port, called back
const IN: u8 = 1; // Input of a native cell that updates the cell (PortIn)
const WIRE: u8 = 2; // Input of a native cell that doesn't (PortWire)
const BIT: u8 = 4; // A bit of a native bus (PortWireBit)
const BUS: u8 = 5; // A bus of bits, up to 64 (PortMultiBitWire)
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
    Python(Py<PyAny>),
}

impl Value {
    fn clone_ref(&self, py: Python<'_>) -> Value {
        match self {
            Value::Bit(v) => Value::Bit(*v),
            Value::Bus(v) => Value::Bus(*v),
            Value::Python(v) => Value::Python(v.clone_ref(py)),
        }
    }
}

struct Event {
    time: i64,
    seq: u64,
    port: u32,
    value: Value,
}

impl Timed for Event {
    fn time(&self) -> i64 {
        self.time
    }
}

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
}

const KINDS: [Kind; 26] = {
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
    ]
};

struct Cell {
    kind: Kind,
    // The input ports, in the order of the kind; flip-flops: C, D, E, R
    inputs: Vec<u32>,
    output: u32,
    clock_edge: u8,
    reset_level: u8,
    reset_value: u8,
    enable_level: u8,
    old_clock: u8,
}

struct Bus {
    bits: Vec<u32>,
    value: BusValue,
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
    // The cell of a cell port, the bus port of a bit, the bus of a bus port
    owner: Vec<u32>,
    value: Vec<u8>,
    wired: Vec<Vec<u32>>,
    delay: Vec<i64>,
    toplevel: Vec<bool>,
    cells: Vec<Cell>,
    buses: Vec<Bus>,
    heap: Vec<Event>,
    latest: Vec<Option<(u64, i64, Value)>>,
    // Statistics: events (native, Python), stale events, Python calls, cell evaluations
    stats: [u64; 5],
}

impl Inner {
    fn push(&mut self, py: Python<'_>, port: u32, value: Value, delay: i64) {
        let time = self.time + delay;
        self.seq += 1;
        let seq = self.seq;
        self.latest[port as usize] = Some((seq, time, value.clone_ref(py)));
        heappush(
            &mut self.heap,
            Event {
                time,
                seq,
                port,
                value,
            },
        );
    }

    /// The new output of a cell, None if it doesn't set it (a flip-flop without an edge)
    fn evaluate(&mut self, c: usize) -> Option<u8> {
        let cell = &mut self.cells[c];
        let v = |i: usize| self.value[cell.inputs[i] as usize];
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
            Dff | Dffe | DffAsyncReset | DffeAsyncReset | Sdff | Sdffe | Sdffce => {
                let clock = v(0);
                let edge = clock != cell.old_clock && clock == cell.clock_edge;
                let enabled = |cell: &Cell| v(2) == cell.enable_level;
                let reset = |cell: &Cell| v(3) == cell.reset_level;
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
                    _ => (edge && enabled(cell)).then(|| {
                        if reset(cell) {
                            cell.reset_value
                        } else {
                            v(1)
                        }
                    }),
                };
                cell.old_clock = clock;
                return out;
            }
        };
        Some(out)
    }
}

/// The simulation of a circuit, see digsim/circuit/_native.py
#[pyclass(frozen)]
struct Engine {
    inner: Mutex<Inner>,
    ports: Mutex<Vec<Py<PyAny>>>,
    // The Python objects of the values 0, 1, "X" and "Z"
    values: [Py<PyAny>; 4],
}

impl Engine {
    fn port<'py>(&self, py: Python<'py>, p: usize) -> Bound<'py, PyAny> {
        self.ports.lock().unwrap()[p].bind(py).clone()
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
    guard: Option<MutexGuard<'a, Inner>>,
}

impl<'a, 'py> Ctx<'a, 'py> {
    fn new(py: Python<'py>, engine: &'a Engine) -> Self {
        Ctx {
            py,
            engine,
            guard: Some(engine.inner.lock().unwrap()),
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
        self.guard = Some(engine.inner.lock().unwrap());
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
        let count = st.wired[p].len();
        for i in 0..count {
            let target = self.st().wired[p][i];
            self.set_value(target, value)?;
        }
        Ok(())
    }

    // Port.set_value() with a one-bit value
    fn set_value(&mut self, port: u32, value: u8) -> PyResult<()> {
        let p = port as usize;
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
        let count = st.wired[p].len();
        for i in 0..count {
            let target = self.st().wired[p][i];
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
        let py = self.py;
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
                    self.st().push(py, port, Value::Bus(BusValue::Code(X)), 0);
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
                self.st().push(py, port, Value::Bus(bus_value), 0);
            }
            return Ok(());
        }
        self.update_bus_wires(port, BusValue::Int(value))?;
        self.st()
            .push(py, port, Value::Bus(BusValue::Int(value)), 0);
        Ok(())
    }

    fn evaluate(&mut self, cell: usize) {
        let py = self.py;
        let st = self.st();
        st.stats[4] += 1;
        if let Some(out) = st.evaluate(cell) {
            let output = st.cells[cell].output;
            let delay = st.delay[output as usize];
            st.push(py, output, Value::Bit(out), delay);
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
            let Some(event) = heappop(&mut st.heap) else {
                return Ok(None);
            };
            let p = event.port as usize;
            let valid = match &st.latest[p] {
                None => false,
                Some((seq, time, value)) => {
                    *seq == event.seq
                        || (*time == event.time
                            && match (value, &event.value) {
                                (Value::Bit(a), Value::Bit(b)) => a == b,
                                (Value::Bus(a), Value::Bus(b)) => a == b,
                                (Value::Python(a), Value::Python(b)) => a.bind(py).eq(b.bind(py))?,
                                _ => false,
                            })
                }
            };
            if !valid {
                st.stats[2] += 1;
                continue;
            }
            if stop_time.is_some_and(|stop| event.time > stop) {
                heappush(&mut st.heap, event);
                return Ok(None);
            }
            st.latest[p] = None;
            st.time = event.time;
            match event.value {
                Value::Bit(value) => {
                    st.stats[0] += 1;
                    self.update_wires(event.port, value)?
                }
                // PortMultiBitWire.delta_cycle() does nothing
                Value::Bus(_) => st.stats[0] += 1,
                Value::Python(value) => {
                    st.stats[1] += 1;
                    let engine = self.engine;
                    self.guard = None;
                    let result = engine
                        .port(py, p)
                        .call_method1(intern!(py, "delta_cycle"), (value.bind(py),));
                    self.guard = Some(engine.inner.lock().unwrap());
                    result?;
                }
            }
            if let Some(vcd) = vcd {
                let time = self.st().time;
                let engine = self.engine;
                self.guard = None;
                let result = vcd.call_method1(intern!(py, "write"), (engine.port(py, p), time));
                self.guard = Some(engine.inner.lock().unwrap());
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
        delays: Vec<i64>,
        toplevel: Vec<bool>,
        cells: Vec<(u8, Vec<u32>, u32, [u8; 4], u8)>,
        buses: Vec<(Vec<u32>, Bound<'_, PyAny>)>,
        time: i64,
    ) -> PyResult<Self> {
        let count = ports.len();
        let mut native_cells = Vec::with_capacity(cells.len());
        for (kind, inputs, output, levels, old_clock) in cells {
            native_cells.push(Cell {
                kind: *KINDS
                    .get(kind as usize)
                    .ok_or_else(|| PyValueError::new_err(format!("Unknown cell kind {kind}")))?,
                inputs,
                output,
                clock_edge: levels[0],
                reset_level: levels[1],
                reset_value: levels[2],
                enable_level: levels[3],
                old_clock,
            });
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
            inner: Mutex::new(Inner {
                time,
                seq: 0,
                kind: kinds,
                owner: owners,
                value: values,
                wired,
                delay: delays,
                toplevel,
                cells: native_cells,
                buses: native_buses,
                heap: Vec::new(),
                latest: (0..count).map(|_| None).collect(),
                stats: [0; 5],
            }),
            ports: Mutex::new(ports),
            values: objects,
        })
    }

    #[getter]
    fn time(&self) -> i64 {
        self.inner.lock().unwrap().time
    }

    #[setter]
    fn set_time(&self, time: i64) {
        self.inner.lock().unwrap().time = time;
    }

    /// The value of a native port, as a Python object
    fn value<'py>(&self, py: Python<'py>, nid: u32) -> PyResult<Bound<'py, PyAny>> {
        let st = self.inner.lock().unwrap();
        let p = nid as usize;
        if st.kind[p] == BUS {
            self.bus_object(py, st.buses[st.owner[p] as usize].value)
        } else {
            Ok(self.values[st.value[p] as usize].bind(py).clone())
        }
    }

    /// Set the value of a native port, without updating anything (Port._value = value)
    fn set_raw(&self, nid: u32, value: Bound<'_, PyAny>) -> PyResult<()> {
        let mut st = self.inner.lock().unwrap();
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
    fn add_event(
        &self,
        py: Python<'_>,
        nid: u32,
        value: Bound<'_, PyAny>,
        delay: i64,
    ) -> PyResult<()> {
        let mut st = self.inner.lock().unwrap();
        let value = match st.kind[nid as usize] {
            PY => Value::Python(value.unbind()),
            BUS => Value::Bus(bus_value(&value)?),
            _ => Value::Bit(bit_code(&value)?),
        };
        st.push(py, nid, value, delay);
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
        let mut st = self.inner.lock().unwrap();
        let nid = st.kind.len() as u32;
        st.kind.push(PY);
        st.owner.push(u32::MAX);
        st.value.push(X);
        st.wired.push(wired);
        st.delay.push(0);
        st.toplevel.push(toplevel);
        st.latest.push(None);
        self.ports.lock().unwrap().push(port);
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
        let st = self.inner.lock().unwrap();
        let mut events = Vec::with_capacity(st.heap.len());
        for e in &st.heap {
            events.push((e.time, e.seq, e.port, self.object(py, &e.value)?));
        }
        let latest = st.latest.iter().map(|l| l.as_ref().map(|l| l.0)).collect();
        let clocks = st
            .cells
            .iter()
            .map(|c| self.values[c.old_clock as usize].bind(py).clone())
            .collect();
        Ok((events, latest, clocks))
    }

    fn __len__(&self) -> usize {
        self.inner.lock().unwrap().heap.len()
    }

    /// Events (native, Python), stale events, Python calls and cell evaluations so far
    fn stats(&self) -> [u64; 5] {
        self.inner.lock().unwrap().stats
    }
}

#[pymodule]
fn digsim_native(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<EventQueue>()?;
    m.add_class::<Engine>()?;
    Ok(())
}
