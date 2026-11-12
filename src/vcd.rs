// Copyright (c) Fredrik Andersson, 2023-2026
// All rights reserved

//! The VCD writer of DigSim (digsim/circuit/_waves_writer.py). It writes the same text as
//! the VCDWriter of pyvcd (vcd/writer.py), for the variables DigSim has (wires and
//! strings), so that the VCD files don't depend on the engine. The engine writes the
//! changes of its own ports here without calling Python.

use num_bigint::BigInt;
use num_traits::{Signed, Zero};
use pyo3::exceptions::{PyKeyError, PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyInt, PyString};
use std::collections::{BTreeMap, HashMap, HashSet};
use std::fmt::Write;

use crate::lock;

/// Text buffered before it is written to the file
const BUFFER: usize = 1 << 16;

/// A value of a variable, as the Python value it was made from: two values are equal
/// when the Python values are. 16 bytes, the variables are many.
#[derive(Clone, PartialEq, Eq, Debug)]
pub(crate) enum V {
    Small(u64),
    // An int that isn't a u64 (big or negative)
    Big(Box<BigInt>),
    X,
    Z,
    // Boxed: a thin pointer keeps V at 16 bytes
    #[allow(clippy::box_collection)]
    Str(Box<String>),
    None,
}

impl V {
    /// The value of a one-bit code of the engine: 0, 1, X and Z
    #[inline]
    pub fn code(code: u8) -> V {
        match code {
            0 => V::Small(0),
            1 => V::Small(1),
            crate::X => V::X,
            _ => V::Z,
        }
    }

    pub fn from_py(value: &Bound<'_, PyAny>) -> PyResult<V> {
        if value.is_none() {
            return Ok(V::None);
        }
        if let Ok(text) = value.cast::<PyString>() {
            let text = text.to_str()?;
            return Ok(match text {
                "X" => V::X,
                "Z" => V::Z,
                _ => V::Str(Box::new(text.to_string())),
            });
        }
        if value.is_instance_of::<PyInt>() {
            if let Ok(small) = value.extract::<u64>() {
                return Ok(V::Small(small));
            }
            return Ok(V::Big(Box::new(value.extract::<BigInt>()?)));
        }
        Err(PyValueError::new_err(format!("Invalid value ({value})")))
    }
}

impl std::fmt::Display for V {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            V::Small(v) => write!(f, "{v}"),
            V::Big(v) => write!(f, "{v}"),
            V::X => f.write_str("X"),
            V::Z => f.write_str("Z"),
            V::Str(s) => f.write_str(s),
            V::None => f.write_str("None"),
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Kind {
    Scalar,
    Vector(u32),
    Text,
}

/// The identifier of a variable, up to 7 characters (94^7 variables)
#[derive(Clone, Copy)]
struct Ident {
    len: u8,
    bytes: [u8; 7],
}

impl Ident {
    fn new(text: &str) -> Ident {
        let mut bytes = [0; 7];
        bytes[..text.len()].copy_from_slice(text.as_bytes());
        Ident {
            len: text.len() as u8,
            bytes,
        }
    }

    #[inline]
    fn as_str(&self) -> &str {
        // Only ASCII from identifier()
        std::str::from_utf8(&self.bytes[..self.len as usize]).unwrap_or("")
    }
}

/// A variable: 32 bytes, they are read for every change
struct Var {
    ident: Ident,
    kind: Kind,
    value: V,
}

pub(crate) struct State {
    file: Option<Py<PyAny>>,
    out: String,
    // A line before it is written
    line: String,
    timescale: String,
    date: String,
    // The variables, by slot: the slot of each variable (in the order they were
    // registered, their numbers in Python) is in slot_of. The engine moves the variables
    // of a port next to each other (see place()), they change together.
    vars: Vec<Var>,
    slot_of: Vec<u32>,
    // The $var lines of each scope, sorted like the tuples of pyvcd
    scope_vars: BTreeMap<Vec<String>, Vec<String>>,
    scope_names: HashMap<Vec<String>, HashSet<String>>,
    timestamp: i64,
    last_dumped: Option<i64>,
    registering: bool,
    closed: bool,
}

/// pyvcd's _encode_identifier(): base 94
fn identifier(mut v: u64) -> String {
    let mut encoded = String::new();
    while v != 0 {
        v -= 1;
        encoded.push(char::from((v % 94) as u8 + 33));
        v /= 94;
    }
    encoded
}

fn value_error(message: String) -> PyErr {
    PyValueError::new_err(message)
}

/// The value of a vector variable (pyvcd's _format_scalar_value())
fn vector_value(value: &V, size: u32, out: &mut String) -> PyResult<()> {
    match value {
        V::Small(v) => {
            if size < 64 && *v >= 1u64 << size {
                return Err(value_error(format!(
                    "Value ({v}) not representable in {size} bits"
                )));
            }
            let _ = write!(out, "{v:b}");
        }
        V::Big(v) => {
            let max: BigInt = BigInt::from(1) << size;
            let mut v = (**v).clone();
            if -&v > (&max >> 1) || v >= max {
                return Err(value_error(format!(
                    "Value ({v}) not representable in {size} bits"
                )));
            }
            if v.is_negative() {
                v += max;
            }
            out.push_str(&v.to_str_radix(2));
        }
        V::None => out.push('z'),
        V::X => out.push('X'),
        V::Z => out.push('Z'),
        V::Str(s) => {
            if s.chars().count() > size as usize || s.chars().any(|c| !"01xzXZ-".contains(c)) {
                return Err(value_error(format!("Invalid vector value ({s})")));
            }
            out.push_str(s);
        }
    }
    Ok(())
}

/// The text of a variable with a value (pyvcd's format_value())
fn format_value(var: &Var, value: &V, out: &mut String) -> PyResult<()> {
    match var.kind {
        Kind::Scalar => {
            match value {
                V::Str(s) => {
                    if s.chars().count() != 1 || !"01xzXZ".contains(s.as_str()) {
                        return Err(value_error(format!("Invalid scalar value ({s})")));
                    }
                    out.push_str(s);
                }
                V::X => out.push('X'),
                V::Z => out.push('Z'),
                V::None => out.push('z'),
                V::Small(v) => out.push(if *v != 0 { '1' } else { '0' }),
                V::Big(v) => out.push(if !v.is_zero() { '1' } else { '0' }),
            }
            out.push_str(var.ident.as_str());
        }
        Kind::Vector(size) => {
            out.push('b');
            vector_value(value, size, out)?;
            out.push(' ');
            out.push_str(var.ident.as_str());
        }
        Kind::Text => {
            out.push('s');
            match value {
                V::None => {}
                V::Str(s) => {
                    for c in s.as_str().chars() {
                        match c {
                            '\t' => out.push_str("\\t"),
                            '\n' => out.push_str("\\n"),
                            '\r' => out.push_str("\\r"),
                            ' ' => out.push_str("\\x20"),
                            '\\' => out.push_str("\\\\"),
                            c => out.push(c),
                        }
                    }
                }
                V::X => out.push('X'),
                V::Z => out.push('Z'),
                other => {
                    return Err(value_error(format!("Invalid string value ({other})")));
                }
            }
            out.push(' ');
            out.push_str(var.ident.as_str());
        }
    }
    Ok(())
}

impl State {
    /// pyvcd's _gen_header()
    fn header(&self) -> Vec<String> {
        let mut lines = Vec::new();
        for (keyword, value) in [("$date", &self.date), ("$timescale", &self.timescale)] {
            if value.is_empty() {
                continue;
            }
            let parts: Vec<&str> = value.split('\n').collect();
            if parts.len() == 1 {
                lines.push(format!("{keyword} {value} $end"));
            } else {
                lines.push(keyword.to_string());
                for part in parts {
                    lines.push(format!("\t{part}"));
                }
                lines.push("$end".to_string());
            }
        }
        let mut previous: Vec<String> = Vec::new();
        for (scope, var_lines) in &self.scope_vars {
            for i in 0..previous.len().max(scope.len()) {
                if previous.get(i) != scope.get(i) {
                    for _ in i..previous.len() {
                        lines.push("$upscope $end".to_string());
                    }
                    for name in &scope[i.min(scope.len())..] {
                        lines.push(format!("$scope module {name} $end"));
                    }
                    break;
                }
            }
            lines.extend(var_lines.iter().cloned());
            previous = scope.clone();
        }
        for _ in &previous {
            lines.push("$upscope $end".to_string());
        }
        lines.push("$enddefinitions $end".to_string());
        lines
    }

    fn dump_timestamp(&mut self) {
        if self.last_dumped != Some(self.timestamp) {
            self.last_dumped = Some(self.timestamp);
            let _ = writeln!(self.out, "#{}", self.timestamp);
        }
    }

    /// pyvcd's _finalize_registration(): the header and the values at the first time
    fn finalize(&mut self) -> PyResult<()> {
        let header = self.header();
        self.out.push_str(&header.join("\n"));
        self.out.push('\n');
        if !self.slot_of.is_empty() {
            self.dump_timestamp();
            self.out.push_str("$dumpvars\n");
            let mut line = String::new();
            for &slot in &self.slot_of {
                let var = &self.vars[slot as usize];
                line.clear();
                format_value(var, &var.value, &mut line)?;
                self.out.push_str(&line);
                self.out.push('\n');
            }
            self.out.push_str("$end\n");
        }
        self.registering = false;
        self.scope_names.clear();
        Ok(())
    }

    /// The slots of variables (by number), the ones that haven't been placed yet are
    /// moved next to each other, after the others
    pub fn place(&mut self, numbers: &[u32]) -> PyResult<Vec<u32>> {
        let registered = self.slot_of.len();
        let mut slots = Vec::with_capacity(numbers.len());
        for &number in numbers {
            let Some(&slot) = self.slot_of.get(number as usize) else {
                return Err(value_error(format!("No variable {number}")));
            };
            if (slot as usize) < registered {
                // Where it was registered: moved, a copy stays there unused
                let var = &self.vars[slot as usize];
                let moved = Var {
                    ident: var.ident,
                    kind: var.kind,
                    value: var.value.clone(),
                };
                self.vars.push(moved);
                let new_slot = (self.vars.len() - 1) as u32;
                self.slot_of[number as usize] = new_slot;
                slots.push(new_slot);
            } else {
                slots.push(slot);
            }
        }
        Ok(slots)
    }

    /// change() of a variable by its number
    pub fn change_number(&mut self, number: usize, timestamp: i64, value: V) -> PyResult<()> {
        let Some(&slot) = self.slot_of.get(number) else {
            return Err(value_error(format!("No variable {number}")));
        };
        self.change(slot as usize, timestamp, value)
    }

    /// pyvcd's change(), of the variable in a slot
    pub fn change(&mut self, var: usize, timestamp: i64, value: V) -> PyResult<()> {
        if self.closed {
            return Err(PyRuntimeError::new_err("Cannot change value after close()"));
        }
        let Some(variable) = self.vars.get(var) else {
            return Err(value_error(format!("No variable {var}")));
        };
        let changed = value != variable.value;
        if !self.registering && timestamp >= self.timestamp {
            // Written directly, and taken back if the value can't be written
            let (mark, last_dumped, old_timestamp) =
                (self.out.len(), self.last_dumped, self.timestamp);
            self.timestamp = timestamp;
            if !changed {
                return Ok(());
            }
            self.dump_timestamp();
            let State { vars, out, .. } = self;
            let variable = &mut vars[var];
            if variable.kind == Kind::Scalar {
                // The common line: a value of one bit, as bytes
                let code = match &value {
                    V::Small(0) => Some(b'0'),
                    V::Small(1) => Some(b'1'),
                    V::X => Some(b'X'),
                    V::Z => Some(b'Z'),
                    _ => None,
                };
                if let Some(code) = code {
                    let ident = &variable.ident.bytes[..variable.ident.len as usize];
                    // SAFETY: ASCII only, the text stays UTF-8
                    let bytes = unsafe { out.as_mut_vec() };
                    bytes.push(code);
                    bytes.extend_from_slice(ident);
                    bytes.push(b'\n');
                    variable.value = value;
                    return Ok(());
                }
            }
            if let Err(error) = format_value(variable, &value, out) {
                self.out.truncate(mark);
                self.last_dumped = last_dumped;
                self.timestamp = old_timestamp;
                return Err(error);
            }
            self.out.push('\n');
            self.vars[var].value = value;
            return Ok(());
        }
        if changed {
            self.line.clear();
            format_value(variable, &value, &mut self.line)?;
        }
        if timestamp < self.timestamp {
            return Err(PyRuntimeError::new_err(format!(
                "Out of order timestamp: {timestamp}"
            )));
        } else if timestamp > self.timestamp {
            if self.registering {
                self.finalize()?;
            }
            self.timestamp = timestamp;
        }
        if !changed {
            return Ok(());
        }
        self.vars[var].value = value;
        if !self.registering {
            self.dump_timestamp();
            self.out.push_str(&self.line);
            self.out.push('\n');
        }
        Ok(())
    }

    /// The buffered text and the file for it, all of it or when the buffer is full. It is
    /// written with write_text() after the lock is released: a write can let another
    /// thread run, and that one could use the writer.
    pub fn take_text(&mut self, py: Python<'_>, all: bool) -> Option<(Py<PyAny>, String)> {
        if self.out.is_empty() || (!all && self.out.len() < BUFFER) {
            return None;
        }
        let text = std::mem::take(&mut self.out);
        self.out.reserve(BUFFER + 1024);
        self.file.as_ref().map(|file| (file.clone_ref(py), text))
    }

    /// pyvcd's flush() but the writing to the file (see take_text())
    fn prepare_flush(&mut self, timestamp: Option<i64>) -> PyResult<()> {
        if self.closed {
            return Err(PyRuntimeError::new_err("Cannot flush() after close()"));
        }
        if self.registering {
            self.finalize()?;
        }
        if let Some(timestamp) = timestamp {
            if timestamp < self.timestamp {
                return Err(PyRuntimeError::new_err(format!(
                    "Out of order timestamp: {timestamp}"
                )));
            }
            self.timestamp = timestamp;
            self.dump_timestamp();
        }
        Ok(())
    }
}

/// Write text from take_text() to its file
pub(crate) fn write_text(py: Python<'_>, text: Option<(Py<PyAny>, String)>) -> PyResult<()> {
    if let Some((file, text)) = text {
        file.bind(py).call_method1("write", (text,))?;
    }
    Ok(())
}

/// A VCD file, like pyvcd's VCDWriter: the text is written to a file object
#[pyclass(frozen)]
pub struct VcdWriter {
    state: lock::Lock<State>,
}

impl VcdWriter {
    pub(crate) fn state(&self) -> lock::Guard<'_, State> {
        self.state.lock()
    }
}

#[pymethods]
impl VcdWriter {
    #[new]
    #[pyo3(signature = (file, timescale="1 us", date="today"))]
    fn new(file: Py<PyAny>, timescale: &str, date: &str) -> Self {
        VcdWriter {
            state: lock::Lock::new(State {
                file: Some(file),
                out: String::new(),
                line: String::new(),
                timescale: timescale.to_string(),
                date: date.to_string(),
                vars: Vec::new(),
                slot_of: Vec::new(),
                scope_vars: BTreeMap::new(),
                scope_names: HashMap::new(),
                timestamp: 0,
                last_dumped: None,
                registering: true,
                closed: false,
            }),
        }
    }

    /// Register a variable ("wire" or "string"), returns its number for change()
    #[pyo3(signature = (scope, name, var_type, size=None, init=None))]
    fn register_var(
        &self,
        scope: &str,
        name: &str,
        var_type: &str,
        size: Option<u32>,
        init: Option<Bound<'_, PyAny>>,
    ) -> PyResult<usize> {
        let mut st = self.state.lock();
        if st.closed {
            return Err(PyRuntimeError::new_err("Cannot register after close()."));
        }
        if !st.registering {
            return Err(PyRuntimeError::new_err("Cannot register after time 0."));
        }
        let scope: Vec<String> = scope.split('.').map(str::to_string).collect();
        let names = st.scope_names.entry(scope.clone()).or_default();
        if names.contains(name) {
            return Err(PyKeyError::new_err(format!(
                "Duplicate var {name} in scope {}",
                scope.join(".")
            )));
        }
        let (kind, size) = match (var_type, size) {
            ("string", _) => (Kind::Text, 1),
            ("wire", Some(1)) => (Kind::Scalar, 1),
            ("wire", Some(size)) if size > 0 => (Kind::Vector(size), size),
            _ => {
                return Err(value_error(format!(
                    "Unsupported variable: {var_type}, size {size:?}"
                )))
            }
        };
        let value = match &init {
            Some(init) if !init.is_none() => V::from_py(init)?,
            _ if kind == Kind::Text => V::Str(Box::default()),
            _ => V::Str(Box::new("x".to_string())),
        };
        let ident = identifier(st.slot_of.len() as u64 + 1);
        let var = Var {
            ident: Ident::new(&ident),
            kind,
            value,
        };
        // pyvcd checks the initial value
        format_value(&var, &var.value, &mut String::new())?;
        let line = format!("$var {var_type} {size} {ident} {name} $end");
        st.scope_names
            .entry(scope.clone())
            .or_default()
            .insert(name.to_string());
        st.scope_vars.entry(scope).or_default().push(line);
        if st.vars.len() != st.slot_of.len() {
            return Err(PyRuntimeError::new_err("Cannot register after a change"));
        }
        let slot = st.vars.len() as u32;
        st.vars.push(var);
        st.slot_of.push(slot);
        Ok(st.slot_of.len() - 1)
    }

    /// Change the value of a variable at a time, like pyvcd
    fn change(
        &self,
        py: Python<'_>,
        var: usize,
        timestamp: i64,
        value: Bound<'_, PyAny>,
    ) -> PyResult<()> {
        let value = V::from_py(&value)?;
        let text = {
            let mut st = self.state.lock();
            st.change_number(var, timestamp, value)?;
            st.take_text(py, false)
        };
        write_text(py, text)
    }

    /// Write the buffered text to the file (it doesn't end the registration, unlike flush())
    fn drain(&self, py: Python<'_>) -> PyResult<()> {
        let text = self.state.lock().take_text(py, true);
        write_text(py, text)
    }

    #[pyo3(signature = (timestamp=None))]
    fn flush(&self, py: Python<'_>, timestamp: Option<i64>) -> PyResult<()> {
        let (text, file) = {
            let mut st = self.state.lock();
            st.prepare_flush(timestamp)?;
            let file = st.file.as_ref().map(|file| file.clone_ref(py));
            (st.take_text(py, true), file)
        };
        write_text(py, text)?;
        if let Some(file) = file {
            file.bind(py).call_method0("flush")?;
        }
        Ok(())
    }

    /// Write what is left, the file stays open
    #[pyo3(signature = (timestamp=None))]
    fn close(&self, py: Python<'_>, timestamp: Option<i64>) -> PyResult<()> {
        let (text, file) = {
            let mut st = self.state.lock();
            if st.closed {
                return Ok(());
            }
            st.prepare_flush(timestamp)?;
            let text = st.take_text(py, true);
            st.closed = true;
            (text, st.file.take())
        };
        write_text(py, text)?;
        if let Some(file) = file {
            file.bind(py).call_method0("flush")?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identifiers() {
        assert_eq!(identifier(1), "!");
        assert_eq!(identifier(94), "~");
        assert_eq!(identifier(95), "!!");
    }

    #[test]
    fn sizes() {
        assert_eq!(std::mem::size_of::<V>(), 16);
        assert_eq!(std::mem::size_of::<Var>(), 32);
    }
}
