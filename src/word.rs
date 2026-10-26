// Copyright (c) Fredrik Andersson, 2023-2026
// All rights reserved

//! The functional (word-level) Yosys cells, the same as _yosys_functional.py: a value is
//! the bits and a mask of the undefined bits, as unbounded numbers like Python's ints.

use crate::bits::Bits;
use num_bigint::{BigInt, BigUint, Sign};
use num_integer::Integer;
use num_traits::{One, Signed, ToPrimitive, Zero};
use std::collections::HashMap;

// Values of one-bit ports, as in lib.rs
const X: u8 = 2;
const Z: u8 = 3;

pub type W = Bits;

/// A value: the bits, and the undefined bits
#[derive(Clone, PartialEq, Default)]
pub struct Word {
    pub v: W,
    pub u: W,
}

impl Word {
    fn new(v: W, u: W) -> Word {
        Word { v, u }
    }

    fn bit(value: u32, undefined: u32) -> Word {
        Word::new(W::from(value), W::from(undefined))
    }
}

pub fn mask(width: usize) -> W {
    Bits::mask(width)
}

/// a & ~b
fn andnot(a: &W, b: &W) -> W {
    a.andnot(b)
}

fn bit(value: &W, index: usize) -> bool {
    value.bit(index)
}

fn big(value: &W) -> BigInt {
    BigInt::from(BigUint::from(value))
}

/// A value of 'width' bits as a two's complement number
fn signed(value: &W, width: usize) -> BigInt {
    let number = big(value);
    if width > 0 && bit(value, width - 1) {
        number - (BigInt::one() << width)
    } else {
        number
    }
}

/// A number cut to 'width' bits (two's complement for a negative number)
fn bits_of(number: &BigInt, width: usize) -> W {
    let modulus = BigInt::one() << width;
    Bits::from(number.mod_floor(&modulus).to_biguint().unwrap_or_default())
}

/// A small number cut to 'width' bits, up to 64 (two's complement for a negative number)
fn small_bits_of(number: i128, width: usize) -> W {
    let modulus = 1i128 << width;
    Bits::from(number.rem_euclid(modulus) as u64)
}

/// A value extended (or cut) to new_width bits, with its sign bit if signed
fn extend(word: &Word, width: usize, signed: bool, new_width: usize) -> Word {
    if new_width == width {
        return word.clone();
    }
    let (mut v, mut u) = (word.v.clone(), word.u.clone());
    if new_width > width && signed && width > 0 {
        let fill = andnot(&mask(new_width), &mask(width));
        if bit(&u, width - 1) {
            u |= &fill;
        } else if bit(&v, width - 1) {
            v |= &fill;
        }
    }
    let m = mask(new_width);
    Word::new(v & &m, u & &m)
}

fn undefined(width: usize) -> Word {
    Word::new(W::zero(), mask(width))
}

/// The bits that are known to be 1, and the bits that are known to be 0
fn known(word: &Word, m: &W) -> (W, W) {
    (
        andnot(&(&word.v & m), &word.u),
        andnot(m, &(&word.v | &word.u)),
    )
}

fn reduce_or(word: &Word) -> Option<u32> {
    if !andnot(&word.v, &word.u).is_zero() {
        return Some(1);
    }
    if word.u.is_zero() {
        Some(0)
    } else {
        None
    }
}

fn to_usize(value: &W) -> usize {
    value.to_usize().unwrap_or(usize::MAX)
}

/// A value of up to 64 bits as a number, signed (two's complement) or not
fn small(value: &W, width: usize, signed: bool) -> i128 {
    let v = value.to_u64().unwrap_or(0) as i128;
    if signed && width > 0 && (v >> (width - 1)) & 1 == 1 {
        v - (1i128 << width)
    } else {
        v
    }
}

fn shift_left(word: &Word, amount: usize, width: usize) -> Word {
    if amount >= width {
        return Word::default();
    }
    Word::new(&word.v << amount, &word.u << amount)
}

/// Shift right, the top bits get the value of 'fill' (a sign bit)
fn shift_right(word: &Word, amount: usize, width: usize, fill: (bool, bool)) -> Word {
    let amount = amount.min(width);
    let fill_mask = andnot(&mask(width), &mask(width - amount));
    let mut v = &word.v >> amount;
    let mut u = &word.u >> amount;
    if fill.0 {
        v |= &fill_mask;
    }
    if fill.1 {
        u |= &fill_mask;
    }
    Word::new(v, u)
}

fn select(s: &Word, a: &Word, b: &Word, m: &W) -> Word {
    if s.u.is_zero() {
        return if s.v.is_zero() { a.clone() } else { b.clone() };
    }
    let same = andnot(&andnot(&andnot(m, &(&a.v ^ &b.v)), &a.u), &b.u);
    Word::new(&a.v & &same, andnot(m, &same))
}

fn set_clear(q: &Word, set: &Word, clear: &Word, set_on: bool, clear_on: bool, m: &W) -> Word {
    let active = |word: &Word, on: bool| {
        if on {
            andnot(&word.v, &word.u) & m
        } else {
            andnot(&andnot(m, &word.v), &word.u)
        }
    };
    let clear_active = active(clear, clear_on);
    let set_active = andnot(&active(set, set_on), &clear_active);
    let v = andnot(&(&q.v | &set_active), &clear_active);
    let u = andnot(&q.u, &(&set_active | &clear_active));
    Word::new(v, u)
}

#[derive(Clone, Copy, PartialEq)]
pub enum Kind {
    Not,
    Pos,
    Neg,
    And,
    Or,
    Xor,
    Xnor,
    ReduceAnd,
    ReduceOr,
    ReduceXor,
    ReduceXnor,
    LogicNot,
    LogicAnd,
    LogicOr,
    Shl,
    Shr,
    Sshr,
    Shift,
    Shiftx,
    Lt,
    Le,
    Gt,
    Ge,
    Eq,
    Ne,
    Eqx,
    Nex,
    Add,
    Sub,
    Mul,
    Div,
    Mod,
    DivFloor,
    ModFloor,
    Pow,
    Mux,
    Pmux,
    Bmux,
    Demux,
    Bwmux,
    Dff,
    Dffe,
    Adff,
    Adffe,
    Aldff,
    Aldffe,
    Sdff,
    Sdffe,
    Sdffce,
    Dffsr,
    Dffsre,
    Dlatch,
    Adlatch,
    Dlatchsr,
    Sr,
    Decoder,
    Alu,
    Lcu,
    Fa,
    MaccV2,
    Lut,
    Sop,
    Bweqx,
    Concat,
    Slice,
    Tribuf,
}

pub const KINDS: [Kind; 66] = {
    use Kind::*;
    [
        Not, Pos, Neg, And, Or, Xor, Xnor, ReduceAnd, ReduceOr, ReduceXor, ReduceXnor, LogicNot,
        LogicAnd, LogicOr, Shl, Shr, Sshr, Shift, Shiftx, Lt, Le, Gt, Ge, Eq, Ne, Eqx, Nex, Add,
        Sub, Mul, Div, Mod, DivFloor, ModFloor, Pow, Mux, Pmux, Bmux, Demux, Bwmux, Dff, Dffe,
        Adff, Adffe, Aldff, Aldffe, Sdff, Sdffe, Sdffce, Dffsr, Dffsre, Dlatch, Adlatch, Dlatchsr,
        Sr, Decoder, Alu, Lcu, Fa, MaccV2, Lut, Sop, Bweqx, Concat, Slice, Tribuf,
    ]
};

pub struct Input {
    pub word: Word,
    pub width: usize,
    // A change updates the cell (not only read at a clock edge)
    pub update: bool,
}

pub struct Output {
    pub mask: W,
    pub width: usize,
    // The last value written, None before the first one
    pub last: Option<Word>,
    // The ports of the bits, and the bits whose ports drive something
    pub bit_ports: Vec<u32>,
    pub bit_outputs: W,
    // The port of the word, for the functional cells it drives: for each cell, its
    // (input, offset in the input, offset in this output, number of bits)
    pub word_port: Option<u32>,
    pub links: Vec<Link>,
}

/// The cells a word output drives: the cell, and its (input, offset in the input, offset
/// in the output, number of bits)
pub type Link = (usize, Vec<(usize, usize, usize, usize)>);

/// An event of a cell: a bit port and its value, or the word port and its value
pub enum Write {
    Bit(u32, u8),
    Word(u32, W),
}

pub struct Cell {
    pub kind: Kind,
    pub inputs: Vec<Input>,
    pub outputs: Vec<Output>,
    params: HashMap<String, BigInt>,
    // Registers: Q, the last clock level and the width
    pub q: Word,
    pub old_clock: u8,
    width: usize,
    // Decoder: the values, the outputs for no match, and the outputs each value flips
    values: Vec<W>,
    none: W,
    flips: HashMap<W, W>,
    // Sum of products: for each term, the bits of A that must be 0 and those that must be 1
    terms: Vec<(W, W)>,
    // Tristate buffer: if it drives its output
    pub driving: bool,
    // The parameters an update uses, read once
    a_signed: bool,
    b_signed: bool,
    set_polarity: bool,
    clr_polarity: bool,
    clk_polarity: u8,
    en_polarity: u8,
    arst_polarity: u8,
    srst_polarity: u8,
    aload_polarity: u8,
    arst_value: Word,
    srst_value: Word,
    y_mask: W,
    lut: BigInt,
    offset: usize,
}

impl Cell {
    pub fn new(
        kind: Kind,
        inputs: Vec<Input>,
        outputs: Vec<Output>,
        params: HashMap<String, BigInt>,
        q: Word,
        old_clock: u8,
        driving: bool,
    ) -> Cell {
        let mut cell = Cell {
            kind,
            inputs,
            outputs,
            params,
            q,
            old_clock,
            width: 0,
            values: Vec::new(),
            none: W::zero(),
            flips: HashMap::new(),
            terms: Vec::new(),
            driving,
            a_signed: false,
            b_signed: false,
            set_polarity: false,
            clr_polarity: false,
            clk_polarity: 0,
            en_polarity: 0,
            arst_polarity: 0,
            srst_polarity: 0,
            aload_polarity: 0,
            arst_value: Word::default(),
            srst_value: Word::default(),
            y_mask: W::zero(),
            lut: BigInt::zero(),
            offset: 0,
        };
        let q_width = cell.outputs.first().map_or(0, |o| o.width);
        cell.width = cell
            .param("WIDTH", q_width as i64)
            .to_usize()
            .unwrap_or(q_width);
        cell.y_mask = mask(q_width);
        cell.a_signed = cell.flag("A_SIGNED");
        cell.b_signed = cell.flag("B_SIGNED");
        cell.set_polarity = cell.flag("SET_POLARITY");
        cell.clr_polarity = cell.flag("CLR_POLARITY");
        // A level that is never 0 or 1 is never active
        let level = |cell: &Cell, name: &str| {
            cell.param(name, 0)
                .to_u8()
                .filter(|l| *l < 2)
                .unwrap_or(u8::MAX)
        };
        cell.clk_polarity = level(&cell, "CLK_POLARITY");
        cell.en_polarity = level(&cell, "EN_POLARITY");
        cell.arst_polarity = level(&cell, "ARST_POLARITY");
        cell.srst_polarity = level(&cell, "SRST_POLARITY");
        cell.aload_polarity = level(&cell, "ALOAD_POLARITY");
        let value = |cell: &Cell, name: &str| {
            let bits = Bits::from(cell.param(name, 0).to_biguint().unwrap_or_default());
            let bits = bits & mask(cell.width);
            Word::new(bits, W::zero())
        };
        cell.arst_value = value(&cell, "ARST_VALUE");
        cell.srst_value = value(&cell, "SRST_VALUE");
        if kind == Kind::Decoder {
            let count = cell.outputs[0].width;
            for index in 0..count {
                let value = cell.param(&format!("VALUE{index}"), 0);
                let value = Bits::from(value.to_biguint().unwrap_or_default());
                if cell.param(&format!("EQUAL{index}"), 1).is_zero() {
                    cell.none |= &(W::one() << index);
                }
                let flip = cell.flips.entry(value.clone()).or_default();
                *flip |= &(W::one() << index);
                cell.values.push(value);
            }
        }
        cell.lut = cell.param("LUT", 0);
        cell.offset = cell.param("OFFSET", 0).to_usize().unwrap_or(usize::MAX);
        if kind == Kind::Sop {
            let width = cell.width_of(0);
            let table = cell.param("TABLE", 0);
            let depth = cell.param("DEPTH", 0).to_usize().unwrap_or(0);
            for term in 0..depth {
                let (mut zeros, mut ones) = (W::zero(), W::zero());
                for index in 0..width {
                    let pair = (&table >> (2 * width * term + 2 * index)) & BigInt::from(3);
                    let pair = pair.to_u8().unwrap_or(0);
                    if pair & 1 != 0 {
                        zeros.set_bit(index, true);
                    }
                    if pair & 2 != 0 {
                        ones.set_bit(index, true);
                    }
                }
                cell.terms.push((zeros, ones));
            }
        }
        cell
    }

    fn param(&self, name: &str, default: i64) -> BigInt {
        self.params
            .get(name)
            .cloned()
            .unwrap_or_else(|| BigInt::from(default))
    }

    fn flag(&self, name: &str) -> bool {
        !self.param(name, 0).is_zero()
    }

    fn read(&self, input: usize) -> &Word {
        &self.inputs[input].word
    }

    fn width_of(&self, input: usize) -> usize {
        self.inputs[input].width
    }

    /// A one-bit input as a value code: 0, 1 or X
    fn read_bit(&self, input: usize) -> u8 {
        let word = self.read(input);
        if bit(&word.u, 0) {
            X
        } else {
            bit(&word.v, 0) as u8
        }
    }

    fn active(&self, input: usize, polarity: u8) -> bool {
        self.read_bit(input) == polarity
    }

    fn clock_edge(&mut self, input: usize) -> bool {
        let clock = self.read_bit(input);
        let edge = clock != self.old_clock && clock == self.clk_polarity;
        self.old_clock = clock;
        edge
    }

    /// _CellInput.set_value(): one bit of an input changed. Returns if the cell updates.
    pub fn set_bit(&mut self, input: usize, index: usize, value: u8) -> bool {
        let word = &mut self.inputs[input].word;
        let bit_mask = W::one() << index;
        match value {
            1 => {
                word.v |= &bit_mask;
                word.u = andnot(&word.u, &bit_mask);
            }
            0 => {
                word.v = andnot(&word.v, &bit_mask);
                word.u = andnot(&word.u, &bit_mask);
            }
            _ => {
                word.v = andnot(&word.v, &bit_mask);
                word.u |= &bit_mask;
            }
        }
        self.inputs[input].update
    }

    /// FunctionalCell.set_bits(): bits of an input from a word output. Returns if the
    /// cell updates: if the bits changed and the input updates the cell.
    pub fn set_bits(&mut self, input: usize, offset: usize, width: usize, word: &Word) -> bool {
        let m = mask(width) << offset;
        let current = &self.inputs[input].word;
        let v = andnot(&current.v, &m) | (&word.v << offset);
        let u = andnot(&current.u, &m) | (&word.u << offset);
        if v == current.v && u == current.u {
            return false;
        }
        self.inputs[input].word = Word::new(v, u);
        self.inputs[input].update
    }

    /// FunctionalCell.write(): the events of a new value of an output
    fn write(&mut self, output: usize, word: Word, events: &mut Vec<Write>) {
        let out = &mut self.outputs[output];
        let u = &word.u & &out.mask;
        let v = andnot(&(&word.v & &out.mask), &u);
        let mut changed = match &out.last {
            None => out.mask.clone(),
            Some(last) => {
                let changed = (&v ^ &last.v) | (&u ^ &last.u);
                if changed.is_zero() {
                    return;
                }
                changed
            }
        };
        if let Some(port) = out.word_port {
            events.push(Write::Word(port, &v | (&u << out.width)));
        }
        changed &= &out.bit_outputs;
        for index in changed.ones() {
            let code = if bit(&u, index) {
                X
            } else {
                bit(&v, index) as u8
            };
            events.push(Write::Bit(out.bit_ports[index], code));
        }
        out.last = Some(Word::new(v, u));
    }

    fn set_q(&mut self, word: Word, events: &mut Vec<Write>) {
        self.q = word.clone();
        self.write(0, word, events);
    }

    pub fn default_state(&mut self, events: &mut Vec<Write>) {
        self.q = Word::default();
        self.write(0, Word::default(), events);
    }

    pub fn is_register(&self) -> bool {
        use Kind::*;
        matches!(
            self.kind,
            Dff | Dffe
                | Adff
                | Adffe
                | Aldff
                | Aldffe
                | Sdff
                | Sdffe
                | Sdffce
                | Dffsr
                | Dffsre
                | Dlatch
                | Adlatch
                | Dlatchsr
                | Sr
        )
    }

    /// FunctionalCell.update(): the events of the new outputs
    pub fn update(&mut self, events: &mut Vec<Write>) {
        use Kind::*;
        let y_width = self.outputs.first().map_or(0, |o| o.width);
        let y_mask = &self.y_mask.clone();
        let word = match self.kind {
            Not | Pos | Neg => {
                let a = extend(self.read(0), self.width_of(0), self.a_signed, y_width);
                match self.kind {
                    Not => Word::new(andnot(&andnot(y_mask, &a.v), &a.u), a.u),
                    Pos => a,
                    _ => {
                        if !a.u.is_zero() {
                            undefined(y_width)
                        } else {
                            Word::new(bits_of(&-big(&a.v), y_width), W::zero())
                        }
                    }
                }
            }
            ReduceAnd | ReduceOr | ReduceXor | ReduceXnor | LogicNot => {
                let a = self.read(0);
                let m = mask(self.width_of(0));
                let result = match self.kind {
                    ReduceAnd => {
                        if !andnot(&m, &(&a.v | &a.u)).is_zero() {
                            Some(0)
                        } else if a.u.is_zero() {
                            Some(1)
                        } else {
                            None
                        }
                    }
                    ReduceOr => reduce_or(a),
                    ReduceXor | ReduceXnor => {
                        if a.u.is_zero() {
                            let parity = (a.v.count_ones() & 1) as u32;
                            Some(if self.kind == ReduceXor {
                                parity
                            } else {
                                1 - parity
                            })
                        } else {
                            None
                        }
                    }
                    _ => reduce_or(a).map(|r| 1 - r),
                };
                match result {
                    None => Word::bit(0, 1),
                    Some(r) => Word::bit(r, 0),
                }
            }
            And | Or | Xor | Xnor => {
                let signed = self.a_signed && self.b_signed;
                let a = extend(self.read(0), self.width_of(0), signed, y_width);
                let b = extend(self.read(1), self.width_of(1), signed, y_width);
                let m = y_mask;
                match self.kind {
                    And | Or => {
                        let (a1, a0) = known(&a, m);
                        let (b1, b0) = known(&b, m);
                        if self.kind == And {
                            let ones = &a1 & &b1;
                            let u = andnot(m, &(&ones | &a0 | &b0));
                            Word::new(ones, u)
                        } else {
                            let ones = &a1 | &b1;
                            let u = andnot(m, &(&ones | (&a0 & &b0)));
                            Word::new(ones, u)
                        }
                    }
                    _ => {
                        let u = (&a.u | &b.u) & m;
                        let x = &a.v ^ &b.v;
                        if self.kind == Xor {
                            Word::new(andnot(&(&x & m), &u), u)
                        } else {
                            Word::new(andnot(&andnot(m, &x), &u), u)
                        }
                    }
                }
            }
            LogicAnd | LogicOr => {
                let a = reduce_or(self.read(0));
                let b = reduce_or(self.read(1));
                let result = if self.kind == LogicAnd {
                    if a == Some(0) || b == Some(0) {
                        Some(0)
                    } else if a == Some(1) && b == Some(1) {
                        Some(1)
                    } else {
                        None
                    }
                } else if a == Some(1) || b == Some(1) {
                    Some(1)
                } else if a == Some(0) && b == Some(0) {
                    Some(0)
                } else {
                    None
                };
                match result {
                    None => Word::bit(0, 1),
                    Some(r) => Word::bit(r, 0),
                }
            }
            Shl | Shr | Sshr | Shift => {
                let b = self.read(1).clone();
                if !b.u.is_zero() {
                    undefined(y_width)
                } else {
                    let a_signed = self.a_signed;
                    let width = self.width_of(0).max(y_width);
                    let a = extend(self.read(0), self.width_of(0), a_signed, width);
                    let shifted = match self.kind {
                        Shl => shift_left(&a, to_usize(&b.v), width),
                        Shr => shift_right(&a, to_usize(&b.v), width, (false, false)),
                        Sshr => {
                            let sign = if a_signed {
                                (bit(&a.v, width - 1), bit(&a.u, width - 1))
                            } else {
                                (false, false)
                            };
                            shift_right(&a, to_usize(&b.v), width, sign)
                        }
                        _ => {
                            let amount = if self.b_signed {
                                signed(&b.v, self.width_of(1))
                            } else {
                                big(&b.v)
                            };
                            let distance = amount.abs().to_usize().unwrap_or(usize::MAX);
                            if amount.sign() != Sign::Minus {
                                shift_right(&a, distance, width, (false, false))
                            } else {
                                shift_left(&a, distance, width)
                            }
                        }
                    };
                    Word::new(shifted.v & y_mask, shifted.u & y_mask)
                }
            }
            Shiftx => {
                let b = self.read(1);
                if !b.u.is_zero() {
                    undefined(y_width)
                } else {
                    let amount = if self.b_signed {
                        signed(&b.v, self.width_of(1))
                    } else {
                        big(&b.v)
                    };
                    let a = self.read(0);
                    let a_width = BigInt::from(self.width_of(0));
                    let mut v = W::zero();
                    let mut u = W::zero();
                    for index in 0..y_width {
                        let source = BigInt::from(index) + &amount;
                        if source.sign() != Sign::Minus && source < a_width {
                            let source = source.to_usize().unwrap_or(0);
                            if bit(&a.v, source) {
                                v.set_bit(index, true);
                            }
                            if bit(&a.u, source) {
                                u.set_bit(index, true);
                            }
                        } else {
                            u.set_bit(index, true);
                        }
                    }
                    Word::new(v, u)
                }
            }
            Lt | Le | Gt | Ge | Add | Sub | Mul | Div | Mod | DivFloor | ModFloor
                if self.width_of(0) <= 64 && self.width_of(1) <= 64 && y_width <= 64 =>
            {
                // Without allocating: the operands and the result fit in an i128
                let (a, b) = (self.read(0), self.read(1));
                if !a.u.is_zero() || !b.u.is_zero() {
                    match self.kind {
                        Lt | Le | Gt | Ge => Word::bit(0, 1),
                        _ => undefined(y_width),
                    }
                } else {
                    let signed = self.a_signed && self.b_signed;
                    let a = small(&a.v, self.width_of(0), signed);
                    let b = small(&b.v, self.width_of(1), signed);
                    let result = match self.kind {
                        Lt => Some((a < b) as i128),
                        Le => Some((a <= b) as i128),
                        Gt => Some((a > b) as i128),
                        Ge => Some((a >= b) as i128),
                        Add => Some(a + b),
                        Sub => Some(a - b),
                        // Only the bits of Y are kept, an overflow doesn't change them
                        Mul => Some(a.wrapping_mul(b)),
                        _ if b == 0 => None,
                        Div => Some(a / b),
                        Mod => Some(a % b),
                        DivFloor => {
                            let q = a / b;
                            Some(if a % b != 0 && ((a < 0) != (b < 0)) {
                                q - 1
                            } else {
                                q
                            })
                        }
                        _ => {
                            let r = a % b;
                            Some(if r != 0 && ((r < 0) != (b < 0)) {
                                r + b
                            } else {
                                r
                            })
                        }
                    };
                    match result {
                        None => undefined(y_width),
                        Some(r) if matches!(self.kind, Lt | Le | Gt | Ge) => Word::bit(r as u32, 0),
                        Some(r) => Word::new(small_bits_of(r, y_width), W::zero()),
                    }
                }
            }
            Lt | Le | Gt | Ge | Add | Sub | Mul | Div | Mod | DivFloor | ModFloor | Pow => {
                let (a, b) = (self.read(0), self.read(1));
                let operands = if !a.u.is_zero() || !b.u.is_zero() {
                    None
                } else if self.a_signed && self.b_signed {
                    Some((
                        signed(&a.v, self.width_of(0)),
                        signed(&b.v, self.width_of(1)),
                    ))
                } else {
                    Some((big(&a.v), big(&b.v)))
                };
                match self.kind {
                    Lt | Le | Gt | Ge => match operands {
                        None => Word::bit(0, 1),
                        Some((a, b)) => {
                            let result = match self.kind {
                                Lt => a < b,
                                Le => a <= b,
                                Gt => a > b,
                                _ => a >= b,
                            };
                            Word::bit(result as u32, 0)
                        }
                    },
                    _ => {
                        let result = operands.and_then(|(a, b)| self.arithmetic(a, b, y_width));
                        match result {
                            None => undefined(y_width),
                            Some(r) => Word::new(bits_of(&r, y_width), W::zero()),
                        }
                    }
                }
            }
            Eq | Ne | Eqx | Nex => {
                let equal = matches!(self.kind, Eq | Eqx) as u32;
                let (a_width, b_width) = (self.width_of(0), self.width_of(1));
                let signed = self.a_signed && self.b_signed;
                let width = a_width.max(b_width).max(y_width);
                let (a_word, b_word) = (self.read(0), self.read(1));
                if matches!(self.kind, Eqx | Nex) {
                    let a = extend(a_word, a_width, signed, width);
                    let b = extend(b_word, b_width, signed, width);
                    Word::bit(if a == b { equal } else { 1 - equal }, 0)
                } else if a_word.u.is_zero() && b_word.u.is_zero() && a_width == b_width {
                    Word::bit(
                        if a_word.v == b_word.v {
                            equal
                        } else {
                            1 - equal
                        },
                        0,
                    )
                } else {
                    let a = extend(a_word, a_width, signed, width);
                    let b = extend(b_word, b_width, signed, width);
                    let m = mask(width);
                    let (a1, a0) = known(&a, &m);
                    let (b1, b0) = known(&b, &m);
                    if !((&a1 & &b0) | (&a0 & &b1)).is_zero() {
                        Word::bit(1 - equal, 0)
                    } else if !(&a.u | &b.u).is_zero() {
                        Word::bit(0, 1)
                    } else {
                        Word::bit(equal, 0)
                    }
                }
            }
            Mux => select(self.read(2), self.read(0), self.read(1), y_mask),
            Pmux => {
                let s = self.read(2);
                // More than one bit of S set
                let several = !s.v.is_zero() && !(&s.v & (&s.v - 1u32)).is_zero();
                if !s.u.is_zero() || several {
                    undefined(y_width)
                } else if s.v.is_zero() {
                    self.read(0).clone()
                } else {
                    let b = self.read(1);
                    let shift = (s.v.bits() as usize - 1) * y_width;
                    Word::new((&b.v >> shift) & y_mask, (&b.u >> shift) & y_mask)
                }
            }
            Bmux => {
                let s = self.read(1);
                if !s.u.is_zero() {
                    undefined(y_width)
                } else {
                    let a = self.read(0);
                    let shift = to_usize(&s.v).saturating_mul(y_width);
                    Word::new((&a.v >> shift) & y_mask, (&a.u >> shift) & y_mask)
                }
            }
            Demux => {
                let s = self.read(1);
                if !s.u.is_zero() {
                    undefined(y_width)
                } else {
                    let a = self.read(0);
                    let shift = to_usize(&s.v) * self.width_of(0);
                    Word::new(&a.v << shift, &a.u << shift)
                }
            }
            Bwmux => {
                let (a, b, s) = (self.read(0), self.read(1), self.read(2));
                let v = andnot(&a.v, &s.v) | (&b.v & &s.v);
                let u = andnot(&a.u, &s.v) | (&b.u & &s.v);
                let differ = (&a.v ^ &b.v) | &a.u | &b.u;
                let u = andnot(&u, &s.u) | (&s.u & &differ);
                let v = andnot(&v, &u);
                Word::new(v & y_mask, u & y_mask)
            }
            Decoder => {
                let a = self.read(0);
                if a.u.is_zero() {
                    let flips = self.flips.get(&a.v).cloned().unwrap_or_default();
                    Word::new(&self.none ^ flips, W::zero())
                } else {
                    let mut u = W::zero();
                    for (index, constant) in self.values.iter().enumerate() {
                        if !andnot(&(&a.v ^ constant), &a.u).is_zero() {
                            continue;
                        }
                        u.set_bit(index, true);
                    }
                    Word::new(andnot(&self.none, &u), u)
                }
            }
            Alu | Lcu | Fa | MaccV2 | Lut | Sop | Bweqx | Concat | Slice | Tribuf => {
                self.update_other(events);
                return;
            }
            _ => {
                self.update_register(events);
                return;
            }
        };
        self.write(0, word, events);
    }

    /// The (offset, width, signed) of each part of a port of $macc_v2
    fn parts(&self, count: usize, widths: &str, signed: &str) -> Vec<(usize, usize, bool)> {
        let (widths, signed) = (self.param(widths, 0), self.param(signed, 0));
        let mut offset = 0;
        (0..count)
            .map(|index| {
                let width = ((&widths >> (16 * index)) & BigInt::from(0xFFFF))
                    .to_usize()
                    .unwrap_or(0);
                let part_signed = !((&signed >> index) & BigInt::one()).is_zero();
                let part = (offset, width, part_signed);
                offset += width;
                part
            })
            .collect()
    }

    // The cells of other synthesis flows, their inputs and outputs in the order of
    // _yosys_functional.py
    fn update_other(&mut self, events: &mut Vec<Write>) {
        use Kind::*;
        match self.kind {
            // A, B, CI, BI; X, Y, CO
            Alu => {
                let width = self.outputs[1].width;
                let m = mask(width);
                let signed = self.a_signed && self.b_signed;
                let a = extend(self.read(0), self.width_of(0), signed, width);
                let mut b = extend(self.read(1), self.width_of(1), signed, width);
                let ci = self.read(2).clone();
                let bi = self.read(3);
                let bi_undefined = !bi.u.is_zero();
                if bi_undefined {
                    b = undefined(width);
                } else if !bi.v.is_zero() {
                    b = Word::new(andnot(&andnot(&m, &b.v), &b.u), b.u);
                }
                let u = (&a.u | &b.u) & &m;
                self.write(0, Word::new(andnot(&(&a.v ^ &b.v), &u), u.clone()), events);
                if !u.is_zero() || !ci.u.is_zero() || bi_undefined {
                    // As in Verilog, an undefined bit makes the sum and the carries undefined
                    self.write(1, undefined(width), events);
                    self.write(2, undefined(width), events);
                    return;
                }
                let total =
                    Bits::from(BigUint::from(&a.v) + BigUint::from(&b.v) + BigUint::from(&ci.v));
                self.write(1, Word::new(&total & &m, W::zero()), events);
                // The carry into bit i+1 is the sum bit XOR the operand bits, for bit i+1
                let carries = ((total ^ &a.v) ^ &b.v) >> 1;
                self.write(2, Word::new(carries & &m, W::zero()), events);
            }
            // P, G, CI; CO
            Lcu => {
                let width = self.outputs[0].width;
                let (p, g, ci) = (self.read(0), self.read(1), self.read(2));
                let word = if !p.u.is_zero() || !g.u.is_zero() || !ci.u.is_zero() {
                    undefined(width)
                } else {
                    let mut carry = bit(&ci.v, 0);
                    let mut co = W::zero();
                    for index in 0..width {
                        carry = bit(&g.v, index) || (bit(&p.v, index) && carry);
                        if carry {
                            co.set_bit(index, true);
                        }
                    }
                    Word::new(co, W::zero())
                };
                self.write(0, word, events);
            }
            // A, B, C; X (the carry), Y (the sum)
            Fa => {
                let m = mask(self.outputs[1].width);
                let (a, b, c) = (self.read(0), self.read(1), self.read(2));
                let u = (&a.u | &b.u | &c.u) & &m;
                let y = andnot(&(&a.v ^ &b.v ^ &c.v), &u);
                let x = andnot(&((&a.v & &b.v) | (&c.v & (&a.v ^ &b.v))), &u);
                self.write(1, Word::new(y, u.clone()), events);
                self.write(0, Word::new(x, u), events);
            }
            // A, B, C; Y
            MaccV2 => {
                let width = self.outputs[0].width;
                let (a, b, c) = (self.read(0), self.read(1), self.read(2));
                if !a.u.is_zero() || !b.u.is_zero() || !c.u.is_zero() {
                    self.write(0, undefined(width), events);
                    return;
                }
                let products = self.param("NPRODUCTS", 0).to_usize().unwrap_or(0);
                let addends = self.param("NADDENDS", 0).to_usize().unwrap_or(0);
                let a_parts = self.parts(products, "A_WIDTHS", "A_SIGNED");
                let b_parts = self.parts(products, "B_WIDTHS", "B_SIGNED");
                let c_parts = self.parts(addends, "C_WIDTHS", "C_SIGNED");
                let product_negated = self.param("PRODUCT_NEGATED", 0);
                let addend_negated = self.param("ADDEND_NEGATED", 0);
                let operand = |value: &W, (offset, width, _): (usize, usize, bool), signed_part| {
                    let bits = (value >> offset) & mask(width);
                    if signed_part {
                        signed(&bits, width)
                    } else {
                        big(&bits)
                    }
                };
                let negated =
                    |flags: &BigInt, index: usize| !((flags >> index) & BigInt::one()).is_zero();
                let mut total = BigInt::zero();
                for index in 0..products {
                    // Signed only if both factors are
                    let signed_part = a_parts[index].2 && b_parts[index].2;
                    let product = operand(&a.v, a_parts[index], signed_part)
                        * operand(&b.v, b_parts[index], signed_part);
                    if negated(&product_negated, index) {
                        total -= product;
                    } else {
                        total += product;
                    }
                }
                for (index, part) in c_parts.iter().enumerate() {
                    let addend = operand(&c.v, *part, part.2);
                    if negated(&addend_negated, index) {
                        total -= addend;
                    } else {
                        total += addend;
                    }
                }
                self.write(0, Word::new(bits_of(&total, width), W::zero()), events);
            }
            // A; Y
            Lut => {
                let a = self.read(0);
                let word = if !a.u.is_zero() {
                    Word::bit(0, 1)
                } else {
                    let index = to_usize(&a.v);
                    let one = !((&self.lut >> index) & BigInt::one()).is_zero();
                    Word::bit(one as u32, 0)
                };
                self.write(0, word, events);
            }
            // A; Y
            Sop => {
                let a = self.read(0);
                let mut maybe = false;
                let mut word = None;
                for (zeros, ones) in &self.terms {
                    let known_one = andnot(&(&a.v & zeros), &a.u);
                    let known_zero = andnot(&andnot(ones, &a.v), &a.u);
                    if !known_one.is_zero() || !known_zero.is_zero() {
                        // A known bit doesn't match the term
                        continue;
                    }
                    if (&a.u & (zeros | ones)).is_zero() {
                        word = Some(Word::bit(1, 0));
                        break;
                    }
                    maybe = true;
                }
                let word = word.unwrap_or_else(|| Word::bit(0, maybe as u32));
                self.write(0, word, events);
            }
            // A, B; Y
            Bweqx => {
                let (a, b) = (self.read(0), self.read(1));
                let differ = (&a.v ^ &b.v) | (&a.u ^ &b.u);
                let word = Word::new(andnot(&self.outputs[0].mask, &differ), W::zero());
                self.write(0, word, events);
            }
            // A, B; Y = {B, A}
            Concat => {
                let shift = self.width_of(0);
                let (a, b) = (self.read(0), self.read(1));
                let word = Word::new(&a.v | (&b.v << shift), &a.u | (&b.u << shift));
                self.write(0, word, events);
            }
            // A; Y = A >> OFFSET
            Slice => {
                let a = self.read(0);
                let word = Word::new(&a.v >> self.offset, &a.u >> self.offset);
                self.write(0, word, events);
            }
            // A, EN; Y
            Tribuf => {
                let enable = self.read_bit(1);
                if enable == 0 {
                    if self.driving {
                        // Not driven: the bits are Z (for a cell it drives as a word, X)
                        self.driving = false;
                        let out = &mut self.outputs[0];
                        for port in &out.bit_ports {
                            events.push(Write::Bit(*port, Z));
                        }
                        if let Some(port) = out.word_port {
                            events.push(Write::Word(port, &out.mask << out.width));
                        }
                        // All bits get an event when it drives again
                        out.last = None;
                    }
                    return;
                }
                self.driving = true;
                let width = self.outputs[0].width;
                let word = if enable == 1 {
                    self.read(0).clone()
                } else {
                    undefined(width)
                };
                self.write(0, word, events);
            }
            _ => {}
        }
    }

    fn arithmetic(&self, a: BigInt, b: BigInt, y_width: usize) -> Option<BigInt> {
        use Kind::*;
        Some(match self.kind {
            Add => a + b,
            Sub => a - b,
            Mul => a * b,
            Div | Mod | DivFloor | ModFloor => {
                if b.is_zero() {
                    return None;
                }
                match self.kind {
                    // Rounded towards zero, the remainder with the sign of A, as in Verilog
                    Div => a / b,
                    Mod => a % b,
                    DivFloor => a.div_floor(&b),
                    _ => a.mod_floor(&b),
                }
            }
            _ => {
                if b.sign() != Sign::Minus {
                    a.modpow(&b, &(BigInt::one() << y_width))
                } else if a == BigInt::one() {
                    BigInt::one()
                } else if a == -BigInt::one() {
                    if b.is_even() {
                        BigInt::one()
                    } else {
                        -BigInt::one()
                    }
                } else if a.is_zero() {
                    return None;
                } else {
                    BigInt::zero()
                }
            }
        })
    }

    // The inputs of the registers, in the order of _FUNCTIONAL in _native.py
    fn update_register(&mut self, events: &mut Vec<Write>) {
        use Kind::*;
        let m = mask(self.width);
        match self.kind {
            // CLK, D
            Dff => {
                if self.clock_edge(0) {
                    let d = self.read(1).clone();
                    self.set_q(d, events);
                }
            }
            // CLK, EN, D
            Dffe => {
                if self.clock_edge(0) && self.active(1, self.en_polarity) {
                    let d = self.read(2).clone();
                    self.set_q(d, events);
                }
            }
            // CLK, ARST, (EN), D
            Adff | Adffe => {
                let edge = self.clock_edge(0);
                let enable = self.kind == Adffe;
                let d = if enable { 3 } else { 2 };
                if self.active(1, self.arst_polarity) {
                    let q = self.arst_value.clone();
                    self.set_q(q, events);
                } else if edge && (!enable || self.active(2, self.en_polarity)) {
                    let d = self.read(d).clone();
                    self.set_q(d, events);
                }
            }
            // CLK, ALOAD, AD, (EN), D
            Aldff | Aldffe => {
                let edge = self.clock_edge(0);
                let enable = self.kind == Aldffe;
                let d = if enable { 4 } else { 3 };
                if self.active(1, self.aload_polarity) {
                    let q = self.read(2).clone();
                    self.set_q(q, events);
                } else if edge && (!enable || self.active(3, self.en_polarity)) {
                    let d = self.read(d).clone();
                    self.set_q(d, events);
                }
            }
            // CLK, SRST, D
            Sdff => {
                if self.clock_edge(0) {
                    let q = if self.active(1, self.srst_polarity) {
                        self.srst_value.clone()
                    } else {
                        self.read(2).clone()
                    };
                    self.set_q(q, events);
                }
            }
            // CLK, SRST, EN, D
            Sdffe => {
                if !self.clock_edge(0) {
                    return;
                }
                if self.active(1, self.srst_polarity) {
                    let q = self.srst_value.clone();
                    self.set_q(q, events);
                } else if self.active(2, self.en_polarity) {
                    let d = self.read(3).clone();
                    self.set_q(d, events);
                }
            }
            Sdffce => {
                if !self.clock_edge(0) || !self.active(2, self.en_polarity) {
                    return;
                }
                let q = if self.active(1, self.srst_polarity) {
                    self.srst_value.clone()
                } else {
                    self.read(3).clone()
                };
                self.set_q(q, events);
            }
            // CLK, SET, CLR, (EN), D
            Dffsr | Dffsre => {
                let enable = self.kind == Dffsre;
                let d = if enable { 4 } else { 3 };
                let mut q = self.q.clone();
                if self.clock_edge(0) && (!enable || self.active(3, self.en_polarity)) {
                    q = self.read(d).clone();
                }
                let q = set_clear(
                    &q,
                    self.read(1),
                    self.read(2),
                    self.set_polarity,
                    self.clr_polarity,
                    &m,
                );
                self.set_q(q, events);
            }
            // EN, D
            Dlatch => {
                if self.active(0, self.en_polarity) {
                    let d = self.read(1).clone();
                    self.set_q(d, events);
                }
            }
            // EN, ARST, D
            Adlatch => {
                if self.active(1, self.arst_polarity) {
                    let q = self.arst_value.clone();
                    self.set_q(q, events);
                } else if self.active(0, self.en_polarity) {
                    let d = self.read(2).clone();
                    self.set_q(d, events);
                }
            }
            // EN, SET, CLR, D
            Dlatchsr => {
                let q = if self.active(0, self.en_polarity) {
                    self.read(3).clone()
                } else {
                    self.q.clone()
                };
                let q = set_clear(
                    &q,
                    self.read(1),
                    self.read(2),
                    self.set_polarity,
                    self.clr_polarity,
                    &m,
                );
                self.set_q(q, events);
            }
            // SET, CLR
            Sr => {
                let q = set_clear(
                    &self.q,
                    self.read(0),
                    self.read(1),
                    self.set_polarity,
                    self.clr_polarity,
                    &m,
                );
                self.set_q(q, events);
            }
            _ => {}
        }
    }
}
