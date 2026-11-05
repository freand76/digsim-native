// Copyright (c) Fredrik Andersson, 2023-2026
// All rights reserved

//! A Yosys netlist compiled to one block, simulated like a compiled simulator (Verilator)
//! instead of with an event for every bit: every cell output is a slot of words (the
//! bits and the undefined bits), the inputs of the cells read slots (or bits gathered from
//! several), and the combinational cells are evaluated in the order of their ranks, once
//! each when an input changed. The flip-flops and latches (and the constants) update their
//! outputs in the next settle, one ns later, like their delay: a flip-flop sees the values
//! from before a clock edge.
//!
//! The values are the same as the cells in word.rs and lib.rs compute (and the Python
//! cells): common cells up to 64 bits (and the bitwise ones, multiplexers and registers
//! of any width) are computed here, the others by a word::Cell.

use crate::bits::Bits;
use crate::word::{self, Kind as W};
use crate::{eval_gate, Cell as GateCell, Kind as G};
use std::collections::HashMap;

pub const NO: u32 = u32::MAX;
const X: u8 = 2;

#[inline]
fn mask(width: u32) -> u64 {
    if width >= 64 {
        u64::MAX
    } else {
        (1u64 << width) - 1
    }
}

#[inline]
fn words(width: u32) -> u32 {
    width.div_ceil(64).max(1)
}

/// `len` bits (at most 64) from bit `bit` of the words from `base`
#[inline]
fn get_bits(array: &[u64], base: usize, bit: u32, len: u32) -> u64 {
    if len == 0 {
        return 0;
    }
    let index = base + (bit / 64) as usize;
    let offset = bit % 64;
    let mut value = array[index] >> offset;
    if offset != 0 && offset + len > 64 {
        value |= array[index + 1] << (64 - offset);
    }
    value & mask(len)
}

#[inline]
fn set_bits(array: &mut [u64], base: usize, bit: u32, len: u32, value: u64) {
    if len == 0 {
        return;
    }
    let index = base + (bit / 64) as usize;
    let offset = bit % 64;
    let m = mask(len);
    let value = value & m;
    array[index] = (array[index] & !(m << offset)) | (value << offset);
    if offset != 0 && offset + len > 64 {
        let high = 64 - offset;
        let m2 = m >> high;
        array[index + 1] = (array[index + 1] & !m2) | (value >> high);
    }
}

/// Bits of a slot copied into an operand: from (slot, bit), or undefined (slot NO)
#[derive(Clone, Copy)]
struct Seg {
    dst: u32,
    slot: u32,
    bit: u32,
    len: u32,
}

#[derive(Clone, Copy, PartialEq)]
enum Op {
    Word(W),
    Gate(G),
    Const(u8),
}

struct BCell {
    op: Op,
    // The operand slots of the inputs: Block.ins[first..first + count], and their bits to
    // gather (Block.gat, a range of segs, empty for an input that is a whole slot)
    first: u32,
    count: u32,
    gathered: bool,
    // The (first) output slot
    out: u32,
    // A flip-flop, latch or constant: its next output, from the next settle
    stateful: bool,
    // A flip-flop that only its clock updates: the inactive edge does nothing, and its
    // clock level is the level of its clock input
    clock_only: bool,
    clock_level: u8,
    // A combinational gate with up to three inputs: its part of GATE_TABLE, NO_TABLE for
    // the others
    table: u16,
    q: u32,
    old_clock: u8,
    gate: Option<GateCell>,
    extra: Option<Box<Extra>>,
    // Parameters of a functional cell: A_SIGNED, B_SIGNED (both for the operations that
    // are signed only if both are), OFFSET
    a_signed: bool,
    b_signed: bool,
    b_signed_param: bool,
    offset: usize,
}

/// What a functional cell has more: the cell of word.rs, all outputs, and for a decoder
/// the outputs for each value of A
struct Extra {
    word: word::Cell,
    outputs: Vec<u32>,
    flips: HashMap<u64, u64>,
    none: u64,
}

/// The state of a cell for Python: Q (bits, undefined bits), the clock level, and if it
/// waits for a settle
pub type CellState = (Option<(Vec<u64>, Vec<u64>)>, u8, bool);

pub struct Block {
    // The code (0, 1 or X) of bit 0 of each slot, for the one-bit inputs of gates
    c1: Vec<u8>,
    ins: Vec<u32>,
    gat: Vec<(u32, u32)>,
    start: Vec<u32>,
    width: Vec<u32>,
    v: Vec<u64>,
    x: Vec<u64>,
    segs: Vec<Seg>,
    // The constant bits of the operands: (slot, bit, value), from GND and VDD
    literals: Vec<(u32, u32, u8)>,
    cells: Vec<BCell>,
    // The cells that read each slot: fan[fan_start[s]..fan_start[s + 1]], cell << 1 |
    // (the slot is an input that updates a stateful cell)
    fan_start: Vec<u32>,
    // (entry, the first bit it reads, the bit after the last one)
    fan: Vec<(u32, u32, u32)>,
    // The ports a slot drives outside the block: (bit, port)
    pin_start: Vec<u32>,
    pins: Vec<(u32, u32)>,
    // The output ports of the cells inside, for the VCD file: (bit, port)
    vcd_start: Vec<u32>,
    vcd: Vec<(u32, u32)>,
    // The cells are ordered by rank (the combinational ones first, ncomb of them): a
    // settle evaluates the dirty ones in the order of their numbers. order[cell] is the
    // number Python gave the cell.
    order: Vec<u32>,
    ncomb: usize,
    // The dirty combinational cells, a bit each, and the lowest word that can have one
    dirty: Vec<u64>,
    low: usize,
    pending: Vec<u32>,
    is_pending: Vec<bool>,
    // Settling the combinational cells: a stateful cell they update waits in triggered
    in_comb: bool,
    triggered: Vec<u32>,
    is_triggered: Vec<bool>,
    updated: Vec<u32>,
    changed: Vec<u32>,
    is_changed: Vec<bool>,
    // Scratch words for results
    sv: Vec<u64>,
    sx: Vec<u64>,
    writes: Vec<word::Write>,
    pub evaluations: u64,
}

/// A cell of a block from Python
pub struct CellArgs {
    pub op: u8, // 0 functional, 1 gate, 2 constant
    // Each input: its bits as (slot, bit), NO for undefined, and if it updates the cell
    pub inputs: Vec<(Vec<(u32, u32)>, bool)>,
    pub outputs: Vec<u32>,
    pub rank: u32,
    pub word: Option<word::Cell>,
    pub gate: Option<GateCell>,
    pub old_clock: u8,
    pub value: u8,
}

impl Block {
    pub fn new(
        widths: Vec<u32>,
        cells: Vec<CellArgs>,
        pins: Vec<(u32, u32, u32)>,
        vcd: Vec<(u32, u32, u32)>,
    ) -> Block {
        let mut block = Block {
            c1: Vec::new(),
            ins: Vec::new(),
            gat: Vec::new(),
            start: Vec::new(),
            width: Vec::new(),
            v: Vec::new(),
            x: Vec::new(),
            segs: Vec::new(),
            literals: Vec::new(),
            cells: Vec::new(),
            fan_start: Vec::new(),
            fan: Vec::new(),
            pin_start: Vec::new(),
            pins: Vec::new(),
            vcd_start: Vec::new(),
            vcd: Vec::new(),
            order: Vec::new(),
            ncomb: 0,
            dirty: Vec::new(),
            low: 0,
            pending: Vec::new(),
            is_pending: Vec::new(),
            in_comb: false,
            triggered: Vec::new(),
            is_triggered: Vec::new(),
            updated: Vec::new(),
            changed: Vec::new(),
            is_changed: Vec::new(),
            sv: Vec::new(),
            sx: Vec::new(),
            writes: Vec::new(),
            evaluations: 0,
        };
        for width in widths {
            block.add_slot(width);
        }
        let slots = block.width.len();
        let mut readers: Vec<Vec<(u32, u32, u32)>> = vec![Vec::new(); slots];
        // The combinational cells by rank, then the stateful ones
        let mut order: Vec<usize> = (0..cells.len()).collect();
        order.sort_by_key(|&i| (cells[i].rank == NO, cells[i].rank));
        block.ncomb = cells.iter().filter(|c| c.rank != NO).count();
        let mut slots_of: Vec<Option<CellArgs>> = cells.into_iter().map(Some).collect();
        let sorted: Vec<CellArgs> = order.iter().map(|&i| slots_of[i].take().unwrap()).collect();
        block.order = order.iter().map(|&i| i as u32).collect();
        for (number, args) in sorted.into_iter().enumerate() {
            let first = block.ins.len() as u32;
            let mut gathered = false;
            for (bits, update) in &args.inputs {
                // A whole slot, in order, or bits gathered into an operand slot
                let whole = !bits.is_empty()
                    && bits[0].0 != NO
                    && bits[0].1 == 0
                    && block.width[bits[0].0 as usize] as usize == bits.len()
                    && bits
                        .iter()
                        .enumerate()
                        .all(|(i, b)| b.0 == bits[0].0 && b.1 == i as u32);
                let slot = if whole {
                    block.gat.push((0, 0));
                    bits[0].0
                } else {
                    let slot = block.add_slot(bits.len() as u32);
                    readers.push(Vec::new());
                    let seg_first = block.segs.len() as u32;
                    let mut index = 0;
                    while index < bits.len() {
                        let (s, b) = bits[index];
                        if s == NO {
                            // Undefined, like the operand slot starts
                            index += 1;
                            continue;
                        }
                        if s < 2 {
                            // GND or VDD: set once, by default_state()
                            block.literals.push((slot, index as u32, s as u8));
                            index += 1;
                            continue;
                        }
                        let mut len = 1;
                        while index + len < bits.len()
                            && bits[index + len].0 == s
                            && bits[index + len].1 == b + len as u32
                        {
                            len += 1;
                        }
                        block.segs.push(Seg {
                            dst: index as u32,
                            slot: s,
                            bit: b,
                            len: len as u32,
                        });
                        index += len;
                    }
                    let range = (seg_first, block.segs.len() as u32);
                    gathered |= range.0 != range.1;
                    block.gat.push(range);
                    slot
                };
                block.ins.push(slot);
                let stateful = args.rank == NO;
                // A stateful reader, and if the input updates it
                let flag = (stateful as u32) << 1 | (stateful && *update) as u32;
                // The bits it reads of each slot
                let mut sources: Vec<(u32, u32, u32)> = Vec::new();
                for &(source, bit) in bits.iter().filter(|b| b.0 != NO) {
                    match sources.iter_mut().find(|s| s.0 == source) {
                        Some(range) => {
                            range.1 = range.1.min(bit);
                            range.2 = range.2.max(bit + 1);
                        }
                        None => sources.push((source, bit, bit + 1)),
                    }
                }
                for (source, lo, hi) in sources {
                    readers[source as usize].push(((number as u32) << 2 | flag, lo, hi));
                }
            }
            let stateful = args.rank == NO;
            let q = if stateful {
                let width = block.width[args.outputs[0] as usize];
                let q = block.add_slot(width);
                readers.push(Vec::new());
                q
            } else {
                NO
            };
            let (flips, none) = match &args.word {
                Some(cell) if cell.kind == W::Decoder && cell.outputs[0].width <= 64 => {
                    let mut flips = HashMap::new();
                    let ok = cell.values.iter().all(|v| v.to_u64().is_some());
                    if ok {
                        for (value, flip) in &cell.flips {
                            if let (Some(v), Some(f)) = (value.to_u64(), flip.to_u64()) {
                                flips.insert(v, f);
                            }
                        }
                    }
                    (flips, cell.none.to_u64().unwrap_or(0))
                }
                _ => (HashMap::new(), 0),
            };
            let (a_signed, b_signed, b_signed_p, offset) = match &args.word {
                Some(word) => (
                    word.a_signed,
                    word.b_signed,
                    b_signed_param(word),
                    word.offset,
                ),
                None => (false, false, false, 0),
            };
            let op = match args.op {
                0 => Op::Word(args.word.as_ref().unwrap().kind),
                1 => Op::Gate(args.gate.as_ref().unwrap().kind),
                _ => Op::Const(args.value),
            };
            let count = block.ins.len() as u32 - first;
            let out = args.outputs[0];
            let edge_flop = match (&args.word, &args.gate) {
                (Some(word), _) => {
                    matches!(word.kind, W::Dff | W::Dffe | W::Sdff | W::Sdffe | W::Sdffce)
                        .then_some(word.clk_polarity)
                }
                (None, Some(gate)) => {
                    matches!(gate.kind, G::Dff | G::Dffe | G::Sdff | G::Sdffe | G::Sdffce)
                        .then_some(gate.clock_edge)
                }
                _ => None,
            };
            let only_clock = !args.inputs.is_empty()
                && args.inputs[0].1
                && args.inputs[1..].iter().all(|(_, update)| !update);
            let (clock_only, clock_level) = match edge_flop {
                Some(level) if only_clock && level < 2 => (true, level),
                _ => (false, 0),
            };
            let extra = args.word.map(|word| {
                Box::new(Extra {
                    word,
                    outputs: args.outputs,
                    flips,
                    none,
                })
            });
            let table = match (&args.gate, args.rank != NO) {
                (Some(gate), true) if args.inputs.len() <= 3 => gate_table_index(gate.kind),
                _ => NO_TABLE,
            };
            block.cells.push(BCell {
                table,
                op,
                first,
                count,
                gathered,
                out,
                stateful,
                clock_only,
                clock_level,
                q,
                old_clock: args.old_clock,
                gate: args.gate,
                extra,
                a_signed,
                b_signed,
                b_signed_param: b_signed_p,
                offset,
            });
        }
        // Readers of each slot, without duplicates
        block.fan_start.push(0);
        for mut list in readers {
            list.sort_unstable();
            list.dedup();
            // A cell that reads a slot both as an updating and a plain input updates
            let mut merged: Vec<(u32, u32, u32)> = Vec::new();
            for entry in list {
                match merged.last_mut() {
                    Some(last) if last.0 >> 2 == entry.0 >> 2 => {
                        last.0 |= entry.0 & 3;
                        last.1 = last.1.min(entry.1);
                        last.2 = last.2.max(entry.2);
                    }
                    _ => merged.push(entry),
                }
            }
            block.fan.extend_from_slice(&merged);
            block.fan_start.push(block.fan.len() as u32);
        }
        let total = block.width.len();
        let csr = |entries: Vec<(u32, u32, u32)>| {
            let mut lists: Vec<Vec<(u32, u32)>> = vec![Vec::new(); total];
            for (slot, bit, port) in entries {
                lists[slot as usize].push((bit, port));
            }
            let mut start = vec![0u32];
            let mut all = Vec::new();
            for list in lists {
                all.extend_from_slice(&list);
                start.push(all.len() as u32);
            }
            (start, all)
        };
        (block.pin_start, block.pins) = csr(pins);
        (block.vcd_start, block.vcd) = csr(vcd);
        block.dirty = vec![0; block.ncomb.div_ceil(64)];
        block.low = block.dirty.len();
        block.is_pending = vec![false; block.cells.len()];
        block.is_triggered = vec![false; block.cells.len()];
        block.is_changed = vec![false; total];
        block
    }

    fn add_slot(&mut self, width: u32) -> u32 {
        let slot = self.width.len() as u32;
        self.start.push(self.v.len() as u32);
        self.width.push(width);
        self.c1.push(if width == 0 { 0 } else { X });
        let count = words(width) as usize;
        // Undefined, like all ports after init()
        self.v.extend(std::iter::repeat_n(0, count));
        let m = mask(width % 64);
        for i in 0..count {
            let full = i + 1 < count || width.is_multiple_of(64);
            self.x.push(if width == 0 {
                0
            } else if full {
                u64::MAX
            } else {
                m
            });
        }
        slot
    }

    /// A bit of a slot as a value code: 0, 1 or X
    pub fn code(&self, slot: u32, bit: u32) -> u8 {
        if slot == NO {
            return X;
        }
        if bit == 0 {
            return self.c1[slot as usize];
        }
        let base = self.start[slot as usize] as usize;
        if get_bits(&self.x, base, bit, 1) != 0 {
            X
        } else {
            get_bits(&self.v, base, bit, 1) as u8
        }
    }

    /// The value of a slot as words: the bits, the undefined bits
    pub fn slot_words(&self, slot: u32) -> (Vec<u64>, Vec<u64>) {
        let base = self.start[slot as usize] as usize;
        let count = words(self.width[slot as usize]) as usize;
        (
            self.v[base..base + count].to_vec(),
            self.x[base..base + count].to_vec(),
        )
    }

    pub fn needs_settle(&self) -> bool {
        self.low < self.dirty.len() || !self.pending.is_empty() || !self.changed.is_empty()
    }

    /// An input bit from outside the block. Returns if the block needs a settle.
    pub fn set_input(&mut self, slot: u32, bit: u32, code: u8) -> bool {
        let base = self.start[slot as usize] as usize;
        let (v, x) = match code {
            0 => (0, 0),
            1 => (1, 0),
            _ => (0, 1),
        };
        if get_bits(&self.v, base, bit, 1) == v && get_bits(&self.x, base, bit, 1) == x {
            return false;
        }
        set_bits(&mut self.v, base, bit, 1, v);
        set_bits(&mut self.x, base, bit, 1, x);
        if bit == 0 {
            self.c1[slot as usize] = code.min(X);
        }
        self.fanout_bits(slot, bit, bit + 1);
        self.needs_settle()
    }

    /// The registers start at 0, in the next settle (after their delay), and the constants
    /// get their values now (GND and VDD have no delay)
    pub fn default_state(&mut self) {
        for i in 0..self.literals.len() {
            let (slot, bit, value) = self.literals[i];
            let base = self.start[slot as usize] as usize;
            set_bits(&mut self.v, base, bit, 1, value as u64);
            set_bits(&mut self.x, base, bit, 1, 0);
            if bit == 0 {
                self.c1[slot as usize] = value;
            }
        }
        for c in 0..self.cells.len() {
            if !self.cells[c].stateful {
                continue;
            }
            let (q, output) = (self.cells[c].q, self.cells[c].out);
            let base = self.start[q as usize] as usize;
            let count = words(self.width[q as usize]) as usize;
            self.v[base..base + count].fill(0);
            self.x[base..base + count].fill(0);
            self.c1[q as usize] = 0;
            match self.cells[c].op {
                Op::Const(value) => {
                    self.v[base] = value as u64;
                    self.c1[q as usize] = value;
                    if self.copy_slot(q, output) {
                        self.mark_changed(output);
                        self.fanout(output);
                    }
                }
                _ => {
                    if let Some(extra) = self.cells[c].extra.as_mut() {
                        extra.word.q = word::Word::default();
                    }
                    self.set_pending(c);
                }
            }
        }
    }

    fn set_pending(&mut self, c: usize) {
        if !self.is_pending[c] {
            self.is_pending[c] = true;
            self.pending.push(c as u32);
        }
    }

    #[inline]
    fn fanout(&mut self, slot: u32) {
        self.fanout_bits(slot, 0, u32::MAX);
    }

    /// The cells that read bits lo..hi of a slot: dirty, or a stateful one updates
    fn fanout_bits(&mut self, slot: u32, lo: u32, hi: u32) {
        let (start, end) = (
            self.fan_start[slot as usize] as usize,
            self.fan_start[slot as usize + 1] as usize,
        );
        for i in start..end {
            let (entry, first, after) = self.fan[i];
            if after <= lo || first >= hi {
                continue;
            }
            let c = (entry >> 2) as usize;
            if entry & 2 == 0 {
                let word = c >> 6;
                self.dirty[word] |= 1u64 << (c & 63);
                if word < self.low {
                    self.low = word;
                }
            } else if entry & 1 != 0 {
                if self.cells[c].clock_only {
                    // Only the active edge does something: the clock changed (or another
                    // bit of its slot, then the level is the same)
                    let clock = self.ins[self.cells[c].first as usize];
                    let old = self.c1[clock as usize];
                    self.gather(c);
                    let level = self.c1[clock as usize];
                    let (first, end) = self.gat[self.cells[c].first as usize];
                    if first != end && level == old {
                        continue;
                    }
                    if level != self.cells[c].clock_level {
                        continue;
                    }
                    let cell = &mut self.cells[c];
                    cell.old_clock = X;
                    if let Some(gate) = cell.gate.as_mut() {
                        gate.old_clock = X;
                    }
                }
                if self.in_comb {
                    // After the combinational cells settle
                    if !self.is_triggered[c] {
                        self.is_triggered[c] = true;
                        self.triggered.push(c as u32);
                    }
                    continue;
                }
                self.eval_stateful(c);
            }
        }
    }

    /// Settle the block: the new outputs of the registers, then the combinational cells
    /// in the order of their ranks. Returns the ports outside that change, (port, code),
    /// and the ports inside for the VCD file (if vcd).
    pub fn settle(&mut self, out: &mut Vec<(u32, u8)>, vcd: Option<&mut Vec<u32>>) {
        // Like the regions of Verilog: the combinational cells settle, the flip-flops whose
        // clock (or reset) a combinational cell changed then see the settled values, then
        // the new outputs of the flip-flops (like non-blocking assignments), again until
        // nothing changes. A circuit that doesn't stop (an oscillator) goes on in the next
        // settle, one ns later.
        for _round in 0..ROUNDS {
            self.in_comb = true;
            let mut word = self.low;
            while word < self.dirty.len() {
                let bits = self.dirty[word];
                if bits == 0 {
                    word += 1;
                    continue;
                }
                let bit = bits.trailing_zeros() as usize;
                self.dirty[word] = bits & (bits - 1);
                self.evaluations += 1;
                self.eval_comb(word << 6 | bit);
            }
            self.low = self.dirty.len();
            self.in_comb = false;
            let triggered = std::mem::take(&mut self.triggered);
            for &c in &triggered {
                self.is_triggered[c as usize] = false;
            }
            for &c in &triggered {
                self.eval_stateful(c as usize);
            }
            self.triggered = triggered;
            self.triggered.clear();
            if self.pending.is_empty() {
                break;
            }
            // All new outputs, then the cells they update: the flip-flops see the new
            // outputs of the others, and the combinational cells are dirty
            let pending = std::mem::take(&mut self.pending);
            let mut updated = std::mem::take(&mut self.updated);
            updated.clear();
            for &c in &pending {
                let c = c as usize;
                self.is_pending[c] = false;
                let (q, output) = (self.cells[c].q, self.cells[c].out);
                if self.copy_slot(q, output) {
                    self.mark_changed(output);
                    updated.push(output);
                }
            }
            for &slot in &updated {
                self.fanout(slot);
            }
            self.updated = updated;
            // pending has the flip-flops that the new outputs clocked, for the next round
            drop(pending);
        }
        let changed = std::mem::take(&mut self.changed);
        for &slot in &changed {
            let s = slot as usize;
            self.is_changed[s] = false;
            for i in self.pin_start[s] as usize..self.pin_start[s + 1] as usize {
                let (bit, port) = self.pins[i];
                out.push((port, self.code(slot, bit)));
            }
        }
        if let Some(vcd) = vcd {
            for &slot in &changed {
                let s = slot as usize;
                for i in self.vcd_start[s] as usize..self.vcd_start[s + 1] as usize {
                    vcd.push(self.vcd[i].1);
                }
            }
        }
        self.changed = changed;
        self.changed.clear();
    }

    fn mark_changed(&mut self, slot: u32) {
        if !self.is_changed[slot as usize] {
            self.is_changed[slot as usize] = true;
            self.changed.push(slot);
        }
    }

    fn copy_slot(&mut self, from: u32, to: u32) -> bool {
        let (a, b) = (
            self.start[from as usize] as usize,
            self.start[to as usize] as usize,
        );
        let count = words(self.width[to as usize]) as usize;
        let mut changed = false;
        for i in 0..count {
            if self.v[b + i] != self.v[a + i] || self.x[b + i] != self.x[a + i] {
                changed = true;
                self.v[b + i] = self.v[a + i];
                self.x[b + i] = self.x[a + i];
            }
        }
        self.c1[to as usize] = self.c1[from as usize];
        changed
    }

    /// Gather the bits of the inputs that aren't whole slots
    #[inline]
    fn gather(&mut self, c: usize) {
        if self.cells[c].gathered {
            self.gather_inputs(c);
        }
    }

    fn gather_inputs(&mut self, c: usize) {
        let (base, count) = (self.cells[c].first as usize, self.cells[c].count as usize);
        for i in base..base + count {
            let (first, end) = self.gat[i];
            if first == end {
                continue;
            }
            let dst = self.start[self.ins[i] as usize] as usize;
            for k in first..end {
                let seg = self.segs[k as usize];
                let mut done = 0;
                while done < seg.len {
                    let len = (seg.len - done).min(64);
                    let src = self.start[seg.slot as usize] as usize;
                    let (v, x) = (
                        get_bits(&self.v, src, seg.bit + done, len),
                        get_bits(&self.x, src, seg.bit + done, len),
                    );
                    set_bits(&mut self.v, dst, seg.dst + done, len, v);
                    set_bits(&mut self.x, dst, seg.dst + done, len, x);
                    done += len;
                }
            }
            let slot = self.ins[i] as usize;
            self.c1[slot] = if self.x[dst] & 1 != 0 {
                X
            } else {
                (self.v[dst] & 1) as u8
            };
        }
    }

    /// A slot of up to 64 bits: (bits, undefined bits, width)
    #[inline]
    fn small(&self, slot: u32) -> (u64, u64, u32) {
        let base = self.start[slot as usize] as usize;
        (self.v[base], self.x[base], self.width[slot as usize])
    }

    /// Write a value of up to 64 bits to a slot. Returns if it changed.
    #[inline]
    fn write_small(&mut self, slot: u32, v: u64, x: u64) -> bool {
        let base = self.start[slot as usize] as usize;
        let m = mask(self.width[slot as usize]);
        let x = x & m;
        let v = v & m & !x;
        if self.v[base] == v && self.x[base] == x {
            return false;
        }
        self.v[base] = v;
        self.x[base] = x;
        self.c1[slot as usize] = if x & 1 != 0 { X } else { (v & 1) as u8 };
        true
    }

    /// Write the scratch words to a slot. Returns if it changed.
    fn write_scratch(&mut self, slot: u32) -> bool {
        let base = self.start[slot as usize] as usize;
        let width = self.width[slot as usize];
        let count = words(width) as usize;
        let mut changed = false;
        for i in 0..count {
            let m = if i + 1 == count {
                mask(width - 64 * i as u32)
            } else {
                u64::MAX
            };
            let x = self.sx[i] & m;
            let v = self.sv[i] & m & !x;
            if self.v[base + i] != v || self.x[base + i] != x {
                changed = true;
                self.v[base + i] = v;
                self.x[base + i] = x;
            }
        }
        if changed {
            self.c1[slot as usize] = if self.x[base] & 1 != 0 {
                X
            } else {
                (self.v[base] & 1) as u8
            };
        }
        changed
    }

    fn eval_comb(&mut self, c: usize) {
        self.gather(c);
        let table = self.cells[c].table;
        if table != NO_TABLE {
            // A gate: a look-up of the codes of its inputs
            let cell = &self.cells[c];
            let (first, count, output) = (cell.first as usize, cell.count as usize, cell.out);
            let code = |i: usize| self.c1[self.ins[first + i] as usize] as usize;
            let a = code(0);
            let b = if count > 1 { code(1) } else { 0 };
            let s = if count > 2 { code(2) } else { 0 };
            let value = gate_table()[table as usize + (a << 4 | b << 2 | s)];
            let o = output as usize;
            if self.c1[o] == value {
                return;
            }
            self.c1[o] = value;
            let base = self.start[o] as usize;
            self.v[base] = (value == 1) as u64;
            self.x[base] = (value > 1) as u64;
            self.mark_changed(output);
            self.fanout(output);
            return;
        }
        let output = self.cells[c].out;
        let changed = match self.cells[c].op {
            Op::Gate(_) => {
                let code = self.eval_gate_cell(c);
                match code {
                    Some(code) => {
                        let (v, x) = match code {
                            0 => (0, 0),
                            1 => (1, 0),
                            _ => (0, 1),
                        };
                        self.write_small(output, v, x)
                    }
                    None => false,
                }
            }
            Op::Word(kind) => match self.fast(c, kind) {
                Some(changed) => changed,
                None => return self.slow(c),
            },
            Op::Const(_) => false,
        };
        if changed {
            self.mark_changed(output);
            self.fanout(output);
        }
    }

    #[inline]
    fn code0(&self, slot: u32) -> u8 {
        self.c1[slot as usize]
    }

    #[inline]
    fn eval_gate_cell(&mut self, c: usize) -> Option<u8> {
        let mut codes = [X; 20];
        let (first, count) = (self.cells[c].first as usize, self.cells[c].count as usize);
        for (i, code) in codes.iter_mut().enumerate().take(count) {
            *code = self.code0(self.ins[first + i]);
        }
        let gate = self.cells[c].gate.as_mut().unwrap();
        eval_gate(gate, |i| if i < count { codes[i] } else { X })
    }

    /// A stateful cell with a changed updating input: its next output, in the next settle
    fn eval_stateful(&mut self, c: usize) {
        self.evaluations += 1;
        self.gather(c);
        match self.cells[c].op {
            Op::Gate(_) => {
                if let Some(code) = self.eval_gate_cell(c) {
                    let q = self.cells[c].q;
                    let (v, x) = match code {
                        0 => (0, 0),
                        1 => (1, 0),
                        _ => (0, 1),
                    };
                    self.write_small(q, v, x);
                    self.set_pending(c);
                }
            }
            Op::Word(kind) => {
                if !self.fast_register(c, kind) {
                    self.slow(c);
                }
            }
            Op::Const(_) => {}
        }
    }

    /// The registers computed here: returns false for the others
    fn fast_register(&mut self, c: usize, kind: W) -> bool {
        let cell = &self.cells[c];
        let word = &cell.extra.as_ref().unwrap().word;
        let (clk_pol, en_pol, arst_pol, srst_pol) = (
            word.clk_polarity,
            word.en_polarity,
            word.arst_polarity,
            word.srst_polarity,
        );
        let mut inputs = [NO; 8];
        let (first, count) = (cell.first as usize, (cell.count as usize).min(8));
        inputs[..count].copy_from_slice(&self.ins[first..first + count]);
        let bit = |block: &Block, i: usize| block.code0(inputs[i]);
        // (D input, or the reset value: None for the srst one, Some(false) arst)
        enum Next {
            Keep,
            D(usize),
            Srst,
            Arst,
        }
        let edge = |block: &mut Block| {
            let clock = bit(block, 0);
            let cell = &mut block.cells[c];
            let edge = clock != cell.old_clock && clock == clk_pol;
            cell.old_clock = clock;
            edge
        };
        let next = match kind {
            W::Dff => {
                if edge(self) {
                    Next::D(1)
                } else {
                    Next::Keep
                }
            }
            W::Dffe => {
                if edge(self) && bit(self, 1) == en_pol {
                    Next::D(2)
                } else {
                    Next::Keep
                }
            }
            W::Adff | W::Adffe => {
                let e = edge(self);
                let enable = kind == W::Adffe;
                if bit(self, 1) == arst_pol {
                    Next::Arst
                } else if e && (!enable || bit(self, 2) == en_pol) {
                    Next::D(if enable { 3 } else { 2 })
                } else {
                    Next::Keep
                }
            }
            W::Sdff => {
                if edge(self) {
                    if bit(self, 1) == srst_pol {
                        Next::Srst
                    } else {
                        Next::D(2)
                    }
                } else {
                    Next::Keep
                }
            }
            W::Sdffe => {
                if !edge(self) {
                    Next::Keep
                } else if bit(self, 1) == srst_pol {
                    Next::Srst
                } else if bit(self, 2) == en_pol {
                    Next::D(3)
                } else {
                    Next::Keep
                }
            }
            W::Sdffce => {
                if !edge(self) || bit(self, 2) != en_pol {
                    Next::Keep
                } else if bit(self, 1) == srst_pol {
                    Next::Srst
                } else {
                    Next::D(3)
                }
            }
            W::Dlatch => {
                if bit(self, 0) == en_pol {
                    Next::D(1)
                } else {
                    Next::Keep
                }
            }
            _ => return false,
        };
        let q = self.cells[c].q;
        let count = words(self.width[q as usize]) as usize;
        self.sv.clear();
        self.sx.clear();
        match next {
            Next::Keep => return true,
            Next::D(input) => {
                let slot = inputs[input];
                let base = self.start[slot as usize] as usize;
                let have = words(self.width[slot as usize]) as usize;
                for i in 0..count {
                    self.sv.push(if i < have { self.v[base + i] } else { 0 });
                    self.sx.push(if i < have { self.x[base + i] } else { 0 });
                }
            }
            Next::Srst | Next::Arst => {
                let word = &self.cells[c].extra.as_ref().unwrap().word;
                let value = if matches!(next, Next::Srst) {
                    &word.srst_value
                } else {
                    &word.arst_value
                };
                let value = value.v.to_words();
                for i in 0..count {
                    self.sv.push(value.get(i).copied().unwrap_or(0));
                    self.sx.push(0);
                }
            }
        }
        // Python's set_q() writes Q even if it is the same
        self.write_scratch(q);
        self.set_pending(c);
        true
    }

    /// A functional cell computed by word.rs, with its inputs from the slots
    fn slow(&mut self, c: usize) {
        let (first, count) = (self.cells[c].first as usize, self.cells[c].count as usize);
        let inputs = self.ins[first..first + count].to_vec();
        for (i, &slot) in inputs.iter().enumerate() {
            let (v, x) = self.slot_words(slot);
            let word = &mut self.cells[c].extra.as_mut().unwrap().word;
            word.inputs[i].word = word::Word {
                v: Bits::from_words(&v),
                u: Bits::from_words(&x),
            };
        }
        let mut writes = std::mem::take(&mut self.writes);
        writes.clear();
        let word = &mut self.cells[c].extra.as_mut().unwrap().word;
        word.update(&mut writes);
        self.writes = writes;
        let stateful = self.cells[c].stateful;
        let count = self.cells[c].extra.as_ref().unwrap().outputs.len();
        for k in 0..count {
            let word = &self.cells[c].extra.as_ref().unwrap().word;
            let Some(last) = word.outputs[k].last.as_ref() else {
                continue;
            };
            let (v, x) = (last.v.to_words(), last.u.to_words());
            let slot = if stateful {
                self.cells[c].q
            } else {
                self.cells[c].extra.as_ref().unwrap().outputs[k]
            };
            let n = words(self.width[slot as usize]) as usize;
            self.sv.clear();
            self.sx.clear();
            for i in 0..n {
                self.sv.push(v.get(i).copied().unwrap_or(0));
                self.sx.push(x.get(i).copied().unwrap_or(0));
            }
            let changed = self.write_scratch(slot);
            if stateful {
                if changed {
                    self.set_pending(c);
                }
            } else if changed {
                self.mark_changed(slot);
                self.fanout(slot);
            }
        }
    }

    /// The combinational cells computed here. Returns None for the others (word.rs),
    /// otherwise if the output changed.
    fn fast(&mut self, c: usize, kind: W) -> Option<bool> {
        let cell = &self.cells[c];
        let output = cell.out;
        let yw = self.width[output as usize];
        let (a_signed, b_signed) = (cell.a_signed, cell.b_signed);
        let b_signed_p = cell.b_signed_param;
        let offset = cell.offset;
        let input_count = cell.count as usize;
        if input_count > 4 {
            return None;
        }
        let mut inputs = [NO; 4];
        let first = cell.first as usize;
        inputs[..input_count].copy_from_slice(&self.ins[first..first + input_count]);
        let wide = |block: &Block, slot: u32| block.width[slot as usize] > 64;
        let ym = mask(yw);
        // Bitwise cells and multiplexers of any width, with equal widths
        match kind {
            W::And | W::Or | W::Xor | W::Xnor if yw > 64 => {
                let (a, b) = (inputs[0], inputs[1]);
                if self.width[a as usize] != yw || self.width[b as usize] != yw {
                    return None;
                }
                let (pa, pb) = (
                    self.start[a as usize] as usize,
                    self.start[b as usize] as usize,
                );
                let count = words(yw) as usize;
                self.sv.clear();
                self.sx.clear();
                for i in 0..count {
                    let (av, ax, bv, bx) = (
                        self.v[pa + i],
                        self.x[pa + i],
                        self.v[pb + i],
                        self.x[pb + i],
                    );
                    let (v, x) = bitwise(kind, av, ax, bv, bx, u64::MAX);
                    self.sv.push(v);
                    self.sx.push(x);
                }
                return Some(self.write_scratch(output));
            }
            W::Not if yw > 64 => {
                let a = inputs[0];
                if self.width[a as usize] != yw {
                    return None;
                }
                let pa = self.start[a as usize] as usize;
                let count = words(yw) as usize;
                self.sv.clear();
                self.sx.clear();
                for i in 0..count {
                    let (av, ax) = (self.v[pa + i], self.x[pa + i]);
                    self.sv.push(!av & !ax);
                    self.sx.push(ax);
                }
                return Some(self.write_scratch(output));
            }
            W::Mux if yw > 64 => {
                let (a, b, s) = (inputs[0], inputs[1], inputs[2]);
                let (_, sx, _) = self.small(s);
                if sx != 0 {
                    return None;
                }
                let sv = self.small(s).0;
                let source = if sv & 1 != 0 { b } else { a };
                if self.width[source as usize] != yw {
                    return None;
                }
                let base = self.start[source as usize] as usize;
                let count = words(yw) as usize;
                self.sv.clear();
                self.sx.clear();
                for i in 0..count {
                    self.sv.push(self.v[base + i]);
                    self.sx.push(self.x[base + i]);
                }
                return Some(self.write_scratch(output));
            }
            W::Shift if yw > 64 => {
                return self.wide_shift(c, a_signed);
            }
            W::Concat | W::Slice => {
                let a = inputs[0];
                let aw = self.width[a as usize];
                let pa = self.start[a as usize] as usize;
                let count = words(yw) as usize;
                self.sv.clear();
                self.sx.clear();
                self.sv.resize(count, 0);
                self.sx.resize(count, 0);
                let copy = |block: &mut Block, base: usize, from: u32, to: u32, len: u32| {
                    let mut done = 0;
                    while done < len {
                        let n = (len - done).min(64);
                        let v = get_bits(&block.v, base, from + done, n);
                        let x = get_bits(&block.x, base, from + done, n);
                        set_bits(&mut block.sv, 0, to + done, n, v);
                        set_bits(&mut block.sx, 0, to + done, n, x);
                        done += n;
                    }
                };
                if kind == W::Concat {
                    let b = inputs[1];
                    let bw = self.width[b as usize];
                    let pb = self.start[b as usize] as usize;
                    copy(self, pa, 0, 0, aw.min(yw));
                    if aw < yw {
                        copy(self, pb, 0, aw, bw.min(yw - aw));
                    }
                } else if offset < aw as usize {
                    let offset = offset as u32;
                    copy(self, pa, offset, 0, (aw - offset).min(yw));
                }
                return Some(self.write_scratch(output));
            }
            W::Bmux if yw <= 64 => {
                let (a, s) = (inputs[0], inputs[1]);
                if wide(self, s) {
                    return None;
                }
                let (sv, sx, _) = self.small(s);
                if sx != 0 {
                    return Some(self.write_small(output, 0, ym));
                }
                let aw = self.width[a as usize] as u64;
                let shift = (sv as u128) * (yw as u128);
                let (v, x) = if shift >= aw as u128 {
                    (0, 0)
                } else {
                    let shift = shift as u32;
                    let len = (aw as u32 - shift).min(yw);
                    let base = self.start[a as usize] as usize;
                    (
                        get_bits(&self.v, base, shift, len),
                        get_bits(&self.x, base, shift, len),
                    )
                };
                return Some(self.write_small(output, v, x));
            }
            W::Pmux if yw <= 64 => {
                let (a, b, s) = (inputs[0], inputs[1], inputs[2]);
                if wide(self, s) || wide(self, a) {
                    return None;
                }
                let (sv, sx, _) = self.small(s);
                if sx != 0 || sv & sv.wrapping_sub(1) != 0 {
                    return Some(self.write_small(output, 0, ym));
                }
                if sv == 0 {
                    let (av, ax, _) = self.small(a);
                    return Some(self.write_small(output, av, ax));
                }
                let shift = sv.trailing_zeros() * yw;
                let base = self.start[b as usize] as usize;
                let bw = self.width[b as usize];
                let len = bw.saturating_sub(shift).min(yw);
                let (v, x) = (
                    get_bits(&self.v, base, shift, len),
                    get_bits(&self.x, base, shift, len),
                );
                return Some(self.write_small(output, v, x));
            }
            _ => {}
        }
        if yw > 64 || inputs[..input_count].iter().any(|&s| wide(self, s)) {
            return None;
        }
        let (av, ax, aw) = self.small(inputs[0]);
        let (bv, bx, bw) = if input_count > 1 {
            self.small(inputs[1])
        } else {
            (0, 0, 0)
        };
        let (v, x): (u64, u64) = match kind {
            W::Not | W::Pos | W::Neg => {
                let (v, x) = extend(av, ax, aw, a_signed, yw);
                match kind {
                    W::Not => (!v & !x & ym, x),
                    W::Pos => (v, x),
                    _ => {
                        if x != 0 {
                            (0, ym)
                        } else {
                            (v.wrapping_neg() & ym, 0)
                        }
                    }
                }
            }
            W::And | W::Or | W::Xor | W::Xnor => {
                let signed = a_signed && b_signed;
                let (av, ax) = extend(av, ax, aw, signed, yw);
                let (bv, bx) = extend(bv, bx, bw, signed, yw);
                bitwise(kind, av, ax, bv, bx, ym)
            }
            W::ReduceAnd | W::ReduceOr | W::ReduceXor | W::ReduceXnor | W::LogicNot => {
                let am = mask(aw);
                let r = match kind {
                    W::ReduceAnd => {
                        if !(av | ax) & am != 0 {
                            Some(0)
                        } else if ax != 0 {
                            None
                        } else {
                            Some(1)
                        }
                    }
                    W::ReduceOr => reduce_or(av, ax),
                    W::ReduceXor | W::ReduceXnor => {
                        if ax != 0 {
                            None
                        } else {
                            let parity = (av.count_ones() & 1) as u64;
                            Some(if kind == W::ReduceXor {
                                parity
                            } else {
                                1 - parity
                            })
                        }
                    }
                    _ => reduce_or(av, ax).map(|r| 1 - r),
                };
                match r {
                    None => (0, 1),
                    Some(r) => (r, 0),
                }
            }
            W::LogicAnd | W::LogicOr => {
                let (a, b) = (reduce_or(av, ax), reduce_or(bv, bx));
                let r = if kind == W::LogicAnd {
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
                match r {
                    None => (0, 1),
                    Some(r) => (r, 0),
                }
            }
            W::Lt | W::Le | W::Gt | W::Ge => {
                if ax != 0 || bx != 0 {
                    (0, 1)
                } else {
                    let signed = a_signed && b_signed;
                    let (a, b) = (number(av, aw, signed), number(bv, bw, signed));
                    let r = match kind {
                        W::Lt => a < b,
                        W::Le => a <= b,
                        W::Gt => a > b,
                        _ => a >= b,
                    };
                    (r as u64, 0)
                }
            }
            W::Eq | W::Ne => {
                let equal = (kind == W::Eq) as u64;
                if ax == 0 && bx == 0 && aw == bw {
                    (if av == bv { equal } else { 1 - equal }, 0)
                } else {
                    let width = aw.max(bw).max(yw);
                    if width > 64 {
                        return None;
                    }
                    let signed = a_signed && b_signed;
                    let (av, ax) = extend(av, ax, aw, signed, width);
                    let (bv, bx) = extend(bv, bx, bw, signed, width);
                    let m = mask(width);
                    let (a1, a0) = (av & !ax & m, !(av | ax) & m);
                    let (b1, b0) = (bv & !bx & m, !(bv | bx) & m);
                    if (a1 & b0) | (a0 & b1) != 0 {
                        (1 - equal, 0)
                    } else if ax | bx != 0 {
                        (0, 1)
                    } else {
                        (equal, 0)
                    }
                }
            }
            W::Eqx | W::Nex => {
                let width = aw.max(bw).max(yw);
                if width > 64 {
                    return None;
                }
                let signed = a_signed && b_signed;
                let a = extend(av, ax, aw, signed, width);
                let b = extend(bv, bx, bw, signed, width);
                let equal = (a == b) as u64;
                (if kind == W::Eqx { equal } else { 1 - equal }, 0)
            }
            W::Add | W::Sub | W::Mul => {
                if ax != 0 || bx != 0 {
                    (0, ym)
                } else {
                    let signed = a_signed && b_signed;
                    let (a, b) = (number(av, aw, signed) as u64, number(bv, bw, signed) as u64);
                    let r = match kind {
                        W::Add => a.wrapping_add(b),
                        W::Sub => a.wrapping_sub(b),
                        _ => a.wrapping_mul(b),
                    };
                    (r & ym, 0)
                }
            }
            W::Shl | W::Shr | W::Sshr | W::Shift => {
                if bx != 0 {
                    (0, ym)
                } else {
                    let width = aw.max(yw);
                    let (v, x) = extend(av, ax, aw, a_signed, width);
                    let amount: i128 = if kind == W::Shift && b_signed_p {
                        number(bv, bw, true)
                    } else {
                        bv as i128
                    };
                    let left = |amount: i128| -> (u64, u64) {
                        if amount >= width as i128 {
                            (0, 0)
                        } else {
                            (v << amount, x << amount)
                        }
                    };
                    let right = |amount: i128, fill: (u64, u64)| -> (u64, u64) {
                        let amount = amount.min(width as i128) as u32;
                        let fill_mask = mask(width) & !mask(width - amount);
                        let shift = |w: u64| if amount >= 64 { 0 } else { w >> amount };
                        (
                            shift(v) | if fill.0 != 0 { fill_mask } else { 0 },
                            shift(x) | if fill.1 != 0 { fill_mask } else { 0 },
                        )
                    };
                    let (v, x) = match kind {
                        W::Shl => left(amount),
                        W::Shr => right(amount, (0, 0)),
                        W::Sshr => {
                            let sign = if a_signed && width > 0 {
                                ((v >> (width - 1)) & 1, (x >> (width - 1)) & 1)
                            } else {
                                (0, 0)
                            };
                            right(amount, sign)
                        }
                        _ => {
                            if amount >= 0 {
                                right(amount, (0, 0))
                            } else {
                                left(-amount)
                            }
                        }
                    };
                    (v & ym, x & ym)
                }
            }
            W::Mux => {
                let (sv, sx, _) = self.small(inputs[2]);
                if sx & 1 == 0 {
                    if sv & 1 != 0 {
                        (bv, bx)
                    } else {
                        (av, ax)
                    }
                } else {
                    let same = !(av ^ bv) & !ax & !bx & ym;
                    (av & same, ym & !same)
                }
            }
            W::Bwmux => {
                let (sv, sx, _) = self.small(inputs[2]);
                let value = (av & !sv) | (bv & sv);
                let undefined = (ax & !sv) | (bx & sv);
                let same = !(av ^ bv) & !ax & !bx;
                let undefined = (undefined & !sx) | (sx & !same);
                (value & !undefined & ym, undefined & ym)
            }
            W::Bweqx => {
                let differ = (av ^ bv) | (ax ^ bx);
                (!differ & ym, 0)
            }
            W::Decoder => {
                let extra = self.cells[c].extra.as_ref().unwrap();
                if ax != 0 || yw > 64 || extra.word.values.len() as u32 != yw {
                    return None;
                }
                if extra.flips.is_empty() && !extra.word.values.is_empty() {
                    return None;
                }
                (extra.none ^ extra.flips.get(&av).copied().unwrap_or(0), 0)
            }
            _ => return None,
        };
        Some(self.write_small(output, v, x))
    }

    /// $shift with an output wider than 64 bits (like a write enable into a register file)
    fn wide_shift(&mut self, c: usize, a_signed: bool) -> Option<bool> {
        let cell = &self.cells[c];
        let first = cell.first as usize;
        let (a, b, output) = (self.ins[first], self.ins[first + 1], cell.out);
        let b_signed = cell.b_signed_param;
        let (aw, bw, yw) = (
            self.width[a as usize],
            self.width[b as usize],
            self.width[output as usize],
        );
        if aw > 64 || bw > 64 {
            return None;
        }
        let (av, ax, _) = self.small(a);
        let (bv, bx, _) = self.small(b);
        let count = words(yw) as usize;
        self.sv.clear();
        self.sx.clear();
        self.sv.resize(count, 0);
        self.sx.resize(count, 0);
        if bx != 0 {
            self.sx.fill(u64::MAX);
            return Some(self.write_scratch(output));
        }
        if a_signed && aw < yw && ((av | ax) >> (aw - 1)) & 1 != 0 {
            // A sign extended above 64 bits, not here
            return None;
        }
        let amount: i128 = if b_signed {
            number(bv, bw, true)
        } else {
            bv as i128
        };
        // A (up to 64 bits, zero extended) moved by amount: right if positive
        let width = aw.max(yw) as i128;
        let position = -amount;
        if position < width && position > -64 {
            let place = |array_v: u64, array_x: u64, block: &mut Block| {
                if position >= 0 {
                    let p = position as u32;
                    let len = (yw - p.min(yw)).min(64);
                    set_bits(&mut block.sv, 0, p, len, array_v);
                    set_bits(&mut block.sx, 0, p, len, array_x);
                } else {
                    let s = (-position) as u32;
                    let len = (64 - s).min(yw);
                    set_bits(&mut block.sv, 0, 0, len, array_v >> s);
                    set_bits(&mut block.sx, 0, 0, len, array_x >> s);
                }
            };
            place(av, ax, self);
        }
        Some(self.write_scratch(output))
    }

    /// The state of the cells for going back to Python: for each cell, Q, the clock level,
    /// if it waits for a settle (dirty or pending)
    pub fn export_cells(&self) -> Vec<CellState> {
        let mut number = vec![0usize; self.cells.len()];
        for (c, &python) in self.order.iter().enumerate() {
            number[python as usize] = c;
        }
        number
            .into_iter()
            .map(|c| {
                let cell = &self.cells[c];
                let q = cell.stateful.then(|| self.slot_words(cell.q));
                let clock = if cell.clock_only {
                    self.c1[self.ins[cell.first as usize] as usize]
                } else {
                    match (&cell.op, &cell.extra, &cell.gate) {
                        (Op::Word(kind), Some(extra), _) => {
                            if is_fast_register(*kind) {
                                cell.old_clock
                            } else {
                                extra.word.old_clock
                            }
                        }
                        (_, _, Some(gate)) => gate.old_clock,
                        _ => cell.old_clock,
                    }
                };
                let dirty = c < self.ncomb && self.dirty[c >> 6] & (1u64 << (c & 63)) != 0;
                (q, clock, dirty || self.is_pending[c])
            })
            .collect()
    }
}

const NO_TABLE: u16 = u16::MAX;

/// The rounds of combinational cells and flip-flops in one settle, before the next one
const ROUNDS: usize = 64;

/// The part of GATE_TABLE of a combinational gate kind with up to three inputs
fn gate_table_index(kind: G) -> u16 {
    match gate_kinds().iter().position(|&k| k == kind) {
        Some(index) => (index * 64) as u16,
        None => NO_TABLE,
    }
}

fn gate_kinds() -> &'static [G] {
    use G::*;
    &[
        Buf, Not, And, Nand, Or, Nor, Xor, Xnor, AndNot, OrNot, Mux, NMux, Aoi3, Oai3,
    ]
}

/// The outputs of the gates for all codes of their inputs (a << 4 | b << 2 | c), from
/// eval_gate() in lib.rs
fn gate_table() -> &'static [u8] {
    static TABLE: std::sync::OnceLock<Vec<u8>> = std::sync::OnceLock::new();
    TABLE.get_or_init(|| {
        let mut table = Vec::new();
        for &kind in gate_kinds() {
            for index in 0..64usize {
                let codes = [
                    (index >> 4) as u8 & 3,
                    (index >> 2) as u8 & 3,
                    index as u8 & 3,
                ];
                let mut cell = GateCell {
                    kind,
                    first: 0,
                    output: 0,
                    clock_edge: 0,
                    reset_level: 0,
                    reset_value: 0,
                    enable_level: 0,
                    set_level: 0,
                    load_level: 0,
                    old_clock: X,
                };
                let value = eval_gate(&mut cell, |i| codes.get(i).copied().unwrap_or(X));
                // In a block a value is 0, 1 or X
                table.push(value.unwrap_or(X).min(X));
            }
        }
        table
    })
}

fn is_fast_register(kind: W) -> bool {
    matches!(
        kind,
        W::Dff | W::Dffe | W::Adff | W::Adffe | W::Sdff | W::Sdffe | W::Sdffce | W::Dlatch
    )
}

fn b_signed_param(word: &word::Cell) -> bool {
    word.params
        .get("B_SIGNED")
        .is_some_and(|value| *value != num_bigint::BigInt::from(0))
}

#[inline]
fn bitwise(kind: W, av: u64, ax: u64, bv: u64, bx: u64, m: u64) -> (u64, u64) {
    match kind {
        W::And | W::Or => {
            let (a1, a0) = (av & !ax & m, !(av | ax) & m);
            let (b1, b0) = (bv & !bx & m, !(bv | bx) & m);
            if kind == W::And {
                let ones = a1 & b1;
                (ones, m & !(ones | a0 | b0))
            } else {
                let ones = a1 | b1;
                (ones, m & !(ones | (a0 & b0)))
            }
        }
        _ => {
            let x = (ax | bx) & m;
            let d = av ^ bv;
            if kind == W::Xor {
                (d & !x & m, x)
            } else {
                (!d & !x & m, x)
            }
        }
    }
}

#[inline]
fn reduce_or(v: u64, x: u64) -> Option<u64> {
    if v & !x != 0 {
        Some(1)
    } else if x != 0 {
        None
    } else {
        Some(0)
    }
}

/// A value of 'width' bits as a number, two's complement if signed
#[inline]
fn number(v: u64, width: u32, signed: bool) -> i128 {
    if signed && width > 0 && (v >> (width - 1)) & 1 != 0 {
        v as i128 - (1i128 << width)
    } else {
        v as i128
    }
}

/// _extend(): a value extended (or cut) to new_width bits, with its sign bit if signed
#[inline]
fn extend(v: u64, x: u64, width: u32, signed: bool, new_width: u32) -> (u64, u64) {
    if new_width == width {
        return (v, x);
    }
    let (mut v, mut x) = (v, x);
    if new_width > width && signed && width > 0 {
        let fill = mask(new_width) & !mask(width);
        if (x >> (width - 1)) & 1 != 0 {
            x |= fill;
        } else if (v >> (width - 1)) & 1 != 0 {
            v |= fill;
        }
    }
    (v & mask(new_width), x & mask(new_width))
}

#[cfg(test)]
mod bench {
    use super::*;
    use std::collections::HashMap;

    fn word_cell(kind: W, widths: &[usize], out: usize) -> word::Cell {
        let inputs = widths
            .iter()
            .map(|&w| word::Input {
                word: word::Word::default(),
                width: w,
                update: true,
            })
            .collect();
        let outputs = vec![word::Output {
            mask: word::mask(out),
            width: out,
            last: None,
            bit_ports: vec![],
            bit_outputs: Bits::zero(),
            word_port: None,
            links: vec![],
        }];
        word::Cell::new(
            kind,
            inputs,
            outputs,
            HashMap::new(),
            word::Word::default(),
            X,
            true,
        )
    }

    /// cargo test --release chain_speed -- --ignored --nocapture
    #[test]
    #[ignore]
    fn chain_speed() {
        // input slot 2 (32 bits), a chain of n cells: Add(prev, input)
        let n = 2000;
        let mut widths = vec![1, 1, 32];
        let mut cells = Vec::new();
        for i in 0..n {
            let prev = if i == 0 { 2 } else { 3 + i - 1 };
            widths.push(32);
            let bits = |s: u32| (0..32).map(|b| (s, b)).collect::<Vec<_>>();
            cells.push(CellArgs {
                op: 0,
                inputs: vec![(bits(prev as u32), true), (bits(2), true)],
                outputs: vec![3 + i as u32],
                rank: i as u32,
                word: Some(word_cell(W::Add, &[32, 32], 32)),
                gate: None,
                old_clock: X,
                value: 0,
            });
        }
        let mut block = Block::new(widths, cells, vec![], vec![]);
        let mut out = Vec::new();
        let start = std::time::Instant::now();
        let rounds = 2000;
        for r in 0..rounds {
            block.set_input(2, 0, (r & 1) as u8);
            block.settle(&mut out, None);
        }
        let ns = start.elapsed().as_nanos() as f64 / (rounds * n) as f64;
        eprintln!(
            "Add chain: {ns:.1} ns per cell, {} evaluations",
            block.evaluations
        );
    }
}
