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
    op: u32,
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

#[derive(Clone)]
struct BCell {
    op: Op,
    // The operand slots of the inputs: Block.ins[first..first + count], and their bits to
    // gather (Block.gat, a range of segs, empty for an input that is a whole slot)
    first: u32,
    count: u32,
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
    // A combinational functional cell of up to 64 bits: its Small, NO for the others
    small: u32,
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
#[derive(Clone)]
struct Extra {
    word: word::Cell,
    outputs: Vec<u32>,
    flips: HashMap<u64, u64>,
    none: u64,
}

/// The state of a cell for Python: Q (bits, undefined bits), the clock level, and if it
/// waits for a settle
pub type CellState = (Option<(Vec<u64>, Vec<u64>)>, u8, bool);

#[derive(Clone)]
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
    // (entry, the first bit it reads, the bit after the last one, the bits it reads of a
    // slot of up to 64 bits; 0 for a wider one, then it reads all bits first..after)
    fan: Vec<(u32, u32, u32, u64)>,
    // Where the readers of one bit of each slot start in fan
    fan_narrow: Vec<u32>,
    // The segs that copy bits of each slot into operands: scat[scat_start[s]..]
    scat_start: Vec<u32>,
    scat: Vec<u32>,
    scat_narrow: Vec<u32>,
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
    updated: Vec<(u32, u64)>,
    wide_diffs: Vec<Vec<u64>>,
    changed: Vec<u32>,
    is_changed: Vec<bool>,
    // A slot has pins (1), and output ports inside for the VCD file (2)
    watch: Vec<u8>,
    // A slot updates a stateful cell: a clock, reset or latch input
    stateful_readers: Vec<bool>,
    vcd_on: bool,
    // Scratch words for results
    sv: Vec<u64>,
    sx: Vec<u64>,
    // The bits that the last write of a slot changed, by word (see fanout_changed())
    diff: Vec<u64>,
    writes: Vec<word::Write>,
    pub evaluations: u64,
    // With the feature prof: the block after default_state() and the calls since, to
    // time them again without Python (see replay())
    #[cfg(feature = "prof")]
    record: Option<(Box<Block>, Vec<Call>)>,
    // A block of gates only: the arrays of the sweep (see prepare_sweep())
    smalls: Vec<Small>,
    sweep: bool,
    sweep_gates: Vec<SweepGate>,
    // The gates (combinational) that read each slot, for eval_sweep_gate()
    gate_readers_start: Vec<u32>,
    gate_readers: Vec<u32>,
    // The groups of gate flip-flops on each clock slot: groups[group_start[s]..]
    group_start: Vec<u32>,
    groups: Vec<EdgeGroup>,
    edge_flops: Vec<EdgeFlop>,
    triggered_groups: Vec<u32>,
    notify: Vec<u8>,
    // The gates the last settle evaluated without a sweep
    last_evaluations: u64,
}

/// A call of a block, for replay()
#[cfg(feature = "prof")]
#[derive(Clone, Copy)]
pub enum Call {
    Input(u32, u32, u8),
    Word(u32, u64, u64),
    Settle,
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
            fan_narrow: Vec::new(),
            scat_narrow: Vec::new(),
            scat_start: Vec::new(),
            scat: Vec::new(),
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
            wide_diffs: Vec::new(),
            changed: Vec::new(),
            is_changed: Vec::new(),
            watch: Vec::new(),
            stateful_readers: Vec::new(),
            vcd_on: true,
            sv: Vec::new(),
            sx: Vec::new(),
            diff: vec![0],
            writes: Vec::new(),
            evaluations: 0,
            #[cfg(feature = "prof")]
            record: None,
            smalls: Vec::new(),
            sweep: false,
            sweep_gates: Vec::new(),
            gate_readers_start: Vec::new(),
            gate_readers: Vec::new(),
            group_start: Vec::new(),
            groups: Vec::new(),
            edge_flops: Vec::new(),
            triggered_groups: Vec::new(),
            notify: Vec::new(),
            last_evaluations: 0,
        };
        for width in widths {
            block.add_slot(width);
        }
        let slots = block.width.len();
        let mut readers: Vec<Vec<(u32, u32, u32, u64)>> = vec![Vec::new(); slots];
        // The operand slots by the bits they gather
        let mut shared: HashMap<Vec<(u32, u32)>, u32> = HashMap::new();
        // The combinational cells by rank, then the stateful ones
        let mut order: Vec<usize> = (0..cells.len()).collect();
        order.sort_by_key(|&i| (cells[i].rank == NO, cells[i].rank));
        block.ncomb = cells.iter().filter(|c| c.rank != NO).count();
        let mut slots_of: Vec<Option<CellArgs>> = cells.into_iter().map(Some).collect();
        let sorted: Vec<CellArgs> = order.iter().map(|&i| slots_of[i].take().unwrap()).collect();
        block.order = order.iter().map(|&i| i as u32).collect();
        for (number, args) in sorted.into_iter().enumerate() {
            let first = block.ins.len() as u32;
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
                } else if let Some(&slot) = shared.get(bits) {
                    // The same bits as an operand of another cell: one slot for both
                    block.gat.push((0, 0));
                    slot
                } else {
                    let slot = block.add_slot(bits.len() as u32);
                    shared.insert(bits.clone(), slot);
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
                            op: slot,
                            dst: index as u32,
                            slot: s,
                            bit: b,
                            len: len as u32,
                        });
                        index += len;
                    }
                    let range = (seg_first, block.segs.len() as u32);
                    block.gat.push(range);
                    slot
                };
                block.ins.push(slot);
                let stateful = args.rank == NO;
                // A stateful reader, and if the input updates it
                let flag = (stateful as u32) << 1 | (stateful && *update) as u32;
                // The bits it reads of each slot: a mask of a slot of up to 64 bits, the
                // runs of bits of a wider one
                let mut sources: Vec<(u32, Vec<u32>)> = Vec::new();
                for &(source, bit) in bits.iter().filter(|b| b.0 != NO) {
                    match sources.iter_mut().find(|s| s.0 == source) {
                        Some(found) => found.1.push(bit),
                        None => sources.push((source, vec![bit])),
                    }
                }
                let entry = (number as u32) << 2 | flag;
                for (source, mut read) in sources {
                    read.sort_unstable();
                    read.dedup();
                    let list = &mut readers[source as usize];
                    if block.width[source as usize] <= 64 {
                        let mask = read.iter().fold(0u64, |m, &b| m | 1 << b);
                        list.push((entry, read[0], read[read.len() - 1] + 1, mask));
                        continue;
                    }
                    let mut first = read[0];
                    let mut last = read[0];
                    for &bit in &read[1..] {
                        if bit != last + 1 {
                            list.push((entry, first, last + 1, 0));
                            first = bit;
                        }
                        last = bit;
                    }
                    list.push((entry, first, last + 1, 0));
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
                small: NO,
                table,
                op,
                first,
                count,
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
            // A cell that reads a slot both as an updating and a plain input updates: the
            // flags of all entries of a cell, one entry for a slot of up to 64 bits
            let mut merged: Vec<(u32, u32, u32, u64)> = Vec::new();
            let mut index = 0;
            while index < list.len() {
                let cell = list[index].0 >> 2;
                let end = index
                    + list[index..]
                        .iter()
                        .take_while(|e| e.0 >> 2 == cell)
                        .count();
                let flags = list[index..end].iter().fold(0, |f, e| f | (e.0 & 3));
                let group = &list[index..end];
                if group[0].3 != 0 {
                    let lo = group.iter().map(|e| e.1).min().unwrap();
                    let hi = group.iter().map(|e| e.2).max().unwrap();
                    let mask = group.iter().fold(0, |m, e| m | e.3);
                    merged.push((cell << 2 | flags, lo, hi, mask));
                } else {
                    // Runs of a wide slot: merged where they overlap
                    let mut runs: Vec<(u32, u32)> = group.iter().map(|e| (e.1, e.2)).collect();
                    runs.sort_unstable();
                    let mut current = runs[0];
                    for &(lo, hi) in &runs[1..] {
                        if lo <= current.1 {
                            current.1 = current.1.max(hi);
                        } else {
                            merged.push((cell << 2 | flags, current.0, current.1, 0));
                            current = (lo, hi);
                        }
                    }
                    merged.push((cell << 2 | flags, current.0, current.1, 0));
                }
                index = end;
            }
            // The readers of more than one bit, then those of one bit by the bit (found
            // with a binary search when one bit changes)
            let (mut narrow, wide): (Vec<_>, Vec<_>) =
                merged.into_iter().partition(|e| e.2 == e.1 + 1);
            narrow.sort_by_key(|e| e.1);
            block.fan.extend_from_slice(&wide);
            block.fan_narrow.push(block.fan.len() as u32);
            block.fan.extend_from_slice(&narrow);
            block.fan_start.push(block.fan.len() as u32);
        }
        let total = block.width.len();
        // The segs of each source slot, copied when it changes
        let mut by_source: Vec<Vec<u32>> = vec![Vec::new(); total];
        for (index, seg) in block.segs.iter().enumerate() {
            by_source[seg.slot as usize].push(index as u32);
        }
        block.scat_start.push(0);
        for list in by_source {
            let (mut narrow, wide): (Vec<u32>, Vec<u32>) = list
                .into_iter()
                .partition(|&i| block.segs[i as usize].len == 1);
            narrow.sort_by_key(|&i| block.segs[i as usize].bit);
            block.scat.extend_from_slice(&wide);
            block.scat_narrow.push(block.scat.len() as u32);
            block.scat.extend_from_slice(&narrow);
            block.scat_start.push(block.scat.len() as u32);
        }
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
        block.watch = (0..total)
            .map(|s| {
                let pins = block.pin_start[s] != block.pin_start[s + 1];
                let vcd = block.vcd_start[s] != block.vcd_start[s + 1];
                if pins {
                    1
                } else if vcd {
                    2
                } else {
                    0
                }
            })
            .collect();
        block.prepare_groups();
        block.prepare_small();
        block.stateful_readers = (0..block.width.len())
            .map(|s| {
                block.group_start[s] != block.group_start[s + 1]
                    || block.fan[block.fan_start[s] as usize..block.fan_start[s + 1] as usize]
                        .iter()
                        .any(|e| e.0 & 3 == 3)
            })
            .collect();
        block.prepare_sweep();
        block
    }

    /// The combinational functional cells of up to 64 bits, decoded once (see Small)
    fn prepare_small(&mut self) {
        for c in 0..self.ncomb {
            let cell = &self.cells[c];
            let Op::Word(kind) = cell.op else {
                continue;
            };
            let (first, count) = (cell.first as usize, cell.count as usize);
            if !small_kind(kind) || count == 0 || count > 3 {
                continue;
            }
            if cell.extra.as_ref().is_some_and(|e| e.outputs.len() != 1) {
                continue;
            }
            let ins = &self.ins[first..first + count];
            let y = cell.out;
            let widths: Vec<u32> = ins.iter().map(|&s| self.width[s as usize]).collect();
            if self.width[y as usize] > 64 || widths.iter().any(|&w| w > 64) {
                continue;
            }
            let at = |i: usize| ins[i.min(count - 1)];
            let small = Small {
                kind,
                a: self.start[at(0) as usize],
                b: self.start[at(1) as usize],
                s: self.start[at(2) as usize],
                aw: widths[0] as u8,
                bw: if count > 1 { widths[1] as u8 } else { 0 },
                yw: self.width[y as usize] as u8,
                signs: (cell.a_signed as u8)
                    | (cell.b_signed as u8) << 1
                    | (cell.b_signed_param as u8) << 2,
                y: self.start[y as usize],
                slot: y,
            };
            self.cells[c].small = self.smalls.len() as u32;
            self.smalls.push(small);
        }
    }

    /// The gate flip-flops that only their clock updates, in groups by clock (see
    /// EdgeGroup), and no longer readers of the clock slot
    fn prepare_groups(&mut self) {
        let slots = self.width.len();
        // Operands with gathered bits change without a fanout of their own
        let mut gathered = vec![false; slots];
        for seg in &self.segs {
            gathered[seg.op as usize] = true;
        }
        let mut by_clock: HashMap<(u32, u8), Vec<EdgeFlop>> = HashMap::new();
        let mut grouped = vec![false; self.cells.len()];
        for (c, cell) in self.cells.iter().enumerate() {
            let Some(gate) = cell.gate.as_ref() else {
                continue;
            };
            if !cell.clock_only || cell.count < 4 {
                continue;
            }
            let kind = match gate.kind {
                G::Dff => 0,
                G::Dffe => 1,
                G::Sdff => 2,
                G::Sdffe => 3,
                G::Sdffce => 4,
                _ => continue,
            };
            let first = cell.first as usize;
            let clock = self.ins[first];
            if gathered[clock as usize] {
                continue;
            }
            let flop = EdgeFlop {
                cell: c as u32,
                kind,
                enable: gate.enable_level,
                reset: gate.reset_level,
                reset_value: gate.reset_value,
                d: self.ins[first + crate::D],
                e: self.ins[first + crate::E],
                r: self.ins[first + crate::R],
                q: cell.q,
            };
            by_clock
                .entry((clock, cell.clock_level))
                .or_default()
                .push(flop);
            grouped[c] = true;
        }
        if by_clock.is_empty() {
            self.group_start = vec![0; slots + 1];
            return;
        }
        let mut keys: Vec<(u32, u8)> = by_clock.keys().copied().collect();
        keys.sort_unstable();
        let mut lists: Vec<Vec<u32>> = vec![Vec::new(); slots];
        for (clock, level) in keys {
            let flops = by_clock.remove(&(clock, level)).unwrap();
            let first = self.edge_flops.len() as u32;
            self.edge_flops.extend_from_slice(&flops);
            lists[clock as usize].push(self.groups.len() as u32);
            self.groups.push(EdgeGroup {
                level,
                // The clock level when the flip-flops were last evaluated
                old: self.cells[flops[0].cell as usize].old_clock,
                first,
                end: self.edge_flops.len() as u32,
            });
        }
        // The groups of each slot, in the order of the slots (groups is that order)
        let mut order: Vec<u32> = Vec::new();
        self.group_start = vec![0];
        for list in &lists {
            order.extend_from_slice(list);
            self.group_start.push(order.len() as u32);
        }
        debug_assert!(order.windows(2).all(|w| w[0] < w[1]));
        // The grouped flip-flops are not readers of their clock slots anymore
        let mut fan = Vec::with_capacity(self.fan.len());
        let mut fan_start = vec![0u32];
        let mut fan_narrow = Vec::new();
        for slot in 0..slots {
            let (start, narrow, end) = (
                self.fan_start[slot] as usize,
                self.fan_narrow[slot] as usize,
                self.fan_start[slot + 1] as usize,
            );
            let keep = |e: &(u32, u32, u32, u64)| {
                let c = (e.0 >> 2) as usize;
                !(grouped[c] && self.ins[self.cells[c].first as usize] == slot as u32)
            };
            fan.extend(self.fan[start..narrow].iter().copied().filter(keep));
            fan_narrow.push(fan.len() as u32);
            fan.extend(self.fan[narrow..end].iter().copied().filter(keep));
            fan_start.push(fan.len() as u32);
        }
        self.fan = fan;
        self.fan_start = fan_start;
        self.fan_narrow = fan_narrow;
    }

    /// The flip-flops of a group at the edge of their clock: their next outputs
    fn eval_group(&mut self, group: usize) {
        let (first, end) = (
            self.groups[group].first as usize,
            self.groups[group].end as usize,
        );
        for i in first..end {
            let f = self.edge_flops[i];
            let code = |slot: u32| self.c1[slot as usize];
            let next = match f.kind {
                0 => Some(code(f.d)),
                1 => (code(f.e) == f.enable).then(|| code(f.d)),
                2 => Some(if code(f.r) == f.reset {
                    f.reset_value
                } else {
                    code(f.d)
                }),
                3 => {
                    if code(f.r) == f.reset {
                        Some(f.reset_value)
                    } else {
                        (code(f.e) == f.enable).then(|| code(f.d))
                    }
                }
                _ => (code(f.e) == f.enable).then(|| {
                    if code(f.r) == f.reset {
                        f.reset_value
                    } else {
                        code(f.d)
                    }
                }),
            };
            if let Some(value) = next {
                let value = value.min(X);
                let q = f.q as usize;
                if self.c1[q] != value {
                    self.c1[q] = value;
                    let base = self.start[q] as usize;
                    self.v[base] = (value == 1) as u64;
                    self.x[base] = (value > 1) as u64;
                    self.set_pending(f.cell as usize);
                }
            }
        }
        self.evaluations += (end - first) as u64;
    }

    /// A block of gates only can sweep: evaluate all its gates in the order of their ranks,
    /// without dirty cells, like a compiled simulator. The gates are in arrays of their
    /// own, and a slot tells in notify what else needs its changes.
    fn prepare_sweep(&mut self) {
        let ncomb = self.ncomb;
        if ncomb == 0 || (0..ncomb).any(|c| self.cells[c].table == NO_TABLE) {
            return;
        }
        let zero = self.add_slot(1);
        self.c1[zero as usize] = 0;
        self.is_changed.push(false);
        self.watch.push(0);
        self.stateful_readers.push(false);
        self.fan_start.push(*self.fan_start.last().unwrap());
        self.fan_narrow.push(*self.fan_narrow.last().unwrap_or(&0));
        self.scat_start.push(*self.scat_start.last().unwrap());
        self.scat_narrow
            .push(*self.scat_narrow.last().unwrap_or(&0));
        self.pin_start.push(*self.pin_start.last().unwrap());
        self.vcd_start.push(*self.vcd_start.last().unwrap());
        for c in 0..ncomb {
            let cell = &self.cells[c];
            let (first, count) = (cell.first as usize, cell.count as usize);
            let input = |i: usize| if i < count { self.ins[first + i] } else { zero };
            self.sweep_gates.push(SweepGate {
                a: input(0),
                b: input(1),
                s: input(2),
                out: cell.out,
                table: cell.table as u32,
            });
        }
        let slots = self.width.len();
        // The sweep reads c1 without bounds checks
        assert!(self.sweep_gates.iter().all(|g| {
            [g.a, g.b, g.s, g.out].iter().all(|&s| (s as usize) < slots)
                && (g.table as usize) + 64 <= gate_table().len()
        }));
        self.notify = vec![0; slots];
        for slot in 0..slots {
            let mut flags = 0;
            if self.pin_start[slot] != self.pin_start[slot + 1] {
                flags |= 1;
            }
            for i in self.fan_start[slot] as usize..self.fan_start[slot + 1] as usize {
                let entry = self.fan[i].0;
                let c = (entry >> 2) as usize;
                if entry & 2 != 0 {
                    flags |= 2;
                }
                if !matches!(self.cells[c].op, Op::Gate(_)) {
                    flags |= 4;
                }
            }
            if self.scat_start[slot] != self.scat_start[slot + 1] {
                flags |= 6;
            }
            self.notify[slot] = flags;
        }
        self.gate_readers_start = vec![0];
        for slot in 0..slots {
            for i in self.fan_start[slot] as usize..self.fan_start[slot + 1] as usize {
                let entry = self.fan[i].0;
                if entry & 2 == 0 {
                    self.gate_readers.push(entry >> 2);
                }
            }
            self.gate_readers_start.push(self.gate_readers.len() as u32);
        }
        self.sweep = true;
    }

    /// Evaluate all gates (see prepare_sweep())
    /// A dirty gate of a block of gates only (see prepare_sweep()): its gates that read
    /// a slot are in gate_readers, the slots with other readers fan out as usual
    #[inline]
    fn eval_sweep_gate(&mut self, c: usize) {
        let gate = self.sweep_gates[c];
        let table = gate_table();
        // SAFETY: prepare_sweep() checked the slots and the part of the table
        let value = unsafe {
            let c1 = &self.c1;
            let index = gate.table as usize
                + ((*c1.get_unchecked(gate.a as usize) as usize) << 4
                    | (*c1.get_unchecked(gate.b as usize) as usize) << 2
                    | *c1.get_unchecked(gate.s as usize) as usize);
            *table.get_unchecked(index)
        };
        let o = gate.out as usize;
        if self.c1[o] == value {
            return;
        }
        self.c1[o] = value;
        let flags = self.notify[o];
        if flags != 0 {
            let base = self.start[o] as usize;
            self.v[base] = (value == 1) as u64;
            self.x[base] = (value > 1) as u64;
            if flags & 1 != 0 {
                self.mark_changed(o as u32);
            }
            if flags & 2 != 0 {
                self.fanout(o as u32);
                return;
            }
        }
        let (start, end) = (
            self.gate_readers_start[o] as usize,
            self.gate_readers_start[o + 1] as usize,
        );
        for i in start..end {
            let r = self.gate_readers[i] as usize;
            let word = r >> 6;
            self.dirty[word] |= 1u64 << (r & 63);
            if word < self.low {
                self.low = word;
            }
        }
    }

    /// Returns how many gate outputs changed.
    fn sweep_gates(&mut self) -> u64 {
        let table = gate_table();
        let mut changes = 0;
        for i in 0..self.sweep_gates.len() {
            let gate = self.sweep_gates[i];
            // SAFETY: prepare_sweep() checked the slots and the part of the table
            let (value, o) = unsafe {
                let c1 = &self.c1;
                let index = gate.table as usize
                    + ((*c1.get_unchecked(gate.a as usize) as usize) << 4
                        | (*c1.get_unchecked(gate.b as usize) as usize) << 2
                        | *c1.get_unchecked(gate.s as usize) as usize);
                (*table.get_unchecked(index), gate.out as usize)
            };
            if unsafe { *self.c1.get_unchecked(o) } == value {
                continue;
            }
            unsafe { *self.c1.get_unchecked_mut(o) = value };
            changes += 1;
            let flags = unsafe { *self.notify.get_unchecked(o) };
            if flags != 0 {
                let base = self.start[o] as usize;
                self.v[base] = (value == 1) as u64;
                self.x[base] = (value > 1) as u64;
                if flags & 1 != 0 {
                    self.mark_changed(o as u32);
                }
                if flags & 2 != 0 {
                    self.fanout(o as u32);
                }
            }
        }
        self.evaluations += self.sweep_gates.len() as u64;
        self.dirty.fill(0);
        self.low = self.dirty.len();
        changes
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

    pub fn slot_width(&self, slot: u32) -> u32 {
        self.width[slot as usize]
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
        #[cfg(feature = "prof")]
        if let Some((_, calls)) = self.record.as_mut() {
            calls.push(Call::Input(slot, bit, code));
        }
        let _t = crate::prof::Timer::new(105);
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

    /// An input word of up to 64 bits from outside the block (bits, undefined bits).
    /// Returns if the block needs a settle.
    pub fn set_input_word(&mut self, slot: u32, v: u64, x: u64) -> bool {
        #[cfg(feature = "prof")]
        if let Some((_, calls)) = self.record.as_mut() {
            calls.push(Call::Word(slot, v, x));
        }
        #[cfg(feature = "prof")]
        let saved = self.record.take();
        let result = self.set_input_word_now(slot, v, x);
        #[cfg(feature = "prof")]
        {
            self.record = saved;
        }
        result
    }

    fn set_input_word_now(&mut self, slot: u32, v: u64, x: u64) -> bool {
        let _t = crate::prof::Timer::new(108);
        let base = self.start[slot as usize] as usize;
        let m = mask(self.width[slot as usize]);
        let (x, v) = (x & m, v & m & !(x & m));
        let diff = (self.v[base] ^ v) | (self.x[base] ^ x);
        if diff == 0 {
            return false;
        }
        if self.updates_stateful(slot) {
            // Like Python sets the bits of a bus: one at a time, the lowest first (a
            // latch can see its data change before its enable)
            let mut bits = diff;
            while bits != 0 {
                let bit = bits.trailing_zeros();
                bits &= bits - 1;
                let code = if (x >> bit) & 1 != 0 {
                    X
                } else {
                    ((v >> bit) & 1) as u8
                };
                self.set_input(slot, bit, code);
            }
            return self.needs_settle();
        }
        self.v[base] = v;
        self.x[base] = x;
        self.c1[slot as usize] = if x & 1 != 0 { X } else { (v & 1) as u8 };
        self.fanout_mask(slot, diff);
        self.needs_settle()
    }

    /// Does a slot update a stateful cell (a clock, reset or latch input)
    #[inline]
    fn updates_stateful(&self, slot: u32) -> bool {
        self.stateful_readers[slot as usize]
    }

    /// The cells that read the bits of diff of a slot of up to 64 bits, see fanout_bits()
    fn fanout_mask(&mut self, slot: u32, diff: u64) {
        if diff.count_ones() == 1 {
            let bit = diff.trailing_zeros();
            self.fanout_bits(slot, bit, bit + 1);
            return;
        }
        let low = diff.trailing_zeros();
        let high = 64 - diff.leading_zeros();
        let s = slot as usize;
        let narrow = (self.fan_start[s + 1] - self.fan_narrow[s])
            .max(self.scat_start[s + 1] - self.scat_narrow[s]);
        let changes = diff.count_ones();
        if narrow > 4 * changes {
            // Many readers of one bit: found for each bit that changed (the operands get
            // all bits before a cell is evaluated)
            for (wide, narrow) in [(1, 2), (4, 8)] {
                self.fanout_part(slot, low, high, diff, None, wide);
                let mut bits = diff;
                while bits != 0 {
                    let bit = bits.trailing_zeros();
                    bits &= bits - 1;
                    self.fanout_part(slot, bit, bit + 1, u64::MAX, None, narrow);
                }
            }
            return;
        }
        self.fanout_range(slot, low, high, diff, None);
    }

    /// The registers start at 0, in the next settle (after their delay), and the constants
    /// get their values now (GND and VDD have no delay)
    pub fn default_state(&mut self) {
        #[cfg(feature = "prof")]
        {
            self.record = None;
        }
        self.default_state_now();
        #[cfg(feature = "prof")]
        {
            let copy = self.clone();
            self.record = Some((Box::new(copy), Vec::new()));
        }
    }

    /// The calls since default_state() again, from a copy of the block then, repeats
    /// times: returns the ns they took and the evaluations
    #[cfg(feature = "prof")]
    pub fn replay(&self, repeats: usize) -> (u64, u64, usize) {
        let Some((initial, calls)) = self.record.as_ref() else {
            return (0, 0, 0);
        };
        let mut total = 0;
        let mut evaluations = 0;
        let mut out = Vec::new();
        for _ in 0..repeats {
            let mut block = (**initial).clone();
            let start = std::time::Instant::now();
            for &call in calls {
                match call {
                    Call::Input(slot, bit, code) => {
                        block.set_input(slot, bit, code);
                    }
                    Call::Word(slot, v, x) => {
                        block.set_input_word(slot, v, x);
                    }
                    Call::Settle => {
                        out.clear();
                        block.settle(&mut out, None);
                    }
                }
            }
            total += start.elapsed().as_nanos() as u64;
            evaluations += block.evaluations;
        }
        (total, evaluations, calls.len())
    }

    fn default_state_now(&mut self) {
        self.vcd_on = true;
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
                        self.fanout_changed(output);
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

    /// The cells that read bits lo..hi of a slot: the bits to their operands, then they are
    /// dirty, or a stateful one updates
    #[inline]
    fn fanout_bits(&mut self, slot: u32, lo: u32, hi: u32) {
        self.fanout_range(slot, lo, hi, u64::MAX, None);
    }

    /// The readers of the bits that the last write of a slot changed (Python updates a
    /// cell only when a bit it reads changes)
    fn fanout_changed(&mut self, slot: u32) {
        if self.diff.len() <= 1 {
            let diff = self.diff.first().copied().unwrap_or(0);
            if diff != 0 {
                self.fanout_mask(slot, diff);
            }
            return;
        }
        let words = std::mem::take(&mut self.diff);
        let Some(first) = words.iter().position(|&w| w != 0) else {
            self.diff = words;
            return;
        };
        let last = words.iter().rposition(|&w| w != 0).unwrap();
        let lo = first as u32 * 64 + words[first].trailing_zeros();
        let hi = last as u32 * 64 + 64 - words[last].leading_zeros();
        self.fanout_range(slot, lo, hi, u64::MAX, Some(&words));
        self.diff = words;
    }

    /// fanout_bits(), and below 64 bits only for the bits of diff (or of words)
    fn fanout_range(&mut self, slot: u32, lo: u32, hi: u32, diff: u64, words: Option<&[u64]>) {
        self.fanout_part(slot, lo, hi, diff, words, 15);
    }

    /// fanout_range() of parts: the segs of more than one bit (1), of one bit (2), the
    /// groups and the readers of more than one bit (4), of one bit (8)
    fn fanout_part(
        &mut self,
        slot: u32,
        lo: u32,
        hi: u32,
        diff: u64,
        words: Option<&[u64]>,
        part: u8,
    ) {
        // The bits first..after of the slot have a bit of diff
        let touches = |first: u32, after: u32| {
            if let Some(words) = words {
                let mut bit = first;
                while bit < after {
                    let (w, o) = ((bit / 64) as usize, bit % 64);
                    let len = (after - bit).min(64 - o);
                    if words.get(w).is_some_and(|&d| (d >> o) & mask(len) != 0) {
                        return true;
                    }
                    bit += len;
                }
                return false;
            }
            if first >= 64 || diff == u64::MAX {
                return true;
            }
            let m = mask(after.min(64) - first) << first;
            m & diff != 0
        };
        let s = slot as usize;
        let (first, narrow, end) = (
            self.scat_start[s] as usize,
            self.scat_narrow[s] as usize,
            self.scat_start[s + 1] as usize,
        );
        // The segs of more than one bit (part 1), and those of the bits lo..hi (part 2):
        // all bits are in the operands before a cell is evaluated
        let first = if part & 1 != 0 { first } else { narrow };
        let (from, to) = if part & 2 == 0 {
            (end, end)
        } else if hi.saturating_sub(lo) <= 1 && narrow < end {
            let list = &self.scat[narrow..end];
            let a = list.partition_point(|&i| self.segs[i as usize].bit < lo);
            let b = list.partition_point(|&i| self.segs[i as usize].bit < hi);
            (narrow + a, narrow + b)
        } else {
            (narrow, end)
        };
        for k in (first..narrow).chain(from..to) {
            let seg = self.segs[self.scat[k] as usize];
            if seg.bit + seg.len <= lo || seg.bit >= hi || !touches(seg.bit, seg.bit + seg.len) {
                continue;
            }
            let src = self.start[slot as usize] as usize;
            let dst = self.start[seg.op as usize] as usize;
            let mut done = 0;
            while done < seg.len {
                let len = (seg.len - done).min(64);
                let v = get_bits(&self.v, src, seg.bit + done, len);
                let x = get_bits(&self.x, src, seg.bit + done, len);
                set_bits(&mut self.v, dst, seg.dst + done, len, v);
                set_bits(&mut self.x, dst, seg.dst + done, len, x);
                done += len;
            }
            if seg.dst == 0 {
                self.c1[seg.op as usize] = if self.x[dst] & 1 != 0 {
                    X
                } else {
                    (self.v[dst] & 1) as u8
                };
            }
        }
        let (start, narrow, end) = (
            self.fan_start[s] as usize,
            self.fan_narrow[s] as usize,
            self.fan_start[s + 1] as usize,
        );
        // The groups of flip-flops on the slot and the readers of more than one bit (part
        // 4), the readers of the bits lo..hi (part 8)
        if part & 4 != 0 {
            let (group, group_end) = (
                self.group_start[s] as usize,
                self.group_start[s + 1] as usize,
            );
            for g in group..group_end {
                let level = self.c1[s];
                if level == self.groups[g].old {
                    continue;
                }
                self.groups[g].old = level;
                if level != self.groups[g].level {
                    continue;
                }
                if self.in_comb {
                    self.triggered_groups.push(g as u32);
                } else {
                    self.eval_group(g);
                }
            }
        }
        let start = if part & 4 != 0 { start } else { narrow };
        let (from, to) = if part & 8 == 0 {
            (end, end)
        } else if hi.saturating_sub(lo) <= 1 && narrow < end {
            let list = &self.fan[narrow..end];
            (
                narrow + list.partition_point(|e| e.1 < lo),
                narrow + list.partition_point(|e| e.1 < hi),
            )
        } else {
            (narrow, end)
        };
        for i in (start..narrow).chain(from..to) {
            let (entry, first, after, read) = self.fan[i];
            if after <= lo || first >= hi {
                continue;
            }
            if read != 0 {
                // A slot of up to 64 bits: the bits it reads
                let changed = match words {
                    Some(words) => words.first().copied().unwrap_or(0),
                    None => diff & mask(hi.min(64)) & !mask(lo.min(64)),
                };
                if read & changed == 0 {
                    continue;
                }
            } else if !touches(first, after) {
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
                    // bit of its slot, then the level is the same). old_clock is the level
                    // of the clock.
                    let clock = self.ins[self.cells[c].first as usize];
                    let level = self.c1[clock as usize];
                    let cell = &mut self.cells[c];
                    if level == cell.old_clock {
                        continue;
                    }
                    cell.old_clock = level;
                    if level != cell.clock_level {
                        continue;
                    }
                    cell.old_clock = X;
                    if let Some(gate) = cell.gate.as_mut() {
                        gate.old_clock = X;
                    }
                }
                // (The edge set old_clock to X: the evaluation sets it to the level)
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
        let vcd_wanted = vcd.is_some();
        #[cfg(feature = "prof")]
        if let Some((_, calls)) = self.record.as_mut() {
            calls.push(Call::Settle);
        }
        let _whole = crate::prof::Timer::new(106);
        self.vcd_on = vcd_wanted;
        // Like the regions of Verilog: the combinational cells settle, the flip-flops whose
        // clock (or reset) a combinational cell changed then see the settled values, then
        // the new outputs of the flip-flops (like non-blocking assignments), again until
        // nothing changes. A circuit that doesn't stop (an oscillator) goes on in the next
        // settle, one ns later.
        for _round in 0..ROUNDS {
            self.in_comb = true;
            if self.sweep && !vcd_wanted && self.low < self.dirty.len() {
                // Many dirty gates (or many in the last settle): a sweep of all is faster
                let frontier: u32 = self.dirty[self.low..].iter().map(|w| w.count_ones()).sum();
                let n = self.ncomb as u64;
                if frontier as u64 * 4 >= n || self.last_evaluations * 2 >= n {
                    // The gates a settle without a sweep would have evaluated, about
                    let t = crate::prof::now();
                    let changes = self.sweep_gates();
                    crate::prof::add(102, t);
                    self.last_evaluations = changes * 2;
                }
            }
            let before = self.evaluations;
            let t = crate::prof::now();
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
                if self.sweep {
                    self.eval_sweep_gate(word << 6 | bit);
                } else {
                    self.eval_comb(word << 6 | bit);
                }
            }
            self.low = self.dirty.len();
            if self.evaluations != before {
                self.last_evaluations = self.evaluations - before;
                crate::prof::add(104, t);
            }
            self.in_comb = false;
            let groups = std::mem::take(&mut self.triggered_groups);
            for &g in &groups {
                self.eval_group(g as usize);
            }
            self.triggered_groups = groups;
            self.triggered_groups.clear();
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
            let t = crate::prof::now();
            let pending = std::mem::take(&mut self.pending);
            let mut updated = std::mem::take(&mut self.updated);
            updated.clear();
            for &c in &pending {
                let c = c as usize;
                self.is_pending[c] = false;
                let (q, output) = (self.cells[c].q, self.cells[c].out);
                if self.copy_slot(q, output) {
                    self.mark_changed(output);
                    // The changed bits: of a wide slot in wide_diffs
                    if self.diff.len() == 1 {
                        updated.push((output, self.diff[0]));
                    } else {
                        updated.push((output, self.wide_diffs.len() as u64));
                        self.wide_diffs.push(self.diff.clone());
                    }
                }
            }
            for &(slot, diff) in &updated {
                if self.width[slot as usize] <= 64 {
                    self.fanout_mask(slot, diff);
                } else {
                    self.diff = std::mem::take(&mut self.wide_diffs[diff as usize]);
                    self.fanout_changed(slot);
                }
            }
            self.wide_diffs.clear();
            self.updated = updated;
            crate::prof::add(103, t);
            // pending has the flip-flops that the new outputs clocked, for the next round
            drop(pending);
        }
        let _output = crate::prof::Timer::new(107);
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

    #[inline]
    fn mark_changed(&mut self, slot: u32) {
        // Only the slots with ports outside, and with ports inside when there is a VCD file
        let watch = self.watch[slot as usize];
        if watch == 0 || (watch == 2 && !self.vcd_on) {
            return;
        }
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
        self.diff.clear();
        for i in 0..count {
            self.diff
                .push((self.v[b + i] ^ self.v[a + i]) | (self.x[b + i] ^ self.x[a + i]));
            if self.v[b + i] != self.v[a + i] || self.x[b + i] != self.x[a + i] {
                changed = true;
                self.v[b + i] = self.v[a + i];
                self.x[b + i] = self.x[a + i];
            }
        }
        self.c1[to as usize] = self.c1[from as usize];
        changed
    }

    /// The operands are up to date: fanout_bits() copies the bits of a slot that changed
    #[inline]
    fn gather(&mut self, _c: usize) {}

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
        let diff = (self.v[base] ^ v) | (self.x[base] ^ x);
        self.diff.clear();
        self.diff.push(diff);
        if diff == 0 {
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
        self.diff.clear();
        for i in 0..count {
            let m = if i + 1 == count {
                mask(width - 64 * i as u32)
            } else {
                u64::MAX
            };
            let x = self.sx[i] & m;
            let v = self.sv[i] & m & !x;
            self.diff
                .push((self.v[base + i] ^ v) | (self.x[base + i] ^ x));
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
        let small = self.cells[c].small;
        if small != NO {
            let m = self.smalls[small as usize];
            let operand =
                |w: u32, width: u8| (self.v[w as usize], self.x[w as usize], width as u32);
            let s = (self.v[m.s as usize], self.x[m.s as usize]);
            let Some((v, x)) = small_op(
                m.kind,
                operand(m.a, m.aw),
                operand(m.b, m.bw),
                s,
                m.yw as u32,
                m.signs & 1 != 0,
                m.signs & 2 != 0,
                m.signs & 4 != 0,
            ) else {
                return self.slow(c);
            };
            let ym = mask(m.yw as u32);
            let x = x & ym;
            let v = v & ym & !x;
            let y = m.y as usize;
            let diff = (self.v[y] ^ v) | (self.x[y] ^ x);
            if diff == 0 {
                return;
            }
            self.v[y] = v;
            self.x[y] = x;
            self.c1[m.slot as usize] = if x & 1 != 0 { X } else { (v & 1) as u8 };
            self.mark_changed(m.slot);
            self.fanout_mask(m.slot, diff);
            return;
        }
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
            self.fanout_bits(output, 0, 1);
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
            self.fanout_changed(output);
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
                self.fanout_changed(slot);
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
        let s3 = if input_count > 2 {
            let (sv, sx, _) = self.small(inputs[2]);
            (sv, sx)
        } else {
            (0, 0)
        };
        if kind == W::Decoder {
            let (v, x) = match kind {
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
            return Some(self.write_small(output, v, x));
        }
        let (v, x) = small_op(
            kind,
            (av, ax, aw),
            (bv, bx, bw),
            s3,
            yw,
            a_signed,
            b_signed,
            b_signed_p,
        )?;
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

/// A gate flip-flop that only its clock updates, in the group of its clock: its kind (0
/// Dff, 1 Dffe, 2 Sdff, 3 Sdffe, 4 Sdffce), the slots of D, E and R, its next output and
/// the levels of E and R, and the value of R
#[derive(Clone, Copy)]
struct EdgeFlop {
    cell: u32,
    kind: u8,
    enable: u8,
    reset: u8,
    reset_value: u8,
    d: u32,
    e: u32,
    r: u32,
    q: u32,
}

/// The flip-flops on one clock slot that one level of it clocks: they are evaluated
/// together, at the edge to the level
#[derive(Clone)]
struct EdgeGroup {
    level: u8,
    old: u8,
    first: u32,
    end: u32,
}

/// A combinational functional cell of up to 64 bits, decoded: the words of its operands
/// (A, B, S) and of Y in Block.v and x, their widths, A_SIGNED, B_SIGNED (both for the
/// operations that are signed only if both are) and B_SIGNED alone, and the slot of Y
#[derive(Clone, Copy)]
struct Small {
    kind: W,
    a: u32,
    b: u32,
    s: u32,
    aw: u8,
    bw: u8,
    yw: u8,
    signs: u8,
    y: u32,
    slot: u32,
}

/// The kinds small_op() computes
fn small_kind(kind: W) -> bool {
    use W::*;
    matches!(
        kind,
        Not | Pos
            | Neg
            | And
            | Or
            | Xor
            | Xnor
            | ReduceAnd
            | ReduceOr
            | ReduceXor
            | ReduceXnor
            | LogicNot
            | LogicAnd
            | LogicOr
            | Lt
            | Le
            | Gt
            | Ge
            | Eq
            | Ne
            | Eqx
            | Nex
            | Add
            | Sub
            | Mul
            | Shl
            | Shr
            | Sshr
            | Shift
            | Mux
            | Bwmux
            | Bweqx
    )
}

/// A gate of a sweep: its inputs (up to three), output and part of GATE_TABLE
#[derive(Clone, Copy)]
struct SweepGate {
    a: u32,
    b: u32,
    s: u32,
    out: u32,
    table: u32,
}

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

/// A combinational cell of up to 64 bits (the operands and Y): its Y, None if it isn't
/// one of these. a and b are (bits, undefined bits, width), s the select input.
#[allow(clippy::too_many_arguments)]
#[inline(always)]
fn small_op(
    kind: W,
    (av, ax, aw): (u64, u64, u32),
    (bv, bx, bw): (u64, u64, u32),
    s: (u64, u64),
    yw: u32,
    a_signed: bool,
    b_signed: bool,
    b_signed_p: bool,
) -> Option<(u64, u64)> {
    let ym = mask(yw);
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
            let (sv, sx) = (s.0, s.1);
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
            let (sv, sx) = (s.0, s.1);
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
        _ => return None,
    };
    Some((v, x))
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

    /// cargo test --release gate_chain -- --ignored --nocapture
    #[test]
    #[ignore]
    fn gate_chain() {
        let n = 4000;
        let mut widths = vec![1, 1, 1];
        let mut cells = Vec::new();
        for i in 0..n {
            let prev = if i == 0 { 2 } else { 3 + i - 1 };
            widths.push(1);
            cells.push(CellArgs {
                op: 1,
                inputs: vec![(vec![(prev as u32, 0)], true), (vec![(2, 0)], true)],
                outputs: vec![3 + i as u32],
                rank: i as u32,
                word: None,
                gate: Some(GateCell {
                    kind: G::Xor,
                    first: 0,
                    output: 0,
                    clock_edge: 0,
                    reset_level: 0,
                    reset_value: 0,
                    enable_level: 0,
                    set_level: 0,
                    load_level: 0,
                    old_clock: X,
                }),
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
            "Xor chain: {ns:.1} ns per gate, {} evaluations",
            block.evaluations
        );
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
