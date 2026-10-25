// Copyright (c) Fredrik Andersson, 2023-2026
// All rights reserved

//! The event queue of the engine: CPython's heapq algorithm (Modules/_heapqmodule.c), so
//! that events at the same time come out in the same order as with the Python engine. It
//! compares only the times, and moves a hole instead of swapping entries: the same steps and the same heap as heapq, with less to
//! read and write.

/// An event without its time: (seq, port, value slot)
#[derive(Clone, Copy)]
pub struct Entry {
    pub seq: u64,
    pub port: u32,
    pub slot: u32,
}

#[derive(Clone, Copy)]
struct Item {
    time: i64,
    entry: Entry,
}

#[derive(Default)]
pub struct Queue {
    items: Vec<Item>,
}

impl Queue {
    pub fn len(&self) -> usize {
        self.items.len()
    }

    /// The events in the order of the heap
    pub fn iter(&self) -> impl Iterator<Item = (i64, &Entry)> {
        self.items.iter().map(|item| (item.time, &item.entry))
    }

    // _siftdown(): move the item at pos up to its place
    fn siftdown(&mut self, start: usize, mut pos: usize) {
        let item = self.items[pos];
        while pos > start {
            let parent = (pos - 1) >> 1;
            if item.time < self.items[parent].time {
                self.items[pos] = self.items[parent];
                pos = parent;
            } else {
                break;
            }
        }
        self.items[pos] = item;
    }

    // _siftup(): move the smaller child up to a leaf, then the item up to its place
    fn siftup(&mut self, mut pos: usize) {
        let end = self.items.len();
        let start = pos;
        let item = self.items[pos];
        let limit = end >> 1;
        while pos < limit {
            let mut child = 2 * pos + 1;
            // heapq: "not heap[child] < heap[child + 1]"
            if child + 1 < end && self.items[child].time >= self.items[child + 1].time {
                child += 1;
            }
            self.items[pos] = self.items[child];
            pos = child;
        }
        self.items[pos] = item;
        self.siftdown(start, pos);
    }

    pub fn push(&mut self, time: i64, entry: Entry) {
        self.items.push(Item { time, entry });
        let last = self.items.len() - 1;
        self.siftdown(0, last);
    }

    pub fn pop(&mut self) -> Option<(i64, Entry)> {
        let last = self.items.pop()?;
        if self.items.is_empty() {
            return Some((last.time, last.entry));
        }
        let first = std::mem::replace(&mut self.items[0], last);
        self.siftup(0);
        Some((first.time, first.entry))
    }
}
