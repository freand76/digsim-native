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

/// The event queue of the fast engine: a bucket for each time, first in first out. Events
/// at the same time come out in the order they were added, not in the order of heapq, so
/// gate-level designs with races can behave differently than with the exact queue.
#[derive(Default)]
pub struct Buckets {
    // By time, the earliest first: (time, events, the next event to pop)
    buckets: std::collections::VecDeque<(i64, Vec<Entry>, usize)>,
    // Emptied event lists, to use again
    pool: Vec<Vec<Entry>>,
    count: usize,
}

impl Buckets {
    pub fn len(&self) -> usize {
        self.count
    }

    pub fn push(&mut self, time: i64, entry: Entry) {
        self.count += 1;
        // Most events are for the latest times, look from the back
        let mut index = self.buckets.len();
        while index > 0 {
            let bucket = &mut self.buckets[index - 1];
            if bucket.0 == time {
                bucket.1.push(entry);
                return;
            }
            if bucket.0 < time {
                break;
            }
            index -= 1;
        }
        let mut entries = self.pool.pop().unwrap_or_default();
        entries.push(entry);
        self.buckets.insert(index, (time, entries, 0));
    }

    /// The time of the next event
    pub fn peek_time(&mut self) -> Option<i64> {
        loop {
            let (time, entries, next) = self.buckets.front()?;
            if *next < entries.len() {
                return Some(*time);
            }
            let (_, mut entries, _) = self.buckets.pop_front()?;
            entries.clear();
            self.pool.push(entries);
        }
    }

    pub fn pop(&mut self) -> Option<(i64, Entry)> {
        self.peek_time()?;
        let (time, entries, next) = self.buckets.front_mut()?;
        let entry = entries[*next];
        *next += 1;
        self.count -= 1;
        Some((*time, entry))
    }

    /// The events in the order they come out (a sorted list is also a heap)
    pub fn iter(&self) -> impl Iterator<Item = (i64, &Entry)> {
        self.buckets
            .iter()
            .flat_map(|(time, entries, next)| entries[*next..].iter().map(move |e| (*time, e)))
    }
}
