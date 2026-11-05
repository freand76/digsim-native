// Copyright (c) Fredrik Andersson, 2023-2026
// All rights reserved

//! Time counters for profiling the engine, with the feature "prof" (maturin build
//! --features prof), see Engine.profile(). Without it they cost nothing.

#[cfg(feature = "prof")]
mod enabled {
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::time::Instant;
    static COUNT: [AtomicU64; 256] = [const { AtomicU64::new(0) }; 256];
    static NS: [AtomicU64; 256] = [const { AtomicU64::new(0) }; 256];
    pub type Stamp = Instant;
    pub fn now() -> Stamp {
        Instant::now()
    }
    pub fn add(index: usize, start: Stamp) {
        COUNT[index].fetch_add(1, Ordering::Relaxed);
        NS[index].fetch_add(start.elapsed().as_nanos() as u64, Ordering::Relaxed);
    }
    pub fn dump() -> Vec<(usize, u64, u64)> {
        (0..256)
            .filter_map(|i| {
                let c = COUNT[i].swap(0, Ordering::Relaxed);
                let n = NS[i].swap(0, Ordering::Relaxed);
                (c > 0).then_some((i, c, n))
            })
            .collect()
    }
}

#[cfg(not(feature = "prof"))]
mod enabled {
    #[derive(Clone, Copy)]
    pub struct Stamp;
    #[inline(always)]
    pub fn now() -> Stamp {
        Stamp
    }
    #[inline(always)]
    pub fn add(_index: usize, _start: Stamp) {}
    pub fn dump() -> Vec<(usize, u64, u64)> {
        Vec::new()
    }
}

pub use enabled::{add, dump, now, Stamp};

/// Adds the time until it is dropped
pub struct Timer(usize, Stamp);

impl Timer {
    #[inline(always)]
    pub fn new(index: usize) -> Timer {
        Timer(index, now())
    }
}

impl Drop for Timer {
    #[inline(always)]
    fn drop(&mut self) {
        add(self.0, self.1)
    }
}
