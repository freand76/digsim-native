// Copyright (c) Fredrik Andersson, 2023-2026
// All rights reserved

//! A lock without atomics for the engine state. Python's GIL already lets one thread at a
//! time run the engine (and an abi3 extension isn't loaded by a free-threaded Python),
//! so the lock only has to catch the engine code itself taking the state twice, like a
//! Python call back into the engine while the state is taken. That panics, as a mutex
//! would deadlock.

use std::cell::{Cell, UnsafeCell};
use std::ops::{Deref, DerefMut};

pub struct Lock<T> {
    busy: Cell<bool>,
    value: UnsafeCell<T>,
}

// Safe with the GIL, see above
unsafe impl<T: Send> Sync for Lock<T> {}

impl<T> Lock<T> {
    pub fn new(value: T) -> Self {
        Lock {
            busy: Cell::new(false),
            value: UnsafeCell::new(value),
        }
    }

    pub fn lock(&self) -> Guard<'_, T> {
        assert!(!self.busy.replace(true), "The engine state is taken twice");
        Guard { lock: self }
    }
}

pub struct Guard<'a, T> {
    lock: &'a Lock<T>,
}

impl<T> Deref for Guard<'_, T> {
    type Target = T;
    fn deref(&self) -> &T {
        // The busy flag makes this the only reference
        unsafe { &*self.lock.value.get() }
    }
}

impl<T> DerefMut for Guard<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        unsafe { &mut *self.lock.value.get() }
    }
}

impl<T> Drop for Guard<'_, T> {
    fn drop(&mut self) {
        self.lock.busy.set(false);
    }
}
