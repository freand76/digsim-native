// Copyright (c) Fredrik Andersson, 2023-2026
// All rights reserved

//! An unbounded non-negative number, like a Python int, that doesn't allocate while it
//! fits in 64 bits. Most words of a netlist are that small, a register file can be wider.

use num_bigint::BigUint;
use num_traits::{One, ToPrimitive};
use std::ops::{BitAnd, BitAndAssign, BitOr, BitOrAssign, BitXor, BitXorAssign, Shl, Shr, Sub};

#[derive(Clone, PartialEq, Eq, Hash, Debug)]
pub enum Bits {
    Small(u64),
    // Only for numbers that don't fit in 64 bits
    Big(BigUint),
}

impl Default for Bits {
    fn default() -> Self {
        Bits::Small(0)
    }
}

impl From<u64> for Bits {
    fn from(value: u64) -> Self {
        Bits::Small(value)
    }
}

impl From<u32> for Bits {
    fn from(value: u32) -> Self {
        Bits::Small(value as u64)
    }
}

impl From<BigUint> for Bits {
    fn from(value: BigUint) -> Self {
        match value.to_u64() {
            Some(small) => Bits::Small(small),
            None => Bits::Big(value),
        }
    }
}

impl From<&Bits> for BigUint {
    fn from(value: &Bits) -> Self {
        match value {
            Bits::Small(v) => BigUint::from(*v),
            Bits::Big(v) => v.clone(),
        }
    }
}

impl Bits {
    pub fn zero() -> Bits {
        Bits::Small(0)
    }

    pub fn one() -> Bits {
        Bits::Small(1)
    }

    pub fn is_zero(&self) -> bool {
        matches!(self, Bits::Small(0))
    }

    pub fn to_u64(&self) -> Option<u64> {
        match self {
            Bits::Small(v) => Some(*v),
            Bits::Big(_) => None,
        }
    }

    pub fn to_usize(&self) -> Option<usize> {
        self.to_u64().and_then(|v| usize::try_from(v).ok())
    }

    /// The number with the bits below 'width'
    pub fn mask(width: usize) -> Bits {
        match width {
            0..=63 => Bits::Small((1u64 << width) - 1),
            64 => Bits::Small(u64::MAX),
            _ => Bits::Big((BigUint::one() << width) - 1u32),
        }
    }

    pub fn bit(&self, index: usize) -> bool {
        match self {
            Bits::Small(v) => index < 64 && (v >> index) & 1 == 1,
            Bits::Big(v) => v.bit(index as u64),
        }
    }

    pub fn set_bit(&mut self, index: usize, value: bool) {
        match self {
            Bits::Small(v) if index < 64 => {
                if value {
                    *v |= 1 << index;
                } else {
                    *v &= !(1 << index);
                }
            }
            _ => {
                let mut big = BigUint::from(&*self);
                big.set_bit(index as u64, value);
                *self = Bits::from(big);
            }
        }
    }

    pub fn count_ones(&self) -> u64 {
        match self {
            Bits::Small(v) => v.count_ones() as u64,
            Bits::Big(v) => v.count_ones(),
        }
    }

    /// The number of bits, up to the highest one
    pub fn bits(&self) -> u64 {
        match self {
            Bits::Small(v) => 64 - v.leading_zeros() as u64,
            Bits::Big(v) => v.bits(),
        }
    }

    /// The positions of the bits that are 1, lowest first
    pub fn ones(&self) -> Vec<usize> {
        let mut positions = Vec::new();
        let mut add = |offset: usize, mut digit: u64| {
            while digit != 0 {
                positions.push(offset + digit.trailing_zeros() as usize);
                digit &= digit - 1;
            }
        };
        match self {
            Bits::Small(v) => add(0, *v),
            Bits::Big(v) => {
                for (index, digit) in v.iter_u64_digits().enumerate() {
                    add(index * 64, digit);
                }
            }
        }
        positions
    }

    /// a & ~b
    pub fn andnot(&self, other: &Bits) -> Bits {
        match (self, other) {
            (Bits::Small(a), Bits::Small(b)) => Bits::Small(a & !b),
            (Bits::Small(a), Bits::Big(b)) => {
                Bits::Small(a & !b.iter_u64_digits().next().unwrap_or(0))
            }
            _ => self ^ &(self & other),
        }
    }
}

macro_rules! bitwise {
    ($trait:ident, $method:ident, $op:tt, $assign_trait:ident, $assign:ident) => {
        impl $trait<&Bits> for &Bits {
            type Output = Bits;
            fn $method(self, other: &Bits) -> Bits {
                match (self, other) {
                    (Bits::Small(a), Bits::Small(b)) => Bits::Small(a $op b),
                    _ => Bits::from(BigUint::from(self) $op BigUint::from(other)),
                }
            }
        }

        impl $trait<Bits> for &Bits {
            type Output = Bits;
            fn $method(self, other: Bits) -> Bits {
                self $op &other
            }
        }

        impl $trait<&Bits> for Bits {
            type Output = Bits;
            fn $method(self, other: &Bits) -> Bits {
                &self $op other
            }
        }

        impl $trait<Bits> for Bits {
            type Output = Bits;
            fn $method(self, other: Bits) -> Bits {
                &self $op &other
            }
        }

        impl $assign_trait<&Bits> for Bits {
            fn $assign(&mut self, other: &Bits) {
                *self = &*self $op other;
            }
        }
    };
}

bitwise!(BitAnd, bitand, &, BitAndAssign, bitand_assign);
bitwise!(BitOr, bitor, |, BitOrAssign, bitor_assign);
bitwise!(BitXor, bitxor, ^, BitXorAssign, bitxor_assign);

impl Shl<usize> for &Bits {
    type Output = Bits;
    fn shl(self, amount: usize) -> Bits {
        match self {
            Bits::Small(0) => Bits::Small(0),
            Bits::Small(v) if amount < 64 && v.leading_zeros() as usize >= amount => {
                Bits::Small(v << amount)
            }
            _ => Bits::from(BigUint::from(self) << amount),
        }
    }
}

impl Shl<usize> for Bits {
    type Output = Bits;
    fn shl(self, amount: usize) -> Bits {
        &self << amount
    }
}

impl Shr<usize> for &Bits {
    type Output = Bits;
    fn shr(self, amount: usize) -> Bits {
        match self {
            Bits::Small(v) => Bits::Small(if amount < 64 { v >> amount } else { 0 }),
            Bits::Big(v) => Bits::from(v >> amount),
        }
    }
}

impl Shr<usize> for Bits {
    type Output = Bits;
    fn shr(self, amount: usize) -> Bits {
        &self >> amount
    }
}

impl Sub<u32> for &Bits {
    type Output = Bits;
    fn sub(self, other: u32) -> Bits {
        match self {
            Bits::Small(v) => Bits::Small(v - other as u64),
            Bits::Big(v) => Bits::from(v - other),
        }
    }
}
