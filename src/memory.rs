// memory.rs

//!
//! Zilog Z80 Memory Implementation in Rust
//!
//! This module provides a memory interface and a simple implementation for the Z80 emulator.
//! The Z80 CPU uses a 16-bit address bus, allowing access to 64KB (65536 bytes) of memory.
//! This implementation includes a trait for generic memory access and a concrete struct
//! representing a 64KB memory block, sufficient for standard Z80 applications.
//!
//! Key Features:
//! - `Memory` trait: Defines `read` and `write` methods for memory operations
//! - `SimpleMemory`: A 64KB array-based memory with straightforward access
//!
//! Usage:
//! Use `SimpleMemory::new()` to create a 64KB memory instance. Implement the `Memory` trait
//! for custom memory models (e.g., banked memory or memory-mapped I/O).
//!
//! Limitations:
//! - Fixed 64KB size; no support for memory banking or larger address spaces in this version
//! - No memory protection or mapping features
//!

/// Trait defining the interface for memory access in the Z80 emulator.
pub trait Memory {
    /// Reads a byte from the specified 16-bit address.
    fn read(&self, address: u16) -> u8;

    /// Writes a byte to the specified 16-bit address.
    fn write(&mut self, address: u16, value: u8);
}

/// A simple memory implementation using a 64KB (65536-byte) array.
pub struct SimpleMemory {
    data: [u8; 65536], // 64KB memory block
}

impl SimpleMemory {
    /// Creates a new `SimpleMemory` instance with all bytes initialized to zero.
    pub fn new() -> Self {
        SimpleMemory { data: [0; 65536] }
    }
}

impl Memory for SimpleMemory {
    /// Reads a byte from the specified 16-bit address.
    fn read(&self, address: u16) -> u8 {
        self.data[address as usize]
    }

    /// Writes a byte to the specified 16-bit address.
    fn write(&mut self, address: u16, value: u8) {
        self.data[address as usize] = value;
    }
}
