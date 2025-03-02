// main.rs

//!
//! Main Program for Z80 CPU Emulator in Rust
//!
//! This module serves as the entry point for the Z80 CPU emulator. It will eventually contain
//! the logic to load Z80 programs (e.g., from a ROM file), initialize the CPU and memory,
//! and run the emulation loop. Test cases have been moved to `tests/cpu_tests.rs` for
//! verification of CPU functionality.
//!
//! Key Features (Planned):
//! - Load and execute Z80 programs from files
//! - Interactive I/O handling
//! - Configurable memory and device setups
//!
//! Dependencies:
//! - `z80_cpu.rs`: Provides the `Z80` CPU implementation and related structs/traits
//! - `memory.rs`: Provides the `SimpleMemory` implementation
//!
//! Usage:
//! Run with `cargo run` to start the emulator. Use `cargo test` to execute integration tests.
//!

use z80_emulator::z80_cpu::{Z80 as CPU, DummyIoDevice};
use z80_emulator::memory::Memory;
use z80_emulator::memory::SimpleMemory;

fn main() {
    let mut memory = SimpleMemory::new();
    let mut io = DummyIoDevice;
    let mut cpu = CPU::new();

    // Load array
    let array = [10, 20, 30, 40, 50];
    for (i, &value) in array.iter().enumerate() {
        memory.write(0x1000 + i as u16, value);
    }

    // Print initial memory
    println!("Initial memory contents:");
    for i in 0..5 {
        println!("Memory[0x{:04X}] = {}", 0x1000 + i, memory.read(0x1000 + i));
    }

    // Corrected program
    let program = [
        0x21, 0x00, 0x10, // LD HL, 1000h
        0x06, 0x05,       // LD B, 5
        0x3E, 0x00,       // LD A, 0
        0x86,             // ADD A, (HL)
        0x23,             // INC HL
        0x10, 0xFC,       // DJNZ -4
        0x32, 0x00, 0x20, // LD (2000h), A
        0x76,             // HALT
    ];
    for (i, &byte) in program.iter().enumerate() {
        memory.write(i as u16, byte);
    }

    // Run with corrected logging
    let max_cycles = 1000;
    let mut cycles = 0;
    while !cpu.is_halted() && cycles < max_cycles {
        let current_pc = cpu.get_pc();
        // println!("Program Counter (before step): {current_pc}");
        cpu.step(&mut memory, &mut io);
        // println!("Program Counter (after step): {current_pc}");
        cycles += 1;
        println!(
            "PC: 0x{:04X}, A: {:02X}, B: {:02X}, HL: 0x{:04X}",
            cpu.get_pc().wrapping_sub(1), // PC after fetch
            cpu.get_a(),
            cpu.get_b(),
            cpu.get_hl()
        );
    }

    // Check the result
    let sum = memory.read(0x2000);
    println!("Sum at 0x2000: {}", sum);
    assert_eq!(sum, 150, "Sum should be 150");
}
