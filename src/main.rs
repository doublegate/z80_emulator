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
    // Initialize the emulator components
    let mut memory = SimpleMemory::new();
    let mut io = DummyIoDevice;
    let mut cpu = CPU::new();

    // Z80 machine code program
    let program: [u8; 28] = [
        // Address 0x0000: Initialize registers
        0x21, 0x00, 0x10,  // LD HL, 0x1000  ; HL points to array start
        0x06, 0x05,        // LD B, 5        ; B is the loop counter (array length)
        0x3E, 0x00,        // LD A, 0        ; A holds the current max value
        0x0E, 0x00,        // LD C, 0        ; C holds the max index
        0x16, 0x00,        // LD D, 0        ; D is the current index

        // Address 0x000B: Loop start
        0x5E,              // LD E, (HL)     ; Load current array element into E
        0xBB,              // CP E           ; Compare A with E (A - E)
        0x30, 0x02,        // JR NC, +2      ; Jump to skip if A >= E (no carry)
        0x7B,              // LD A, E        ; Update max value
        0x4A,              // LD C, D        ; Update max index

        // Address 0x0011: Skip label
        0x23,              // INC HL         ; Move to next array element
        0x14,              // INC D          ; Increment current index
        0x10, 0xF6,        // DJNZ -10       ; Decrement B, jump to loop if B != 0

        // Address 0x0015: Store results
        0x21, 0x00, 0x20,  // LD HL, 0x2000  ; HL points to result storage
        0x77,              // LD (HL), A     ; Store max value
        0x23,              // INC HL         ; Move to next address
        0x71,              // LD (HL), C     ; Store max index
        0x76,              // HALT           ; Stop execution
    ];

    // Load the program into memory at 0x0000
    for (i, &byte) in program.iter().enumerate() {
        memory.write(0x0000 + i as u16, byte);
    }

    // Load the array [3, 5, 4, 2, 1] at 0x1000
    let array = [3, 5, 4, 2, 1];
    for (i, &value) in array.iter().enumerate() {
        memory.write(0x1000 + i as u16, value);
    }

    // Run the CPU until it halts or reaches a cycle limit
    let mut cycles = 0;
    let max_cycles = 1000;
    while !cpu.is_halted() && cycles < max_cycles {
        cpu.step(&mut memory, &mut io);
        cycles += 1;
    }

    // Verify execution
    assert!(cpu.is_halted(), "CPU did not halt within {} cycles", max_cycles);

    // Check results
    let max_value = memory.read(0x2000);
    let max_index = memory.read(0x2001);
    assert_eq!(max_value, 5, "Expected max value 5, got {}", max_value);
    assert_eq!(max_index, 1, "Expected max index 1, got {}", max_index);

    println!("Program executed successfully in {} cycles!", cycles);
}
