// main.rs

//!
//! Main Program for Testing Z80 CPU and Memory in Rust
//!
//! This module serves as the entry point for testing the Z80 CPU emulator. It defines a series
//! of test cases that exercise various aspects of the Z80 CPU, including register operations,
//! arithmetic, memory access, stack operations, jumps, interrupts, I/O, and indexed instructions.
//! The tests are run sequentially in the `main` function, with assertions to verify correct behavior.
//!
//! Key Features:
//! - Nine comprehensive test cases covering core Z80 functionality
//! - Uses `SimpleMemory` for a 64KB address space
//! - Employs `DummyIoDevice` and a mock I/O device for testing I/O operations
//! - Resets CPU state between tests for isolation
//!
//! Dependencies:
//! - `z80_cpu.rs`: Provides the `Z80` CPU implementation and related structs/traits
//! - `memory.rs`: Provides the `SimpleMemory` implementation
//!
//! Usage:
//! Run with `cargo run` to execute all tests. Successful completion prints "All tests passed successfully!".
//! Use `cargo build` or `cargo test` to verify compilation (note: no unit tests are defined here).
//!
//! Limitations:
//! - Tests assume implemented opcodes; unimplemented opcodes will panic
//! - Memory is fixed at 64KB, not reflecting potential real-world configurations
//!

use z80_emulator::z80_cpu::{Z80 as CPU, FLAG_Z, FLAG_C, FLAG_N, IoDevice, DummyIoDevice};
use z80_emulator::memory::Memory;
use z80_emulator::memory::SimpleMemory;

fn main() {
    let mut memory = SimpleMemory::new();
    let mut io = DummyIoDevice;
    let mut cpu = CPU::new();

    // **Test 1: Basic Instructions and Register Loads**
    // Program: LD A, 0xAB; LD B, A; LD HL, 0x1234; HALT
    memory.write(0x0000, 0x3E); // LD A, n
    memory.write(0x0001, 0xAB);
    memory.write(0x0002, 0x47); // LD B, A
    memory.write(0x0003, 0x21); // LD HL, nn
    memory.write(0x0004, 0x34);
    memory.write(0x0005, 0x12);
    memory.write(0x0006, 0x76); // HALT
    cpu.step(&mut memory, &mut io); // LD A, 0xAB
    cpu.step(&mut memory, &mut io); // LD B, A
    cpu.step(&mut memory, &mut io); // LD HL, 0x1234
    cpu.step(&mut memory, &mut io); // HALT
    assert_eq!(cpu.get_a(), 0xAB, "Test 1: A should be 0xAB");
    assert_eq!(cpu.get_b(), 0xAB, "Test 1: B should be 0xAB");
    assert_eq!(cpu.get_hl(), 0x1234, "Test 1: HL should be 0x1234");
    assert!(cpu.is_halted(), "Test 1: CPU should be halted");

    // Reset for next test
    cpu = CPU::new();

    // **Test 2: Arithmetic and Flag Manipulations**
    // Program: LD A, 0xFE; LD C, 0x02; ADD A, C; LD C, 0x01; SUB C; HALT
    memory.write(0x0000, 0x3E); // LD A, n
    memory.write(0x0001, 0xFE); // A = 0xFE
    memory.write(0x0002, 0x0E); // LD C, n
    memory.write(0x0003, 0x02); // C = 0x02
    memory.write(0x0004, 0x81); // ADD A, C
    memory.write(0x0005, 0x0E); // LD C, n
    memory.write(0x0006, 0x01); // C = 0x01
    memory.write(0x0007, 0x91); // SUB C
    memory.write(0x0008, 0x76); // HALT

    cpu.step(&mut memory, &mut io); // LD A, 0xFE
    cpu.step(&mut memory, &mut io); // LD C, 0x02
    cpu.step(&mut memory, &mut io); // ADD A, C (0xFE + 0x02 = 0x00, sets Z=1, C=1)
    assert_eq!(cpu.get_a(), 0x00, "Test 2: A should be 0x00 after ADD");
    assert!(cpu.get_flag(FLAG_Z), "Test 2: Zero flag should be set after ADD");
    assert!(cpu.get_flag(FLAG_C), "Test 2: Carry flag should be set after ADD");
    cpu.step(&mut memory, &mut io); // LD C, 0x01
    cpu.step(&mut memory, &mut io); // SUB C (0x00 - 0x01 = 0xFF)
    cpu.step(&mut memory, &mut io); // HALT
    assert_eq!(cpu.get_a(), 0xFF, "Test 2: A should be 0xFF after SUB");
    assert!(!cpu.get_flag(FLAG_Z), "Test 2: Zero flag should be clear after SUB");
    assert!(cpu.get_flag(FLAG_C), "Test 2: Carry flag should be set after SUB");
    assert!(cpu.get_flag(FLAG_N), "Test 2: Subtract flag should be set after SUB");

    // Reset for next test
    cpu = CPU::new();

    // **Test 3: Memory Interactions and Stack Operations**
    // Program: LD SP, 0xFFFE; LD BC, 0xABCD; PUSH BC; POP DE; LD (0x1000), A; HALT
    memory.write(0x0000, 0x31); // LD SP, nn
    memory.write(0x0001, 0xFE);
    memory.write(0x0002, 0xFF);
    memory.write(0x0003, 0x01); // LD BC, nn
    memory.write(0x0004, 0xCD);
    memory.write(0x0005, 0xAB);
    memory.write(0x0006, 0xC5); // PUSH BC
    memory.write(0x0007, 0xD1); // POP DE
    memory.write(0x0008, 0x32); // LD (nn), A
    memory.write(0x0009, 0x00);
    memory.write(0x000A, 0x10);
    memory.write(0x000B, 0x76); // HALT
    cpu.set_a(0x55);
    cpu.step(&mut memory, &mut io); // LD SP, 0xFFFE
    cpu.step(&mut memory, &mut io); // LD BC, 0xABCD
    cpu.step(&mut memory, &mut io); // PUSH BC
    cpu.step(&mut memory, &mut io); // POP DE
    cpu.step(&mut memory, &mut io); // LD (0x1000), A
    cpu.step(&mut memory, &mut io); // HALT
    assert_eq!(cpu.get_sp(), 0xFFFE, "Test 3: SP should be 0xFFFE after PUSH/POP");
    assert_eq!(cpu.get_bc(), 0xABCD, "Test 3: BC should be 0xABCD");
    assert_eq!(cpu.get_de(), 0xABCD, "Test 3: DE should be 0xABCD after POP");
    assert_eq!(memory.read(0x1000), 0x55, "Test 3: Memory at 0x1000 should be 0x55");

    // Reset for next test
    cpu = CPU::new();

    // **Test 4: Jump and Conditional Jump Instructions**
    // Program: LD A, 0x00; JP Z, 0x0006; LD A, 0x01; HALT; LD A, 0xFF; HALT
    memory.write(0x0000, 0x3E); // LD A, n
    memory.write(0x0001, 0x00);
    memory.write(0x0002, 0xCA); // JP Z, nn
    memory.write(0x0003, 0x06);
    memory.write(0x0004, 0x00);
    memory.write(0x0005, 0x76); // HALT (skipped)
    memory.write(0x0006, 0x3E); // LD A, 0xFF
    memory.write(0x0007, 0xFF);
    memory.write(0x0008, 0x76); // HALT
    cpu.step(&mut memory, &mut io); // LD A, 0x00
    cpu.set_flag(FLAG_Z, true); // Set Zero flag manually
    cpu.step(&mut memory, &mut io); // JP Z, 0x0006
    cpu.step(&mut memory, &mut io); // LD A, 0xFF
    cpu.step(&mut memory, &mut io); // HALT
    assert_eq!(cpu.get_a(), 0xFF, "Test 4: A should be 0xFF after JP Z");
    assert_eq!(cpu.get_pc(), 0x0009, "Test 4: PC should be after second HALT");

    // Reset for next test
    cpu = CPU::new();

    // **Test 5: Call and Return Instructions**
    // Program: CALL 0x0004; HALT; LD A, 0xAA; RET; HALT
    memory.write(0x0000, 0xCD); // CALL nn
    memory.write(0x0001, 0x04);
    memory.write(0x0002, 0x00);
    memory.write(0x0003, 0x76); // HALT (skipped)
    memory.write(0x0004, 0x3E); // LD A, 0xAA
    memory.write(0x0005, 0xAA);
    memory.write(0x0006, 0xC9); // RET
    memory.write(0x0007, 0x76); // HALT
    cpu.set_sp(0xFFFE);
    cpu.step(&mut memory, &mut io); // CALL 0x0004
    cpu.step(&mut memory, &mut io); // LD A, 0xAA
    cpu.step(&mut memory, &mut io); // RET
    cpu.step(&mut memory, &mut io); // HALT at 0x0003
    assert_eq!(cpu.get_a(), 0xAA, "Test 5: A should be 0xAA after RET");
    assert_eq!(cpu.get_sp(), 0xFFFE, "Test 5: SP should be restored");
    assert_eq!(cpu.get_pc(), 0x0004, "Test 5: PC should be after HALT at 0x0003");

    // Reset for next test
    cpu = CPU::new();

    // **Test 6: Interrupt Handling (NMI and IM 1)**
    // Program: IM 1; EI; HALT (followed by manual NMI and IM 1 interrupt)
    memory.write(0x0000, 0xED); // IM 1
    memory.write(0x0001, 0x56);
    memory.write(0x0002, 0xFB); // EI
    memory.write(0x0003, 0x76); // HALT
    memory.write(0x0038, 0x3E); // LD A, 0x11 (IM 1 vector)
    memory.write(0x0039, 0x11);
    memory.write(0x003A, 0xC9); // RET
    memory.write(0x0066, 0x3E); // LD A, 0x22 (NMI vector)
    memory.write(0x0067, 0x22);
    memory.write(0x0068, 0xC9); // RET
    cpu.step(&mut memory, &mut io); // IM 1
    cpu.step(&mut memory, &mut io); // EI
    cpu.step(&mut memory, &mut io); // HALT
    cpu.trigger_nmi(&mut memory);    // Trigger NMI
    cpu.step(&mut memory, &mut io); // LD A, 0x22
    cpu.step(&mut memory, &mut io); // RET
    assert_eq!(cpu.get_a(), 0x22, "Test 6: A should be 0x22 after NMI");
    cpu = CPU::new(); // Reset for IM 1 test
    memory.write(0x0000, 0xED); // IM 1
    memory.write(0x0001, 0x56);
    memory.write(0x0002, 0xFB); // EI
    memory.write(0x0003, 0x76); // HALT
    cpu.step(&mut memory, &mut io); // IM 1
    cpu.step(&mut memory, &mut io); // EI
    cpu.step(&mut memory, &mut io); // HALT
    cpu.trigger_int(&mut memory, &mut io, 0); // Trigger IM 1 interrupt
    cpu.step(&mut memory, &mut io); // LD A, 0x11
    cpu.step(&mut memory, &mut io); // RET
    assert_eq!(cpu.get_a(), 0x11, "Test 6: A should be 0x11 after IM 1");

    // Reset for next test
    cpu = CPU::new();

    // **Test 7: I/O Operations**
    struct MockIoDevice {
        port_value: u8,
    }
    impl IoDevice for MockIoDevice {
        fn read(&self, _port: u16) -> u8 { self.port_value }
        fn write(&mut self, _port: u16, value: u8) { self.port_value = value }
    }
    let mut mock_io = MockIoDevice { port_value: 0x99 };
    // Program: OUT (0x10), A; IN A, (0x10); HALT
    memory.write(0x0000, 0xD3); // OUT (n), A
    memory.write(0x0001, 0x10);
    memory.write(0x0002, 0xDB); // IN A, (n)
    memory.write(0x0003, 0x10);
    memory.write(0x0004, 0x76); // HALT
    cpu.set_a(0x77);
    cpu.step(&mut memory, &mut mock_io); // OUT (0x10), A
    cpu.step(&mut memory, &mut mock_io); // IN A, (0x10)
    cpu.step(&mut memory, &mut mock_io); // HALT
    assert_eq!(mock_io.port_value, 0x77, "Test 7: Port should be 0x77 after OUT");
    assert_eq!(cpu.get_a(), 0x77, "Test 7: A should be 0x77 after IN");

    // Reset for next test
    cpu = CPU::new();

    // **Test 8: CB-Prefixed Bit Operations**
    // Program: LD B, 0x01; BIT 0, B; SET 1, B; RES 0, B; HALT
    memory.write(0x0000, 0x06); // LD B, n
    memory.write(0x0001, 0x01);
    memory.write(0x0002, 0xCB); // BIT 0, B
    memory.write(0x0003, 0x40);
    memory.write(0x0004, 0xCB); // SET 1, B
    memory.write(0x0005, 0xC8);
    memory.write(0x0006, 0xCB); // RES 0, B
    memory.write(0x0007, 0x80);
    memory.write(0x0008, 0x76); // HALT
    cpu.step(&mut memory, &mut io); // LD B, 0x01
    cpu.step(&mut memory, &mut io); // BIT 0, B (Z=0)
    assert!(!cpu.get_flag(FLAG_Z), "Test 8: Zero flag should be clear after BIT");
    cpu.step(&mut memory, &mut io); // SET 1, B (B = 0x03)
    cpu.step(&mut memory, &mut io); // RES 0, B (B = 0x02)
    cpu.step(&mut memory, &mut io); // HALT
    assert_eq!(cpu.get_b(), 0x02, "Test 8: B should be 0x02 after bit operations");

    // Reset for next test
    cpu = CPU::new();

    // **Test 9: DD/FD-Prefixed IX/IY Operations**
    // Program: LD IX, 0x3000; LD (IX+2), 0xDD; LD IY, 0x4000; LD A, (IY+3); HALT
    memory.write(0x0000, 0xDD); // LD IX, nn
    memory.write(0x0001, 0x21);
    memory.write(0x0002, 0x00);
    memory.write(0x0003, 0x30);
    memory.write(0x0004, 0xDD); // LD (IX+d), n
    memory.write(0x0005, 0x36);
    memory.write(0x0006, 0x02);
    memory.write(0x0007, 0xDD);
    memory.write(0x0008, 0xFD); // LD IY, nn
    memory.write(0x0009, 0x21);
    memory.write(0x000A, 0x00);
    memory.write(0x000B, 0x40);
    memory.write(0x000C, 0xFD); // LD A, (IY+d)
    memory.write(0x000D, 0x7E);
    memory.write(0x000E, 0x03);
    memory.write(0x000F, 0x76); // HALT
    memory.write(0x4003, 0xEE); // Preload memory for IY test
    cpu.step(&mut memory, &mut io); // LD IX, 0x3000
    cpu.step(&mut memory, &mut io); // LD (IX+2), 0xDD
    cpu.step(&mut memory, &mut io); // LD IY, 0x4000
    cpu.step(&mut memory, &mut io); // LD A, (IY+3)
    cpu.step(&mut memory, &mut io); // HALT
    assert_eq!(cpu.get_ix(), 0x3000, "Test 9: IX should be 0x3000");
    assert_eq!(memory.read(0x3002), 0xDD, "Test 9: (IX+2) should be 0xDD");
    assert_eq!(cpu.get_iy(), 0x4000, "Test 9: IY should be 0x4000");
    assert_eq!(cpu.get_a(), 0xEE, "Test 9: A should be 0xEE from (IY+3)");

    println!("All tests passed successfully!");
}
