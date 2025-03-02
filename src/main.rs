use rust_z80::{Z80, Memory, SimpleMemory, DummyIoDevice};

fn main() {
    // Initialize components
    let mut memory = SimpleMemory::new();
    let mut io = DummyIoDevice;
    let mut cpu = Z80::new();

    // Load a program: LD A, 0x12; LD B, A; HALT
    memory.write(0x0000, 0x3E); // LD A, n
    memory.write(0x0001, 0x12); // Immediate value 0x12
    memory.write(0x0002, 0x47); // LD B, A
    memory.write(0x0003, 0x76); // HALT

    // Execute instructions
    cpu.step(&mut memory, &mut io); // LD A, 0x12
    cpu.step(&mut memory, &mut io); // LD B, A
    cpu.step(&mut memory, &mut io); // HALT

    // Verify results
    assert_eq!(cpu.registers.a, 0x12);
    assert_eq!(cpu.registers.b, 0x12);
    assert!(cpu.halted);
    println!("Test passed: A = {:#04x}, B = {:#04x}, halted = {}", 
             cpu.registers.a, cpu.registers.b, cpu.halted);
}
