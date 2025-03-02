// Constants for flag bits in the F register
const FLAG_C: u8 = 0b00000001;  // Carry
const FLAG_N: u8 = 0b00000010;  // Add/Subtract
const FLAG_PV: u8 = 0b00000100; // Parity/Overflow
const FLAG_H: u8 = 0b00010000;  // Half-carry
const FLAG_Z: u8 = 0b01000000;  // Zero
const FLAG_S: u8 = 0b10000000;  // Sign

// Registers struct to hold all Z80 registers
struct Registers {
    a: u8,
    f: u8,
    b: u8,
    c: u8,
    d: u8,
    e: u8,
    h: u8,
    l: u8,
    a_alt: u8,
    f_alt: u8,
    b_alt: u8,
    c_alt: u8,
    d_alt: u8,
    e_alt: u8,
    h_alt: u8,
    l_alt: u8,
    ix: u16,
    iy: u16,
    sp: u16,
    pc: u16,
    i: u8,
    r: u8,
}

impl Registers {
    fn new() -> Self {
        Registers {
            a: 0, f: 0, b: 0, c: 0, d: 0, e: 0, h: 0, l: 0,
            a_alt: 0, f_alt: 0, b_alt: 0, c_alt: 0, d_alt: 0, e_alt: 0, h_alt: 0, l_alt: 0,
            ix: 0, iy: 0, sp: 0, pc: 0, i: 0, r: 0,
        }
    }

    // 16-bit register pair accessors
    fn get_bc(&self) -> u16 { ((self.b as u16) << 8) | self.c as u16 }
    fn set_bc(&mut self, value: u16) { self.b = (value >> 8) as u8; self.c = value as u8; }
    fn get_de(&self) -> u16 { ((self.d as u16) << 8) | self.e as u16 }
    fn set_de(&mut self, value: u16) { self.d = (value >> 8) as u8; self.e = value as u8; }
    fn get_hl(&self) -> u16 { ((self.h as u16) << 8) | self.l as u16 }
    fn set_hl(&mut self, value: u16) { self.h = (value >> 8) as u8; self.l = value as u8; }
    fn get_af(&self) -> u16 { ((self.a as u16) << 8) | self.f as u16 }
    fn set_af(&mut self, value: u16) { self.a = (value >> 8) as u8; self.f = value as u8; }

    // Flag manipulation
    fn set_flag(&mut self, flag: u8, value: bool) {
        if value { self.f |= flag; } else { self.f &= !flag; }
    }
    fn get_flag(&self, flag: u8) -> bool { self.f & flag != 0 }
}

// Memory trait for flexible memory implementations
trait Memory {
    fn read(&self, address: u16) -> u8;
    fn write(&mut self, address: u16, value: u8);
}

// Simple 64KB memory implementation
struct SimpleMemory {
    data: [u8; 65536],
}

impl SimpleMemory {
    fn new() -> Self {
        SimpleMemory { data: [0; 65536] }
    }
}

impl Memory for SimpleMemory {
    fn read(&self, address: u16) -> u8 { self.data[address as usize] }
    fn write(&mut self, address: u16, value: u8) { self.data[address as usize] = value; }
}

// I/O device trait
trait IoDevice {
    fn read(&self, port: u16) -> u8;
    fn write(&mut self, port: u16, value: u8);
}

// Dummy I/O device that does nothing
struct DummyIoDevice;

impl IoDevice for DummyIoDevice {
    fn read(&self, _port: u16) -> u8 { 0 }
    fn write(&mut self, _port: u16, _value: u8) {}
}

// Z80 CPU struct
struct Z80 {
    registers: Registers,
    halted: bool,
    iff1: bool, // Interrupt Flip-Flop 1
    iff2: bool, // Interrupt Flip-Flop 2
    im: u8,     // Interrupt Mode (0, 1, or 2)
    cycles: u64, // Total cycles elapsed
}

impl Z80 {
    fn new() -> Self {
        Z80 {
            registers: Registers::new(),
            halted: false,
            iff1: false,
            iff2: false,
            im: 0,
            cycles: 0,
        }
    }

    // Execute one CPU step
    fn step(&mut self, memory: &mut dyn Memory, io: &mut dyn IoDevice) -> u8 {
        if self.halted {
            self.registers.r = self.registers.r.wrapping_add(1);
            return 4; // HALT consumes 4 cycles
        }
        let opcode = self.fetch_byte(memory);
        let cycles = self.execute(opcode, memory, io);
        self.cycles += cycles as u64;
        cycles
    }

    fn fetch_byte(&mut self, memory: &dyn Memory) -> u8 {
        let byte = memory.read(self.registers.pc);
        self.registers.pc = self.registers.pc.wrapping_add(1);
        self.registers.r = self.registers.r.wrapping_add(1);
        byte
    }

    fn fetch_word(&mut self, memory: &dyn Memory) -> u16 {
        let low = self.fetch_byte(memory) as u16;
        let high = self.fetch_byte(memory) as u16;
        (high << 8) | low
    }

    // Get register by index (0-7: B, C, D, E, H, L, (HL), A)
    fn get_register(&self, r: u8, memory: &dyn Memory) -> u8 {
        match r {
            0 => self.registers.b,
            1 => self.registers.c,
            2 => self.registers.d,
            3 => self.registers.e,
            4 => self.registers.h,
            5 => self.registers.l,
            6 => memory.read(self.registers.get_hl()),
            7 => self.registers.a,
            _ => unreachable!(),
        }
    }

    fn set_register(&mut self, r: u8, value: u8, memory: &mut dyn Memory) {
        match r {
            0 => self.registers.b = value,
            1 => self.registers.c = value,
            2 => self.registers.d = value,
            3 => self.registers.e = value,
            4 => self.registers.h = value,
            5 => self.registers.l = value,
            6 => memory.write(self.registers.get_hl(), value),
            7 => self.registers.a = value,
            _ => unreachable!(),
        }
    }

    fn push_pc(&mut self, memory: &mut dyn Memory) {
        self.registers.sp = self.registers.sp.wrapping_sub(2);
        memory.write(self.registers.sp, self.registers.pc as u8);
        memory.write(self.registers.sp + 1, (self.registers.pc >> 8) as u8);
    }

    fn pop_pc(&mut self, memory: &dyn Memory) {
        let low = memory.read(self.registers.sp);
        let high = memory.read(self.registers.sp + 1);
        self.registers.pc = ((high as u16) << 8) | low as u16;
        self.registers.sp = self.registers.sp.wrapping_add(2);
    }

    // Execute an opcode
    fn execute(&mut self, opcode: u8, memory: &mut dyn Memory, io: &mut dyn IoDevice) -> u8 {
        match opcode {
            0x00 => 4, // NOP
            0x01 => { let nn = self.fetch_word(memory); self.registers.set_bc(nn); 10 }, // LD BC, nn
            0x02 => { memory.write(self.registers.get_bc(), self.registers.a); 7 }, // LD (BC), A
            0x0E => { self.registers.c = self.fetch_byte(memory); 7 }, // LD C, n
            0x21 => { let nn = self.fetch_word(memory); self.registers.set_hl(nn); 10 }, // LD HL, nn
            0x31 => { self.registers.sp = self.fetch_word(memory); 10 }, // LD SP, nn
            0x3E => { self.registers.a = self.fetch_byte(memory); 7 }, // LD A, n
            0x47 => { self.registers.b = self.registers.a; 4 }, // LD B, A
            0x76 => { self.halted = true; 4 }, // HALT
            0x80..=0x87 => self.add_a_r(self.get_register(opcode & 0x07, memory)), // ADD A, r
            0x90..=0x97 => self.sub_a_r(self.get_register(opcode & 0x07, memory)), // SUB r
            0xC3 => { self.registers.pc = self.fetch_word(memory); 10 }, // JP nn
            0xCD => { let nn = self.fetch_word(memory); self.push_pc(memory); self.registers.pc = nn; 17 }, // CALL nn
            0xC9 => { self.pop_pc(memory); 10 }, // RET
            0xCB => { let cb_opcode = self.fetch_byte(memory); self.execute_cb(cb_opcode, memory) },
            0xDD => { let dd_opcode = self.fetch_byte(memory); self.execute_dd(dd_opcode, memory, io) },
            0xED => { let ed_opcode = self.fetch_byte(memory); self.execute_ed(ed_opcode, memory, io) },
            0xFD => { let fd_opcode = self.fetch_byte(memory); self.execute_fd(fd_opcode, memory, io) },
            _ => unimplemented!("Opcode {:#04x} not implemented", opcode),
        }
    }

    // CB-prefixed instructions (bit operations)
    fn execute_cb(&mut self, opcode: u8, memory: &mut dyn Memory) -> u8 {
        let r = opcode & 0x07;
        let b = (opcode >> 3) & 0x07;
        match opcode >> 6 {
            0 => match (opcode >> 3) & 0x07 {
                0 => self.rlc_r(r, memory),
                _ => unimplemented!("CB opcode {:#04x} not implemented", opcode),
            },
            1 => self.bit_b_r(b, r, memory),
            2 => self.res_b_r(b, r, memory),
            3 => self.set_b_r(b, r, memory),
            _ => unreachable!(),
        }
    }

    // DD-prefixed instructions (IX operations)
    fn execute_dd(&mut self, opcode: u8, memory: &mut dyn Memory, _io: &mut dyn IoDevice) -> u8 {
        match opcode {
            0x21 => { self.registers.ix = self.fetch_word(memory); 14 }, // LD IX, nn
            _ => unimplemented!("DD opcode {:#04x} not implemented", opcode),
        }
    }

    // ED-prefixed instructions (extended operations)
    fn execute_ed(&mut self, opcode: u8, memory: &mut dyn Memory, _io: &mut dyn IoDevice) -> u8 {
        match opcode {
            0xB0 => self.ldir(memory),
            _ => unimplemented!("ED opcode {:#04x} not implemented", opcode),
        }
    }

    // FD-prefixed instructions (IY operations)
    fn execute_fd(&mut self, opcode: u8, memory: &mut dyn Memory, _io: &mut dyn IoDevice) -> u8 {
        match opcode {
            0x21 => { self.registers.iy = self.fetch_word(memory); 14 }, // LD IY, nn
            _ => unimplemented!("FD opcode {:#04x} not implemented", opcode),
        }
    }

    // Instruction implementations
    fn add_a_r(&mut self, r: u8) -> u8 {
        let a = self.registers.a;
        let result = a.wrapping_add(r);
        let half_carry = (a & 0x0F) + (r & 0x0F) > 0x0F;
        let overflow = ((a ^ r) & 0x80) == 0 && ((a ^ result) & 0x80) != 0;
        let carry = (a as u16) + (r as u16) > 0xFF;
        self.registers.a = result;
        self.registers.set_flag(FLAG_S, result & 0x80 != 0);
        self.registers.set_flag(FLAG_Z, result == 0);
        self.registers.set_flag(FLAG_H, half_carry);
        self.registers.set_flag(FLAG_PV, overflow);
        self.registers.set_flag(FLAG_N, false);
        self.registers.set_flag(FLAG_C, carry);
        if r == 6 { 7 } else { 4 }
    }

    fn sub_a_r(&mut self, r: u8) -> u8 {
        let a = self.registers.a;
        let result = a.wrapping_sub(r);
        let half_carry = (a & 0x0F) < (r & 0x0F);
        let overflow = ((a ^ r) & 0x80) != 0 && ((a ^ result) & 0x80) != 0;
        let carry = a < r;
        self.registers.a = result;
        self.registers.set_flag(FLAG_S, result & 0x80 != 0);
        self.registers.set_flag(FLAG_Z, result == 0);
        self.registers.set_flag(FLAG_H, half_carry);
        self.registers.set_flag(FLAG_PV, overflow);
        self.registers.set_flag(FLAG_N, true);
        self.registers.set_flag(FLAG_C, carry);
        if r == 6 { 7 } else { 4 }
    }

    fn bit_b_r(&mut self, b: u8, r: u8, memory: &dyn Memory) -> u8 {
        let value = self.get_register(r, memory);
        let bit = 1 << b;
        let zero = value & bit == 0;
        self.registers.set_flag(FLAG_Z, zero);
        self.registers.set_flag(FLAG_N, false);
        self.registers.set_flag(FLAG_H, true);
        if r == 6 { 12 } else { 8 }
    }

    fn res_b_r(&mut self, b: u8, r: u8, memory: &mut dyn Memory) -> u8 {
        let value = self.get_register(r, memory);
        let result = value & !(1 << b);
        self.set_register(r, result, memory);
        if r == 6 { 15 } else { 8 }
    }

    fn set_b_r(&mut self, b: u8, r: u8, memory: &mut dyn Memory) -> u8 {
        let value = self.get_register(r, memory);
        let result = value | (1 << b);
        self.set_register(r, result, memory);
        if r == 6 { 15 } else { 8 }
    }

    fn rlc_r(&mut self, r: u8, memory: &mut dyn Memory) -> u8 {
        let value = self.get_register(r, memory);
        let carry = value & 0x80 != 0;
        let result = (value << 1) | if carry { 1 } else { 0 };
        self.set_register(r, result, memory);
        self.registers.set_flag(FLAG_S, result & 0x80 != 0);
        self.registers.set_flag(FLAG_Z, result == 0);
        self.registers.set_flag(FLAG_H, false);
        self.registers.set_flag(FLAG_PV, Self::parity(result));
        self.registers.set_flag(FLAG_N, false);
        self.registers.set_flag(FLAG_C, carry);
        if r == 6 { 15 } else { 8 }
    }

    fn ldir(&mut self, memory: &mut dyn Memory) -> u8 {
        let hl = self.registers.get_hl();
        let de = self.registers.get_de();
        let bc = self.registers.get_bc();
        let value = memory.read(hl);
        memory.write(de, value);
        let new_bc = bc.wrapping_sub(1);
        self.registers.set_hl(hl.wrapping_add(1));
        self.registers.set_de(de.wrapping_add(1));
        self.registers.set_bc(new_bc);
        self.registers.set_flag(FLAG_H, false);
        self.registers.set_flag(FLAG_N, false);
        self.registers.set_flag(FLAG_PV, new_bc != 0);
        if new_bc != 0 {
            self.registers.pc = self.registers.pc.wrapping_sub(2);
            21
        } else {
            16
        }
    }

    fn parity(value: u8) -> bool {
        value.count_ones() % 2 == 0
    }

    // Interrupt handling
    fn trigger_nmi(&mut self, memory: &mut dyn Memory) {
        self.halted = false;
        self.iff2 = self.iff1;
        self.iff1 = false;
        self.push_pc(memory);
        self.registers.pc = 0x0066;
        self.cycles += 11;
    }

    fn trigger_int(&mut self, memory: &mut dyn Memory, io: &mut dyn IoDevice, bus_value: u8) {
        if self.iff1 {
            self.halted = false;
            self.iff1 = false;
            self.iff2 = false;
            match self.im {
                0 => {
                    self.execute(bus_value, memory, io);
                    // Cycles depend on the executed instruction
                },
                1 => {
                    self.push_pc(memory);
                    self.registers.pc = 0x0038;
                    self.cycles += 13;
                },
                2 => {
                    let addr = ((self.registers.i as u16) << 8) | bus_value as u16;
                    let low = memory.read(addr);
                    let high = memory.read(addr + 1);
                    let jump_addr = ((high as u16) << 8) | low as u16;
                    self.push_pc(memory);
                    self.registers.pc = jump_addr;
                    self.cycles += 19;
                },
                _ => unreachable!(),
            }
        }
    }
}

// Example usage
fn main() {
    let mut memory = SimpleMemory::new();
    let mut io = DummyIoDevice;
    let mut cpu = Z80::new();

    // Sample program: LD A, 0x12; LD B, A; HALT
    memory.write(0x0000, 0x3E); // LD A, n
    memory.write(0x0001, 0x12);
    memory.write(0x0002, 0x47); // LD B, A
    memory.write(0x0003, 0x76); // HALT

    cpu.step(&mut memory, &mut io); // LD A, 0x12
    cpu.step(&mut memory, &mut io); // LD B, A
    cpu.step(&mut memory, &mut io); // HALT

    assert_eq!(cpu.registers.a, 0x12);
    assert_eq!(cpu.registers.b, 0x12);
    assert!(cpu.halted);
    println!("Test passed: A = {:#04x}, B = {:#04x}, halted = {}", cpu.registers.a, cpu.registers.b, cpu.halted);
}
