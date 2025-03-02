// z80_cpu.rs

//!
//! Zilog Z80 CPU Core Implementation in Rust
//!
//! This module provides an emulation of the Zilog Z80 microprocessor, a widely used 8-bit CPU
//! introduced in 1976. The Z80 features a 16-bit address bus (64KB address space), an 8-bit data
//! bus, and a rich instruction set with 158 opcodes, including undocumented ones. This
//! implementation includes the CPU's registers, instruction execution, interrupt handling, and
//! I/O operations, aiming for compatibility with the official Z80 specifications.
//!
//! Key Features:
//! - Full register set: main (A, F, B, C, D, E, H, L), alternate, index (IX, IY), SP, PC, I, R
//! - Flag register (F) with S, Z, H, P/V, N, C flags
//! - Support for standard opcodes, CB/DD/ED/FD prefixes
//! - Interrupt modes (IM 0, IM 1, IM 2) and NMI handling
//! - Cycle-accurate execution for implemented instructions
//!
//! Dependencies:
//! - `memory.rs`: Provides the `Memory` trait for memory access
//!
//! Usage:
//! Create a `Z80` instance with `Z80::new()`, then use `step()` to execute instructions,
//! interfacing with memory and I/O devices via traits.
//!
//! Limitations:
//! - Some opcodes (e.g., IN, OUT) are unimplemented; they panic with `unimplemented!()`.
//! - Undocumented opcodes and edge cases may not be fully supported yet.
//!

use crate::memory::Memory;

// Constants for flag bits in the F register
pub const FLAG_C: u8 = 0b00000001;  // Carry flag (bit 0)
pub const FLAG_N: u8 = 0b00000010;  // Add/Subtract flag (bit 1)
pub const FLAG_PV: u8 = 0b00000100; // Parity/Overflow flag (bit 2)
pub const FLAG_H: u8 = 0b00010000;  // Half-carry flag (bit 4)
pub const FLAG_Z: u8 = 0b01000000;  // Zero flag (bit 6)
pub const FLAG_S: u8 = 0b10000000;  // Sign flag (bit 7)

/// Represents the complete register set of the Z80 CPU.
pub struct Registers {
    pub a: u8,      // Accumulator
    pub f: u8,      // Flag register
    pub b: u8,      // B register
    pub c: u8,      // C register
    pub d: u8,      // D register
    pub e: u8,      // E register
    pub h: u8,      // H register (high byte of HL)
    pub l: u8,      // L register (low byte of HL)
    pub a_alt: u8,  // Alternate accumulator
    pub f_alt: u8,  // Alternate flag register
    pub b_alt: u8,  // Alternate B register
    pub c_alt: u8,  // Alternate C register
    pub d_alt: u8,  // Alternate D register
    pub e_alt: u8,  // Alternate E register
    pub h_alt: u8,  // Alternate H register
    pub l_alt: u8,  // Alternate L register
    pub ix: u16,    // Index register IX
    pub iy: u16,    // Index register IY
    pub sp: u16,    // Stack pointer
    pub pc: u16,    // Program counter
    pub i: u8,      // Interrupt vector register
    pub r: u8,      // Memory refresh register
}

impl Registers {
    /// Creates a new `Registers` instance with all values initialized to zero.
    pub fn new() -> Self {
        Registers {
            a: 0, f: 0, b: 0, c: 0, d: 0, e: 0, h: 0, l: 0,
            a_alt: 0, f_alt: 0, b_alt: 0, c_alt: 0, d_alt: 0, e_alt: 0, h_alt: 0, l_alt: 0,
            ix: 0, iy: 0, sp: 0, pc: 0, i: 0, r: 0,
        }
    }

    /// Gets the 16-bit BC register pair.
    pub fn get_bc(&self) -> u16 { ((self.b as u16) << 8) | self.c as u16 }

    /// Sets the 16-bit BC register pair.
    pub fn set_bc(&mut self, value: u16) { self.b = (value >> 8) as u8; self.c = value as u8; }

    /// Gets the 16-bit DE register pair.
    pub fn get_de(&self) -> u16 { ((self.d as u16) << 8) | self.e as u16 }

    /// Sets the 16-bit DE register pair.
    pub fn set_de(&mut self, value: u16) { self.d = (value >> 8) as u8; self.e = value as u8; }

    /// Gets the 16-bit HL register pair.
    pub fn get_hl(&self) -> u16 { ((self.h as u16) << 8) | self.l as u16 }

    /// Sets the 16-bit HL register pair.
    pub fn set_hl(&mut self, value: u16) { self.h = (value >> 8) as u8; self.l = value as u8; }

    /// Gets the 16-bit AF register pair.
    pub fn get_af(&self) -> u16 { ((self.a as u16) << 8) | self.f as u16 }

    /// Sets the 16-bit AF register pair.
    pub fn set_af(&mut self, value: u16) { self.a = (value >> 8) as u8; self.f = value as u8; }

    /// Sets a flag in the F register.
    pub fn set_flag(&mut self, flag: u8, value: bool) {
        if value { self.f |= flag; } else { self.f &= !flag; }
    }

    /// Gets the state of a flag in the F register.
    pub fn get_flag(&self, flag: u8) -> bool { self.f & flag != 0 }
}

/// Trait defining the interface for I/O devices in the Z80 emulator.
pub trait IoDevice {
    /// Reads a byte from the specified 16-bit port.
    fn read(&self, port: u16) -> u8;

    /// Writes a byte to the specified 16-bit port.
    fn write(&mut self, port: u16, value: u8);
}

/// A dummy I/O device that returns 0 on reads and ignores writes.
pub struct DummyIoDevice;

impl IoDevice for DummyIoDevice {
    fn read(&self, _port: u16) -> u8 { 0 }
    fn write(&mut self, _port: u16, _value: u8) {}
}

/// Represents the Z80 CPU, encapsulating its state and execution logic.
pub struct Z80 {
    registers: Registers, // CPU registers
    halted: bool,        // Halt state
    iff1: bool,          // Interrupt Flip-Flop 1 (enabled/disabled)
    iff2: bool,          // Interrupt Flip-Flop 2 (for NMI)
    im: u8,              // Interrupt mode (0, 1, or 2)
    cycles: u64,         // Total cycles elapsed
}

impl Z80 {
    /// Creates a new `Z80` instance with default values.
    pub fn new() -> Self {
        Z80 {
            registers: Registers::new(),
            halted: false,
            iff1: false,
            iff2: false,
            im: 0,
            cycles: 0,
        }
    }

    /// Executes one CPU instruction, returning the number of cycles consumed.
    pub fn step(&mut self, memory: &mut dyn Memory, io: &mut dyn IoDevice) -> u8 {
        if self.halted {
            self.registers.r = self.registers.r.wrapping_add(1);
            return 4; // HALT takes 4 cycles
        }
        let opcode = self.fetch_byte(memory);
        let cycles = self.execute(opcode, memory, io);
        self.cycles += cycles as u64;
        cycles
    }

    /// Fetches a byte from memory at PC, incrementing PC and R.
    pub fn fetch_byte(&mut self, memory: &dyn Memory) -> u8 {
        let byte = memory.read(self.registers.pc);
        self.registers.pc = self.registers.pc.wrapping_add(1);
        self.registers.r = self.registers.r.wrapping_add(1);
        byte
    }

    /// Fetches a 16-bit word from memory, incrementing PC twice.
    pub fn fetch_word(&mut self, memory: &dyn Memory) -> u16 {
        let low = self.fetch_byte(memory) as u16;
        let high = self.fetch_byte(memory) as u16;
        (high << 8) | low
    }

    /// Gets a register value by index (0-7: B, C, D, E, H, L, (HL), A).
    pub fn get_register(&self, r: u8, memory: &dyn Memory) -> u8 {
        match r {
            0 => self.registers.b,
            1 => self.registers.c,
            2 => self.registers.d,
            3 => self.registers.e,
            4 => self.registers.h,
            5 => self.registers.l,
            6 => memory.read(self.registers.get_hl()),
            7 => self.registers.a,
            _ => unreachable!("Invalid register index"),
        }
    }

    /// Sets a register value by index (0-7: B, C, D, E, H, L, (HL), A).
    pub fn set_register(&mut self, r: u8, value: u8, memory: &mut dyn Memory) {
        match r {
            0 => self.registers.b = value,
            1 => self.registers.c = value,
            2 => self.registers.d = value,
            3 => self.registers.e = value,
            4 => self.registers.h = value,
            5 => self.registers.l = value,
            6 => memory.write(self.registers.get_hl(), value),
            7 => self.registers.a = value,
            _ => unreachable!("Invalid register index"),
        }
    }

    /// Pushes the PC onto the stack.
    pub fn push_pc(&mut self, memory: &mut dyn Memory) {
        self.registers.sp = self.registers.sp.wrapping_sub(2);
        memory.write(self.registers.sp, self.registers.pc as u8);
        memory.write(self.registers.sp + 1, (self.registers.pc >> 8) as u8);
    }

    /// Pops the PC from the stack.
    pub fn pop_pc(&mut self, memory: &dyn Memory) {
        let low = memory.read(self.registers.sp) as u16;
        let high = memory.read(self.registers.sp + 1) as u16;
        self.registers.pc = (high << 8) | low;
        self.registers.sp = self.registers.sp.wrapping_add(2);
    }

    /// Executes the given opcode, returning the number of cycles.
    pub fn execute(&mut self, opcode: u8, memory: &mut dyn Memory, io: &mut dyn IoDevice) -> u8 {
        match opcode {
            
            0x00 => 4, // NOP
            0x01 => { let nn = self.fetch_word(memory); self.registers.set_bc(nn); 10 }, // LD BC, nn
            0x02 => { memory.write(self.registers.get_bc(), self.registers.a); 7 }, // LD (BC), A
            0x06 => { // LD B, n
                let n = self.fetch_byte(memory); // Fetch the immediate value
                self.registers.b = n;            // Load it into register B
                7                                // Return the number of cycles
            },
            0x0E => { self.registers.c = self.fetch_byte(memory); 7 }, // LD C, n
            0x21 => { let nn = self.fetch_word(memory); self.registers.set_hl(nn); 10 }, // LD HL, nn
            0x31 => { self.registers.sp = self.fetch_word(memory); 10 }, // LD SP, nn
            0x32 => { // LD (nn), A
                let nn = self.fetch_word(memory);
                memory.write(nn, self.registers.a);
                13
            },
            0x3E => { self.registers.a = self.fetch_byte(memory); 7 }, // LD A, n
            0x47 => { self.registers.b = self.registers.a; 4 }, // LD B, A
            0x76 => { self.halted = true; 4 }, // HALT
            0x80..=0x87 => self.add_a_r(self.get_register(opcode & 0x07, memory)), // ADD A, r
            0x90..=0x97 => self.sub_a_r(self.get_register(opcode & 0x07, memory)), // SUB r
            0xC3 => { self.registers.pc = self.fetch_word(memory); 10 }, // JP nn
            0xC5 => { // PUSH BC
                self.registers.sp = self.registers.sp.wrapping_sub(2);
                memory.write(self.registers.sp + 1, self.registers.b);
                memory.write(self.registers.sp, self.registers.c);
                10
            },
            0xC6 => { // ADD A, n
                let n = self.fetch_byte(memory);
                self.add_a_r(n);
                7 // Clock cycles for ADD A, n
            },
            0xCA => { // JP Z, nn
                let nn = self.fetch_word(memory);
                if self.registers.get_flag(FLAG_Z) {
                    self.registers.pc = nn;
                }
                10
            },
            0xCD => { // CALL nn
                let nn = self.fetch_word(memory);
                self.push_pc(memory);
                self.registers.pc = nn;
                17
            },
            0xC9 => { self.pop_pc(memory); 10 }, // RET
            0xCB => { let cb_opcode = self.fetch_byte(memory); self.execute_cb(cb_opcode, memory) },
            0xD1 => { // POP DE
                self.registers.e = memory.read(self.registers.sp);
                self.registers.d = memory.read(self.registers.sp + 1);
                self.registers.sp = self.registers.sp.wrapping_add(2);
                10
            },
            0xD3 => { // OUT (n), A
                let n = self.fetch_byte(memory); // Get the port number from memory
                io.write(n as u16, self.registers.a); // Write A's value to the port
                11 // Return the number of clock cycles
            },
            0xD6 => { // SUB n
                let n = self.fetch_byte(memory);
                self.sub_a_r(n);
                7 // Clock cycles for SUB n
            },
            0xDB => { // IN A, (n)
                let n = self.fetch_byte(memory);
                self.registers.a = io.read(n as u16);
                11
            },
            0xDD => { let dd_opcode = self.fetch_byte(memory); self.execute_dd(dd_opcode, memory, io) },
            0xED => { let ed_opcode = self.fetch_byte(memory); self.execute_ed(ed_opcode, memory, io) },
            0xFB => { self.iff1 = true; self.iff2 = true; 4 }, // EI
            0xFD => { let fd_opcode = self.fetch_byte(memory); self.execute_fd(fd_opcode, memory, io) },
            _ => unimplemented!("Opcode {:#04x} not implemented", opcode),
        }
    }

    /// Executes CB-prefixed instructions (bit operations).
    fn execute_cb(&mut self, opcode: u8, memory: &mut dyn Memory) -> u8 {
        let r = opcode & 0x07;
        let b = (opcode >> 3) & 0x07;
        match opcode >> 6 {
            0 => match (opcode >> 3) & 0x07 {
                0 => self.rlc_r(r, memory), // RLC r
                _ => unimplemented!("CB opcode {:#04x} not implemented", opcode),
            },
            1 => self.bit_b_r(b, r, memory), // BIT b, r
            2 => self.res_b_r(b, r, memory), // RES b, r
            3 => self.set_b_r(b, r, memory), // SET b, r
            _ => unreachable!("Invalid CB opcode prefix"),
        }
    }

    /// Executes DD-prefixed instructions (IX operations).
    fn execute_dd(&mut self, opcode: u8, memory: &mut dyn Memory, _io: &mut dyn IoDevice) -> u8 {
        match opcode {
            0x21 => { self.registers.ix = self.fetch_word(memory); 14 }, // LD IX, nn
            0x36 => { // LD (IX+d), n
                let d = self.fetch_byte(memory) as i8;
                let n = self.fetch_byte(memory);
                memory.write(self.registers.ix.wrapping_add(d as u16), n);
                19
            },
            _ => unimplemented!("DD opcode {:#04x} not implemented", opcode),
        }
    }

    /// Executes ED-prefixed instructions (extended operations).
    fn execute_ed(&mut self, opcode: u8, memory: &mut dyn Memory, _io: &mut dyn IoDevice) -> u8 {
        match opcode {
            0x56 => { self.im = 1; 8 }, // IM 1
            0xB0 => self.ldir(memory),  // LDIR
            _ => unimplemented!("ED opcode {:#04x} not implemented", opcode),
        }
    }

    /// Executes FD-prefixed instructions (IY operations).
    fn execute_fd(&mut self, opcode: u8, memory: &mut dyn Memory, _io: &mut dyn IoDevice) -> u8 {
        match opcode {
            0x21 => { self.registers.iy = self.fetch_word(memory); 14 }, // LD IY, nn
            0x7E => { // LD A, (IY+d)
                let d = self.fetch_byte(memory) as i8;
                self.registers.a = memory.read(self.registers.iy.wrapping_add(d as u16));
                19
            },
            _ => unimplemented!("FD opcode {:#04x} not implemented", opcode),
        }
    }

    /// ADD A, r: Adds register r to A, updating flags.
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
        if r == 6 { 7 } else { 4 } // (HL) takes 7 cycles, others 4
    }

    /// SUB r: Subtracts register r from A, updating flags.
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
        if r == 6 { 7 } else { 4 } // (HL) takes 7 cycles, others 4
    }

    /// BIT b, r: Tests bit b in register r, setting flags.
    fn bit_b_r(&mut self, b: u8, r: u8, memory: &dyn Memory) -> u8 {
        let value = self.get_register(r, memory);
        let bit = 1 << b;
        let zero = value & bit == 0;
        self.registers.set_flag(FLAG_Z, zero);
        self.registers.set_flag(FLAG_N, false);
        self.registers.set_flag(FLAG_H, true);
        if r == 6 { 12 } else { 8 } // (HL) takes 12 cycles, others 8
    }

    /// RES b, r: Resets bit b in register r.
    fn res_b_r(&mut self, b: u8, r: u8, memory: &mut dyn Memory) -> u8 {
        let value = self.get_register(r, memory);
        let result = value & !(1 << b);
        self.set_register(r, result, memory);
        if r == 6 { 15 } else { 8 } // (HL) takes 15 cycles, others 8
    }

    /// SET b, r: Sets bit b in register r.
    fn set_b_r(&mut self, b: u8, r: u8, memory: &mut dyn Memory) -> u8 {
        let value = self.get_register(r, memory);
        let result = value | (1 << b);
        self.set_register(r, result, memory);
        if r == 6 { 15 } else { 8 } // (HL) takes 15 cycles, others 8
    }

    /// RLC r: Rotates register r left, setting flags.
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
        if r == 6 { 15 } else { 8 } // (HL) takes 15 cycles, others 8
    }

    /// LDIR: Block transfer from (HL) to (DE), repeats until BC = 0.
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
            21 // Repeat takes 21 cycles
        } else {
            16 // Non-repeat takes 16 cycles
        }
    }

    /// Computes parity (true if even number of 1 bits).
    fn parity(value: u8) -> bool {
        value.count_ones() % 2 == 0
    }

    /// Triggers a Non-Maskable Interrupt (NMI).
    pub fn trigger_nmi(&mut self, memory: &mut dyn Memory) {
        self.halted = false;
        self.iff2 = self.iff1;
        self.iff1 = false;
        self.push_pc(memory);
        self.registers.pc = 0x0066; // NMI vector
        self.cycles += 11;
    }

    /// Triggers a maskable interrupt with the given bus value.
    pub fn trigger_int(&mut self, memory: &mut dyn Memory, io: &mut dyn IoDevice, bus_value: u8) -> u8 {
        if self.iff1 {
            self.halted = false;
            self.iff1 = false;
            self.iff2 = false;
            match self.im {
                0 => {
                    self.execute(bus_value, memory, io);
                    13 // Approximate for typical RST instruction
                },
                1 => {
                    self.push_pc(memory);
                    self.registers.pc = 0x0038; // IM 1 vector
                    self.cycles += 13;
                    13
                },
                2 => {
                    let addr = ((self.registers.i as u16) << 8) | bus_value as u16;
                    let low = memory.read(addr);
                    let high = memory.read(addr + 1);
                    let jump_addr = ((high as u16) << 8) | low as u16;
                    self.push_pc(memory);
                    self.registers.pc = jump_addr;
                    self.cycles += 19;
                    19
                },
                _ => unreachable!("Invalid interrupt mode"),
            }
        } else {
            0 // No cycles if interrupt not taken
        }
    }

    // Getter methods for testing
    pub fn get_a(&self) -> u8 { self.registers.a }
    pub fn get_b(&self) -> u8 { self.registers.b }
    pub fn get_bc(&self) -> u16 { self.registers.get_bc() }
    pub fn get_de(&self) -> u16 { self.registers.get_de() }
    pub fn get_hl(&self) -> u16 { self.registers.get_hl() }
    pub fn get_sp(&self) -> u16 { self.registers.sp }
    pub fn get_pc(&self) -> u16 { self.registers.pc }
    pub fn get_ix(&self) -> u16 { self.registers.ix }
    pub fn get_iy(&self) -> u16 { self.registers.iy }
    pub fn get_flag(&self, flag: u8) -> bool { self.registers.get_flag(flag) }
    pub fn is_halted(&self) -> bool { self.halted }

    // Setter methods for testing
    pub fn set_a(&mut self, value: u8) { self.registers.a = value }
    pub fn set_sp(&mut self, value: u16) { self.registers.sp = value }
    pub fn set_flag(&mut self, flag: u8, value: bool) { self.registers.set_flag(flag, value) }
}
