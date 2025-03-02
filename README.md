# z80_emulator - Zilog Z80 'z80' CPU Core in Rust

`rust-z80` is a cycle-accurate emulator of the Zilog Z80 microprocessor, written in Rust. It aims to faithfully replicate the behavior of the original Z80 CPU, as found in classic systems like the ZX Spectrum, Amstrad CPC, and arcade machines. This project is ideal for retrocomputing enthusiasts, emulator developers, and anyone interested in low-level programming with Rust.

## Features

- **Full Instruction Set**: Supports all documented Z80 instructions, including 8-bit and 16-bit operations.
- **Cycle Accuracy**: Emulates instruction timing to match the real Z80's behavior.
- **Interrupt Handling**: Implements NMI and maskable interrupts (modes 0, 1, and 2).
- **Trait-Based I/O and Memory**: Flexible interfaces for custom memory and I/O device implementations.
- **Register Access**: Direct access to main and alternate register sets (extend as implemented).
- **Extensible Design**: Easy to integrate into larger projects or modify for specific use cases.

## Technical Details

The emulator is implemented as a Rust library crate, with a focus on modularity and performance. The core `Z80` struct manages the CPU state, including registers, program counter, and flags. Instruction execution is handled via the `step` method, which processes one opcode per call, updating the state based on memory and I/O interactions. The `Memory` and `IoDevice` traits allow users to define custom address spaces and peripherals, while default implementations (`SimpleMemory` and `DummyIoDevice`) provide a starting point.

## Installation

To use `rust-z80` in your Rust project, add it as a dependency in your `Cargo.toml`:

```toml
[dependencies]
rust-z80 = "0.1.0"
```

Alternatively, clone and build the repository locally:

```sh
git clone https://github.com/doublegate/z80-emulator.git
cd rust-z80
cargo build
```

Run tests to verify the build:

```sh
cargo test
```

## Usage

Here’s a simple example demonstrating how to initialize the emulator, load a program, and execute it:

```rust
use rust_z80::{Z80, Memory, SimpleMemory, IoDevice, DummyIoDevice};

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
```

This example loads a small Z80 program into memory, executes it step-by-step, and checks the CPU state. Extend this by implementing your full instruction set and adding more complex programs.

## Contributing

Contributions are welcome! If you find a bug, have a feature request, or want to improve the documentation, please:

1. Open an issue on the GitHub repository.
2. Submit a pull request with your changes.

Feel free to add test cases, optimize performance, or enhance the feature set.

## License

This project is licensed under the MIT License. See the LICENSE file for details.

## Acknowledgments

- Inspired by the Zilog Z80 microprocessor documentation.
- Built with the Rust programming language for safety and performance.

Happy emulating!
