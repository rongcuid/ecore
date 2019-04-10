# ecore
A tiny RISC-V RV32E core, for the sole purpose to load program for another RISC-V core

This is meant to be an extremely simple, single-file implementation for a RV32E core.
It is built for control and IO, and the most important use is to house a firmware for bootloader.
Ecore also has a good amount of GPIO, but it has no interrupts.

Ecore uses a ssram register file with a single port.

All traps are fatal because the Privileged instruction set is not implemented,
there is no way to return from a trap. However, we never generate instruction misaligned exception.

It implements the absolute minimum RISC-V ISA: RV32E 1.9 with Integer base subset v2.1.

# Vectors

- 0x00000000: Reset
- 0x00000004: Trap
