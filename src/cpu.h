#pragma once

#include "memory.h"
#include <cstdint>
#include <cstdio>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

// https://gbdev.io/pandocs/CPU_Instruction_Set.html used for the grouping of
// different CPU opcodes

static bool lookup_table[256];

typedef struct {
  uint16_t AF; // accumulator and flags
  uint16_t BC;
  uint16_t DE;
  uint16_t HL;
  uint16_t SP; // stack pointer
  uint16_t PC; // program counter

  uint8_t operand;

} CPU;

typedef void (*I_Handler)(CPU *cpu);
I_Handler i_handlers[256];

void nop(CPU *cpu);

// ;
void create_instruction_table() {
  memset(lookup_table, false, sizeof(lookup_table));
  memset(i_handlers, 0, sizeof(I_Handler));

  i_handlers[0b00000000] = nop;  // nop
  i_handlers[0b00000111] = NULL; // rcla
  i_handlers[0b00001111] = NULL; // rrca
  i_handlers[0b00010111] = NULL; // rla
  i_handlers[0b00011111] = NULL; // rra
  i_handlers[0b00100111] = NULL; // daa
  i_handlers[0b00101111] = NULL; // cpl
  i_handlers[0b00110111] = NULL; // scf
  i_handlers[0b00111111] = NULL; // ccf
  i_handlers[0b00011000] = NULL; // jr imm8
  i_handlers[0b00010000] = NULL; // stop
  i_handlers[0b00001000] = NULL; // ld imm16 sp
}

bool is_constant(uint8_t value) { return lookup_table[value]; }

void decode_instruction(CPU *cpu) {
  uint8_t opcode = read_byte(cpu->PC);

  // get upper 2 msb and 4 lsb to decode instructions easily
  // can group instructions on their register access

  // first handle instructions that dont need any extra decoding
  if (i_handlers[opcode] != NULL) {
    i_handlers[opcode](cpu);
    return;
  }

  // create masks to get the required bits
  uint8_t u_mask = 0b11 << 6;

  uint8_t upper = (opcode & u_mask) >> 6;

  // still need to handle the instruction clock of each instruction
  // this can be done inside the instruction handler

  // need to devise a way wheter this is a 16bit register or 8bit register
  // access for 8 bit register, bit 2 is always 1

  switch (upper) {
  // my strategy is to start with the exact matches, and then
  // ones that may operate on different registers

  // block 0
  case 0x0:
    // check if jr cond, imm8 instruction
    // use the upper and lower 3 bits to check
    uint8_t j_u_mask = 0b111 << 5;
    uint8_t j_l_mask = 0b111;

    uint8_t j_upper = (opcode & j_u_mask) >> 5;
    uint8_t j_lower = (opcode & j_u_mask);

    if (j_upper == 0b001 && j_lower == 0b000) {
      jr_cond_imm8();
      return;
    }

    // check if 8 bit register accesss
    // for 8 bit instructions in block 0, bit 2 is always 1
    uint8_t mask = 1 << 2;
    uint8_t is_8bit = (opcode & mask) >> 2;

    if (is_8bit) {
      uint8_t l_mask = 0b111;
      uint8_t lower = opcode & l_mask;

      switch (lower) {
      case 0b100:
        inc_r8();
        return;

      case 0b101:
        dec_r8();
        return;
      case 0b110:
        ld_r8_imm8();
        return;
      }
    }

    uint8_t l_mask = 0b1111;
    uint8_t lower = opcode & l_mask;

    switch (lower) {
    case 0b0001:
      ld_r1_imm16();
      return;

    case 0b0010:
      ld_memr16_a();
      return;

    case 0b1010:
      ld_a_r16mem();
      return;

    case 0b0011:
      inc_r16();
      return;

    case 0b1011:
      dec_r16();
      return;

    case 0b1001:
      add_hl_r16();
      return;
    }
  }
}

void add8_half_carry(uint8_t value, CPU *cpu) {
  uint8_t ln = value & 0xF;
  uint16_t mask = 1 << 6;

  if (ln == 0xF) {
    cpu->AF = cpu->AF | mask;
  } else {
    cpu->AF = cpu->AF & ~mask;
  }
}

void sub8_half_carry_s8(uint8_t value, CPU *cpu) {
  uint8_t ln = value & 0xF;
  uint16_t mask = 1 << 6;

  if (ln == 0x0) {
    cpu->AF = cpu->AF | mask;
  } else {
    cpu->AF = cpu->AF & ~mask;
  }
}

void emulate_cycle(CPU *cpu) {
  uint8_t opcode = read_byte(cpu->PC);
  cpu->PC++;
};

void nop(CPU *cpu) {}

// load 16 bit immediate value into BC
void ld_bc_imm16(CPU *cpu) {
  uint16_t val = read_word(cpu->PC);
  cpu->PC += 2;
  cpu->BC = val;
}

// copy value in A to addr[BC]
void ld_bc_a(CPU *cpu) { memory[cpu->BC] = cpu->AF >> 8; }

// increment BC register
void inc_bc(CPU *cpu) {

  // set half carry
  uint8_t ln_b = (cpu->BC >> 8) & 0b00001111;
  add8_half_carry(ln_b, cpu);

  // check for addition overflow

  // if (ln_b == 0b1111) {
  //     cpu->AF = cpu->AF | 0b0000011111
  // }
  // else {
  // }

  // if ()
  // cpu->BC += 1;
}

// decrement BC register
void dec_bc(CPU *cpu) { cpu->BC -= 1; }

void ld_b_u8(CPU *cpu) {}

typedef void (*I_Handler)(CPU *cpu);

I_Handler handlers[] = {
    nop, ld_bc_imm16, ld_bc_a, inc_bc, dec_b, ld_b_u8,

};
