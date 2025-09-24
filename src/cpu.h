#pragma once

#include "gb_mem.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define REG_B 0
#define REG_C 1
#define REG_D 2
#define REG_E 3
#define REG_H 4
#define REG_L 5
#define REG_HL_8 6
#define REG_A 7
// 16 bit registers (pairs of 8 bit registers)
#define REG_BC 0
#define REG_DE 1
#define REG_HL 2

// both SP and AF reg have same index, their uses depend on the instruction
#define REG_SP 3
#define REG_AF 3



// https://archive.gbdev.io/salvage/decoding_gbz80_opcodes/Decoding%20Gamboy%20Z80%20Opcodes.html
// used for the grouping of
// different CPU opcodes

static bool lookup_table[256];

typedef struct {
    uint8_t A; // registers
    uint8_t F;
    // 7: Z
    // 6: N
    // 5: H
    // 4: CY

    uint8_t B;
    uint8_t C;

    uint8_t D;
    uint8_t E;

    uint8_t H;
    uint8_t L;

    uint16_t SP; // stack pointer
    uint16_t PC; // program counter

    uint8_t opcode;

    int cycles;

} CPU;


// instruction handlers //

// block 0
void nop(CPU *cpu);
void ld_nn_sp(CPU *cpu);
void stop(CPU *cpu);
void jr_d(CPU *cpu);
void jr_cc_d(CPU *cpu);

// block 1
void ld_r16_imm16(CPU *cpu);
void add_hl_r16(CPU *cpu);

// block 2
void ld_bc_a(CPU *cpu);
void ld_hli_a(CPU *cpu);
void ld_de_a(CPU *cpu);
void ld_hld_a(CPU *cpu);
void ld_a_bc(CPU* cpu);
void ld_a_hli(CPU *cpu);
void ld_a_de(CPU *cpu);
void ld_a_hld(CPU *cpu);


void inc_r16(CPU *cpu);
void dec_r16(CPU *cpu);

void inc_r8(CPU *cpu);
void dec_r8(CPU *cpu);

void ld_r8_imm8(CPU *cpu);
// writing to register pairs



// utils for common cpu operations //

// returns whether the flag is set for the specified condition
bool condition(uint8_t idx, CPU *cpu);

// reads immediate value and increments pc
uint8_t read_n8(CPU *cpu);
uint16_t read_n16(CPU *cpu);

// reads bytes immediately after instruction and increments PC
uint16_t read_r16(uint8_t idx, CPU *cpu, bool has_sp);
uint8_t read_r8(uint8_t idx, CPU *cpu);

// writes bytes to registers
void write_r16(uint16_t data, uint8_t reg, CPU *cpu);
void write_r8(uint8_t data, uint8_t reg, CPU *cpu);


// utils for calculating half carries
void add8_half_carry(const uint8_t value, CPU *cpu);
void dec8_half_carry(const uint8_t value, CPU *cpu);
void add16_half_carry(const uint8_t value, CPU *cpu);
void dec16_half_carry(const uint8_t value, CPU *cpu);


void decode_instruction(CPU *cpu);

#ifdef __cplusplus
}
#endif
