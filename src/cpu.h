#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


// https://archive.gbdev.io/salvage/decoding_gbz80_opcodes/Decoding%20Gamboy%20Z80%20Opcodes.html
// used for the grouping of
// different CPU opcodes

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

// extract bit patterns in opcode
static inline uint8_t get_y(uint8_t opcode) {
    return (opcode & (0b111 << 3)) >> 3;
}

static inline uint8_t get_x(uint8_t opcode) {
    return (opcode & (0b11 << 6)) >> 6;
}

static inline uint8_t get_z(uint8_t opcode) {
    return opcode & 0b111;
}

static inline uint8_t get_q(uint8_t opcode) {
    return (opcode & (1 << 3)) >> 3;
}

static inline uint8_t get_p(uint8_t opcode) {
    return (opcode & (0b11 << 4)) >> 4;
}


typedef enum {
    FLAG_C = (1 << 4), // 0b00010000
    FLAG_H = (1 << 5), // 0b00100000
    FLAG_N = (1 << 6), // 0b01000000
    FLAG_Z = (1 << 7)  // 0b10000000
} Flags;

void set_flag(CPU *cpu, Flags flag);
void clear_flag(CPU *cpu, Flags flag);
bool is_flag_set(CPU*cpu, Flags flag);

typedef enum {
    REG_B,
    REG_C,
    REG_D,
    REG_E,
    REG_H,
    REG_L,
    REG_HL_8,
    REG_A
} Reg8;

typedef enum {
    REG_BC,
    REG_DE,
    REG_HL_16,
    REG_SP,
    REG_AF = REG_SP,
} Reg16;


// instruction handlers //

// x == 0

// z= 0
void nop(CPU *cpu);
void ld_nn_sp(CPU *cpu);
void stop(CPU *cpu);
void jr_d(CPU *cpu);
void jr_cc_d(CPU *cpu);


// z= 1
void ld_r16_imm16(CPU *cpu);
void add_hl_r16(CPU *cpu);

// z= 2
void ld_bc_a(CPU *cpu);
void ld_hli_a(CPU *cpu);
void ld_de_a(CPU *cpu);
void ld_hld_a(CPU *cpu);
void ld_a_bc(CPU* cpu);
void ld_a_hli(CPU *cpu);
void ld_a_de(CPU *cpu);
void ld_a_hld(CPU *cpu);

// z= 3
void inc_r16(CPU *cpu);
void dec_r16(CPU *cpu);

// z= 4
void inc_r8(CPU *cpu);

// z= 5
void dec_r8(CPU *cpu);

// z= 6
void ld_r8_imm8(CPU *cpu);

// z= 7
void rlca(CPU* cpu);
void rrca(CPU* cpu);
void rla(CPU* cpu);
void rra(CPU* cpu);
void daa(CPU* cpu);
void cpl(CPU* cpu);
void scf(CPU* cpu);
void ccf(CPU* cpu);

// x = 1
void ld_r8_r8(CPU *cpu);
void halt(CPU *cpu);

// x = 2
void alu_r8(CPU *cpu);


// alu ops
void add_a_r8(CPU *cpu);
void sub_a_r8(CPU *cpu);
void subc_a_r8(CPU *cpu);
void and_a_r8(CPU *cpu);
void xor_a_r8(CPU *cpu);
void or_a_r8(CPU *cpu);
void cp_r_r8(CPU *cpu);


// utils for common cpu operations //

// returns whether the flag is set for the specified condition
bool condition(uint8_t idx, CPU *cpu);

// reads immediate value and increments pc
uint8_t read_imm8(CPU *cpu);
uint16_t read_imm16(CPU *cpu);

// reads bytes immediately after instruction and increments PC
uint16_t read_r16(Reg16 reg, CPU *cpu, bool has_sp);
uint8_t read_r8(Reg8 reg, CPU *cpu);

// writes bytes to registers
void write_r16(uint16_t data, Reg16 reg, CPU *cpu);
void write_r8(uint8_t data, Reg8 reg, CPU *cpu);


// calculating half carries for inc / dec instructions
void inc8_half_carry(const uint8_t value, CPU *cpu);
void dec8_half_carry(const uint8_t value, CPU *cpu);
void inc16_half_carry(const uint8_t value, CPU *cpu);
void dec16_half_carry(const uint8_t value, CPU *cpu);

// half carries for add / sub instructions
void add8_half_carry(const uint8_t value, CPU *cpu);
void add8_carry(const uint8_t value, CPU *cpu);

void sub8_half_carry(const uint8_t value, CPU *cpu);
void sub8_carry(const uint8_t value, CPU *cpu);


void decode_instruction(CPU *cpu);

#ifdef __cplusplus
}
#endif
