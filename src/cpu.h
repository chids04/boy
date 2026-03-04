#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "common.h"

/*
    decoded according to the schema at:
    https://archive.gbdev.io/salvage/decoding_gbz80_opcodes/Decoding%20Gamboy%20Z80%20Opcodes.html
 *
 */



struct CPU {
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
    bool ime;

    uint8_t opcode;
    int cycles;

    bool interrupt_delay;
    bool enable_interrupts ;

} ;

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
bool is_flag_set(CPU *cpu, Flags flag);

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

void init_cpu(CPU *cpu, uint8_t header_checksum);

// instruction handlers //

// x == 0

// z= 0
void nop(BOY *boy);
void ld_nn_sp(BOY *boy);
void stop(BOY *boy);
void jr_d(BOY *boy);
void jr_cc_d(BOY *boy);


// z= 1
void ld_r16_imm16(BOY *boy);
void add_hl_r16(BOY *boy);

// z= 2
void ld_bc_a(BOY *boy);
void ld_hli_a(BOY *boy);
void ld_de_a(BOY *boy);
void ld_hld_a(BOY *boy);
void ld_a_bc(BOY *boy);
void ld_a_hli(BOY *boy);
void ld_a_de(BOY *boy);
void ld_a_hld(BOY *boy);

// z= 3
void inc_r16(BOY *boy);
void dec_r16(BOY *boy);

// z= 4
void inc_r8(BOY *boy);

// z= 5
void dec_r8(BOY *boy);

// z= 6
void ld_r8_imm8(BOY *boy);

// z= 7
void rlca(BOY *boy);
void rrca(BOY *boy);
void rla(BOY *boy);
void rra(BOY *boy);
void daa(BOY *boy);
void cpl(BOY *boy);
void scf(BOY *boy);
void ccf(BOY *boy);

// x = 1
void ld_r8_r8(BOY *boy);
void halt(BOY *boy);

// x = 2

// alu ops
void alu_a_r8(BOY *boy);
void add_a_r8(BOY *boy);
void adc_a_r8(BOY *boy);
void sub_a_r8(BOY *boy);
void subc_a_r8(BOY *boy);
void and_a_r8(BOY *boy);
void xor_a_r8(BOY *boy);
void or_a_r8(BOY *boy);
void cp_a_r8(BOY *boy);


// x = 3

// z = 0
void ret_cc(BOY *boy);
void ld_0xFF00_n_A(BOY *boy);
void ld_A_0xFF00_n(BOY *boy);


void add_sp_d(BOY *boy);
void ld_hl_sp_d(BOY *boy);

// z = 1
void pop_r16(BOY *boy);
void ret(BOY *boy);
void reti(BOY *boy);
void jp_hl(BOY *boy);
void ld_sp_hl(BOY *boy);

// z=2
void jp_cc_nn(BOY *boy);
void ld_0xFF00_C_A(BOY *boy);
void ld_A_0xFF00_C(BOY *boy);
void ld_nn_a(BOY *boy);
void ld_a_nn(BOY *boy);

// z = 3
void jp_nn(BOY *boy);
void ei(BOY *boy);
void di(BOY *boy);

// z = 4
void call_cc_nn(BOY *boy);

// z = 5
void call_nn(BOY *boy);
void push_r16(BOY *boy);

// z = 6
// alu ops on 8 bit immediate value
void alu_a_n(BOY *boy);
void add_a_n(BOY *boy);
void adc_a_n(BOY *boy);
void sub_a_n(BOY *boy);
void subc_a_n(BOY *boy);
void and_a_n(BOY *boy);
void xor_a_n(BOY *boy);
void or_a_n(BOY *boy);
void cp_a_n(BOY *boy);

// z = 7
void rst(BOY *boy);

// CB Prefix

// x = 0
void rot(BOY *boy);
void rlc_r8(BOY *boy);
void rrc_r8(BOY *boy);
void rl_r8(BOY *boy);
void rr_r8(BOY *boy);
void sla_r8(BOY *boy);
void sra_r8(BOY *boy);
void swap_r8(BOY *boy);
void srl_r8(BOY *boy);

// x = 1
void bit_y_r8(BOY *boy);
void res_y_r8(BOY *boy);
void set_y_r8(BOY *boy);


// utils for common cpu operations //

// returns whether the flag is set for the specified condition
bool condition(CPU *cpu, uint8_t idx);

// reads immediate value and increments pc
uint8_t read_imm8(BOY *boy);
uint16_t read_imm16(BOY *boy);

uint16_t read_r16(CPU *cpu, Reg16 reg, bool has_af);
uint8_t read_r8(BOY *boy, Reg8 reg);

// writes bytes to registers
void write_r16(CPU *cpu, uint16_t data, Reg16 reg, bool has_af);
void write_r8(BOY *boy, uint8_t data, Reg8 reg);


// calculating half carries for inc / dec instructions
void inc8_half_carry(const uint8_t value, CPU *cpu);
void dec8_half_carry(const uint8_t value, CPU *cpu);
void inc16_half_carry(const uint8_t value, CPU *cpu);
void dec16_half_carry(const uint8_t value, CPU *boy);

void half_carry8(CPU *cpu, uint8_t a, uint8_t b);
void carry8(CPU *cpu, uint8_t a, uint8_t b);

// half carries for add / sub instructions
void add8_half_carry(const uint8_t value, CPU *cpu);
void add8_carry(const uint8_t value, CPU *cpu);

void sub8_half_carry(const uint8_t value, CPU *cpu);
void sub8_carry(const uint8_t value, CPU *cpu);


void decode_instruction(BOY *boy);
