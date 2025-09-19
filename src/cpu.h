#pragma once

#include "memory.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// https://gbdev.io/pandocs/CPU_Instruction_Set.html used for the grouping of
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

typedef void (*I_Handler)(CPU *cpu);
I_Handler i_handlers[256];

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


// writing to register pairs



// utils for common cpu operations //

// returns whether the flag is set for the specified condition
bool condition(uint8_t idx, CPU *cpu);

// reads immediate value and increments pc
uint8_t read_n8(CPU *cpu);
uint16_t read_n16(CPU *cpu);

// reads bytes immediately after instruction and increments PC
uint16_t read_r16(uint8_t idx, CPU *cpu, bool has_sp=false);
uint8_t read_r8(uint8_t idx, CPU *cpu);

// writes bytes to the specificed 8 bit register pair
void write_rp(uint16_t data, uint8_t reg, CPU *cpu);

// utils for calculating half carries
void add8_half_carry(const uint8_t value, CPU *cpu);
void dec8_half_carry(const uint8_t value, CPU *cpu);
void add16_half_carry(const uint8_t value, CPU *cpu);
void dec16_half_carry(const uint8_t value, CPU *cpu);

void decode_instruction(CPU *cpu) {
   cpu->opcode = read_byte(cpu->PC++);

  uint8_t x_mask = 0b11 << 6;
  uint8_t x = (cpu->opcode & x_mask) >> 6;

  switch (x) {

  // block 0
  case 0:
    uint8_t z_mask = 0b111;
    uint8_t z = cpu->opcode & z_mask;

    uint8_t y_mask = 0b111 << 3;
    uint8_t y = (cpu->opcode & y_mask) >> 3;

    switch(z) {
        case 0:
            switch (y) {
                case 0:
                    nop(cpu);
                    return;

                case 1:
                    ld_nn_sp(cpu);
                    return;

                case 2:
                    stop(cpu);
                    return;

                case 3:
                    jr_d(cpu);
                    return;

                case 4 ... 7:
                    jr_cc_d(cpu);
                    return;
            }

            break;

        case 1:
            {
            uint8_t q = y % 2;
            switch (q) {
                case 0:
                    ld_r16_imm16(cpu);
                    return;
                case 1:
                    add_hl_r16(cpu);
                    return;
            }

            }


        case 2:
            {
            uint8_t q = y % 2;
            uint8_t p = y >> 1;
            switch (q) {
                case 0:
                    switch (p) {
                        case 0:

                        case 1:

                        case 2:

                        case 3:
                            return;
                    }

                    break;


                case 1:
                    switch (p) {
                        case 0:

                        case 1:

                        case 2:

                        case 3:
                            return;
                    }

                    break;
            }
            }

        }

    }
}
