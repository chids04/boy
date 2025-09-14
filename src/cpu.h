#include <cstdint>
#include <stdlib.h>
#include "memory.h"

uint16_t mem[0xFFFF];

typedef struct {
    uint16_t AF; // accumulator and flags
    uint16_t BC;
    uint16_t DE;
    uint16_t HL;
    uint16_t SP; // stack pointer
    uint16_t PC; // program counter

    uint8_t operand;

} CPU;

void emulate_cycle(CPU *cpu){
    uint8_t opcode = read_byte(cpu->PC);
    cpu->PC++;


};

void nop(CPU *cpu) {
}

// load 16 bit immediate value into BC
void  ld_bc_imm16(CPU *cpu){
    uint16_t val = read_word(cpu->PC);
    cpu->PC += 2;
    cpu->BC = val;
}

// copy value in A to addr[BC]
void ld_bc_a(CPU *cpu) {
    memory[cpu->BC] = cpu->AF >> 8;
}

// increment BC register
void inc_bc(CPU *cpu) {
    cpu->BC += 1;
}

void dec_b(CPU *cpu) {
    cpu->BC -= 1;
}

void



void ld_b_u8(CPU *cpu) {

}








typedef void(*I_Handler)(CPU* cpu);

I_Handler handlers [] = {
    nop,
    ld_bc_imm16,
    ld_bc_a,
    inc_bc,

};
