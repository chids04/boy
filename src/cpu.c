#include "cpu.h"
#include <stdint.h>

void nop(CPU *cpu){
    printf("NOP instruction not implemented\n");
    cpu->cycles = 1;
}

void stop(CPU *cpu) {
    printf("STOP instruction not implemented\n");
}

void ld_nn_sp(CPU *cpu) {
    //copy stack pointer to mem[nn], with nn being word after opcode
    uint16_t addr = read_n16(cpu);
    write_word(addr, cpu->SP);

    cpu->cycles = 5;

}

void jr_d(CPU *cpu) {
    //add signed 8bit 'd' to next instruction,
    int8_t d = (int8_t)read_n8(cpu);
    cpu->PC += d;

    cpu->cycles = 2;
}

void jr_cc_d(CPU *cpu) {
    uint8_t mask = 0b111 << 3;

    int8_t d = (int8_t)read_n8(cpu);
    uint8_t cond = ((cpu->opcode & mask) >> 3) - 4;

    if(condition(cond, cpu)) {
        cpu->PC += d;
        cpu->cycles = 3;
    }
    else {
        cpu->cycles = 2;
    }
}

void lr_r16_imm16(CPU *cpu) {
    uint16_t imm16 = read_word(cpu->PC);
    uint8_t y_mask = 0b111 << 3;

    // p is top 2 msb of y (bits 5 - 4 of opcode)
    uint8_t p = (cpu->opcode & y_mask) >> 4;

    write_rp(imm16, p, cpu);

    cpu->cycles = 3;
    // switch on the different register pairs

}

void add_hl_r16(CPU *cpu) {
    // p is top 2 msb of y (bits 5 - 4 of opcode)
    uint8_t y_mask = 0b111 << 3;
    uint8_t p = (cpu->opcode & y_mask) >> 4;

    uint16_t data = read_r16(p, cpu);
    uint16_t hl = (cpu->H << 8) | cpu->L;

    write_rp(data + hl, 2, cpu);

    cpu->cycles = 2;
}

void inc_r8(CPU *cpu){
    // increment register r8 by 1
    //match on bit 3, 4 and 5
    uint8_t mask = 0b111 << 3;
    uint8_t sel = (cpu->opcode & mask) >> 3;

    switch (sel) {
        case 0b111:
            add8_half_carry(cpu->A, cpu);
        case 0b000:
            add8_half_carry(cpu->B, cpu);
        case 0b001:
            add8_half_carry(cpu->C, cpu);
        case 0b010:
            add8_half_carry(cpu->D, cpu);

        case 0b011:
            add8_half_carry(cpu->E, cpu);

        case 0b100:
            add8_half_carry(cpu->H, cpu);

        case 0b101:
            add8_half_carry(cpu->L, cpu);
    }
}

bool condition(uint8_t idx, CPU *cpu){
    switch (idx){
        case 0:
        {
            uint8_t mask = 1 << 7;
            uint8_t z = (cpu->F & mask) >> 7;
            if (z) return true;
            break;
        }
        case 1:
        {
            uint8_t mask = 1 << 6;
            uint8_t n = (cpu->F & mask) >> 6;
            if (n) return true;
            break;
        }

        case 2:
        {
            uint8_t mask = 1 << 5;
            uint8_t h = (cpu->F & mask) >> 6;
            if (h) return true;
            break;
        }

        case 3:
        {
            uint8_t mask = 1 << 4;
            uint8_t c = (cpu->F & mask) >> 4;
            if (c) return true;
            break;
        }

    }

    return false;


}

uint8_t read_n8(CPU *cpu){
    uint16_t data = read_byte(cpu->PC);
    cpu->PC += 1;

    return data;
}

uint16_t read_n16(CPU *cpu){
    uint16_t data = read_word(cpu->PC);
    cpu->PC += 2;

    return data;
}

uint8_t read_r8(uint8_t idx, CPU *cpu) {
    switch (idx) {
        case 0:
            return cpu->B;
        case 1:
            return cpu->C;
        case 2:
            return cpu->D;
        case 3:
            return cpu->E;
        case 4:
            return cpu->H;
        case 5:
            return cpu->L;
        case 6:
            return read_byte((cpu->H << 8) | cpu->L);
        case 7:
            return cpu->A;
    }
}

uint16_t read_r16(uint8_t idx, CPU *cpu, bool has_sp) {
    switch (idx) {
        case 0:
            return (cpu->B << 8) | cpu->C;

        case 1:
            return (cpu->D << 8) | cpu->E;

        case 2:
            return (cpu->H << 8) | cpu->L;

        case 3:
            if (has_sp){
                return cpu->SP;
            }
            else{
                return (cpu->A << 8) | cpu->F;
            }
    }

}

void write_rp(uint16_t data, uint8_t reg, CPU *cpu){
    switch (reg) {
        case 0:
            cpu->B = data >> 8;
            cpu->C = data & 0x00FF;
        case 1:
            cpu->D = data >> 8;
            cpu->E = data & 0x00FF;

        case 2:
            cpu->H = data >> 8;
            cpu->L = data & 0x00FF;

        case 3:
            cpu->SP = data;
    }
}



void add8_half_carry(const uint8_t value, CPU *cpu) {
  uint8_t lower = value & 0b111;
  uint8_t mask = 1 << 5; // at bit 5 (0 index)

  if (lower == 0b111) {
    cpu->F = cpu->F | mask;
  } else {
    cpu->F = cpu->F & ~mask;
  }
}

void dec8_half_carry(const uint8_t value, CPU *cpu) {
  uint8_t ln = value & 0xF;
  uint16_t mask = 1 << 5;

  if (ln == 0x0) {
    cpu->F = cpu->F | mask;
  } else {
    cpu->F = cpu->F & ~mask;
  }
}

void add16_half_carry(const uint8_t value, CPU *cpu) {
  uint8_t lower = value & 0xFF;
  uint8_t mask = 1 << 5; // at bit 7 (0 index)

  if (lower == 0xFF) {
    cpu->F = cpu->F | mask;
  } else {
    cpu->F = cpu->F & ~mask;
  }
}

void dec16_half_carry(const uint8_t value, CPU *cpu) {
  uint8_t ln = value & 0b111;
  uint16_t mask = 1 << 5;

  if (ln == 0x0) {
    cpu->F = cpu->F | mask;
  } else {
    cpu->F = cpu->F & ~mask;
  }
}
