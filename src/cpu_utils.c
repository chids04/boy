#include "cpu.h"
#include "gb_mem.h"

#include <stdio.h>
#include <stdint.h>

// CPU utils //

void set_flag(CPU* cpu, Flags flag) {
    cpu->F |= (uint8_t)flag;
}

void clear_flag(CPU *cpu, Flags flag) {
    cpu->F |= (uint8_t)~flag;
}

bool is_flag_set(CPU *cpu, Flags flag) {
    return (cpu->F & (uint8_t)flag) != 0;
}

bool condition(uint8_t idx, CPU *cpu){
    switch (idx){
        case 0:
        // NZ
        {
            if (!is_flag_set(cpu, FLAG_Z)) return true;
            break;
        }
        case 1:
        // Z
        {
            if (is_flag_set(cpu, FLAG_Z)) return true;
            break;
        }

        case 2:
        // NC
        {
            if (!is_flag_set(cpu, FLAG_C)) return true;
            break;
        }

        case 3:
        // C
        {
            if (is_flag_set(cpu, FLAG_C)) return true;
            break;
        }

    }

    return false;


}

uint8_t read_imm8(CPU *cpu){
    uint16_t data = read_byte(cpu->PC);
    cpu->PC += 1;

    return data;
}

uint16_t read_imm16(CPU *cpu){
    uint16_t data = read_word(cpu->PC);
    cpu->PC += 2;

    return data;
}

uint8_t read_r8(Reg8 reg, CPU *cpu) {
    switch (reg) {
        case REG_B:
            return cpu->B;
        case REG_C:
            return cpu->C;
        case REG_D:
            return cpu->D;
        case REG_E:
            return cpu->E;
        case REG_H:
            return cpu->H;
        case REG_L:
            return cpu->L;
        case REG_HL_8:
            return read_byte((cpu->H << 8) | cpu->L);
        case REG_A:
            return cpu->A;
    }

    printf("read_r8: invalid 8 bit register, returning 0\n");
    return 0x0;
}

uint16_t read_r16(Reg16 reg, CPU *cpu, bool has_sp) {
    switch (reg) {
        case REG_BC:
            return (cpu->B << 8) | cpu->C;

        case REG_DE:
            return (cpu->D << 8) | cpu->E;

        case REG_HL_16:
            return (cpu->H << 8) | cpu->L;

        case REG_AF:
            if (has_sp){
                return cpu->SP;
            }
            else{
                return (cpu->A << 8) | cpu->F;
            }
    }

    printf("read_r16: invalid register pair, returning 0\n");
    return 0x0;

}

void write_r16(uint16_t data, Reg16 reg, CPU *cpu){
    switch (reg) {
        case REG_BC:
            cpu->B = data >> 8;
            cpu->C = data & 0xFF;
            break;

        case REG_DE:
            cpu->D = data >> 8;
            cpu->E = data & 0xFF;
            break;

        case REG_HL_16:
            cpu->H = data >> 8;
            cpu->L = data & 0xFF;
            break;

        // todo: add support for AF reg too
        case REG_SP:
            cpu->SP = data;
            break;
    }
}

void write_r8(uint8_t data, Reg8 reg, CPU *cpu){
    switch (reg) {
        case REG_B:
            cpu->B = data;
            break;
        case REG_C:
            cpu->C = data;
            break;

        case REG_D:
            cpu->D = data;
            break;

        case REG_E:
            cpu->E = data;
            break;

        case REG_H:
            cpu->H = data;
            break;

        case REG_L:
            cpu->L = data;
            break;

        case REG_HL_8:
            {
                uint16_t addr = read_r16(REG_HL_16, cpu, false);
                write_byte(addr, data);

            }
            break;
        case REG_A:
            cpu->A = data;
    }
}

void inc8_half_carry(const uint8_t value, CPU *cpu) {
  uint8_t lower = value & 0b1111;
  uint8_t mask = 1 << 5; // at bit 5 (0 index)

  if (lower == 0b1111) {
      set_flag(cpu, FLAG_H);
  } else {
      clear_flag(cpu, FLAG_H);
  }
}

void dec8_half_carry(const uint8_t value, CPU *cpu) {
  uint8_t ln = value & 0xF;
  uint16_t mask = 1 << 5;

  if (ln == 0x0) {
    set_flag(cpu, FLAG_H);
  } else {
    clear_flag(cpu, FLAG_H);
  }
}

void inc16_half_carry(const uint8_t value, CPU *cpu) {
  uint8_t lower = value & 0xFF;
  uint8_t mask = 1 << 7; // at bit 7 (0 index)

  if (lower == 0xFF) {
    set_flag(cpu, FLAG_H);
  } else {
    clear_flag(cpu, FLAG_H);
  }
}

void dec16_half_carry(const uint8_t value, CPU *cpu) {
  uint8_t ln = value & 0b111;
  uint16_t mask = 1 << 7;

  if (ln == 0x0) {
    set_flag(cpu, FLAG_H);
  } else {
    clear_flag(cpu, FLAG_H);
  }
}

void add8_half_carry(const uint8_t value, CPU *cpu) {
    uint8_t lower_a = cpu->A & 0xF;
    uint8_t lower_r8 = value & 0xF;

    if((lower_a + lower_r8) > 0xF) {
        set_flag(cpu, FLAG_H);
    }
    else {
        clear_flag(cpu, FLAG_H);
    }
}

void add8_carry(const uint8_t value, CPU *cpu) {
    if((cpu->A + value) > 0xFF) {
        set_flag(cpu, FLAG_C);
    }
    else {
        clear_flag(cpu, FLAG_C);
    }
}

void sub8_half_carry(const uint8_t value, CPU *cpu){
  if((cpu->A & 0xF) < (value & 0xF)) {
    set_flag(cpu, FLAG_H);
  }
  else {
    clear_flag(cpu, FLAG_H);
  }

}

void sub8_carry(const uint8_t value, CPU *cpu){
  if(cpu->A < value) {
    set_flag(cpu, FLAG_C);
  }
  else {
    clear_flag(cpu, FLAG_C);

  }

}

