#include "boy.h"
#include "cpu.h"
#include "log.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

// CPU utils //

void set_flag(CPU *cpu, Flags flag) { cpu->F |= (uint8_t)flag; }

void clear_flag(CPU *cpu, Flags flag) { cpu->F &= (uint8_t)~flag; }

bool is_flag_set(CPU *cpu, Flags flag) { return (cpu->F & (uint8_t)flag) != 0; }

inline uint8_t get_lsb(uint16_t val) { return val & 0xFF; }

inline uint8_t get_msb(uint16_t val) { return (val >> 8) & 0xFF; }

bool condition(CPU *cpu, uint8_t idx) {
  switch (idx) {
  case 0:
    // NZ
    {
      if (!is_flag_set(cpu, FLAG_Z))
        return true;
      break;
    }
  case 1:
    // Z
    {
      if (is_flag_set(cpu, FLAG_Z))
        return true;
      break;
    }

  case 2:
    // NC
    {
      if (!is_flag_set(cpu, FLAG_C))
        return true;
      break;
    }

  case 3:
    // C
    {
      if (is_flag_set(cpu, FLAG_C))
        return true;
      break;
    }
  }

  return false;
}

uint8_t read_imm8(BOY *boy) {
  uint8_t byte = read_byte(boy, boy->cpu.PC++);

  return byte;
}

uint16_t read_imm16(BOY *boy) {
  uint8_t lsb = read_byte(boy, boy->cpu.PC++);
  uint8_t msb = read_byte(boy, boy->cpu.PC++);

  return (msb << 8) | lsb;
}

// if HL is accessed, then we read the byte at MEMORY[HL]:
uint8_t read_r8(BOY *boy, uint8_t reg) {
  switch (reg) {
  case 0:
    return boy->cpu.B;
  case 1:
    return boy->cpu.C;
  case 2:
    return boy->cpu.D;
  case 3:
    return boy->cpu.E;
  case 4:
    return boy->cpu.H;
  case 5:
    return boy->cpu.L;
  case 6: {
    uint16_t hl = read_r16(&boy->cpu, 2);
    return read_byte(boy, hl);
  }
  case 7:
    return boy->cpu.A;
  }

  printf("read_r8: invalid 8 bit register, returning 0\n");
  return 0x0;
}

void write_r8(BOY *boy, uint8_t data, uint8_t reg) {
  switch (reg) {
  case 0:
    boy->cpu.B = data;
    break;
  case 1:
    boy->cpu.C = data;
    break;

  case 2:
    boy->cpu.D = data;
    break;

  case 3:
    boy->cpu.E = data;
    break;

  case 4:
    boy->cpu.H = data;
    break;

  case 5:
    boy->cpu.L = data;
    break;

  case 6: {
    uint16_t address = read_r16(&boy->cpu, 2);
    write_byte(boy, address, data);
  } break;
  case 7:
    boy->cpu.A = data;
  }
}

void write_r16mem(BOY *boy, uint8_t reg, uint8_t data) {
  switch (reg) {
  case 0:
    write_byte(boy, (boy->cpu.B << 8) | (boy->cpu.C), data);
    return;
  case 1:
    write_byte(boy, (boy->cpu.D << 8) | (boy->cpu.E), data);
    return;
  case 2:
  case 3: {
    uint16_t hl = (boy->cpu.H << 8) | boy->cpu.L;
    write_byte(boy, hl, data);

    if (reg == 2) {
      hl += 1;
    } else {
      hl -= 1;
    }
    boy->cpu.H = hl >> 8;
    boy->cpu.L = hl & 0xFF;
    return;
  }
  }
}

uint8_t read_r16mem(BOY *boy, uint8_t reg) {
  switch (reg) {
  case 0:
    return read_byte(boy, (boy->cpu.B << 8) | boy->cpu.C);
  case 1:
    return read_byte(boy, (boy->cpu.D << 8) | boy->cpu.E);

  case 2:
  case 3: {
    uint16_t hl = (boy->cpu.H << 8) | boy->cpu.L;
    uint8_t data = read_byte(boy, hl);

    if (reg == 2) {
      hl += 1;
    } else {
      hl -= 1;
    }

    boy->cpu.H = hl >> 8;
    boy->cpu.L = hl & 0xFF;
    return data;
  }
  }

  log_error("invalid r16 register access");
  exit(1);
}

uint16_t read_r16(CPU *cpu, uint8_t reg) {
  switch (reg) {
  case 0:
    return (cpu->B << 8) | cpu->C;

  case 1:
    return (cpu->D << 8) | cpu->E;

  case 2:
    return (cpu->H << 8) | cpu->L;

  case 3:
    return cpu->SP;
  }

  printf("read_r16: invalid register pair, returning 0\n");
  return 0x0;
}

void write_r16(CPU *cpu, uint16_t data, uint8_t reg) {
  switch (reg) {
  case 0:
    cpu->B = data >> 8;
    cpu->C = data & 0xFF;
    break;

  case 1:
    cpu->D = data >> 8;
    cpu->E = data & 0xFF;
    break;

  case 2:
    cpu->H = data >> 8;
    cpu->L = data & 0xFF;
    break;

  case 3:
    cpu->SP = data;
  }
}

uint16_t read_r16stk(CPU *cpu, uint8_t reg) {
  switch (reg) {
  case 0:
    return (cpu->B << 8) | cpu->C;

  case 1:
    return (cpu->D << 8) | cpu->E;

  case 2:
    return (cpu->H << 8) | cpu->L;

  case 3:
    return (cpu->A << 8) | cpu->F;
  }

  printf("read_r16: invalid register pair, returning 0\n");
  return 0x0;
}

void write_r16stk(CPU *cpu, uint8_t reg, uint16_t data) {
  switch (reg) {
  case 0:
    cpu->B = data >> 8;
    cpu->C = data & 0xFF;
    break;

  case 1:
    cpu->D = data >> 8;
    cpu->E = data & 0xFF;
    break;

  case 2:
    cpu->H = data >> 8;
    cpu->L = data & 0xFF;
    break;

  case 3:
    cpu->A = data >> 8;
    cpu->F = data & 0xFF;
  }
}

// if HL is accessed, then we read the byte at MEMORY[HL]:

void inc8_half_carry(const uint8_t value, CPU *cpu) {
  uint8_t lower = value & 0b1111;

  if (lower == 0b1111) {
    set_flag(cpu, FLAG_H);
  } else {
    clear_flag(cpu, FLAG_H);
  }
}

void dec8_half_carry(const uint8_t value, CPU *cpu) {
  uint8_t ln = value & 0xF;

  if (ln == 0x0) {
    set_flag(cpu, FLAG_H);
  } else {
    clear_flag(cpu, FLAG_H);
  }
}

void inc16_half_carry(const uint8_t value, CPU *cpu) {
  uint8_t lower = value & 0xFF;

  if (lower == 0xFF) {
    set_flag(cpu, FLAG_H);
  } else {
    clear_flag(cpu, FLAG_H);
  }
}

void dec16_half_carry(const uint8_t value, CPU *cpu) {
  uint8_t ln = value & 0b111;

  if (ln == 0x0) {
    set_flag(cpu, FLAG_H);
  } else {
    clear_flag(cpu, FLAG_H);
  }
}

void add8_half_carry(const uint8_t value, CPU *cpu) {
  uint8_t lower_a = cpu->A & 0xF;
  uint8_t lower_r8 = value & 0xF;

  if ((lower_a + lower_r8) > 0xF) {
    set_flag(cpu, FLAG_H);
  } else {
    clear_flag(cpu, FLAG_H);
  }
}

void add8_carry(const uint8_t value, CPU *cpu) {
  if ((cpu->A + value) > 0xFF) {
    set_flag(cpu, FLAG_C);
  } else {
    clear_flag(cpu, FLAG_C);
  }
}

void sub8_half_carry(const uint8_t value, CPU *cpu) {
  if ((cpu->A & 0xF) < (value & 0xF)) {
    set_flag(cpu, FLAG_H);
  } else {
    clear_flag(cpu, FLAG_H);
  }
}

void sub8_carry(const uint8_t value, CPU *cpu) {
  if (cpu->A < value) {
    set_flag(cpu, FLAG_C);
  } else {
    clear_flag(cpu, FLAG_C);
  }
}

void half_carry8(CPU *cpu, uint8_t a, uint8_t b) {
  clear_flag(cpu, FLAG_H);

  uint8_t mask = 0xF;

  uint8_t a_half = a & mask;
  uint8_t b_half = b & mask;

  if (a_half + b_half > 0xF) {
    set_flag(cpu, FLAG_H);
  }
}

void carry8(CPU *cpu, uint8_t a, uint8_t b) {
  uint8_t mask = 0xFF;

  a = a & mask;
  b = b & mask;

  clear_flag(cpu, FLAG_C);

  if (a + b > 0xFF) {
    set_flag(cpu, FLAG_C);
  }
}
