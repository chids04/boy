#include "cpu.h"
#include "gb_mem.h"

#include <iso646.h>
#include <stdint.h>
#include <stdio.h>

void nop(CPU *cpu) {
  printf("NOP instruction not implemented\n");
  cpu->cycles = 1;
}

void stop(CPU *cpu) { printf("STOP instruction not implemented\n"); }

void ld_nn_sp(CPU *cpu) {
  // copy stack pointer to mem[nn], with nn being word after opcode
  uint16_t addr = read_imm16(cpu);
  write_word(addr, cpu->SP);

  cpu->cycles = 5;
}

void jr_d(CPU *cpu) {
  // add signed 8bit 'd' to next instruction,
  int8_t d = (int8_t)read_imm8(cpu);
  cpu->PC += d;

  cpu->cycles = 3;
}

void jr_cc_d(CPU *cpu) {

  uint8_t cond = get_y(cpu->opcode) - 4;

  printf("jr_cc_d: current condition %d\n", cond);

  if (condition(cpu, cond)) {
    int8_t d = displacement(cpu);
    cpu->PC += d;
    cpu->cycles = 3;
    printf("jr_cc_d: condition met, PC after displacement: %d\n", cpu->PC);
  }

  else {
    printf("jr_cc_d: condition not met\n");
    cpu->cycles = 2;
  }
}

void ld_r16_imm16(CPU *cpu) {

  uint16_t imm16 = read_imm16(cpu);

  // p is top 2 msb of y (bits 5 - 4 of opcode)
  uint8_t p = get_p(cpu->opcode);

  write_r16(cpu, imm16, p, false);

  cpu->cycles = 3;
}

void add_hl_r16(CPU *cpu) {
  // p is top 2 msb of y (bits 5 - 4 of opcode)
  uint8_t p = get_p(cpu->opcode);

  uint16_t data = read_r16(cpu, p, false);

  uint16_t hl = read_r16(cpu, REG_HL_16, false);
  write_r16(cpu, data + hl, 2, false);

  cpu->cycles = 2;
}

void ld_bc_a(CPU *cpu) {
  // mem[bc] = A

  uint16_t addr = read_r16(cpu, REG_BC, false);
  write_word(addr, cpu->A);

  cpu->cycles = 2;
}

void ld_hli_a(CPU *cpu) {
  // mem[hl] = A, HL--

  uint16_t addr = (cpu->H << 8) | cpu->L;
  write_word(addr, cpu->A);

  addr += 1;

  cpu->H = (addr >> 8) & 0xFF;
  cpu->L = addr & 0x0FF;

  cpu->cycles = 2;
}

void ld_de_a(CPU *cpu) {
  // mem[de] = A

  uint16_t addr = read_r16(cpu, REG_DE, false);
  write_word(addr, cpu->A);

  cpu->cycles = 2;
}

void ld_hld_a(CPU *cpu) {
  // mem[HL] = A, HL--
  uint16_t addr = read_r16(cpu, REG_HL_16, false);
  write_word(addr, cpu->A);
  addr -= 1;
  write_r16(cpu, addr, REG_HL_16, false);

  cpu->cycles = 2;
}

void ld_a_bc(CPU *cpu) {
  // a = memory[bc].

  uint8_t address = read_r16(cpu, REG_BC, false);
  uint8_t data = read_byte(address);
  cpu->A = data;

  cpu->cycles = 2;
}

void ld_a_hli(CPU *cpu) {
  // load a from memory location specified by hl, then increment hl.

  uint8_t address = read_r16(cpu, REG_BC, false);
  uint8_t data = read_byte(address);
  cpu->A = data;

  write_r16(cpu, address + 1, REG_BC, false);
}

void ld_a_de(CPU *cpu) {
  // load a from memory location specified by de.

  uint8_t address = read_r16(cpu, REG_DE, false);
  uint8_t data = read_byte(address);
  cpu->A = data;

  write_r16(cpu, address + 1, REG_DE, false);
}

void ld_a_hld(CPU *cpu) {
  // load from memory location specified by hl into A, then decrement hl.
  uint16_t address = read_r16(cpu, REG_HL_16, false);
  uint8_t data = read_byte(address);
  cpu->A = data;

  write_r8(cpu, address - 1, REG_A);
}

void inc_r8(CPU *cpu) {
  uint8_t y = get_y(cpu->opcode);

  uint8_t q = y >> 2;

  uint8_t data = read_r8(cpu, q);

  // first check for half carries
  inc8_half_carry(data, cpu);
  data += 1;

  // set zero and negate and write to registers

  if (data == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  clear_flag(cpu, FLAG_N);

  write_r8(cpu, data, q);

  cpu->cycles = 1;

  // Add extra cycle for HL register memory operation
  if (q == REG_HL_8) {
    cpu->cycles += 1;
  }
}
void dec_r8(CPU *cpu) {
  uint8_t y = get_y(cpu->opcode);

  uint8_t q = y >> 2;

  uint8_t data = read_r8(cpu, q);

  // first check for half carries
  dec8_half_carry(data, cpu);
  data -= 1;

  // set zero and negate and write to registers
  if (data == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  set_flag(cpu, FLAG_N);
  write_r8(cpu, data, q);

  cpu->cycles = 1;

  // Add extra cycle for HL register memory operation
  if (q == REG_HL_8) {
    cpu->cycles += 1;
  }
}

void inc_r16(CPU *cpu) {
  uint8_t p = get_p(cpu->opcode);

  uint16_t data = read_r16(cpu, p, false);

  write_r16(cpu, data += 1, p, false);
  cpu->cycles = 2;
}

void dec_r16(CPU *cpu) {
  uint8_t p = get_p(cpu->opcode);

  uint16_t data = read_r16(cpu, p, false);

  write_r16(cpu, data -= 1, p, false);
  cpu->cycles = 2;
}

void ld_r8_imm8(CPU *cpu) {
  // write the immediate next byte to 8 bit register
  uint8_t y = get_y(cpu->opcode);

  uint8_t imm8 = read_imm8(cpu);
  write_r8(cpu, imm8, y);

  cpu->cycles = 2;

  // Add extra cycle for HL register memory operation
  if (y == REG_HL_8) {
    cpu->cycles += 1;
  }
}

void rlca(CPU *cpu) {
  // set carry to msb and left circle rotate A register

  uint8_t msb = (cpu->A & (1 << 7)) >> 7;

  cpu->A = (cpu->A << 1) | msb;

  clear_flag(cpu, FLAG_Z);
  clear_flag(cpu, FLAG_H);
  clear_flag(cpu, FLAG_N);

  if (msb) {
    set_flag(cpu, FLAG_C);
  } else {
    clear_flag(cpu, FLAG_C);
  }

  cpu->cycles = 1;
}

void rrca(CPU *cpu) {
  // right rotate, setting carry and bit 0

  uint8_t lsb = cpu->A & 1;
  cpu->A = (cpu->A >> 1) | (lsb << 7);

  clear_flag(cpu, FLAG_Z);
  clear_flag(cpu, FLAG_H);
  clear_flag(cpu, FLAG_N);

  if (lsb) {
    set_flag(cpu, FLAG_C);
  } else {
    clear_flag(cpu, FLAG_C);
  }

  cpu->cycles = 1;
}

void rla(CPU *cpu) {
  // rotate left

  uint8_t msb = (cpu->A & (1 << 7)) >> 7;
  cpu->A <<= 1;

  if (is_flag_set(cpu, FLAG_C)) {
    cpu->A |= 1;
  } else {
    cpu->A &= ~1;
  }

  clear_flag(cpu, FLAG_Z);
  clear_flag(cpu, FLAG_H);
  clear_flag(cpu, FLAG_N);

  if (msb) {
    set_flag(cpu, FLAG_C);
  } else {
    clear_flag(cpu, FLAG_C);
  }

  cpu->cycles = 1;
}
void rra(CPU *cpu) {
  // rotate right
  uint8_t lsb = cpu->A & 1;
  cpu->A >>= 1;

  if (is_flag_set(cpu, FLAG_C)) {
    cpu->A |= 0x80;
  } else {
    cpu->A &= ~0x80;
  }

  clear_flag(cpu, FLAG_Z);
  clear_flag(cpu, FLAG_H);
  clear_flag(cpu, FLAG_N);

  if (lsb) {
    set_flag(cpu, FLAG_C);
  } else {
    clear_flag(cpu, FLAG_C);
  }

  cpu->cycles = 1;
}

void daa(CPU *cpu) {
  // adjusts addition after BCD addition

  uint8_t adjustment = 0;

  if (is_flag_set(cpu, FLAG_N)) {
    if (is_flag_set(cpu, FLAG_H)) {
      adjustment += 0x6;
    }

    if (is_flag_set(cpu, FLAG_C)) {
      adjustment += 0x60;
    }

    cpu->A -= adjustment;

  } else {

    if (is_flag_set(cpu, FLAG_H) || (cpu->A & 0xF) > 0x9) {
      adjustment += 0x6;
    }

    if (is_flag_set(cpu, FLAG_C) || cpu->A > 0x99) {
      adjustment += 0x60;
    }

    cpu->A += adjustment;
  }

  cpu->cycles = 1;
}

void cpl(CPU *cpu) {
  // sets A to its complement and sets flags
  cpu->A = ~cpu->A;

  set_flag(cpu, FLAG_H);
  set_flag(cpu, FLAG_N);

  cpu->cycles = 1;
}

void scf(CPU *cpu) {
  // sets the carry flag

  set_flag(cpu, FLAG_C);
  clear_flag(cpu, FLAG_N);
  clear_flag(cpu, FLAG_H);

  cpu->cycles = 1;
}

void ccf(CPU *cpu) {
  // set C flag to its complement

  if (is_flag_set(cpu, FLAG_C)) {
    clear_flag(cpu, FLAG_C);
  } else {
    set_flag(cpu, FLAG_C);
  }

  clear_flag(cpu, FLAG_N);
  clear_flag(cpu, FLAG_H);

  cpu->cycles = 1;
}

void ld_r8_r8(CPU *cpu) {
  // load val in right register into left
  uint8_t y = get_y(cpu->opcode);
  uint8_t z = get_z(cpu->opcode);

  uint8_t right_data = read_r8(cpu, z);
  write_r8(cpu, right_data, y);

  cpu->cycles = 1;

  // Add extra cycle for HL register memory operation
  if (y == REG_HL_8 || z == REG_HL_8) {
    cpu->cycles += 1;
  }
}

void halt(CPU *cpu) { printf("halt(): instruction not implemented\n"); }

void alu_a_r8(CPU *cpu) {
  switch (get_y(cpu->opcode)) {
  case 0:
    add_a_r8(cpu);
    return;
  case 1:
    adc_a_r8(cpu);
    return;
  case 2:
    sub_a_r8(cpu);
    return;
  case 3:
    subc_a_r8(cpu);
    return;
  case 4:
    and_a_r8(cpu);
    return;
  case 5:
    xor_a_r8(cpu);
    return;
  case 6:
    or_a_r8(cpu);
    return;
  case 7:
    cp_a_r8(cpu);
    return;
  }
}

void add_a_r8(CPU *cpu) {
  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_r8(cpu, z);

  // set flags
  add8_half_carry(data, cpu);
  add8_carry(data, cpu);
  clear_flag(cpu, FLAG_N);

  cpu->A += data;

  if (cpu->A == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  cpu->cycles = 1;

  // Add extra cycle for HL register memory operation
  if (z == REG_HL_8) {
    cpu->cycles += 1;
  }
}

void adc_a_r8(CPU *cpu) {
  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_r8(cpu, z);

  uint8_t carry = (uint8_t)is_flag_set(cpu, FLAG_C);

  // set flags
  add8_half_carry(data + carry, cpu);
  add8_carry(data + carry, cpu);
  clear_flag(cpu, FLAG_N);

  cpu->A += data + carry;

  if (cpu->A == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  cpu->cycles = 1;

  // Add extra cycle for HL register memory operation
  if (z == REG_HL_8) {
    cpu->cycles += 1;
  }
}

void sub_a_r8(CPU *cpu) {
  // A = A - r8

  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_r8(cpu, z);

  set_flag(cpu, FLAG_N);
  sub8_half_carry(data, cpu);
  sub8_carry(data, cpu);

  cpu->A -= data;

  if (cpu->A == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  cpu->cycles = 1;

  if (z == REG_HL_8)
    cpu->cycles += 1;
}

void subc_a_r8(CPU *cpu) {
  // A = A - r8 - carry

  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_r8(cpu, z);

  uint8_t carry = (uint8_t)is_flag_set(cpu, FLAG_C);
  data += carry;

  set_flag(cpu, FLAG_N);
  sub8_half_carry(data, cpu);
  sub8_carry(data, cpu);

  cpu->A -= data;

  if (cpu->A == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  cpu->cycles = 1;

  if (z == REG_HL_8)
    cpu->cycles += 1;
}

void and_a_r8(CPU *cpu) {

  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_r8(cpu, z);

  cpu->A &= data;

  clear_flag(cpu, FLAG_N);
  clear_flag(cpu, FLAG_C);
  set_flag(cpu, FLAG_H);

  if (cpu->A == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  cpu->cycles = 1;

  if (z == REG_HL_8)
    cpu->cycles += 1;
}

void xor_a_r8(CPU *cpu) {
  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_r8(cpu, z);

  cpu->A ^= data;
  clear_flag(cpu, FLAG_N);
  clear_flag(cpu, FLAG_H);
  clear_flag(cpu, FLAG_C);

  if (cpu->A == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  cpu->cycles = 1;

  if (z == REG_HL_8)
    cpu->cycles += 1;
}

void or_a_r8(CPU *cpu) {

  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_r8(cpu, z);

  cpu->A |= data;

  clear_flag(cpu, FLAG_N);
  clear_flag(cpu, FLAG_H);
  clear_flag(cpu, FLAG_C);

  if (cpu->A == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  cpu->cycles = 1;

  if (z == REG_HL_8)
    cpu->cycles += 1;
}

void cp_a_r8(CPU *cpu) {

  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_r8(cpu, z);

  set_flag(cpu, FLAG_N);
  sub8_half_carry(data, cpu);
  sub8_carry(data, cpu);

  uint8_t res = cpu->A - data;

  if (res == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  cpu->cycles = 1;

  if (z == REG_HL_8)
    cpu->cycles += 1;
}

void ret_cc(CPU *cpu) {
  uint8_t y = get_y(cpu->opcode);

  if (condition(cpu, y)) {
    uint16_t data = read_word(cpu->SP);
    cpu->SP += 2;

    cpu->PC = data;
    cpu->cycles = 5;
  } else {
    cpu->cycles = 2;
  }
}

void ld_0xFF00_n_A(CPU *cpu) {
  uint8_t n = read_imm8(cpu);
  uint16_t dest = 0xFF00 + n;

  write_byte(dest, cpu->A);
  cpu->cycles = 3;
}

void ld_A_0xFF00_n(CPU *cpu) {
  uint8_t n = read_imm8(cpu);
  uint16_t src = 0xFF00 + n;
  cpu->A = read_byte(src);
}

void add_sp_d(CPU *cpu) {
  int8_t d = displacement(cpu);
  uint8_t sp_lower = cpu->SP & 0xFF;

  half_carry8(cpu, sp_lower, d);
  carry8(cpu, sp_lower, d);

  clear_flag(cpu, FLAG_N);
  clear_flag(cpu, FLAG_Z);

  cpu->SP += d;
  cpu->cycles = 4;
}

void ld_hl_sp_d(CPU *cpu) {
  int8_t d = displacement(cpu);
  uint8_t sp_lower = cpu->SP & 0xFF;

  clear_flag(cpu, FLAG_Z);
  clear_flag(cpu, FLAG_N);
  half_carry8(cpu, sp_lower, d);
  carry8(cpu, sp_lower, d);

  write_r16(cpu, cpu->SP + d, REG_HL_16, false);

  cpu->cycles = 3;
}

void ret(CPU *cpu) {
  uint16_t data = read_word(cpu->SP);
  cpu->SP += 2;

  cpu->PC = data;
  cpu->cycles = 4;
}

void pop_r16(CPU *cpu) {
  uint16_t data = read_word(cpu->SP);
  cpu->SP += 2;

  uint8_t p = get_p(cpu->opcode);
  write_r16(cpu, data, p, true);

  cpu->cycles = 3;
}

void reti(CPU *cpu) {
  uint16_t data = read_word(cpu->SP);
  cpu->SP += 2;

  cpu->PC = data;
  cpu->enable_interrupts = true;

  cpu->cycles = 4;
}

void jp_hl(CPU *cpu) {
  cpu->PC = read_r16(cpu, REG_HL_16, false);
  cpu->cycles = 1;
}

void ld_sp_hl(CPU *cpu) {
  uint8_t hl = read_r16(cpu, REG_HL_16, false);
  write_r16(cpu, hl, REG_SP, false);
}

void jp_cc_nn(CPU *cpu) {
  uint8_t y = get_y(cpu->opcode);

  if (condition(cpu, y)) {
    cpu->PC = read_imm16(cpu);
    cpu->cycles = 4;
  } else {
    cpu->cycles = 3;
  }
}

void ld_0xFF00_C_A(CPU *cpu) {
  uint8_t dest = 0xFF00 + cpu->C;
  write_byte(dest, cpu->A);
  cpu->cycles = 2;
}

void ld_A_0xFF00_C(CPU *cpu) {
  uint8_t dest = 0xFF00 + cpu->C;
  cpu->A = read_byte(dest);
  cpu->cycles = 2;
}

void ld_nn_a(CPU *cpu) {
  uint8_t dest = read_imm16(cpu);
  write_byte(dest, cpu->A);
  cpu->cycles = 4;
}

void ld_a_nn(CPU *cpu) {
  uint8_t src = read_imm16(cpu);
  cpu->A = read_byte(src);
  cpu->cycles = 4;
}

void jp_nn(CPU *cpu) {
  cpu->PC = read_imm16(cpu);
  cpu->cycles = 4;
}

void ei(CPU *cpu) {
  cpu->interrupt_delay = true;
  cpu->cycles = 1;
}

void di(CPU *cpu) {
  cpu->interrupt_delay = false;
  cpu->ime = false;
}

void call_cc_nn(CPU *cpu) {
  uint8_t y = get_y(cpu->opcode);

  if (condition(cpu, y)) {
    uint16_t r16 = read_imm16(cpu);
    cpu->SP -= 2;
    write_word(cpu->SP, cpu->PC);

    // set pc
    cpu->PC = r16;

    cpu->cycles = 6;

  } else {
    cpu->cycles = 3;
  }
}

void alu_a_n(CPU *cpu) {
  switch (get_y(cpu->opcode)) {
  case 0:
    add_a_n(cpu);
    return;
  case 1:
    adc_a_n(cpu);
    return;
  case 2:
    sub_a_n(cpu);
    return;
  case 3:
    subc_a_n(cpu);
    return;
  case 4:
    and_a_n(cpu);
    return;
  case 5:
    xor_a_n(cpu);
    return;
  case 6:
    or_a_n(cpu);
    return;
  case 7:
    cp_a_n(cpu);
    return;
  }
}

void add_a_n(CPU *cpu) {
  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_imm8(cpu);

  // set flags
  add8_half_carry(data, cpu);
  add8_carry(data, cpu);
  clear_flag(cpu, FLAG_N);

  cpu->A += data;

  if (cpu->A == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  cpu->cycles = 2;
}

void adc_a_n(CPU *cpu) {
  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_imm8(cpu);

  uint8_t carry = (uint8_t)is_flag_set(cpu, FLAG_C);

  // set flags
  add8_half_carry(data + carry, cpu);
  add8_carry(data + carry, cpu);
  clear_flag(cpu, FLAG_N);

  cpu->A += data + carry;

  if (cpu->A == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  cpu->cycles = 2;
}

void sub_a_n(CPU *cpu) {
  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_imm8(cpu);

  set_flag(cpu, FLAG_N);
  sub8_half_carry(data, cpu);
  sub8_carry(data, cpu);

  cpu->A -= data;

  if (cpu->A == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  cpu->cycles = 2;
}

void subc_a_n(CPU *cpu) {
  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_imm8(cpu);

  uint8_t carry = (uint8_t)is_flag_set(cpu, FLAG_C);
  data += carry;

  set_flag(cpu, FLAG_N);
  sub8_half_carry(data, cpu);
  sub8_carry(data, cpu);

  cpu->A -= data;

  if (cpu->A == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  cpu->cycles = 2;
}

void and_a_n(CPU *cpu) {
  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_imm8(cpu);

  cpu->A &= data;

  clear_flag(cpu, FLAG_N);
  clear_flag(cpu, FLAG_C);
  set_flag(cpu, FLAG_H);

  if (cpu->A == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  cpu->cycles = 1;
}

void xor_a_n(CPU *cpu) {

  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_imm8(cpu);

  cpu->A ^= data;
  clear_flag(cpu, FLAG_N);
  clear_flag(cpu, FLAG_H);
  clear_flag(cpu, FLAG_C);

  if (cpu->A == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  cpu->cycles = 2;
}

void or_a_n(CPU *cpu) {
  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_imm8(cpu);

  cpu->A |= data;

  clear_flag(cpu, FLAG_N);
  clear_flag(cpu, FLAG_H);
  clear_flag(cpu, FLAG_C);

  if (cpu->A == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  cpu->cycles = 2;
}

void cp_a_n(CPU *cpu) {
  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_imm8(cpu);

  set_flag(cpu, FLAG_N);
  sub8_half_carry(data, cpu);
  sub8_carry(data, cpu);

  uint8_t res = cpu->A - data;

  if (res == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  cpu->cycles = 2;
}

void call_nn(CPU *cpu) {
  // pc already incremented at the beginning of opcode handler
  uint16_t r16 = read_imm16(cpu);

  // pc will be holding the address of instruction to save

  // push this to the stack
  cpu->SP -= 2;
  write_word(cpu->SP, cpu->PC);

  // set pc
  cpu->PC = r16;

  cpu->cycles = 6;
}

void push_r16(CPU *cpu) {
  // pushes value from r16 reg onto the stack
  // decr sp read
  uint8_t p = get_p(cpu->opcode);
  uint16_t data = read_r16(cpu, p, true);

  cpu->SP -= 2;
  write_word(cpu->SP, data);

  cpu->cycles = 4;
}

void rst(CPU *cpu) {

  // no op cus of rst bug in tetris lol

  // uint16_t addr = get_y(cpu->opcode) * 8;

  // cpu->SP -= 2;
  // write_word(cpu->SP, addr);
  // cpu->PC = addr;

  // cpu->cycles = 4;
}

void rot(CPU *cpu) {
  switch (get_y(cpu->opcode)) {
  case 0:
    rlc_r8(cpu);
    break;
  case 1:
    rrc_r8(cpu);
    break;
  case 2:
    rl_r8(cpu);
    break;

  case 3:
    rr_r8(cpu);
    break;

  case 4:
    sla_r8(cpu);
    break;

  case 5:
    sra_r8(cpu);
    break;

  case 6:
    swap_r8(cpu);
    break;

  case 7:
    srl_r8(cpu);
    break;
  }
}

void rlc_r8(CPU *cpu) {
  uint8_t z = get_z(cpu->opcode);

  uint8_t data = read_r8(cpu, z);
  uint8_t msb = (data & 0x80) >> 7;

  uint8_t rl = (data << 1) | msb;
  write_r8(cpu, rl, z);

  if (rl == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  clear_flag(cpu, FLAG_H);
  clear_flag(cpu, FLAG_N);

  if (msb) {
    set_flag(cpu, FLAG_C);
  } else {
    clear_flag(cpu, FLAG_C);
  }

  if (z == REG_HL_8) {
    cpu->cycles = 4;
  } else {
    cpu->cycles = 2;
  }
}

void rrc_r8(CPU *cpu) {
  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_r8(cpu, z);

  uint8_t lsb = data & 1;
  uint8_t rrc = (data >> 1) | (lsb << 7);
  write_r8(cpu, rrc, z);

  if (rrc == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  clear_flag(cpu, FLAG_H);
  clear_flag(cpu, FLAG_N);

  if (lsb) {
    set_flag(cpu, FLAG_C);
  } else {
    clear_flag(cpu, FLAG_C);
  }

  if (z == REG_HL_8) {
    cpu->cycles = 4;
  } else {
    cpu->cycles = 2;
  }
}

void rl_r8(CPU *cpu) {
  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_r8(cpu, z);

  uint8_t msb = (data & (1 << 7)) >> 7;
  data <<= 1;

  if (is_flag_set(cpu, FLAG_C)) {
    data |= 1;
  } else {
    data &= ~1;
  }

  write_r8(cpu, data, z);

  if (data == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  clear_flag(cpu, FLAG_H);
  clear_flag(cpu, FLAG_N);

  if (msb) {
    set_flag(cpu, FLAG_C);
  } else {
    clear_flag(cpu, FLAG_C);
  }

  if (z == REG_HL_8) {
    cpu->cycles = 4;
  } else {
    cpu->cycles = 2;
  }
}

void rr_r8(CPU *cpu) {
  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_r8(cpu, z);

  uint8_t lsb = data & 1;

  data >>= 1;

  if (is_flag_set(cpu, FLAG_C)) {
    data |= 0x80;
  } else {
    data &= ~0x80;
  }

  write_r8(cpu, data, z);

  if (data == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  clear_flag(cpu, FLAG_H);
  clear_flag(cpu, FLAG_N);

  if (lsb) {
    set_flag(cpu, FLAG_C);
  } else {
    clear_flag(cpu, FLAG_C);
  }

  if (z == REG_HL_8) {
    cpu->cycles = 4;
  } else {
    cpu->cycles = 2;
  }
}

void sla_r8(CPU *cpu) {
  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_r8(cpu, z);

  uint8_t msb = (data & 0x80) >> 7;
  data <<= 1;

  write_r8(cpu, data, z);

  if (data == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  clear_flag(cpu, FLAG_H);
  clear_flag(cpu, FLAG_N);

  if (msb) {
    set_flag(cpu, FLAG_C);
  } else {
    clear_flag(cpu, FLAG_C);
  }

  if (z == REG_HL_8) {
    cpu->cycles = 4;
  } else {
    cpu->cycles = 2;
  }
}

void sra_r8(CPU *cpu) {
  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_r8(cpu, z);

  uint8_t lsb = data & 1;
  uint8_t msb = data & 0x80;

  data = (data >> 1) | msb;

  write_r8(cpu, data, z);

  if (data == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  clear_flag(cpu, FLAG_H);
  clear_flag(cpu, FLAG_N);

  if (lsb) {
    set_flag(cpu, FLAG_C);
  } else {
    clear_flag(cpu, FLAG_C);
  }

  if (z == REG_HL_8) {
    cpu->cycles = 4;
  } else {
    cpu->cycles = 2;
  }
}

void swap_r8(CPU *cpu) {
  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_r8(cpu, z);

  data = (data >> 4) | (data << 4);

  write_r8(cpu, data, z);

  if (data == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  clear_flag(cpu, FLAG_H);
  clear_flag(cpu, FLAG_N);
  clear_flag(cpu, FLAG_C);

  if (z == REG_HL_8) {
    cpu->cycles = 4;
  } else {
    cpu->cycles = 2;
  }
}

void srl_r8(CPU *cpu) {
  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_r8(cpu, z);

  uint8_t lsb = data & 1;

  data >>= 1;

  write_r8(cpu, data, z);

  if (data == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  clear_flag(cpu, FLAG_H);
  clear_flag(cpu, FLAG_N);

  if (lsb) {
    set_flag(cpu, FLAG_C);
  } else {
    clear_flag(cpu, FLAG_C);
  }

  if (z == REG_HL_8) {
    cpu->cycles = 4;
  } else {
    cpu->cycles = 2;
  }
}

void bit_y_r8(CPU *cpu) {
  // check if bit y is set
  // shift y times then mask off all upper 7 bits

  uint8_t y = get_y(cpu->opcode);
  uint8_t z = get_z(cpu->opcode);

  uint8_t bit_y = (read_r8(cpu, z) >> y) & 1;

  if (bit_y == 0) {
    set_flag(cpu, FLAG_Z);
  } else {
    clear_flag(cpu, FLAG_Z);
  }

  clear_flag(cpu, FLAG_N);
  set_flag(cpu, FLAG_H);

  if (z == REG_HL_8) {
    cpu->cycles = 3;
  } else {
    cpu->cycles = 2;
  }
}

void res_y_r8(CPU *cpu) {
  uint8_t mask = ~(1 << get_y(cpu->opcode));

  uint8_t z = get_z(cpu->opcode);
  uint8_t old = read_r8(cpu, z);

  uint8_t new = old | mask;

  write_r8(cpu, new, z);

  if (z == REG_HL_8) {
    cpu->cycles = 4;
  } else {
    cpu->cycles = 2;
  }
}

void set_y_r8(CPU *cpu) {
  uint8_t mask = (1 << get_y(cpu->opcode));

  uint8_t z = get_z(cpu->opcode);
  uint8_t old = read_r8(cpu, z);

  uint8_t new = old | mask;

  write_r8(cpu, new, z);

  if (z == REG_HL_8) {
    cpu->cycles = 4;
  } else {
    cpu->cycles = 2;
  }
}

// CPU UTILS BEGINbibin/
//
//
//

void decode_instruction(CPU *cpu) {

  if (cpu->interrupt_delay) {
    cpu->interrupt_delay = false;
    cpu->enable_interrupts = true;
  } else if (cpu->enable_interrupts) {
    cpu->ime = true;
  }

  cpu->opcode = read_byte(cpu->PC++);

  // check for prefix byte first

  switch (cpu->opcode) {
  case 0xCB:
    cpu->opcode = read_byte(cpu->PC++);
    uint8_t x = get_x(cpu->opcode);

    switch (x) {
    case 0:
      rot(cpu);
      return;
    case 1:
      bit_y_r8(cpu);
      return;
    case 2:
      res_y_r8(cpu);
      break;
    case 3:
      set_y_r8(cpu);
      return;
      break;
    }

  case 0xDD:
    cpu->opcode = read_byte(cpu->PC++);
    return;
  case 0xED:
    cpu->opcode = read_byte(cpu->PC++);
    return;
  case 0xFD:
    cpu->opcode = read_byte(cpu->PC++);
    return;
  }

  uint8_t x = get_x(cpu->opcode);

  switch (x) {
    uint8_t z = get_z(cpu->opcode);
    uint8_t y = get_y(cpu->opcode);
    uint8_t q = get_q(cpu->opcode);
    uint8_t p = get_p(cpu->opcode);

    // x = 0

  case 0: {

    switch (z) {
    // z = 0
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

      // z = 1

    case 1: {
      switch (q) {
      case 0:
        ld_r16_imm16(cpu);
        return;
      case 1:
        add_hl_r16(cpu);
        return;
      }

      break;
    }

      // z = 2

    case 2: {
      switch (q) {
      case 0:
        switch (p) {
        case 0:
          ld_bc_a(cpu);
          return;

        case 1:
          ld_de_a(cpu);
          return;

        case 2:
          ld_hli_a(cpu);
          return;

        case 3:
          ld_hld_a(cpu);
          return;
        }

        break;

      case 1:
        switch (p) {
        case 0:
          ld_a_bc(cpu);
          return;

        case 1:
          ld_a_hli(cpu);
          return;

        case 2:
          ld_a_de(cpu);
          return;

        case 3:
          ld_a_hld(cpu);
          return;
        }

        break;
      }
    }
      // z = 3

    case 3: {
      switch (q) {
      case 0:
        inc_r16(cpu);
        return;
      case 1:
        dec_r16(cpu);
        return;
      }
      break;
    }

    // z = 4
    case 4: {
      inc_r8(cpu);
      return;
    }

    // z = 5
    case 5: {
      dec_r8(cpu);
      return;
    }

    // z = 6
    case 6: {
      ld_r8_imm8(cpu);
      return;
    }

    // z = 7
    case 7: {
      switch (y) {
      case 0:
        rlca(cpu);
        return;
      case 1:
        rrca(cpu);
        return;
      case 2:
        rla(cpu);
        return;
      case 3:
        rra(cpu);
        return;
      case 4:
        daa(cpu);
        return;
      case 5:
        cpl(cpu);
        return;
      case 6:
        scf(cpu);
        return;
      case 7:
        ccf(cpu);
        return;
      }
    }
    }
  }

  // x = 1
  case 1: {
    switch (z) {
      switch (y) {
      case 6:
        halt(cpu);
        return;
      }
    }
    ld_r8_r8(cpu);
    return;
  }

  // x = 2
  case 2: {
    alu_a_r8(cpu);
    return;
  }

  // x = 3
  case 3: {
    switch (z) {

    // z = 0
    case 0: {
      switch (y) {
      case 0 ... 3:
        ret_cc(cpu);
        return;
      case 4:
        ld_0xFF00_n_A(cpu);
        return;
      case 5:
        add_sp_d(cpu);
        return;
      case 6:
        ld_A_0xFF00_n(cpu);
        return;
      case 7:
        ld_hl_sp_d(cpu);
        return;
      }
      break;
    }

    // z = 1
    case 1: {
      switch (q) {
      case 0:
        pop_r16(cpu);
        return;
      case 1: {
        switch (p) {
        case 0:
          ret(cpu);
          return;
        case 1:
          reti(cpu);
          return;
        case 2:
          jp_hl(cpu);
        case 3:
          ld_sp_hl(cpu);
          return;
        }
      }
      }
      break;
    }

    // z = 2
    case 2: {
      switch (y) {
      case 0 ... 3:
        jp_cc_nn(cpu);
        return;

      case 4:
        ld_0xFF00_C_A(cpu);
        return;

      case 5:
        ld_nn_a(cpu);
        return;

      case 6:
        ld_A_0xFF00_C(cpu);
        return;

      case 7:
        ld_a_nn(cpu);
        return;
      }
      break;
    }

    // z = 3
    case 3: {
      switch (y) {
      case 0:
        jp_nn(cpu);
        return;

      case 6:
        di(cpu);
        return;

      case 7:
        ei(cpu);
        return;
      }
      break;
    }

    // z = 4
    case 4: {
      switch (y) {
      case 0 ... 3:
        call_cc_nn(cpu);
        return;
      }
      break;
    }

    // z = 5
    case 5: {
      switch (q) {
      case 0:
        push_r16(cpu);
        return;

      case 1: {
        switch (p) {
        case 0:
          call_nn(cpu);
          return;
        }
        break;
      }
      }
      break;
    }

    // z = 6
    case 6: {
      alu_a_n(cpu);
      return;
    }

    case 7: {
      rst(cpu);
      return;
    }
    }
  }
  }
}
