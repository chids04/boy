#include "cpu.h"
#include "boy.h"
#include "log.h"
#include "mmu.h"

#include <iso646.h>
#include <stdint.h>
#include <stdio.h>

void init_cpu(CPU *cpu, uint8_t header_checksum) {
  cpu->A = 0x01;
  if (header_checksum == 0x00) {
    cpu->F = 0x0;
    set_flag(cpu, FLAG_Z);
  } else {
    cpu->F = 0x0;
    set_flag(cpu, FLAG_Z);
    set_flag(cpu, FLAG_H);
    set_flag(cpu, FLAG_C);
  }

  cpu->B = 0x00;
  cpu->C = 0x13;
  cpu->D = 0x00;
  cpu->E = 0xD8;
  cpu->H = 0x01;
  cpu->L = 0x4D;
  cpu->PC = 0x100;
  cpu->SP = 0xFFFE;
}

void nop(BOY *boy) {
  log_debug("Executing %s", __func__);
  log_warn("NOP instruction not implemented");
  boy->cpu.cycles = 1;
}

void stop(BOY *boy) {
  log_debug("Executing %s", __func__);
  log_warn("STOP instruction not implemented\n");
}

void ld_nn_sp(BOY *boy) {
  log_debug("Executing %s", __func__);
  // copy stack pointer to mem[nn], with nn being word after opcode

  // 2 cycles to read the 16 bit memory address
  uint8_t lsb = read_byte(&boy->mmu, boy->cpu.PC);
  uint8_t msb = read_byte(&boy->mmu, ++boy->cpu.PC);

  uint16_t addr = (msb << 8) | lsb;

  // 2 cycles to write the 16 bit value to memory
  write_byte(&boy->mmu, addr, boy->cpu.SP & 0xFF);
  write_byte(&boy->mmu, addr + 1, (boy->cpu.SP >> 8) & 0xFF);

  boy->cpu.cycles = 5;
}

void jr_d(BOY *boy) {
  log_debug("Executing %s", __func__);
  // add signed 8bit 'd' to next instruction,

  int8_t d = (int8_t)read_byte(&boy->mmu, boy->cpu.PC++);

  boy->cpu.PC += d;

  tick(boy, 1);

  boy->cpu.cycles = 3;
}

void jr_cc_d(BOY *boy) {
  log_debug("Executing %s", __func__);

  uint8_t cond = get_y(boy->cpu.opcode) - 4;

  printf("jr_cc_d: current condition %d\n", cond);
  int8_t d = (int8_t)read_byte(&boy->mmu, boy->cpu.PC++);

  if (condition(&boy->cpu, cond)) {
    boy->cpu.PC += d;
    tick(boy, 1);
    boy->cpu.cycles = 3;
  }

  else {
    printf("jr_cc_d: condition not met\n");
    boy->cpu.cycles = 2;
  }
}

void ld_r16_imm16(BOY *boy) {
  log_debug("Executing %s", __func__);

  uint16_t imm16 = read_imm16(boy);

  // p is top 2 msb of y (bits 5 - 4 of opcode)
  uint8_t p = get_p(boy->cpu.opcode);

  write_r16(&boy->cpu, imm16, p, false);

  boy->cpu.cycles = 3;
}

void add_hl_r16(BOY *boy) {
  log_debug("Executing %s", __func__);
  // p is top 2 msb of y (bits 5 - 4 of opcode)
  uint8_t p = get_p(boy->cpu.opcode);

  uint16_t data = read_r16(&boy->cpu, p, false);

  uint16_t hl = read_r16(&boy->cpu, REG_HL_16, false);
  write_r16(&boy->cpu, data + hl, REG_HL_16, false);

  tick(boy, 1);

  boy->cpu.cycles = 2;
}

void ld_bc_a(BOY *boy) {
  log_debug("Executing %s", __func__);
  // mem[bc] = A

  uint16_t bc = read_r16(&boy->cpu, REG_BC, false);
  write_byte(&boy->mmu, bc, boy->cpu.A);

  boy->cpu.cycles = 2;
}

void ld_hli_a(BOY *boy) {
  log_debug("Executing %s", __func__);
  // mem[hl] = A, HL++

  uint16_t hl = read_r16(&boy->cpu, REG_HL_16, false);
  write_byte(&boy->mmu, hl, boy->cpu.A);
  write_r16(&boy->cpu, hl + 1, REG_HL_16, false);

  boy->cpu.cycles = 2;
}

void ld_de_a(BOY *boy) {
  log_debug("Executing %s", __func__);
  // mem[de] = A

  uint16_t de = read_r16(&boy->cpu, REG_DE, false);
  write_r16(&boy->cpu, de, REG_DE, false);

  boy->cpu.cycles = 2;
}

void ld_hld_a(BOY *boy) {
  log_debug("Executing %s", __func__);
  // mem[HL] = A, HL--
  //
  uint16_t hl = read_r16(&boy->cpu, REG_HL_16, false);
  write_byte(&boy->mmu, hl, boy->cpu.A);
  write_r16(&boy->cpu, hl - 1, REG_HL_16, false);

  boy->cpu.cycles = 2;
}

void ld_a_bc(BOY *boy) {
  log_debug("Executing %s", __func__);
  // a = memory[bc].

  uint16_t bc = read_r16(&boy->cpu, REG_BC, false);
  uint8_t data = read_byte(&boy->mmu, bc);

  boy->cpu.A = data;
  boy->cpu.cycles = 2;
}

void ld_a_hli(BOY *boy) {
  log_debug("Executing %s", __func__);
  // load a from memory location specified by hl, then increment hl.

  uint16_t hl = read_r16(&boy->cpu, REG_HL_16, false);
  uint8_t data = read_byte(&boy->mmu, hl);

  boy->cpu.A = data;

  write_r16(&boy->cpu, hl + 1, REG_HL_16, false);
}

void ld_a_hld(BOY *boy) {
  log_debug("Executing %s", __func__);

  uint16_t hl = read_r16(&boy->cpu, REG_HL_16, false);
  uint8_t data = read_byte(&boy->mmu, hl);

  boy->cpu.A = data;

  write_r16(&boy->cpu, hl - 1, REG_HL_16, false);
}

void ld_a_de(BOY *boy) {
  log_debug("Executing %s", __func__);
  // load a from memory location pointed to by de.
  //
  uint16_t de = read_r16(&boy->cpu, REG_DE, false);
  uint8_t data = read_byte(&boy->mmu, de);

  boy->cpu.A = data;
  boy->cpu.cycles = 2;
}

void inc_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t y = get_y(boy->cpu.opcode);
  uint8_t q = y >> 2;

  uint8_t data = read_r8(boy, q);

  // first check for half carries
  inc8_half_carry(data, &boy->cpu);
  data += 1;

  // set zero and negate and write to registers

  if (data == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  clear_flag(&boy->cpu, FLAG_N);
  write_r8(boy, data, q);

  boy->cpu.cycles = 1;

  // Add extra cycle for HL register memory operation
  if (q == REG_HL_8) {
    boy->cpu.cycles += 1;
  }
}
void dec_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t y = get_y(boy->cpu.opcode);

  uint8_t q = y >> 2;

  uint8_t data = read_r8(boy, q);

  // first check for half carries
  dec8_half_carry(data, &boy->cpu);
  data -= 1;

  // set zero and negate and write to registers
  if (data == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  set_flag(&boy->cpu, FLAG_N);
  write_r8(boy, data, q);

  boy->cpu.cycles = 1;

  // add extra cycle for HL register memory operation
  if (q == REG_HL_8) {
    boy->cpu.cycles = 3;
  }
}

void inc_r16(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t p = get_p(boy->cpu.opcode);

  uint16_t data = read_r16(&boy->cpu, p, false);
  write_r16(&boy->cpu, data += 1, p, false);

  tick(boy, 1);

  boy->cpu.cycles = 2;
}

void dec_r16(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t p = get_p(boy->cpu.opcode);
  uint16_t data = read_r16(&boy->cpu, p, false);
  write_r16(&boy->cpu, data - 1, p, false);
  tick(boy, 1);

  boy->cpu.cycles = 2;
}

void ld_r8_imm8(BOY *boy) {
  log_debug("Executing %s", __func__);
  // write the immediate next byte to 8 bit register
  uint8_t y = get_y(boy->cpu.opcode);

  uint8_t imm8 = read_imm8(boy);
  write_r8(boy, imm8, y);

  boy->cpu.cycles = 2;

  // Add extra cycle for HL register memory operation
  if (y == REG_HL_8) {
    boy->cpu.cycles += 1;
  }
}

void rlca(BOY *boy) {
  log_debug("Executing %s", __func__);
  // set carry to msb and left circle rotate A register

  uint8_t msb = (boy->cpu.A & (1 << 7)) >> 7;

  boy->cpu.A = (boy->cpu.A << 1) | msb;

  clear_flag(&boy->cpu, FLAG_Z);
  clear_flag(&boy->cpu, FLAG_H);
  clear_flag(&boy->cpu, FLAG_N);

  if (msb) {
    set_flag(&boy->cpu, FLAG_C);
  } else {
    clear_flag(&boy->cpu, FLAG_C);
  }

  boy->cpu.cycles = 1;
}

void rrca(BOY *boy) {
  log_debug("Executing %s", __func__);
  // right rotate, setting carry and bit 0

  uint8_t lsb = boy->cpu.A & 1;
  boy->cpu.A = (boy->cpu.A >> 1) | (lsb << 7);

  clear_flag(&boy->cpu, FLAG_Z);
  clear_flag(&boy->cpu, FLAG_H);
  clear_flag(&boy->cpu, FLAG_N);

  if (lsb) {
    set_flag(&boy->cpu, FLAG_C);
  } else {
    clear_flag(&boy->cpu, FLAG_C);
  }

  boy->cpu.cycles = 1;
}

void rla(BOY *boy) {
  log_debug("Executing %s", __func__);
  // rotate left

  uint8_t msb = (boy->cpu.A & (1 << 7)) >> 7;
  boy->cpu.A <<= 1;

  if (is_flag_set(&boy->cpu, FLAG_C)) {
    boy->cpu.A |= 1;
  } else {
    boy->cpu.A &= ~1;
  }

  clear_flag(&boy->cpu, FLAG_Z);
  clear_flag(&boy->cpu, FLAG_H);
  clear_flag(&boy->cpu, FLAG_N);

  if (msb) {
    set_flag(&boy->cpu, FLAG_C);
  } else {
    clear_flag(&boy->cpu, FLAG_C);
  }

  boy->cpu.cycles = 1;
}
void rra(BOY *boy) {
  log_debug("Executing %s", __func__);
  // rotate right
  uint8_t lsb = boy->cpu.A & 1;
  boy->cpu.A >>= 1;

  if (is_flag_set(&boy->cpu, FLAG_C)) {
    boy->cpu.A |= 0x80;
  } else {
    boy->cpu.A &= ~0x80;
  }

  clear_flag(&boy->cpu, FLAG_Z);
  clear_flag(&boy->cpu, FLAG_H);
  clear_flag(&boy->cpu, FLAG_N);

  if (lsb) {
    set_flag(&boy->cpu, FLAG_C);
  } else {
    clear_flag(&boy->cpu, FLAG_C);
  }

  boy->cpu.cycles = 1;
}

void daa(BOY *boy) {
  log_debug("Executing %s", __func__);
  // adjusts addition after BCD addition

  uint8_t adjustment = 0;

  if (is_flag_set(&boy->cpu, FLAG_N)) {
    if (is_flag_set(&boy->cpu, FLAG_H)) {
      adjustment += 0x6;
    }

    if (is_flag_set(&boy->cpu, FLAG_C)) {
      adjustment += 0x60;
    }

    boy->cpu.A -= adjustment;

  } else {

    if (is_flag_set(&boy->cpu, FLAG_H) || (boy->cpu.A & 0xF) > 0x9) {
      adjustment += 0x6;
    }

    if (is_flag_set(&boy->cpu, FLAG_C) || boy->cpu.A > 0x99) {
      adjustment += 0x60;
    }

    boy->cpu.A += adjustment;
  }

  boy->cpu.cycles = 1;
}

void cpl(BOY *boy) {
  log_debug("Executing %s", __func__);
  // sets A to its complement and sets flags
  boy->cpu.A = ~boy->cpu.A;

  set_flag(&boy->cpu, FLAG_H);
  set_flag(&boy->cpu, FLAG_N);

  boy->cpu.cycles = 1;
}

void scf(BOY *boy) {
  log_debug("Executing %s", __func__);
  // sets the carry flag

  set_flag(&boy->cpu, FLAG_C);
  clear_flag(&boy->cpu, FLAG_N);
  clear_flag(&boy->cpu, FLAG_H);

  boy->cpu.cycles = 1;
}

void ccf(BOY *boy) {
  log_debug("Executing %s", __func__);
  // set C flag to its complement

  if (is_flag_set(&boy->cpu, FLAG_C)) {
    clear_flag(&boy->cpu, FLAG_C);
  } else {
    set_flag(&boy->cpu, FLAG_C);
  }

  clear_flag(&boy->cpu, FLAG_N);
  clear_flag(&boy->cpu, FLAG_H);

  boy->cpu.cycles = 1;
}

void ld_r8_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  // load val in right register into left
  uint8_t y = get_y(boy->cpu.opcode);
  uint8_t z = get_z(boy->cpu.opcode);

  uint8_t right_data = read_r8(boy, z);
  write_r8(boy, right_data, y);

  boy->cpu.cycles = 1;

  // Add extra cycle for HL register memory operation
  if (y == REG_HL_8 || z == REG_HL_8) {
    boy->cpu.cycles += 1;
  }
}

void halt(BOY *boy) {
  log_debug("Executing %s", __func__);
  printf("halt(): instruction not implemented\n");
}

void alu_a_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  switch (get_y(boy->cpu.opcode)) {
  case 0:
    add_a_r8(boy);
    return;
  case 1:
    adc_a_r8(boy);
    return;
  case 2:
    sub_a_r8(boy);
    return;
  case 3:
    subc_a_r8(boy);
    return;
  case 4:
    and_a_r8(boy);
    return;
  case 5:
    xor_a_r8(boy);
    return;
  case 6:
    or_a_r8(boy);
    return;
  case 7:
    cp_a_r8(boy);
    return;
  }
}

void add_a_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t data = read_r8(boy, z);

  // set flags
  add8_half_carry(data, &boy->cpu);
  add8_carry(data, &boy->cpu);
  clear_flag(&boy->cpu, FLAG_N);

  boy->cpu.A += data;

  if (boy->cpu.A == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  boy->cpu.cycles = 1;

  // Add extra cycle for HL register memory operation
  if (z == REG_HL_8) {
    boy->cpu.cycles += 1;
  }
}

void adc_a_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t data = read_r8(boy, z);

  uint8_t carry = (uint8_t)is_flag_set(&boy->cpu, FLAG_C);

  // set flags
  add8_half_carry(data + carry, &boy->cpu);
  add8_carry(data + carry, &boy->cpu);
  clear_flag(&boy->cpu, FLAG_N);

  boy->cpu.A += data + carry;

  if (boy->cpu.A == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  boy->cpu.cycles = 1;

  // Add extra cycle for HL register memory operation
  if (z == REG_HL_8) {
    boy->cpu.cycles += 1;
  }
}

void sub_a_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  // A = A - r8

  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t data = read_r8(boy, z);

  set_flag(&boy->cpu, FLAG_N);
  sub8_half_carry(data, &boy->cpu);
  sub8_half_carry(data, &boy->cpu);
  sub8_carry(data, &boy->cpu);

  boy->cpu.A -= data;

  if (boy->cpu.A == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  boy->cpu.cycles = 1;

  if (z == REG_HL_8)
    boy->cpu.cycles += 1;
}

void subc_a_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  // A = A - r8 - carry

  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t data = read_r8(boy, z);

  uint8_t carry = (uint8_t)is_flag_set(&boy->cpu, FLAG_C);
  data += carry;

  set_flag(&boy->cpu, FLAG_N);
  sub8_half_carry(data, &boy->cpu);
  sub8_carry(data, &boy->cpu);

  boy->cpu.A -= data;

  if (boy->cpu.A == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  boy->cpu.cycles = 1;

  if (z == REG_HL_8)
    boy->cpu.cycles += 1;
}

void and_a_r8(BOY *boy) {
  log_debug("Executing %s", __func__);

  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t data = read_r8(boy, z);

  boy->cpu.A &= data;

  clear_flag(&boy->cpu, FLAG_N);
  clear_flag(&boy->cpu, FLAG_C);
  set_flag(&boy->cpu, FLAG_H);

  if (boy->cpu.A == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  boy->cpu.cycles = 1;

  if (z == REG_HL_8)
    boy->cpu.cycles += 1;
}

void xor_a_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t data = read_r8(boy, z);

  boy->cpu.A ^= data;
  clear_flag(&boy->cpu, FLAG_N);
  clear_flag(&boy->cpu, FLAG_H);
  clear_flag(&boy->cpu, FLAG_C);

  if (boy->cpu.A == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  boy->cpu.cycles = 1;

  if (z == REG_HL_8)
    boy->cpu.cycles += 1;
}

void or_a_r8(BOY *boy) {
  log_debug("Executing %s", __func__);

  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t data = read_r8(boy, z);

  boy->cpu.A |= data;

  clear_flag(&boy->cpu, FLAG_N);
  clear_flag(&boy->cpu, FLAG_H);
  clear_flag(&boy->cpu, FLAG_C);

  if (boy->cpu.A == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  boy->cpu.cycles = 1;

  if (z == REG_HL_8)
    boy->cpu.cycles += 1;
}

void cp_a_r8(BOY *boy) {
  log_debug("Executing %s", __func__);

  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t data = read_r8(boy, z);

  set_flag(&boy->cpu, FLAG_N);
  sub8_half_carry(data, &boy->cpu);
  sub8_carry(data, &boy->cpu);

  uint8_t res = boy->cpu.A - data;

  if (res == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  boy->cpu.cycles = 1;

  if (z == REG_HL_8)
    boy->cpu.cycles += 1;
}

void ret(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t lsb = read_byte(&boy->mmu, boy->cpu.SP++);
  uint8_t msb = read_byte(&boy->mmu, boy->cpu.SP++);

  boy->cpu.PC = (msb << 8) | lsb;
  boy->cpu.cycles = 4;
}

void ret_cc(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t y = get_y(boy->cpu.opcode);

  if (condition(&boy->cpu, y)) {
    ret(boy);
  } else {
    boy->cpu.cycles = 2;
  }
}

void ld_0xFF00_n_A(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t n = read_imm8(boy);
  uint16_t dest = 0xFF00 + n;

  write_byte(&boy->mmu, dest, boy->cpu.A);
  boy->cpu.cycles = 3;
}

void ld_A_0xFF00_n(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t n = read_imm8(boy);
  uint16_t src = 0xFF00 + n;
  boy->cpu.A = read_byte(&boy->mmu, src);
}

void add_sp_d(BOY *boy) {
  log_debug("Executing %s", __func__);
  int8_t d = (int8_t)read_byte(&boy->mmu, boy->cpu.PC++);
  uint8_t sp_lower = boy->cpu.SP & 0xFF;

  half_carry8(&boy->cpu, sp_lower, d);
  carry8(&boy->cpu, sp_lower, d);

  clear_flag(&boy->cpu, FLAG_N);
  clear_flag(&boy->cpu, FLAG_Z);

  boy->cpu.SP += d;
  boy->cpu.cycles = 4;
}

void ld_hl_sp_d(BOY *boy) {
  log_debug("Executing %s", __func__);
  int8_t d = (int8_t)read_byte(&boy->mmu, boy->cpu.PC++);
  uint8_t sp_lower = boy->cpu.SP & 0xFF;

  clear_flag(&boy->cpu, FLAG_Z);
  clear_flag(&boy->cpu, FLAG_N);
  half_carry8(&boy->cpu, sp_lower, d);
  carry8(&boy->cpu, sp_lower, d);

  write_r16(&boy->cpu, boy->cpu.SP + d, REG_HL_16, false);

  boy->cpu.cycles = 3;
}

void pop_r16(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t lsb = read_byte(&boy->mmu, boy->cpu.SP++);
  uint8_t msb = read_byte(&boy->mmu, boy->cpu.SP++);

  uint8_t p = get_p(boy->cpu.opcode);
  write_r16(&boy->cpu, (msb << 8) | lsb, p, true);

  boy->cpu.cycles = 3;
}

void reti(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t lsb = read_byte(&boy->mmu, boy->cpu.SP++);
  uint8_t msb = read_byte(&boy->mmu, boy->cpu.SP++);

  boy->cpu.PC = (msb << 8) | lsb;
  boy->cpu.enable_interrupts = true;

  tick(boy, 1);

  boy->cpu.cycles = 4;
}

void jp_hl(BOY *boy) {
  log_debug("Executing %s", __func__);
  boy->cpu.PC = read_r16(&boy->cpu, REG_HL_16, false);
  boy->cpu.cycles = 1;
}

void ld_sp_hl(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t hl = read_r16(&boy->cpu, REG_HL_16, false);
  write_r16(&boy->cpu, hl, REG_SP, false);
}

void jp_cc_nn(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t y = get_y(boy->cpu.opcode);

  uint16_t dest = read_imm16(boy);

  if (condition(&boy->cpu, y)) {
    boy->cpu.PC = dest;
    tick(boy, 1);
    boy->cpu.cycles = 4;
  } else {
    boy->cpu.cycles = 3;
  }
}

void ld_0xFF00_C_A(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t dest = 0xFF00 + boy->cpu.C;
  write_byte(&boy->mmu, dest, boy->cpu.A);
  boy->cpu.cycles = 2;
}

void ld_A_0xFF00_C(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t dest = 0xFF00 + boy->cpu.C;
  boy->cpu.A = read_byte(&boy->mmu, dest);
  boy->cpu.cycles = 2;
}

void ld_nn_a(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t dest = read_imm16(boy);
  write_byte(&boy->mmu, dest, boy->cpu.A);
  boy->cpu.cycles = 4;
}

void ld_a_nn(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint16_t src = read_imm16(boy);
  boy->cpu.A = read_byte(&boy->mmu, src);
  boy->cpu.cycles = 4;
}

void jp_nn(BOY *boy) {
  log_debug("Executing %s", __func__);
  boy->cpu.PC = read_imm16(boy);
  tick(boy, 1);
  boy->cpu.cycles = 4;
}

void ei(BOY *boy) {
  log_debug("Executing %s", __func__);
  boy->cpu.interrupt_delay = true;
  boy->cpu.cycles = 1;
}

void di(BOY *boy) {
  log_debug("Executing %s", __func__);
  boy->cpu.interrupt_delay = false;
  boy->cpu.ime = false;
}

void call_cc_nn(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t y = get_y(boy->cpu.opcode);

  uint16_t r16 = read_imm16(boy);

  if (condition(&boy->cpu, y)) {
    write_byte(&boy->mmu, --boy->cpu.SP, boy->cpu.PC >> 8);
    write_byte(&boy->mmu, --boy->cpu.SP, boy->cpu.PC & 0xFF);

    // set pc
    boy->cpu.PC = r16;
    tick(boy, 1);

    boy->cpu.cycles = 6;

  } else {
    boy->cpu.cycles = 3;
  }
}

void alu_a_n(BOY *boy) {
  log_debug("Executing %s", __func__);
  switch (get_y(boy->cpu.opcode)) {
  case 0:
    add_a_n(boy);
    return;
  case 1:
    adc_a_n(boy);
    return;
  case 2:
    sub_a_n(boy);
    return;
  case 3:
    subc_a_n(boy);
    return;
  case 4:
    and_a_n(boy);
    return;
  case 5:
    xor_a_n(boy);
    return;
  case 6:
    or_a_n(boy);
    return;
  case 7:
    cp_a_n(boy);
    return;
  }
}

void add_a_n(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t data = read_imm8(boy);

  // set flags
  add8_half_carry(data, &boy->cpu);
  add8_carry(data, &boy->cpu);
  clear_flag(&boy->cpu, FLAG_N);

  boy->cpu.A += data;

  if (boy->cpu.A == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  boy->cpu.cycles = 2;
}

void adc_a_n(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t data = read_imm8(boy);

  uint8_t carry = (uint8_t)is_flag_set(&boy->cpu, FLAG_C);

  // set flags
  add8_half_carry(data + carry, &boy->cpu);
  add8_carry(data + carry, &boy->cpu);
  clear_flag(&boy->cpu, FLAG_N);

  boy->cpu.A += data + carry;

  if (boy->cpu.A == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  boy->cpu.cycles = 2;
}

void sub_a_n(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t data = read_imm8(boy);

  set_flag(&boy->cpu, FLAG_N);
  sub8_half_carry(data, &boy->cpu);
  sub8_carry(data, &boy->cpu);

  boy->cpu.A -= data;

  if (boy->cpu.A == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  boy->cpu.cycles = 2;
}

void subc_a_n(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t data = read_imm8(boy);

  uint8_t carry = (uint8_t)is_flag_set(&boy->cpu, FLAG_C);
  data += carry;

  set_flag(&boy->cpu, FLAG_N);
  sub8_half_carry(data, &boy->cpu);
  sub8_carry(data, &boy->cpu);

  boy->cpu.A -= data;

  if (boy->cpu.A == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  boy->cpu.cycles = 2;
}

void and_a_n(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t data = read_imm8(boy);

  boy->cpu.A &= data;

  clear_flag(&boy->cpu, FLAG_N);
  clear_flag(&boy->cpu, FLAG_C);
  set_flag(&boy->cpu, FLAG_H);

  if (boy->cpu.A == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  boy->cpu.cycles = 1;
}

void xor_a_n(BOY *boy) {
  log_debug("Executing %s", __func__);

  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t data = read_imm8(boy);

  boy->cpu.A ^= data;
  clear_flag(&boy->cpu, FLAG_N);
  clear_flag(&boy->cpu, FLAG_H);
  clear_flag(&boy->cpu, FLAG_C);

  if (boy->cpu.A == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  boy->cpu.cycles = 2;
}

void or_a_n(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t data = read_imm8(boy);

  boy->cpu.A |= data;

  clear_flag(&boy->cpu, FLAG_N);
  clear_flag(&boy->cpu, FLAG_H);
  clear_flag(&boy->cpu, FLAG_C);

  if (boy->cpu.A == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  boy->cpu.cycles = 2;
}

void cp_a_n(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t data = read_imm8(boy);

  set_flag(&boy->cpu, FLAG_N);
  sub8_half_carry(data, &boy->cpu);
  sub8_carry(data, &boy->cpu);

  uint8_t res = boy->cpu.A - data;

  if (res == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  boy->cpu.cycles = 2;
}

void call_nn(BOY *boy) {
  log_debug("Executing %s", __func__);
  // get the address to jump too
  // 2 m cycles
  uint16_t dest = read_imm16(boy);

  // pc will be holding the address of instruction to save

  // push this to the stack
  // 2 m cycles
  write_byte(&boy->mmu, --boy->cpu.SP, boy->cpu.PC >> 8);
  write_byte(&boy->mmu, --boy->cpu.SP, boy->cpu.PC & 0xFF);

  // set pc
  boy->cpu.PC = dest;

  // 1 m cycle to set dest
  tick(boy, 1);

  boy->cpu.cycles = 6;
}

void push_r16(BOY *boy) {
  log_debug("Executing %s", __func__);
  // pushes value from r16 reg onto the stack
  // decr sp read
  uint8_t p = get_p(boy->cpu.opcode);
  uint16_t data = read_r16(&boy->cpu, p, true);

  write_byte(&boy->mmu, --boy->cpu.SP, data >> 8);
  write_byte(&boy->mmu, --boy->cpu.SP, data & 0xFF);

  boy->cpu.cycles = 4;
}

void rst(BOY *boy) {
  log_debug("Executing %s", __func__);

  // no op cus of rst bug in tetris lol

  // uint16_t addr = get_y(boy->cpu.opcode) * 8;

  // cpu->SP -= 2;
  // write_word(cpu->SP, addr);
  // cpu->PC = addr;

  // boy->cpu.cycles = 4;
}

void rot(BOY *boy) {
  log_debug("Executing %s", __func__);
  switch (get_y(boy->cpu.opcode)) {
  case 0:
    rlc_r8(boy);
    break;
  case 1:
    rrc_r8(boy);
    break;
  case 2:
    rl_r8(boy);
    break;

  case 3:
    rr_r8(boy);
    break;

  case 4:
    sla_r8(boy);
    break;

  case 5:
    sra_r8(boy);
    break;

  case 6:
    swap_r8(boy);
    break;

  case 7:
    srl_r8(boy);
    break;
  }
}

void rlc_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t z = get_z(boy->cpu.opcode);

  uint8_t data = read_r8(boy, z);
  uint8_t msb = (data & 0x80) >> 7;

  uint8_t rl = (data << 1) | msb;
  write_r8(boy, rl, z);

  if (rl == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  clear_flag(&boy->cpu, FLAG_H);
  clear_flag(&boy->cpu, FLAG_N);

  if (msb) {
    set_flag(&boy->cpu, FLAG_C);
  } else {
    clear_flag(&boy->cpu, FLAG_C);
  }

  if (z == REG_HL_8) {
    boy->cpu.cycles = 4;
  } else {
    boy->cpu.cycles = 2;
  }
}

void rrc_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t data = read_r8(boy, z);

  uint8_t lsb = data & 1;
  uint8_t rrc = (data >> 1) | (lsb << 7);
  write_r8(boy, rrc, z);

  if (rrc == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  clear_flag(&boy->cpu, FLAG_H);
  clear_flag(&boy->cpu, FLAG_N);

  if (lsb) {
    set_flag(&boy->cpu, FLAG_C);
  } else {
    clear_flag(&boy->cpu, FLAG_C);
  }

  if (z == REG_HL_8) {
    boy->cpu.cycles = 4;
  } else {
    boy->cpu.cycles = 2;
  }
}

void rl_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t data = read_r8(boy, z);

  uint8_t msb = (data & (1 << 7)) >> 7;
  data <<= 1;

  if (is_flag_set(&boy->cpu, FLAG_C)) {
    data |= 1;
  } else {
    data &= ~1;
  }

  write_r8(boy, data, z);

  if (data == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  clear_flag(&boy->cpu, FLAG_H);
  clear_flag(&boy->cpu, FLAG_N);

  if (msb) {
    set_flag(&boy->cpu, FLAG_C);
  } else {
    clear_flag(&boy->cpu, FLAG_C);
  }

  if (z == REG_HL_8) {
    boy->cpu.cycles = 4;
  } else {
    boy->cpu.cycles = 2;
  }
}

void rr_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t data = read_r8(boy, z);

  uint8_t lsb = data & 1;

  data >>= 1;

  if (is_flag_set(&boy->cpu, FLAG_C)) {
    data |= 0x80;
  } else {
    data &= ~0x80;
  }

  write_r8(boy, data, z);

  if (data == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  clear_flag(&boy->cpu, FLAG_H);
  clear_flag(&boy->cpu, FLAG_N);

  if (lsb) {
    set_flag(&boy->cpu, FLAG_C);
  } else {
    clear_flag(&boy->cpu, FLAG_C);
  }

  if (z == REG_HL_8) {
    boy->cpu.cycles = 4;
  } else {
    boy->cpu.cycles = 2;
  }
}

void sla_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t data = read_r8(boy, z);

  uint8_t msb = (data & 0x80) >> 7;
  data <<= 1;

  write_r8(boy, data, z);

  if (data == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  clear_flag(&boy->cpu, FLAG_H);
  clear_flag(&boy->cpu, FLAG_N);

  if (msb) {
    set_flag(&boy->cpu, FLAG_C);
  } else {
    clear_flag(&boy->cpu, FLAG_C);
  }

  if (z == REG_HL_8) {
    boy->cpu.cycles = 4;
  } else {
    boy->cpu.cycles = 2;
  }
}

void sra_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t data = read_r8(boy, z);

  uint8_t lsb = data & 1;
  uint8_t msb = data & 0x80;

  data = (data >> 1) | msb;

  write_r8(boy, data, z);

  if (data == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  clear_flag(&boy->cpu, FLAG_H);
  clear_flag(&boy->cpu, FLAG_N);

  if (lsb) {
    set_flag(&boy->cpu, FLAG_C);
  } else {
    clear_flag(&boy->cpu, FLAG_C);
  }

  if (z == REG_HL_8) {
    boy->cpu.cycles = 4;
  } else {
    boy->cpu.cycles = 2;
  }
}

void swap_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t data = read_r8(boy, z);

  data = (data >> 4) | (data << 4);

  write_r8(boy, data, z);

  if (data == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  clear_flag(&boy->cpu, FLAG_H);
  clear_flag(&boy->cpu, FLAG_N);
  clear_flag(&boy->cpu, FLAG_C);

  if (z == REG_HL_8) {
    boy->cpu.cycles = 4;
  } else {
    boy->cpu.cycles = 2;
  }
}

void srl_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t data = read_r8(boy, z);

  uint8_t lsb = data & 1;

  data >>= 1;

  write_r8(boy, data, z);

  if (data == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  clear_flag(&boy->cpu, FLAG_H);
  clear_flag(&boy->cpu, FLAG_N);

  if (lsb) {
    set_flag(&boy->cpu, FLAG_C);
  } else {
    clear_flag(&boy->cpu, FLAG_C);
  }

  if (z == REG_HL_8) {
    boy->cpu.cycles = 4;
  } else {
    boy->cpu.cycles = 2;
  }
}

void bit_y_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  // check if bit y is set
  // shift y times then mask off all upper 7 bits

  uint8_t y = get_y(boy->cpu.opcode);
  uint8_t z = get_z(boy->cpu.opcode);

  uint8_t bit_y = (read_r8(boy, z) >> y) & 1;

  if (bit_y == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  clear_flag(&boy->cpu, FLAG_N);
  set_flag(&boy->cpu, FLAG_H);

  if (z == REG_HL_8) {
    boy->cpu.cycles = 3;
  } else {
    boy->cpu.cycles = 2;
  }
}

void res_y_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t mask = ~(1 << get_y(boy->cpu.opcode));

  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t old = read_r8(boy, z);

  uint8_t new = old | mask;

  write_r8(boy, new, z);

  if (z == REG_HL_8) {
    boy->cpu.cycles = 4;
  } else {
    boy->cpu.cycles = 2;
  }
}

void set_y_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t mask = (1 << get_y(boy->cpu.opcode));

  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t old = read_r8(boy, z);

  uint8_t new = old | mask;

  write_r8(boy, new, z);

  if (z == REG_HL_8) {
    boy->cpu.cycles = 4;
  } else {
    boy->cpu.cycles = 2;
  }
}
// CPU UTILS BEGINbibin/
//
//
//

void decode_instruction(BOY *boy) {
  if (boy->cpu.interrupt_delay) {
    boy->cpu.interrupt_delay = false;
    boy->cpu.enable_interrupts = true;
  } else if (boy->cpu.enable_interrupts) {
    boy->cpu.ime = true;
  }

  boy->cpu.opcode = read_byte(&boy->mmu, boy->cpu.PC++);

  // specific bitfields in the opcode which aid in decoding
  uint8_t y = get_y(boy->cpu.opcode);
  uint8_t q = get_q(boy->cpu.opcode);
  uint8_t p = get_p(boy->cpu.opcode);
  uint8_t z = get_z(boy->cpu.opcode);
  uint8_t x = get_x(boy->cpu.opcode);

  // check for prefix byte first
  switch (boy->cpu.opcode) {
  case 0xCB:
    boy->cpu.opcode = read_byte(&boy->mmu, boy->cpu.PC++);
    uint8_t x = get_x(boy->cpu.opcode);

    switch (x) {
    case 0:
      rot(boy);
      return;
    case 1:
      bit_y_r8(boy);
      return;
    case 2:
      res_y_r8(boy);
      break;
    case 3:
      set_y_r8(boy);
      return;
      break;
    }

  case 0xDD:
  case 0xED:
  case 0xFD:
    boy->cpu.opcode = read_byte(&boy->mmu, boy->cpu.PC++);
    return;
  }

  switch (x) {

    // x = 0
  case 0: {
    switch (z) {
    // z = 0
    case 0:
      switch (y) {
      case 0:
        nop(boy);
        return;

      case 1:
        ld_nn_sp(boy);
        return;

      case 2:
        stop(boy);
        return;

      case 3:
        jr_d(boy);
        return;

      case 4 ... 7:
        jr_cc_d(boy);
        return;
      }

      break;

      // z = 1

    case 1: {
      switch (q) {
      case 0:
        ld_r16_imm16(boy);
        return;
      case 1:
        add_hl_r16(boy);
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
          ld_bc_a(boy);
          return;

        case 1:
          ld_de_a(boy);
          return;

        case 2:
          ld_hli_a(boy);
          return;

        case 3:
          ld_hld_a(boy);
          return;
        }

        break;

      case 1:
        switch (p) {
        case 0:
          ld_a_bc(boy);
          return;

        case 1:
          ld_a_hli(boy);
          return;

        case 2:
          ld_a_de(boy);
          return;

        case 3:
          ld_a_hld(boy);
          return;
        }

        break;
      }
    }
      // z = 3

    case 3: {
      switch (q) {
      case 0:
        inc_r16(boy);
        return;
      case 1:
        dec_r16(boy);
        return;
      }
      break;
    }

    // z = 4
    case 4: {
      inc_r8(boy);
      return;
    }

    // z = 5
    case 5: {
      dec_r8(boy);
      return;
    }

    // z = 6
    case 6: {
      ld_r8_imm8(boy);
      return;
    }

    // z = 7
    case 7: {
      switch (y) {
      case 0:
        rlca(boy);
        return;
      case 1:
        rrca(boy);
        return;
      case 2:
        rla(boy);
        return;
      case 3:
        rra(boy);
        return;
      case 4:
        daa(boy);
        return;
      case 5:
        cpl(boy);
        return;
      case 6:
        scf(boy);
        return;
      case 7:
        ccf(boy);
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
        halt(boy);
        return;
      }
    }
    ld_r8_r8(boy);
    return;
  }

  // x = 2
  case 2: {
    alu_a_r8(boy);
    return;
  }

  // x = 3
  case 3: {
    switch (z) {

    // z = 0
    case 0: {
      switch (y) {
      case 0 ... 3:
        ret_cc(boy);
        return;
      case 4:
        ld_0xFF00_n_A(boy);
        return;
      case 5:
        add_sp_d(boy);
        return;
      case 6:
        ld_A_0xFF00_n(boy);
        return;
      case 7:
        ld_hl_sp_d(boy);
        return;
      }
      break;
    }

    // z = 1
    case 1: {
      switch (q) {
      case 0:
        pop_r16(boy);
        return;
      case 1: {
        switch (p) {
        case 0:
          ret(boy);
          return;
        case 1:
          reti(boy);
          return;
        case 2:
          jp_hl(boy);
        case 3:
          ld_sp_hl(boy);
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
        jp_cc_nn(boy);
        return;

      case 4:
        ld_0xFF00_C_A(boy);
        return;

      case 5:
        ld_nn_a(boy);
        return;

      case 6:
        ld_A_0xFF00_C(boy);
        return;

      case 7:
        ld_a_nn(boy);
        return;
      }
      break;
    }

    // z = 3
    case 3: {
      switch (y) {
      case 0:
        jp_nn(boy);
        return;

      case 6:
        di(boy);
        return;

      case 7:
        ei(boy);
        return;
      }
      break;
    }

    // z = 4
    case 4: {
      switch (y) {
      case 0 ... 3:
        call_cc_nn(boy);
        return;
      }
      break;
    }

    // z = 5
    case 5: {
      switch (q) {
      case 0:
        push_r16(boy);
        return;

      case 1: {
        switch (p) {
        case 0:
          call_nn(boy);
          return;
        }
        break;
      }
      }
      break;
    }

    // z = 6
    case 6: {
      alu_a_n(boy);
      return;
    }

    case 7: {
      rst(boy);
      return;
    }
    }
  }
  }
}
