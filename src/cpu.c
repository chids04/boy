#include "cpu.h"
#include "boy.h"
#include "log.h"
#include "mmu.h"

#include "utils.h"
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
  cpu->PC = 0x0100;
  cpu->SP = 0xFFFE;
  cpu->enable_interrupts = false;
  cpu->interrupt_delay = false;
  cpu->is_halted = false;
}

void handle_cb_prefix(BOY *boy) {
  boy->cpu.opcode = read_byte(boy, boy->cpu.PC++);

  switch (get_bit_range(boy->cpu.opcode, 7, 3)) {
  case 0b00000:
    rlc_r8(boy);
    return;
  case 0b00001:
    rrc_r8(boy);
    return;
  case 0b00010:
    rl_r8(boy);
    return;
  case 0b00011:
    rr_r8(boy);
    return;
  case 0b00100:
    sla_r8(boy);
    return;
  case 0b00101:
    sra_r8(boy);
    return;
  case 0b00110:
    swap_r8(boy);
    return;
  case 0b00111:
    srl_r8(boy);
    return;
  }

  switch (get_bit_range(boy->cpu.opcode, 7, 6)) {
  case 0b01:
    bit_r8(boy);
    return;
  case 0b10:
    res_r8(boy);
    return;
  case 0b11:
    set_r8(boy);
    return;
  }
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

void ld_imm16_sp(BOY *boy) {
  log_debug("Executing %s", __func__);
  // load stack pointer to mem[nn], with nn being word after opcode

  uint16_t imm16 = read_imm16(boy);

  // 2 cycles to write the 16 bit value to memory
  write_byte(boy, imm16, boy->cpu.SP & 0xFF);
  write_byte(boy, imm16 + 1, (boy->cpu.SP >> 8) & 0xFF);

  boy->cpu.cycles = 5;
}

void ld_imm16_a(BOY *boy) {
  log_debug("Executing %s", __func__);
  // load register A to mem[nn], with nn being word after opcode
  uint16_t imm16 = read_imm16(boy);
  write_byte(boy, imm16, boy->cpu.A);
  boy->cpu.cycles = 4;
}

void ld_a_imm16(BOY *boy) {
  log_debug("Executing %s", __func__);
  // load mem[nn] into register A, with nn being word after opcode
  uint16_t imm16 = read_imm16(boy);
  boy->cpu.A = read_byte(boy, imm16);
}

void ld_r16mem_a(BOY *boy) {
  log_debug("Executing %s", __func__);
  // store A in mem[r16mem]
  uint8_t reg = get_bit_range(boy->cpu.opcode, 5, 4);
  write_r16mem(boy, reg, boy->cpu.A);
}

void ld_a_r16mem(BOY *boy) {
  log_debug("Executing %s", __func__);
  // store [r16mem] into A
  uint8_t reg = get_bit_range(boy->cpu.opcode, 5, 4);
  uint8_t data = read_r16mem(boy, reg);

  boy->cpu.A = data;
}

void jr_d(BOY *boy) {
  log_debug("Executing %s", __func__);
  // add signed 8bit 'd' to next instruction,

  int8_t d = (int8_t)read_byte(boy, boy->cpu.PC++);

  boy->cpu.PC += d;

  tick(boy, 1);

  boy->cpu.cycles = 3;
}

void jr_cc_d(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t cond = get_bit_range(boy->cpu.opcode, 4, 3);

  int8_t d = (int8_t)read_byte(boy, boy->cpu.PC++);

  if (condition(&boy->cpu, cond)) {
    boy->cpu.PC += d;
    tick(boy, 1);
    boy->cpu.cycles = 3;
  } else {
    boy->cpu.cycles = 2;
  }
}

void ld_r16_imm16(BOY *boy) {
  log_debug("Executing %s", __func__);

  uint16_t imm16 = read_imm16(boy);
  // p is top 2 msb of y (bits 5 - 4 of opcode)
  uint8_t reg = get_bit_range(boy->cpu.opcode, 5, 4);

  write_r16(&boy->cpu, imm16, reg);

  boy->cpu.cycles = 3;
}

void add_hl_r16(BOY *boy) {
  log_debug("Executing %s", __func__);
  // p is top 2 msb of y (bits 5 - 4 of opcode)
  uint8_t reg = get_bit_range(boy->cpu.opcode, 5, 4);
  uint16_t data = read_r16(&boy->cpu, reg);

  // calc flags
  uint16_t hl = read_r16(&boy->cpu, 2);

  if (data + hl > 0xFFFF) {
    set_flag(&boy->cpu, FLAG_C);
  } else {
    clear_flag(&boy->cpu, FLAG_C);
  }

  if ((data & 0x0FFF) + (hl & 0x0FFF) > 0xFFF) {
    set_flag(&boy->cpu, FLAG_H);
  } else {
    clear_flag(&boy->cpu, FLAG_H);
  }

  clear_flag(&boy->cpu, FLAG_N);

  write_r16(&boy->cpu, data + hl, 2);
  tick(boy, 1);

  boy->cpu.cycles = 2;
}

void inc_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t reg = get_bit_range(boy->cpu.opcode, 5, 3);

  uint8_t data = read_r8(boy, reg);

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
  write_r8(boy, data, reg);
}

void dec_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t reg = get_bit_range(boy->cpu.opcode, 5, 3);

  uint8_t data = read_r8(boy, reg);

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
  write_r8(boy, data, reg);

  boy->cpu.cycles = 1;
}

void inc_r16(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t reg = get_bit_range(boy->cpu.opcode, 5, 4);

  uint16_t data = read_r16(&boy->cpu, reg);
  write_r16(&boy->cpu, data += 1, reg);

  tick(boy, 1);

  boy->cpu.cycles = 2;
}

void dec_r16(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t reg = get_bit_range(boy->cpu.opcode, 5, 4);
  uint16_t data = read_r16(&boy->cpu, reg);
  write_r16(&boy->cpu, data - 1, reg);
  tick(boy, 1);

  boy->cpu.cycles = 2;
}

void ld_r8_imm8(BOY *boy) {
  log_debug("Executing %s", __func__);
  // write the immediate next byte to 8 bit register
  uint8_t reg = get_bit_range(boy->cpu.opcode, 5, 3);

  uint8_t imm8 = read_imm8(boy);
  write_r8(boy, imm8, reg);

  boy->cpu.cycles = 2;

  // Add extra cycle for HL register memory operation
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

  // subtraction adjustment
  if (is_flag_set(&boy->cpu, FLAG_N)) {
    if (is_flag_set(&boy->cpu, FLAG_H)) {
      adjustment |= 0x06;
    }

    if (is_flag_set(&boy->cpu, FLAG_C)) {
      adjustment |= 0x60;
    }

    boy->cpu.A -= adjustment;

  } else {

    // addition adjustment
    if (is_flag_set(&boy->cpu, FLAG_H) || (boy->cpu.A & 0xF) > 0x09) {
      adjustment |= 0x06;
    }

    if (is_flag_set(&boy->cpu, FLAG_C) || boy->cpu.A > 0x99) {
      adjustment |= 0x60;

      // ensure carry flag is set
      // this could be unset if A is greater thatn 0x99 but less than 0xFF (too
      // big for BCD but small enough to fit in 8 bit register)
      set_flag(&boy->cpu, FLAG_C);
    }

    boy->cpu.A += adjustment;
  }

  if (boy->cpu.A == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  clear_flag(&boy->cpu, FLAG_H);

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

  uint8_t source = get_bit_range(boy->cpu.opcode, 2, 0);
  uint8_t dest = get_bit_range(boy->cpu.opcode, 5, 3);

  uint8_t right_data = read_r8(boy, source);
  write_r8(boy, right_data, dest);
}

void halt(BOY *boy) {
  log_debug("Executing %s", __func__);
  boy->cpu.is_halted = true;
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
  uint8_t reg = get_bit_range(boy->cpu.opcode, 2, 0);
  uint8_t data = read_r8(boy, reg);

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
}

void adc_a_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t reg = get_bit_range(boy->cpu.opcode, 2, 0);
  uint8_t data = read_r8(boy, reg);

  uint8_t carry = (uint8_t)is_flag_set(&boy->cpu, FLAG_C);

  // set flags

  // check for half carry
  if (((boy->cpu.A & 0xF) + (data & 0xF) + carry) > 0xF) {
    set_flag(&boy->cpu, FLAG_H);
  } else {
    clear_flag(&boy->cpu, FLAG_H);
  }

  // check for carry
  if ((int)(boy->cpu.A + data + carry) > 0xFF) {
    set_flag(&boy->cpu, FLAG_C);
  } else {
    clear_flag(&boy->cpu, FLAG_C);
  }

  clear_flag(&boy->cpu, FLAG_N);

  boy->cpu.A += data + carry;

  if (boy->cpu.A == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  boy->cpu.cycles = 1;

  // Add extra cycle for HL register memory operation
}

void sub_a_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  // A = A - r8

  uint8_t reg = get_bit_range(boy->cpu.opcode, 2, 0);
  uint8_t data = read_r8(boy, reg);

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
}

void subc_a_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  // A = A - r8 - carry

  uint8_t reg = get_bit_range(boy->cpu.opcode, 2, 0);
  uint8_t data = read_r8(boy, reg);

  uint8_t carry = (uint8_t)is_flag_set(&boy->cpu, FLAG_C);

  if (((int)(boy->cpu.A & 0xF) - (int)(data & 0xF) - (int)carry) < 0) {
    set_flag(&boy->cpu, FLAG_H);
  } else {
    clear_flag(&boy->cpu, FLAG_H);
  }

  if (((int)(boy->cpu.A) - (int)data - (int)carry) < 0) {
    set_flag(&boy->cpu, FLAG_C);
  } else {
    clear_flag(&boy->cpu, FLAG_C);
  }

  set_flag(&boy->cpu, FLAG_N);

  boy->cpu.A -= data + carry;

  if (boy->cpu.A == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }


}

void and_a_r8(BOY *boy) {
  log_debug("Executing %s", __func__);

  uint8_t reg = get_bit_range(boy->cpu.opcode, 2, 0);
  uint8_t data = read_r8(boy, reg);

  boy->cpu.A &= data;

  clear_flag(&boy->cpu, FLAG_N);
  clear_flag(&boy->cpu, FLAG_C);
  set_flag(&boy->cpu, FLAG_H);

  if (boy->cpu.A == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }
}

void xor_a_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t reg = get_bit_range(boy->cpu.opcode, 2, 0);
  uint8_t data = read_r8(boy, reg);

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
}

void or_a_r8(BOY *boy) {
  log_debug("Executing %s", __func__);

  uint8_t reg = get_bit_range(boy->cpu.opcode, 2, 0);
  uint8_t data = read_r8(boy, reg);

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
}

void cp_a_r8(BOY *boy) {
  log_debug("Executing %s", __func__);

  uint8_t reg = get_bit_range(boy->cpu.opcode, 2, 0);
  uint8_t data = read_r8(boy, reg);

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
}

void ret(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t lsb = read_byte(boy, boy->cpu.SP++);
  uint8_t msb = read_byte(boy, boy->cpu.SP++);

  boy->cpu.PC = (msb << 8) | lsb;
  boy->cpu.cycles = 4;
}

void ret_cc(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t reg = get_bit_range(boy->cpu.opcode, 4, 3);

  if (condition(&boy->cpu, reg)) {
    ret(boy);
  } else {
    boy->cpu.cycles = 2;
  }
}

void ldh_imm8_a(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t n = read_imm8(boy);
  uint16_t dest = 0xFF00 | n;

  write_byte(boy, dest, boy->cpu.A);
  boy->cpu.cycles = 3;
}

void ldh_a_imm8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t n = read_imm8(boy);
  uint16_t src = 0xFF00 | n;
  boy->cpu.A = read_byte(boy, src);
}

void add_sp_imm8(BOY *boy) {
  log_debug("Executing %s", __func__);
  int8_t d = (int8_t)read_byte(boy, boy->cpu.PC++);
  uint8_t sp_lower = boy->cpu.SP & 0xFF;

  half_carry8(&boy->cpu, sp_lower, d);
  carry8(&boy->cpu, sp_lower, d);

  clear_flag(&boy->cpu, FLAG_N);
  clear_flag(&boy->cpu, FLAG_Z);

  boy->cpu.SP += d;
  boy->cpu.cycles = 4;
}

void ld_hl_sp_imm8(BOY *boy) {
  log_debug("Executing %s", __func__);
  int8_t d = (int8_t)read_byte(boy, boy->cpu.PC++);

  clear_flag(&boy->cpu, FLAG_Z);
  clear_flag(&boy->cpu, FLAG_N);

  half_carry8(&boy->cpu, boy->cpu.SP, d);
  carry8(&boy->cpu, boy->cpu.SP, d);

  write_r16(&boy->cpu, boy->cpu.SP + d, 2);

  boy->cpu.cycles = 3;
}

void pop_r16stk(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t lsb = read_byte(boy, boy->cpu.SP++);
  uint8_t msb = read_byte(boy, boy->cpu.SP++);

  uint8_t reg = get_bit_range(boy->cpu.opcode, 5, 4);
  write_r16stk(&boy->cpu, reg, (msb << 8) | lsb);

  boy->cpu.cycles = 3;
}

void push_r16stk(BOY *boy) {
  log_debug("Executing %s", __func__);
  // pushes value from r16 reg onto the stack
  // decr sp read
  uint8_t reg = get_bit_range(boy->cpu.opcode, 5, 4);
  uint16_t data = read_r16stk(&boy->cpu, reg);

  write_byte(boy, --boy->cpu.SP, data >> 8);
  write_byte(boy, --boy->cpu.SP, data & 0xFF);

  tick(boy, 1);

  boy->cpu.cycles = 4;
}

void reti(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t lsb = read_byte(boy, boy->cpu.SP++);
  uint8_t msb = read_byte(boy, boy->cpu.SP++);

  boy->cpu.PC = (msb << 8) | lsb;
  boy->cpu.enable_interrupts = true;

  tick(boy, 1);

  boy->cpu.cycles = 4;
}

void jp_hl(BOY *boy) {
  log_debug("Executing %s", __func__);
  boy->cpu.PC = read_r16(&boy->cpu, 2);
  boy->cpu.cycles = 1;
}

void ld_sp_hl(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint16_t hl = read_r16(&boy->cpu, 2);
  write_r16(&boy->cpu, hl, 3);
}

void jp_cc_imm16(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t cond = get_bit_range(boy->cpu.opcode, 4, 3);

  uint16_t dest = read_imm16(boy);

  if (condition(&boy->cpu, cond)) {
    boy->cpu.PC = dest;
    tick(boy, 1);
    boy->cpu.cycles = 4;
  } else {
    boy->cpu.cycles = 3;
  }
}

void ldh_c_a(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t dest = 0xFF00 + boy->cpu.C;
  write_byte(boy, dest, boy->cpu.A);
  boy->cpu.cycles = 2;
}

void ldh_a_c(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t src = 0xFF00 + boy->cpu.C;
  boy->cpu.A = read_byte(boy, src);
  boy->cpu.cycles = 2;
}

void ld_nn_a(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t dest = read_imm16(boy);
  write_byte(boy, dest, boy->cpu.A);
  boy->cpu.cycles = 4;
}

void ld_a_nn(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint16_t src = read_imm16(boy);
  boy->cpu.A = read_byte(boy, src);
  boy->cpu.cycles = 4;
}

void jp_imm16(BOY *boy) {
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

void call_cond_imm16(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t cond = get_bit_range(boy->cpu.opcode, 4, 3);

  uint16_t r16 = read_imm16(boy);

  if (condition(&boy->cpu, cond)) {
    write_byte(boy, --boy->cpu.SP, boy->cpu.PC >> 8);
    write_byte(boy, --boy->cpu.SP, boy->cpu.PC & 0xFF);

    // set pc
    boy->cpu.PC = r16;
    tick(boy, 1);

    boy->cpu.cycles = 6;

  } else {
    boy->cpu.cycles = 3;
  }
}

void add_a_imm8(BOY *boy) {
  log_debug("Executing %s", __func__);
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
}

void adc_a_imm8(BOY *boy) {
  log_debug("Executing %s", __func__);
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

void sub_a_imm8(BOY *boy) {
  log_debug("Executing %s", __func__);
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

void subc_a_imm8(BOY *boy) {
  log_debug("Executing %s", __func__);
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

void and_a_imm8(BOY *boy) {
  log_debug("Executing %s", __func__);
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

void xor_a_imm8(BOY *boy) {
  log_debug("Executing %s", __func__);

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

void or_a_imm8(BOY *boy) {
  log_debug("Executing %s", __func__);
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

void cp_a_imm8(BOY *boy) {
  log_debug("Executing %s", __func__);
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

void call_imm16(BOY *boy) {
  log_debug("Executing %s", __func__);
  // get the address to jump too
  // 2 m cycles
  uint16_t dest = read_imm16(boy);

  // pc will be holding the address of instruction to save
  // push this to the stack
  // 2 m cycles
  write_byte(boy, --boy->cpu.SP, boy->cpu.PC >> 8);
  write_byte(boy, --boy->cpu.SP, boy->cpu.PC & 0xFF);

  // set pc
  boy->cpu.PC = dest;

  // 1 m cycle to set dest
  tick(boy, 1);

  boy->cpu.cycles = 6;
}

void rst(BOY *boy) {
  log_debug("Executing %s", __func__);

  uint16_t addr = get_bit_range(boy->cpu.opcode, 4, 3) * 8;

  write_byte(boy, --boy->cpu.SP, boy->cpu.PC >> 8);
  write_byte(boy, --boy->cpu.SP, boy->cpu.PC & 0xFF);

  boy->cpu.PC = addr;
  tick(boy, 1);
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
  uint8_t reg = get_bit_range(boy->cpu.opcode, 2, 0);
  uint8_t data = read_r8(boy, reg);

  uint8_t msb = (data & 0x80) >> 7;

  uint8_t rl = (data << 1) | msb;

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

  write_r8(boy, rl, reg);
}

void rrc_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t reg = get_bit_range(boy->cpu.opcode, 2, 0);
  uint8_t data = read_r8(boy, reg);

  uint8_t lsb = data & 1;
  uint8_t rrc = (data >> 1) | (lsb << 7);

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

  write_r8(boy, rrc, reg);
}

void rl_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t reg = get_bit_range(boy->cpu.opcode, 2, 0);
  uint8_t data = read_r8(boy, reg);

  uint8_t msb = (data & (1 << 7)) >> 7;
  data <<= 1;

  if (is_flag_set(&boy->cpu, FLAG_C)) {
    data |= 1;
  } else {
    data &= ~1;
  }

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

  write_r8(boy, data, reg);
}

void rr_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t reg = get_bit_range(boy->cpu.opcode, 2, 0);
  uint8_t data = read_r8(boy, reg);

  uint8_t lsb = data & 1;

  data >>= 1;

  if (is_flag_set(&boy->cpu, FLAG_C)) {
    data |= 0x80;
  } else {
    data &= ~0x80;
  }

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

  write_r8(boy, data, reg);
}

void sla_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t reg = get_bit_range(boy->cpu.opcode, 2, 0);
  uint8_t data = read_r8(boy, reg);

  uint8_t msb = (data & 0x80) >> 7;
  data <<= 1;

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

  write_r8(boy, data, reg);
}

void sra_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t reg = get_bit_range(boy->cpu.opcode, 2, 0);
  uint8_t data = read_r8(boy, reg);

  uint8_t lsb = data & 1;
  uint8_t msb = data & 0x80;

  data = (data >> 1) | msb;

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

  write_r8(boy, data, reg);
}

void swap_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t reg = get_bit_range(boy->cpu.opcode, 2, 0);
  uint8_t data = read_r8(boy, reg);

  data = (data >> 4) | (data << 4);

  if (data == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  clear_flag(&boy->cpu, FLAG_H);
  clear_flag(&boy->cpu, FLAG_N);
  clear_flag(&boy->cpu, FLAG_C);

  write_r8(boy, data, reg);
}

void srl_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  uint8_t reg = get_bit_range(boy->cpu.opcode, 2, 0);
  uint8_t data = read_r8(boy, reg);

  uint8_t lsb = data & 1;

  data >>= 1;

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

  write_r8(boy, data, reg);
}

void bit_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  // check if bit y is set
  // shift y times right then mask off all upper 7 bits

  uint8_t reg = get_bit_range(boy->cpu.opcode, 2, 0);
  int bit_index = get_bit_range(boy->cpu.opcode, 5, 3);

  uint8_t bit = get_bit(read_r8(boy, reg), bit_index);

  if (bit == 0) {
    set_flag(&boy->cpu, FLAG_Z);
  } else {
    clear_flag(&boy->cpu, FLAG_Z);
  }

  clear_flag(&boy->cpu, FLAG_N);
  set_flag(&boy->cpu, FLAG_H);

  tick(boy, 1);
}

void res_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  int bit_index = get_bit_range(boy->cpu.opcode, 5, 3);
  uint8_t mask = ~(1 << bit_index);

  uint8_t reg = get_bit_range(boy->cpu.opcode, 2, 0);
  uint8_t old = read_r8(boy, reg);

  uint8_t new = old &mask;
  tick(boy, 1);

  write_r8(boy, new, reg);
}

void set_r8(BOY *boy) {
  log_debug("Executing %s", __func__);
  int bit_index = get_bit_range(boy->cpu.opcode, 5, 3);
  uint8_t mask = (1 << bit_index);

  uint8_t reg = get_bit_range(boy->cpu.opcode, 2, 0);
  uint8_t old = read_r8(boy, reg);

  uint8_t new = old | mask;

  tick(boy, 1);

  write_r8(boy, new, reg);
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

  if (boy->cpu.is_halted)
    return;

  boy->cpu.opcode = read_byte(boy, boy->cpu.PC++);

  // local read-only copy to save typing lol
  uint8_t op = boy->cpu.opcode;
  // block 0

  // check for no op first

  if (op == 0x0) {
    nop(boy);
    return;
  } else if (get_bit_range(op, 7, 6) == 0b00) {
    if (op == 0b00000111) {
      rlca(boy);
      return;
    } else if (op == 0b00001111) {
      rrca(boy);
      return;
    } else if (op == 0b00010111) {
      rla(boy);
      return;
    } else if (op == 0b00011111) {
      rra(boy);
      return;
    } else if (op == 0b00100111) {
      daa(boy);
      return;
    } else if (op == 0b00101111) {
      cpl(boy);
      return;
    } else if (op == 0b00110111) {
      scf(boy);
      return;
    } else if (op == 0b00111111) {
      ccf(boy);
      return;
    } else if (op == 0b00011000) {
      jr_d(boy);
      return;
    } else if (get_bit_range(op, 7, 5) == 0b001 &&
               get_bit_range(op, 2, 0) == 0x0) {
      jr_cc_d(boy);
      return;
    } else if (op == 0b00010000) {
      stop(boy);
      return;
    } else if (get_bit_range(op, 3, 0) == 0b0001) {
      ld_r16_imm16(boy);
      return;
    } else if (get_bit_range(op, 3, 0) == 0b0010) {
      ld_r16mem_a(boy);
      return;
    } else if (get_bit_range(op, 3, 0) == 0b1010) {
      ld_a_r16mem(boy);
      return;
    } else if (get_bit_range(op, 3, 0) == 0b1000) {
      ld_imm16_sp(boy);
      return;
    } else if (get_bit_range(op, 3, 0) == 0b0011) {
      inc_r16(boy);
      return;
    } else if (get_bit_range(op, 3, 0) == 0b1011) {
      dec_r16(boy);
      return;
    } else if (get_bit_range(op, 3, 0) == 0b1001) {
      add_hl_r16(boy);
      return;
    } else if (get_bit_range(op, 2, 0) == 0b100) {
      inc_r8(boy);
      return;
    } else if (get_bit_range(op, 2, 0) == 0b101) {
      dec_r8(boy);
      return;
    } else if (get_bit_range(op, 2, 0) == 0b110) {
      ld_r8_imm8(boy);
      return;
    }
  }

  // block 1, 8 bit register to register load
  if (get_bit_range(op, 7, 6) == 0b01) {
    // ld [hl] [hl] maps to halt
    if (op == 0b01110110) {
      halt(boy);
      return;
    }

    else {
      ld_r8_r8(boy);
      return;
    }
  }

  // block 2 8 bit arithmetic

  if (get_bit_range(op, 7, 3) == 0b10000) {
    add_a_r8(boy);
    return;
  } else if (get_bit_range(op, 7, 3) == 0b10001) {
    adc_a_r8(boy);
    return;
  } else if (get_bit_range(op, 7, 3) == 0b10010) {
    sub_a_r8(boy);
    return;
  } else if (get_bit_range(op, 7, 3) == 0b10011) {
    subc_a_r8(boy);
    return;
  } else if (get_bit_range(op, 7, 3) == 0b10100) {
    and_a_r8(boy);
    return;
  } else if (get_bit_range(op, 7, 3) == 0b10101) {
    xor_a_r8(boy);
    return;
  } else if (get_bit_range(op, 7, 3) == 0b10110) {
    or_a_r8(boy);
    return;
  } else if (get_bit_range(op, 7, 3) == 0b10111) {
    cp_a_r8(boy);
    return;
  }

  // block 3

  if (op == 0b11000110) {
    add_a_imm8(boy);
    return;
  } else if (op == 0b11001110) {
    adc_a_imm8(boy);
    return;
  } else if (op == 0b11010110) {
    sub_a_imm8(boy);
    return;
  } else if (op == 0b11011110) {
    subc_a_imm8(boy);
    return;
  } else if (op == 0b11100110) {
    and_a_imm8(boy);
    return;
  } else if (op == 0b11101110) {
    xor_a_imm8(boy);
    return;
  } else if (op == 0b11110110) {
    or_a_imm8(boy);
    return;
  } else if (op == 0b11111110) {
    cp_a_imm8(boy);
    return;
  } else if (get_bit_range(op, 7, 5) == 0b110 &&
             get_bit_range(op, 2, 0) == 0b000) {
    ret_cc(boy);
    return;
  } else if (op == 0b11001001) {
    ret(boy);
    return;
  } else if (op == 0b11011001) {
    reti(boy);
    return;
  } else if (get_bit_range(op, 7, 5) == 0b110 &&
             get_bit_range(op, 2, 0) == 0b10) {
    jp_cc_imm16(boy);
    return;
  } else if (op == 0b11000011) {
    jp_imm16(boy);
    return;
  } else if (op == 0b11101001) {
    jp_hl(boy);
    return;
  } else if (get_bit_range(op, 7, 5) == 0b110 &&
             get_bit_range(op, 2, 0) == 0b100) {
    call_cond_imm16(boy);
    return;
  } else if (op == 0b11001101) {
    call_imm16(boy);
    return;
  } else if (get_bit_range(op, 7, 6) == 0b11 &&
             get_bit_range(op, 2, 0) == 0b111) {
    rst(boy);
    return;
  } else if (get_bit_range(op, 7, 6) == 0b11 &&
             get_bit_range(op, 3, 0) == 0b0001) {
    pop_r16stk(boy);
    return;
  } else if (get_bit_range(op, 7, 6) == 0b11 &&
             get_bit_range(op, 3, 0) == 0b0101) {
    push_r16stk(boy);
    return;

    // TODO: handle CB prefix insturctions
  } else if (op == 0b11001011) {
    handle_cb_prefix(boy);
    return;
  } else if (op == 0b11100010) {
    ldh_c_a(boy);
    return;
  } else if (op == 0b11100000) {
    ldh_imm8_a(boy);
    return;
  } else if (op == 0b11101010) {
    ld_imm16_a(boy);
    return;
  } else if (op == 0b11110010) {
    ldh_a_c(boy);
    return;
  } else if (op == 0b11110000) {
    ldh_a_imm8(boy);
    return;
  } else if (op == 0b11111010) {
    ld_a_imm16(boy);
    return;
  } else if (op == 0b11101000) {
    add_sp_imm8(boy);
    return;
  } else if (op == 0b11111000) {
    ld_hl_sp_imm8(boy);
    return;
  } else if (op == 0b11111001) {
    ld_sp_hl(boy);
    return;
  } else if (op == 0b11110011) {
    di(boy);
    return;
  } else if (op == 0b11111011) {
    ei(boy);
    return;
  }

  // check for prefix byte first
  // switch (boy->cpu.opcode) {
  // case 0x
  //
  // CB:
  //   boy->cpu.opcode = read_byte(boy, boy->cpu.PC++);
  //   uint8_t x = get_x(boy->cpu.opcode);

  //   switch (x) {
  //   case 0:
  //     rot(boy);
  //     return;
  //   case 1:
  //     bit_y_r8(boy);
  //     return;
  //   case 2:
  //     res_y_r8(boy);
  //     break;
  //   case 3:
  //     set_y_r8(boy);
  //     return;
  //     break;
  //   }

  // case 0xDD:
  // case 0xED:
  // case 0xFD:
  //   boy->cpu.opcode = read_byte(boy, boy->cpu.PC++);
  //   return;
  // }

  // switch (x) {

  //   // x = 0
  // case 0: {
  //   switch (z) {
  //   // z = 0
  //   case 0:
  //     switch (y) {
  //     case 0:
  //       nop(boy);
  //       return;

  //     case 1:
  //       ld_nn_sp(boy);
  //       return;

  //     case 2:
  //       stop(boy);
  //       return;

  //     case 3:
  //       jr_d(boy);
  //       return;

  //     case 4 ... 7:
  //       jr_cc_d(boy);
  //       return;
  //     }

  //     break;

  //     // z = 1

  //   case 1: {
  //     switch (q) {
  //     case 0:
  //       ld_r16_imm16(boy);
  //       return;
  //     case 1:
  //       add_hl_r16(boy);
  //       return;
  //     }

  //     break;
  //   }

  //     // z = 2

  //   case 2: {
  //     switch (q) {
  //     case 0:
  //       switch (p) {
  //       case 0:
  //         ld_bc_a(boy);
  //         return;

  //       case 1:
  //         ld_de_a(boy);
  //         return;

  //       case 2:
  //         ld_hli_a(boy);
  //         return;

  //       case 3:
  //         ld_hld_a(boy);
  //         return;
  //       }

  //       break;

  //     case 1:
  //       switch (p) {
  //       case 0:
  //         ld_a_bc(boy);
  //         return;

  //       case 1:
  //         ld_a_hli(boy);
  //         return;

  //       case 2:
  //         ld_a_de(boy);
  //         return;

  //       case 3:
  //         ld_a_hld(boy);
  //         return;
  //       }

  //       break;
  //     }
  //   }
  //     // z = 3

  //   case 3: {
  //     switch (q) {
  //     case 0:
  //       inc_r16(boy);
  //       return;
  //     case 1:
  //       dec_r16(boy);
  //       return;
  //     }
  //     break;
  //   }

  //   // z = 4
  //   case 4: {
  //     inc_r8(boy);
  //     return;
  //   }

  //   // z = 5
  //   case 5: {
  //     dec_r8(boy);
  //     return;
  //   }

  //   // z = 6
  //   case 6: {
  //     ld_r8_imm8(boy);
  //     return;
  //   }

  //   // z = 7
  //   case 7: {
  //     switch (y) {
  //     case 0:
  //       rlca(boy);
  //       return;
  //     case 1:
  //       rrca(boy);
  //       return;
  //     case 2:
  //       rla(boy);
  //       return;
  //     case 3:
  //       rra(boy);
  //       return;
  //     case 4:
  //       daa(boy);
  //       return;
  //     case 5:
  //       cpl(boy);
  //       return;
  //     case 6:
  //       scf(boy);
  //       return;
  //     case 7:
  //       ccf(boy);
  //       return;
  //     }
  //   }
  //   }
  // }

  // // x = 1
  // case 1: {
  //   switch (z) {
  //     switch (y) {
  //     case 6:
  //       halt(boy);
  //       return;
  //     }
  //   }
  //   ld_r8_r8(boy);
  //   return;
  // }

  // // x = 2
  // case 2: {
  //   alu_a_r8(boy);
  //   return;
  // }

  // // x = 3
  // case 3: {
  //   switch (z) {

  //   // z = 0
  //   case 0: {
  //     switch (y) {
  //     case 0 ... 3:
  //       ret_cc(boy);
  //       return;
  //     case 4:
  //       ld_0xFF00_n_A(boy);
  //       return;
  //     case 5:
  //       add_sp_d(boy);
  //       return;
  //     case 6:
  //       ld_A_0xFF00_n(boy);
  //       return;
  //     case 7:
  //       ld_hl_sp_d(boy);
  //       return;
  //     }
  //     break;
  //   }

  //   // z = 1
  //   case 1: {
  //     switch (q) {
  //     case 0:
  //       pop_r16(boy);
  //       return;
  //     case 1: {
  //       switch (p) {
  //       case 0:
  //         ret(boy);
  //         return;
  //       case 1:
  //         reti(boy);
  //         return;
  //       case 2:
  //         jp_hl(boy);
  //       case 3:
  //         ld_sp_hl(boy);
  //         return;
  //       }
  //     }
  //     }
  //     break;
  //   }

  //   // z = 2
  //   case 2: {
  //     switch (y) {
  //     case 0 ... 3:
  //       jp_cc_nn(boy);
  //       return;

  //     case 4:
  //       ld_0xFF00_C_A(boy);
  //       return;

  //     case 5:
  //       ld_nn_a(boy);
  //       return;

  //     case 6:
  //       ld_A_0xFF00_C(boy);
  //       return;

  //     case 7:
  //       ld_a_nn(boy);
  //       return;
  //     }
  //     break;
  //   }

  //   // z = 3
  //   case 3: {
  //     switch (y) {
  //     case 0:
  //       jp_nn(boy);
  //       return;

  //     case 6:
  //       di(boy);
  //       return;

  //     case 7:
  //       ei(boy);
  //       return;
  //     }
  //     break;
  //   }

  //   // z = 4
  //   case 4: {
  //     switch (y) {
  //     case 0 ... 3:
  //       call_cc_nn(boy);
  //       return;
  //     }
  //     break;
  //   }

  //   // z = 5
  //   case 5: {
  //     switch (q) {
  //     case 0:
  //       push_r16(boy);
  //       return;

  //     case 1: {
  //       switch (p) {
  //       case 0:
  //         call_nn(boy);
  //         return;
  //       }
  //       break;
  //     }
  //     }
  //     break;
  //   }

  //   // z = 6
  //   case 6: {
  //     alu_a_n(boy);
  //     return;
  //   }

  //   case 7: {
  //     rst(boy);
  //     return;
  //   }
  //   }
  // }
  // }
}
