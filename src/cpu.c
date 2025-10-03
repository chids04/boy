#include "cpu.h"
#include "gb_mem.h"

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

  int8_t d = (int8_t)read_imm8(cpu);
  uint8_t cond = get_y(cpu->opcode) - 4;

  printf("jr_cc_d: current condition %d\n", cond);

  if (condition(cond, cpu)) {
    printf(
        "jr_cc_d: condition met, PC before displacement: %d displacement: %d\n",
        cpu->PC, d);
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

  write_r16(imm16, p, cpu);

  cpu->cycles = 3;
}

void add_hl_r16(CPU *cpu) {
  // p is top 2 msb of y (bits 5 - 4 of opcode)
  uint8_t p = get_p(cpu->opcode);

  uint16_t data = read_r16(p, cpu, false);

  uint16_t hl = read_r16(REG_HL_16, cpu, false);
  write_r16(data + hl, 2, cpu);

  cpu->cycles = 2;
}

void ld_bc_a(CPU *cpu) {
  // mem[bc] = A

  uint16_t addr = read_r16(REG_BC, cpu, false);
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

  uint16_t addr = read_r16(REG_DE, cpu, false);
  write_word(addr, cpu->A);

  cpu->cycles = 2;
}

void ld_hld_a(CPU *cpu) {
  // mem[HL] = A, HL--
  uint16_t addr = read_r16(REG_HL_16, cpu, false);
  write_word(addr, cpu->A);
  addr -= 1;
  write_r16(addr, REG_HL_16, cpu);

  cpu->cycles = 2;
}

void ld_a_bc(CPU *cpu) {
  // a = memory[bc].

  uint8_t address = read_r16(REG_BC, cpu, false);
  uint8_t data = read_byte(address);
  cpu->A = data;

  cpu->cycles = 2;
}

void ld_a_hli(CPU *cpu) {
  // load a from memory location specified by hl, then increment hl.

  uint8_t address = read_r16(REG_BC, cpu, false);
  uint8_t data = read_byte(address);
  cpu->A = data;

  write_r16(address + 1, REG_BC, cpu);
}

void ld_a_de(CPU *cpu) {
  // load a from memory location specified by de.

  uint8_t address = read_r16(REG_DE, cpu, false);
  uint8_t data = read_byte(address);
  cpu->A = data;

  write_r16(address + 1, REG_DE, cpu);
}

void ld_a_hld(CPU *cpu) {
  // load from memory location specified by hl into A, then decrement hl.
  uint16_t address = read_r16(REG_HL_16, cpu, false);
  uint8_t data = read_byte(address);
  cpu->A = data;

  write_r8(address - 1, REG_A, cpu);
}

void inc_r8(CPU *cpu) {
  uint8_t y = get_y(cpu->opcode);

  uint8_t q = y >> 2;

  uint8_t data = read_r8(q, cpu);

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

  write_r8(data, q, cpu);

  cpu->cycles = 1;

  // Add extra cycle for HL register memory operation
  if (q == REG_HL_8) {
    cpu->cycles += 1;
  }
}
void dec_r8(CPU *cpu) {
  uint8_t y = get_y(cpu->opcode);

  uint8_t q = y >> 2;

  uint8_t data = read_r8(q, cpu);

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
  write_r8(data, q, cpu);

  cpu->cycles = 1;

  // Add extra cycle for HL register memory operation
  if (q == REG_HL_8) {
    cpu->cycles += 1;
  }
}

void inc_r16(CPU *cpu) {
  uint8_t p = get_p(cpu->opcode);

  uint16_t data = read_r16(p, cpu, false);

  write_r16(data += 1, p, cpu);
  cpu->cycles = 2;
}

void dec_r16(CPU *cpu) {
  uint8_t p = get_p(cpu->opcode);

  uint16_t data = read_r16(p, cpu, false);

  write_r16(data -= 1, p, cpu);
  cpu->cycles = 2;
}

void ld_r8_imm8(CPU *cpu) {
  // write the immediate next byte to 8 bit register
  uint8_t y = get_y(cpu->opcode);

  uint8_t imm8 = read_imm8(cpu);
  write_r8(imm8, y, cpu);

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
  // set carry to lsb and right circle rotate A register

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
  // set carry to msb and left circle rotate A register
  uint8_t msb = (cpu->A & (1 << 7)) >> 7;

  cpu->A = (cpu->A << 1);
  cpu->A <<= 1;

  if (is_flag_set(cpu, FLAG_H)) {
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
  // left rotate A register through C register

  uint8_t lsb = cpu->A & 1;
  uint8_t reg_a = read_r8(REG_A, cpu);

  cpu->A >>= 1;

  if (is_flag_set(cpu, FLAG_H)) {
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

  uint8_t right_data = read_r8(z, cpu);
  write_r8(right_data, y, cpu);

  cpu->cycles = 1;

  // Add extra cycle for HL register memory operation
  if (y == REG_HL_8 || z == REG_HL_8) {
    cpu->cycles += 1;
  }
}

void halt(CPU *cpu) { printf("halt(): instruction not implemented\n"); }

void alu_r8(CPU *cpu) {}

void add_a_r8(CPU *cpu) {
  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_r8(z, cpu);

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

void addc_a_r8(CPU *cpu) {
  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_r8(z, cpu);

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
  uint8_t data = read_r8(z, cpu);


  set_flag(cpu, FLAG_N);
  sub8_half_carry(data, cpu);
  sub8_carry(data, cpu);

  cpu->A -= data;

  if (cpu->A == 0) {
    set_flag(cpu, FLAG_Z);
  }
  else {
    clear_flag(cpu, FLAG_Z);
  }

  cpu->cycles = 1;

  if (z == REG_HL_8) cpu->cycles+=1;
}

void subc_a_r8(CPU *cpu) {
  // A = A - r8 - carry

  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_r8(z, cpu);

  uint8_t carry = (uint8_t)is_flag_set(cpu, FLAG_C);
  data += carry;

  set_flag(cpu, FLAG_N);
  sub8_half_carry(data, cpu);
  sub8_carry(data, cpu);

  cpu->A -= data;

  if (cpu->A == 0) {
    set_flag(cpu, FLAG_Z);
  }
  else {
    clear_flag(cpu, FLAG_Z);
  }

  cpu->cycles = 1;

  if (z == REG_HL_8) cpu->cycles+=1;

}

void and_a_r8(CPU *cpu){

  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_r8(z, cpu);

  cpu->A &= data;

  clear_flag(cpu, FLAG_N);
  clear_flag(cpu, FLAG_C);
  set_flag(cpu, FLAG_H);
  
  if(cpu->A == 0) {
    set_flag(cpu, FLAG_Z);
  }
  else {
    clear_flag(cpu, FLAG_Z);
  }

  cpu->cycles = 1;

  if (z == REG_HL_8) cpu->cycles+=1;


}

void xor_a_r8(CPU *cpu){
  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_r8(z, cpu);

  cpu->A ^= data;
  clear_flag(cpu, FLAG_N);
  clear_flag(cpu, FLAG_H);
  clear_flag(cpu, FLAG_C);

  if(cpu->A == 0) {
    set_flag(cpu, FLAG_Z);
  }
  else {
    clear_flag(cpu, FLAG_Z);
  }

  cpu->cycles = 1;

  if (z == REG_HL_8) cpu->cycles+=1;
}


void or_a_r8(CPU *cpu) {

  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_r8(z, cpu);

  cpu->A |= data;

  clear_flag(cpu, FLAG_N);
  clear_flag(cpu, FLAG_H);
  clear_flag(cpu, FLAG_C);

  if(cpu->A == 0) {
    set_flag(cpu, FLAG_Z);
  }
  else {
    clear_flag(cpu, FLAG_Z);
  }

  cpu->cycles = 1;

  if (z == REG_HL_8) cpu->cycles+=1;
}


void cp_r_r8(CPU *cpu) {

  uint8_t z = get_z(cpu->opcode);
  uint8_t data = read_r8(z, cpu);


  set_flag(cpu, FLAG_N);
  sub8_half_carry(data, cpu);
  sub8_carry(data, cpu);

  uint8_t res = cpu->A - data;

  if (res == 0) {
    set_flag(cpu, FLAG_Z);
  }
  else {
    clear_flag(cpu, FLAG_Z);
  }

  cpu->cycles = 1;

  if (z == REG_HL_8) cpu->cycles+=1;
}





// CPU UTILS BEGINbibin/
//
//
//

void decode_instruction(CPU *cpu) {
  cpu->opcode = read_byte(cpu->PC++);

  uint8_t x = get_x(cpu->opcode);

  switch (x) {

  // block 0
  case 0: {
    uint8_t z = get_z(cpu->opcode);
    uint8_t y = get_y(cpu->opcode);

    switch (z) {
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

    case 1: {
      uint8_t q = get_q(cpu->opcode);
      switch (q) {
      case 0:
        ld_r16_imm16(cpu);
        return;
      case 1:
        add_hl_r16(cpu);
        return;
      }
    }

    case 2: {
      uint8_t q = get_q(cpu->opcode);
      uint8_t p = get_p(cpu->opcode);
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
    }
  }
  }
}
