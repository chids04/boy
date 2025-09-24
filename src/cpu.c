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

    cpu->cycles = 3;
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

void ld_r16_imm16(CPU *cpu) {

    uint16_t imm16 = read_word(cpu->PC);
    uint8_t y_mask = 0b111 << 3;

    // p is top 2 msb of y (bits 5 - 4 of opcode)
    uint8_t p = (cpu->opcode & y_mask) >> 4;

    write_r16(imm16, p, cpu);

    cpu->cycles = 3;

}

void add_hl_r16(CPU *cpu) {
    // p is top 2 msb of y (bits 5 - 4 of opcode)
    //
    uint8_t y_mask = 0b111 << 3;
    uint8_t p = (cpu->opcode & y_mask) >> 4;

    uint16_t data = read_r16(p, cpu, false);

    uint16_t hl = read_r16(REG_HL, cpu, false);
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
    uint16_t addr = read_r16(REG_HL, cpu, false);
    write_word(addr, cpu->A);
    addr -= 1;
    write_r16(addr, REG_HL, cpu);

    cpu->cycles = 2;
}

void ld_a_bc(CPU* cpu) {
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

    write_r16(address + 1, REG_BC, cpu);
}

void ld_a_hld(CPU *cpu) {
    // load from memory location specified by hl into A, then decrement hl.
    uint8_t address = read_r16(REG_HL, cpu, false);
    uint8_t data = read_byte(address);
    cpu->A = data;

    write_r16(address - 1, REG_BC, cpu);
}

void inc_r8(CPU *cpu){
    uint8_t mask = 0b111 << 3;
    uint8_t y = (cpu->opcode & mask) >> 3;

    uint8_t q = y >> 2;

    uint8_t data = read_r8(q, cpu);

    // first check for half carries
    add8_half_carry(data, cpu);
    data += 1;

    // check zero flag and right to registers
    write_r8(data, q, cpu);

}

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

    printf("read_r8: invalid 8 bit register, returning 0\n");
    return 0x0;
}

uint16_t read_r16(uint8_t idx, CPU *cpu, bool has_sp) {
    switch (idx) {
        case REG_BC:
            return (cpu->B << 8) | cpu->C;

        case REG_DE:
            return (cpu->D << 8) | cpu->E;

        case REG_HL:
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

void write_r16(uint16_t data, uint8_t reg, CPU *cpu){
    switch (reg) {
        case REG_BC:
            cpu->B = data >> 8;
            cpu->C = data & 0xFF;
            break;

        case REG_DE:
            cpu->D = data >> 8;
            cpu->E = data & 0xFF;
            break;

        case REG_HL:
            cpu->H = data >> 8;
            cpu->L = data & 0xFF;
            break;

        // todo: add support for AF reg too
        case REG_SP:
            cpu->SP = data;
            break;
    }
}

void write_r8(uint8_t data, uint8_t reg, CPU *cpu){
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
                uint16_t addr = read_r16(REG_HL, cpu, false);
                write_byte(addr, data);

            }
            break;
        case REG_A:
            cpu->A = data;
    }
}


void add8_half_carry(const uint8_t value, CPU *cpu) {
  uint8_t lower = value & 0b1111;
  uint8_t mask = 1 << 5; // at bit 5 (0 index)

  if (lower == 0b1111) {
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
  uint8_t mask = 1 << 7; // at bit 7 (0 index)

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
