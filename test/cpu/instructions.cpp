#include <gtest/gtest.h>
#include "cpu-test.h"
#include "cpu.h"
#include "gb_mem.h"

// flag bit masks (from cpu.h comments)
#define FLAG_Z (1 << 7) // zero flag
#define FLAG_N (1 << 6) // subtract flag
#define FLAG_H (1 << 5) // half carry flag
#define FLAG_C (1 << 4) // carry flag

// helper macros for flags
#define SET_FLAG(cpu_ptr, flag) ((cpu_ptr)->F |= (flag))
#define CLEAR_FLAG(cpu_ptr, flag) ((cpu_ptr)->F &= ~(flag))
#define IS_FLAG_SET(cpu_ptr, flag) (((cpu_ptr)->F & (flag)) != 0)


TEST_F(CPUTest, LOAD_N16_SP) {

    uint8_t instr = 0b00001000;
    write_byte(0x0, instr);

    uint16_t n16 = 0xBE42;
    write_word(0x1, n16);

    uint16_t data = 0xCDAB;
    cpu->SP = data;
    decode_instruction(cpu);

    uint16_t loaded = read_word(0xBE42);

    int expected_cycles = 20; // adjusted based on common gameboy timings for this op

    ASSERT_EQ(data, loaded);
    ASSERT_EQ(cpu->cycles, expected_cycles);
}

// test for jr d (unconditional jump)
TEST_F(CPUTest, JR_D) {
    // initial pc is 0 from memset in fixture set up
    uint16_t initial_pc = cpu->PC;
    uint8_t opcode = 0b00011000; // jr d (0x18)
    int8_t displacement = 0x05; // jump forward 5 bytes

    write_byte(initial_pc, opcode);
    write_byte(initial_pc + 1, (uint8_t)displacement);

    decode_instruction(cpu);

    // jr d instruction is 2 bytes (opcode + displacement)
    // new pc = (initial pc + 2) + displacement
    ASSERT_EQ(cpu->PC, initial_pc + 2 + displacement);
    // cycles for jr d: 12
    ASSERT_EQ(cpu->cycles, 12);
}

// jr nz, d (jump if not zero)
TEST_F(CPUTest, JR_NZ_D_Taken) {
    uint16_t initial_pc = cpu->PC; // starts at 0
    uint8_t opcode = 0b00100000; // jr nz, d (0x20)
    int8_t displacement = 0x10; // jump forward 16 bytes

    // condition: z flag is clear (not zero) -> jump should be taken
    CLEAR_FLAG(cpu, FLAG_Z);

    write_byte(initial_pc, opcode);
    write_byte(initial_pc + 1, (uint8_t)displacement);

    decode_instruction(cpu);

    ASSERT_EQ(cpu->PC, initial_pc + 2 + displacement);
    // cycles for jr cc, d (taken): 12
    ASSERT_EQ(cpu->cycles, 12);
}

TEST_F(CPUTest, JR_NZ_D_NotTaken) {
    uint16_t initial_pc = cpu->PC; // starts at 0
    uint8_t opcode = 0b00100000; // jr nz, d (0x20)
    int8_t displacement = 0x10; // jump forward 16 bytes

    // condition: z flag is set (zero) -> jump should not be taken
    SET_FLAG(cpu, FLAG_Z);

    write_byte(initial_pc, opcode);
    write_byte(initial_pc + 1, (uint8_t)displacement);

    decode_instruction(cpu);

    ASSERT_EQ(cpu->PC, initial_pc + 2); // pc should just advance past the instruction
    // cycles for jr cc, d (not taken): 8
    ASSERT_EQ(cpu->cycles, 8);
}

// jr z, d (jump if zero)
TEST_F(CPUTest, JR_Z_D_Taken) {
    uint16_t initial_pc = cpu->PC; // starts at 0
    uint8_t opcode = 0b00101000; // jr z, d (0x28)
    int8_t displacement = -0x05; // jump backward 5 bytes

    // condition: z flag is set (zero) -> jump should be taken
    SET_FLAG(cpu, FLAG_Z);

    write_byte(initial_pc, opcode);
    write_byte(initial_pc + 1, (uint8_t)displacement);

    decode_instruction(cpu);

    ASSERT_EQ(cpu->PC, initial_pc + 2 + displacement);
    ASSERT_EQ(cpu->cycles, 12);
}

TEST_F(CPUTest, JR_Z_D_NotTaken) {
    uint16_t initial_pc = cpu->PC; // starts at 0
    uint8_t opcode = 0b00101000; // jr z, d (0x28)
    int8_t displacement = -0x05; // jump backward 5 bytes

    // condition: z flag is clear (not zero) -> jump should not be taken
    CLEAR_FLAG(cpu, FLAG_Z);

    write_byte(initial_pc, opcode);
    write_byte(initial_pc + 1, (uint8_t)displacement);

    decode_instruction(cpu);

    ASSERT_EQ(cpu->PC, initial_pc + 2);
    ASSERT_EQ(cpu->cycles, 8);
}

// jr nc, d (jump if not carry)
TEST_F(CPUTest, JR_NC_D_Taken) {
    uint16_t initial_pc = cpu->PC; // starts at 0
    uint8_t opcode = 0b00110000; // jr nc, d (0x30)
    int8_t displacement = 0x20; // jump forward 32 bytes

    // condition: c flag is clear (no carry) -> jump should be taken
    CLEAR_FLAG(cpu, FLAG_C);

    write_byte(initial_pc, opcode);
    write_byte(initial_pc + 1, (uint8_t)displacement);

    decode_instruction(cpu);

    ASSERT_EQ(cpu->PC, initial_pc + 2 + displacement);
    ASSERT_EQ(cpu->cycles, 12);
}

TEST_F(CPUTest, JR_NC_D_NotTaken) {
    uint16_t initial_pc = cpu->PC; // starts at 0
    uint8_t opcode = 0b00110000; // jr nc, d (0x30)
    int8_t displacement = 0x20; // jump forward 32 bytes

    // condition: c flag is set (carry) -> jump should not be taken
    SET_FLAG(cpu, FLAG_C);

    write_byte(initial_pc, opcode);
    write_byte(initial_pc + 1, (uint8_t)displacement);

    decode_instruction(cpu);

    ASSERT_EQ(cpu->PC, initial_pc + 2);
    ASSERT_EQ(cpu->cycles, 8);
}

// jr c, d (jump if carry)
TEST_F(CPUTest, JR_C_D_Taken) {
    uint16_t initial_pc = cpu->PC; // starts at 0
    uint8_t opcode = 0b00111000; // jr c, d (0x38)
    int8_t displacement = -0x10; // jump backward 16 bytes

    // condition: c flag is set (carry) -> jump should be taken
    SET_FLAG(cpu, FLAG_C);

    write_byte(initial_pc, opcode);
    write_byte(initial_pc + 1, (uint8_t)displacement);

    decode_instruction(cpu);

    ASSERT_EQ(cpu->PC, initial_pc + 2 + displacement);
    ASSERT_EQ(cpu->cycles, 12);
}

TEST_F(CPUTest, JR_C_D_NotTaken) {
    uint16_t initial_pc = cpu->PC; // starts at 0
    uint8_t opcode = 0b00111000; // jr c, d (0x38)
    int8_t displacement = -0x10; // jump backward 16 bytes

    // condition: c flag is clear (no carry) -> jump should not be taken
    CLEAR_FLAG(cpu, FLAG_C);

    write_byte(initial_pc, opcode);
    write_byte(initial_pc + 1, (uint8_t)displacement);

    decode_instruction(cpu);

    ASSERT_EQ(cpu->PC, initial_pc + 2);
    ASSERT_EQ(cpu->cycles, 8);
}


TEST_F(CPUTest, LD_BC_IMM16) {
    uint16_t initial_pc = cpu->PC;
    uint8_t opcode = 0b00000001; // ld bc, imm16 (0x01)
    uint16_t immediate_value = 0x1234;

    write_byte(initial_pc, opcode);
    write_word(initial_pc + 1, immediate_value);

    decode_instruction(cpu);

    // verify bc register pair
    uint16_t bc_result = (cpu->B << 8) | cpu->C;
    ASSERT_EQ(bc_result, immediate_value);
    ASSERT_EQ(cpu->PC, initial_pc + 3); // opcode (1 byte) + imm16 (2 bytes)
    ASSERT_EQ(cpu->cycles, 12);
}

TEST_F(CPUTest, LD_DE_IMM16) {
    uint16_t initial_pc = cpu->PC;
    uint8_t opcode = 0b00010001; // ld de, imm16 (0x11)
    uint16_t immediate_value = 0xABCD;

    write_byte(initial_pc, opcode);
    write_word(initial_pc + 1, immediate_value);

    decode_instruction(cpu);

    uint16_t de_result = (cpu->D << 8) | cpu->E;
    ASSERT_EQ(de_result, immediate_value);
    ASSERT_EQ(cpu->PC, initial_pc + 3);
    ASSERT_EQ(cpu->cycles, 12);
}

TEST_F(CPUTest, LD_HL_IMM16) {
    uint16_t initial_pc = cpu->PC;
    uint8_t opcode = 0b00100001; // ld hl, imm16 (0x21)
    uint16_t immediate_value = 0xFEDC;

    write_byte(initial_pc, opcode);
    write_word(initial_pc + 1, immediate_value);

    decode_instruction(cpu);

    // verify hl register pair
    uint16_t hl_result = (cpu->H << 8) | cpu->L;
    ASSERT_EQ(hl_result, immediate_value);
    ASSERT_EQ(cpu->PC, initial_pc + 3);
    ASSERT_EQ(cpu->cycles, 12);
}

TEST_F(CPUTest, LD_SP_IMM16) {
    uint16_t initial_pc = cpu->PC;
    uint8_t opcode = 0b00110001; // ld sp, imm16 (0x31)
    uint16_t immediate_value = 0x9876;

    write_byte(initial_pc, opcode);
    write_word(initial_pc + 1, immediate_value);

    decode_instruction(cpu);

    // verify sp register
    ASSERT_EQ(cpu->SP, immediate_value);
    ASSERT_EQ(cpu->PC, initial_pc + 3);
    ASSERT_EQ(cpu->cycles, 12);
}
