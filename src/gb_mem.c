#include "gb_mem.h"

GBMem gb_mem;

uint8_t read_byte(uint16_t address) {
    return gb_mem.memory[address];
}

uint16_t read_word(uint16_t address) {
    uint8_t low = gb_mem.memory[address];
    uint8_t high = gb_mem.memory[address+1];

    return (high << 8) | low;
}

void write_word(uint16_t address, uint16_t data) {
    // store little endian (lsb in lower address)
    gb_mem.memory[address] = data & 0x00FF;
    gb_mem.memory[address+1] = data >> 8;
}

void write_byte(uint16_t address, uint8_t data) {
    gb_mem.memory[address] = data;
}
