#include <stdint.h>

uint8_t memory[0xFFFF];

uint8_t read_byte(uint16_t address) {
    return memory[address];
}

uint16_t read_word(uint16_t address) {
    uint8_t low = memory[address];
    uint8_t high = memory[address+1];

    return (high << 8) | low;
}

void write_word(uint16_t address, uint16_t data) {
    // need to store little endian (lsb in lower address)
    memory[address] = data & 0x00FF;
    memory[address+1] = data >> 8;
}
