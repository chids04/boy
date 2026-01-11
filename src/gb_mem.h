#pragma once

#include <stdint.h>

typedef struct {
    uint8_t memory[0x10000]; // 0xFFFF is 65535, so 0x10000 bytes for a full 64KB address space
} GBMem;

#ifdef __cplusplus
extern "C" {
#endif

extern GBMem gb_mem;

uint8_t read_byte(uint16_t address);
uint16_t read_word(uint16_t address);
void write_word(uint16_t address, uint16_t data);
void write_byte(uint16_t address, uint8_t data);

#ifdef __cplusplus
}
#endif
