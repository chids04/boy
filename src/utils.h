#include <stdlib.h>
#include <cstdint>
#include "cpu.h"

uint8_t upper_8(uint16_t instruction) {
    return instruction >> 8;
}

uint8_t lower8(uint16_t instruction) {
    return instruction & 0xFF;
}
