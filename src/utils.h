#include <stdio.h>
#include <stdint.h>

uint8_t get_bit(uint16_t bytes, int n) {
    return (bytes >> n) & 1;
}
