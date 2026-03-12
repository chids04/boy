#pragma once

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

static inline uint8_t get_bit(uint16_t bytes, int n) {
  return (bytes >> n) & 1;
}

static inline uint8_t get_bit_range(uint8_t byte, int high, int low) {
  // this creates a mask that is high - low + 1 bits wide
  assert(high > low);

  uint8_t mask = (1U << ((high - low) + 1)) - 1;

  return (byte >> low) & mask;
}
