#pragma once

#include <stdint.h>   // For uint8_t, uint16_t, etc.
#include <stdbool.h>  // For bool, true, false
#include <stddef.h>   // For NULL and size_t

typedef struct BOY BOY;
typedef struct CPU CPU;
typedef struct MMU MMU;
typedef struct PPU PPU;
typedef struct TIMERS TIMERS;
typedef struct PAD PAD;
typedef struct SPRITE SPRITE;

typedef enum {
    GB_OK = 0,
    GB_ERROR_ROM_LOAD,
    GB_ERROR_INVALID_OPCODE,
    GB_ERROR_MEMORY_ACCESS
} gb_status;
