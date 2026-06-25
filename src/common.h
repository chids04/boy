#pragma once

#include <stdbool.h> // For bool, true, false
#include <stddef.h>  // For NULL and size_t
#include <stdint.h>  // For uint8_t, uint16_t, etc.

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

typedef enum {
  EVENT_NONE = 1,
  EVENT_FRAME_READY = 1 << 1,
  EVENT_AUDIO_READY = 1 << 2,
} BoyEvent;
