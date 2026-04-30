#pragma once
#include "common.h"



typedef enum PPU_MODE {
  PPU_MODE_0, // hblank
  PPU_MODE_1, // vblank
  PPU_MODE_2, // oam search
  PPU_MODE_3, // drawing
} PPU_MODE;

struct PPU {
  SPRITE* sprites;
  PPU_MODE mode;

  uint8_t oam_offset;
};

PPU ppu_init();

void handle_ppu(BOY *boy);
void handle_oam_scan(BOY *boy);
