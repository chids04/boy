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
  int cycle;

  uint8_t oam_offset;
};

void init_ppu(PPU *ppu);

void handle_ppu(BOY *boy, int cycles);
void handle_oam_scan(BOY *boy);
void set_mode(PPU *ppu);
