#pragma once
#include "common.h"

#define MAX_SPRITES 10


typedef enum PPU_MODE {
  PPU_MODE_0, // hblank
  PPU_MODE_1, // vblank
  PPU_MODE_2, // oam search
  PPU_MODE_3, // drawing
} PPU_MODE;


struct PPU {
  SPRITE* sprite_buffer;
  size_t sprite_buffer_offset;
  PPU_MODE mode;
  int cycle;

  uint8_t oam_offset;
};

void init_ppu(PPU *ppu);

void handle_ppu(BOY *boy, int cycles);
void handle_oam_scan(BOY *boy);
void set_mode(PPU *ppu);
bool to_sprite_buffer(BOY *boy, SPRITE *sprite);
