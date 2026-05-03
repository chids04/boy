#include "ppu.h"
#include "boy.h"
#include "common.h"
#include "utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void init_ppu(PPU *ppu) {
  ppu->mode = PPU_MODE_2;
  ppu->cycle = 0;
  ppu->oam_offset = 0;
  ppu->sprite_buffer = calloc(10, sizeof(SPRITE));
  ppu->sprite_buffer_offset = 0;
}

// called every M cycle ( 4 T Cycles )
void handle_ppu(BOY *boy, int cycles) {
  boy->ppu.cycle += cycles * 4;

  if(boy->ppu.mode == PPU_MODE_2) {
    handle_oam_scan(boy);
  }

  set_mode(&boy->ppu);
  // handle state transitions
//s
  //
}

void set_mode(PPU *ppu) {
  if(ppu->mode == PPU_MODE_2 && ppu->cycle == 80) {
    ppu->mode = PPU_MODE_3;
    ppu->cycle = 0;
  } else if(ppu->mode == PPU_MODE_3 && ppu->cycle == 172) {
    ppu->mode = PPU_MODE_0;
    ppu->cycle = 0;

    // off by one cycle, since 87 not directly divisble by 4
  } else if(ppu->mode == PPU_MODE_0 && ppu->cycle == 88) {
    ppu->mode = PPU_MODE_1;
    ppu->cycle = 0;
  }
}

void handle_oam_scan(BOY *boy){
  // this is called every 4 T cycles but oam is searched every 2 T cycles
  // so we read 2 entries each time
  uint8_t offset1 = boy->ppu.oam_offset;
  uint8_t offset2 = boy->ppu.oam_offset + 1;

  SPRITE *entry1 = handle_oam_read(&boy->mmu, offset1);
  SPRITE *entry2 = handle_oam_read(&boy->mmu, offset2);

  if(to_sprite_buffer(boy, entry1)) {
    boy->ppu.sprite_buffer[boy->ppu.sprite_buffer_offset] = *entry1;
    boy->ppu.sprite_buffer_offset++;
  }

  if(to_sprite_buffer(boy, entry2)) {
    boy->ppu.sprite_buffer[boy->ppu.sprite_buffer_offset] = *entry2;
    boy->ppu.sprite_buffer_offset++;
  }

}

bool to_sprite_buffer(BOY *boy, SPRITE *sprite){
  if(
    sprite->x > 0 &&
    sprite->y <= boy->mmu.LY + 16 &&
    sprite->y + sprite_height(&boy->mmu) > boy->mmu.LY + 16 &&
    boy->ppu.sprite_buffer_offset < MAX_SPRITES
  ) {
      return true;
  }

  return false;
}

uint8_t sprite_height(MMU *mmu) {
  if(get_bit(mmu->LCDC, 2) == 0) {
    return 8;
  }

  return 16;
}
