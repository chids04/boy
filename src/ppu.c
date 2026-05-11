#include "ppu.h"
#include "boy.h"
#include "common.h"
#include "log.h"
#include "mmu.h"
#include "utils.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void init_ppu(PPU *ppu) {
  ppu->mode = PPU_MODE_2;
  ppu->cycle = 0;
  ppu->oam_offset = 0;
  ppu->sprite_buffer = calloc(10, sizeof(SPRITE));
  ppu->sprite_buffer_offset = 0;
  ppu->pixel_fetcher.x_offset = 0;
  ppu->


  ppu->pixel_fetcher.state = PixelFetcher_BG;
}

void mode3_init(PPU *ppu) {
  ppu->ppu_state.PPU_DRAW.mode_3_state = MODE_3_TILE_NUM_TILE_LOW;
}

// called every M cycle ( 4 T Cycles )
void handle_ppu(BOY *boy, int cycles) {
  boy->ppu.cycle += cycles * 4;

  if(boy->ppu.mode == PPU_MODE_2) {
    handle_oam_scan(boy);
  }
  else if(boy->ppu.mode == PPU_MODE_3) {
    handle_ppu_draw(boy);
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

void handle_ppu_draw(BOY *boy) {
  // first get the tile numner of the tile to render

  switch (boy->ppu.ppu_state.PPU_DRAW.mode_3_state) {
    case MODE_3_TILE_NUM_TILE_LOW:

    mode_3_tile_num_tile_high(boy);
    // update state after
    boy->ppu.ppu_state.PPU_DRAW.mode_3_state = MODE_3_FETCHER_TILE_HIGH_PUSH_FIFO;
    break;

    case MODE_3_FETCHER_TILE_HIGH_PUSH_FIFO:
    mode_3_tile_high_fifo(boy);
    // handle the state after here
    break;
  }
}

void check_if_window_next(BOY *boy) {
  if(
    get_bit(boy->mmu.LCDC, 5) == 1 &&
    boy->ppu.wy_crossed_ly &&
    boy->ppu.pixel_fetcher.x_offset >= boy->mmu.WX - 7
  )
  {
    boy->ppu.pixel_fetcher.state = PixelFetcher_WIN;
  }
}


void mode_3_tile_num_tile_high(BOY *boy) {
  uint16_t BG_MAP_ADDR;

  // if bit 3 of LCDC set then bg map $9C00-$9FFF is used, otherwise it uses the one at $9800-$9BFF.
  if(get_bit(boy->mmu.LCDC, 3) == 1) {
    BG_MAP_ADDR = 0x9C00;
  }
  else {
    BG_MAP_ADDR = 0x9800;
  }

  // if drawing the window, then get the tilemap for the window
  if (boy->ppu.pixel_fetcher.state == PixelFetcher_WIN) {
    if(get_bit(boy->mmu.LCDC, 6) == 1) {
        BG_MAP_ADDR = 0x9C00;
      }
    else {
      BG_MAP_ADDR = 0x9800;
    }
  }

  uint16_t x_offset;

  // add the x-offset
  if(boy->ppu.pixel_fetcher.state == PixelFetcher_WIN) {
    x_offset = boy->ppu.pixel_fetcher.x_offset;
  }
  else if(boy->ppu.pixel_fetcher.state == PixelFetcher_WIN){
    // add x offset and scroll ofdset and wrap
    x_offset = (boy->ppu.pixel_fetcher.x_offset + (boy->mmu.SCX / 8)) & 0x1F;
  }
  else {
    log_warn("tile x-offset for pixel state %d not implemented", boy->ppu.pixel_fetcher.state);
    return;

  }

  uint16_t y_offset;

  // add y offset
  if(boy->ppu.pixel_fetcher.state == PixelFetcher_WIN) {
    y_offset = 32 * (boy->ppu.pixel_fetcher.window_line / 8);
  }
  else if(boy->ppu.pixel_fetcher.state == PixelFetcher_WIN){
    y_offset = 32 * (((boy->mmu.LY + boy->mmu.SCY) & 0xFF) / 8);
  }
  else {
    log_warn("tile x-offset for pixel state %d not implemented", boy->ppu.pixel_fetcher.state);
    return;

  }

  // ensure offset stays within the tilemap region
  BG_MAP_ADDR += (x_offset + y_offset) & 0x3FF;

  // get the tile number in the tilemap
  uint8_t tile_num = read_byte_no_tick(boy, BG_MAP_ADDR);

  uint16_t base_tile_address = get_tile_base_address(&boy->mmu, tile_num);

  uint8_t tile_offset;
  if(boy->ppu.pixel_fetcher.state == PixelFetcher_WIN) {
    tile_offset = 2 * (boy->ppu.pixel_fetcher.window_line % 8);
  }
  else {
    tile_offset = 2 * ((boy->mmu.LY + boy->mmu.SCY) % 8);
  }

  boy->ppu.ppu_state.PPU_DRAW.tile_address = base_tile_address + tile_offset;
  boy->ppu.ppu_state.PPU_DRAW.tile_low = read_byte_no_tick(boy, boy->ppu.ppu_state.PPU_DRAW.tile_address);
}

void mode_3_tile_high_fifo(BOY *boy){
  uint8_t tile_high = read_byte_no_tick(boy, boy->ppu.ppu_state.PPU_DRAW.tile_address + 1);

  for(int i=0; i<8; ++i) {
    uint8_t color_idx = (get_bit(boy->ppu.ppu_state.PPU_DRAW.tile_low, i) << 1 ) | get_bit(boy->ppu.ppu_state.PPU_DRAW.tile_low, i);
  }

}

uint16_t get_tile_base_address(MMU *mmu, uint8_t tile_number) {
  if(get_bit(mmu->LCDC, 4) == 1) {
    return (uint8_t)TILE_8000 + (tile_number * 16);
  }

  return (uint8_t)TILE_8800 + ((int8_t)tile_number * 16);

}
