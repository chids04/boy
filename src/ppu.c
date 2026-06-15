#include "ppu.h"
#include "boy.h"
#include "common.h"
#include "log.h"
#include "mmu.h"
#include "ppu_queue.h"
#include "utils.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void init_ppu(PPU *ppu) {
  ppu->ppu_mode = PPU_MODE_2;
  ppu->dots = 0;
  ppu->oam_offset = 0;
  ppu->sprite_buffer = calloc(10, sizeof(SPRITE));
  ppu->sprite_buffer_offset = 0;
  ppu->pixel_fetcher.x_offset = 0;
  ppu->pixel_fetcher.state = PixelFetcher_BG;

  ppu_queue_init(&ppu->background_fifo, FIFO_SIZE);
  ppu_queue_init(&ppu->sprite_fifo, FIFO_SIZE);
}

void mode3_init(PPU *ppu) {
  ppu->ppu_mode = PPU_MODE_3;
  ppu->ppu_state.PPU_DRAW.mode_3_state = MODE_3_TILE_NUM;
  ppu->ppu_state.PPU_DRAW.tile_low = 0;
  ppu->ppu_state.PPU_DRAW.tile_high = 0;
  ppu->ppu_state.PPU_DRAW.tile_address = 0;
  ppu->ppu_state.PPU_DRAW.scx_delay = 0;
  ppu->ppu_state.PPU_DRAW.dot_delay = 0;

  // each step of pixel fetching takes two steps
  // this variable gets decremented each call to hanlde_ppu_draw()
  // cycle 1 = 1 - 1 = 0, do no work
  // cycle 2 = 0, do fetcher work so it's ready for the 3rd cycle, reset cycles
  // remaining, update fetcher state cycle 3 = work from prev cycle availiable
  // by cycle 3
  reset_fetcher_cycles(ppu);
}

void reset_fetcher_cycles(PPU *ppu) { ppu->pixel_fetcher.cycles_remaining = 1; }

// called every M cycle ( 4 T Cycles )
void handle_ppu(BOY *boy, int dots) {
  boy->ppu.dots += dots;

  if (boy->ppu.ppu_mode == PPU_MODE_2) {
    handle_oam_scan(boy);
  } else if (boy->ppu.ppu_mode == PPU_MODE_3) {
    handle_ppu_draw(boy);
  } else if (boy->ppu.ppu_mode == PPU_MODE_0) {
  }

  set_mode(&boy->ppu);
}

void set_mode(PPU *ppu) {
  if (ppu->ppu_mode == PPU_MODE_2 && ppu->dots == 80) {
    // init ppu mode 3 state
    mode3_init(ppu);
    ppu->dots = 0;
  } else if ((ppu->ppu_mode == PPU_MODE_3) &&
             (ppu->dots == 172 + ppu->ppu_state.PPU_DRAW.dot_delay)) {
    ppu->ppu_mode = PPU_MODE_0;
    ppu->dots = 0;

  } else if ((ppu->ppu_mode == PPU_MODE_0) &&
             (ppu->dots == 87 - ppu->ppu_state.PPU_DRAW.dot_delay)) {
    ppu->ppu_mode = PPU_MODE_1;
    ppu->dots = 0;
  }
}

void handle_oam_scan(BOY *boy) {
  // this is called every 4 T cycles but oam is searched every 2 T cycles
  // so we read 2 entries each time
  uint8_t offset1 = boy->ppu.oam_offset;
  uint8_t offset2 = boy->ppu.oam_offset + 1;

  SPRITE *entry1 = handle_oam_read(&boy->mmu, offset1);
  SPRITE *entry2 = handle_oam_read(&boy->mmu, offset2);

  if (to_sprite_buffer(boy, entry1)) {
    boy->ppu.sprite_buffer[boy->ppu.sprite_buffer_offset] = *entry1;
    boy->ppu.sprite_buffer_offset++;
  }

  if (to_sprite_buffer(boy, entry2)) {
    boy->ppu.sprite_buffer[boy->ppu.sprite_buffer_offset] = *entry2;
    boy->ppu.sprite_buffer_offset++;
  }
}

bool to_sprite_buffer(BOY *boy, SPRITE *sprite) {
  if (sprite->y <= boy->mmu.LY + 16 &&
      sprite->y + sprite_height(&boy->mmu) > boy->mmu.LY + 16 &&
      boy->ppu.sprite_buffer_offset < MAX_SPRITES) {
    return true;
  }

  return false;
}

uint8_t sprite_height(MMU *mmu) {
  if (get_bit(mmu->LCDC, 2) == 0) {
    return 8;
  }

  return 16;
}

void handle_ppu_draw(BOY *boy) {
  // push pixels in queue
  mode_3_push(boy);

  if (boy->ppu.pixel_fetcher.cycles_remaining > 0) {
    boy->ppu.pixel_fetcher.cycles_remaining--;
  } else {
    // do all work on second dot;
    switch (boy->ppu.ppu_state.PPU_DRAW.mode_3_state) {
    case MODE_3_TILE_NUM:
      boy->ppu.ppu_state.PPU_DRAW.mode_3_state = mode_3_tile_num(boy);
      reset_fetcher_cycles(&boy->ppu);
      break;

    case MODE_3_TILE_LOW:
      boy->ppu.ppu_state.PPU_DRAW.mode_3_state = mode_3_tile_low(boy);
      reset_fetcher_cycles(&boy->ppu);
      break;

    case MODE_3_TILE_HIGH:
      boy->ppu.ppu_state.PPU_DRAW.mode_3_state = mode_3_tile_high(boy);
      reset_fetcher_cycles(&boy->ppu);
      break;

    case MODE_3_FIFO:
      boy->ppu.ppu_state.PPU_DRAW.mode_3_state = mode_3_fifo(boy);
      reset_fetcher_cycles(&boy->ppu);
      break;
    }
  }
}

void check_if_window_next(BOY *boy) {
  if (get_bit(boy->mmu.LCDC, 5) == 1 && boy->ppu.wy_crossed_ly &&
      boy->ppu.pixel_fetcher.x_offset >= boy->mmu.WX - 7) {
    boy->ppu.pixel_fetcher.state = PixelFetcher_WIN;
  }
}

MODE_3_STATE mode_3_tile_num(BOY *boy) {
  uint16_t BG_MAP_ADDR;

  // if bit 3 of LCDC set then bg map $9C00-$9FFF is used, otherwise it uses the
  // one at $9800-$9BFF.

  if (get_bit(boy->mmu.LCDC, 3) == 1) {
    BG_MAP_ADDR = 0x9C00;
  } else {
    BG_MAP_ADDR = 0x9800;
  }

  // if drawing the window, then get the tilemap for the window
  if (boy->ppu.pixel_fetcher.state == PixelFetcher_WIN) {
    if (get_bit(boy->mmu.LCDC, 6) == 1) {
      BG_MAP_ADDR = 0x9C00;
    } else {
      BG_MAP_ADDR = 0x9800;
    }
  }

  uint16_t x_offset;

  // add the x-offset
  if (boy->ppu.pixel_fetcher.state == PixelFetcher_BG) {
    x_offset = (boy->ppu.pixel_fetcher.x_offset + (boy->mmu.SCX / 8)) & 0x1F;
  } else if (boy->ppu.pixel_fetcher.state == PixelFetcher_WIN) {
    // window does not scroll
    x_offset = boy->ppu.pixel_fetcher.x_offset;
  } else {

    log_error("tile y-offset for objects %d not implemented",
              boy->ppu.pixel_fetcher.state);

    exit(1);
  }

  // add y offset
  uint16_t y_offset;

  if (boy->ppu.pixel_fetcher.state == PixelFetcher_WIN) {
    y_offset = 32 * (boy->ppu.pixel_fetcher.window_line / 8);

  } else if (boy->ppu.pixel_fetcher.state == PixelFetcher_BG) {
    y_offset = 32 * (((boy->mmu.LY + boy->mmu.SCY) & 0xFF) / 8);

  } else {
    log_error("tile y-offset for objects %d not implemented",
              boy->ppu.pixel_fetcher.state);
    exit(1);
  }

  // ensure offset stays within the tilemap region
  BG_MAP_ADDR += (x_offset + y_offset) & 0x3FF;

  // get the tile number (x offset) in the tilemap
  boy->ppu.ppu_state.PPU_DRAW.tile_num = read_byte_no_tick(boy, BG_MAP_ADDR);

  return MODE_3_TILE_LOW;
}

MODE_3_STATE mode_3_tile_low(BOY *boy) {

  uint16_t base_tile_address =
      get_tile_base_address(&boy->mmu, boy->ppu.ppu_state.PPU_DRAW.tile_num);

  uint8_t tile_y_offset;
  if (boy->ppu.pixel_fetcher.state == PixelFetcher_WIN) {
    tile_y_offset = 2 * (boy->ppu.pixel_fetcher.window_line % 8);
  } else {
    tile_y_offset = 2 * ((boy->mmu.LY + boy->mmu.SCY) % 8);
  }

  boy->ppu.ppu_state.PPU_DRAW.tile_address = base_tile_address + tile_y_offset;
  boy->ppu.ppu_state.PPU_DRAW.tile_low =
      read_byte_no_tick(boy, boy->ppu.ppu_state.PPU_DRAW.tile_address);

  return MODE_3_TILE_HIGH;
}

MODE_3_STATE mode_3_tile_high(BOY *boy) {
  boy->ppu.ppu_state.PPU_DRAW.tile_high =
      read_byte_no_tick(boy, boy->ppu.ppu_state.PPU_DRAW.tile_address + 1);

  return MODE_3_FIFO;
}

MODE_3_STATE mode_3_fifo(BOY *boy) {

  if (!ppu_queue_is_empty(&boy->ppu.background_fifo)) {
    // add one cycle delay to mode 3;
    boy->ppu.ppu_state.PPU_DRAW.dot_delay += 1;
    return MODE_3_FIFO;
  }

  for (int i = 0; i < 8; ++i) {
    uint8_t color_idx = get_color_idx(boy->ppu.ppu_state.PPU_DRAW.tile_low,
                                      boy->ppu.ppu_state.PPU_DRAW.tile_high, i);

    BGWinFifoEntry *pixel = malloc(sizeof(BGWinFifoEntry));
    pixel->color_idx = color_idx;
    ppu_queue_enqueue(&boy->ppu.background_fifo, pixel);
  }

  if ((boy->mmu.SCX % 8) != 0 && (boy->ppu.pixel_fetcher.x_offset == 0)) {
    boy->ppu.ppu_state.PPU_DRAW.scx_delay = boy->mmu.SCX % 8;
  }

  // go fetch the next tile
  return MODE_3_TILE_NUM;
}

void mode_3_push(BOY *boy) {
  if (ppu_queue_is_empty(&boy->ppu.background_fifo)) {
    return;
  }

  // discard scx % 8 pixel
  if (boy->ppu.ppu_state.PPU_DRAW.scx_delay != 0 &&
      boy->ppu.pixel_fetcher.x_offset == 0) {
    ppu_queue_dequeue(&boy->ppu.background_fifo);
    boy->ppu.ppu_state.PPU_DRAW.scx_delay--;
    boy->ppu.ppu_state.PPU_DRAW.dot_delay++;
    return;
  }

  // dequeue a background pixel;
  BGWinFifoEntry *entry = ppu_queue_dequeue(&boy->ppu.background_fifo);
}

uint16_t get_tile_base_address(MMU *mmu, uint8_t tile_number) {
  if (get_bit(mmu->LCDC, 4) == 1) {
    return (uint16_t)TILE_8000 + (tile_number * 16);
  }

  return (uint16_t)TILE_8800 + ((int8_t)tile_number * 16);
}

uint8_t get_color_idx(uint8_t low, uint8_t high, int bit_idx) {
  return (get_bit(high, bit_idx) << 1) | get_bit(low, bit_idx);
}
