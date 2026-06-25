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
  ppu->vblank_ended = false;

  ppu_queue_init(&ppu->background_fifo, FIFO_SIZE);
  ppu_queue_init(&ppu->sprite_fifo, FIFO_SIZE);
}
void mode2_init(PPU *ppu) {
  ppu->ppu_mode = PPU_MODE_2;
  ppu->oam_offset = 0;
  memset(ppu->sprite_buffer, 0, sizeof(SPRITE) * 10);
  ppu->sprite_buffer_offset = 0;
}

void mode3_init(PPU *ppu) {
  ppu->dots = 0;
  ppu->ppu_mode = PPU_MODE_3;
  ppu->ppu_state.PPU_DRAW.mode_3_state = MODE_3_TILE_NUM;
  ppu->ppu_state.PPU_DRAW.tile_low = 0;
  ppu->ppu_state.PPU_DRAW.tile_high = 0;
  ppu->ppu_state.PPU_DRAW.tile_address = 0;
  ppu->ppu_state.PPU_DRAW.scx_delay = 0;
  ppu->ppu_state.PPU_DRAW.dot_delay = 0;
  ppu->ppu_state.PPU_DRAW.scanline_start = true;

  // init the pixel fetcher
  ppu->pixel_fetcher.x_offset = 0;
  ppu->pixel_fetcher.state = PixelFetcher_BG;
  ppu_queue_reset(&ppu->background_fifo);
  ppu_queue_reset(&ppu->sprite_fifo);

  // each step of pixel fetching takes two steps
  // this variable gets decremented each call to hanlde_ppu_draw()
  // cycle 1 = 1 - 1 = 0, do no work
  // cycle 2 = 0, do fetcher work so it's ready for the 3rd cycle, reset cycles
  reset_fetcher_cycles(ppu);
}

void reset_fetcher_cycles(PPU *ppu) { ppu->pixel_fetcher.cycles_remaining = 1; }

// called every M cycle ( 4 T Cycles )
void tick_ppu(BOY *boy) {
  boy->ppu.dots += 1;
  // update the state bits in the stat register

  set_ppu_stat_bits(&boy->mmu, boy->ppu.ppu_mode);
  check_stat_line(boy);

  check_vblank(boy);

  switch (boy->ppu.ppu_mode) {
  case PPU_MODE_0:
    handle_ppu_hblank(boy);
    return;
  case PPU_MODE_1:
    handle_ppu_vblank(boy);
    return;
  case PPU_MODE_2:
    handle_oam_scan(boy);
    return;
  case PPU_MODE_3:
    handle_ppu_hblank(boy);
    return;
  }

  if (boy->ppu.ppu_mode == PPU_MODE_2) {
    handle_oam_scan(boy);
  } else if (boy->ppu.ppu_mode == PPU_MODE_3) {
    handle_ppu_draw(boy);
  } else if (boy->ppu.ppu_mode == PPU_MODE_0) {
    handle_ppu_hblank(boy);
  }
}

void set_ppu_stat_bits(MMU *mmu, PPU_MODE mode) {
  switch (mode) {

  case PPU_MODE_0:
    mmu->STAT |= 0b00;
    break;
  case PPU_MODE_1:
    mmu->STAT |= 0b01;
    break;

  case PPU_MODE_2:
    mmu->STAT |= 0b10;
    break;

  case PPU_MODE_3:
    mmu->STAT |= 0b11;
    break;
  }
}

void check_stat_line(BOY *boy) {
  bool stat_line =
      (is_mode(&boy->ppu, PPU_MODE_0) && get_bit(boy->mmu.STAT, 3)) ||
      (is_mode(&boy->ppu, PPU_MODE_1) && get_bit(boy->mmu.STAT, 4)) ||
      (is_mode(&boy->ppu, PPU_MODE_2) && get_bit(boy->mmu.STAT, 5)) ||
      (get_bit(boy->mmu.STAT, 6) && ly_eq_lyc(&boy->mmu));

  // stat interrupt occurs on the rising edge,
  if (!boy->mmu.prev_stat_line && stat_line) {
    // request stat interrupt here
    set_bit(&boy->mmu.IF, 1);
  }

  boy->mmu.prev_stat_line = stat_line;
}

void check_vblank(BOY *boy) {

  // sets the event to signal to ui to draw the screen
  // done at the start of each scanline
  if (is_mode(&boy->ppu, PPU_MODE_1)) {
    boy->event |= EVENT_FRAME_READY;
  }
}

bool is_mode(PPU *ppu, PPU_MODE mode) {
  switch (mode) {
  case PPU_MODE_0:
    return ppu->ppu_mode == PPU_MODE_0;
  case PPU_MODE_1:
    return ppu->ppu_mode == PPU_MODE_1;
  case PPU_MODE_2:
    return ppu->ppu_mode == PPU_MODE_2;
  case PPU_MODE_3:
    return ppu->ppu_mode == PPU_MODE_3;
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

  if (boy->ppu.dots == 80) {
    mode3_init(&boy->ppu);
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

  // push pixels to lcd,
  mode_3_push(boy);

  // return new ppu state
  if (boy->ppu.pixel_fetcher.x_offset == 160) {
    boy->ppu.ppu_mode = PPU_MODE_0;

    // hblank pads to duration of a scanline to 456 dots
    // dot counter gets reset at start of mode 3
    // mode 2 is 80 dots long
    // hblank is the remaining
    boy->ppu.ppu_state.PPU_HBLANK.hblank_len = 456 - boy->ppu.dots - 80;
    boy->ppu.dots = 0;
  }
}

void handle_ppu_hblank(BOY *boy) {

  if (boy->ppu.dots == boy->ppu.ppu_state.PPU_HBLANK.hblank_len &&
      boy->mmu.LY == 144) {
    boy->ppu.dots = 0;
    boy->ppu.ppu_mode = PPU_MODE_1;
  } else if (boy->ppu.dots == boy->ppu.ppu_state.PPU_HBLANK.hblank_len) {
    boy->mmu.LY += 1;
    mode2_init(&boy->ppu);
  }
}

void handle_ppu_vblank(BOY *boy) {
  if (boy->ppu.dots % 456 == 0) {
    boy->mmu.LY += 1;
  }

  if (boy->ppu.dots == 4560) {
    mode2_init(&boy->ppu);
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

  // restart these steps at the start of the scanline
  if (boy->ppu.ppu_state.PPU_DRAW.scanline_start) {
    boy->ppu.ppu_state.PPU_DRAW.scanline_start = false;
    return MODE_3_TILE_NUM;
  }

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

  // check for pixel 160 after dequeue
  if (boy->ppu.pixel_fetcher.x_offset == 160) {
    return;
  }

  if (entry != NULL) {
  }
}

// for now this only handles bg color palette but will be expanded to handle
// sprites too
BoyColor get_color_value(MMU *mmu, uint8_t color_idx) {
  uint8_t color_val;

  switch (color_idx) {
  case 0:
    color_val = get_bit_range(mmu->BGP, 1, 0);
    break;
  case 1:
    color_val = get_bit_range(mmu->BGP, 3, 2);
    break;
  case 2:
    color_val = get_bit_range(mmu->BGP, 5, 4);
    break;
  case 3:
    color_val = get_bit_range(mmu->BGP, 7, 6);
    break;
  }

  switch (color_val) { case 0: }
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
