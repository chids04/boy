#pragma once
#include "common.h"
#include "ppu_queue.h"

#include <stdint.h>

#define MAX_SPRITES 10
#define FIFO_SIZE 8
#define FRAMEBUFFER_SIZE 0x5A00

enum TILE_ADDRESS_MODE { TILE_8000 = 0x8000, TILE_8800 = 0x9000 };

static const BoyColor CLASSIC_DMG_COLOR[4] = {
    (BoyColor){.r = 155, .g = 188, .b = 15, .a = 255},
    (BoyColor){.r = 139, .g = 172, .b = 15, .a = 255},
    (BoyColor){.r = 48, .g = 98, .b = 48, .a = 255},
    (BoyColor){.r = 15, .g = 56, .b = 15, .a = 255},
};

static const BoyColor MODERN_DMG_COLOR[4] = {
    (BoyColor){.r = 155, .g = 188, .b = 15, .a = 255},
    (BoyColor){.r = 139, .g = 172, .b = 15, .a = 255},
    (BoyColor){.r = 48, .g = 98, .b = 48, .a = 255},
    (BoyColor){.r = 15, .g = 56, .b = 15, .a = 255},
};

typedef enum PPU_MODE {
  PPU_MODE_0, // hblank
  PPU_MODE_1, // vblank
  PPU_MODE_2, // oam search
  PPU_MODE_3, // drawing
} PPU_MODE;

typedef enum MODE_3_STATE {
  MODE_3_TILE_NUM,
  MODE_3_TILE_LOW,
  MODE_3_TILE_HIGH,
  MODE_3_FIFO,
  MODE_3_FIFO_IDLE,
} MODE_3_STATE;

typedef struct PixelFetcher {
  enum PixelFetcherState {
    PixelFetcher_BG,
    PixelFetcher_WIN,
    PixelFetcher_OBJ,
  } state;

  int x_offset;
  int window_line;
  int cycles_remaining;

} PixelFetcher;

struct PPU {
  long long dots;
  int hblank_len;
  int draw_len;
  int total_dots;

  bool dot_delay;
  int dot_delay_len;

  SPRITE *sprite_buffer;
  size_t sprite_buffer_offset;

  PPU_MODE ppu_mode;

  BoyColor framebuffer[144][160];
  BoyColor bgMapBuffer[1024];

  BoyColor *pallette_colors;

  union {
    struct {
    } PPU_SCAN;
    struct {
      MODE_3_STATE mode_3_state;
      uint8_t tile_num;
      uint8_t tile_low;
      uint8_t tile_high;
      uint16_t tile_address;
      uint8_t scx_delay;
      uint8_t dot_delay;
      bool scanline_start;
      int repeat_count;
    } PPU_DRAW;

    struct {
      uint16_t hblank_len;
    } PPU_HBLANK;

  } ppu_state;

  PPU_QUEUE background_fifo;
  PPU_QUEUE sprite_fifo;

  PixelFetcher pixel_fetcher;
  bool wy_crossed_ly;
  bool vblank_ended;

  uint8_t oam_offset;
};

typedef struct ObjFifoEntry {
  uint8_t color_idx;

  // each object can have a different palette
  uint8_t pallette;

  // from bit 7 of sprite
  uint8_t bg_priority;
} ObjFifoEntry;

typedef struct BGWinFifoEntry {
  uint8_t color_idx;
} BGWinFifoEntry;

void init_ppu(PPU *ppu);
void tick_ppu(BOY *boy);

void handle_oam_scan(BOY *boy);
void handle_ppu_draw(BOY *boy);
void handle_ppu_hblank(BOY *boy);
void handle_ppu_vblank(BOY *boy);

void set_mode(PPU *ppu);
bool is_mode(PPU *ppu, PPU_MODE mode);
void set_ppu_stat_bits(MMU *mmu, PPU_MODE mode);
void check_stat_line(BOY *boy);
void check_vblank(BOY *boy);

bool to_sprite_buffer(BOY *boy, SPRITE *sprite);
uint8_t sprite_height(MMU *mmu);

void mode2_init(PPU *ppu);
void mode3_init(PPU *ppu);

MODE_3_STATE mode_3_tile_num(BOY *boy);
MODE_3_STATE mode_3_tile_low(BOY *boy);
MODE_3_STATE mode_3_tile_high(BOY *boy);
MODE_3_STATE mode_3_fifo(BOY *boy);
void mode_3_push(BOY *boy);

// mode 3 utils
void reset_fetcher_cycles(PPU *ppu);
BoyColor get_color_value(BOY *boy, uint8_t color_idx);
uint16_t get_bg_base(MMU *mmu);
uint16_t get_window_base(BOY *boy);
uint16_t get_bgmap_base(BOY *boy);
uint16_t get_tile_x(BOY *boy);
uint16_t get_tile_y(BOY *boy);

// returns true if state machine can advance
uint16_t get_tile_base_address(MMU *mmu, uint8_t tile_num);

// get the color index from
uint8_t get_color_idx(uint8_t low, uint8_t high, int bit_idx);
