#pragma once
#include "common.h"
#include "ppu_queue.h"

#include <stdint.h>

#define MAX_SPRITES 10
#define FIFO_SIZE 8
#define FRAMEBUFFER_SIZE 0x5A00

enum TILE_ADDRESS_MODE { TILE_8000 = 0x8000, TILE_8800 = 0x9000 };

typedef struct BoyColor {
  unsigned char r; // Color red value
  unsigned char g; // Color green value
  unsigned char b; // Color blue value
  unsigned char a; // Color alpha value
} BoyColor;

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

  SPRITE *sprite_buffer;
  size_t sprite_buffer_offset;

  PPU_MODE ppu_mode;

  BoyColor framebuffer[FRAMEBUFFER_SIZE];

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

void reset_fetcher_cycles(PPU *ppu);
MODE_3_STATE mode_3_tile_num(BOY *boy);
MODE_3_STATE mode_3_tile_low(BOY *boy);
MODE_3_STATE mode_3_tile_high(BOY *boy);
MODE_3_STATE mode_3_fifo(BOY *boy);
void mode_3_push(BOY *boy);
BoyColor get_color_value(MMU *mmu, uint8_t color_idx);

// returns true if state machine can advance
uint16_t get_tile_base_address(MMU *mmu, uint8_t tile_num);
uint8_t get_color_idx(uint8_t low, uint8_t high, int bit_idx);
