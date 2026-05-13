#pragma once
#include "common.h"
#include "ppu_queue.h"

#include <stdint.h>

#define MAX_SPRITES 10
#define FIFO_SIZE 8

enum TILE_ADDRESS_MODE { TILE_8000 = 8000, TILE_8800 = 9000 };

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
  MODE_3_TILE_FIFO,
} MODE_3_STATE;

typedef struct PixelFetcher {
  enum PixelFetcherState {
    PixelFetcher_BG,
    PixelFetcher_WIN,
    PixelFetcher_OBJ,
  } state;

  int x_offset;
  int window_line;


} PixelFetcher;


struct PPU {
  long long dots;

  SPRITE *sprite_buffer;
  size_t sprite_buffer_offset;

  PPU_MODE ppu_mode;

  union {
    struct {
    } PPU_SCAN;
    struct {
      MODE_3_STATE mode_3_state;
      uint8_t tile_num;
      uint8_t tile_low;
      uint8_t tile_high;
      uint16_t tile_address;
    } PPU_DRAW;
  } ppu_state;



  PPU_QUEUE background_fifo;
  PPU_QUEUE sprite_fifo;

  PixelFetcher pixel_fetcher;
  bool wy_crossed_ly;

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
} BGWinFifoEntry ;

void init_ppu(PPU *ppu);
void handle_ppu(BOY *boy, int cycles);
void handle_oam_scan(BOY *boy);
void handle_ppu_draw(BOY *boy);
void set_mode(PPU *ppu);
bool to_sprite_buffer(BOY *boy, SPRITE *sprite);
uint8_t sprite_height(MMU *mmu);

void mode3_init(PPU *ppu);
void mode_3_tile_num(BOY *boy);
void mode_3_tile_low(BOY *boy);
void mode_3_tile_high(BOY *boy);
void mode_3_tile_high_fifo(BOY *boy);
uint16_t get_tile_base_address(MMU *mmu, uint8_t tile_num);
