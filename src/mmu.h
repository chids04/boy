#pragma once

#include "common.h"
#include <stdint.h>

#define ROM_BANK0_START 0x0000
#define ROM_BANK0_END 0x3FFF

#define ROM_BANK1_START 0x4000
#define ROM_BANK1_END 0x7FFF

#define RAM_BANK_SIZE 0x2000
#define ROM_BANK_SIZE 0x4000

#define VRAM_START 0x8000
#define VRAM_END 0x9FFF

#define SRAM_START 0xA000
#define SRAM_END 0xBFFF

#define WRAM_START 0xC000
#define WRAM_END 0xDFFF

#define ECHO_RAM_START 0xE000
#define ECHO_RAM_END 0xFDFF

#define OAM_START 0xFE00
#define OAM_END 0xFE9F

#define IO_START 0xFF00
#define IO_END 0xFF7F

#define HRAM_START 0xFF80
#define HRAM_END 0xFFFE

#define IE_REG 0xFFFF

#define WRAM_SIZE 0x2000
#define VRAM_SIZE 0x2000
#define HRAM_SIZE 0x7F
#define OAM_SIZE 0xA0


enum MBC_TYPE {
  MBC_NONE = 0x0,
  MBC_1,
  MBC_1_RAM,
  MBC_1_BATTERY_RAM,
  MBC_2 = 0x05,
  MBC_2_BATTERY_RAM,
  MBC_NONE_RAM = 0x08,
  MBC_NONE_BATTERY_RAM,
};

struct mbc {
  enum MBC_TYPE mbc_type;

  union {
    struct {
      uint8_t rom_bank_num;
      uint8_t ram_bank_num;
      bool ram_enable;
      int mode;
    } mbc_1;
  };
};

enum RAM_SIZE {
  RAM_NONE,
  RAM_2KB,
  RAM_8KB,
  RAM_32KB,
  RAM_128KB,
  RAM_64KB,
};

enum ROM_SIZE {
  ROM_32KB,
  ROM_64KB,
  ROM_128KB,
  ROM_256KB,
  ROM_512KB,
  ROM_1MB,
  ROM_2MB,
  ROM_4MB,
  ROM_8MB,
  ROM_1_1MB = 0x52,
  ROM_1_2MB,
  ROM_1_5MB,
};

struct SPRITE {
  uint8_t y;
  uint8_t x;
  uint8_t tile;
  uint8_t flags;
};

struct MMU {
  uint8_t memory[0x10000]; // 0xFFFF is 65535, so 0x10000 bytes for a full 64KB
                           // address space
  uint8_t *rom;
  uint8_t *sram;
  uint8_t wram[WRAM_SIZE];
  uint8_t vram[VRAM_SIZE];
  uint8_t hram[HRAM_SIZE];
  SPRITE oam[40];

  struct mbc mbc;
  enum RAM_SIZE ram_size;
  enum ROM_SIZE rom_size;

  bool dma_transfer;
  bool enabling_dma;
  bool dma_delay;
  uint8_t dma_progress;
  uint16_t dma_src;

  // interrupt registers
  uint8_t IF;
  uint8_t IE;

  // joypad
  uint8_t JOYP;

  // serial transfer data
  uint8_t SB;

  // serial transfer control
  uint8_t SC;

  // timer registers
  uint16_t DIV;
  uint8_t TIMA;
  uint8_t TMA;
  uint8_t TAC;

  uint8_t LY;
  uint8_t LCDC;
};

MMU* init_mmu(uint8_t *rom);
void init_mbc(MMU *mmu);
void init_hardware_registers(MMU *mmu);
uint8_t rom_header_checksum(MMU *mmu);
struct mbc get_mbc(MMU *mmu);

void handle_cart_write(MMU *mmu, uint16_t address, uint8_t data);
uint8_t handle_cart_read(MMU *mmu, uint16_t address);

int get_zero_bank_num(MMU *mmu);
int get_high_bank_num(MMU *mmu);

uint8_t read_byte(BOY *boy, uint16_t address);
uint8_t read_byte_no_tick(BOY *boy, uint16_t address);
void write_byte(BOY *boy, uint16_t address, uint8_t data);

void write_sram(MMU *mmu, uint16_t address, uint8_t data);
uint8_t read_sram(MMU *mmu, uint16_t address);

uint8_t handle_lcd_read(BOY *boy, uint16_t address);
uint8_t handle_timers_read(BOY *boy, uint16_t address);
void handle_timers_write(BOY *boy, uint16_t address, uint8_t data);

uint8_t handle_io_read(BOY *boy, uint16_t address);
void handle_io_write(BOY *boy, uint16_t address, uint8_t data);

uint8_t handle_mbc1_read(MMU *mmu, uint16_t address);
void handle_mbc1_write(MMU *mmu, uint16_t address, uint8_t data);

uint8_t handle_dma_read(BOY *boy, uint16_t address);
void handle_dma_write(MMU *mmu, uint8_t offset, uint8_t data);

SPRITE *handle_oam_read(MMU *mmu, uint8_t offset);

int ram_size_bytes(enum RAM_SIZE size);

// depending on the rom size, a certain number of the upper bits are ignored
// when selecting the bank number
uint8_t rom_mask(enum ROM_SIZE size);
