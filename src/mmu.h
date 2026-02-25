#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define ROM_BANK0_START 0x0000
#define ROM_BANK0_END   0x3FFF

#define ROM_BANK1_START 0x4000
#define ROM_BANK1_END   0x7FFF

#define RAM_BANK_SIZE   0x2000
#define ROM_BANK_SIZE   0x4000

#define VRAM_START      0x8000
#define VRAM_END        0x9FFF

#define SRAM_START      0xA000
#define SRAM_END        0xBFFF

#define WRAM_START      0xC000
#define WRAM_END        0xDFFF

#define ECHO_RAM_START  0xE000
#define ECHO_RAM_END    0xFDFF

#define OAM_START       0xFE00
#define OAM_END         0xFE9F

#define IO_START 0xFF00
#define IO_END 0xFF7F

#define HRAM_START    0xFF80
#define HRAM_END      0xFFFE

#define IE_REG        0xFFFF

typedef enum {
  MBC_NONE = 0x0,
  MBC_1,
  MBC_1_RAM,
  MBC_1_BATTERY_RAM,
  MBC_2 = 0x05,
  MBC_2_BATTERY_RAM,
  MBC_NONE_RAM = 0x08,
  MBC_NONE_BATTERY_RAM,
} MBC_TYPE;

struct mbc {
  MBC_TYPE mbc_type;

  union {
    struct {
      uint8_t rom_bank_num;
      uint8_t ram_bank_num;
      bool ram_enable;
      int mode;
    } mbc_1;
  };
};

typedef enum {
  RAM_NONE,
  RAM_2KB,
  RAM_8KB,
  RAM_32KB,
  RAM_128KB,
  RAM_64KB,
} RAM_SIZE;

typedef enum {
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
} ROM_SIZE;

struct mmu {
  uint8_t memory[0x10000]; // 0xFFFF is 65535, so 0x10000 bytes for a full 64KB
                           // address space
  uint8_t *rom;
  uint8_t *ram;
  struct mbc mbc;
  RAM_SIZE ram_size;
  ROM_SIZE rom_size;

  // io registers
};

struct mmu *init_mmu(uint8_t *rom);
struct mbc get_mbc(struct mmu *mmu);

void handle_cart_write(struct mmu *mmu, uint16_t address, uint8_t data);
void handle_mbc1_write(struct mmu *mmu, uint16_t address, uint8_t data);

uint8_t handle_cart_read(struct mmu *mmu, uint16_t address);
uint8_t handle_mbc1_read(struct mmu *mmu, uint16_t address);

int get_zero_bank_num(struct mmu *mmu);
int get_high_bank_num(struct mmu *mmu);

uint8_t read_byte(struct mmu *mmu, uint16_t address);
void write_byte(struct mmu *mmu, uint16_t address, uint8_t data);

void handle_io_read(struct mmu* mmu, uint16_t address);

int ram_size_bytes(RAM_SIZE size);

// depending on the rom size, a certain number of the upper bits are ignored
// when selecting the bank number
uint8_t rom_mask(ROM_SIZE size);
