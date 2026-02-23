#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define SRAM_START      0xA000
#define SRAM_END        0xBFFF

#define RAM_BANK_SIZE   0x2000
#define ROM_BANK_SIZE   0x4000

#define ROM_BANK0_START 0x0000
#define ROM_BANK0_END   0x3FFF

#define ROM_BANK1_START 0x4000
#define ROM_BANK1_END   0x7FFF

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
};

struct mmu *init_mmu(uint8_t *rom);
struct mbc get_mbc(struct mmu *mmu);

void handle_mbc1_write(struct mmu *mmu, uint16_t address, uint8_t data);

uint8_t handle_mbc1_read(struct mmu *mmu, uint16_t address);
int get_zero_bank_num(struct mmu *mmu);
int get_high_bank_num(struct mmu *mmu);

uint8_t read_byte(struct mmu *mmu, uint16_t address);
uint16_t read_word(uint16_t address);

void write_word(uint16_t address, uint16_t data);
void write_byte(struct mmu *mmu, uint16_t address, uint8_t data);

int ram_size_bytes(RAM_SIZE size);

// depending on the rom size, a certain number of the upper bits are ignored
// when selecting the bank number
uint8_t rom_mask(ROM_SIZE size);
