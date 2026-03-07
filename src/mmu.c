#include "mmu.h"
#include "log.h"
#include <stdio.h>
#include <stdlib.h>

MMU *init_mmu(uint8_t *rom) {
  MMU *mmu = malloc(sizeof(MMU));
  mmu->rom = rom;

  mmu->mbc.mbc_type = mmu->rom[0x0147];

  mmu->ram_size = mmu->rom[0x0149];
  mmu->rom_size = mmu->rom[0x0148];

  mmu->rom = rom;
  mmu->sram = malloc(ram_size_bytes(mmu->ram_size));

  init_hardware_registers(mmu);

  return mmu;
}

void init_hardware_registers(MMU *mmu) {
  mmu->JOYP = 0xCF;
  mmu->SB = 0x00;
  mmu->SC = 0x7E;

  mmu->DIV = 0xAB;
  mmu->TIMA = 0x00;
  mmu->TMA = 0x00;
  mmu->TAC = 0xF8;

  mmu->IF = 0xE1;
  mmu->IE = 0x00;
}

uint8_t rom_header_checksum(MMU *mmu) {
  uint8_t checksum = 0;
  for (uint16_t address = 0x0134; address <= 0x014C; address++) {
    checksum = checksum - mmu->rom[address] - 1;
  }

  return checksum;
}

uint8_t read_byte(MMU *mmu, uint16_t address) {
  log_set_level(1);

  if (address >= ROM_BANK0_START && address <= ROM_BANK1_END) {
    // handle cartridge read
    // may need to tick timers inside the mbc to handle the timing differences
    return handle_cart_read(mmu, address);
  } else if (address >= VRAM_START && address <= VRAM_END) {
    log_warn("VRAM handler for address 0x%04X not implemented\n", address);
    // handle vram reads here
    // not implemented yet
  } else if (address >= SRAM_START && address <= SRAM_END) {
    // handle external ram read here
    return read_sram(mmu, address);

  } else if (address >= WRAM_START && address <= WRAM_END) {
    return mmu->wram[address - WRAM_START];

  } else if (address >= 0xE000 && address <= 0xFDFF) {
    // handle echo ram
    // technically use of this area is prohibted so no need to emulate
    log_warn("ECHO RAM handler for address 0x%04X not implemented", address);
  } else if (address >= 0xFE00 && address <= 0xFE9F) {
    // handle oam
    log_warn("OAM handler for address 0x%04X not implemented", address);
  } else if (address >= 0xFEA0 && address <= 0xFEFF) {
    // use of this area prohibited
    log_warn("prohibited area handler for address 0x%04X not implemented",
             address);
  } else if (address >= 0xFF00 && address <= 0xFF7F) {
    // handle IO registers
    switch (address) {
    case 0xFF00:
      return mmu->JOYP;
    case 0xFF01:
      return mmu->SB;
    case 0xFF02:
      return mmu->SC;
    case 0xFF04:
      return mmu->DIV;
    case 0xFF05:
      return mmu->TIMA;
    case 0xFF06:
      return mmu->TMA;
    case 0xFF07:
      return mmu->TAC;
    case 0xFF0F:
      return mmu->IF;
    case 0xFFFF:
      return mmu->IE;
    default:
      log_warn("IO handler for address 0x%04X not implemented", address);
      return 0xFF; // Default for unimplemented IO
    }
  } else if (address >= 0xFF80 && address <= 0xFFFE) {
    // handle HRAM
    log_warn("HRAM handler for address 0x%04X not implemented", address);
  }

  return 0;
}

uint8_t handle_cart_read(MMU *mmu, uint16_t address) {
  switch (mmu->mbc.mbc_type) {
  case MBC_NONE:
    break;
  case MBC_NONE_RAM:
    break;
  case MBC_NONE_BATTERY_RAM:
    break;
  case MBC_1:
  case MBC_1_RAM:
  case MBC_1_BATTERY_RAM:
    return handle_mbc1_read(mmu, address);
  default:
    log_warn("read_byte: mbc unimplemented");
  }

  return 0;
}

uint8_t read_sram(MMU *mmu, uint16_t address) {
  if (!mmu->mbc.mbc_1.ram_enable) {
    return 0xFF;
  }

  uint16_t relative_addr = address - SRAM_START;
  size_t ram_offset = 0;

  switch (mmu->ram_size) {
  case RAM_2KB:
  case RAM_8KB: {
    ram_offset = relative_addr % ram_size_bytes(mmu->ram_size);
  }
  case RAM_32KB: {
    if (mmu->mbc.mbc_1.mode == 1) {
      ram_offset = RAM_BANK_SIZE * mmu->mbc.mbc_1.ram_bank_num + relative_addr;
    } else if (mmu->mbc.mbc_1.mode == 0) {
      ram_offset = relative_addr;
    }
  }
  default:
    return 0xFF;
  }

  return mmu->sram[ram_offset];
}

void write_sram(MMU *mmu, uint16_t address, uint8_t data) {
  if (!mmu->mbc.mbc_1.ram_enable || mmu->ram_size == RAM_NONE) {
    return;
  }

  uint16_t relative_addr = address - SRAM_START;
  size_t ram_offset = 0;

  switch (mmu->ram_size) {
  case RAM_2KB:
  case RAM_8KB:
    ram_offset = relative_addr % ram_size_bytes(mmu->ram_size);
    break;

  case RAM_32KB:
    if (mmu->mbc.mbc_1.mode == 1) {
      ram_offset = RAM_BANK_SIZE * mmu->mbc.mbc_1.ram_bank_num + relative_addr;
    } else if (mmu->mbc.mbc_1.mode == 0) {
      ram_offset = relative_addr;
    }
    break;

  default:
    fprintf(stderr, "Unsupported RAM size in MBC1 for writing");
    return;
  }

  mmu->sram[ram_offset] = data;
}

void write_byte(MMU *mmu, uint16_t address, uint8_t data) {

  if (address >= ROM_BANK0_START && address <= ROM_BANK1_END) {
    // handle cartridge write
    // may need to tick timers inside the mbc to handle the timing differences
    handle_cart_write(mmu, address, data);

  } else if (address >= VRAM_START && address <= VRAM_END) {
    // handle vram writes here
    // not implemented yet

  } else if (address >= SRAM_START && address <= SRAM_END) {
    write_sram(mmu, address, data);

    // handle external ram write here
  } else if (address >= WRAM_START && address <= WRAM_END) {
    mmu->wram[address - WRAM_START] = data;

  } else if (address >= ECHO_RAM_START && address <= ECHO_RAM_END) {
    // handle echo ram
    // technically use of this area is prohibted so no need to emulate
  } else if (address >= OAM_START && address <= OAM_END) {
    // handle oam
  } else if (address >= ECHO_RAM_START && address <= ECHO_RAM_END) {
    // use of this area prohibited
  } else if (address >= IO_START && address <= IO_END) {

    if (address == 0xFF01) {
      mmu->SB = data;

      // hook for viewing debug output in test roms
    } else if (address == 0xFF02 && data == 0x81) {
      printf("%c", mmu->SB);
      fflush(stdout);
    }

    // handle IO registers
  } else if (address >= 0xFF80 && address <= 0xFFFE) {

    // handle HRAM
  }
}

void handle_cart_write(MMU *mmu, uint16_t address, uint8_t data) {
  switch (mmu->mbc.mbc_type) {
  case MBC_NONE:
    break;
  case MBC_NONE_RAM:
    break;
  case MBC_NONE_BATTERY_RAM:
    break;
  case MBC_1:
  case MBC_1_RAM:
  case MBC_1_BATTERY_RAM:
    handle_mbc1_write(mmu, address, data);
    break;

  default:
    printf("mbc not implemented");
  }
}

void handle_mbc1_write(MMU *mmu, uint16_t address, uint8_t data) {
  if (address >= 0x0 && address <= 0x1FFF) {
    if ((data & 0xF) == 0xA) {
      mmu->mbc.mbc_1.ram_enable = true;
    } else {
      mmu->mbc.mbc_1.ram_enable = false;
    }

  }
  // set rom bank num
  else if (address >= 0x2000 && address <= 0x3FFF) {
    switch (mmu->rom_size) {
    case ROM_32KB:
      // 32kb rom only has 2 banks, so bank 1 is always set in swappable slot
      mmu->mbc.mbc_1.rom_bank_num = 1;
      break;
    case ROM_64KB:
    case ROM_128KB:
    case ROM_256KB:
    case ROM_512KB:
    case ROM_1MB:
    case ROM_2MB:
      if (data == 0x0) {
        mmu->mbc.mbc_1.rom_bank_num = 1;
      } else {
        mmu->mbc.mbc_1.rom_bank_num = data & rom_mask(mmu->rom_size);
      }
      break;
    default:
      printf("mbc1: unknown rom size for rom bank selection");
      break;
    }
  } else if (address >= 0x4000 && address <= 0x5FFF) {
    mmu->mbc.mbc_1.ram_bank_num = data & 0b00000011;
  } else if (address >= 0x6000 && address <= 0x7FFF) {
    // mode flag set to lowest bit of written value
    mmu->mbc.mbc_1.mode = data & 1;
  }
}

uint8_t handle_mbc1_read(MMU *mmu, uint16_t address) {

  // writing to rom bank 0
  if (address >= ROM_BANK0_START && address <= ROM_BANK0_END) {
    if (mmu->mbc.mbc_1.mode == 0) {
      return mmu->rom[address];
    } else if (mmu->mbc.mbc_1.mode == 1) {
      int zero_bank_num = get_zero_bank_num(mmu);
      return mmu->rom[ROM_BANK_SIZE * zero_bank_num + address];
    }
  }

  // reading from bank 1
  else if (address >= ROM_BANK1_START && address <= ROM_BANK1_END) {
    uint16_t relative_addr = address - ROM_BANK1_START;
    int high_bank_num = get_high_bank_num(mmu);
    size_t rom_offset = ROM_BANK_SIZE * high_bank_num + relative_addr;

    return mmu->rom[rom_offset];
  }

  // reading from external ram
  else if (address >= SRAM_START && address <= SRAM_END) {
  }
}

int get_zero_bank_num(MMU *mmu) {
  switch (mmu->rom_size) {
  case ROM_32KB:
  case ROM_64KB:
  case ROM_128KB:
  case ROM_256KB:
  case ROM_512KB:
    return 0;
  case ROM_1MB:
    // have not yet added support for multi cart roms yet
    return (mmu->mbc.mbc_1.ram_bank_num & 1) << 5;
  case ROM_2MB:
    return (mmu->mbc.mbc_1.ram_bank_num & 0x03) << 5;
  default:
    fprintf(stderr, "mbc1: no zero bank number for selected ROM size");
    exit(1);
  }
}

int get_high_bank_num(MMU *mmu) {
  switch (mmu->rom_size) {
  case ROM_32KB:
  case ROM_64KB:
  case ROM_128KB:
  case ROM_256KB:
  case ROM_512KB:
    return mmu->mbc.mbc_1.rom_bank_num & rom_mask(mmu->rom_size);
  case ROM_1MB: {
    uint8_t bank = mmu->mbc.mbc_1.rom_bank_num & rom_mask(mmu->rom_size);
    uint8_t ram_bit_mask = (mmu->mbc.mbc_1.ram_bank_num & 1) << 5;
    return bank | ram_bit_mask;
  }

  case ROM_2MB: {
    uint8_t bank = mmu->mbc.mbc_1.rom_bank_num & rom_mask(mmu->rom_size);
    uint8_t ram_bit_mask = (mmu->mbc.mbc_1.ram_bank_num & 0x03) << 5;
    return bank | ram_bit_mask;
  }
  default:
    fprintf(stderr, "mbc1: no high bank number for selected ROM size");
    exit(1);
  }
}

int ram_size_bytes(enum RAM_SIZE size) {
  switch (size) {
  case RAM_NONE:
    return 0;
  case RAM_2KB:
    return 2048;
  case RAM_8KB:
    return 8192;
  case RAM_32KB:
    return 32768;
  case RAM_64KB:
    return 65536;
  case RAM_128KB:
    return 131072;
  }
}

uint8_t rom_mask(enum ROM_SIZE size) {
  switch (size) {
  case ROM_32KB:
    return 0b00000001;
  case ROM_64KB:
    return 0b00000011;
  case ROM_128KB:
    return 0b00000111;
  case ROM_256KB:
    return 0b00001111;
  case ROM_512KB:
  case ROM_1MB:
  case ROM_2MB:
    return 0b00011111;
  default:
    fprintf(stderr, "unsupported rom size selected for masking");
    exit(1);
  }
}
