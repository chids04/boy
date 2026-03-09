#include "boy.h"
#include "cpu.h"
#include "log.h"
#include "mmu.h"
#include "timer.h"
#include "utils.h"
#include <stdio.h>
#include <stdlib.h>

static FILE *log_fp = NULL;

void close_log_file(void) {
  if (log_fp)
    fclose(log_fp);
}

void load_rom(BOY *boy, uint8_t *rom) { boy->mmu = *init_mmu(rom); }

void init_components(BOY *boy) {
  uint8_t header_checksum = rom_header_checksum(&boy->mmu);

  init_cpu(&boy->cpu, header_checksum);

  boy->timers.mmu = &boy->mmu;

  atexit(close_log_file);
}

void tick(BOY *boy, int cycles) {

  increment_timers(&boy->timers, cycles);
  // also tick the ppu here too
};

void log_state(BOY *boy) {
  if (log_fp == NULL) {
    log_fp = fopen("test/state.log", "w");

    if (log_fp == NULL) {
      log_error("failed to open file for logging state, exiting.....");
      exit(1);
    }
  }

  uint16_t pc = boy->cpu.PC;
  uint8_t m0 = read_byte(boy, pc);
  uint8_t m1 = read_byte(boy, (pc + 1) & 0xFFFF);
  uint8_t m2 = read_byte(boy, (pc + 2) & 0xFFFF);
  uint8_t m3 = read_byte(boy, (pc + 3) & 0xFFFF);

  fprintf(log_fp,
          "A:%02X F:%02X B:%02X C:%02X D:%02X E:%02X H:%02X L:%02X "
          "SP:%04X PC:%04X PCMEM:%02X,%02X,%02X,%02X\n",
          boy->cpu.A, boy->cpu.F, boy->cpu.B, boy->cpu.C, boy->cpu.D,
          boy->cpu.E, boy->cpu.H, boy->cpu.L, boy->cpu.SP, pc, m0, m1, m2, m3);

  printf("A: %02X F: %02X B: %02X C: %02X D: %02X E: %02X H: %02X L: %02X "
         "SP: %04X PC: %04X PCMEM: %02X,%02X,%02X,%02X\n",
         boy->cpu.A, boy->cpu.F, boy->cpu.B, boy->cpu.C, boy->cpu.D, boy->cpu.E,
         boy->cpu.H, boy->cpu.L, boy->cpu.SP, pc, m0, m1, m2, m3);
  fflush(log_fp);
}

void run(BOY *boy) {

  // log state just before execution
  log_state(boy);

  while (true) {
    decode_instruction(boy);
    handle_interrupts(boy);
    log_state(boy);
  }
}

void handle_interrupts(BOY *boy) {
  log_set_level(1);

  if (boy->cpu.ime && boy->mmu.IE) {
    if (get_bit(boy->mmu.IE, 0) && get_bit(boy->mmu.IF, 0)) {
      // handle vblank
      log_warn("vblank requested\n");
    }

    if (get_bit(boy->mmu.IE, 1) && get_bit(boy->mmu.IF, 1)) {
      // handle LCD
      log_warn("lcd interrupt requested\n");
    }

    if (get_bit(boy->mmu.IE, 2) && get_bit(boy->mmu.IF, 2)) {
      // handle Timer
      log_warn("timer interrupt requested\n");
    }

    if (get_bit(boy->mmu.IE, 3) && get_bit(boy->mmu.IF, 3)) {
      // handle Serial
      log_warn("serial interrupt requested\n");
      //
    }

    if (get_bit(boy->mmu.IE, 4) && get_bit(boy->mmu.IF, 4)) {
      // handle handle joypad
      log_warn("joypad interrupt requested\n");
    }
  }
}
