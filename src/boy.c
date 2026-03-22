#include "boy.h"
#include "cpu.h"
#include "log.h"
#include "mmu.h"
#include "timer.h"
#include "utils.h"
#include <stdio.h>
#include <stdlib.h>


static FILE *log_fp = NULL;
static int line = 0;

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

  line++;

  uint16_t pc = boy->cpu.PC;
  uint8_t m0 = read_byte_no_tick(boy, pc);
  uint8_t m1 = read_byte_no_tick(boy, (pc + 1) & 0xFFFF);
  uint8_t m2 = read_byte_no_tick(boy, (pc + 2) & 0xFFFF);
  uint8_t m3 = read_byte_no_tick(boy, (pc + 3) & 0xFFFF);

  fprintf(log_fp,
          "A:%02X F:%02X B:%02X C:%02X D:%02X E:%02X H:%02X L:%02X "
          "SP:%04X PC:%04X PCMEM:%02X,%02X,%02X,%02X\n",
          boy->cpu.A, boy->cpu.F, boy->cpu.B, boy->cpu.C, boy->cpu.D,
          boy->cpu.E, boy->cpu.H, boy->cpu.L, boy->cpu.SP, pc, m0, m1, m2, m3);

  // printf("A: %02X F: %02X B: %02X C: %02X D: %02X E: %02X H: %02X L: %02X "
  //        "SP: %04X PC: %04X PCMEM: %02X,%02X,%02X,%02X\n",
  //        boy->cpu.A, boy->cpu.F, boy->cpu.B, boy->cpu.C, boy->cpu.D, boy->cpu.E,
  //        boy->cpu.H, boy->cpu.L, boy->cpu.SP, pc, m0, m1, m2, m3);
  //fflush(log_fp);
}

void run(BOY *boy) {

  // log state just before execution

  log_state(boy);

  while (true) {
    decode_instruction(boy, false);
    log_state(boy);
  }
}

void check_interrupts(BOY *boy) {
  log_set_level(1);

  // bitwise & will set the bit 1 the interrupt has been requested and enabled
  uint8_t pending_interrupts = boy->mmu.IF & boy->mmu.IE;

  if(pending_interrupts != 0){
      boy->cpu.is_halted = false;

      // only handle interrupts if IME is set
      if(boy->cpu.IME == true) {
          handle_interrupts(boy, pending_interrupts);
      }
  }

}

void handle_interrupts(BOY *boy, uint8_t interrupts){
    // set so no other interrupts can occur mid interrupt
    boy->cpu.IME = false;

    // clear the bits of the IF flag to signal that the interrupt has been serviced

    if(get_bit(interrupts, VBLANK) == 1) {
        clear_bit(&boy->mmu.IF, VBLANK);
        call_interrupt(boy, VBLANK);

    }
    else if(get_bit(interrupts, LCD) == 1) {
        clear_bit(&boy->mmu.IF, LCD);
        call_interrupt(boy, LCD);
    }
    else if(get_bit(interrupts, TIMER) == 1) {
        clear_bit(&boy->mmu.IF, TIMER);
        call_interrupt(boy, TIMER);

    }

    else if(get_bit(interrupts, SERIAL) == 1) {
        clear_bit(&boy->mmu.IF, SERIAL);
        call_interrupt(boy, SERIAL);
    }

    else if(get_bit(interrupts, JOYPAD) == 1) {
        clear_bit(&boy->mmu.IF, JOYPAD);
        call_interrupt(boy, JOYPAD);
    }
}

void call_interrupt(BOY *boy, INTERRUPTS interrupt){
    // two wait state where cpu does nothing
    tick(boy, 2);

    uint16_t addr;

    switch(interrupt) {
        case VBLANK: addr = 0x40; break;
        case LCD: addr = 0x48; break;
        case TIMER: addr = 0x50; break;
        case SERIAL: addr = 0x58; break;
        case JOYPAD: addr = 0x60; break;
    }

    write_byte(boy, --boy->cpu.SP, boy->cpu.PC >> 8);
    write_byte(boy, --boy->cpu.SP, boy->cpu.PC & 0xFF);

    boy->cpu.PC = addr;
    tick(boy, 1);
}
