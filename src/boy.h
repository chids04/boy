#pragma once

#include "common.h"
#include "cpu.h"
#include "mmu.h"
#include "timer.h"

struct BOY {
  CPU cpu;
  MMU mmu;
  TIMERS timers;
};

typedef enum {
    VBLANK,
    LCD,
    TIMER,
    SERIAL,
    JOYPAD,
} INTERRUPTS;

void load_rom(BOY *boy, uint8_t *rom);
void init_components(BOY *boy);
void run(BOY *boy);
void check_interrupts(BOY *boy);
void log_state(BOY *boy);
void close_log_file(void);
void handle_interrupts(BOY *boy, uint8_t interrupts);
void call_interrupt(BOY *boy, INTERRUPTS interrupt);

void tick(BOY *boy, int cycles);
