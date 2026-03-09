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

void load_rom(BOY *boy, uint8_t *rom);
void init_components(BOY *boy);
void run(BOY *boy);
void handle_interrupts(BOY *boy);
void log_state(BOY *boy);
void close_log_file(void);

void tick(BOY *boy, int cycles);
