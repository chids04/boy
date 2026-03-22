#include "timer.h"
#include "boy.h"
#include "utils.h"

void inc_tima(TIMERS *timers) {
  if (timers->mmu->TIMA == 0xFF) {
    timers->mmu->TIMA = 0;
    timers->pending_tima_reset = true;
  } else {
    timers->mmu->TIMA += 1;
  }
}

void increment_timers(TIMERS *timers, int m_cycles) {
  // incremented every M cycle ( 4 T Cycles )

  timers->mmu->DIV += m_cycles * 4; // meant to be incremented every T cycle
  uint8_t clock_select = timers->mmu->TAC & 0b11;

  uint8_t selected_bit;

  switch (clock_select) {
  case 0b00:
    selected_bit = get_bit(timers->mmu->DIV, 9);
    break;
  case 0b01:
    selected_bit = get_bit(timers->mmu->DIV, 3);
    break;

  case 0b10:
    selected_bit = get_bit(timers->mmu->DIV, 5);
    break;

  case 0b11:
    selected_bit = get_bit(timers->mmu->DIV, 7);
    break;
  }

  uint8_t timer_enable = get_bit(timers->mmu->TAC, 2);
  uint8_t and_result = timer_enable & selected_bit;

  // first check for if pending TMA reset
  // this will pending_tima_reset will be set from the previous cycle

  if (timers->pending_tima_reset) {
    timers->mmu->TIMA = timers->mmu->TMA;
    timers->pending_tima_reset = false;

    // request timer interrupt;
    set_bit(&timers->mmu->IF, TIMER);
  } else if (and_result == 0 && timers->prev_and_result == 1 &&
             !timers->pending_tima_reset) {
    inc_tima(timers);
  }

  timers->prev_and_result = and_result;
}
