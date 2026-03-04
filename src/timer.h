#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "common.h"

// need a hook that catches all writes to TMA and checks whethere a pending tima reset is about to occur so that we ensure TMA is set before it copied to TIMA
//

struct TIMERS{
    // holds the memory mapped timer registers
    MMU *mmu;

    int div_counter;

    int prev_and_result;

    bool pending_tima_reset;
    bool cycles_at_reset;

} ;

void inc_tima(TIMERS *timers);
bool is_tima_incrementing(TIMERS *timers);
void increment_timers(TIMERS* timers, int m_cycles);

// should only be called AFTER div register has been incremented
