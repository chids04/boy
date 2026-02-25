#include <stdint.h>
#include <stdbool.h>
#include "utils.h"

// need a hook that catches all writes to TMA and checks whethere a pending tima reset is about to occur so that we ensure TMA is set before it copied to TIMA
//

typedef struct {
    uint16_t DIV;
    uint8_t TIMA;
    uint8_t TMA;
    uint8_t TAC;

    int div_counter;

    int prev_and_result;

    bool pending_tima_reset;
    bool cycles_at_reset;

} Timers;

void inc_tima(Timers *timers) {
    if(timers->TIMA == 0xFF){
        timers->TIMA = 0;
        timers->pending_tima_reset = true;
        timers->cycles_at_reset = timers->DIV;
    }
    else {
        timers->TIMA += 1;
    }
}

bool is_tima_incrementing(Timers *timers) {
    if(timers->pending_tima_reset &&
        timers->DIV == timers->cycles_at_reset + 8
    ){
        return true;
    }

    return false;
}

void increment_timers(Timers* timers, int m_cycles) {
    // incremented every M cycle ( 4 T Cycles )

    timers->DIV += m_cycles * 4; // meant to be incremented every T cycle
    uint8_t tac_lower2 = timers->TAC & 0b11;

    uint8_t selected_bit;

    switch (tac_lower2) {
        case 0b00:
            selected_bit = get_bit(timers->DIV, 9);
            break;
        case 0b01:
            selected_bit = get_bit(timers->DIV, 3);
            break;

        case 0b10:
            selected_bit = get_bit(timers->DIV, 5);
            break;

        case 0b11:
            selected_bit = get_bit(timers->DIV, 7);
            break;
    }

    uint8_t timer_enable = get_bit(timers->TAC, 2);
    uint8_t and_result = timer_enable & selected_bit;

    // first check for if pending TMA reset
    if(timers->pending_tima_reset) {
        if(timers->DIV >= timers->cycles_at_reset + 8){
            timers->TIMA = timers->TMA;
            timers->pending_tima_reset = false;
        }
    }
    else if(and_result == 0 && timers->prev_and_result == 1 && !timers->pending_tima_reset){
        inc_tima(timers);
    }

    timers->prev_and_result = and_result;
}

// should only be called AFTER div register has been incremented
