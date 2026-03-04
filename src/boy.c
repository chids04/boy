#include "boy.h"
#include "cpu.h"
#include "log.h"
#include "mmu.h"
#include "timer.h"
#include "utils.h"


void load_rom(BOY *boy, uint8_t *rom) {
    boy->mmu = *init_mmu(rom);
}

void init_components(BOY *boy){
    uint8_t header_checksum = rom_header_checksum(&boy->mmu);

    init_cpu(&boy->cpu, header_checksum);

    boy->timers.mmu = &boy->mmu;
}


void tick(BOY *boy, int cycles){
    increment_timers(&boy->timers, cycles);
    // also tick the ppu here too
};

void run(BOY *boy) {
    while(true){
        decode_instruction(boy);
        handle_interrupts(boy);
    }
}

void handle_interrupts(BOY *boy){
    log_set_level(1);

    if(boy->cpu.ime && boy->mmu.IE){
        if(get_bit(boy->mmu.IE, 0) && get_bit(boy->mmu.IF, 0)){
            // handle vblank
            log_warn("vblank requested\n");
        }

        if(get_bit(boy->mmu.IE, 1) && get_bit(boy->mmu.IF, 1)){
            // handle LCD
            log_warn("lcd interrupt requested\n");

        }

        if(get_bit(boy->mmu.IE, 2) && get_bit(boy->mmu.IF, 2)){
            // handle Timer
            log_warn("timer interrupt requested\n");
        }

        if(get_bit(boy->mmu.IE, 3) && get_bit(boy->mmu.IF, 3)){
            // handle Serial
            log_warn("serial interrupt requested\n");
            //
        }

        if(get_bit(boy->mmu.IE, 4) && get_bit(boy->mmu.IF, 4)){
            // handle handle joypad
            log_warn("joypad interrupt requested\n");
        }
    }

}
