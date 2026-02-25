#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "cpu.h"
#include "mmu.h"
#include "timer.h"

struct gameboy {
    CPU cpu;
    struct mmu mmu;
    Timers timers;

};

int main(){
    // read the rom, start cpu and start decoding
    FILE *f = fopen("cpu_instrs.gb", "rb");

    if (f == NULL) {
        perror("error opening rom for reading");
        return 1;
    }

    fseek(f, 0, SEEK_END);
    long f_size = ftell(f);
    rewind(f);

    uint8_t *rom = malloc(f_size);
    if (rom == NULL) {
        fprintf(stderr, "mem alloc failed\n");
        fclose(f);
        return 1;
    }

    size_t bytes_read = fread(rom, 1, f_size, f);
    if (bytes_read != f_size) {
        fprintf(stderr, "Error reading f\n");
        free(rom);
        fclose(f);
        return 1;
    }

    // check for mbc

    // skip the bootrom for now
    struct gameboy boy;
    boy.cpu.PC = 0x1000;


    while(true){
        decode_instruction(&boy.cpu);
        // the number of cycles after each instruction is then in cpu.cycles

        int cycles = boy.cpu.cycles;
        increment_timers(&boy.timers, cycles);

        //tick(cycles);
    }

    fclose(f);
    printf("Successfully loaded %ld bytes.\n", f_size);

    free(rom);

    return 0;
}

void check_mbc(uint16_t *rom) {
    uint8_t mbc_type = rom[0x0147];

    switch (mbc_type) {
        case 0x00: {
            printf("no mbc");
            break;
        }
    }

}
