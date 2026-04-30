#include "ppu.h"
#include "boy.h"

PPU ppu_init() {
  PPU ppu = { .mode = PPU_MODE_2 };

  return ppu;
}

// called every M cycle ( 4 T Cycles )
void handle_ppu(BOY *boy) {

  if(boy->ppu.mode == PPU_MODE_2) {
    handle_oam_scan(boy);
  }

  // handle state transitions
//s
  //
}

void handle_oam_scan(BOY *boy){
  // this is called every 4 T cycles but oam is searched every 2 T cycles
  // so we read 2 entries each time
  uint8_t offset1 = boy->ppu.oam_offset;
  uint8_t offset2 = boy->ppu.oam_offset + 1;

  SPRITE *entry1 = handle_oam_read(&boy->mmu, offset1);
  SPRITE *entry2 = handle_oam_read(&boy->mmu, offset2);


}
