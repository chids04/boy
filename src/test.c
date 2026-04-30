#include "test.h"
#include "boy.h"
#include "common.h"
#include <assert.h>
#include <stdint.h>
#include <string.h>

BOY* test_init() {
  FILE *f = fopen("./cpu_instrs/cpu_instrs.gb", "r");

  if (f == NULL) {
    perror("error opening rom for reading");
    return NULL;
  }

  fseek(f, 0, SEEK_END);
  long f_size = ftell(f);
  rewind(f);

  uint8_t *rom = malloc(f_size);
  if (rom == NULL) {
    fprintf(stderr, "mem alloc failed\n");
    fclose(f);
    return NULL;
  }

  size_t bytes_read = fread(rom, 1, f_size, f);
  if (bytes_read != f_size) {
    fprintf(stderr, "Error reading f\n");
    free(rom);
    fclose(f);
    return NULL;
  }

  fclose(f);
  printf("Successfully loaded %ld bytes.\n", f_size);

  BOY *boy = malloc(sizeof(BOY));
  if (boy == NULL) {
    fprintf(stderr, "mem alloc failed\n");
    free(rom);
    return NULL;
  }

  load_rom(boy, rom);
  init_components(boy);

  return boy;
};

void generate_test_sprites(SPRITE *oam) {
  for (int i = 0; i < 40; i++) {
      oam[i].y = i;
      oam[i].x = i + 40;
      oam[i].tile = (i * 2) % 256;
      oam[i].flags = i % 4;
  }
}


void test_dma_timing() {
  BOY *boy = test_init();
  if (boy == NULL) {
    return;
  }

  // copy sprite into work ram
  SPRITE sprite = { .y = 10, .x = 10, .tile = 0, .flags = 0 };
  memcpy(&boy->mmu.wram[0], &sprite, sizeof(SPRITE));

  // write to mem location to start dma from beginning of wram
  // write byte internally ticks the timer
  // this M cycle is where the dma is requested
  write_byte(boy, 0xFF46, 0xC0);

  assert(boy->mmu.dma_src == 0xC000);
  assert(boy->mmu.dma_delay == true);

  // assert first byte in work ram is first byte of sprite
  assert(((uint8_t*)boy->mmu.wram)[0] == 10);

  // dma delay m cycle, dma gets enabled at the end
  tick(boy, 1);
  assert(boy->mmu.dma_transfer == true);
  assert(boy->mmu.dma_delay == false);
  assert(boy->mmu.enabling_dma == false);
  assert(boy->mmu.dma_progress == 0);
  assert(((uint8_t*)boy->mmu.oam)[0] == 0);


  tick(boy, 1);
  assert(boy->mmu.dma_transfer == true);
  assert(boy->mmu.dma_progress == 1);
  assert(((uint8_t*)boy->mmu.oam)[0] == 10);

  printf("DMA timing test passed\n");
}


void test_dma_transfer() {
  BOY *boy = test_init();
  if (boy == NULL) {
    return;
  }

  SPRITE expected[40];

  generate_test_sprites(expected);

  memcpy(&boy->mmu.wram[0], (uint8_t*)expected, 160);

  write_byte(boy, 0xFF46, 0xC0);

  // tick to skip delay
  tick(boy, 1);

  // i cancel dma on the 160th cycle, maybe it needs to be cancelled at end of 159th?
  for (int i = 0; i <= 160; i++) {
      tick(boy, 1);
  }

  // ensure dma ended
  assert(boy->mmu.dma_transfer == false);

  for (int i = 0; i < 40; i++) {
      assert(boy->mmu.oam[i].y == expected[i].y);
      assert(boy->mmu.oam[i].x == expected[i].x);
      assert(boy->mmu.oam[i].tile == expected[i].tile);
      assert(boy->mmu.oam[i].flags == expected[i].flags);
  }

  printf("DMA transfer test passed\n");
}

void test_ppu_transitions() {
  BOY *boy = test_init();
  if (boy == NULL) {
    return;
  }

  assert(boy->ppu.mode == PPU_MODE_2);

  // 79 T cycles
  for (int i = 0; i < 19; i++) {
    tick(boy, 1);
  }

  // assert still in oam search mode
  assert(boy->ppu.mode == PPU_MODE_2);

  tick(boy, 1);

  assert(boy->ppu.mode == PPU_MODE_3);

  // 171 T cycles
  for(int i = 0; i < 42; i++) {
    tick(boy, 1);
  }

  assert(boy->ppu.mode == PPU_MODE_3);

  tick(boy, 1);

  assert(boy->ppu.mode == PPU_MODE_0);

  for(int i = 0; i < 21; i++) {
    tick(boy, 1);
  }

  assert(boy->ppu.mode == PPU_MODE_0);

  tick(boy, 1);

  assert(boy->ppu.mode == PPU_MODE_1);

  printf("PPU state transition test passed\n");

}
