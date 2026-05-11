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

void test_ppu() {
  test_ppu_transitions();
  test_ppu_single_oam_scan();
  test_sprites_mixed_validity();
  test_sprites_oam_overflow();
  test_sprites_x_zero_filtered();
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

void test_ppu_single_oam_scan() {
  BOY *boy = test_init();
  if (boy == NULL) {
    return;
  }

  SPRITE sprite = { .y = 10, .x = 20, .tile = 0 , .flags = 0 };
  SPRITE sprite2 = { .y = 11, .x = 30, .tile = 1 , .flags = 2 };

  boy->mmu.LY = 1;

  boy->mmu.oam[0] = sprite;
  boy->mmu.oam[1] = sprite2;

  handle_oam_scan(boy);

  assert(boy->ppu.sprite_buffer[0].y == sprite.y);
  assert(boy->ppu.sprite_buffer[0].x == sprite.x);
  assert(boy->ppu.sprite_buffer[0].tile == sprite.tile);
  assert(boy->ppu.sprite_buffer[0].flags == sprite.flags);

  assert(boy->ppu.sprite_buffer[1].y == sprite2.y);
  assert(boy->ppu.sprite_buffer[1].x == sprite2.x);
  assert(boy->ppu.sprite_buffer[1].tile == sprite2.tile);
  assert(boy->ppu.sprite_buffer[1].flags == sprite2.flags);
}

// Helper: scan all 40 OAM entries (20 calls of handle_oam_scan, each scans 2 entries)
void scan_full_oam(BOY *boy) {
  for (int i = 0; i < 20; i++) {
    handle_oam_scan(boy);
    boy->ppu.oam_offset += 2;
  }
}

void test_sprites_x_zero_filtered() {
  BOY *boy = test_init();
  if (boy == NULL) {
    return;
  }

  // Set up conditions for sprites to be visible by Y
  boy->mmu.LY = 20;
  boy->mmu.LCDC = 0x4;  // 8x8 sprite mode

  // Create 10 sprites all with x=0 (off-screen horizontally)
  for (int i = 0; i < 10; i++) {
    boy->mmu.oam[i].x = 0;
    boy->mmu.oam[i].y = 25;  // In valid Y range for LY=20
    boy->mmu.oam[i].tile = i;
    boy->mmu.oam[i].flags = 0;
  }

  scan_full_oam(boy);

  // Verify no sprites were added (all filtered out by x=0)
  assert(boy->ppu.sprite_buffer_offset == 0);
  printf("✅ Sprites with x=0 correctly filtered out\n");
}

void test_sprites_oam_overflow() {
  BOY *boy = test_init();
  if (boy == NULL) {
    return;
  }

  // Set up conditions
  boy->mmu.LY = 20;
  boy->mmu.LCDC = 0x4;  // 8x8 sprite mode

  // fill OAM with 40 valid sprites
  for (int i = 0; i < 40; i++) {
    boy->mmu.oam[i].x = 10 + i;  // all have x > 0
    boy->mmu.oam[i].y = 25;      // all in valid Y range
    boy->mmu.oam[i].tile = i;
    boy->mmu.oam[i].flags = 0;
  }

  scan_full_oam(boy);

  // verify only first 10 sprites were added
  assert(boy->ppu.sprite_buffer_offset == 10);

  // check that sprites 0-9 are in buffer
  for (int i = 0; i < 10; i++) {
    assert(boy->ppu.sprite_buffer[i].x == 10 + i);
    assert(boy->ppu.sprite_buffer[i].tile == i);
  }

  printf("✅ Only 10 sprites added when OAM overflows\n");
}

void test_sprites_mixed_validity() {
  BOY *boy = test_init();
  if (boy == NULL) {
    return;
  }

  // set up conditions
  boy->mmu.LY = 20;
  boy->mmu.LCDC = 0x4;  // 8x8 sprite mode

  // sprite 0: valid (x > 0, y in range)
  boy->mmu.oam[0].x = 10;
  boy->mmu.oam[0].y = 25;
  boy->mmu.oam[0].tile = 0;
  boy->mmu.oam[0].flags = 0;

  // sprite 1: invalid x (x = 0)
  boy->mmu.oam[1].x = 0;
  boy->mmu.oam[1].y = 25;
  boy->mmu.oam[1].tile = 1;
  boy->mmu.oam[1].flags = 0;

  // sprite 2: valid (x > 0, y in range)
  boy->mmu.oam[2].x = 20;
  boy->mmu.oam[2].y = 26;
  boy->mmu.oam[2].tile = 2;
  boy->mmu.oam[2].flags = 0;

  // sprite 3: invalid y (y too high, outside range for LY=20)
  boy->mmu.oam[3].x = 30;
  boy->mmu.oam[3].y = 100;
  boy->mmu.oam[3].tile = 3;
  boy->mmu.oam[3].flags = 0;

  // Sprite 4: valid (x > 0, y in range)
  boy->mmu.oam[4].x = 40;
  boy->mmu.oam[4].y = 27;
  boy->mmu.oam[4].tile = 4;
  boy->mmu.oam[4].flags = 0;

  scan_full_oam(boy);

  // Should have exactly 3 valid sprites
  assert(boy->ppu.sprite_buffer_offset == 3);

  // Verify correct sprites are in buffer
  assert(boy->ppu.sprite_buffer[0].x == 10);
  assert(boy->ppu.sprite_buffer[0].tile == 0);

  assert(boy->ppu.sprite_buffer[1].x == 20);
  assert(boy->ppu.sprite_buffer[1].tile == 2);

  assert(boy->ppu.sprite_buffer[2].x == 40);
  assert(boy->ppu.sprite_buffer[2].tile == 4);

  printf("✅ Mixed valid/invalid sprites correctly filtered\n");
}

void test_fetch_window() {
  BOY *boy = test_init();
  if (boy == NULL) {
    return;
  }

  /**
   * window tiles are fetched when
    WX is greater than or equal to
   */


}
