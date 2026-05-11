#include "common.h"
#include <stdio.h>
#include <stdlib.h>

// helpers
BOY* test_init();
void scan_full_oam(BOY *boy);
void generate_test_sprites(SPRITE *oam);

// dma tests
void test_dma_transfer();
void test_dma_timing();

// ppu tests
void test_ppu();
void test_ppu_transitions();
void test_ppu_single_oam_scan();

// sprite fetch tests
void test_sprites_mixed_validity();
void test_sprites_oam_overflow();
void test_sprites_x_zero_filtered();

// sprite draw tests

// write test the verifies window is fetched when it should be
void test_fetch_window();
