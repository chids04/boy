#include "boy.h"
// #include "log.h"
#include "test.h"
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

int main() {
  // test_ppu_transitions();
  //  test_ppu_mcycle_mode3();
  //  test_background_tile_fetch();
  //  test_queue_full();
  //  test_queue_order();
  // test_scanline_start_delay();
  // test_ppu_colors();
  FILE *f = fopen("./cpu_instrs/cpu_instrs.gb", "r");

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

  fclose(f);
  printf("Successfully loaded %ld bytes.\n", f_size);

  const int WINDOW_WIDTH = 800;
  const int WINDOW_HEIGHT = 700;

  // stick to original 10:9 aspect ratio of the gameboy
  const int SCREEN_WIDTH = 160;
  const int SCREEN_HEIGHT = 144;

  // -10 for some padding
  const int DEBUG_PANEL_WIDTH = WINDOW_WIDTH - 10;
  const int DEBUG_PANEL_HEIGHT = 200;

  InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "boy");

  Image img = GenImageColor(SCREEN_WIDTH, SCREEN_HEIGHT, BLACK);
  Texture2D screen_tex = LoadTextureFromImage(img);
  Texture2D bgmap_tex = LoadTextureFromImage(img);

  // skip the bootrom for now
  BOY boy;
  load_rom(&boy, rom);
  init_components(&boy);

  log_state(&boy);

  while (!WindowShouldClose()) {
    step_boy(&boy);

    // update texture with ppu framebuffer
    // still need to wire up sending the frames to the buffer and setting the
    // colour correctly
    if (boy.event & EVENT_FRAME_READY) {
      UpdateTexture(screen_tex, &boy.ppu.framebuffer);
      // UpdateTexture(bgmap_tex);
      boy.event &= ~EVENT_FRAME_READY;
    }

    BeginDrawing();
    ClearBackground(WHITE);

    DrawTexture(screen_tex, WINDOW_WIDTH / 2 - SCREEN_WIDTH / 2, 10, WHITE);

    GuiLabel((Rectangle){WINDOW_WIDTH / 2 - SCREEN_WIDTH / 2,
                         SCREEN_HEIGHT + 10, 100, 20},
             "background map");

    DrawTexture(bgmap_tex, WINDOW_WIDTH / 2 - SCREEN_WIDTH / 2,
                SCREEN_HEIGHT + 15, WHITE);

    GuiLabel((Rectangle){0, 0, 100, 20}, "test");

    // DrawRectangle(225, 132, 24, 84, BLACK);
    // DrawRectangle(195, 161, 84, 25, BLACK);
    EndDrawing();
  }

  CloseWindow();

  free(rom);

  return 0;
}
