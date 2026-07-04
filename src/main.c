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
  const int BG_MAP_WIDTH = 256;
  const int BG_MAP_HEIGHT = 256;
  const int MAX_STEPS_PER_HOST_FRAME = 100000;
  const float SCREEN_SCALE = 1.4f;
  const float BG_MAP_SCALE = 1.0f;
  const float PANEL_TOP = 38.0f;
  const float BG_MAP_GAP = 28.0f;
  const float SECTION_GAP = 62.0f;

  InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "boy");

  Image img = GenImageColor(SCREEN_WIDTH, SCREEN_HEIGHT, BLACK);
  Texture2D screen_tex = LoadTextureFromImage(img);
  Image bgmap_img = GenImageColor(BG_MAP_WIDTH, BG_MAP_HEIGHT, BLACK);
  Texture2D bgmap_9800_tex = LoadTextureFromImage(bgmap_img);
  Texture2D bgmap_9c00_tex = LoadTextureFromImage(bgmap_img);

  const float scaled_screen_w = SCREEN_WIDTH * SCREEN_SCALE;
  const float scaled_screen_h = SCREEN_HEIGHT * SCREEN_SCALE;
  const float scaled_bg_w = BG_MAP_WIDTH * BG_MAP_SCALE;
  const float scaled_bg_h = BG_MAP_HEIGHT * BG_MAP_SCALE;
  const float screen_x = (WINDOW_WIDTH - scaled_screen_w) / 2.0f;
  const float screen_y = PANEL_TOP;
  const float bg_y = screen_y + scaled_screen_h + SECTION_GAP;
  const float bg_row_w = (scaled_bg_w * 2.0f) + BG_MAP_GAP;
  const float bg_9800_x = (WINDOW_WIDTH - bg_row_w) / 2.0f;
  const Rectangle screen_src = {0, 0, SCREEN_WIDTH, SCREEN_HEIGHT};
  const Rectangle bg_src = {0, 0, BG_MAP_WIDTH, BG_MAP_HEIGHT};
  const Rectangle screen_dest = {screen_x, screen_y, scaled_screen_w,
                                 scaled_screen_h};
  const Rectangle bg_9800_dest = {bg_9800_x, bg_y, scaled_bg_w, scaled_bg_h};
  const Rectangle bg_9c00_dest = {bg_9800_x + scaled_bg_w + BG_MAP_GAP, bg_y,
                                  scaled_bg_w, scaled_bg_h};

  // skip the bootrom for now
  BOY boy;
  load_rom(&boy, rom);
  init_components(&boy);

  log_state(&boy);

  while (!WindowShouldClose()) {
    int steps = 0;
    while (!(boy.event & EVENT_FRAME_READY) &&
           steps < MAX_STEPS_PER_HOST_FRAME) {
      step_boy(&boy);
      steps++;
    }

    // update texture with ppu framebuffer
    if (boy.event & EVENT_FRAME_READY) {
      UpdateTexture(screen_tex, &boy.ppu.framebuffer);
      UpdateTexture(bgmap_9800_tex, &boy.ppu.bgMap9800Buffer);
      UpdateTexture(bgmap_9c00_tex, &boy.ppu.bgMap9C00Buffer);
      boy.event &= ~EVENT_FRAME_READY;
    }

    BeginDrawing();
    ClearBackground((Color){236, 232, 216, 255});

    GuiLabel((Rectangle){bg_9800_dest.x, bg_9800_dest.y - 26,
                         bg_9800_dest.width, 20},
             "BG map $9800");
    GuiLabel((Rectangle){screen_dest.x, screen_dest.y - 26, screen_dest.width,
                         20},
             "LCD");
    GuiLabel((Rectangle){bg_9c00_dest.x, bg_9c00_dest.y - 26,
                         bg_9c00_dest.width, 20},
             "BG map $9C00");

    DrawTexturePro(bgmap_9800_tex, bg_src, bg_9800_dest, (Vector2){0, 0}, 0,
                   WHITE);
    DrawTexturePro(screen_tex, screen_src, screen_dest, (Vector2){0, 0}, 0,
                   WHITE);
    DrawTexturePro(bgmap_9c00_tex, bg_src, bg_9c00_dest, (Vector2){0, 0}, 0,
                   WHITE);

    // DrawRectangle(225, 132, 24, 84, BLACK);
    // DrawRectangle(195, 161, 84, 25, BLACK);
    EndDrawing();
  }

  CloseWindow();

  free(rom);

  return 0;
}
