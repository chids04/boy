#include "boy.h"
#include "common.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "raylib.h"
#include "test.h"

#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

int main() {
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
  const int SCREEN_WIDTH = 400;
  const int SCREEN_HEIGHT = 360;


  // -10 for some padding
  const int DEBUG_PANEL_WIDTH = WINDOW_WIDTH - 10;
  const int DEBUG_PANEL_HEIGHT = 200;

  InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "boy");

  // skip the bootrom for now
  BOY boy;
  load_rom(&boy, rom);
  init_components(&boy);

  log_state(&boy);


  while (!WindowShouldClose()) {
    step_boy(&boy);

    BeginDrawing();
    ClearBackground(RAYWHITE);

    // placeholder for the gameboy display
    DrawRectangle(WINDOW_WIDTH/2 - SCREEN_WIDTH / 2, 10, SCREEN_WIDTH, SCREEN_HEIGHT, BLACK);
    DrawRectangleLines(WINDOW_WIDTH/2 - DEBUG_PANEL_WIDTH / 2, SCREEN_HEIGHT + 15, DEBUG_PANEL_WIDTH, DEBUG_PANEL_HEIGHT, PINK);


    GuiLabel((Rectangle){0,0, 100, 20},  "test");





    // DrawRectangle(225, 132, 24, 84, BLACK);
    // DrawRectangle(195, 161, 84, 25, BLACK);
    EndDrawing();
  }

  CloseWindow();

  free(rom);

  return 0;
}
