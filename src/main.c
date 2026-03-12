#include "boy.h"
#include "common.h"
#include "log.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

int main() {
  // read the rom, start cpu and start decoding
  FILE *f = fopen("./cpu_instrs/individual/03-op sp,hl.gb", "r");

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

  // skip the bootrom for now
  BOY boy;
  load_rom(&boy, rom);
  init_components(&boy);
  run(&boy);

  free(rom);

  return 0;
}
