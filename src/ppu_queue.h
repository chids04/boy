#include "common.h"

typedef struct PPU_QUEUE {
  void **queue;
  int cap;
  int head;
  int tail;
} PPU_QUEUE;

void ppu_queue_init(PPU_QUEUE *q, size_t cap);
bool ppu_queue_enqueue(PPU_QUEUE *q, void *pixel);
void* ppu_queue_dequeue(PPU_QUEUE *q);
bool ppu_queue_is_full(PPU_QUEUE *q);
bool ppu_queue_is_empty(PPU_QUEUE *q);
