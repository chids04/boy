#include "ppu_queue.h"
#include <stdlib.h>
#include <string.h>

void ppu_queue_init(PPU_QUEUE *q, size_t cap) {
  q->queue = malloc(sizeof(void *) * cap);
  q->cap = cap;
  q->head = 0;
  q->tail = 0;
  q->size = 0;
}

bool ppu_queue_enqueue(PPU_QUEUE *q, void *pixel) {
  if (ppu_queue_is_full(q)) {
    return false;
  }

  q->queue[q->tail] = pixel;
  q->tail = (q->tail + 1) % q->cap;
  q->size += 1;

  return true;
}

void *ppu_queue_dequeue(PPU_QUEUE *q) {
  if (ppu_queue_is_empty(q)) {
    return NULL;
  }

  void *item = q->queue[q->head];
  q->head = (q->head + 1) % q->cap;
  q->size -= 1;

  return item;
}

bool ppu_queue_is_full(PPU_QUEUE *q) { return q->size == q->cap; }

bool ppu_queue_is_empty(PPU_QUEUE *q) { return q->size == 0; }

void ppu_queue_reset(PPU_QUEUE *q) {
  q->size = 0;
  q->head = 0;
  q->tail = 0;
}
