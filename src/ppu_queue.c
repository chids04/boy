#include "ppu_queue.h"
#include <stdlib.h>

void ppu_queue_init(PPU_QUEUE *q, size_t cap){
  q->queue = malloc(sizeof(void*) * cap);
  q->cap = cap;
  q->head = 0;
  q->tail = 0;
}

bool ppu_queue_enqueue(PPU_QUEUE *q, void *pixel){
  if(ppu_queue_is_full(q)) {
    return false;
  }

  q->queue[++q->tail] = pixel;
  return true;
}

void* ppu_queue_dequeue(PPU_QUEUE *q){
  if(ppu_queue_is_empty(q)) {
    return NULL;
  }

  return q->queue[++q->head];
}

bool ppu_queue_is_full(PPU_QUEUE *q) {
  return ((q->tail + 1) % q->cap) == q->head;
}

bool ppu_queue_is_empty(PPU_QUEUE *q) {
  return q->tail == 0;
}
