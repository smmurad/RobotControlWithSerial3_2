#ifndef _BUFFER_H_
#define _BUFFER_H_

#include <stdint.h>

typedef struct {
  uint8_t *buf;
  uint16_t capacity;
  uint16_t head;
  uint16_t tail;
  uint16_t len;
} buffer_t;


uint8_t buffer_init(buffer_t *b, uint8_t *buf, uint16_t size);
uint16_t buffer_append(buffer_t *b, uint8_t *data, uint16_t len);
uint16_t buffer_remove(buffer_t *b, uint8_t *data, uint16_t len);
uint16_t buffer_remove_token(buffer_t *b, uint8_t* data, uint8_t token, uint16_t nbytes);
uint16_t buffer_read(buffer_t *b, uint8_t *data, uint16_t idx, uint16_t len);

#endif