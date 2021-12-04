#include "buffer.h"
#include "string.h"

uint8_t buffer_init(buffer_t *b, uint8_t *buf, uint16_t size) {
  if(buf == NULL) return 0; // Make sure the memory is allocated
  b->buf = buf;
  b->capacity = size;
  b->head = b->tail = b->len = 0;
  return 1;
}

uint16_t buffer_append(buffer_t *b, uint8_t *data, uint16_t len) {
  if(len > b->capacity - b->len) return 0; //Not enough room in the buffer
  if(len < b->capacity - b->head) {
    memcpy(b->buf+b->head, data, len);
    b->head += len;
    if(b->head == b->capacity) b->head = 0;
  } else {
    memcpy(b->buf+b->head, data, (b->capacity - b->head) );
    memcpy(b->buf, data + (b->capacity - b->head), len-(b->capacity - b->head));
    b->head = len-(b->capacity - b->head);
  }
  b->len += len;
  return 1;
}

//This reads bytes from the buffer until token is found, or nbytes is reached
//The number of bytes read is returned
uint16_t buffer_remove_token(buffer_t *b, uint8_t* data, uint8_t token, uint16_t nbytes){
  uint16_t i;
  uint8_t * p;
  p = data;
  for(i=0; i < nbytes; i++){
    if( b->tail != b->head ){ //see if any data is available
      *p++ = b->buf[b->tail];  //grab a byte from the buffer
      b->tail++;  //increment the tail
      b->len--;
      if( b->tail == b->capacity ){  //check for wrap-around
        b->tail = 0;
      }
      if(*(p-1) == token) return i+1;
    } else {
      return i; //number of bytes read
    }
  }
  return nbytes;
}
//Read, but dont remove from the buffer, 'len' bytes starting at idx
uint16_t buffer_read(buffer_t *b, uint8_t *data, uint16_t idx, uint16_t len) {
  if(data == NULL) return 0;
  if(idx >= b->capacity) idx -= b->capacity;
  if(b->capacity - idx >= len) {
    memcpy(data, b->buf+idx, len);
  } else {
    memcpy(data, b->buf+idx, b->capacity - idx);
    memcpy(data + (b->capacity - idx), b->buf, len - (b->capacity - idx) );
  }
  return len;
}

//Remove 'len' bytes from the buffer and add them to 'data'
uint16_t buffer_remove(buffer_t *b, uint8_t *data, uint16_t len) {
  if(len > b->len) len = b->len;
  if(b->capacity - b->tail >= len) {
    if(data != NULL) memcpy(data, b->buf+b->tail, len);
    b->tail = b->tail + len;
    if(b->tail == b->capacity) b->tail = 0;
  } else {
    if(data != NULL) {
      memcpy(data, b->buf+b->tail, b->capacity - b->tail);
      memcpy(data + (b->capacity - b->tail), b->buf, len - (b->capacity - b->tail) );
    }
    b->tail = len - (b->capacity - b->tail);
  }
  b->len -= len;
  return len;
}