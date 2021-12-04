#include "network.h"
#include "simple_protocol.h"
#include <string.h>
#include "FreeRTOS.h"
#include <math.h>

#define MAX_MESSAGES 1

#define MAX_MESSAGE_SIZE 100

typedef struct {
  uint8_t status;
  uint8_t address;
  uint16_t num_received_bytes;
  uint8_t next_part;
  uint8_t message[MAX_MESSAGE_SIZE];
} simple_message_t;

static simple_message_t messages[MAX_MESSAGES];
void (*callback_data_received)(uint8_t*, uint16_t); 

void simple_p_reassembly(uint8_t sender, uint8_t *data, uint16_t length);

void simple_p_init(void (*cb)(uint8_t*, uint16_t)) {
  callback_data_received = cb;
  uint8_t i;
  network_set_callback(PROTOCOL_SIMPLE, simple_p_reassembly);
  for(i=0;i<MAX_MESSAGES;i++) {
    messages[i].address = 0xFF;
  }
}

uint8_t simple_p_send(uint8_t address, uint8_t *data, uint16_t length) {  
  uint16_t tmp;
  uint16_t remaining = length;
  uint16_t offset = 0;
  uint8_t *part = pvPortMalloc(MAX_PAYLOAD_SIZE);
  uint8_t part_number=0;
  uint8_t number_of_parts = (length/(MAX_PAYLOAD_SIZE-2)) + (length % (MAX_PAYLOAD_SIZE-2) != 0);
  while(remaining > 0) {
    tmp = remaining < (MAX_PAYLOAD_SIZE-2) ? remaining : (MAX_PAYLOAD_SIZE-2);
    part[0] = part_number++;
    part[1] = number_of_parts-1;
    memcpy(part+2, data+offset, tmp);
    network_send(address, PROTOCOL_SIMPLE, part, tmp+2);
    offset += tmp;
    remaining -= tmp;
  }
  vPortFree(part);
  return 1;
}


void simple_p_reassembly(uint8_t sender, uint8_t *data, uint16_t length) {
  uint8_t i;
  uint8_t id = 0xFF;
  uint8_t free = 0xFF;
  for(i=0;i<MAX_MESSAGES;i++) {
    if(messages[i].address == 0xFF && free == 0xFF) free = i;
    else if(messages[i].address == sender) {
      id = i;
      break;
    }
  }
  if(id == 0xFF && free == 0xFF) return; // Not room for any more messages, and none is stored for this address
  else if(id == 0xFF && free != 0xFF) { // Did not find any part messages from this sender, but there is room to store a new one
    id = free; 
  }
    
  if(data[0] == 0) { // First part of a new message
    messages[id].num_received_bytes = 0;
  } else if(data[0] != messages[id].next_part) {
    messages[id].next_part = 0;
    messages[id].num_received_bytes = 0;
    messages[id].address = 0xFF;
    return;
  }

  if(messages[id].num_received_bytes + (length-2) > MAX_MESSAGE_SIZE) { // Message is larger than what can be handled, discard it
    messages[id].num_received_bytes = 0;
    messages[id].next_part = 0;
    messages[id].address = 0xFF;
    return;
  }
  
  messages[id].next_part++;
  memcpy(messages[id].message+messages[id].num_received_bytes, data+2, length-2);
  messages[id].num_received_bytes += (length-2);
  
  if(data[0] == data[1]) {
    callback_data_received(messages[id].message, messages[id].num_received_bytes);
    messages[id].num_received_bytes = messages[id].next_part = 0;
    messages[id].address = 0xFF;
  }
}