
#define WINDOW_SIZE	        4    // Because this implementation uses 7-bit sequence numbers, window size MUST be less than 128 to ensure proper function
#define MAX_MESSAGE_SIZE	  100 // The protocol supports up to 65535, but that would require an appropriately large MAX_SEGMENTS and SEND_BUF_SIZE 
#define SEND_BUF_SIZE       256 // Must be a power of two (for easy wrap around of the circular buffer), and must be large enough to handle the amount of traffic the application uses. Could make the actual window size smaller than WINDOW_SIZE if less than MAX_PAYLOAD_SIZE * WINDOW_SIZE
#define MAX_SEGMENTS        10   // Max queued segments either waiting to be sent or waiting to be acked.
#define MAX_DATA            MAX_PAYLOAD_SIZE-2 // The networks frame capacity minus two for the ARQ header bytes (sequence number and type)
#include <stdlib.h>
#include "math.h"
#include "arq.h"
#include "string.h"
#include "network.h"
#include "buffer.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "nrf_log.h"
#include "boards.h"

#define LOST_CONNECTION_TIMEOUT_MS     1000 
#define RETRANSMISSION_TIMEOUT_MS      200

#define MAX_CONNECTIONS     1

#define STATUS_NONE         0
#define STATUS_CLOSED       1
#define STATUS_CONNECTED    2
#define STATUS_CONNECTING   3

#define TYPE_DATA       0
#define TYPE_ACK        1
#define TYPE_SYN        2
#define TYPE_SYNACK     3
#define TYPE_ALIVE_TEST 4

typedef struct {
  uint8_t status;
  buffer_t send_buffer;
  buffer_t segment_lengths;
  uint16_t send_buffer_window_end;
  void (*callback_data_received)(uint8_t*, uint16_t); // Function to call when data is received on this connection
  uint8_t sequence_number; //Number of next packet to be sent
  uint8_t request_number; // Next packet expected received
  uint8_t sequence_base; // Next packet receiver is expecting
  uint8_t timer;
  uint16_t timeout;
  uint8_t timer_started;
  uint8_t remote_address;
  uint16_t num_received_bytes;
  uint16_t receive_message_length;
  uint8_t message[MAX_MESSAGE_SIZE];
  SemaphoreHandle_t mutex;
  TaskHandle_t blocked_task;
} arq_connection_t;

static arq_connection_t connections[MAX_CONNECTIONS];
static TaskHandle_t listening_task;

uint8_t arq_send_ack(arq_connection id, uint8_t sequence_number);
void arq_reassembly(arq_connection id, uint8_t *data, uint16_t len);
void sender(arq_connection id);
void receiver(uint8_t address, uint8_t *data, uint16_t len);

void arq_init(void) {
  network_set_callback(PROTOCOL_ARQ, receiver);
  listening_task = NULL;
  uint8_t i=0;
  for(i=0;i<MAX_CONNECTIONS;i++) {
    memset(&connections[i], 0, sizeof(arq_connection_t));  //is this neccesary ?
    connections[i].mutex = xSemaphoreCreateMutex();
    connections[i].status = STATUS_NONE;
  }
}

arq_connection arq_new_connection(void) {
  arq_connection_t *con = NULL;
  uint8_t id, i;
  
  for(i=0;i<MAX_CONNECTIONS;i++) {
    if(connections[i].status == STATUS_NONE) {
      con = &connections[i];
      id = i;
      break;
    }
  }
  if(con == NULL) return 0xFF; // Max connections reached
  
  con->num_received_bytes = con->receive_message_length = con->send_buffer_window_end = con->timeout = 0;
  con->sequence_base = con->sequence_number = con->request_number = con->timer = con->timer_started = 0;
  con->blocked_task = NULL;
  
  con->status = STATUS_CLOSED;
  return id;
}

arq_connection arq_listen(void (*func)(uint8_t*, uint16_t)) {
  if(listening_task != NULL) return 0xFF;
  uint32_t sender;
  listening_task = xTaskGetCurrentTaskHandle();
  xTaskNotifyStateClear(listening_task);
  xTaskNotifyWait(0xFFFFFFFF, 0x0, &sender, portMAX_DELAY);

  arq_connection id = arq_new_connection();
  if(id == 0xFF) return 0xFF;
  connections[id].callback_data_received = func;
  connections[id].remote_address = sender;
  connections[id].status = STATUS_CONNECTED; 
  uint8_t data = TYPE_SYNACK;
  network_send(sender, PROTOCOL_ARQ, &data, 1);
  
  uint8_t *buf = pvPortMalloc(SEND_BUF_SIZE);
  if(buf == NULL) return 0xFF;
  buffer_init(&connections[id].send_buffer, buf, SEND_BUF_SIZE);
  
  buf = pvPortMalloc(2*MAX_SEGMENTS); // 2 bytes to store the length for each segment
  if(buf == NULL) return 0xFF;
  buffer_init(&connections[id].segment_lengths, buf, 2*MAX_SEGMENTS);
  
  return id;
}

uint8_t arq_connect(arq_connection id, uint8_t remote_addr, void (*func)(uint8_t*, uint16_t), uint16_t timeout) {
  NRF_LOG_INFO("Arq connect: called");
  arq_connection_t *con = &connections[id];
  con->status = STATUS_CONNECTING;
  con->callback_data_received = func;
  con->remote_address = remote_addr;
  uint8_t data = TYPE_SYN;

  con->blocked_task = xTaskGetCurrentTaskHandle();
  xTaskNotifyStateClear(con->blocked_task);
  bsp_board_led_invert(0);
  network_send(con->remote_address, PROTOCOL_ARQ, &data, 1);
  bsp_board_led_invert(1);
  if(ulTaskNotifyTake(pdTRUE, timeout) == 0) { // Wait 1000 ms for a SYN ACK msg
  //timeout*portTickPeriodMS
    con->status = STATUS_CLOSED; // Connection failed
    id = 0xFF;
    NRF_LOG_INFO("Arq connection error");
    return 0;
  } else {
    NRF_LOG_INFO("Arq connection established");
    uint8_t *buf = pvPortMalloc(SEND_BUF_SIZE);
    if(buf == NULL) return 0xFF;
    buffer_init(&con->send_buffer, buf, SEND_BUF_SIZE);
    
    buf = pvPortMalloc(2*MAX_SEGMENTS); // 2 bytes to store the length for each segment
    if(buf == NULL) return 0xFF;
    buffer_init(&con->segment_lengths, buf, 2*MAX_SEGMENTS);
    
    con->status = STATUS_CONNECTED;
    
    return 1;
  }
}

uint8_t arq_close_connection(arq_connection id) {
  if(id >= MAX_CONNECTIONS) return 0;
  arq_connection_t *con = &connections[id];
  
  xSemaphoreTake(con->mutex, portMAX_DELAY);
  
  if(connections[id].status != STATUS_CONNECTED) {
    xSemaphoreGive(con->mutex);
    return 0;
  }
  vPortFree(con->send_buffer.buf);
  vPortFree(con->segment_lengths.buf);
  
  con->status = STATUS_CLOSED;
  
  con->callback_data_received(NULL, 0);
  
  xSemaphoreGive(con->mutex);
  
  return 1;
}
//Send 'len' bytes from 'data' 
uint8_t arq_send(arq_connection id, uint8_t *data, uint16_t len) {
  if(id >= MAX_CONNECTIONS) return 0;
  arq_connection_t *con = &connections[id];
  xSemaphoreTake(con->mutex, portMAX_DELAY);
  
  if(con->status != STATUS_CONNECTED || data == NULL || len == 0 || len > MAX_MESSAGE_SIZE) {
    xSemaphoreGive(con->mutex);
    return 0;
  }
  uint16_t total_len = len+2; // + 2 for the two header bytes containing the length (16 bit length)
  if(total_len < SEND_BUF_SIZE - con->send_buffer.len && 2*MAX_SEGMENTS - con->segment_lengths.len >= 2) {
    uint16_t tmp = MAX_DATA;
    uint16_t remaining = total_len;
    buffer_append(&con->send_buffer, (uint8_t*) &len, 2); // Add the header bytes containing the total message length
    buffer_append(&con->send_buffer, data, len);
    
    while(remaining > 0) {
      tmp = remaining < MAX_DATA ? remaining : MAX_DATA;
      buffer_append(&con->segment_lengths, (uint8_t*) &tmp, 2);
      remaining -= tmp;
    }
  } else {
    xSemaphoreGive(con->mutex);
    return 0;
  }
  xSemaphoreGive(con->mutex);
  return len;
}

uint8_t arq_send_string(arq_connection id, char *str) {
  return arq_send(id, (uint8_t*) str, strlen(str));
}

uint8_t arq_send_ack(arq_connection id, uint8_t sequence_number) {
  if(id >= MAX_CONNECTIONS) return 0;
  arq_connection_t *con = &connections[id];
  
  if(con->status != STATUS_CONNECTED || sequence_number > 127) {
    return 0;
  }
  
  uint8_t data[2];
  data[0] = TYPE_ACK;
  data[1] = sequence_number;
  //NRF_LOG_INFO("Sending ACK");
  return network_send(con->remote_address, PROTOCOL_ARQ, data, 2);
}

void receiver(uint8_t address, uint8_t *data, uint16_t len) {
  arq_connection_t *con = NULL;
  uint8_t id;
  uint8_t i;
  for(i=0;i<MAX_CONNECTIONS;i++) {
    if(connections[i].remote_address == address && connections[i].status != STATUS_NONE) {
      con = &connections[i];
      id = i;
      break;
    }
  }
  uint8_t type = data[0];
  
  if(con == NULL && type != TYPE_SYN) return;
  if(con == NULL && type == TYPE_SYN && listening_task != NULL) {
    xTaskNotify(listening_task, address, eSetValueWithOverwrite);
    return;
  }
  xSemaphoreTake(con->mutex, portMAX_DELAY);
  
  if(con->status == STATUS_CLOSED || con->status == STATUS_NONE || len == 0 || data == NULL) {
    xSemaphoreGive(con->mutex);
    return;
  }
  
  if(con->status == STATUS_CONNECTING && type == TYPE_SYNACK) {
    if(con->blocked_task != NULL) {
      xTaskNotifyGive(con->blocked_task);
      con->blocked_task = NULL;
      arq_send_ack(id, 0x00);
    }
    
    xSemaphoreGive(con->mutex);
    return;
  }
  
  uint8_t sequence = data[1];
  if(type == TYPE_DATA || type == TYPE_ALIVE_TEST) {
    if(sequence == con->request_number) {
      if(type == TYPE_DATA) arq_reassembly(id, &data[2], len-2);
      con->request_number = (con->request_number+1) & 127;
    }
    //NRF_LOG_INFO("sending ACK ON ALIVE TEST %d",con->request_number);
    arq_send_ack(id, con->request_number);
  } else if(type == TYPE_ACK) {
    uint8_t i;
    uint8_t count;
    count = (sequence-con->sequence_base) & 127;
    if(count != 0) {
      uint8_t len;
      for(i=0;i<count;i++) {
        buffer_remove(&con->segment_lengths, &len, 2);
        buffer_remove(&con->send_buffer, NULL, len);
      }
      con->sequence_base = sequence;

      con->timer = 0;
      con->timeout = 0;
      if(con->sequence_base == con->sequence_number) con->timer_started = 0; // No more un-acked packets
    }
  }
  
  xSemaphoreGive(con->mutex);
  
}

/* This function should be called regularily, at the moment it is called every 10 ms from ARQTask.
/  When the transmit window has available space, it removes segments from the send buffer 
/  and sends them. It also increments a retransmit timer and timeout timer. When the retransmit timer
/  expires the function resends all the segments in the transmit window. On timeout the connection is
/  closed. 
*/
void sender(arq_connection id) { 
  if(id >= MAX_CONNECTIONS) return;
  arq_connection_t *con = &connections[id];
  if(con->status == STATUS_NONE) return;
  xSemaphoreTake(con->mutex, portMAX_DELAY);
  if(con->status != STATUS_CONNECTED) {
    xSemaphoreGive(con->mutex);
    return;
  }
  if(con->timer_started) {
    con->timeout += 10;
    con->timer += 10;
    if(con->timeout > LOST_CONNECTION_TIMEOUT_MS) { 
      xSemaphoreGive(con->mutex);
      arq_close_connection(id);
      return;
    }
    if(con->timer > RETRANSMISSION_TIMEOUT_MS) {
      uint8_t i=0;
      uint16_t read_pos = con->send_buffer.tail;
      uint16_t len = 0;
      uint8_t resend_count = (con->sequence_number-con->sequence_base) & 127;
      while(i < resend_count) {
        uint8_t data[MAX_PAYLOAD_SIZE];
        data[0] = TYPE_DATA;
        data[1] = (con->sequence_base+i) & 127; 
        buffer_read(&con->segment_lengths, (uint8_t*) &len, con->segment_lengths.tail+2*i, 2);
        buffer_read(&con->send_buffer, data+2, read_pos, len);
        read_pos = (read_pos + len) & (con->send_buffer.capacity-1);
        network_send(con->remote_address, PROTOCOL_ARQ, data, len+2);
        ++i;
      }
      con->timer = 0;
    }
  }
  if( ((con->sequence_number-con->sequence_base) & 127) < WINDOW_SIZE && con->send_buffer.head != con->send_buffer_window_end) {
    uint8_t data[MAX_PAYLOAD_SIZE];
    uint16_t len;
    data[0] = TYPE_DATA;
    data[1] = con->sequence_number;
    buffer_read(&con->segment_lengths, (uint8_t*) &len, con->segment_lengths.tail+2*((con->sequence_number-con->sequence_base) & 127), 2);
    buffer_read(&con->send_buffer, &data[2], con->send_buffer_window_end, len);
    con->send_buffer_window_end = (con->send_buffer_window_end+len) & (con->send_buffer.capacity-1);
    con->timer_started = 1;
    con->sequence_number = (con->sequence_number+1) & 127;
    network_send(con->remote_address, PROTOCOL_ARQ, data, len+2);
  } 
  
  xSemaphoreGive(con->mutex);
}

void arq_reassembly(arq_connection id, uint8_t *data, uint16_t len) { 
  if(id >= MAX_CONNECTIONS) return;
  
  arq_connection_t *con = &connections[id];
  // Dont need to take the mutex because this funcion is only called from receiver, and at that point the task already holds the mutex. Could use a recursive mutex, but it is not necessary
  
  if(con->receive_message_length == 0) { // Not in the midle of receiving, so this is the start of a message
    con->receive_message_length = data[0] | (data[1] << 8); //First two bytes of messsage is length
    len-=2; // Remove the header from the length, left with the length of the payload
    data+=2; // Move the pointer to skip past the length bytes and point to the actual data
    
    if(con->receive_message_length > MAX_MESSAGE_SIZE) {
      return;
    }
  } 
  memcpy(con->message+con->num_received_bytes, data, len);
  con->num_received_bytes += len;
  
  if(con->num_received_bytes == con->receive_message_length) {
    con->callback_data_received(con->message, con->num_received_bytes);
    con->num_received_bytes = con->receive_message_length = 0;
  }
}

void vARQTask(void *pvParamters) {
  uint8_t i;
  while(1) {
    for(i=0;i<MAX_CONNECTIONS;i++) {
		sender(i);
    }
    //network_getMessage();
    vTaskDelay(10);
  }
}