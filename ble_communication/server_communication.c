



#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "arq.h"
#include "defines.h"
#include "functions.h"
#include "server_communication.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "simple_protocol.h"
#include "NRF_LOG.h"

#include "robot_config.h"

#define SERVER_ADDRESS  0

arq_connection server_connection;

uint8_t connected;
extern message_t message_in;
extern SemaphoreHandle_t xCommandReadyBSem;
extern uint8_t gHandshook;

uint8_t use_arq[11] = { 
  [TYPE_HANDSHAKE] = 1,
  [TYPE_UPDATE] = 0, //Send update msg over simple. ARQ spazzes out after a while. 
  [TYPE_IDLE] = 1, 
  [TYPE_PING_RESPONSE] = 1, 
  [TYPE_DEBUG] = 0 
};

#define TYPE_HANDSHAKE      0
#define TYPE_UPDATE         1
#define TYPE_ORDER          2
#define TYPE_IDLE           3
#define TYPE_PAUSE          4
#define TYPE_UNPAUSE        5
#define TYPE_CONFIRM        6
#define TYPE_FINISH         7
#define TYPE_PING           8
#define TYPE_PING_RESPONSE  9
#define TYPE_DEBUG          10

void server_communication_init(void) {
  if(connected) return;
  server_connection = arq_new_connection();
}

uint8_t server_connect(void) {
  if (arq_connect(server_connection, SERVER_ADDRESS, server_receiver, 1000)) {
    connected = 1;
  }
  return connected;
}

uint8_t send_handshake(void) {
  if(!connected) return 0;
  message_t msg;
  msg.type = TYPE_HANDSHAKE;
  msg.message.handshake.name_length = ROBOT_NAME_LENGTH;
  strcpy((char*)msg.message.handshake.name, ROBOT_NAME);
  msg.message.handshake.width = ROBOT_TOTAL_WIDTH_MM;
  msg.message.handshake.length = ROBOT_TOTAL_LENGTH_MM;
  msg.message.handshake.axel_offset = ROBOT_AXEL_OFFSET_MM;
  msg.message.handshake.tower_offset_x = SENSOR_TOWER_OFFSET_X_MM;
  msg.message.handshake.tower_offset_y = SENSOR_TOWER_OFFSET_Y_MM;
  msg.message.handshake.sensor_offset1 = SENSOR_OFFSET_RADIUS_MM;
  msg.message.handshake.sensor_offset2 = SENSOR_OFFSET_RADIUS_MM;
  msg.message.handshake.sensor_offset3 = SENSOR_OFFSET_RADIUS_MM;
  msg.message.handshake.sensor_offset4 = SENSOR_OFFSET_RADIUS_MM;
  msg.message.handshake.sensor_heading1 = SENSOR1_HEADING_DEG;
  msg.message.handshake.sensor_heading2 = SENSOR2_HEADING_DEG;
  msg.message.handshake.sensor_heading3 = SENSOR3_HEADING_DEG;
  msg.message.handshake.sensor_heading4 = SENSOR4_HEADING_DEG;
  msg.message.handshake.deadline = ROBOT_DEADLINE_MS;
  
  uint8_t data[sizeof(handshake_message_t)+1];
  memcpy(data, (uint8_t*) &msg, sizeof(data));
  if(use_arq[TYPE_HANDSHAKE]) arq_send(server_connection, data, sizeof(data));
  else simple_p_send(server_connection, data, sizeof(data));
  return 1;
}

void send_update(int16_t x_cm, int16_t y_cm, int16_t heading_deg, int16_t towerAngle_deg, uint8_t S1_cm, uint8_t S2_cm, uint8_t S3_cm, uint8_t S4_cm){
  //NRF_LOG_INFO("connected is %d", connected);
  if(!connected) return;
  //NRF_LOG_INFO("Connected, sending update");
  message_t msg;
  msg.type = TYPE_UPDATE;
  msg.message.update.x = x_cm;
  msg.message.update.y = y_cm;
  msg.message.update.heading = heading_deg;
  msg.message.update.tower_angle = towerAngle_deg;
  msg.message.update.sensor1 = S1_cm;// > IR_MAX_DETECT_DISTANCE_MM +1 / 10 ? 0 : S1_cm;
  msg.message.update.sensor2 = S2_cm;// > IR_MAX_DETECT_DISTANCE_MM +1 / 10 ? 0 : S2_cm;
  msg.message.update.sensor3 = S3_cm;// > IR_MAX_DETECT_DISTANCE_MM +1/ 10 ? 0 : S3_cm;
  msg.message.update.sensor4 = S4_cm;// > IR_MAX_DETECT_DISTANCE_MM +1/ 10 ? 0 : S4_cm;
  uint8_t data[sizeof(update_message_t)+1];
  memcpy(data, (uint8_t*) &msg, sizeof(data));
  if(use_arq[TYPE_UPDATE]){
    arq_send(server_connection, data, sizeof(data));
   // NRF_LOG_INFO("Update message sent");
  } 
  else simple_p_send(SERVER_ADDRESS, data, sizeof(data));
}

void send_idle(void) {
 // NRF_LOG_INFO("IDLE connected is %d", connected);
  if(!connected) return;
  uint8_t status = TYPE_IDLE;
  if(use_arq[TYPE_IDLE]) arq_send(server_connection, &status, 1);
  else simple_p_send(SERVER_ADDRESS, &status, 1);
}

void debug(const char *fmt, ...) {

  if(USEBLUETOOTH)  return;
  
	uint8_t buf[100];
	va_list ap;
	buf[0] = TYPE_DEBUG;
	va_start(ap, fmt);
	uint8_t ret = vsprintf((char*)buf+1, fmt, ap);
	va_end(ap);
	if (ret > 0) {
		if(use_arq[TYPE_DEBUG]) arq_send(server_connection, buf, ret+1);
		else simple_p_send(SERVER_ADDRESS, buf, ret+1);
	}
}

void send_ping_response(void) {
 // NRF_LOG_INFO("PING connected is %d", connected);
  if(!connected) return;
  uint8_t status = TYPE_PING_RESPONSE;
  if(use_arq[TYPE_PING_RESPONSE]) arq_send(server_connection, &status, 1);
  else simple_p_send(SERVER_ADDRESS, &status, 1);
}

void server_receiver(uint8_t *data, uint16_t len) {
  if(data == NULL) { // ARQ passes NULL to the callback when connection is lost
      NRF_LOG_INFO("ARQ connection lost");
      gHandshook = 0;
  }
  NRF_LOG_INFO("GIVING command semaphore with len:%d",len);//TODO remove
  memcpy(&message_in, data, len);
  xSemaphoreGive(xCommandReadyBSem);
}
