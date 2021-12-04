#include <string.h>
#include <stdlib.h>
#include "network.h"
#include "crc.h"
#include "cobs.h"
#include "FreeRTOS.h"
#include "bluetooth.h"
#include "nrf_log.h"
#define ADDRESS         1//this is the address of the robot, should not conflict with other robots


void network_receive(uint8_t* frame, uint8_t len);

void (*receive_callbacks[10])(uint8_t, uint8_t*, uint16_t);

void network_set_callback(uint8_t protocol, void (*cb)(uint8_t, uint8_t*, uint16_t)) {
  if(protocol == PROTOCOL_ARQ || protocol == PROTOCOL_SIMPLE) receive_callbacks[protocol] = cb;
}
    
uint8_t network_send(uint8_t remote_address, uint8_t protocol, uint8_t *data, uint16_t len) {
  uint8_t *packet = pvPortMalloc(len+4);
  uint8_t *encoded_data = pvPortMalloc(len+6);
  if(packet == NULL || encoded_data == NULL) {
    NRF_LOG_INFO("NETWORK send failed NULL");
    vPortFree(packet);
    vPortFree(encoded_data);
    return 0;
  }
  packet[0] = remote_address;
  packet[1] = ADDRESS;
  packet[2] = protocol;
  memcpy(packet+3, data, len);
  packet[3+len] = calculate_crc(packet, 3+len);
  cobs_encode_result result = cobs_encode(encoded_data, len+5, packet, len+4);
  if(result.status != COBS_ENCODE_OK) {
    NRF_LOG_INFO("NETWORK send failed cobs");
    vPortFree(packet);
    vPortFree(encoded_data);
    return 0;
  }
  encoded_data[result.out_len] = 0x00;
  //NRF_LOG_INFO("ENCODED MESSAGE LEN: %d",result.out_len+1);
  //NRF_LOG_HEXDUMP_INFO(encoded_data,result.out_len+1 );
  ble_send(encoded_data, result.out_len+1);
  //NRF_LOG_INFO("network_send: DATA SENDT");
  vPortFree(packet);
  vPortFree(encoded_data);
  return 1;
}

uint8_t network_get_address(void) {
  return ADDRESS;
}

void network_receive(uint8_t* frame, uint8_t len) {
        
	uint8_t *decoded_data = pvPortMalloc(len);
	cobs_decode_result result = cobs_decode(decoded_data, len, frame, len-1);
	
	if(result.status != COBS_DECODE_OK) {
                NRF_LOG_INFO(" in RECIVED MESSAGE: COBS_DECODE FAILED");
		vPortFree(decoded_data);
		return;
	}
	if(decoded_data[result.out_len-1] != calculate_crc(decoded_data, result.out_len-1) ) {
                NRF_LOG_INFO(" in RECIVED MESSAGE: CRC FAILED");
		vPortFree(decoded_data);
		return;
	}
	uint8_t receiver = decoded_data[0];
	uint8_t sender = decoded_data[1];
	uint8_t protocol = decoded_data[2];
        //NRF_LOG_HEXDUMP_INFO(decoded_data,len); //TODO remove

	if(receiver != ADDRESS) {
                NRF_LOG_INFO("receiver != ADDRESS");
		vPortFree(decoded_data);
		return;
	}
	receive_callbacks[protocol](sender, decoded_data+3, result.out_len-4);
	vPortFree(decoded_data);
}

//volatile bool NewMessage[5];//buffer to handle inncoming messages so we dont drop messages if more than 1 per run of arqTask
//uint8_t Message[5][21];
//uint8_t messageLength[5];
//removed since the system works fine without it.

void network_ReciveFromBle(uint8_t* data, uint8_t length){
 network_receive(data, length);
    /*
     int j = 0;
     for (;j<5;j++){
        if(NewMessage[j]==false){
        break;}
     }
    //NRF_LOG_INFO("GOT A MESSAGE SET TRUE put in : %d " ,j);
     messageLength[j] = length;
     for (int i = 0; i < length; i++){
        Message[j][i] = data[i];
     }
  NewMessage[j] = true;
 */   
}

void network_getMessage(){
/*
  for(int i =0 ;i<5;i++){
    if (NewMessage[i] == true){
        network_receive(Message[i], messageLength[i]);
        //NRF_LOG_INFO("READ A MESSAGE FROM : %d ",i);
        NewMessage[i] = false;
    }
  }
  */
}
