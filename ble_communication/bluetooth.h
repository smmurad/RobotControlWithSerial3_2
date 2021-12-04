#include <stdint.h>
#include "app_uart.h"
void BLE_init();
void ble_send(uint8_t* data, uint16_t len);
void advertising_start();
void uart_event_handle(app_uart_evt_t * p_event);