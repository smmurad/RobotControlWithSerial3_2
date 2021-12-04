#ifndef _ARQ_H_
#define _ARG_H_

#include "stdint.h"

typedef uint8_t arq_connection;

void arq_init(void);

arq_connection arq_new_connection(void);
uint8_t arq_connect(arq_connection id, uint8_t remote_addr, void (*func)(uint8_t*, uint16_t), uint16_t timeout);
arq_connection arq_listen(void (*func)(uint8_t*, uint16_t));
uint8_t arq_close_connection(arq_connection id);
uint8_t arq_send(arq_connection id, uint8_t *data, uint16_t len);
uint8_t arq_send_string(arq_connection id, char *str);
void vARQTask(void *pvParamters);
#endif