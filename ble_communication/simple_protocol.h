#ifndef _ARQ_H_
#define _ARG_H_

#include "stdint.h"

void simple_p_init(void (*cb)(uint8_t*, uint16_t));
uint8_t simple_p_send(uint8_t address, uint8_t *data, uint16_t length);

#endif