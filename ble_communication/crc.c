//************************************************************************/
// File:			crc.c
// Author:			Kristian Lien, NTNU Spring 2017              
//
// Functions for calcualating CRC using Optimized Dallas (now Maxim) iButton 8-bit CRC calculation.
// 
// Polynomial: x^8 + x^5 + x^4 + 1 (0x8C)
// Initial value: 0x0
// 
// Source of update function: http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html
/************************************************************************/

#include "crc.h"

char crc_ibutton_update(unsigned char crc, char data);

char calculate_crc(unsigned char *data, char len) {
  char crc = 0;
  int i;
  for(i=0;i<len;i++) {
    crc = crc_ibutton_update(crc, data[i]);
  }
  return crc;
}

char crc_ibutton_update(unsigned char crc, char data) {
  char i;
  crc = crc ^ data;
  for (i = 0; i < 8; i++) {
    if (crc & 0x01) crc = (crc >> 1) ^ 0x8C;
    else crc >>= 1;
  }
  return crc;
}