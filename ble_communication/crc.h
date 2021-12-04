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

#ifndef _CRC_H_
#define _CRC_H_
char calculate_crc(unsigned char *data, char len);
#endif