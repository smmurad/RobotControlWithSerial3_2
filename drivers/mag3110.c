#include "mag3110.h"
#include "math.h"
#include "nrf_delay.h"
#include "nrf_twi_mngr.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "i2c.h"


#define MAG3110_ADDR		0x0E

#define MAG3110_DR_STATUS	0x00
#define MAG3110_OUT_X_MSB	0x01
#define MAG3110_OUT_X_LSB	0x02
#define MAG3110_OUT_Y_MSB	0x03
#define MAG3110_OUT_Y_LSB	0x04
#define MAG3110_OUT_Z_MSB	0x05
#define MAG3110_OUT_Z_LSB	0x06
#define MAG3110_WHO_AM_I	0x07
#define MAG3110_SYSMOD		0x08
#define MAG3110_OFF_X_MSB	0x09
#define MAG3110_OFF_X_LSB	0x0A
#define MAG3110_OFF_Y_MSB	0x0B
#define MAG3110_OFF_Y_LSB	0x0C
#define MAG3110_OFF_Z_MSB	0x0D
#define MAG3110_OFF_Z_LSB	0x0E
#define MAG3110_DIE_TEMP	0x0F
#define MAG3110_CTRL_REG1	0x10
#define MAG3110_CTRL_REG2	0x11


static int x_offset = -2953;  //values gotten by mag_calibrate
static int y_offset = 3254;
static float x_scale = 1.05096512;
static float y_scale = 0.95384923;


static void mag3110_write(uint8_t reg, uint8_t val) {
        i2cSend(MAG3110_ADDR,reg,&val,1);
}

static uint8_t mag3110_read(uint8_t reg) {
        static uint8_t ret = 0;
        i2cRecive(MAG3110_ADDR, reg, &ret,1);	
	return ret;
}

void mag_init(mag_oversampling_t os) {

	mag3110_write(MAG3110_CTRL_REG2, 0xA0);
	mag3110_write(MAG3110_CTRL_REG1, 0x01 | (os<<3));
}

void mag_calibrate() {
	int readings = 0;
	uint8_t mag_status;
	
	NRF_LOG_INFO("Reading initial values");
	NRF_LOG_FLUSH();
	while (((mag_status = mag3110_read(MAG3110_DR_STATUS)) & 8) == 0)
		;
			
	uint8_t x_msb = mag3110_read(MAG3110_OUT_X_MSB);
	uint8_t x_lsb = mag3110_read(MAG3110_OUT_X_LSB);
	uint8_t y_msb = mag3110_read(MAG3110_OUT_Y_MSB);
	uint8_t y_lsb = mag3110_read(MAG3110_OUT_Y_LSB);
	mag3110_read(MAG3110_OUT_Z_MSB);
	mag3110_read(MAG3110_OUT_Z_LSB);
	int16_t x_min = (x_msb<<8) | (x_lsb);
	int16_t y_min = (y_msb<<8) | (y_lsb);
	int16_t x_max = x_min;
	int16_t y_max = y_min;
	
	NRF_LOG_INFO("Starting calibration");
	NRF_LOG_FLUSH();
	while (readings < 30*10) {
		while (((mag_status = mag3110_read(MAG3110_DR_STATUS)) & 8) == 0)
			;
			
		NRF_LOG_INFO("Value available\n");
		NRF_LOG_FLUSH();

		x_msb = mag3110_read(MAG3110_OUT_X_MSB);
		x_lsb = mag3110_read(MAG3110_OUT_X_LSB);
		y_msb = mag3110_read(MAG3110_OUT_Y_MSB);
		y_lsb = mag3110_read(MAG3110_OUT_Y_LSB);
		mag3110_read(MAG3110_OUT_Z_MSB);
		mag3110_read(MAG3110_OUT_Z_LSB);

		int16_t x = (x_msb<<8) | (x_lsb);
		int16_t y = (y_msb<<8) | (y_lsb);
		
		NRF_LOG_INFO("Value read: %d\n", x);
		NRF_LOG_FLUSH();

		if (x > x_max)
			x_max = x;
		if (x < x_min)
			x_min = x;
		if (y > y_max) 
			y_max = y;
		if (y < y_min) 
			y_min = y;
			
		readings++;
	}

	NRF_LOG_INFO("Calibration complete: %d, %d, %d, %d", x_max, x_min, y_max, y_min);
	NRF_LOG_FLUSH();

	x_offset = (x_min + x_max)/2;
	y_offset = (y_min + y_max)/2;
	x_scale = abs(x_offset)+abs(y_offset)/(2 * abs(x_max - x_min));
	y_scale =  abs(x_offset)+abs(y_offset)/(2 * abs(y_max - y_min));
}


float mag_heading() {
	uint8_t mag_status;
	while (((mag_status = mag3110_read(MAG3110_DR_STATUS)) & 8) == 0)
		;

	uint8_t x_msb = mag3110_read(MAG3110_OUT_X_MSB);
	uint8_t x_lsb = mag3110_read(MAG3110_OUT_X_LSB);
	uint8_t y_msb = mag3110_read(MAG3110_OUT_Y_MSB);
	uint8_t y_lsb = mag3110_read(MAG3110_OUT_Y_LSB);

	int16_t x = (x_msb<<8) | (x_lsb);
	int16_t y = (y_msb<<8) | (y_lsb);

	x = x - x_offset;
	y = y - y_offset;
	
	float xf = (float) x * 1.0f;
	float yf = (float) y * 1.0f;
	
	return (atan2f(yf*y_scale, xf*x_scale) * (180.0/M_PI));
}

MAG_reading_t mag_read(){
	uint8_t x_msb = mag3110_read(MAG3110_OUT_X_MSB);
	uint8_t x_lsb = mag3110_read(MAG3110_OUT_X_LSB);
	uint8_t y_msb = mag3110_read(MAG3110_OUT_Y_MSB);
	uint8_t y_lsb = mag3110_read(MAG3110_OUT_Y_LSB);
        uint8_t z_msb = mag3110_read(MAG3110_OUT_Z_MSB);
	uint8_t z_lsb = mag3110_read(MAG3110_OUT_Z_LSB);
        
        MAG_reading_t mag;
	mag.x = (x_msb<<8) | (x_lsb);
	mag.y = (y_msb<<8) | (y_lsb);
        mag.z = (z_msb<<8) | (z_lsb);       
        return mag;
}

