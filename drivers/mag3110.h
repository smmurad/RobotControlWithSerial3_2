#ifndef _MAG3110_H
#define _MAG3110_H
#include <stdint.h>
typedef enum {
	MAG_OS_16,
	MAG_OS_32,
	MAG_OS_64,
	MAG_OS_128
} mag_oversampling_t;
typedef struct{
int16_t x;
int16_t y;
int16_t z;
}MAG_reading_t;
/**
 * @brief Function for init the mangetometer
 * @details This function initializes the magneteometer, and configrues it with
 * the sample-rate given. Prior to using mag_heading() a calibration has to be
 * performed by calling mag_calibrate(), and physcially rotating the contrrol
 * system 360 degree within 30 seconds.
 *
 * @param[in] os		Over-sampling ratio (OSR)
 */
void mag_init(mag_oversampling_t os);

/**
 * @brief Function for performing magnetometer calibration.
 * @details After calling this function, the control system must be rotated 360
 * degrees within 30 seconds. The calibration has to be performed prior to using
 * mag_heading().
 */
void mag_calibrate();


/**
 * @brief Function for obtaining control system heading.
 * @details Returns the heading in the range -180 to 180 degrees. Remember:
 * calibration must be performed prior to calling this function!
 */
float mag_heading();
MAG_reading_t mag_read();

#endif /* _MAG3110_H */