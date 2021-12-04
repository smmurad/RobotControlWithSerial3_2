#ifndef IMU_TESTER_H
#define IMU_TESTER_H

void IMU_tester(void *pvParameters);
FILE * f_open( const char *pcFileName, const char *pcMode );
long f_write( const void *pvBuffer, long lSize, long lItems, FILE pxFileHandle );
#endif 