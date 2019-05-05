/*
  Original Copyright notice:

  MPU9250_MS5637_t3 Basic Example Code
  by: Kris Winer
  date: April 1, 2014
  license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.
*/



#ifndef _MPU9250_DEBUG_PRINT_H_
#define _MPU9250_DEBUG_PRINT_H_




#include "MPU9250.h"
#include "MPU9250_ahrs.h"




void debug_print_meas(const mpu_meas_t & meas);
#if WITH_LOCAL_AHRS
void debug_print_quaternion(const float * q);
void debug_print_ypr(const mpu_ahrs_t & ahrs);
void debug_print_other_ahrs(const mpu_ahrs_t & ahrs);
#endif




#endif /* #ifndef _MPU9250_DEBUG_PRINT_H_ */
