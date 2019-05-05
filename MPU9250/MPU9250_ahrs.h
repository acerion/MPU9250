/*
  Original Copyright notice:

  MPU9250_MS5637_t3 Basic Example Code
  by: Kris Winer
  date: April 1, 2014
  license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.
*/




#ifndef _MPU9250_AHRS_H_
#define _MPU9250_AHRS_H_




#include "MPU9250.h"




void calculate_quaternions(mpu_meas_t & meas);
void calculate_from_quaternions(mpu_calc_t & calc, mpu_meas_t & meas);




#endif /* #ifndef _MPU9250_AHRS_H_ */
