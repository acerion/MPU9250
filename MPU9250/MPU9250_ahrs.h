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




#if WITH_LOCAL_AHRS




typedef struct {
	float yaw;
	float pitch;
	float roll;

	/* Rotation matrix coefficients for Euler angles and gravity components. */
	float a12;
	float a22;
	float a31;
	float a32;
	float a33;

	/* Linear acceleration (acceleration with gravity component subtracted). */
	float lin_ax;
	float lin_ay;
	float lin_az;
} mpu_ahrs_t;




/* Wrapper around functions from quaternionFilters.ino.
   The functions put results in global 'float q[4]' variable. */
void calculate_quaternions(const data_t & data, float filter_time_delta);

/* Use quaternions from global array q[4] and from @data to calculate
   AHRS data and put it in @ahrs. */
void calculate_ahrs(const data_t & data, mpu_ahrs_t & ahrs, float rate);

#endif /* #if WITH_LOCAL_AHRS */




#endif /* #ifndef _MPU9250_AHRS_H_ */
