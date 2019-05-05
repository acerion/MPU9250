/*
  Original Copyright notice:

  MPU9250_MS5637_t3 Basic Example Code
  by: Kris Winer
  date: April 1, 2014
  license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.
*/




#ifndef _MPU9250_H_
#define _MPU9250_H_




typedef struct {
	/* Sensor data values calculated from raw values. */
	float ax;
	float ay;
	float az;

	float gx;
	float gy;
	float gz;

	float mx;
	float my;
	float mz;

	float temperature;
} mpu_meas_t;




typedef struct {
	float pitch;
	float yaw;
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

} mpu_calc_t;


#endif /* #ifndef _MPU9250_H_ */
