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




/* Do we want temperature measurements from MPU chip? They are not
   essential to IMU operations, so we may want to disable them. */
#define WITH_TEMPERATURE    1


/* Do we want calculations of quaternions and yaw/pitch/roll to be
   done on microcontroller? */
#define WITH_LOCAL_AHRS     1
#if WITH_LOCAL_AHRS
#define USE_MADGWICK        1   /* 1 = Madgwick filter; 0 = Mahony. */
#endif


/* Do we want to send data (primarily measurements) to PC over serial? */
#define WITH_DATA_TO_PC     1




/* Sensor data values calculated from raw values. */
typedef struct {
	/* Acceleration. */
	float ax;
	float ay;
	float az;

	/* Gyro. */
	float gx;
	float gy;
	float gz;

	/* Mag. */
	float mx;
	float my;
	float mz;

	/* In theory we should enable/disable this field based on
	   value of WITH_TEMPERATURE, but since this struct will may
	   be sent to PC, let's keep number of its member constant for
	   easier maintenance and interoperability. */
	float temperature;

	uint8_t new_mag_data_ready;

#if WITH_DATA_TO_PC
	uint32_t timestamp; /* [microseconds] */
	uint8_t checksum;
#endif
} __attribute__((packed)) mpu_meas_t;




#endif /* #ifndef _MPU9250_H_ */
