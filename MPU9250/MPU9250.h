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


/* Do we want to print any results locally? */
#define WITH_LOCAL_DISPLAY  0


#define HEADER_SIZE       6
#define HEADER_BEGIN   0x17
#define HEADER_BYTE_1  0x6b
#define HEADER_BYTE_2  0x61
#define HEADER_BYTE_3  0x6d
#define HEADER_BYTE_4  0x69
#define HEADER_BYTE_5  0x6c




/* Sensor data values calculated from raw values. */
typedef struct {
	uint32_t timestamp; /* [microseconds] */

	/* Packet counter increased with every transmission from
	   Arduino to PC. Used to detect missing transmissions.
	   First sent counter will have value 1. */
	uint32_t counter;

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

	/* Temperature read from IMU chip [Celsius degrees].

	   In theory we should enable/disable this field based on
	   value of WITH_TEMPERATURE, but since this struct will may
	   be sent to PC, let's keep number of its member constant for
	   easier maintenance and interoperability. */
	float imu_temperature;

	/* When trying to read data from IMU magnetometer, did the
	   magnetometer report that there was new data to be read? */
	uint8_t new_mag_data_ready;

	uint8_t checksum;
} __attribute__((packed)) data_t;




#endif /* #ifndef _MPU9250_H_ */
