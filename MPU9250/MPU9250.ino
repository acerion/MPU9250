/*
  Original Copyright notice:

  MPU9250_MS5637_t3 Basic Example Code
  by: Kris Winer
  date: April 1, 2014
  license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.


  File modified by Kamil Ignacak, 2019.


  Demonstrate basic MPU-9250 functionality including parameterizing
  the register addresses, initializing the sensor, getting properly
  scaled accelerometer, gyroscope, and magnetometer data out. Addition
  of 9 DoF sensor fusion using open source Madgwick and Mahony filter
  algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini.

  SDA and SCL should have external pull-up resistors (to 3.3V).

  Hardware setup:
  MPU9250 Breakout --------- Arduino
  VDD ---------------------- 3.3V
  VDDI --------------------- 3.3V
  SDA ----------------------- A4
  SCL ----------------------- A5
  GND ---------------------- GND

  Note: The MPU9250 is an I2C sensor and uses the Arduino Wire
  library.

  Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz
  Pro Mini.

  ???
  We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
  We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
  ???
*/




#include <Wire.h>
#include <SPI.h>




#include "MPU9250.h"
#include "MPU9250_debug_print.h"
#include "MPU9250_ahrs.h"
#include "MPU9250_regs.h"
#include "MPU9250_i2c.h"




/* Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1. */
#define ADO 0
#if ADO
#define MPU9250_ADDRESS 0x69  /* Device address when ADO = 1. */
#else
#define MPU9250_ADDRESS 0x68  /* Device address when ADO = 0. */
#endif
#define AK8963_ADDRESS  0x0C  /* Address of magnetometer. */




/*
  Serial line baud rate.
  300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, or 115200.
*/
const uint32_t serial_baud_rate = 115200;




/*
  [milliseconds] Interval between debug printing new data on serial
  console.  This value is independent of sensor of filter data rates.
*/
const int serial_debug_interval = 100;

const bool serial_debug_meas = true;        /* Show accel, gyro, mag measurements on serial line. */
#if WITH_LOCAL_AHRS
const bool serial_debug_quaternion = true;  /* Show quaternions on serial line. */
const bool serial_debug_ypr = true;          /* Show calculated basic ARHS values (yaw/pitch/roll) on serial line. */
const bool serial_debug_other_ahrs = true;   /* Show other calculated AHRS values on serial line. */
const bool serial_debug_filter_rate = true;  /* Show filter rate on serial line. */
#endif
#if WITH_TEMPERATURE
const bool serial_debug_temperature = true; /* Show temperature on serial line. */
#endif




/* TODO: hex values 0x00, 0x01, 0x10, 0x11? */
enum Ascale {
	AFS_2G = 0,
	AFS_4G,
	AFS_8G,
	AFS_16G
};

/* TODO: hex values 0x00, 0x01, 0x10, 0x11? */
enum Gscale {
	GFS_250DPS = 0,
	GFS_500DPS,
	GFS_1000DPS,
	GFS_2000DPS
};

/* Either 14-bit or 16-bit magnetometer resolution. */
enum Mscale {
	MFS_14BITS = 0, // 0.6 mG per LSB
	MFS_16BITS      // 0.15 mG per LSB
};

/* Magnetometer data rate. */
enum MMode {
	MAG_RATE_8HZ   = 0x02, /* 8 Hz */
	MAG_RATE_100HZ = 0x06  /* 100 Hz */
};

/* Select sensor parameters to be used in this project. */
const uint8_t Gscale = GFS_250DPS;
const uint8_t Ascale = AFS_2G;
const uint8_t Mscale = MFS_16BITS;
const uint8_t Mmode  = MAG_RATE_100HZ;

float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

/* Pin definitions. */
const int intPin = 8;
const int ledPin = 13;

volatile bool newData = false;

int16_t raw_agt[7]; /* Raw data from accelerometer (values 0-2), gyroscope (4-6) and temperature (3) sensor. */
int16_t raw_mag[3]; /* Raw data from magnetometer. */
float magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias

/* Bias corrections for gyro and accelerometer. */
float gyroBias[3] = {0, 0, 0};
float accelBias[3] = {0, 0, 0};

float magBias[3] = {0, 0, 0};
float magScale[3] = {0, 0, 0};

/* Results of gyro and accelerometer self test. */
float SelfTest[6];

/* Variable to hold latest sensor data values calculated from raw values. */
mpu_meas_t g_meas;

/* Used to control rate of displaying data on serial port. */
uint32_t display_time_prev = 0;




/*
  With certain settings the filter is updating at a ~145 Hz rate using
  the Madgwick scheme and >200 Hz using the Mahony.

  The filter update rate is determined mostly by the mathematical
  steps in the respective algorithms, the processor speed (8 MHz for
  the 3.3V Pro Mini), and the magnetometer ODR: an ODR of 10 Hz for
  the magnetometer produce the above rates, maximum magnetometer ODR
  of 100 Hz produces filter update rates of 36 - 145 and ~38 Hz for
  the Madgwick and Mahony schemes, respectively.

  This is presumably because the magnetometer read takes longer than
  the gyro or accelerometer reads.

  This filter update rate should be fast enough to maintain accurate
  platform orientation for stabilization control of a fast-moving
  robot or quadcopter. Compare to the update rate of 200 Hz produced
  by the on-board Digital Motion Processor of Invensense's MPU6050 6
  DoF and MPU9150 9DoF sensors.

  The 3.3 V 8 MHz Pro Mini is doing pretty well!
*/

/*
  Global constants for 9 DoF fusion and AHRS (Attitude and Heading
  Reference System).
*/
const float GyroMeasError = PI * (4.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
const float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
/*
  There is a tradeoff in the beta parameter between accuracy and
  response speed.

  In the original Madgwick study, beta of 0.041 (corresponding to
  GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
  However, with this value, the LSM9SD0 response time is about 10
  seconds to a stable initial quaternion.

  Subsequent changes also require a longish lag time to a stable
  output, not fast enough for a quadcopter or robot car!  By
  increasing beta (GyroMeasError) by about a factor of fifteen, the
  response time constant is reduced to ~2 sec

  I haven't noticed any reduction in solution accuracy. This is
  essentially the I coefficient in a PID control sense; the bigger the
  feedback coefficient, the faster the solution converges, usually at
  the expense of accuracy.

  In any case, this is the free parameter in the Madgwick filtering
  and fusion scheme.
*/
const float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
const float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

/* These are the free parameters in the Mahony filter and fusion
   scheme, Kp for proportional feedback, Ki for integral. */
const float Kp = 2.0 * 5.0;
const float Ki = 0.0;

/* Integration interval for both filter schemes. */
float deltat = 0.0f;

/* Variables used to calculate integration interval and frequency of
   calculations of output variables. */
float filter_time_sum = 0.0f;
uint32_t filter_time_sum_count = 0;
uint32_t filter_time_prev = 0;

/* Variable to hold results of calculations made with quaternions. */
mpu_ahrs_t g_ahrs;

extern float q[4];

/* Integral error for Mahony method. */
float eInt[3] = {0.0f, 0.0f, 0.0f};




void setAres(void);
void setGres(void);
void setMres(void);
void initMPU9250(void);
void initAK8963(float * destination);

void myinthandler(void);

void readMPU9250Data(int16_t * destination);
/* Returns true if new Magnetometer data has been read into @destination. */
bool readMagData(int16_t * destination);

void MPU9250SelfTest(float * destination);
void accelGyroCalMPU9250(float * dest_g_bias, float * dest_a_bias);
void magCalMPU9250(float * dest_bias, float * dest_scale);




void setup(void)
{
	Wire.begin();
	// TWBR = 12;  // 400 kbit/sec I2C speed for Pro Mini
	Serial.begin(serial_baud_rate);
	delay(1000);
	Serial.println("\n\nMicrocontroller is online\n");
	delay(3000);

	// Set up the interrupt pin, its set as active high, push-pull
	pinMode(intPin, INPUT);
	pinMode(ledPin, OUTPUT);
	digitalWrite(ledPin, HIGH);


	/* Read the WHO_AM_I register, this is a good test of communication. */
	byte who = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
	if (who != 0x71) {
		Serial.print("Could not connect to accel/gyro :"); Serial.print("I AM 0x"); Serial.print(who, HEX); Serial.print(" I should be 0x"); Serial.println(0x71, HEX);
		goto on_error;
	}
	Serial.println("Accel/gyro is online\n");

	/* Start by performing self test and reporting values. */
	MPU9250SelfTest(SelfTest);
	Serial.println("Accel/gyro self test:");
	Serial.print("    x-axis: acceleration trim within "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
	Serial.print("    y-axis: acceleration trim within "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
	Serial.print("    z-axis: acceleration trim within "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
	Serial.print("    x-axis: gyration trim within "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
	Serial.print("    y-axis: gyration trim within "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
	Serial.print("    z-axis: gyration trim within "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
	Serial.print("\n");
	delay(1000);

	/* Configure sensor resolutions, only need to do this once. */
	setAres();
	setGres();
	setMres();

	/* Calibrate gyro and accelerometers, load biases in bias
	   registers. */
	Serial.println("Calibrate gyro and accel");
	accelGyroCalMPU9250(gyroBias, accelBias);
	Serial.print("    accel biases (mg): "); Serial.print(1000.*accelBias[0]); Serial.print(""); Serial.print(1000.*accelBias[1]); Serial.print(""); Serial.println(1000.*accelBias[2]);
	Serial.print("    gyro biases (dps): "); Serial.print(gyroBias[0]); Serial.print("");        Serial.print(gyroBias[1]); Serial.print("");        Serial.println(gyroBias[2]);

	/* Initialize device for active mode read of
	   accelerometer, gyroscope, and temperature. */
	initMPU9250();
	Serial.println("\nMPU9250 initialized for active data mode\n");



	/* Read the WHO_AM_I register of the magnetometer,
	   this is a good test of communication. */
	who = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);
	if (who != 0x48) {
		Serial.print("Could not connect to magnetometer :"); Serial.print("I AM 0x"); Serial.print(who, HEX); Serial.print(" I should be 0x"); Serial.println(0x48, HEX);
		goto on_error;
	}
	Serial.println("Magnetometer is online\n");

	/* Get magnetometer calibration from AK8963 ROM. */
	/* Initialize device for active mode read of magnetometer. */
	initAK8963(magCalibration);
	Serial.println("AK8963 initialized for active data mode");

	magCalMPU9250(magBias, magScale);
	Serial.print("    mag biases (mG): "); Serial.print(magBias[0]); Serial.print(""); Serial.print(magBias[1]); Serial.print(""); Serial.println(magBias[2]);
	Serial.print("    mag scale (mG): "); Serial.print(magScale[0]); Serial.print(""); Serial.print(magScale[1]); Serial.print(""); Serial.println(magScale[2]);

	// Serial.println("Calibration values: ");
	Serial.print("    x-axis sensitivity adjustment value: "); Serial.println(magCalibration[0], 2);
	Serial.print("    y-axis sensitivity adjustment value: "); Serial.println(magCalibration[1], 2);
	Serial.print("    z-axis sensitivity adjustment value: "); Serial.println(magCalibration[2], 2);
	Serial.print("\n");



	/* Add delay to see results before serial spew of data. */
	delay(1000);

	/* Define interrupt for INT pin output of MPU9250. */
	attachInterrupt(intPin, myinthandler, RISING);

	return;

 on_error:
	while (1) {
		; /* Loop forever on errors. */
	}
}




void loop(void)
{
	/* If intPin goes high, all data registers have new data. */
	if (true || newData == true) {  /* On interrupt, read data. */
		newData = false;  /* Reset newData flag. */

		readMPU9250Data(raw_agt); // INT cleared on any read


		/* Convert the acceleration value into actual
		   meters per second^2. This depends on scale being set. */
		g_meas.ax = (float) raw_agt[0] * aRes - accelBias[0];
		g_meas.ay = (float) raw_agt[1] * aRes - accelBias[1];
		g_meas.az = (float) raw_agt[2] * aRes - accelBias[2];


		/* Convert the gyro value into actual degrees per
		   second. This depends on scale being set. */
		g_meas.gx = (float) raw_agt[4] * gRes;
		g_meas.gy = (float) raw_agt[5] * gRes;
		g_meas.gz = (float) raw_agt[6] * gRes;


#if WITH_TEMPERATURE
		g_meas.temperature = ((float) raw_agt[3]) / 333.87 + 21.0;
#endif


		const bool new_mag_data_ready = readMagData(raw_mag);
		if (new_mag_data_ready) {
			/* Calculate the magnetometer values in
			   milliGauss.  Include factory calibration
			   per data sheet and user environmental
			   corrections. Calculated value depends on
			   scale being set*/
			g_meas.mx = (float) raw_mag[0] * mRes * magCalibration[0] - magBias[0];
			g_meas.my = (float) raw_mag[1] * mRes * magCalibration[1] - magBias[1];
			g_meas.mz = (float) raw_mag[2] * mRes * magCalibration[2] - magBias[2];
			g_meas.mx *= magScale[0];
			g_meas.my *= magScale[1];
			g_meas.mz *= magScale[2];
		}
	}


#if WITH_LOCAL_AHRS
	/*
	  Set integration time by time elapsed since last filter
	  update.  'deltat' variable is used in both quaternion
	  filters.
	*/
	const uint32_t filter_time_now = micros();
	deltat = ((filter_time_now - filter_time_prev)/1000000.0f);
	filter_time_prev = filter_time_now;

	filter_time_sum += deltat; // sum for averaging filter update rate
	filter_time_sum_count++;

	calculate_quaternions(g_meas);
#endif


	const uint32_t display_time_now = millis();
	if ((display_time_now - display_time_prev) > serial_debug_interval) {

		if (serial_debug_meas) {
			debug_print_meas(g_meas);
		}


#if WITH_LOCAL_AHRS
		calculate_ahrs(q, g_meas, g_ahrs);

		if (serial_debug_quaternion) {
			debug_print_quaternion(q);
		}
		if (serial_debug_ypr) {
			debug_print_ypr(g_ahrs);
		}
		if (serial_debug_other_ahrs) {
			debug_print_other_ahrs(g_ahrs);
		}
		if (serial_debug_filter_rate) {
			Serial.print("Filter rate = "); Serial.print((float) filter_time_sum_count/filter_time_sum, 2); Serial.println(" Hz");
		}

		filter_time_sum_count = 0;
		filter_time_sum = 0;
#endif


#if WITH_TEMPERATURE
		if (serial_debug_temperature) {
			Serial.print("Chip temperature is "); Serial.print(g_meas.temperature, 3); Serial.println(" degrees C");
		}
#endif


		digitalWrite(ledPin, !digitalRead(ledPin));
		display_time_prev = millis();
	}
}




void myinthandler(void)
{
	newData = true;
}




void setMres(void)
{
	switch (Mscale) {
	case MFS_14BITS:
		mRes = 10.0 * 4912.0 / 8190.0; // Proper scale to return milliGauss
		break;
	case MFS_16BITS:
		mRes = 10.0 * 4912.0 / 32760.0; // Proper scale to return milliGauss
		break;
	}
}




void setGres(void)
{
	switch (Gscale) {
	case GFS_250DPS:
		gRes = 250.0 / 32768.0;
		break;
	case GFS_500DPS:
		gRes = 500.0 / 32768.0;
		break;
	case GFS_1000DPS:
		gRes = 1000.0 / 32768.0;
		break;
	case GFS_2000DPS:
		gRes = 2000.0 / 32768.0;
		break;
	}
}




void setAres(void)
{
	switch (Ascale) {
	case AFS_2G:
		aRes = 2.0 / 32768.0;
		break;
	case AFS_4G:
		aRes = 4.0 / 32768.0;
		break;
	case AFS_8G:
		aRes = 8.0 / 32768.0;
		break;
	case AFS_16G:
		aRes = 16.0 / 32768.0;
		break;
	}
}




void readMPU9250Data(int16_t * destination)
{
	uint8_t rawData[14];
	readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 14, &rawData[0]);

	/* Turn the MSB and LSB into a signed 16-bit value. */
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];
#if WITH_TEMPERATURE
	destination[3] = ((int16_t)rawData[6] << 8) | rawData[7]; /* Temperature. */
#endif
	destination[4] = ((int16_t)rawData[8] << 8) | rawData[9];
	destination[5] = ((int16_t)rawData[10] << 8) | rawData[11];
	destination[6] = ((int16_t)rawData[12] << 8) | rawData[13];
}




bool readMagData(int16_t * destination)
{
	uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	const uint8_t ready = (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01);
	if (ready) { // wait for magnetometer data ready bit to be set
		readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
		const uint8_t c = rawData[6]; // End data read by reading ST2 register
		if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
			destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
			destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];  // Data stored as little Endian
			destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
		}
	}

	return (bool) ready;
}




void initAK8963(float * destination)
{
	/* First extract the factory calibration for each magnetometer axis. */
	uint8_t rawData[3];
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
	delay(10);
	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	delay(10);
	readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
	destination[0] = (float)(rawData[0] - 128)/256.0 + 1.0;   // Return x-axis sensitivity adjustment values, etc.
	destination[1] = (float)(rawData[1] - 128)/256.0 + 1.0;
	destination[2] = (float)(rawData[2] - 128)/256.0 + 1.0;

	writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
	delay(10);

	/* Configure magnetometer's rate and resolution in CNTL
	   register. */
	writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode);
	delay(10);
}




void initMPU9250()
{
	// wake up device
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
	delay(100); // Wait for all registers to reset

	// get stable time source
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
	delay(200);

	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
	// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	// be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
	writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
	// determined inset in CONFIG above

	// Set gyroscope full scale range
	// Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x03; // Clear Fchoice bits [1:0]
	c = c & ~0x18; // Clear GFS bits [4:3]
	c = c | Gscale << 3; // Set full scale range for the gyro
	// c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c); // Write new GYRO_CONFIG value to register

	// Set accelerometer full-scale range configuration
	c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x18;  // Clear AFS bits [4:3]
	c = c | Ascale << 3; // Set full scale range for the accelerometer
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
	c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
	// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	//   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
	writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x12);  // INT is 50 microsecond pulse and any read to clear
	writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
	delay(100);
}




/*
  Function which accumulates gyro and accelerometer data after device
  initialization. It calculates the average of the at-rest readings
  and then loads the resulting offsets into accelerometer and gyro
  bias registers.
*/
void accelGyroCalMPU9250(float * dest_g_bias, float * dest_a_bias)
{
	uint16_t ii;
	uint8_t data[12];
	int32_t gyro_bias[3]  = {0, 0, 0};
	int32_t accel_bias[3] = {0, 0, 0};

	// reset device
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	delay(100);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	// else use the internal oscillator, bits 2:0 = 001
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
	writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
	delay(200);

	// Configure device for bias calculation
	writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
	writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
	writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
	writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
	delay(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	const uint16_t gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	const uint16_t accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
	writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	const uint16_t fifo_count = ((uint16_t)data[0] << 8) | data[1];
	const uint16_t packet_count = fifo_count/12; // How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];

	}
	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	if (accel_bias[2] > 0L) {
		accel_bias[2] -= (int32_t) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
	} else {
		accel_bias[2] += (int32_t) accelsensitivity;
	}

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;

	// Push gyro biases to hardware registers
	writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
	writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
	writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
	writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
	writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
	writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

	// Output scaled gyro biases for display in the main program
	dest_g_bias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
	dest_g_bias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	dest_g_bias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

	/*
	  Construct the accelerometer biases for push to the hardware
	  accelerometer bias registers. These registers contain
	  factory trim values which must be added to the calculated
	  accelerometer biases; on boot up these registers will hold
	  non-zero values. In addition, bit 0 of the lower byte must
	  be preserved since it is used for temperature compensation
	  calculations. Accelerometer bias registers expect bias input
	  as 2048 LSB per g, so that the accelerometer biases
	  calculated above must be divided by 8.
	*/

	int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

	const uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

	for (ii = 0; ii < 3; ii++) {
		if ((accel_bias_reg[ii] & mask)) {
			mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
		}
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	// Apparently this is not working for the acceleration biases in the MPU-9250
	// Are we handling the temperature correction bit properly?
	// Push accelerometer biases to hardware registers
	/*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
	    writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
	    writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
	    writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
	    writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
	    writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
	*/
	// Output scaled accelerometer biases for display in the main program
	dest_a_bias[0] = (float)accel_bias[0] / (float)accelsensitivity;
	dest_a_bias[1] = (float)accel_bias[1] / (float)accelsensitivity;
	dest_a_bias[2] = (float)accel_bias[2] / (float)accelsensitivity;
}




void magCalMPU9250(float * dest_bias, float * dest_scale)
{
	uint16_t ii = 0;
	uint16_t sample_count = 0;
	int32_t mag_bias[3] = {0, 0, 0};
	int32_t mag_scale[3] = {0, 0, 0};
	int16_t mag_max[3] = {-32767, -32767, -32767};
	int16_t mag_min[3] = {32767, 32767, 32767};
	int16_t mag_temp[3] = {0, 0, 0};

	Serial.println("Mag Calibration: Wave device in a figure eight until done!");
	delay(4000);

	// shoot for ~fifteen seconds of mag data
	switch (Mmode) {
	case MAG_RATE_8HZ:
		sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
		break;
	case MAG_RATE_100HZ:
	default:
		sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
		break;
	}

	for (ii = 0; ii < sample_count; ii++) {
		readMagData(mag_temp);  // Read the mag data
		for (int jj = 0; jj < 3; jj++) {
			if (mag_temp[jj] > mag_max[jj]) {
				mag_max[jj] = mag_temp[jj];
			}

			if (mag_temp[jj] < mag_min[jj]) {
				mag_min[jj] = mag_temp[jj];
			}
		}
		switch (Mmode) {
		case MAG_RATE_8HZ:
			delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
			break;
		case MAG_RATE_100HZ:
		default:
			delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
			break;
		}
	}

	// Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
	// Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
	// Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

	// Get hard iron correction
	mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
	mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
	mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

	dest_bias[0] = (float) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
	dest_bias[1] = (float) mag_bias[1]*mRes*magCalibration[1];
	dest_bias[2] = (float) mag_bias[2]*mRes*magCalibration[2];

	// Get soft iron correction estimate
	mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
	mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
	mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	dest_scale[0] = avg_rad/((float)mag_scale[0]);
	dest_scale[1] = avg_rad/((float)mag_scale[1]);
	dest_scale[2] = avg_rad/((float)mag_scale[2]);

	Serial.println("Mag Calibration done!");
}



void get_average_data(int32_t * avg_a_data, int32_t * avg_g_data, int n_samples)
{
	uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};

	/* Accumulate N samples from gyro and acclerometer. */
	for (int ii = 0; ii < n_samples; ii++) {
		readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
		avg_a_data[0] += (int16_t)(((int16_t) rawData[0] << 8) | rawData[1]);
		avg_a_data[1] += (int16_t)(((int16_t) rawData[2] << 8) | rawData[3]);
		avg_a_data[2] += (int16_t)(((int16_t) rawData[4] << 8) | rawData[5]);

		readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
		avg_g_data[0] += (int16_t)(((int16_t) rawData[0] << 8) | rawData[1]);
		avg_g_data[1] += (int16_t)(((int16_t) rawData[2] << 8) | rawData[3]);
		avg_g_data[2] += (int16_t)(((int16_t) rawData[4] << 8) | rawData[5]);
	}

	/* Calculate averages from accumulated values. */
	for (int ii = 0; ii < 3; ii++) {
		avg_a_data[ii] /= n_samples;
		avg_g_data[ii] /= n_samples;
	}
}




/*
  Accelerometer and gyroscope self test; check calibration wrt factory
  settings.

  Should return percent deviation from factory trim values, +/- 14 or
  less deviation is a pass.
*/
void MPU9250SelfTest(float * destination)
{
	uint8_t FS = 0;
	const int n_samples = 200;


	writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
	writeByte(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG, FS<<3);  // Set full scale range for the gyro to 250 dps
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, FS<<3); // Set full scale range for the accelerometer to 2 g


	int32_t aAvg[3] = {0};
	int32_t gAvg[3] = {0};
	get_average_data(aAvg, gAvg, n_samples);


	// Configure the accelerometer for self-test
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	delay(25);  // Delay a while to let the device stabilize


	int32_t aSTAvg[3] = {0};
	int32_t gSTAvg[3] = {0};
	get_average_data(aSTAvg, gSTAvg, n_samples);


	// Configure the gyro and accelerometer for normal operation
	writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
	writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
	delay(25);  // Delay a while to let the device stabilize


	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	uint8_t selfTest[6];
	selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
	selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
	selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
	selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
	selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
	selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

	// Retrieve factory self-test value from self-test code reads
	float factoryTrim[6];
	factoryTrim[0] = (float)(2620/1<<FS)*(pow(1.01, ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float)(2620/1<<FS)*(pow(1.01, ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float)(2620/1<<FS)*(pow(1.01, ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
	factoryTrim[3] = (float)(2620/1<<FS)*(pow(1.01, ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float)(2620/1<<FS)*(pow(1.01, ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float)(2620/1<<FS)*(pow(1.01, ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	for (int i = 0; i < 3; i++) {
		destination[i]   = 100.0 * ((float) (aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.0;   // Report percent differences
		destination[i+3] = 100.0 * ((float) (gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.0; // Report percent differences
	}
}
