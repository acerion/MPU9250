/*
  Original Copyright notice:

  MPU9250_MS5637_t3 Basic Example Code
  by: Kris Winer
  date: April 1, 2014
  license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.
*/




#include <math.h>




#include "MPU9250_ahrs.h"




#if WITH_LOCAL_AHRS

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);




// Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
static const float local_declination = 13.8;

/* Quaternion. */
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};




void calculate_quaternions(mpu_meas_t & meas)
{
	/*
	  Sensors x (y)-axis of the accelerometer/gyro is aligned with
	  the y (x)-axis of the magnetometer; the magnetometer z-axis
	  (+ down) is misaligned with z-axis (+ up) of accelerometer
	  and gyro!

	  We have to make some allowance for this orientation mismatch
	  in feeding the output to the quaternion filter.

	  For the MPU9250+MS5637 Mini breakout the +x accel/gyro is
	  North, then -y accel/gyro is East. So if we want te
	  quaternions properly aligned we need to feed into the
	  Madgwick function Ax, -Ay, -Az, Gx, -Gy, -Gz, My, -Mx, and
	  Mz. But because gravity is by convention positive down, we
	  need to invert the accel data, so we pass -Ax, Ay, Az, Gx,
	  -Gy, -Gz, My, -Mx, and Mz into the Madgwick function to get
	  North along the accel +x-axis, East along the accel -y-axis,
	  and Down along the accel -z-axis.

	  This orientation choice can be modified to allow any
	  convenient (non-NED) orientation convention.

	  Pass gyro rate as rad/s.
	*/
#if WITH_LOCAL_AHRS
#if PICK_MADGWICK
	MadgwickQuaternionUpdate(-meas.ax, meas.ay, meas.az, meas.gx*M_PI/180.0f, -meas.gy*M_PI/180.0f, -meas.gz*M_PI/180.0f, meas.my, -meas.mx, meas.mz);
#else
	MahonyQuaternionUpdate(-meas.ax, meas.ay, meas.az, meas.gx*M_PI/180.0f, -meas.gy*M_PI/180.0f, -meas.gz*M_PI/180.0f, meas.my, -meas.mx, meas.mz);
#endif
#endif
}




void calculate_ahrs(const float * quat, const mpu_meas_t & meas, mpu_ahrs_t & ahrs)
{
	/*
	  Define output variables from updated
	  quaternion---these are Tait-Bryan angles, commonly
	  used in aircraft orientation.

	  In this coordinate system, the positive z-axis is
	  down toward Earth.

	  Yaw is the angle between Sensor x-axis and Earth
	  magnetic North (or true North if corrected for local
	  declination, looking down on the sensor positive yaw
	  is counterclockwise.

	  Pitch is angle between sensor x-axis and Earth
	  ground plane, toward the Earth is positive, up
	  toward the sky is negative.

	  Roll is angle between sensor y-axis and Earth ground
	  plane, y-axis up is positive roll.

	  These arise from the definition of the homogeneous
	  rotation matrix constructed from quaternions.

	  Tait-Bryan angles as well as Euler angles are
	  non-commutative; that is, the get the correct
	  orientation the rotations must be applied in the
	  correct order which for this configuration is yaw,
	  pitch, and then roll.

	  For more see
	  http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	  which has additional links.
	*/

	/*
	  Software AHRS:
	  yaw   = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
	  pitch = -asinf(2.0f * (q[1] * q[3] - q[0] * q[2]));
	  roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
	  pitch *= 180.0f / PI;
	  yaw   *= 180.0f / PI;
	  yaw   += local_declination;
	  if (yaw < 0) {
	          yaw += 360.0f; // Ensure yaw stays between 0 and 360.
	  }
	  roll  *= 180.0f / PI;
	*/

	ahrs.a12 = 2.0f * (quat[1] * quat[2] + quat[0] * quat[3]);
	ahrs.a22 = quat[0] * quat[0] + quat[1] * quat[1] - quat[2] * quat[2] - quat[3] * quat[3];
	ahrs.a31 = 2.0f * (quat[0] * quat[1] + quat[2] * quat[3]);
	ahrs.a32 = 2.0f * (quat[1] * quat[3] - quat[0] * quat[2]);
	ahrs.a33 = quat[0] * quat[0] - quat[1] * quat[1] - quat[2] * quat[2] + quat[3] * quat[3];

	ahrs.pitch = -asinf(ahrs.a32);
	ahrs.roll  = atan2f(ahrs.a31, ahrs.a33);
	ahrs.yaw   = atan2f(ahrs.a12, ahrs.a22);
	ahrs.pitch *= 180.0f / M_PI;
	ahrs.yaw   *= 180.0f / M_PI;
	ahrs.yaw   += local_declination;
	if (ahrs.yaw < 0) {
		ahrs.yaw += 360.0f; /* Ensure yaw stays between 0 and 360. */
	}
	ahrs.roll  *= 180.0f / M_PI;

	ahrs.lin_ax = meas.ax + ahrs.a31;
	ahrs.lin_ay = meas.ay + ahrs.a32;
	ahrs.lin_az = meas.az - ahrs.a33;
}

#endif /* #if WITH_LOCAL_AHRS */
