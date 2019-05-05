/*
  Original Copyright notice:

  MPU9250_MS5637_t3 Basic Example Code
  by: Kris Winer
  date: April 1, 2014
  license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.
*/




#include <Arduino.h> /* For 'Serial'. */




#include "MPU9250.h"
#include "MPU9250_debug_print.h"




void debug_print_meas(const mpu_meas_t & meas)
{
	Serial.print("ax = "); Serial.print((int) 1000 * meas.ax);
	Serial.print(" ay = "); Serial.print((int) 1000 * meas.ay);
	Serial.print(" az = "); Serial.print((int) 1000 * meas.az); Serial.print(" mg,");
	Serial.print(" gx = "); Serial.print(meas.gx, 2);
	Serial.print(" gy = "); Serial.print(meas.gy, 2);
	Serial.print(" gz = "); Serial.print(meas.gz, 2); Serial.print(" deg/s,");
	Serial.print(" mx = "); Serial.print((int) meas.mx);
	Serial.print(" my = "); Serial.print((int) meas.my);
	Serial.print(" mz = "); Serial.print((int) meas.mz); Serial.println(" mG");
}




#if WITH_LOCAL_AHRS
void debug_print_ypr(const mpu_ahrs_t & ahrs)
{
	Serial.print("Yaw, Pitch, Roll: ");
	Serial.print(ahrs.yaw, 2);
	Serial.print(", ");
	Serial.print(ahrs.pitch, 2);
	Serial.print(", ");
	Serial.println(ahrs.roll, 2);
}




void debug_print_quaternion(const float * quat)
{
	Serial.print("q0 = "); Serial.print(quat[0]);
	Serial.print(" qx = "); Serial.print(quat[1]);
	Serial.print(" qy = "); Serial.print(quat[2]);
	Serial.print(" qz = "); Serial.println(quat[3]);
}




void debug_print_other_ahrs(const mpu_ahrs_t & ahrs)
{
	Serial.print("Grav_x, Grav_y, Grav_z: ");
	Serial.print(-ahrs.a31 * 1000, 2);
	Serial.print(", ");
	Serial.print(-ahrs.a32 * 1000, 2);
	Serial.print(", ");
	Serial.print(ahrs.a33 * 1000, 2);  Serial.println(" mg");
	Serial.print("Lin_ax, Lin_ay, Lin_az: ");
	Serial.print(ahrs.lin_ax * 1000, 2);
	Serial.print(", ");
	Serial.print(ahrs.lin_ay * 1000, 2);
	Serial.print(", ");
	Serial.print(ahrs.lin_az * 1000, 2);  Serial.println(" mg");
}

#endif /* #if WITH_LOCAL_AHRS */
