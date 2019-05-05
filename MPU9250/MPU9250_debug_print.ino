/*
  Original Copyright notice:

  MPU9250_MS5637_t3 Basic Example Code
  by: Kris Winer
  date: April 1, 2014
  license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.
*/


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




void debug_print_ypr(const mpu_calc_t & calc)
{
	Serial.print("Yaw, Pitch, Roll: ");
	Serial.print(calc.yaw, 2);
	Serial.print(", ");
	Serial.print(calc.pitch, 2);
	Serial.print(", ");
	Serial.println(calc.roll, 2);
}




void debug_print_quaternion(const float * a_q)
{
	Serial.print("q0 = "); Serial.print(q[0]);
	Serial.print(" qx = "); Serial.print(q[1]);
	Serial.print(" qy = "); Serial.print(q[2]);
	Serial.print(" qz = "); Serial.println(q[3]);
}




void debug_print_other(const mpu_calc_t & calc)
{
	Serial.print("Grav_x, Grav_y, Grav_z: ");
	Serial.print(-calc.a31 * 1000, 2);
	Serial.print(", ");
	Serial.print(-calc.a32 * 1000, 2);
	Serial.print(", ");
	Serial.print(calc.a33 * 1000, 2);  Serial.println(" mg");
	Serial.print("Lin_ax, Lin_ay, Lin_az: ");
	Serial.print(calc.lin_ax * 1000, 2);
	Serial.print(", ");
	Serial.print(calc.lin_ay * 1000, 2);
	Serial.print(", ");
	Serial.print(calc.lin_az * 1000, 2);  Serial.println(" mg");
}
