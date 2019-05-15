My experiments with MPU9250.
Repo forked from kriswiner/MPU9250.

Using MPU9250_MS5637_AHRS_t3.ino as a basis for my purposes since it's "Most modern and correct version".

This project will focus on the following:
 * Getting raw measurements from IMU chip registers to Arduino over i2c,
 * Converting (on Arduino) the raw measurements to real acceleration/rotation/mag/temperature values (calculating real values from raw measurements using sensor scales and biases),
 * Sending the values, together with timestamp and checksum, from Arduino to PC (initially through serial port, later maybe by other means),
 * Receiving the values, checksum and timestamp on PC and displaying them to user. What is done further with the values on PC is another story and another project.
AHRS is not a part of this project. In particular, AHRS on Arduino is not part of this project. Refer to original repo (kriswiner/MPU9250) or one of its forks if you need AHRS to be executed on Arduino.

My HW:
 * Arduino Pro Mini 328 - 3.3V/8MHz from SparkFun
 * SparkFun IMU Breakout - MPU-9250 (9DOF, i2c)
 * PL2303 USB UART Board (micro) (programmer)

My SW:
 * Arduino IDE 1.0.5
   * menu Tools -> Programmer -> Arduino as ISP
 * Linux OS
   * Arduino's Robot_Control library removed from OS (https://github.com/arduino/Arduino/issues/1709).


