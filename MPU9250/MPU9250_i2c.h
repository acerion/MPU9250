/*
  Original Copyright notice:

  MPU9250_MS5637_t3 Basic Example Code
  by: Kris Winer
  date: April 1, 2014
  license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.
*/




#ifndef _MPU9250_I2C_H_
#define _MPU9250_I2C_H_




uint8_t readByte(uint8_t address, uint8_t subAddress);
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);




#endif /* #ifndef _MPU9250_I2C_H_ */
