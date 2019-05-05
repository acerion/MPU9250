/*
  Original Copyright notice:

  MPU9250_MS5637_t3 Basic Example Code
  by: Kris Winer
  date: April 1, 2014
  license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.
*/




#if 0




unsigned char MS5637checkCRC(uint16_t * n_prom)  // calculate checksum from PROM register contents
{
	int cnt;
	unsigned int n_rem = 0;
	unsigned char n_bit;

	n_prom[0] = ((n_prom[0]) & 0x0FFF);  // replace CRC byte by 0 for checksum calculation
	n_prom[7] = 0;
	for (cnt = 0; cnt < 16; cnt++) {
		if (cnt%2==1) {
			n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
		} else {
			n_rem ^= (unsigned short)  (n_prom[cnt>>1]>>8);
		}
		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem<<1) ^ 0x3000;
			} else {
				n_rem = (n_rem<<1);
			}
		}
	}
	n_rem = ((n_rem>>12) & 0x000F);
	return (n_rem ^ 0x00);
}

void I2Cscan()
{
	// scan for i2c devices
	byte error, address;
	int nDevices;

	Serial.println("Scanning...");

	nDevices = 0;
	for (address = 1; address < 127; address++) {
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();

		if (error == 0) {
			Serial.print("I2C device found at address 0x");
			if (address<16) {
				Serial.print("0");
			}
			Serial.print(address,HEX);
			Serial.println("  !");

			nDevices++;
		} else if (error==4) {
			Serial.print("Unknow error at address 0x");
			if (address<16) {
				Serial.print("0");
			}
			Serial.println(address,HEX);
		}
	}
	if (nDevices == 0) {
		Serial.println("No I2C devices found\n");
	} else {
		Serial.println("done\n");
	}
}




void readAccelData(int16_t * destination)
{
	uint8_t rawData[6];  // x/y/z accel register data stored here
	readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}




void readGyroData(int16_t * destination)
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
	destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}




int16_t readTempData()
{
	uint8_t rawData[2];  // x/y/z gyro register data stored here
	readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
	return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}




#endif
