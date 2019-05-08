#ifndef _H_QUATERNION_FILTERS_H_
#define _H_QUATERNION_FILTERS_H_




void MadgwickQuaternionUpdate(float * q, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);
void MahonyQuaternionUpdate(float * q, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);




#endif /* #ifndef _H_QUATERNION_FILTERS_H_ */
