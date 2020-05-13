#ifndef MADGWICKAHRS_H_
#define MADGWICKAHRS_H_

void MadgwickSetBeta(float _beta);
void MadgwickSetDelta(float _deltat);
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float *angle);

#endif /* MADGWICKAHRS_H_ */
