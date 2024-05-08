#ifndef SERVO_CALC_H
#define SERVO_CALC_H

/* prototypes */
int servoCalc(double alpha[6], double P_base[3][6], double B[3][6], double l[6], double s, double a, double beta[6], int maxAlphaRange, int minAlphaRange);

#endif