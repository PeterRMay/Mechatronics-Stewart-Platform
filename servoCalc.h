#ifndef SERVO_CALC_H
#define SERVO_CALC_H

/* prototypes */
double servoCalc(double P_base_i[3], double B_i[3], double l, double s, double a, double beta, int maxAlphaRange, int minAlphaRange)
void servoCalcWrapper(double P_base[3][6], double B[3][6], double l_2norm[6], double s, double a, double beta[6], double alpha_max, double alpha_min, double alpha[6], double servoArms[3][6])

#endif