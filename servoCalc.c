#include "servoCalc.h"

int servoCalc(double alpha[6], double P_base[3][6], double B[3][6], double legLengths[6], double s, double a, double beta[6], int maxAlphaRange, int minAlphaRange) {
    double L, M, N, foo;
    int errorFlag = 0;
    int i;
    for (i = 0; i < 6; i++) {
        L = legLengths[i] * legLengths[i] - (s * s - a * a);
        M = 2 * a * (P_base[2][i] - B[2][i]);
        N = 2 * a * (cos(deg2rad(beta)) * (P_base[0][i] - B[0][i]) + sin(deg2rad(beta)) * (P_base[1][i] - B[1][i]));
        foo = L / sqrt(M * M + N * N);
        if (foo < -1 || foo > 1) {
            errorFlag = 1;
        }
        foo = fmax(-1, fmin(1, foo)); // Clamping foo to be within [-1, 1]
        alpha[i] = rad2deg(asin(foo)) - rad2deg(atan2(N, M));
        if (isnan(alpha[i])) {
            alpha[i] = 0;
            errorFlag = 1;
        }
        if (alpha[i] > maxAlphaRange) {
            alpha[i] = maxAlphaRange;
            errorFlag = 1;
        } else if (alpha < minAlphaRange) {
            alpha[i] = minAlphaRange;
            errorFlag = 1;
        }
    }
    
    return errorFlag;
}