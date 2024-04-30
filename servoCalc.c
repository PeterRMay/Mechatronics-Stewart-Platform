#include "servoCalc.h"

double servoCalc(double P_base_i[3][6], double B_i[3][6], double l[6], double s, double a, double beta, int maxAlphaRange, int minAlphaRange) {
    double L = l * l - (s * s - a * a);
    double M = 2 * a * (P_base_i[2] - B_i[2]);
    double N = 2 * a * (cos(deg2rad(beta)) * (P_base_i[0] - B_i[0]) + sin(deg2rad(beta)) * (P_base_i[1] - B_i[1]));

    double foo = L / sqrt(M * M + N * N);
    foo = fmax(-1, fmin(1, foo)); // Clamping foo to be within [-1, 1]

    double alpha = rad2deg(asin(foo)) - rad2deg(atan2(N, M));
    if (isnan(alpha)) {
        alpha = 0;
    }
    if (alpha > maxAlphaRange) {
        alpha = maxAlphaRange;
    } else if (alpha < minAlphaRange) {
        alpha = minAlphaRange;
    }
    return alpha;
}