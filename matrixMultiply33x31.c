#include "matrixMath.h"

void matrixMultiply33x31(double R[3][3], double v[3], double result[3]) {
    result[0] = R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2];
    result[1] = R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2];
    result[2] = R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2];
}