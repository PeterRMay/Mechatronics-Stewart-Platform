#include "matrixMath.h"

void matrixMultiply31x31(double v1[3], double v2[3], double result[3]) {
    result[0] = v1[0] * v2[0];
    result[1] = v1[1] * v2[1];
    result[2] = v1[2] * v2[2];
}