#include "matrixMath.h"

void vectorScalarMultiply(double v[3], double scalar, double result[3]) {
    result[0] = v[0] * scalar;
    result[1] = v[1] * scalar;
    result[2] = v[2] * scalar;
}