#include "matrixMath.h"

void vectorSubtract(double A[3], double B[3], double result[3]) {
    result[0] = A[0] - B[0];
    result[1] = A[1] - B[1];
    result[2] = A[2] - B[2];
}