#include "matrixMath.h"

double* matrixAdd33x33(double A[3][3], double B[3][3], double result[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = A[i][j] + B[i][j];
        }
    }
    
    return result;
}