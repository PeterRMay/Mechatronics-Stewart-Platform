#include "matrixMath.h"

double* matrixSubtract33x33(double A[3][3], double B[3][3], double result[3][3]) {
    int i,j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            result[i][j] = A[i][j] - B[i][j];
        }
    }
}