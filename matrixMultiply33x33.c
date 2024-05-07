#include "matrixMath.h"

void matrixMultiply33x33(double A[3][3], double B[3][3], double result[3][3]) {
    int i,j,k;

    for (i = 0; i < 3; i++) {        // loop over rows of A
        for (j = 0; j < 3; j++) {    // loop over columns of B
            result[i][j] = 0.0;
            for (k = 0; k < 3; k++) {  // loop for the dot product
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}