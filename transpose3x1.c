#include "matrixMath.h"

void transpose3x1(double matrix[3][1], double transposed[1][3]) {
    for (int i = 0; i < 3; i++) {
        transposed[0][i] = matrix[i][0];
    }
}