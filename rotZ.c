#include "matrixMath.h"

void rotZ(double angles[3], double R[3][3]) {
    double psi = angles[0];
    R[0][0] = cos(psi);   R[0][1] = -sin(psi);   R[0][2] = 0;
    R[1][0] = sin(psi);   R[1][1] = cos(psi);    R[1][2] = 0;
    R[2][0] = 0;          R[2][1] = 0;           R[2][2] = 1;
}