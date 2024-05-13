#include "matrixMath.h"
#include <math.h>

void rotX(double angles[3], double R[3][3]) {
    double phi = deg2rad(angles[0]);
    R[0][0] = 1; R[0][1] = 0;         R[0][2] = 0;
    R[1][0] = 0; R[1][1] = cos(phi);  R[1][2] = -sin(phi);
    R[2][0] = 0; R[2][1] = sin(phi);  R[2][2] = cos(phi);
}