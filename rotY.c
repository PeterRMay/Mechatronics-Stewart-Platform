#include "matrixMath.h"
#include <math.h>

void rotY(double angles[3], double R[3][3]) {
    double theta = deg2rad(angles[1]);
    R[0][0] = cos(theta);  R[0][1] = 0; R[0][2] = sin(theta);
    R[1][0] = 0;           R[1][1] = 1; R[1][2] = 0;
    R[2][0] = -sin(theta); R[2][1] = 0; R[2][2] = cos(theta);
}