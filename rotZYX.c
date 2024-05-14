#include "matrixMath.h"
#include <math.h>

void rotZYX(double angles[3], double R[3][3]) {
    double psi = deg2rad(angles[2]);   
    double theta = deg2rad(angles[1]); 
    double phi = deg2rad(angles[0]);   

    R[0][0] = cos(psi)*cos(theta);
    R[0][1] = -sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi);
    R[0][2] = sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi);

    R[1][0] = sin(psi)*cos(theta);
    R[1][1] = cos(psi)*cos(phi) + sin(psi)*sin(theta)*sin(phi);
    R[1][2] = -cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi);

    R[2][0] = -sin(theta);
    R[2][1] = cos(theta)*sin(phi);
    R[2][2] = cos(theta)*cos(phi);
}