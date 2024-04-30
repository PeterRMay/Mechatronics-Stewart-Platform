#include "servoCalc.h"

void servoCalcWrapper(double P_base[3][6], double B[3][6], double l_2norm[6], double s, double a, double beta[6], double alpha_max, double alpha_min, double alpha[6], double servoArms[3][6]) {
    bool errorFlag = false;
    double angles[3];
    double Rz[3][3], Ry[3][3], Rzyx[3][3];
    double temp[3], tempVec[3] = {1, 0, 0};

    for (int i = 0; i < 6; i++) {
        alpha[i] = servoCalc(P_base[i], B[i], l_2norm[i], s, a, beta[i], (int)alpha_max, (int)alpha_min);

        // Using the matrix operations from the header for rotations
        angles[0] = deg2rad(beta[i]);
        angles[1] = deg2rad(alpha[i]);
        angles[2] = 0;  // No rotation around X-axis
        rotZ(angles, Rz);
        rotY(angles, Ry);

        matrixMultiply33x33(Rz, Ry, Rzyx);  // Combine rotations
        matrixMultiply33x31(Rzyx, tempVec, temp);   // Apply the rotation to the vector [1; 0; 0]

        // Scale by `a` and add `B[:,i]`
        vectorScalarMultiply(temp, a, temp);
        vectorAdd(B[i], temp, servoArms[i]);
    }

    // Saturate servo angles
    for (int i = 0; i < 6; i++) {
        if (alpha[i] < alpha_min) {
            errorFlag = true;
            alpha[i] = alpha_min;
        } else if (alpha[i] > alpha_max) {
            errorFlag = true;
            alpha[i] = alpha_max;
        }
    }
}