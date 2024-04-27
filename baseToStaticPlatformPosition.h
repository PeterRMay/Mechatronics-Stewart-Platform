#ifndef BASE_TO_STATIC_PLATFORM_H
#define BASE_TO_STATIC_PLATFORM_H

/* includes */
#include <math.h>
#include "matrixMath.h"

/* prototypes */
typedef struct {
    Vector3 BP;
    Vector3 correctionAngles;
} PlatformPositionOutput;

/* global variables */
double BP[3];
double correctionAngles[3];

/* definitions*/
void baseToStaticPlatformPosition(double D[3], double Gamma[3], double T[3], double Phi[3], double h0) {
    double M[3][3];
    double gamma[3] = {Gamma[0] + Phi[0], Gamma[1] + Phi[1], Gamma[2] + Phi[2]};
    double OP[3] = {T[0], T[1], T[2] + h0};

    rotZYX(gamma, M);

    // OP - D
    double OP_minus_D[3];
    for (int i = 0; i < 3; i++) {
        OP_minus_D[i] = OP[i] - D[i];
    }

    // R * (OP - D)
    double temp[3];
    matrixMultiply(M, OP_minus_D, temp);

    // D + R*(OP - D)
    for (int i = 0; i < 3; i++) {
        BP[i] = D[i] + temp[i];
    }

    // Assigning correction angles
    for (int i = 0; i < 3; i++) {
        correctionAngles[i] = -Gamma[i];
    }
}