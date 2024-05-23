#include "baseToStaticPlatformPosition.h"
#include "matrixMath.h"

void baseToStaticPlatformPosition(double BP[3], double correctionAngles[3], double D[3], double Gamma[3], double T[3], double Phi[3], double h0) {
    double M[3][3];
    double MTranspose[3][3];
    double gamma[3] = {-Gamma[0] + Phi[0], -Gamma[1] + Phi[1], -Gamma[2] + Phi[2]};
    double OP[3] = {T[0], T[1], T[2] + h0};
    int i, j;

    rotZYX(Gamma, M);
    for (i = 0; i < 3; i++) {
        MTranspose[i][i] = M[i][i];
        MTranspose[0][i] = M[i][0];
        MTranspose[i][0] = M[0][i];
    }
    MTranspose[1][2] = M[2][1];
    MTranspose[2][1] = M[1][2];

    // OP - D
    double OP_minus_D[3];
    for (i = 0; i < 3; i++) {
        OP_minus_D[i] = OP[i] - D[i];
    }

    // R * (OP - D)
    double temp[3];
    matrixMultiply33x31(MTranspose, OP_minus_D, temp);

    // D + R*(OP - D)
    for (i = 0; i < 3; i++) {
        BP[i] = D[i] + temp[i];
    }

    // Assigning correction angles
    for (i = 0; i < 3; i++) {
        correctionAngles[i] = gamma[i];
    }
}