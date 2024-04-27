#ifndef MATRIX_MATH_H
#define MATRIX_MATH_H

/* includes */
#include <math.h>

/* prototypes */
double deg2rad(double degrees);
void rotX(double angles[3], double R[3][3]);
void rotY(double angles[3], double R[3][3]);
void rotZ(double angles[3], double R[3][3]);
void rotZYX(double angles[3], double R[3][3]);
void vectorAdd(double A[3], double B[3], double result[3]);
void vectorSubtract(double A[3], double B[3], double result[3]);
void matrixMultiply(double R[3][3], double v[3], double result[3]);
void matrixMultiply2(double A[3][3], double B[3][3], double result[3][3]);
void vectorScalarMultiply(double v[3], double scalar, double result[3]);

/* global variables */
#define PI 3.14159265358979323846

/* definitions */
double deg2rad(double degrees) {
    return degrees * (PI / 180.0);
}

void rotZ(double angles[3], double R[3][3]) {
    double psi = angles[0];
    R[0][0] = cos(psi);   R[0][1] = -sin(psi);   R[0][2] = 0;
    R[1][0] = sin(psi);   R[1][1] = cos(psi);    R[1][2] = 0;
    R[2][0] = 0;          R[2][1] = 0;           R[2][2] = 1;
}

void rotY(double angles[3], double R[3][3]) {
    double theta = angles[1];
    R[0][0] = cos(theta);  R[0][1] = 0; R[0][2] = sin(theta);
    R[1][0] = 0;           R[1][1] = 1; R[1][2] = 0;
    R[2][0] = -sin(theta); R[2][1] = 0; R[2][2] = cos(theta);
}

void rotX(double angles[3], double R[3][3]) {
    double phi = angles[2];
    R[0][0] = 1; R[0][1] = 0;         R[0][2] = 0;
    R[1][0] = 0; R[1][1] = cos(phi);  R[1][2] = -sin(phi);
    R[2][0] = 0; R[2][1] = sin(phi);  R[2][2] = cos(phi);
}

void rotZYX(double angles[3], double R[3][3]) {
    double psi = angles[0];   
    double theta = angles[1]; 
    double phi = angles[2];   

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

void vectorAdd(double A[3], double B[3], double result[3]) {
    result[0] = A[0] + B[0];
    result[1] = A[1] + B[1];
    result[2] = A[2] + B[2];
}

void vectorSubtract(double A[3], double B[3], double result[3]) {
    result[0] = A[0] - B[0];
    result[1] = A[1] - B[1];
    result[2] = A[2] - B[2];
}

void matrixMultiply31x31(double v1[3], double v2[3], double result[3]) {
    result[0] = v1[0] * v2[0];
    result[1] = v1[1] * v2[1];
    result[2] = v1[2] * v2[2];
}


void matrixMultiply33x31(double R[3][3], double v[3], double result[3]) {
    result[0] = R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2];
    result[1] = R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2];
    result[2] = R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2];
}

void matrixMultiply33x33(double A[3][3], double B[3][3], double result[3][3]) {
    int i,j,k;

    for (int i = 0; i < 3; i++) {        // loop over rows of A
        for (int j = 0; j < 3; j++) {    // loop over columns of B
            result[i][j] = 0.0;
            for (int k = 0; k < 3; k++) {  // loop for the dot product
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void vectorScalarMultiply(double v[3], double scalar, double result[3]) {
    result[0] = v[0] * scalar;
    result[1] = v[1] * scalar;
    result[2] = v[2] * scalar;
}

void transpose3x1(double matrix[3][1], double transposed[1][3]) {
    for (int i = 0; i < 3; i++) {
        transposed[0][i] = matrix[i][0];
    }
}

#endif