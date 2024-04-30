#ifndef MATRIX_MATH_H
#define MATRIX_MATH_H

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

#endif