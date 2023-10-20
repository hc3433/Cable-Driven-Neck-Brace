#pragma once
#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>//Pi

double** createMatrix(int m, int n);
void freeMatrix( double** matrix, int m, int n);
void matrices_mul(double** A, double** B, double** C, int m, int n, int p);
void matrices_scaler_mul(double** A, double a, int m, int n);
double** stackMatrix(double** jacobian, int m, int n);
void convertToCSC(double** dense, int m, int n, double* values, int* row_indices, int* col_pointers);