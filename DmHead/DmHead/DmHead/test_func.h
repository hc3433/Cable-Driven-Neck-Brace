#pragma once
#include "osqp.h"
void print_sphere(sphere* s);
void verify_sphere(sphere* s, const int size);
void print_vector(const vector* a, int size);
void print_vg(const vector_group* vg);
void print_vga(const vector_group_array* vga);
void print_matrix(double** matrix, int m, int n);
void printCSC(int m, int n, double* values, int* row_indices, int* col_pointers);
void print_csc_matrix(OSQPCscMatrix* A);