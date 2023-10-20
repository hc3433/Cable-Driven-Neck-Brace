#include <stdio.h>//print
#include "Vectors.h"
#include "ForwardKinematics.h"
#include "test_func.h"

void print_vector(const vector* a, int size) {
	for (int i = 0; i < size; i++) {
		printf("Vector: %f %f %f\n", a[i].x, a[i].y, a[i].z);
	}
};

void verify_sphere(sphere* s, const int size) {
	printf("\nverify sphere list:\n\n");
	for (int i = 0; i < size; i++) {
		printf("No.%d : ", i);
		print_sphere(&s[i]);
	};
}

void print_sphere(sphere* s) {
	if (s == NULL) return;
	print_vector(&s->c, 1);
	printf("  radius: %f\n", s->r);
}

void print_vg(const vector_group* vg) {
	if (vg == NULL) return;
	printf("vector_group:\n");
	print_vector(&vg->v1, 1);
	print_vector(&vg->v2, 1);
	print_vector(&vg->v3, 1);
}

void print_vga(const vector_group_array* vga) {
	if (vga == NULL) return;
	printf("vector_group_array:\n");
	printf("\nsize: %d\n", vga->size);
	for (int i = 0; i < vga->size; i++) {
		printf("No.%d:\n", i);
		print_vg(vga->data[i]);
	}
}

void print_matrix(double** matrix, int m, int n)
{
	double* p;
	for (int i = 0; i < m; i++) {
		p = matrix[i];
		for (int j = 0; j < n; j++) {
			printf("%f ", p[j]);
		}
		printf("\n");

	}
	printf("\n");
}


// function to print a CSC format matrix
void printCSC(int m, int n, double* values, int* row_indices, int* col_pointers)
{
    printf("values: ");
    for (int i = 0; i < col_pointers[n]; i++) {
        printf("%f ", values[i]);
    }
    printf("\n");

    printf("row_indices: ");
    for (int i = 0; i < col_pointers[n]; i++) {
        printf("%d ", row_indices[i]);
    }
    printf("\n");

    printf("col_pointers: ");
    for (int j = 0; j <= n; j++) {
        printf("%d ", col_pointers[j]);
    }
    printf("\n");
}

// A function that prints a CSC matrix in dense form
void print_csc_matrix(OSQPCscMatrix* A) {
    // Check if the matrix is valid
    if (A == NULL || A->m <= 0 || A->n <= 0 || A->p == NULL || A->i == NULL || A->x == NULL) {
        printf("Invalid matrix\n");
        return;
    }
    printf("Matrix\n");
    // Loop through the rows of the matrix
    for (int i = 0; i < A->m; i++) {
        // Loop through the columns of the matrix
        for (int j = 0; j < A->n; j++) {
            // Get the start and end index of the column in the CSC arrays
            int start = A->p[j];
            int end = A->p[j + 1];

            // Initialize the element to zero
            double element = 0.0;

            // Check if there is a non-zero element in the current row and column
            for (int k = start; k < end; k++) {
                // If the row index matches, get the element value and break the loop
                if (A->i[k] == i) {
                    element = A->x[k];
                    break;
                }
            }

            // Print the element with a space
            printf("%f ", element);
        }

        // Print a new line after each row
        printf("\n");
    }
}