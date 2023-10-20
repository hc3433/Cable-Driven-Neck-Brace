#include "matrices.h"

//double** createMatrix(int m, int n)
//{
//	double** matrix = (double**)malloc(m * sizeof(double*));
//	for (int i = 0; i < m; i++) {
//		matrix[i] = (double*)malloc(n * sizeof(double));
//	}
//	return matrix;
//}


double** createMatrix(int m, int n)
{
    // allocate memory for the whole matrix
    double* data = (double*)malloc(m * n * sizeof(double));
    if (data == NULL) return NULL;

    // allocate memory for the row pointers
    double** matrix = (double**)malloc(m * sizeof(double*));
    if (matrix == NULL) {
        free(data); // free the data if allocation fails
        return NULL;
    }

    // assign the row pointers to the data
    for (int i = 0; i < m; i++) {
        matrix[i] = data + i * n;
    }

    // fill the matrix with random values
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            matrix[i][j] = rand() % 100; // change this to your desired range
        }
    }

    return matrix;
}

void freeMatrix( double** matrix, int m, int n)
{
	for (int i = 0; i < m; i++) {
		free(matrix[i]);
	}
	free(matrix);
}

void matrices_mul(double** A, double** B, double** C, int m, int n, int p)
{
    // Declare pointers to access the elements of A, B and C
    double* a, * b, * c;
    // Iterate over the rows of matrix A
    for (int i = 0; i < m; i++)
    {
        // Get the pointer to the ith row of A
        a = A[i];

        // Get the pointer to the ith row of C
        c = C[i];

        // Iterate over the columns of matrix B
        for (int j = 0; j < p; j++)
        {
            // Initialize the result element C[i][j] to 0
            c[j] = 0;
            // Iterate over the common dimension of A and B
            for (int k = 0; k < n; k++)
            {   
                // Get the pointer to the jth column of B
                b = B[k] + j;
                //printf("%f\n", *b);
                //// Multiply and add the corresponding elements of A and B
                //printf("%f * %f\n", a[k], *b);
                c[j] += a[k] * (*b);

                //// Move the pointer b to the next row of B
                //b += p;
            }
        }
    }
}
void matrices_scaler_mul(double** A, double a,int m,int n) {
    double* p;
	for (int i = 0; i < m; i++) {
		p = A[i];
       
		for (int j = 0; j < n; j++) {
            printf("%f \n", p[j]);
			p[j] *= a;
            
		}
	}
}

// function to stack a 7x7 identical matrix under a 6x7 jacobian matrix and make a new 13x7 A matrix
double** stackMatrix(double** jacobian, int m, int n)
{
    // allocate memory for the A matrix
    double** A = createMatrix(m + n, n);
    if (A == NULL) return NULL;

    // copy the jacobian matrix to the upper part of A
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            A[i][j] = jacobian[i][j];
        }
    }

    // copy the identical matrix to the lower part of A
    for (int i = m; i < m + n; i++) {
        for (int j = 0; j < n; j++) {
            A[i][j] = (i - m == j) ? 1.0 : 0.0; // change this to your desired identical matrix
        }
    }

    return A;
}

// function to convert a dense matrix to CSC format
void convertToCSC(double** dense, int m, int n, double* values, int* row_indices, int* col_pointers)
{
    int k = 0; // index for values and row_indices arrays
    col_pointers[0] = 0; // first column pointer is always zero

    // loop over the columns of the dense matrix
    for (int j = 0; j < n; j++) {
        // loop over the rows of the dense matrix
        for (int i = 0; i < m; i++) {
            // check if the element is nonzero
            if (dense[i][j] != 0.0) {
                // store the value and the row index in the arrays
                values[k] = dense[i][j];
                row_indices[k] = i;
                k++;
            }
        }
        // store the column pointer in the array
        col_pointers[j + 1] = k;
    }
}
