#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>//exit()
#include "Vectors.h"

// This file is for the calculation and operation of vectors

// Declare vector operations as  functions for efficiency
void vector_sub(const vector* a, const vector* b, vector* c) {
	// Subtract two vectors and store the result in c, c = a-b
	c->x = a->x - b->x;
	c->y = a->y - b->y;
	c->z = a->z - b->z;
};

void vector_mul(const vector* a, const double k, vector* b) {
	// Multiply a vector by a scalar and store the result in b, b = a*k
	b->x = a->x * k;
	b->y = a->y * k;
	b->z = a->z * k;
};

void vector_add(const vector* a, const vector* b, vector* c) {
	// Add two vectors and store the result in c, c = a+b
	c->x = a->x + b->x;
	c->y = a->y + b->y;
	c->z = a->z + b->z;
};

double vector_dot(const vector* a, const vector* b) {
	// Compute the dot product of two vectors and return the result
	return a->x * b->x + a->y * b->y + a->z * b->z;
};

void vector_cross(const vector* a, const vector* b, vector* c) {
	// Compute the cross product of two vectors and store the result in c
	c->x = a->y * b->z - a->z * b->y;
	c->y = a->z * b->x - a->x * b->z;
	c->z = a->x * b->y - a->y * b->x;
};

double vector_norm(const vector* a) {
	// Compute the L2 norm (length) of a vector and return the result
	return (double)sqrt(vector_dot(a, a));
};

void vector_unit(const vector* a, vector* b) {
	// Compute the unit (normalized) vector and store the result in b
	vector_mul(a, 1.0 / vector_norm(a), b);
};

double vector_distance(const vector* a, const vector* b) {
	vector c;
	vector_sub(a, b, &c);
	return sqrt(c.x * c.x + c.y * c.y + c.z * c.z);
};

void convert_to_vectors(const double points[][Dim], vector vectors[], const int n) {
	for (int i = 0; i < n; i++) { // convert points arrays to vectors
		vectors[i].x = (double)points[i][0];
		vectors[i].y = (double)points[i][1];
		vectors[i].z = (double)points[i][2]; // points shape: nx3
	}
};

void new_vector_group_array(vector_group_array* vga, int size) {
	// Allocate memory for the array of vector_group pointers, vga means vectors group array
	vga->data = (vector_group**)malloc(size * sizeof(vector_group*));
	if (vga->data == NULL) {
		printf("Error: memory allocation failed\n");
		exit(1);
	};
	// Initialize the size to zero
	vga->size = 0;
};

void append_vector_group_array(vector_group_array* vga, vector_group* vg) {
	//if (vg == NULL) {
	//	//If needed
	//};
	// Append the new vector_group pointer to the array
	vga->data[vga->size] = vg;
	vga->size++;
};

void free_vector_group_array(vector_group_array* vga) {
	// Free the memory allocated for the array of vector_group pointers
	free(vga->data);
	free(vga);
};