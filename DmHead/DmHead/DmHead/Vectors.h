#pragma once
#include "Shared_definition.h"
// Define a vector struct with three components
typedef struct {
	double x;
	double y;
	double z;
} vector;

typedef struct {
	vector v1;
	vector v2;
	vector v3;
} vector_group;


typedef struct {
	vector_group** data; // Pointer to the array of vector_group pointers
	int size; // Number of elements in the array
} vector_group_array;





// Declare vector operations as inline functions for efficiency
extern inline void vector_sub(const vector* a, const vector* b,vector* c);
extern inline void vector_mul(const vector* a, const double k, vector* b);
extern inline void vector_add(const vector* a, const vector* b, vector* c);
extern inline double vector_dot(const vector* a, const vector* b);
extern inline void vector_cross(const vector* a, const vector* b, vector* c);
extern inline double vector_norm(const vector* a);
extern inline void vector_unit(const vector* a, vector* b);
extern inline double vector_distance(const vector* a, const vector* b);
extern inline void convert_to_vectors(const double points[][Dim], vector vectors[], const int n);
void new_vector_group_array(vector_group_array* vga, int size);
void append_vector_group_array(vector_group_array* vga,vector_group* vg);
void free_vector_group_array(vector_group_array* vga);


