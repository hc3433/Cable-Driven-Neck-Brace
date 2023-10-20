#pragma once
#include <stdint.h>

typedef struct sphere {
	vector c;
	double r;
}sphere;


//vector group for 2+4+8 brace anchor points
typedef struct anchor_group {
	vector v2[2];
	vector v1[4];
	vector v3[8];
}anchor_group;

typedef struct node {
	vector v;
	struct node* left;
	struct node* right;
} node;

// A tree structure that contains a pointer to the root node
typedef struct tree {
	node* root;
} tree;

typedef struct position_orientation {
	vector position; // contains x,y,z
	double orientation[3];
}position_orientation;

typedef struct Jacobian {
	double J[6][7];
}Jacobian;

//Find the intersection of three spheres
extern inline void create_sphere(const vector* center, const double* radius, sphere* s);
extern inline void trilaterate(sphere* s1, sphere* s2, sphere* s3, vector* v);

//

void Calculate_CableLength(uint32_t Present_position[], double Cable_initial_length[], double cableLength[], int size);
void* Brace_Anchor_distance(const double Brace_anchor_points[][Dim], double brace_dis[]);
//
void* possible_position_tree(tree* t, const vector* Cable_exit_v, sphere* s, const int s_size, double* cableLength, const double* brace_dis);
node* create_node(vector v);
int check_range(vector* v, double min, double max);
int check_angle_range(double* orientation, double min, double max);
void print_node(node* n);
void print_tree(tree t);


void output_pairs_array(vector_group_array* vga, tree t, double min, double max);
//
double rad2deg(double rad);
void calculate_orientation(vector_group* vg, double* orientation);
void dcm2euler(vector dcm[3], double* orientation);
void calculate_position(vector_group* vg, vector* position);
void calculate_position_orientation(vector_group_array* vga, position_orientation* posi_orien);
//
void calculate_RsE(double** Rs, double** Bc, double** RsE, position_orientation* posi_orient);
void calculate_b(vector* b[], double** RsE);
void calculate_q(vector* Cable_exit_v, vector* position, double** RsE, vector** q);

void calculate_RsE_b_q(double** Rs, double** Bc, double** RsE, position_orientation* posi_orient, vector* Cable_exit_v, vector* position, vector* b[], vector* q[]);

void calculate_jacobian(double** J, vector* b[], vector* q[]);