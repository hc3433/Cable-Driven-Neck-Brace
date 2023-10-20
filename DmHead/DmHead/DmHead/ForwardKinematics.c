#include <stdio.h>//print
#include <stdint.h>
#include <stdlib.h>//exit()
#define _USE_MATH_DEFINES
#include <math.h>//Pi
//project headers
#include "Vectors.h"
#include "ForwardKinematics.h"
#include "test_func.h"


// Find the intersection of three spheres
void create_sphere(const vector* center, const double* radius, sphere* s) {
	s->c = *center;
	s->r = *radius;
};

void trilaterate(sphere* s1, sphere* s2, sphere* s3, vector* v) {

	//Calculate some intermediate variables
	vector temp1, e_x, temp2, temp3, e_y, e_z;
	double i, j, d;

	vector_sub(&s2->c, &s1->c, &temp1); // temp1 = P2 - P1
	vector_unit(&temp1, &e_x); // e_x = temp1 / ||temp1||
	vector_sub(&s3->c, &s1->c, &temp2); // temp2 = P3 - P1
	i = vector_dot(&e_x, &temp2); // i = e_x dot temp2
	vector_mul(&e_x, i, &temp3); // temp3 = e_x * i
	vector_sub(&temp2, &temp3, &temp3); // temp3 = temp2 - temp3
	vector_unit(&temp3, &e_y); // e_y = temp3 / ||temp3||
	vector_cross(&e_x, &e_y, &e_z); // e_z = e_x cross e_y

	d = vector_norm(&temp1); // d = ||temp1||

	j = vector_dot(&e_y, &temp2); // j = e_y . temp2


	//Solve for x and y
	double x = (s1->r * s1->r - s2->r * s2->r + d * d) / (2 * d);
	double y = (s1->r * s1->r - s3->r * s3->r - 2 * i * x + i * i + j * j) / (2 * j);

	//Check if there is a solution for z
	double temp4 = s1->r * s1->r - x * x - y * y;

	if (temp4 < 0) {
		printf("The three spheres do not intersect!\n");
		/*exit(1);*/
	}

	//Solve for z
	double z = (double)sqrt(temp4);

	//Convert back to the original coordinate system
   //P_12_a = P1 + x*e_x + y*e_y + z*e_z 
	//&v[0]=p_12_a &v[1]=p_12_b
	vector_mul(&e_x, x, &v[0]); // p_12_a = e_x * x
	vector_mul(&e_y, y, &temp3); // temp3 = e_y * y
	vector_add(&v[0], &temp3, &v[0]); // p_12_a = p_12_a + temp3
	vector_mul(&e_z, z, &temp3); // temp3 = e_z * z
	vector_add(&v[0], &temp3, &v[0]); // p_12_a = p_12_a + temp3
	vector_add(&s1->c, &v[0], &v[0]); // p_12_a = P1 + p_12_a
	//P_12_b = P1 + x*e_x + y*e_y - z*e_z 
	vector_mul(&e_z, -z * 2, &temp3); // temp3 = e_z * 2*(-z)
	vector_add(&v[0], &temp3, &v[1]); // p_12_b = p_12_a + temp3
	//temp1 = temp2 = temp3 = e_x = e_y = e_z = (vector){ 0,0,0 };
	//i = d = j = temp4 = 0;
};

void Calculate_CableLength(uint32_t Present_position[], double Cable_initial_length[], double cableLength[], int size)
{
	if (cableLength == NULL) {
		printf("cableLength memory allocation failed\n");
		exit(1);
	}
	for (int i = 0; i < size; i++) {
		cableLength[i] = 57 * M_PI * 0.088 * Present_position[i] / 360 + Cable_initial_length[i];
	}
};

void* Brace_Anchor_distance(const double Brace_anchor_points[][Dim],double brace_dis []) {
	vector Brace_v[Anchor_number];
	convert_to_vectors(Brace_anchor_points, Brace_v, Anchor_number);
	//printf("Brace Anchor vectors:\n");
	//print_vector(Brace_v, Ancor_number);
	brace_dis[0] = vector_distance(&Brace_v[0], &Brace_v[1]);
	brace_dis[1] = vector_distance(&Brace_v[0], &Brace_v[2]);
};

void* possible_position_tree(tree* t, const vector* Cable_exit_v, sphere* s, const int s_size, double* cableLength, const double* brace_dis) {
	int i, j, k;
	anchor_group vg;
	node* v2, * v1, * v3;

	//initialize s0-s6
	for (int i = 0; i < nMotor; i++) {
		create_sphere(&Cable_exit_v[i], &cableLength[i], s + i);
	};

	trilaterate(&s[2], &s[3], &s[4], vg.v2);
	for (i = 0; i < 2; i++) {
		//s7-s8
		create_sphere(&vg.v2[i], &brace_dis[0], &s[i + nMotor]);
		v2 = create_node(vg.v2[i]);
		if (t->root == NULL) {
			t->root = v2;
		}
		else {
			v2->right = t->root; // set the right sibling of v2 to the root
			t->root = v2; // set the root to v2
		}
		trilaterate(&s[0], &s[1], &s[i + nMotor], &vg.v1[2 * i]);
		//print_sphere(&s[0]);
		//print_sphere(&s[1]);
		//print_sphere(&s[i + nMotor]);
		for (j = 0; j < 4; j++) {
			//s9-s12
			create_sphere(&vg.v1[j], &brace_dis[1], &s[j + nMotor + 2]);
			if (j / 2 == i) {
				v1 = create_node(vg.v1[j]);
				if (v2->left == NULL) {
					v2->left = v1;
				}
				else {
					v1->right = v2->left;
					v2->left = v1;
				}
				trilaterate(&s[5], &s[6], &s[j + nMotor + 2], &vg.v3[2 * j]);
				/*print_sphere(&s[5]);
				print_sphere(&s[6]);
				print_sphere(&s[j + nMotor + 2]);*/
				for (k = 0; k < 8; k++) {
					v3 = create_node(vg.v3[k]);
					if (k / 2 == j) {
						if (v1->left == NULL) {
							v1->left = v3;
						}
						else {
							v3->right = v1->left;
							v1->left = v3;
						}
					}
				}
			}
		}
	}
	//verify_sphere(s, s_size);
	//printf("\nBrace anchor point 2 : \n");
	//print_vector(vg.v2, 2);
	//printf("\nBrace anchor point 1 : \n");
	//print_vector(vg.v1, 4);
	//printf("\nBrace anchor point 3 : \n");
	//print_vector(vg.v3, 8);
};

// A function that creates a new node with a given vector value
node* create_node(vector v) {
	node* n = (node*)malloc(sizeof(node)); // Allocate memory for the node
	if (n == NULL) {
		printf("Error creating a new node.\n");
		exit(0);
	}
	n->v = v; // Set the vector value
	n->left = NULL; // Set the left child to NULL
	n->right = NULL; // Set the right child to NULL
	return n; // Return the node
}

int check_range(vector* v, double min, double max) {
	if (v->x >= min && v->x <= max && v->y >= min && v->y <= max && v->z >= min && v->z <= max) {
		return 1; // True
	}
	else {
		return 0;
	}
};


//			  v2[0]
//			 /     \
//		v1[0]       v1[1]
//		/  \	    /    \
//	v3[0]  v3[1] v3[2] v3[3]
//		\
//		v2[1]
//		 /    \
//	v1[2]      v1[3]
//	/  \         /  \
//v3[4] v3[5] v3[6] v3[7]


void print_node(node* n) {
	if (n != NULL) { // If the node is not NULL
		printf("(%f, %f, %f)\n", n->v.x, n->v.y, n->v.z); // Print the vector value
		printf("Left child: "); // Print the left child
		print_node(n->left); // Recursively print the left child
		printf("Right sibling: "); // Print the right sibling
		print_node(n->right); // Recursively print the right sibling
	}
	else { // If the node is NULL
		printf("NULL\n"); // Print NULL
	}
}

// A function that prints the tree
void print_tree(tree t) {
	printf("\nRoot: "); // Print the root
	print_node(t.root); // Recursively print the root and its children and siblings
}

double rad2deg(double rad) {
	return rad * 180 / M_PI;
}

void calculate_orientation(vector_group* vg, double* orientation) {
	// Calculate orientation based on 3 brace anchor points
	vector X, Y, Z;
	// Calculate X axis
	vector_sub(&vg->v1, &vg->v3, &X);
	vector_unit(&X, &X);
	// Calculate temp Y axis
	vector_sub(&vg->v1, &vg->v2, &Y);
	// Calculate Z axis
	vector_cross(&X, &Y, &Z);
	vector_unit(&Z, &Z);
	// Calculate Y axis
	vector_cross(&Z, &X, &Y);
	vector_unit(&Y, &Y);

	//printf("Axis: \n");
	//print_vector(&X, 1);
	//print_vector(&Y, 1);
	//print_vector(&Z, 1);
	double x = vector_norm(&X);
	double y = vector_norm(&Y);
	double z = vector_norm(&Z);
	//printf("Norm: \n");
	//printf("%f,  %f,  %f\n", x, y, z);
	vector dcm[3] = { X,Y,Z };
	//print_vector(dcm, 3);
	dcm2euler(dcm, orientation);
	//printf("\nphi = %f degrees\n", orientation[0]);
	//printf("theta = %f degrees\n", orientation[1]);
	//printf("psi = %f degrees\n", orientation[2]);
}

int check_angle_range(double* orientation, double min, double max)
{
	if (orientation[0] >= min && orientation[0] <= max && orientation[1] >= min && orientation[1] <= max && orientation[2] >= min && orientation[2] <= max) {
		//printf("orientation in the range\n");
		return 1; //True
	}
	else {

		printf("orientation out of range\n");
		return 0;
	}
}

// Modify your function to return a vector_group_array pointer instead of void
void output_pairs_array(vector_group_array* vga, tree t, double min, double max) {
	node* v2, * v1, * v3; // Pointers to the nodes
	vector_group* vg; // Pointer to a new vector group
	//vector_group_array* vga; // Pointer to a new vector group array
	int capacity = 8; // Initial capacity for the vector group array
	// Create a new vector group array with the initial capacity
	new_vector_group_array(vga, capacity);
	//print_vga(vga);
	v2 = t.root; // Start from the root node
	while (v2 != NULL) { // While v2 is not NULL
		// Check if v2 is within the range and does not contain -nan
		if (check_range(&v2->v, min, max) && !isnan(v2->v.x) && !isnan(v2->v.y) && !isnan(v2->v.z)) {
			v1 = v2->left; // Go to the left child of v2
			while (v1 != NULL) { // While v1 is not NULL
				// Check if v1 is within the range and does not contain -nan
				if (check_range(&v1->v, min, max) && !isnan(v1->v.x) && !isnan(v1->v.y) && !isnan(v1->v.z)) {
					v3 = v1->left; // Go to the left child of v1
					while (v3 != NULL) { // While v3 is not NULL
						// Check if v3 is within the range and does not contain -nan
						if (check_range(&v3->v, min, max) && !isnan(v3->v.x) && !isnan(v3->v.y) && !isnan(v3->v.z)) {
							// Allocate memory for a new vector group that stores the three vectors
							vg = (vector_group*)malloc(sizeof(vector_group));
							// Check if the allocation was successful
							if (vg == NULL) {
								printf("Memory allocation failed\n");
								exit(1);
							}
							// Copy the vectors from v2, v1 and v3 to vg
							vg->v1 = v1->v;
							vg->v2 = v2->v;
							vg->v3 = v3->v;
							// Append the new vector group pointer to the vector group array
							append_vector_group_array(vga, vg);
						}
						v3 = v3->right; // Go to the right sibling of v3
					}
				}
				v1 = v1->right; // Go to the right sibling of v1
			}
		}
		v2 = v2->right; // Go to the right sibling of v2
	}
	// Return the vector group array pointer
}


// Define a function to convert an array of three vectors into a double array of euler angles
void dcm2euler(vector dcm[3], double* orientation) {
	// Check for singularity at theta = +/- 90 degrees
	if (dcm[0].z == 1 || dcm[0].z == -1) {
		printf("Singularity at theta = +/- 90 degrees\n");
		orientation[0] = 0;
		orientation[1] = rad2deg(asin(dcm[2].x));
		orientation[2] = rad2deg(orientation[0] + atan2(dcm[0].y, dcm[0].z));
	}
	else {
		orientation[0] = rad2deg(atan2(-dcm[2].y, dcm[2].z));
		orientation[1] = rad2deg(asin(dcm[2].x));
		orientation[2] = rad2deg(atan2(-dcm[1].x, dcm[0].x));
	}
}

void calculate_position(vector_group* vg, vector* posi) {
	// Calculate the position of the vector group
	posi->x = (vg->v1.x + vg->v3.x) / 2;
	posi->y = (vg->v1.y + vg->v3.y) / 2;
	posi->z = (vg->v1.z + vg->v3.z) / 2;
};

// Define a function to calculate the POSSIBLE position and orientation of a vector group
void calculate_position_orientation(vector_group_array* vga, position_orientation* posi_orien) {
	double orientation[3];
	for (int i = 0; i < vga->size; i++) {
		// Calculate the position of the vector group
		calculate_position(vga->data[i], &posi_orien->position);
		// Calculate the orientation of the vector group
		calculate_orientation(&vga->data[i], orientation);
		// Check if the orientation is within the range
		if (check_angle_range(orientation, -60, 60)) {
			// Copy the orientation to the posi_orien struct
			posi_orien->orientation[0] = orientation[0];
			posi_orien->orientation[1] = orientation[1];
			posi_orien->orientation[2] = orientation[2];
			// Print the position and orientation
			//printf("Position: \n");
			//print_vector(&posi_orien->position, 1);
			//printf("Orientation: \n");
			//printf("phi = %f degrees\n", posi_orien->orientation[0]);
			//printf("theta = %f degrees\n", posi_orien->orientation[1]);
			//printf("psi = %f degrees\n\n", posi_orien->orientation[2]);
		}
	}
};

//void calculate_RsE(double** Rs, double** Bc, double** RsE, position_orientation* posi_orient) {
//
//	double* phi = &posi_orient->orientation[0];
//	double* theta = &posi_orient->orientation[1];
//	double* psi = &posi_orient->orientation[2];
//
//	// calculate RsE
//	Rs[0][0] = cos(*theta) * cos(*psi);
//	Rs[0][1] = cos(*theta) * -sin(*psi);
//	Rs[0][2] = sin(*theta);
//	Rs[1][0] = sin(*phi) * sin(*theta) * cos(*psi) + cos(*phi) * sin(*psi);
//	Rs[1][1] = sin(*phi) * sin(*theta) * -sin(*psi) + cos(*phi) * cos(*psi);
//	Rs[1][2] = -sin(*phi) * cos(*theta);
//	Rs[2][0] = -cos(*phi) * sin(*theta) * cos(*psi) + sin(*phi) * sin(*psi);
//	Rs[2][1] = cos(*phi) * sin(*theta) * sin(*psi) + sin(*phi) * cos(*psi);
//	Rs[2][2] = cos(*phi) * cos(*theta);
//	matrices_mul(Rs, Bc, RsE, 3, 3, 3);
//	// Initialize the matrices A and B with some values
//	printf("Rs\n"); print_matrix(Rs, 3, 3);
//	printf("Bc\n"); print_matrix(Bc, 3, 3);
//	printf("RsE\n"); print_matrix(RsE, 3, 3);
//}
//
//void calculate_b(vector* b[], double** RsE) {
//	b[0] = RsE[0];
//	b[1] = RsE[0];
//	b[2] = RsE[1];
//	b[3] = RsE[1];
//	b[4] = RsE[1];
//	b[5] = RsE[2];
//	b[6] = RsE[2];
//}
//
//void calculate_q(vector* Cable_exit_v, vector* position, double** RsE, vector** q) {
//	vector** b;
//	b = malloc(sizeof(vector*) * 7);
//	if (b == NULL) {
//		printf("b Memory allocation failed\n");
//		return 1;
//	}
//	calculate_b(b, RsE);
//	q[0] = malloc(sizeof(vector) * 7);
//	for (int i = 0; i < 7; i++) {
//		q[i] = q[0] + i;
//		vector_sub(&Cable_exit_v[i], b[i], q[i]);
//		vector_sub(q[i], position, q[i]);
//		vector_unit(q[i], q[i]);
//		print_vector(q[i], 1);
//	};
//}

void calculate_RsE_b_q(double** Rs, double** Bc, double** RsE, position_orientation* posi_orient, vector* Cable_exit_v, vector* position, vector* b[], vector* q[]) {
	// get the orientation angles from posi_orient
	double phi = posi_orient->orientation[0];
	double theta = posi_orient->orientation[1];
	double psi = posi_orient->orientation[2];

	// calculate RsE
	Rs[0][0] = cos(theta) * cos(psi);
	Rs[0][1] = cos(theta) * -sin(psi);
	Rs[0][2] = sin(theta);
	Rs[1][0] = sin(phi) * sin(theta) * cos(psi) + cos(phi) * sin(psi);
	Rs[1][1] = sin(phi) * sin(theta) * -sin(psi) + cos(phi) * cos(psi);
	Rs[1][2] = -sin(phi) * cos(theta);
	Rs[2][0] = -cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);
	Rs[2][1] = cos(phi) * sin(theta) * sin(psi) + sin(phi) * cos(psi);
	Rs[2][2] = cos(phi) * cos(theta);
	matrices_mul(Rs, Bc, RsE, 3, 3, 3);

	// print the matrices Rs, Bc, and RsE
	//printf("Rs\n"); print_matrix(Rs, 3, 3);
	//printf("Bc\n"); print_matrix(Bc, 3, 3);
	//printf("RsE\n"); print_matrix(RsE, 3, 3);

	// calculate b and q
	b[0] = RsE[0];
	b[1] = RsE[0];
	b[2] = RsE[1];
	b[3] = RsE[1];
	b[4] = RsE[1];
	b[5] = RsE[2];
	b[6] = RsE[2];
	q[0] = malloc(sizeof(vector) * 7);
	for (int i = 0; i < 7; i++) {
		q[i] = q[0] + i;
		vector_sub(&Cable_exit_v[i], b[i], q[i]);
		vector_sub(q[i], position, q[i]);
		vector_unit(q[i], q[i]);
		//print_vector(q[i], 1);
	}
}

void calculate_jacobian(double** J, vector* b[], vector* q[]) {
	vector bq[7];
	for (int i = 0; i < 7; i++) {
		J[0][i] = q[i]->x;
		J[1][i] = q[i]->y;
		J[2][i] = q[i]->z;
		vector_cross(b[i], q[i], &bq[i]);
		J[3][i] = bq[i].x / 1000;
		J[4][i] = bq[i].y / 1000;
		J[5][i] = bq[i].z / 1000;
	}
}

//{
//    // Declare pointers to access the elements of A, B and C
//    double* a, * b, * c;
//    // Iterate over the rows of matrix A
//    for (int i = 0; i < m; i++)
//    {
//        // Get the pointer to the ith row of A
//        a = A[i];
//
//        // Get the pointer to the ith row of C
//        c = C[i];
//
//        // Iterate over the columns of matrix B
//        for (int j = 0; j < p; j++)
//        {
//            // Initialize the result element C[i][j] to 0
//            c[j] = 0;
//            // Iterate over the common dimension of A and B
//            for (int k = 0; k < n; k++)
//            {   
//                // Get the pointer to the jth column of B
//                b = B[k] + j;
//                //printf("%f\n", *b);
//                //// Multiply and add the corresponding elements of A and B
//                //printf("%f * %f\n", a[k], *b);
//                c[j] += a[k] * (*b);
//
//                //// Move the pointer b to the next row of B
//                //b += p;
//            }
//        }
//    }
//}