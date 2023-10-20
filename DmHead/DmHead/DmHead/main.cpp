#pragma warning(disable:4996)
#include <stdio.h>//print
#include <stdint.h>
#include <stdlib.h>//exit()
#define _USE_MATH_DEFINES
#include <math.h>// isnan
#include <string.h>//memcpy
#include <time.h>
#define _X86_  // or x64?
#include <sysinfoapi.h>
#define _Win32 // or Win64?
#define _WINSOCK_DEPRECATED_NO_WARNINGS

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#else
#include <unistd.h>
#endif

extern "C" {
	// Dynamixel SDK library 
	#include "dynamixel_sdk.h"

	// QP solver library
	#include "osqp.h" // Oxford lib for quadratic programming. Other options: ALGLIB (Robert is using).

	// project headers
	#include "Shared_definition.h"
	#include "Motor_Settings.h"
	#include "Vectors.h"
	#include "ForwardKinematics.h"
	#include "Matrices.h"
	#include "UDPechoServer.h"
	#include "osqp_api_types.h"
	#include "test_func.h"
	#include "SaveData.h"
}


int cgetch()  // from cohio.h, returns the ASCII value of the single character read from stdin
{
#if defined(__linux__) || defined(__APPLE__)
	struct termios oldt, newt;
	int ch;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	ch = getchar();
	ch = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	return ch;
#elif defined(_WIN32) || defined(_WIN64)
	return _getch();
#endif
}

int ckbhit(void) // from cohio.h, determine if a key has been pressed or not
{
#if defined(__linux__) || defined(__APPLE__)
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if (ch != EOF)
	{
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
#elif defined(_WIN32) || defined(_WIN64)
	return _kbhit();
#endif
}


static double Cable_exit_points[nMotor][Dim] = {
	{236.887	,-72.0126	,263.33},
	{255.647	,-57.0921	,-475.35},
	{101.038	,-260.012	,268.956},
	{-5.6		,-259.987	,-198.07},
	{-95.52		,-273.5		,274.63},
	{-253.232	,-26.58		,-495.86},
	{-240.247	,-72.73		,279.55} };

// TODO set this as dynamic variable
// Range of present_position : [0 ~ 4095]
//uint32_t Present_position[nMotor] = { 2000,2000,500,2000,1000,200,3000 };
uint32_t Present_position[nMotor] = { 3000,500,10,10,10,10,10 };
static double Cable_initial_length[nMotor] = { 165,480,150,345,170,620,250 };

static double Brace_anchor_points[Anchor_number][Dim] = { // attachment points
	{170.078,5,0.0053},
	{-0.39583,-130.503,-0.00519},
	{-170.87,5,0.00507} };

// The upon static variables should be measured by Mocap system during benchtop working
// However, we can change them ourselves when testing codes

int main() {
	// cables length calculation
	double* cableLength = (double*)malloc(sizeof(double) * nMotor);  // malloc make its size changeable
	if (cableLength == NULL) { printf("Memory allocation failed\n"); return 1; }
	Calculate_CableLength(Present_position, Cable_initial_length, cableLength, nMotor); // Included in FK.h
	printf("Cable Length : \n"); for (int i = 0; i < nMotor; i++) { printf("%f\n", cableLength[i]); }
	printf("\n\n");

	//brace anchor distance
	double* brace_dis = (double*)malloc(sizeof(double) * 2);
	if (brace_dis == NULL) { printf("Memory allocation failed\n"); return 1; }
	Brace_Anchor_distance(Brace_anchor_points, brace_dis); // Included in FK.h
	printf("brace dis:\n"); for (int i = 0; i < 2; i++) { printf("%f\n", brace_dis[i]); }

	//Convert to vectors
	vector* Cable_exit_v = (vector*)malloc(sizeof(vector) * nMotor);
	if (Cable_exit_v == NULL) { printf("Memory allocation failed\n"); return 1; }
	//vector Cable_exit_v[nMotor];
	convert_to_vectors(Cable_exit_points, Cable_exit_v, nMotor); // Included in Vectors.h
	printf("Cable exit vectors\n");
	print_vector(Cable_exit_v, nMotor);

	// At least 7+2+2*2=13 spheres for forward kinematics
	// s1-s7: motors  s8, s9: probable BraceAnc 2->1  s10-s13 probable BraceAnc 1->3
	struct sphere s[13];
	//s = (struct sphere){};
	int s_size = sizeof(s) / sizeof(s[0]);
	printf("sphere created\n");
	//verify_sphere(s,13);
	//create vector group for 2+4+8 points of three brace anchor points
	tree t;
	t.root = NULL; // node at the top of the tree
	possible_position_tree(&t, Cable_exit_v, s, s_size, cableLength, brace_dis);
	print_tree(t);
	vector_group_array* vga;
	//vga = (vector_group_array*)malloc(sizeof(vector_group_array));
	vga = (vector_group_array*)calloc(8, sizeof(vector_group_array)); // = malloc + 0 initialized
	print_vga(vga);
	output_pairs_array(vga, t, -260, 260);
	// Modify vga to return a vector_group_array pointer instead of void
	print_vga(vga);

	//double* orientation;
	//orientation=(double*)malloc(sizeof(double) * 3);
	//calculate_orientation(vga->data[0], orientation);	


	//double orientation[3];
	//for (int i = 0; i < vga->size; i++) {
	//	printf("orientation %d\n", i);
	//	calculate_orientation(vga->data[i], orientation);
	//	check_angle_range(orientation, -50, 50);
	//};
	position_orientation* posi_orient;
	posi_orient = static_cast<position_orientation*>(malloc(sizeof(position_orientation))); // static_cast
	calculate_position_orientation(vga, posi_orient);

	//Just for test
	posi_orient->orientation[0] = 0;
	posi_orient->orientation[1] = 0;
	posi_orient->orientation[2] = 0;
	posi_orient->position.x = 0;
	posi_orient->position.y = -100;
	posi_orient->position.z = -50;
	//
	
	double** Rs = createMatrix(3, 3); //a pointer to a pointer to a double variable
	double** Bc = createMatrix(3, 3);
	double** RsE = createMatrix(3, 3);
	for (int i = 0; i < 3; i++)
	{
		memcpy(Bc[i], Brace_anchor_points[i], 3 * sizeof(double));
	}

	//for (int i = 0; i < Ancor_number; i++) {
	//	for (int j = 0; j < Dim; j++) {
	//		Bc[i][j] = Brace_anchor_points[i][j];
	//	}
	//}

	vector** b;
	b = static_cast<vector**>(malloc(sizeof(vector*) * 7));
	if (b == NULL) {
		printf("b Memory allocation failed\n");
		return 1;
	}
	vector** q;
	q = static_cast<vector**>(malloc(sizeof(vector*) * 7));
	if (q == NULL) {
		printf("q Memory allocation failed\n");
		return 1;
	}
	calculate_RsE_b_q(Rs, Bc, RsE, posi_orient, Cable_exit_v, &posi_orient->position, b, q);
	//calculate Jacobian
	printf("Jacobian\n");
	double** J = createMatrix(6, 7);
	calculate_jacobian(J, b, q);
	print_matrix(J, 6, 7);

	//QP test
	/* Load problem data */
	OSQPInt n = 7;
	OSQPInt m = 13;
	OSQPFloat P_x[7] = { 2,2,2,2,2,2,2 };
	OSQPInt P_nnz = 7;
	OSQPInt P_i[7] = { 0,1,2,3,4,5,6, };
	OSQPInt P_p[8] = { 0,1,2,3,4,5,6,7, };
	OSQPCscMatrix* P = static_cast<OSQPCscMatrix*>(malloc(sizeof(OSQPCscMatrix)));
	csc_set_data(P, n, n, P_nnz, P_x, P_i, P_p);
	print_csc_matrix(P);
	printf("CSC format matrix:\n");
	printCSC(n, n, P_x, P_i, P_p);

	OSQPFloat q_[7] = { 0,0,0,0,0,0,0, };
	OSQPFloat l[13] = { -5,-5,-5,-5,-5,5.5,
						.5,.5,.5,.5,.5,.5,.5, };
	OSQPFloat u[13] = { 5,5,5,5,5,6.5,
						120,120,120,120,120,120,120 };

	// stack a 7x7 identical matrix under the jacobian matrix and make a new 13x7 A matrix
	double** A = stackMatrix(J, 6, 7);
	// print the A matrix
	printf("A matrix:\n");
	print_matrix(A, 13, 7);
	// allocate memory for the CSC format arrays
	OSQPFloat* A_x = (double*)malloc(13 * 7 * sizeof(double));
	OSQPInt* A_i = (int*)malloc(13 * 7 * sizeof(int));
	OSQPInt* A_p = (int*)malloc(8 * sizeof(int));
	OSQPInt A_nnz = 49;
	// convert the A matrix to CSC format
	convertToCSC(A, 13, 7, A_x, A_i, A_p);
	// print the CSC format matrix
	printf("CSC format matrix:\n");
	printCSC(13, 7, A_x, A_i, A_p);

	OSQPCscMatrix* Ax = static_cast<OSQPCscMatrix*>(malloc(sizeof(OSQPCscMatrix)));
	csc_set_data(Ax, m, n, A_nnz, A_x, A_i, A_p);
	print_csc_matrix(Ax);

	/* Exitflag */
	OSQPInt exitflag = 0;
	/* Solver, settings, matrices */
	OSQPSolver* solver;
	OSQPSettings* settings;
	/* Set default settings */
	settings = (OSQPSettings*)malloc(sizeof(OSQPSettings));
	if (settings) {
		osqp_set_default_settings(settings);
		settings->alpha = 1.0; /* Change alpha parameter */
		//settings->verbose = False;
	}

	/* Setup solver */
	exitflag = osqp_setup(&solver, P, q_, Ax, l, u, m, n, settings);

	/* Solve problem */
	if (!exitflag) exitflag = osqp_solve(solver);

	OSQPSolution* sol = solver->solution;
	printf("Solution:\n");
	for (int i = 0; i < 7; i++) {
		printf("x%d: %f \n", i, sol->x[i]);
	}

	/* Below are codes for motors*/

	// Initialize PortHandler Struct
	// Set the port path
	// Get method and members of PortHandlerLinux or PortHandlerWindows
	int port_num = portHandler(DEVICENAME);

	// Initialize PacketHandler Structs
	packetHandler();

	// Initialize Groupsyncwrite Structs
	int groupwrite_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION);

	// Initialize Groupsyncread Structs for Present Position
	int groupread_num = groupFastSyncRead(port_num, PROTOCOL_VERSION, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

	int index = 0;
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	uint8_t dxl_addparam_result = False;              // AddParam result
	uint8_t dxl_getdata_result = False;               // GetParam result
	int dxl_goal_position[2] = { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };         // Goal position

	uint8_t dxl_error = 0;                          // Dynamixel error
	int32_t dxl1_present_position = 0, dxl2_present_position = 0;                           // Present position

	//Open port
	if (openPort(port_num))
	{
		printf("Succeeded to open the port!\n");
	}
	else
	{
		printf("Failed to open the port!\n");
		printf("Press any key to terminate...\n");
		_getch();
		return 0;
	}

	// Set port baudrate
	if (setBaudRate(port_num, BAUDRATE))
	{
		printf("Succeeded to change the baudrate!\n");
	}
	else
	{
		printf("Failed to change the baudrate!\n");
		printf("Press any key to terminate...\n");
		_getch();
		return 0;
	}

	//Enable Dynamixel 1 Torque
	write1ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
	if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
	{
		printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
	}
	else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
	{
		printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
	}
	else
	{
		printf("Dynamixel has been successfully connected \n");
	}

	////Enable Dynamixel 2 Torque
	//write1ByteTxRx(port_num, PROTOCOL_VERSION, 2, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
	//if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
	//{
	//	printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
	//}
	//else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
	//{
	//	printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
	//}
	//else
	//{
	//	printf("Dynamixel has been successfully connected \n");
	//}

	// Add parameter storage for Dynamixel#1 present position value
	dxl_addparam_result = groupFastSyncReadAddParam(groupread_num, 6);
	if (dxl_addparam_result != True)
	{
		fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", 6);
		return 0;
	}

	//// Add parameter storage for Dynamixel#2 present position value
	//dxl_addparam_result = groupFastSyncReadAddParam(groupread_num, 2);
	//if (dxl_addparam_result != True)
	//{
	//	fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", 2);
	//	return 0;
	//}


	int i;
	int loops;
	loops = 600;
	clock_t start, end, rlstart, rlend;
	double elapsed, delay, freq, elap, actual_frequency, ec;

	start = clock();
	rlstart = clock();// get the start time

	double desired_value[6];

	FILE* datafile = createFile(nMotor);
	

	double elapsed_time = 0.0; // Variable to record running time

	while (1) {
		printf("Press any key to continue! (or press ESC to quit!)\n");
		if(cgetch()== ESC_ASCII_VALUE)
			break;
		printf("Enter a desired array with 6 numbers:\n");
		for (int i = 0; i < 6; ++i) {
			scanf_s("%lf", &desired_value[i]);
		}
		do {
			//// SyncWrite 
			// Add Dynamixel#6 goal position value to the Syncwrite storage
			dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, 6, dxl_goal_position[0], LEN_PRO_GOAL_POSITION);
			if (dxl_addparam_result != True)
			{
				fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", 1);
				return 0;
			}
			//// Add Dynamixel#2 goal position value to the Syncwrite parameter storage
			//dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, 2, dxl_goal_position[0], LEN_PRO_GOAL_POSITION);
			//if (dxl_addparam_result != True)
			//{
			//	fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", 2);
			//	return 0;
			//}
			// Syncwrite goal position
			groupSyncWriteTxPacket(groupwrite_num);
			if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
				printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
			// Clear syncwrite parameter storage
			groupSyncWriteClearParam(groupwrite_num);

			// Fast SyncRead
			// Syncread present position
			groupFastSyncReadTxRxPacket(groupread_num);
			if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
				printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));

			// Check if groupsyncread data of Dynamixel#1 is available
			dxl_getdata_result = groupFastSyncReadIsAvailable(groupread_num, 1, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
			if (dxl_getdata_result != True)
			{
				fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", 1);
				return 0;
			}

			// Check if groupsyncread data of Dynamixel#2 is available
			dxl_getdata_result = groupFastSyncReadIsAvailable(groupread_num, 2, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
			if (dxl_getdata_result != True)
			{
				fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", 2);
				return 0;
			}

			// Get Dynamixel#1 present position value
			dxl1_present_position = groupFastSyncReadGetData(groupread_num, 1, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);

			// Get Dynamixel#2 present position value
			dxl2_present_position = groupFastSyncReadGetData(groupread_num, 2, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
			printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n", 1, dxl_goal_position[index], dxl1_present_position, 2, dxl_goal_position[index], dxl2_present_position);


			Calculate_CableLength(Present_position, Cable_initial_length, cableLength, nMotor);
			Brace_Anchor_distance(Brace_anchor_points, brace_dis);
			convert_to_vectors(Cable_exit_points, Cable_exit_v, nMotor);
			//struct sphere s[13];
			//int s_size = sizeof(s) / sizeof(s[0]);
			//tree t;
			t.root = NULL;
			possible_position_tree(&t, Cable_exit_v, s, s_size, cableLength, brace_dis);
			output_pairs_array(vga, t, -260, 260);
			calculate_position_orientation(vga, posi_orient);
			//
			posi_orient->orientation[0] = 0;
			posi_orient->orientation[1] = 0;
			posi_orient->orientation[2] = 0;
			posi_orient->position.x = 0;
			posi_orient->position.y = -100;
			posi_orient->position.z = -50;

			//double** Rs = createMatrix(3, 3);
			//double** Bc = createMatrix(3, 3);
			//double** RsE = createMatrix(3, 3);
			//for (int i = 0; i < 3; i++)
			//{
			//	memcpy(Bc[i], Brace_anchor_points[i], 3 * sizeof(double));
			//}
			//vector** b;
			//b = malloc(sizeof(vector*) * 7);
			//if (b == NULL) { printf("b Memory allocation failed\n"); return 1; }
			//vector** q;
			//q = malloc(sizeof(vector*) * 7);
			//if (q == NULL) { printf("q Memory allocation failed\n"); return 1; }
			calculate_RsE_b_q(Rs, Bc, RsE, posi_orient, Cable_exit_v, &posi_orient->position, b, q);
			double** J = createMatrix(6, 7);
			calculate_jacobian(J, b, q);


			//QP test
			/* Load problem data */
			OSQPInt n = 7;
			OSQPInt m = 13;
			OSQPFloat P_x[7] = { 2,2,2,2,2,2,2 };
			OSQPInt P_nnz = 7;
			OSQPInt P_i[7] = { 0,1,2,3,4,5,6, };
			OSQPInt P_p[8] = { 0,1,2,3,4,5,6,7, };
			//OSQPCscMatrix* P = malloc(sizeof(OSQPCscMatrix));
			csc_set_data(P, n, n, P_nnz, P_x, P_i, P_p);
			//print_csc_matrix(P);
			//printf("CSC format matrix:\n");
			//printCSC(n, n, P_x, P_i, P_p);

			OSQPFloat q_[7] = { 0,0,0,0,0,0,0, };
			OSQPFloat l[13] = { -5,-5,-5,-5,-5,5.5,
								.5,.5,.5,.5,.5,.5,.5, };
			OSQPFloat u[13] = { 5,5,5,5,5,6.5,
								120,120,120,120,120,120,120 };

			// stack a 7x7 identical matrix under the jacobian matrix and make a new 13x7 A matrix
			double** A = stackMatrix(J, 6, 7);
			// print the A matrix
			//printf("A matrix:\n");
			//print_matrix(A, 13, 7);
			// allocate memory for the CSC format arrays
			//OSQPFloat* A_x = (double*)malloc(13 * 7 * sizeof(double));
			//OSQPInt* A_i = (int*)malloc(13 * 7 * sizeof(int));
			//OSQPInt* A_p = (int*)malloc(8 * sizeof(int));
			OSQPInt A_nnz = 49;
			// convert the A matrix to CSC format
			convertToCSC(A, 13, 7, A_x, A_i, A_p);
			// print the CSC format matrix
			//printf("CSC format matrix:\n");
			//printCSC(13, 7, A_x, A_i, A_p);

			//OSQPCscMatrix* Ax = malloc(sizeof(OSQPCscMatrix));
			csc_set_data(Ax, m, n, A_nnz, A_x, A_i, A_p);
			//print_csc_matrix(Ax);

			/* Exitflag */
			OSQPInt exitflag = 0;
			/* Solver, settings, matrices */
			OSQPSolver* solver;
			OSQPSettings* settings;
			/* Set default settings */
			settings = (OSQPSettings*)malloc(sizeof(OSQPSettings));
			if (settings) {
				osqp_set_default_settings(settings);
				settings->alpha = 1.0; /* Change alpha parameter */
				settings->verbose = False;
			}

			/* Setup solver */
			exitflag = osqp_setup(&solver, P, q_, Ax, l, u, m, n, settings);

			/* Solve problem */
			if (!exitflag) exitflag = osqp_solve(solver);

			OSQPSolution* sol = solver->solution;
			printf("Solution:\n");
			for (int i = 0; i < 7; i++) {
				printf("x%d: %f \n", i, sol->x[i]);
			}

			/* Clean solver */
			osqp_cleanup(solver);
			/* Clean settings */
			if (settings) { free(settings); }
			/* Free matrices */
			//if (P) { free(P); }
			//if (Ax) { free(Ax); }
			//if (A_x) { free(A_x); }
			//if (A_i) { free(A_i); }
			//if (A_p) { free(A_p); }



			//END 
			end = clock(); // get the end time
			elapsed = (double)(end - start) / CLOCKS_PER_SEC; // calculate the elapsed time in seconds
			actual_frequency = 1 / elapsed; // calculate the actual frequency in Hz
			printf("Actual frequency: %f Hz\n", actual_frequency);
			delay = PERIOD - elapsed; // calculate the required delay in seconds
			if (delay > 0) // if there is time left
			{
				clock_t target = end + delay * CLOCKS_PER_SEC;
				while (clock() < target); // do nothing until the target time is reached
			}
			ec = (double)(clock() - start) / CLOCKS_PER_SEC;
			actual_frequency = 1 / ec; // calculate the actual frequency in Hz
			elapsed_time += ec; // update the program running time
			printf("Modified Actual frequency: %f Hz\n", actual_frequency);

			// Save the datas into csv files
			fprintf(dataFile, "%f\n", elapsed_time);
			
			printf("Press any key to continue...\n");
			//if (_getch() == ESC_ASCII_VALUE)
			//	break;
			start = clock(); // update the start time for the next iteration

			if (ckbhit()) {
				if (cgetch() == ESC_ASCII_VALUE)
				{
					break;
				}
			}
			for (int i = 0; i < 6; ++i) {
				printf("%f ", desired_value[i]);
			}
			printf("\n");

		}while(index == 0);
		
		
		


	}
	rlend = clock();
	elap = (double)(rlend - rlstart) / CLOCKS_PER_SEC; // calculate the elapsed time in seconds
	freq = loops / elap; // calculate the loop sampling frequency in Hz

	printf("Elapsed time: %f seconds\n", elap);
	printf("Loop sampling frequency: %f Hz\n", freq);

	fclose(dataFile); //close the data file

	return (int)exitflag;

}





