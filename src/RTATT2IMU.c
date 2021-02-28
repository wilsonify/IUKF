// PROGRAM RTATT2IMU.c -  Implements the discrete Kalman Filter algorithm
// for estimation of the attitude of a 3-Space Sensor LX IMU, from Yost Labs(c),
// using the quaternion orientation as state vector and a measurement attitude
// quaternion derived from accelerometer measurements, under 2 assumptions:
// - Rotations are performed in only 2 axes
// - The matrices Q, H, R and the initial PA, are all "scaled Identity" matrices
// The program runs only for a user-defined number of seconds (TIMERUN).
// Implementation parameters (INCLUDING COMM PORT TO BE USED) are read at run
// time from the file IVALS.txt expected in the same directory as the executable.
// The management of the IMU data streaming is based on the file streaming_example.c
// provided by Yost Labs (c)

#include "RTATT2IMU.h"
#include <conio.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// Declarations for CUSTOM FUNCTIONS to implement the KALMAN FILTER

vec4f M44V41(float M44[4][4], vec4f V41);

void M44PLUSM44(float M1[4][4], float M2[4][4], float MR[4][4]);
void M44MINUSM44(float M1[4][4], float M2[4][4], float MR[4][4]);
vec4f V41PLUSV41(vec4f V1, vec4f V2);
vec4f V41MINUSV41(vec4f V1, vec4f V2);

void MAKESCALEDI4(float ScaleFactor, float M1[4][4]);
void SI4SI4(float M1[4][4], float M2[4][4], float MR[4][4]);
void SI4SI4SI4(float M1[4][4], float M2[4][4], float M3[4][4], float MR[4][4]);
vec4f SI4V41(float SI[4][4], vec4f V);
void SI4INV(float M1[4][4], float MR[4][4]);

void FPFT(float F[4][4], float P[4][4], float MR[4][4]);

void PRNM44(float M[4][4]);
void PRNV41(vec4f V);

float ML[4][4], MN[4][4], MO[4][4], MS[4][4], MT[4][4];   // Scratch-Pad matrices for intermediate results
float a, b, c;
float Acmag, AcxN, AcyN, AczN;
float tht, psi, phi;
float th2, ps2, ph2;
float flttht, fltpsi, fltphi;
float r2d = 180 / (3.1416);

int main(void)
{
	HANDLE com_handle; 
	DWORD bytes_written; 
	DWORD bytes_read;  
	unsigned char write_stream_bytes[3]; 
	unsigned char write_tare_bytes[3];
	
	Batch_Data batch_data = { 0 }; 
	unsigned int sample_count; // the amount of packets received
	char commNum;
	char decisionChar; 
	
	TCHAR commStr[] = TEXT("\\\\.\\COM8");  // The comm port # to use is overwritten from IVALS.txt

	float CurrentTime;
	FILE* fp; 
	FILE* fini;

	vec3f Gy, Ac, Compass;
	vec4f IMUquat;

	// THESE PARAMETERS ARE ACTUALLY RED-IN FROM IVALS.txt - These are "example" values
	int TIMERUN = 10;   // Duration of the streaming process (e.g.,  10 seconds)
	float PAfact = 0.001;  // Values for the main diagonal of INITIAL PA (e.g., 0.001)
	float Qfact = 0.001;   // Values for the main diagonal of Q (e.g., 0.001)
	float Hfact = 1.0;     // Values for the main diagonal of H (must be 1.0)
	float Rfact = 0.01;    // Values for the main diagonal of R (e.g., 0.01)
	float dt = 0.06;       // Values for the "Delta T" factor in equation 11.16 (e.g., 0.06)
	float dt2;

	// READING INITIAL VALUES FROM FILE IVALS.txt EXPECTED IN SAME DIRECTORY AS EXECUTABLE
	fini = fopen("IVALS.txt", "r");   // Reads Initial values and parms. from this file, same directory
	rewind(fini);
	fscanf(fini, "%c %f %f %f %f %f %d", &commNum, &dt, &Hfact, &PAfact, &Qfact, &Rfact, &TIMERUN);
	printf("RUN PARAMETERS    : COMM, dt, Hfact, PAfact, Qfact, Rfact, TIMERUN \n");
	printf("READ FROM IVALS.txt: %c %f %f %f %f %f %d \n", commNum, dt, Hfact, PAfact, Qfact, Rfact, TIMERUN);
	fclose(fini);

	vec4f xA = { 0 }; // state vector into KF (quaternion)
	vec4f xB = { 0 };
	vec4f z = { 0 };
	float F[4][4] = { 0 };
	float PA[4][4] = { 0 };
	float PB[4][4] = { 0 };
	float Q[4][4] = { 0 };
	float R[4][4] = { 0 };
	float H[4][4] = { 0 };
	float KG[4][4] = { 0 };

	
	float Stillness;
	
	int i;
	
	sample_count = 0;

	float STREAM_DURATION = TIMERUN; //Unit SEC ++++++++++++  TIME ASSIGNMENT

	// High-resolution performance counter varibles. This acts as a timer
	LARGE_INTEGER frequency; // counts per second
	LARGE_INTEGER start_time, current_time; // counts
	QueryPerformanceFrequency(&frequency); // gets performance-counter frequency, in counts per second

	printf("ENTER [r] to accept prameters and run or [q] to quit and change IVALS.txt \n");
	decisionChar=getchar();
	printf("\nCharacter entered: ");
	putchar(decisionChar);
	printf("  \n");
	if (decisionChar == 'q')
	{
		printf("Execution will be terminated. \n");
		printf("Modify the file IVALS.txt to set different parameters, \n");
		printf("including COMM PORT NUMBER TO BE USED-> 1st entry in IVALS.txt \n \n");
		exit(EXIT_FAILURE);
	}
	else
	{
		printf("Execution will continue, using COMM PORT %c \n", commNum);
	}
    
	commStr[7] = commNum;

		// OPEN FILE "RTATT2IMUOUT.txt" - RESULTS WILL BE WRITTEN TO THIS FILE AS C S V
	fp = fopen("RTATT2IMUOUT.txt", "w"); // "w"=re-write, "a"=addline
	
	
	// ++++++++++++++++++++++++++++++++++

	// INITIALIZATION OF KF MATRICES
	MAKESCALEDI4(PAfact, PA);    // Scaling PA
	MAKESCALEDI4(Qfact, Q);       // Scaling Q
	MAKESCALEDI4(Hfact, H);       // Scaling H
	MAKESCALEDI4(Rfact, R);       // Scaling R
	

	xA.x = 1;    // Initialize xA to [1, 0, 0, 0]T
	xA.y = 0;
	xA.z = 0;
	xA.w = 0;

		com_handle = openAndSetupComPort(commStr);
		if (com_handle == INVALID_HANDLE_VALUE) {
			printf("comm port open failed\n");
		}

		if (setupStreaming(com_handle)) {
			printf("Streaming set up failed\n");
		}

		printf("\n \n Press <Enter> to tare the sensor and start streaming. . .\n");
		getchar(); getchar();
	
		write_tare_bytes[0] = TSS_START_BYTE;
		write_tare_bytes[1] = TSS_TARE_CURRENT_ORIENTATION;
		write_tare_bytes[2] = createChecksum(&write_tare_bytes[1], 1);

		if (!WriteFile(com_handle, write_tare_bytes, sizeof(write_tare_bytes), &bytes_written, 0)) {
			printf("Error to tare the sensor\n");
		}

		printf("\n STREAMING STARTED - IT WILL CONTINUE FOR %d SECONDS\n", TIMERUN);

		// With parameterless wired commands the command byte will be the same as the checksum
		write_stream_bytes[0] = TSS_START_BYTE;
		write_stream_bytes[1] = TSS_START_STREAMING;
		write_stream_bytes[2] = TSS_START_STREAMING;

		// Write the bytes to the serial
		if (!WriteFile(com_handle, write_stream_bytes, sizeof(write_stream_bytes), &bytes_written, 0)) {
			printf("Error writing to port\n");
			//return 3;
		}

		QueryPerformanceCounter(&start_time); // Retrieves the current value of the high-resolution performance counter.
		QueryPerformanceCounter(&current_time); // Retrieves the current value of the high-resolution performance counter.

		// while loop runs for as many seconds as assigned in STREAM_DURATION = TIMERUN
		while ((float)STREAM_DURATION > ((current_time.QuadPart - start_time.QuadPart) * 1.0f / frequency.QuadPart)) 
		{

			QueryPerformanceCounter(&current_time); // Retrieves the current value of the high-resolution performance counter.
			// Read the bytes returned from the serial
			if (!ReadFile(com_handle, &batch_data, sizeof(batch_data), &bytes_read, 0)) {
				printf("Error reading from port\n");
			}

			if (bytes_read != sizeof(batch_data)) {
				continue;
			}

			// The data must be fliped to little endian to be read correctly
			for (i = 0; i < sizeof(batch_data.Data) / sizeof(float); i++) {
				endian_swap_32((unsigned int*)&batch_data.Data[i]);
			}

			// Calculate current time
			CurrentTime = (current_time.QuadPart - start_time.QuadPart) * 1.0f / frequency.QuadPart;

			// Parsing Sensor Data
			Stillness = batch_data.Data[0];
			Gy.x = batch_data.Data[1];
			Gy.y = batch_data.Data[2];
			Gy.z = batch_data.Data[3];
			Ac.x = batch_data.Data[4];
			Ac.y = batch_data.Data[5];
			Ac.z = batch_data.Data[6];
			IMUquat.x = batch_data.Data[7];
			IMUquat.y = batch_data.Data[8];
			IMUquat.z = batch_data.Data[9];
			IMUquat.w = batch_data.Data[10];
			Compass.x = batch_data.Data[11];
			Compass.y = batch_data.Data[12];
			Compass.z = batch_data.Data[13];
			//Confidence = batch_data.Data[14];

		    // *** *********  START -Read Gyro, accel to KF variables
			a = (-1) * Gy.z;
			b = (-1) * Gy.x;
			c = Gy.y;

			dt2 = dt / 2;

			//  Forming Matrix F, as F = I4 + (dt/2) * OMEGA_MATRIX
			F[0][0] = 1;
			F[0][1] = (-1) * dt2 * a;
			F[0][2] = (-1) * dt2 * b;
			F[0][3] = (-1) * dt2 * c;
			F[1][0] = (dt2)*a;
			F[1][1] = 1;
			F[1][2] = (dt2)*c;
			F[1][3] = (-1) * dt2 * b;
			F[2][0] = (dt2)*b;
			F[2][1] = (-1) * dt2 * c;
			F[2][2] = 1;
			F[2][3] = (dt2)*a;
			F[3][0] = (dt2)*c;
			F[3][1] = (dt2)*b;
			F[3][2] = (-1) * dt2 * a;
			F[3][3] = 1;

			Acmag = sqrt((Ac.x * Ac.x) + (Ac.y * Ac.y) + (Ac.z * Ac.z));
			AcxN = Ac.z / Acmag;
			AcyN = Ac.x / Acmag;
			//	AczN = Ac.y / Acmag;

			tht = asin(AcxN);  // Calculate theta
			phi = asin(((-1) * AcyN) / (cos(tht)));
			psi = 0;

			th2 = tht / 2;
			ph2 = phi / 2;
			ps2 = psi / 2;

			// LODING OF ACCELERATIONS TO Z VECTOR
			z.x = cos(ph2) * cos(th2) * cos(ps2) + sin(ph2) * sin(th2) * sin(ps2);
			z.y = sin(ph2) * cos(th2) * cos(ps2) - cos(ph2) * sin(th2) * sin(ps2);
			z.z = cos(ph2) * sin(th2) * cos(ps2) + sin(ph2) * cos(th2) * sin(ps2);
			z.w = cos(ph2) * cos(th2) * sin(ps2) - sin(ph2) * sin(th2) * cos(ps2);

			// *** *********  END -Read Gyro, accel to KF vars

			//  KF PROCCESSING   ######## (Uses generic 4x4 matrices ML, MO, MP, MT for TEMPORARY storage)
			
			// ----- PREDICTION EQUATIONS
			xB = M44V41(F, xA);              // EQ 12.1: xB = F * XA (There is no u vector in this application)
			
			FPFT(F, PA, ML);                 // Term F * P * FT of EQ. 12.2 is calculated and stored in ML
			M44PLUSM44(ML, Q, PB);           // Completing EQ. 12.2. Result stored in PB
			
											 // ----- CORRECTION EQUATIONS
			SI4SI4SI4(H, PB, H, MN);         // PROD HPBHT in MN - Can use H instead of HT b/c H is symmetric
			M44PLUSM44(MN, R, MO);           // (HPBHT + R) computed and held in MO
			SI4INV(MO, MS);                  // INVERSE OF(HPBHT + R) computed and held in MS
			SI4SI4SI4(PB, H, MS, KG);        // Completing EQ. 12.3 - Can use H instead of HT becuase its symm.
			
			xA = V41PLUSV41(xB, (M44V41(KG, (V41MINUSV41(z, (M44V41(H, xB)))))));   // EQ. 12.4

			SI4SI4SI4(KG, H, PB, MT);        // The term (Kg * H * PB) is computed. Result in MT
			M44MINUSM44(PB, MT, PA);         // COMPLETING EQ. 12.5. RESULT IN PA
			//  END OF KF PROCCESSING   ######## 

			// CONVERSION TO THETA, PSI, PHI OF RESULTING QUATERNION
			fltphi = atan2((2 * (xA.z * xA.w + xA.x * xA.y)), (1 - 2 * (pow(xA.y, 2) + pow(xA.z, 2)))) * r2d;
			flttht = (-1) * (asin(2 * (xA.y * xA.w - xA.x * xA.z))) * r2d;
			fltpsi = atan2((2 * (xA.y * xA.z + xA.x * xA.w)), (1 - 2 * (pow(xA.z, 2) + pow(xA.w, 2)))) * r2d;
			
		fprintf(fp, "%f,%f,%f,%f,%f,%f,%f \n", CurrentTime, (phi * r2d), (tht * r2d), (psi * r2d), fltphi, flttht, fltpsi);

			sample_count++;
		} // End of while loop - Streaming went on for STREAM_DURATION = TIMERUN seconds

		CloseHandle(com_handle);
 
	fclose(fp);
	printf("\nSample Count      =     %u samples in %.4f seconds\n", sample_count, CurrentTime);
	printf("Effective Sampling Interval, dt =     %f seconds \n", ( CurrentTime / (float)sample_count));
	printf("\nPort is closed");

	// END of PROGRAM 
	printf("\nFinished! press <Enter> to terminate");
	getchar();

	return 0;
}

// ###################################################################################
// ADDITIONAL FUNCTIONS FOR MATRIX, VECTOR OPERATIONS IN KALMAN FILTER ###############
// ###################################################################################

//FUNCTION M44V41
vec4f M44V41(float M[4][4], vec4f V)   //MULTPLIES a GRAL 4x4 MTX times a 4X1 VECTOR
{
	vec4f R;

	R.x = (M[0][0] * V.x) + (M[0][1] * V.y) + (M[0][2] * V.z) + (M[0][3] * V.w);
	R.y = (M[1][0] * V.x) + (M[1][1] * V.y) + (M[1][2] * V.z) + (M[1][3] * V.w);
	R.z = (M[2][0] * V.x) + (M[2][1] * V.y) + (M[2][2] * V.z) + (M[2][3] * V.w);
	R.w = (M[3][0] * V.x) + (M[3][1] * V.y) + (M[3][2] * V.z) + (M[3][3] * V.w);

	return R;
}

//FUNCTION M44PLUSM44
void M44PLUSM44(float M1[4][4], float M2[4][4], float MR[4][4])   //ADDS TWO 4x4 General  matrices
{
	int i, j;

	for (i = 0; i <= 3; i++)
	{
		for (j = 0; j <= 3; j++)
		{
			MR[i][j] = M1[i][j] + M2[i][j];
		}
	}
}

//FUNCTION M44MINUSM44
void M44MINUSM44(float M1[4][4], float M2[4][4], float MR[4][4])   //SUBSTRACTS M1 - M2, both 4 x 4
{
	int i, j;

	for (i = 0; i <= 3; i++)
	{
		for (j = 0; j <= 3; j++)
		{
			MR[i][j] = M1[i][j] - M2[i][j];
		}
	}
}

//FUNCTION V41PLUSV41
vec4f V41PLUSV41(vec4f V1, vec4f V2)   // ADDS V1 + V2 , BOTH 4x1 VECTORS
{
	vec4f R;

	R.x = V1.x + V2.x;
	R.y = V1.y + V2.y;
	R.z = V1.z + V2.z;
	R.w = V1.w + V2.w;

	return R;
}

//FUNCTION V41MINUSV41
vec4f V41MINUSV41(vec4f V1, vec4f V2)   // SUBTRACTS V1 - V2 , BOTH 4x1 VECTORS
{
	vec4f R;

	R.x = V1.x - V2.x;
	R.y = V1.y - V2.y;
	R.z = V1.z - V2.z;
	R.w = V1.w - V2.w;

	return R;
}

//FUNCTION MAKESCALEDI4
void MAKESCALEDI4(float ScaleFactor, float M1[4][4])   // CREATES SCALED I4 IN M1, APPLYING ScaleFactor
{
	int i, j;
	for (i = 0; i <= 3; i++)     // writing zeros on all entries
	{
		for (j = 0; j <= 3; j++)
		{
			M1[i][j] = 0;
		}
	}
	M1[0][0] = ScaleFactor;     // WRITING ScaleFactor ON THE 4 DIAGONAL ELEMENTS
	M1[1][1] = ScaleFactor;
	M1[2][2] = ScaleFactor;
	M1[3][3] = ScaleFactor;
}

//FUNCTION SI4SI4
void SI4SI4(float M1[4][4], float M2[4][4], float MR[4][4])   //MULTPLIES TWO 4x4 Scaled Id. matrices
{
	float ScalarRes = 0;
	
	int i, j;

	for (i = 0; i <= 3; i++)     // writing zeros on all entries
	{
		for (j = 0; j <= 3; j++)
		{
			MR[i][j] = 0;
		}
	}

	ScalarRes = M1[0][0] * M2[0][0];  // calculate scalar for diag. of result

	MR[0][0] = ScalarRes;      //write the scalar for result in the 4 diag entries of result
	MR[1][1] = ScalarRes;
	MR[2][2] = ScalarRes;
	MR[3][3] = ScalarRes;
}

//FUNCTION SI4SI4SI4
void SI4SI4SI4(float M1[4][4], float M2[4][4], float M3[4][4], float MR[4][4])   //MULTPLIES THREE 4x4 Scaled Id. matrices
{
	float ScalarRes = 0;
	
	int i, j;

	for (i = 0; i <= 3; i++)     // writing zeros on all entries
	{
		for (j = 0; j <= 3; j++)
		{
			MR[i][j] = 0;
		}
	}

	ScalarRes = M1[0][0] * M2[0][0] * M3[0][0];  // calculate scalar for diag. of result

	MR[0][0] = ScalarRes;      //write the scalar for result in the 4 diag entries of result
	MR[1][1] = ScalarRes;
	MR[2][2] = ScalarRes;
	MR[3][3] = ScalarRes;
}

//FUNCTION SI4V41
vec4f SI4V41(float SI[4][4], vec4f V)   //MULTPLIES a scaled I4 times a 4X1 VECTOR
{
	vec4f R;
	float ScaleF;

	ScaleF = SI[0][0];

	R.x = ScaleF * V.x;
	R.y = ScaleF * V.y;
	R.z = ScaleF * V.z;
	R.w = ScaleF * V.w;

	return R;
}

//FUNCTION SI4INV
void SI4INV(float M1[4][4], float MR[4][4])   //GETS INVERSE OF 4x4 Scaled Id. matrix
{
	
	int i, j;
	for (i = 0; i <= 3; i++)     // writing zeros on all entries
	{
		for (j = 0; j <= 3; j++)
		{
			MR[i][j] = 0;
		}
	}

	MR[0][0] = 1 / (M1[0][0]);      //write the inv for each of the 4 diag entries of result
	MR[1][1] = 1 / (M1[1][1]);
	MR[2][2] = 1 / (M1[2][2]);
	MR[3][3] = 1 / (M1[3][3]);
}

//FUNCTION FPFT
void FPFT(float F[4][4], float P[4][4], float MR[4][4])   //IMPLEMENTS F*P*FT ASSUMING P is Scaled ID
{
	int i, j;
	float ScalarRes, a2, b2, c2;

	a2 = (F[0][1] * F[0][1]);
	b2 = (F[0][2] * F[0][2]);
	c2 = (F[0][2] * F[0][2]);
	ScalarRes = P[0][0] * (a2 + b2 + c2 + 1);

	// initialize the result matrix with all 16 zeros
	for (i = 0; i <= 3; i++)     // writing zeros on all entries
	{
		for (j = 0; j <= 3; j++)
		{
			MR[i][j] = 0;
		}
	}

	MR[0][0] = ScalarRes;      //write the scalar for result in the 4 diag entries of result
	MR[1][1] = ScalarRes;
	MR[2][2] = ScalarRes;
	MR[3][3] = ScalarRes;
}

//FUNCTION PRNM44 - ONLY USEFUL FOR DEBUGGING
void PRNM44(float M[4][4])   //PRINTS (TO CONSOLE) a GENERAL 4 x 4 FLOAT MATRIX TO CONSOLE
{
	printf(" \n");
	printf("%f, %f, %f, %f\n", M[0][0], M[0][1], M[0][2], M[0][3]);
	printf("%f, %f, %f, %f\n", M[1][0], M[1][1], M[1][2], M[1][3]);
	printf("%f, %f, %f, %f\n", M[2][0], M[2][1], M[2][2], M[2][3]);
	printf("%f, %f, %f, %f\n", M[3][0], M[3][1], M[3][2], M[3][3]);

}

//FUNCTION PRNV41 - ONLY USEFUL FOR DEBUGGING
void PRNV41(vec4f V)   //PRINTS (TO CONSOLE) a GENERAL 4 x 1 FLOAT VECTOR TO CONSOLE
{
	printf(" \n");
	printf("%f \n", V.x);
	printf("%f \n", V.y);
	printf("%f \n", V.z);
	printf("%f \n", V.w);

}




