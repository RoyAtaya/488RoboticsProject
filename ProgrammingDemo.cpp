// ProgrammingDemo.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdio.h>
#include <conio.h>
#include "ensc-488.h"
#include <iostream>
#include <string>
#include <cmath>
using namespace std;

//robot constants for lengths
const double a1 = 0.195;
const double a2 = 0.142;
const double d1 = 0.405;
const double d2 = 0.07;
const double d4 = -0.140;

double* where(double theta1, double theta2, double d3, double theta4);	//Where function used to find where the robot will end up with joint parameters
double* kin(double theta1, double theta2, double d3, double theta4);	//Solves the Transform Matrices from joint parameters
double* solve(double x, double y, double z, double phi);				//Solve function used to find joint parameters of the end effector location
double* invkin(double x, double y, double z, double phi);				//Solves the Inverse Kinematics using the robots parameters 

struct T {
	double result[4][4] = {
		{0, 0, 0, 0},
		{0, 0, 0, 0},
		{0, 0, 0, 0},
		{0, 0, 0, 0}
	};
};

T matrixMul(double T1[4][4], double T2[4][4]);

double valueRounding(double value);

int main(int argc, char* argv[]) {
	double theta1 = 0;	//First Joint Parameter
	double theta2 = 0;	//Second Joint Parameter
	double d3 = 0;		//Third Joint Parameter
	double theta4 = 0;	//Fourth Joint Parameter

	double x;
	double y;
	double z;
	double phi;

	/*JOINT q1;
	JOINT q2;*/
	printf("Keep this window in focus, and...\n");

	char ch;
	int c;

	double i = 1.0;
	bool validConfig;
	double* q;

	const int ESC = 27;

	printf("Press any key to continue \n");
	printf("Press ESC to exit \n");

	c = _getch();

	while (1) {

		if (c != ESC) {
			cout << "Press 1 to initiate WHERE function. Press 2 to initiate SOLVE function. Press 3 to Grasp.\n";
			ch = _getch();

			if (ch == '1') { //Forward Kinematics (WHERE/KIN)
				cout << "Where\nPlease input the values for each Joint (Theta 1 (deg), Theta 2 (deg), Distance 3 (m), Theta 4 (deg))\nTheta 1: ";
				cin >> theta1; //Enter in the First Joint Angle
				cout << "Theta 2: ";
				cin >> theta2; //Enter in the Second Joint Angle
				cout << "Distance 3: ";
				cin >> d3; //Enter in the Third Joint Translation
				cout << "Theta 4: ";
				cin >> theta4;//Enter in the Fourth Joint Angle

				if (theta1 < -150.0 || theta1 > 150.0 || theta2 < -100.0 || theta2 > 100.0 || d3 < -200.0 || d3 > -100.0 || theta4 < -160.0 || theta4 > 160.0) {
					printf("The joint parameters entered are outside of the robots range of motion");
				}
				else {
					//Use where to get return the x,y,z,phi values
					q = where(theta1, theta2, d3, theta4);
					printf("Joint vector 1: [%lf, %lf, %lf, %lf]\n", theta1, theta2, d3, theta4); //prints the joints
					printf("The End Effector Values are: {%lf, %lf, %lf, %lf]\n", q[0], q[1], q[2], q[3]); //prints the x,y,z,phi
					JOINT qFinal = { theta1, theta2, d3, theta4};
					MoveToConfiguration(qFinal, true);
				}
			}
			else if (ch == '2') {
				cout << "SOLVE\nPlease input the values for x (m), y (m), z (m), and phi (deg)\nx : ";
				cin >> x;
				cout << "y : ";
				cin >> y;
				cout << "z : ";
				cin >> z;
				cout << "phi : ";
				cin >> phi;
				phi = phi * PI / 180;
				q = solve(x, y, z, phi);
				if (q[4] == -1.0) {
					printf("The coordinates entered are outside the joint space of the robot.\n\n");
				}
				else {
					JOINT qFinal = { q[0], q[1], q[2], q[3] };
					MoveToConfiguration(qFinal, true);
				}
			}
			else if (ch == '3') {
				printf("Please input 1 to close the gripper, 2 to open the gripper.\n");
				ch = _getch();
				if (ch == '1') {
					bool closed = Grasp(true);
					printf("Closed = %d\n", closed);
				}
				else if (ch == '2') {
					bool opened = Grasp(false);
					printf("Opened = %d\n", opened);
				}
			}
			
			printf("Press any key to continue \n\n");
			printf("Press ESC to exit \n\n");
			c = _getch();

		}
		else break;
	}
	return 0;
}

double* where(double theta1, double theta2, double d3, double theta4) {
	double* jointVectors;
	jointVectors = kin(theta1, theta2, d3, theta4);

	JOINT curr;
	GetConfiguration(curr);
	return jointVectors;
}

double* kin(double theta1, double theta2, double d3, double theta4) {
	static double jointVectors[4] = {};
	theta1 = DEG2RAD(theta1); 
	theta2 = DEG2RAD(theta2);
	theta4 = DEG2RAD(theta4);

	double T01[4][4] = { //Translation Matrix
		{valueRounding(cos(theta1)), valueRounding(-sin(theta1)), 0, 0},
		{valueRounding(sin(theta1)), valueRounding(cos(theta1)), 0, 0},
		{0, 0, 1, d1},
		{0, 0, 0, 1}
	};
	double T12[4][4] = { //Translation Matrix
		{valueRounding(cos(theta2)), valueRounding(-sin(theta2)), 0, a1},
		{valueRounding(sin(theta2)), valueRounding(cos(theta2)), 0, 0},
		{0, 0, 1, d2},
		{0, 0, 0, 1}
	};
	double T23[4][4] = { //Translation Matrix
		{1,  0,  0,  a2},
		{0, -1,  0,  0},
		{0,  0, -1, -d2},
		{0,  0,  0,  1}
	};

	double T34[4][4] = { //Translation Matrix
		{valueRounding(cos(theta4)), valueRounding(-sin(theta4)), 0, 0}, 
		{valueRounding(sin(theta4)), valueRounding(cos(theta4)), 0, 0}, 
		{0, 0, 1, d3},
		{0, 0, 0, 1}
	};

	double T45[4][4] = {//Translation Matrix
		{1, 0, 0, 0},
		{0, 1, 0, 0},
		{0, 0, 1, 0},
		{0, 0, 0, 1}
	};

	//T05 = T01 * T12 * T23 * T34 * T45;
	T T1 = matrixMul(T01, T12);
	T T2 = matrixMul(T1.result, T23);
	T T3 = matrixMul(T2.result, T34);
	T T4 = matrixMul(T3.result, T45);

	// phi calculation 
	double phi = theta1 + theta2 + theta4;
	phi = RAD2DEG(phi);

	jointVectors[0] = T4.result[0][3];
	jointVectors[1] = T4.result[1][3];
	jointVectors[2] = T4.result[2][3];
	jointVectors[3] = phi;
	return jointVectors;
}

double valueRounding(double value) {
	return round(value * 1000.0) / 1000.0;
}

T matrixMul(double T1[4][4], double T2[4][4]) {
	T result;

	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			for (int k = 0; k < 4; ++k) {
				result.result[i][j] += T1[i][k] * T2[k][j];
			}
	return result;
}

double* solve(double x, double y, double z, double phi) {
	bool q1Valid = true;
	bool q2Valid = true;
	double q[5] = {};
	q[4] = 0.0;
	double* jointVectors;
	jointVectors = invkin(x, y, z, phi);
	if (jointVectors[8] == -1.0) {
		q[4] = -1.0;
		return q;
	}
	else {
		double theta1_1 = jointVectors[0];
		double theta2_1 = jointVectors[1];
		double d3 = jointVectors[2];
		double theta4_1 = jointVectors[3];
		double theta1_2 = jointVectors[4];
		double theta2_2 = jointVectors[5];
		double theta4_2 = jointVectors[7];

		if (theta1_1 < -150.0 || theta1_1 > 150.0 ||
			theta2_1 < -100.0 || theta2_1 > 100.0 ||
			d3 < -200.0 || d3 > -100.0 ||
			theta4_1 < -160.0 || theta4_1 > 160.0) {
			printf("q1 is invalid\n");
			q1Valid = false;
		}
		else if (theta1_2 < -150.0 || theta1_2 > 150.0 ||
			theta2_2 < -100.0 || theta2_2 > 100.0 ||
			d3 < -200.0 || d3 > -100.0 ||
			theta4_2 < -160.0 || theta4_2 > 160.0) {
			printf("q2 is invalid\n");
			q2Valid = false;
		}

		if (q1Valid && q2Valid) {
			JOINT curr;
			GetConfiguration(curr);

			double dif1_1 = abs(curr[0] - jointVectors[0]);
			double dif2_1 = abs(curr[1] - jointVectors[1]);
			double dif3_1 = abs(curr[3] - jointVectors[3]);

			double dif1_2 = abs(curr[0] - jointVectors[4]);
			double dif2_2 = abs(curr[1] - jointVectors[5]);
			double dif3_2 = abs(curr[3] - jointVectors[7]);

			double difTot1 = dif1_1 + dif2_1 + dif3_1;
			double difTot2 = dif1_2 + dif2_2 + dif3_2;

			if (difTot1 > difTot2) {
				q[0] = jointVectors[4];
				q[1] = jointVectors[5];
				q[2] = jointVectors[6];
				q[3] = jointVectors[7];
			}
			else {
				q[0] = jointVectors[0];
				q[1] = jointVectors[1];
				q[2] = jointVectors[2];
				q[3] = jointVectors[3];
			}
		}
		else if (!q1Valid && !q2Valid) {
			q[4] = -1.0;
		}
		else if (!q1Valid) {
			q[0] = jointVectors[4];
			q[1] = jointVectors[5];
			q[2] = jointVectors[6];
			q[3] = jointVectors[7];
		}
		else {
			q[0] = jointVectors[0];
			q[1] = jointVectors[1];
			q[2] = jointVectors[2];
			q[3] = jointVectors[3];
		}

		return q;
	}
}

double* invkin(double x, double y, double z, double phi) {
	static double jointVectors[9] = {};
	jointVectors[8] = 0.0;
	double c2 = (pow(x, 2) + pow(y, 2) - pow(a1, 2) - pow(a2, 2)) / (2 * a1 * a2);
	double s2 = sqrt(1 - pow(c2, 2));
	if (c2 > 1 || c2 < -1 || isnan(c2) || s2 > 1 || s2 < -1 || isnan(s2)) {
		jointVectors[8] = -1.0;
		return jointVectors;
	}
	else {
		double theta2_1 = atan2(s2, c2);
		double theta2_2 = atan2(-s2, c2);
		double theta1_1 = atan2(y, x) - atan2(a2 * s2, a1 + a2 * c2);
		double theta1_2 = atan2(y, x) - atan2(a2 * -s2, a1 + a2 * c2);
		double theta4_1 = phi - theta1_1 - theta2_1;
		double theta4_2 = phi - theta1_2 - theta2_2;
		double d3 = z - d1 - d2 - d4;

		printf("Joint vector 1: [%lf, %lf, %lf, %lf]\n",
			theta1_1 * 180 / PI, theta2_1 * 180 / PI, d3, theta4_1 * 180 / PI);

		printf("Joint vector 2: [%lf, %lf, %fl, %lf]\n",
			theta1_2 * 180 / PI, theta2_2 * 180 / PI, d3, theta4_2 * 180 / PI);

		jointVectors[0] = theta1_1 * 180 / PI;
		jointVectors[1] = theta2_1 * 180 / PI;
		jointVectors[2] = d3 * 1000;
		jointVectors[3] = theta4_1 * 180 / PI;
		jointVectors[4] = theta1_2 * 180 / PI;
		jointVectors[5] = theta2_2 * 180 / PI;
		jointVectors[6] = d3 * 1000;
		jointVectors[7] = theta4_2 * 180 / PI;
		return jointVectors;
	}

}
