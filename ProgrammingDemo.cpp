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
const double d4 = 0.140;

bool where(double theta1, double theta2, double d3, double theta4, JOINT& conf);	//Where function used to find where the robot will end up with joint parameters
bool kin(double theta1, double theta2, double d3, double theta4, JOINT &conf);	//Solves the Transform Matrices from joint parameters
bool solve(double x, double y, double z, double phi, JOINT &conf1);				//Solve function used to find joint parameters of the end effector location
bool invkin(double x, double y, double z, double phi, JOINT& conf1, JOINT& conf2);				//Solves the Inverse Kinematics using the robots parameters 

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
	JOINT q;
	printf("Keep this window in focus, and...\n");

	char ch;
	int c;

	double i = 1.0;

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

				printf("Joint vector 1: [%lf, %lf, %lf, %lf]\n", theta1, theta2, d3, theta4);
				if (where(theta1, theta2, d3, theta4, q)) {
					printf("The End Effector Values are: {%lf, %lf, %lf, %lf]\n", q[0], q[1], q[2], q[3]);
					JOINT qWhere = { theta1, theta2, d3, theta4 };
					MoveToConfiguration(qWhere, true);
				}
				else {
					printf("One or more of the joint limits provided exceed joint limits.\n");
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
				if (!solve(x, y, z, phi, q)) {
					printf("The coordinates entered are outside the joint space of the robot.\n\n");
				}
				else {
					MoveToConfiguration(q, true);
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

bool where(double theta1, double theta2, double d3, double theta4, JOINT &conf) {
	if (!kin(theta1, theta2, d3, theta4, conf)) {
		return false;
	} 
	else {
		return true;
	}
}

bool kin(double theta1, double theta2, double d3, double theta4, JOINT &conf) {
	//static double jointVectors[4] = {};
	if (theta1 < -150.0 || theta1 > 150.0 || 
		theta2 < -100.0 || theta2 > 100.0 || 
		d3 < -200.0 || d3 > -100.0 || 
		theta4 < -160.0 || theta4 > 160.0) {
		return false;
	}
	theta1 = DEG2RAD(theta1); 
	theta2 = DEG2RAD(theta2);
	theta4 = DEG2RAD(theta4);
	d3 = d3 / 1000.0;

	conf[0] = a2 * cos(theta1 + theta2) + a1 * cos(theta1);
	conf[1] = a2 * sin(theta1 + theta2) + a1 * sin(theta1);
	conf[2] = (d1 + d2 - d3 - d4 - .410) * 1000.0;
	conf[3] = theta1 + theta2 - theta4;

	return true;

	//double T01[4][4] = { //Translation Matrix
	//	{valueRounding(cos(theta1)), valueRounding(-sin(theta1)), 0, 0},
	//	{valueRounding(sin(theta1)), valueRounding(cos(theta1)), 0, 0},
	//	{0, 0, 1, d1},
	//	{0, 0, 0, 1}
	//};
	//double T12[4][4] = { //Translation Matrix
	//	{valueRounding(cos(theta2)), valueRounding(-sin(theta2)), 0, a1},
	//	{valueRounding(sin(theta2)), valueRounding(cos(theta2)), 0, 0},
	//	{0, 0, 1, d2},
	//	{0, 0, 0, 1}
	//};
	//double T23[4][4] = { //Translation Matrix
	//	{1,  0,  0,  a2},
	//	{0, -1,  0,  0},
	//	{0,  0, -1, -d2},
	//	{0,  0,  0,  1}
	//};

	//double T34[4][4] = { //Translation Matrix
	//	{valueRounding(cos(theta4)), valueRounding(-sin(theta4)), 0, 0}, 
	//	{valueRounding(sin(theta4)), valueRounding(cos(theta4)), 0, 0}, 
	//	{0, 0, 1, d3},
	//	{0, 0, 0, 1}
	//};

	//double T45[4][4] = {//Translation Matrix
	//	{1, 0, 0, 0},
	//	{0, 1, 0, 0},
	//	{0, 0, 1, 0},
	//	{0, 0, 0, 1}
	//};

	////T05 = T01 * T12 * T23 * T34 * T45;
	//T T1 = matrixMul(T01, T12);
	//T T2 = matrixMul(T1.result, T23);
	//T T3 = matrixMul(T2.result, T34);
	//T T4 = matrixMul(T3.result, T45);

	//// phi calculation 
	//double phi = theta1 + theta2 + theta4;
	//phi = RAD2DEG(phi);

	//jointVectors[0] = T4.result[0][3];
	//jointVectors[1] = T4.result[1][3];
	//jointVectors[2] = T4.result[2][3];
	//jointVectors[3] = phi;
	//return jointVectors;
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

bool solve(double x, double y, double z, double phi, JOINT &conf) {
	bool q1Valid = true;
	bool q2Valid = true;
	JOINT q1;
	JOINT q2;
	if (!invkin(x, y, z, phi, q1, q2)) {
		return false;
	}
	else {
		double theta1_1 = q1[0];
		double theta2_1 = q1[1];
		double d3 = q1[2];
		double theta4_1 = q1[3];
		double theta1_2 = q2[0];
		double theta2_2 = q2[1];
		double theta4_2 = q2[3];

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

			double dif1_1 = abs(curr[0] - q1[0]);
			double dif2_1 = abs(curr[1] - q1[1]);
			double dif3_1 = abs(curr[3] - q1[3]);

			double dif1_2 = abs(curr[0] - q2[0]);
			double dif2_2 = abs(curr[1] - q2[1]);
			double dif3_2 = abs(curr[3] - q2[3]);

			double difTot1 = dif1_1 + dif2_1 + dif3_1;
			double difTot2 = dif1_2 + dif2_2 + dif3_2;

			if (difTot1 > difTot2) {
				conf[0] = q2[0];
				conf[1] = q2[1];
				conf[2] = q2[2];
				conf[3] = q2[3];
			}
			else {
				conf[0] = q1[0];
				conf[1] = q1[1];
				conf[2] = q1[2];
				conf[3] = q1[3];
			}
		}
		else if (!q1Valid && !q2Valid) {
			return false;
		}
		else if (!q1Valid) {
			conf[0] = q2[0];
			conf[1] = q2[1];
			conf[2] = q2[2];
			conf[3] = q2[3];
		}
		else {
			conf[0] = q1[0];
			conf[1] = q1[1];
			conf[2] = q1[2];
			conf[3] = q1[3];
		}
		return true;
	}
}

bool invkin(double x, double y, double z, double phi, JOINT &q1, JOINT &q2) {
	double c2 = (pow(x, 2) + pow(y, 2) - pow(a1, 2) - pow(a2, 2)) / (2 * a1 * a2);
	double s2 = sqrt(1 - pow(c2, 2));
	if (c2 > 1 || c2 < -1 || isnan(c2) || s2 > 1 || s2 < -1 || isnan(s2)) {
		return false;
	}
	else {
		double theta2_1 = atan2(s2, c2);
		double theta2_2 = atan2(-s2, c2);
		double theta1_1 = atan2(y, x) - atan2(a2 * s2, a1 + a2 * c2);
		double theta1_2 = atan2(y, x) - atan2(a2 * -s2, a1 + a2 * c2);
		double theta4_1 = theta1_1 + theta2_1 - phi;
		double theta4_2 = theta1_2 + theta2_2 - phi;
		double d3 = valueRounding(d1 + d2 - z - d4 - 0.410);

		printf("Joint vector 1: [%lf, %lf, %lf, %lf]\n",
			theta1_1 * 180 / PI, theta2_1 * 180 / PI, d3*1000, theta4_1 * 180 / PI);

		printf("Joint vector 2: [%lf, %lf, %fl, %lf]\n",
			theta1_2 * 180 / PI, theta2_2 * 180 / PI, d3*1000, theta4_2 * 180 / PI);

		q1[0] = theta1_1 * 180 / PI;
		q1[1] = theta2_1 * 180 / PI;
		q1[2] = d3 * 1000;
		q1[3] = theta4_1 * 180 / PI;
		q2[0] = theta1_2 * 180 / PI;
		q2[1] = theta2_2 * 180 / PI;
		q2[2] = d3 * 1000;
		q2[3] = theta4_2 * 180 / PI;
		return true;
	}

}
