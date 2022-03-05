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

const double a1 = 0.195;
const double a2 = 0.142;
const double d1 = 0.405;
const double d2 = 0.07;
const double d4 = -0.140;

double* solve(double x, double y, double z, double phi);

double* invkin(double x, double y, double z, double phi);

double* where(double theta1, double theta2, double d3, double theta4);

double* kin(double theta1, double theta2, double d3, double theta4);

int main(int argc, char* argv[])
{
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

	while (1)
	{

		if (c != ESC)
		{
			cout << "Press 1 to initiate WHERE function. Press 2 to initiate SOLVE function\n";
			ch = _getch();

			if (ch == '1') {
				// do where func
				double theta1 = 0;
				double theta2 = 0;
				double d3 = 0;
				double theta4 = 0;

				cout << "Where\nPlease input the values for each Joint (Theta 1, Theta 2, Distance 3, Theta 4)\nTheta 1:";
				cin >> theta1;
				cout << "Theta 2 : ";
				cin >> theta2;
				cout << "Distance 3 : ";
				cin >> d3;
				cout << "Theta 4 : ";
				cin >> theta4;

				where(theta1, theta2, d3, theta4);
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

			printf("Press any key to continue \n");
			printf("Press ESC to exit \n");
			c = _getch();

		}
		else break;
	}


	return 0;
}

double* where(double theta1, double theta2, double d3, double theta4) {
	double T01[4][4];
	double T12[4][4];
	double T23[4][4];
	double T34[4][4];
	double T45[4][4];
	double T05[4][4];

	double result[5] = {};;
	//T01, T12, T34, same rotation matrices Z
	//Distance need to be entered
	//T01 = { {cos(theta1), -sin(theta1), 0, }, {sin(theta1), cos(theta1), 0, }, {0, 0, 1, }, {0, 0, 0, 1}};
	//T12 = {{cos(theta2), -sin(theta2), 0, }, {sin(theta2), cos(theta2), 0, }, {0, 0, 1, }, {0, 0, 0, 1} };
	//T23 = { {1, 0, 0, }, {0, -1, 0, }, {0, 0, -1, }, {0, 0, 0, } };
	//T34 = { {cos(theta4), -sin(theta4), 0, }, {sin(theta4), cos(theta4), 0, }, {0, 0, 1, }, {0, 0, 0, 1} };
	//T45 = {{1, 0, 0, }, {0, 1, 0, }, {0, 0, 1, }, {0, 0, 0, 1} };
	//T05 = T01 * T12 * T23 * T34 * T45;
	return result;
}

double* kin(double theta1, double theta2, double d3, double theta4) {
	double T01[4][4];
	double T12[4][4];
	double T23[4][4];
	double T34[4][4];
	double T45[4][4];
	double T05[4][4];

	double result[5] = {};

	//T01, T12, T34, same rotation matrices Z
	//Distance need to be entered
	//T01 = { {cos(theta1), -sin(theta1), 0, }, {sin(theta1), cos(theta1), 0, }, {0, 0, 1, }, {0, 0, 0, 1} };
	//T12 = { {cos(theta2), -sin(theta2), 0, }, {sin(theta2), cos(theta2), 0, }, {0, 0, 1, }, {0, 0, 0, 1} };
	//T23 = { {1, 0, 0, }, {0, -1, 0, }, {0, 0, -1, }, {0, 0, 0, } };
	//T34 = { {cos(theta4), -sin(theta4), 0, }, {sin(theta4), cos(theta4), 0, }, {0, 0, 1, }, {0, 0, 0, 1} };
	//T45 = {{1, 0, 0, }, {0, 1, 0, }, {0, 0, 1, }, {0, 0, 0, 1} };
	//T05 = T01 * T12 * T23 * T34 * T45;
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
			q[0] = jointVectors[0];
			q[1] = jointVectors[1];
			q[2] = jointVectors[2];
			q[3] = jointVectors[3];
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
