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
typedef double SPLCOEFF[4][4];
typedef double JOINTMAT[4][5];

//robot constants for lengths
const double a1 = 0.195;
const double a2 = 0.142;
const double d1 = 0.405;
const double d2 = 0.07;
const double d4 = 0.140;

//functions
bool where(double theta1, double theta2, double d3, double theta4, JOINT& conf);		//Where function used to find where the robot will end up with joint parameters
bool kin(double theta1, double theta2, double d3, double theta4, JOINT& conf);			//Solves the Transform Matrices from joint parameters
bool solve(double x, double y, double z, double phi, JOINT& conf1);						//Solve function used to find joint parameters of the end effector location
bool invkin(double x, double y, double z, double phi, JOINT& conf1, JOINT& conf2);		//Solves the Inverse Kinematics using the robots parameters 
																						//Trajectory Planner of the movements
bool planTrajectory(JOINT& qv0, JOINT& qv1, JOINT& qv2, JOINT& qv3, JOINT& qv4, double time, SPLCOEFF& spl0, SPLCOEFF& spl1, SPLCOEFF& spl2, SPLCOEFF& spl3);
//bool execTrajectory(moves_Conf, velocity_array, acceleration_array, time);				//Executes the motion of the Trajectory planning using POS, Vec, Acc=0
double minOrMax(double left, double middle, double right, double t);					//chooses the slope??
void buildVelocityMat(JOINTMAT& pts, JOINTMAT& vels, double t);							//Build a matrix of avg velocity

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

	double x;		//Start Frame
	double y;
	double z;
	double phi;

	JOINT q;

	double x1;		//2nd Frame (Interm 1)
	double y1;
	double z1;
	double phi1;

	double x2;		//3rd Frame (Interm 2)
	double y2;
	double z2;
	double phi2;

	double x3;		//4th Frame (Interm 3)
	double y3;
	double z3;
	double phi3;

	double x4;		//Goal Frame
	double y4;
	double z4;
	double phi4;

	JOINT qv0;		//Joint Configurations
	JOINT qv1;
	JOINT qv2;
	JOINT qv3;
	JOINT qv4;

	SPLCOEFF spl0;	//Spline Trajectories
	SPLCOEFF spl1;
	SPLCOEFF spl2;
	SPLCOEFF spl3;

	double time;

	char ch;
	int c;

	double i = 1.0;

	const int ESC = 27;

	printf("Press any key to continue \n");
	printf("Press ESC to exit \n");

	c = _getch();

	while (1) {

		if (c != ESC) {
			cout << "Press 1 to initiate WHERE function. Press 2 to initiate SOLVE function. Press 3 to initiate trajectory planner. Press 4 to Grasp.\n";
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
					MoveToConfiguration(q, true);//Not Sure this works
				}
			}
			else if (ch == '3') {
				printf("Trajectory Planner:  You must input 4 joint vectors and the total time of the trajectory.\n");
				cout << "Please input the values for your first vector: x (m), y (m), z (m), and phi (deg)\nx : ";
				cin >> x1;
				cout << "y : ";
				cin >> y1;
				cout << "z : ";
				cin >> z1;
				cout << "phi : ";
				cin >> phi1;
				phi1 = DEG2RAD(phi1);
				if (!solve(x1, y1, z1, phi1, qv1)) {
					printf("The coordinates entered are outside the joint space of the robot.\n\n");
				}
				else {
					cout << "Please input the values for your second vector: x (m), y (m), z (m), and phi (deg)\nx : ";
					cin >> x2;
					cout << "y : ";
					cin >> y2;
					cout << "z : ";
					cin >> z2;
					cout << "phi : ";
					cin >> phi2;
					phi2 = DEG2RAD(phi2);
					if (!solve(x2, y2, z1, phi2, qv2)) {
						printf("The coordinates entered are outside the joint space of the robot.\n\n");
					}
					else {
						cout << "Please input the values for your third vector: x (m), y (m), z (m), and phi (deg)\nx : ";
						cin >> x3;
						cout << "y : ";
						cin >> y3;
						cout << "z : ";
						cin >> z3;
						cout << "phi : ";
						cin >> phi3;
						phi3 = DEG2RAD(phi3);
						if (!solve(x3, y3, z3, phi3, qv3)) {
							printf("The coordinates entered are outside the joint space of the robot.\n\n");
						}
						else {
							cout << "Please input the values for your fourth and final vector: x (m), y (m), z (m), and phi (deg)\nx : ";
							cin >> x4;
							cout << "y : ";
							cin >> y4;
							cout << "z : ";
							cin >> z4;
							cout << "phi : ";
							cin >> phi4;
							phi4 = DEG2RAD(phi4);
							if (!solve(x4, y4, z4, phi4, qv4)) {
								printf("The coordinates entered are outside the joint space of the robot.\n\n");
							}
							else {
								cout << "Please input the total time the trajectory should take in seconds \ntime (s) : ";
								cin >> time;
								GetConfiguration(qv0);
								if (!planTrajectory(qv0, qv1, qv2, qv3, qv4, time, spl0, spl1, spl2, spl3)) {
									printf("Somewhere in your planned trajectory a joint limit was exceeded\n");
								}
								else {
									printf("Succes!!\n");
								}
							}
						}
					}
				}
				//values entered now run trajectory


			}
			else if (ch == '4') {
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

bool where(double theta1, double theta2, double d3, double theta4, JOINT& conf) {
	if (!kin(theta1, theta2, d3, theta4, conf)) {
		return false;
	}
	else {
		return true;
	}
}

bool kin(double theta1, double theta2, double d3, double theta4, JOINT& conf) {
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

bool solve(double x, double y, double z, double phi, JOINT& conf) {
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
			q1Valid = false;
		}
		else if (theta1_2 < -150.0 || theta1_2 > 150.0 ||
			theta2_2 < -100.0 || theta2_2 > 100.0 ||
			d3 < -200.0 || d3 > -100.0 ||
			theta4_2 < -160.0 || theta4_2 > 160.0) {
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
		printf("Joint vector: [%lf, %lf, %lf, %lf]\n",
			conf[0], conf[1], conf[2], conf[3]);
		return true;
	}
}

bool invkin(double x, double y, double z, double phi, JOINT& q1, JOINT& q2) {
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

bool planTrajectory(JOINT& qv0, JOINT& qv1, JOINT& qv2, JOINT& qv3, JOINT& qv4, double time, SPLCOEFF& spl0, SPLCOEFF& spl1, SPLCOEFF& spl2, SPLCOEFF& spl3) {
	double t = time / 4.0;//i think using this t is wrong in spline calcs
	int i = 0;
	JOINTMAT pts = {
		{qv0[0], qv1[0], qv2[0], qv3[0], qv4[0]},
		{qv0[1], qv1[1], qv2[1], qv3[1], qv4[1]},
		{qv0[2], qv1[2], qv2[2], qv3[2], qv4[2]},
		{qv0[3], qv1[3], qv2[3], qv3[3], qv4[3]}
	};

	JOINTMAT vels = {
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0}
	};

	buildVelocityMat(pts, vels, t);

	for (i = 0; i < 4; i++) {//not sure the right use of t
		spl0[i][0] = pts[0][i];
		spl0[i][1] = vels[0][i];
		spl0[i][2] = (3 / pow(t, 2)) * (pts[0][i + 1] - pts[0][i]) - (vels[0][i] * 2 / t) - (vels[0][i + 1] * 1 / t);
		spl0[i][3] = (-2 / pow(t, 3)) * (pts[0][i + 1] - pts[0][i]) + ((vels[0][i + 1] + vels[0][i]) * 1 / pow(t, 2));

		spl1[i][0] = pts[1][i];
		spl1[i][1] = vels[1][i];
		spl1[i][2] = (3 / pow(t, 2)) * (pts[1][i + 1] - pts[1][i]) - (vels[1][i] * 2 / t) - (vels[1][i + 1] * 1 / t);
		spl1[i][3] = (-2 / pow(t, 3)) * (pts[1][i + 1] - pts[1][i]) + ((vels[1][i + 1] + vels[1][i]) * 1 / pow(t, 2));

		spl2[i][0] = pts[2][i];
		spl2[i][1] = vels[2][i];
		spl2[i][2] = (3 / pow(t, 2)) * (pts[2][i + 1] - pts[2][i]) - (vels[2][i] * 2 / t) - (vels[2][i + 1] * 1 / t);
		spl2[i][3] = (-2 / pow(t, 3)) * (pts[2][i + 1] - pts[2][i]) + ((vels[2][i + 1] + vels[2][i]) * 1 / pow(t, 2));

		spl3[i][0] = pts[3][i];
		spl3[i][1] = vels[3][i];
		spl3[i][2] = (3 / pow(t, 2)) * (pts[3][i + 1] - pts[3][i]) - (vels[3][i] * 2 / t) - (vels[3][i + 1] * 1 / t);
		spl3[i][3] = (-2 / pow(t, 3)) * (pts[3][i + 1] - pts[3][i]) + ((vels[3][i + 1] + vels[3][i]) * 1 / pow(t, 2));
	}

	double numcycles = time / 0.020; // think we need to use the time divided by 4
	int numcycles = int(numcycles);
	bool limitExceeded = false;

	double PosArray[4][numcycles];
	double VecArray[4][numcycles];
	double AccArray[4][numcycles];

	double pos0[numcycles];
	//this can be done better

	for (int i = 0; i < 4; i++) {//should be 4
		for (int j = 0; j <= numcycles; j++) {
			double t = j * 0.020; //is this supposed to be j
			double pos0[j] = spl0[i][0] + spl0[i][1] * t + spl0[i][2] * pow(t, 2) + spl0[i][3] * pow(t, 3);
			double pos1[j] = spl1[i][0] + spl1[i][1] * t + spl1[i][2] * pow(t, 2) + spl1[i][3] * pow(t, 3);
			double pos2[j] = spl2[i][0] + spl2[i][1] * t + spl2[i][2] * pow(t, 2) + spl2[i][3] * pow(t, 3);
			double pos3[j] = spl3[i][0] + spl3[i][1] * t + spl3[i][2] * pow(t, 2) + spl3[i][3] * pow(t, 3);

			double vel0[j] = spl0[i][1] + 2 * spl0[i][2] * t + 3 * spl0[i][3] * pow(t, 2);
			double vel1[j] = spl1[i][1] + 2 * spl1[i][2] * t + 3 * spl1[i][3] * pow(t, 2);
			double vel2[j] = spl2[i][1] + 2 * spl2[i][2] * t + 3 * spl2[i][3] * pow(t, 2);
			double vel3[j] = spl3[i][1] + 2 * spl3[i][2] * t + 3 * spl3[i][3] * pow(t, 2);

			/* just put to have could also put 0 think we need to have something
			* but it doesnt work
			double acc1[j] = 2 * spl0[i][2] + 6 * spl0[i][3] * t;
			double acc2[j] = 2 * spl1[i][2] + 6 * spl1[i][3] * t;
			double acc3[j] = 2 * spl2[i][2] + 6 * spl2[i][3] * t;
			double acc4[j] = 2 * spl3[i][2] + 6 * spl3[i][3] * t;
			*/
			/*
			double acc1[j] = 0;
			double acc2[j] = 0;
			double acc3[j] = 0;
			double acc4[j] = 0;
			*/
			

			if (pos0 < -150.0 || pos0 > 150.0 ||
				pos1 < -100.0 || pos1 > 100.0 ||
				pos2 < -200.0 || pos2 > -100.0 ||
				pos3 < -160.0 || pos3 > 160.0) {
				limitExceeded = true;
				break;
			}
			else if (vel0 < -150.0 || vel0 > 150.0 ||
				vel1 < -150.0 || vel1 > 150.0 ||
				vel2 < -50.0 || vel2 > 50.0 ||
				vel3 < -150.0 || vel3 > 150.0) {
				limitExceeded = true;
				break;
			}
			//acc maybe too
			/*
			else if (acc0 < -150.0 || acc0 > 150.0 ||
				acc1 < -150.0 || acc1 > 150.0 ||
				acc2 < -50.0 || acc2 > 50.0 ||
				acc3 < -150.0 || acc3 > 150.0) {
				limitExceeded = true;
				break;
			}
			*/
		}
		if (limitExceeded) {
			break;
		}
	}

	if (limitExceeded) return false;
	else return true;
}
/*
bool execTrajectory(conf, vel, acc, time) {
	double numcycles = time / 0.020;
	for (int i = 0; i < numcycles; i++)
	{
		MoveWithConfVelAcc(PositionArray[i], VelArray[i], AccArray[i]);
	}

}
*/

double minOrMax(double left, double middle, double right, double t) {
	if ((left <= middle) and (right <= middle) ||
		(left >= middle) and (right >= middle)) {
		return 0.0;
	}
	else {
		double slopeLeft = (middle - left) / t;
		double slopeRight = (right - middle) / t;
		double slopeAvg = (slopeLeft + slopeRight) / 2;
		return slopeAvg;
	}
}

void buildVelocityMat(JOINTMAT& pts, JOINTMAT& vels, double t) {
	for (int i = 0; i < 4; i++) {
		if (i == 0 || i == 4) {//think this is 4 (start, 1, 2, 3, goal)
			vels[0][i] = 0.0;
			vels[1][i] = 0.0;
			vels[2][i] = 0.0;
			vels[3][i] = 0.0;
		}
		else {
			vels[0][i] = minOrMax(pts[0][i - 1], pts[0][i], pts[0][i + 1], t);
			vels[1][i] = minOrMax(pts[1][i - 1], pts[1][i], pts[1][i + 1], t);
			vels[2][i] = minOrMax(pts[2][i - 1], pts[2][i], pts[2][i + 1], t);
			vels[3][i] = minOrMax(pts[3][i - 1], pts[3][i], pts[3][i + 1], t);
		}
	}
}
