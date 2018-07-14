#pragma once

#include "ServoLX.h"

class Runner
{
	
	
	double Leg0 = 56.3, Leg1 = 167, Leg2 = 136;
	bool Side;
	int pin1, pin2, pin3;
	double St1, St2, St3;
	double LT, L0, L1, L2, L3, L4, L5, L6, L7, X, Y, Z, T1, T2, T3, K1, K2, K3, K4, K5, K6;
	double radian = 57.2958;
	int Lp;

	


public:
	//int feedBack[3];
	
	double table[2];

	/*double returnVal1;
	double returnVal2;
	double returnVal3;*/

	
	Runner(int Pin1, int Pin2, int Pin3, double st1, double st2, double st3, bool side, int lp);
	void Step(double x, double y, double z);
	//Give command to leg

	int calibrate(int repetytions, int calSpeed);

	void getFeedback();
	
	
};