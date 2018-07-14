#pragma once

#include "math.h"



#define directionL1X  1
#define directionL1Z -1

#define directionL2X -1
#define directionL2Z -1 

#define directionP1X  1
#define directionP1Z  1

#define directionP2X -1
#define directionP2Z  1
 
#define SELECTIVE_MODE 1
#define ALL_MOVE_MODE 2
#define DELTAS_MODE 3


#define P11 1
#define P12 2
#define P13 3
#define P14 4

#define P21 11
#define P22 12
#define P23 13
#define P24 14

#define L11 21
#define L12 22
#define L13 23
#define L14 24

#define L21 31
#define L22 32
#define L23 33
#define L24 34

#include "MPU6050.h"
#include "Adafruit_VL53L0X.h"
#include "KalmanFilter.h"

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

MPU6050 mpu1;
MPU6050 mpu2;
MPU6050 mpu3;


class VirtualModel
{
	double Radius;
	double xRoll;
	double yRoll;
	double zRoll;

	
	position getAbsolutPosition(position rawPosition);

	position unDoAbsolutPosition(position rawPosition);

	void serialFlush();

	void enableGyro(char which);

	struct orintationTable {
		double Xgyro;
		double Ygyro;
		double Xaccel;
		double Yaccel;
	};

	void getOrientationMain();

	void getOrientationCal1();

	void getOrientationCal2();

	short int getMeasurent();

public:
	double posTableVirualMode[4][3];




	VirtualModel(double radius);
	
	

	/*void rotate2axed(double xPos, double yPos, double zPos,
		double azymuth, double increase, int valZero, int valZero1) {}*/

	void rotate2axed(double xPos, double yPos, double zPos, double azymuth, double increase, int valZero, int valZero1);

	void rotate2axedNew(double xR, double yR);

	void rotate2axedNew(double xR, double yR, char type);

	void tranzitSelectiveToDefault(int literation, int timeIntegral);



//	void rotate3axed(double xPos, double yPos, double zPos,
//		double azymuth, double increase, double increaseBiser, int valZero, int valZero1);

	void rotate3axed(double xR, double yR, double zR);

	void returnPointZero();

	/*
	ta pozycja jest nak³adk¹ na normaln¹ pozycje w
	stane spoczynku lub po uruchomieniu robot przyjmuje t¹ pozycje
	*/
	void setDefaultPosMain(double x, double y, double z);

	void setDefaultPosLeg(char which ,double x, double y, double z);

	void translate(double rollX, double rollY, double rollZ);

	int calibrateLeg(char which , char who, boolean why);

	void calibrateBody();



	void linearMove(double x, double y, double z);

	void moveAll(double XposT, double YposT, double ZposT, double XstaT, double YstaT, double ZstaT);

	void screwUrSelfWoof(int Xdefault, int Ydefault, int Zdefault, double substractAngle, double heightPlus);

};


