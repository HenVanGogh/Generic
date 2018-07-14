#pragma once
typedef struct {
	double an[1];
	//double z;
	double eq[2];
} setOf;

typedef struct {
	double z;
	double eq[2];
}convData;

class angleConventer
{

	double A1, A2, A3, A4, hX1, hZ1, hX2, hZ2, hX3, hZ3, hX4, hZ4;

	double a, b, c, d;
	double H1a, H1b, H1, H2, vR, vRu, vRk;
	double K1, K2, K3, K4, L1, L2, L3, L4, yBis1, yBis2, yBis3, yBis4, Lz1, Lz2;
	double  xBis1, xBis2, xBis3, xBis4;
	double  zBis1, zBis2, zBis3, zBis4;

	double increaseBis1;
	double increaseBis2;



	double newXZ1;
	double newXZ2;
	double newXZ3;
	double newXZ4;

	double KR1, Kx1;
	double KR2, Kx2;



public:

	

	angleConventer(double Radius);

	double rortatingPlaneHeight(double angle);

	void rotate3axedAbsolut(setOf input);

	void rotate2axed(double xPos, double yPos, double zPos, double azymuth, double increase, int valZero, int valZero1);

	double getExtremum( double point1, double point2);

	convData getMissingPoint(double t[3][2]);

	setOf conv(double angle1, double angle2);

	double conv();

	double map(long x, long in_min, long in_max, long out_min, long out_max);

	~angleConventer();
};

