#include "stdafx.h"
#include "angleConventer.h"
#include <math.h>
double radius;

double ou[4][3];

angleConventer::angleConventer(double Radius)
{
	radius = Radius;
}

double angleConventer::rortatingPlaneHeight(double angle)
{
	return tan(angle) * radius;
}

void angleConventer::rotate3axedAbsolut(setOf input) {

}

void angleConventer::rotate2axed(double xPos, double yPos, double zPos, double azymuth, double increase, int valZero, int valZero1)
{



	A1 = azymuth + 0.785398;
	A2 = 0.785398 - azymuth;

	a = radius * sin(A1);
	c = -radius * sin(A2);

	b = radius * cos(A1);
	d = -radius * cos(A2);

	
	
	vRu = hypot(xPos, zPos) + radius;
	vR =  hypot(vRu, yPos);
	vRk = asin(yPos / vR);

	KR1 = hypot(a, b);
	KR2 = hypot(c, d);

	Kx1 = (sin(increase)*a) / 2;
	Kx2 = (sin(increase)*c) / 2;

	increaseBis1 = acos(1 - (Kx1 * Kx1) / 14400.06);
	increaseBis2 = acos(1 - (Kx2 * Kx2) / 14400.06);

	K1 = vRk + increaseBis1 * valZero;
	K2 = vRk + increaseBis2 * valZero;
	K3 = vRk - increaseBis2 * valZero;
	K4 = vRk - increaseBis1 * valZero;

	L1 = cos(K1)*vR;
	L2 = cos(K2)*vR;
	L3 = cos(K3)*vR;
	L4 = cos(K4)*vR;

	newXZ1 = L1 - radius;
	newXZ2 = L2 - radius;
	newXZ3 = L3 - radius;
	newXZ4 = L4 - radius;


	yBis1 = sin(K1)*vR;
	yBis2 = sin(K2)*vR;
	yBis3 = sin(K3)*vR;
	yBis4 = sin(K4)*vR;

	yBis1 = hypot(vR, L1);
	yBis2 = hypot(vR, L2);
	yBis3 = hypot(vR, L3);
	yBis4 = hypot(vR, L4);

	xBis1 = newXZ1 / sqrt(2);
	xBis2 = newXZ2 / sqrt(2);
	xBis3 = newXZ3 / sqrt(2);
	xBis4 = newXZ4 / sqrt(2);

	zBis1 = newXZ1 / sqrt(2);
	zBis2 = newXZ2 / sqrt(2);
	zBis3 = newXZ3 / sqrt(2);
	zBis4 = newXZ4 / sqrt(2);

	if (valZero1 == 1) {

		ou[1][1] = xBis1;
		ou[1][2] = yBis1;
		ou[1][3] = zBis1;

		ou[2][1] = xBis2;
		ou[2][2] = yBis2;
		ou[2][3] = zBis2;

		ou[3][1] = xBis3;
		ou[3][2] = yBis3;
		ou[3][3] = zBis3;

		ou[4][1] = xBis4;
		ou[4][2] = yBis4;
		ou[4][3] = zBis4;

	}
	else {
		ou[2][1] = xBis1;
		ou[2][2] = yBis1;
		ou[2][3] = zBis1;

		ou[4][1] = xBis2;
		ou[4][2] = yBis2;
		ou[4][3] = zBis2;

		ou[1][1] = xBis3;
		ou[1][2] = yBis3;
		ou[1][3] = zBis3;

		ou[3][1] = xBis4;
		ou[3][2] = yBis4;
		ou[3][3] = zBis4;

	}


}


double angleConventer::getExtremum( double point1, double point2) {

	int place;

	if ((point1 > 0) && (point2 > 0)) {
		place = 0;
	}
    else if((point1 < 0) && (point2 > 0)) {
		place = 1;
	}
	else if ((point1 < 0) && (point2 < 0)) {
		place = 2;
	}
	else if ((point1 > 0) && (point2 < 0)) {
		place = 3;
	}
	
	point1 = llabs(point1);
	point2 = llabs(point2);
	return (90.0 * point2) / (point1 * point2) + (place * 90.0);
}


convData angleConventer::getMissingPoint(double t[3][2]) {
	double a = t[1][1] - ((t[0][1] - t[0][1]) * (t[2][2] - t[0][2])) 
		- ((t[2][2] - t[0][2]) * (t[2][1] - t[0][1]));
	double b = t[1][1] - ((t[1][0] - t[0][0]) * (t[2][2] - t[0][2]))
		- ((t[1][2] - t[0][2]) * (t[2][0] - t[0][0]));
	double c = t[1][1] - ((t[1][0] - t[0][0]) * (t[2][1] - t[0][1]))
		- ((t[1][1] - t[0][1]) * (t[2][0] - t[0][0]));

	convData returnData;

	returnData.z = ((a *t[3][0]) + (b *t[3][1])) / c;
	returnData.eq[0] = a;
	returnData.eq[1] = b;
	returnData.eq[2] = c;
	return returnData;
}


setOf angleConventer::conv(double angle1 , double angle2) {
	double z1 = rortatingPlaneHeight(angle1);
	double z2 = rortatingPlaneHeight(angle2);

	double azymuth = getExtremum(z1,z2);

	double x = cos(azymuth) * radius;
	double y = sin(azymuth) * radius;

	double p[3][2];
	p[0][0] = 0.0;
	p[0][1] = 0.0;
	p[0][2] = 0.0;

	p[0][0] = 0.0;
	p[0][0] = radius;
	p[0][0] = z1;

	p[0][0] = radius;
	p[0][0] = 0.0;
	p[0][0] = z2;

	p[0][0] = x;
	p[0][0] = y;

	convData equasion = getMissingPoint(p);
	setOf returnVal;

	double z3 = equasion.z;

	returnVal.eq[0] = equasion.eq[0];
	returnVal.eq[1] = equasion.eq[1];
	returnVal.eq[2] = equasion.eq[2];

	
	returnVal.an[1] = atan(z3 / radius);
	returnVal.an[0] = azymuth;
	return returnVal;
}

/*
double angleConventer::map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
*/
angleConventer::~angleConventer()
{
}
