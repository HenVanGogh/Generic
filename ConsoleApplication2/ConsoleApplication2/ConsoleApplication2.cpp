// ConsoleApplication2.cpp: Okreœla punkt wejœcia dla aplikacji konsoli.
//

#include "stdafx.h"
#include <iostream> 
#include <string>
#include <sstream>

#include "angleConventer.h"

using namespace std;
string mystr;
double input[7];

angleConventer axial(84.853);

struct position {
	
	double xRoll;
	double yRoll;
	double zRoll;

	bool absolut = false;
};
double Leg[4][3];
position leggedDefault;

double Radius;


#define PI 3.14159


#define defSel true

void rotate3axed(double xR, double yR, double zR)
{
	xR = (PI / double(180)) * xR;
	yR = (PI / double(180)) * yR;
	zR = (PI / double(180)) * zR;

	double deltaTable[4][3];

	deltaTable[0][0] = sin(xR) * Radius;
	deltaTable[0][1] = 0;
	deltaTable[0][2] = cos(xR) * Radius;

	deltaTable[1][0] = 0;
	deltaTable[1][1] = sin(yR) * Radius;
	deltaTable[1][2] = cos(yR) * Radius;

	deltaTable[2][0] = 0;
	deltaTable[2][1] = sin(-yR) * Radius;
	deltaTable[2][2] = cos(-yR) * Radius;

	deltaTable[3][0] = sin(-xR) * Radius;
	deltaTable[3][1] = 0;
	deltaTable[3][2] = cos(-xR) * Radius;

	if (defSel == true) {
		for (int n = 0; n < 4; n++) {
			for (int i = 0; i < 3; i++) {
				deltaTable[n][i] = Leg[n][i] + deltaTable[n][i];
				
				//cout << "n - " << (n) << " i -" << (i) << endl;
			}
		}
	}
	else {
		for (int n = 3; n >= 0; n--) {
			for (int i = 2; i >= 0; i--) {
				//deltaTable[n][i] = leggedselective.Leg[n][i] + deltaTable[n][i];
			}
		}
	}
	double D[4];
	D[0] = hypot(deltaTable[0][0], deltaTable[0][2]);
	D[1] = hypot(deltaTable[1][1], deltaTable[1][2]);
	D[2] = hypot(deltaTable[2][1], deltaTable[2][2]);
	D[3] = hypot(deltaTable[3][0], deltaTable[3][2]);

	double KT[4];
	KT[0] = cos(deltaTable[0][0] / D[0]) - xR;
	KT[1] = cos(deltaTable[1][1] / D[1]) - yR;
	KT[2] = cos(deltaTable[2][1] / D[2]) + yR;
	KT[3] = cos(deltaTable[3][0] / D[3]) + xR;

	double Zp[4];                 //End point Z pos WARRING this positions start at (0 , 0 , 0)
	Zp[0] = cos(KT[0]) * D[0];
	Zp[1] = cos(KT[1]) * D[1];
	Zp[2] = cos(KT[2]) * D[2];
	Zp[3] = cos(KT[3]) * D[3];

	double Sp[4];
	Sp[0] = sin(KT[0]) * D[0];
	Sp[1] = sin(KT[1]) * D[1];
	Sp[2] = sin(KT[2]) * D[2];
	Sp[3] = sin(KT[3]) * D[3];

	double Xc[4];                 //End point X pos WARRING this positions start at (0 , 0 , 0)
	Xc[0] =  cos(zR) * Sp[0];
	Xc[1] =  sin(zR) * Sp[1];
	Xc[2] = -sin(zR) * Sp[2];
	Xc[3] = -cos(zR) * Sp[3];

	double Yc[4];                 //End point Y pos WARRING this positions start at (0 , 0 , 0)
	Yc[0] = -sin(zR) * Sp[0];
	Yc[1] =  cos(zR) * Sp[1];
	Yc[2] = -cos(zR) * Sp[2];
	Yc[3] =  sin(zR) * Sp[3];

	cout << "if(ID != IDlimit){";
	cout << "stroke(" << "255" << "," << "105" << "," << "180" << ");" << endl;
	cout << "point(" << (Xc[0]) << "," << (Yc[0]) << "," << (Zp[0]) << ");" << endl;
	cout << "stroke(" << "255" << "," << "0" << "," << "0" << ");" << endl;
	cout << "point(" << (Xc[1]) << "," << (Yc[1]) << "," << (Zp[1]) << ");" << endl;
	cout << "stroke(" << "0" << "," << "255" << "," << "0" << ");" << endl;
	cout << "point(" << (Xc[2]) << "," << (Yc[2]) << "," << (Zp[2]) << ");" << endl;
	cout << "stroke(" << "0" << "," << "0" << "," << "255" << ");" << endl;
	cout << "point(" << (Xc[3]) << "," << (Yc[3]) << "," << (Zp[3]) << ");" << endl << endl;


	cout << "ID = ID + 1;" << endl;
	cout << "}else if(loopV == true){" << endl;
	cout << "tableV[1][1] = " << Xc[0] << ";" << endl;
	cout << "tableV[2][1] = " << Xc[1] << ";" << endl;
	cout << "tableV[3][1] = " << Xc[2] << ";" << endl;
	cout << "tableV[4][1] = " << Xc[3] << ";" << endl;

	cout << "tableV[1][2] = " << Yc[0] << ";" << endl;
	cout << "tableV[2][2] = " << Yc[1] << ";" << endl;
	cout << "tableV[3][2] = " << Yc[2] << ";" << endl;
	cout << "tableV[4][2] = " << Yc[3] << ";" << endl;

	cout << "tableV[1][3] = " << Zp[0] << ";" << endl;
	cout << "tableV[2][3] = " << Zp[1] << ";" << endl;
	cout << "tableV[3][3] = " << Zp[2] << ";" << endl;
	cout << "tableV[4][3] = " << Zp[3] << ";" << endl;
	
	cout << "loopV = false;" << endl;
	cout << "}" << endl;
}

void rotate3axedAbsolut(double xR , double yR , double zR) {
	setOf beel = axial.conv(xR,yR);
	/*
	beel.an[0];          //Azymuth
	beel.an[1];          //Increase
	 
	beel.eq[0];          //a plane
	beel.eq[1];          //b plane
	beel.eq[2];          //c plane
	*/
	 
	double elipse[3][2];
	double inputAngle[3];

	double outPutTable[3][2];

	cout << "t = " << endl;
	for (int i = 0; i < 4; i++) {
		elipse[0][0] = 1 / cos(beel.an[1]);
		elipse[i][1] = Radius * cos(inputAngle[i] + beel.an[0]);
		elipse[i][2] = (Radius * sin(inputAngle[i] + beel.an[0])) / elipse[0][0];

		outPutTable[i][0] = elipse[i][1];
		outPutTable[i][1] = elipse[i][2];
		outPutTable[i][2] = -((beel.eq[0] * elipse[i][1]) + (beel.eq[1] * elipse[i][2])) / beel.eq[2];

		if (i != 3) {
			cout << " {" << elipse[i][0] << ","
				<< elipse[i][1] << ","
				<< elipse[i][2] << "}," << endl;
		}
		else {
			cout << " {" << elipse[i][0] << ","
				<< elipse[i][1] << ","
				<< elipse[i][2] << "};" << endl;
		}
	
	}
	cout << "executePoints" << endl;
}



void rotate3axedGraphic(double xR, double yR, double zR)
{
	xR = (PI / double(180)) * xR;
	yR = (PI / double(180)) * yR;
	zR = (PI / double(180)) * zR;

	double deltaTable[4][3];

	deltaTable[0][0] = sin(xR) * Radius;
	deltaTable[0][1] = 0;
	deltaTable[0][2] = cos(xR) * Radius;

	deltaTable[1][0] = 0;
	deltaTable[1][1] = sin(yR) * Radius;
	deltaTable[1][2] = cos(yR) * Radius;

	deltaTable[2][0] = 0;
	deltaTable[2][1] = sin(-yR) * Radius;
	deltaTable[2][2] = cos(-yR) * Radius;

	deltaTable[3][0] = sin(-xR) * Radius;
	deltaTable[3][1] = 0;
	deltaTable[3][2] = cos(-xR) * Radius;

	if (defSel == true) {
		for (int n = 0; n < 4; n++) {
			for (int i = 0; i < 3; i++) {
				deltaTable[n][i] = Leg[n][i] + deltaTable[n][i];

				//cout << "n - " << (n) << " i -" << (i) << endl;
			}
		}
	}
	else {
		for (int n = 3; n >= 0; n--) {
			for (int i = 2; i >= 0; i--) {
				//deltaTable[n][i] = leggedselective.Leg[n][i] + deltaTable[n][i];
			}
		}
	}
	double D[4];
	D[0] = hypot(deltaTable[0][0], deltaTable[0][2]);
	D[1] = hypot(deltaTable[1][1], deltaTable[1][2]);
	D[2] = hypot(deltaTable[2][1], deltaTable[2][2]);
	D[3] = hypot(deltaTable[3][0], deltaTable[3][2]);

	double KT[4];
	KT[0] = cos(deltaTable[0][0] / D[0]) - xR;
	KT[1] = cos(deltaTable[1][1] / D[1]) - yR;
	KT[2] = cos(deltaTable[2][1] / D[2]) + yR;
	KT[3] = cos(deltaTable[3][0] / D[3]) + xR;

	double Zp[4];                 //End point Z pos WARRING this positions start at (0 , 0 , 0)
	Zp[0] = deltaTable[0][2];
	Zp[1] = deltaTable[1][2];
	Zp[2] = deltaTable[2][2];
	Zp[3] = deltaTable[3][2];

	double Sp[4];
	Sp[0] = sin(KT[0]) * D[0];
	Sp[1] = sin(KT[1]) * D[1];
	Sp[2] = sin(KT[2]) * D[2];
	Sp[3] = sin(KT[3]) * D[3];

	double Xc[4];                 //End point X pos WARRING this positions start at (0 , 0 , 0)
	Xc[0] = cos(zR) * deltaTable[0][0];
	Xc[1] = sin(zR) * 0;
	Xc[2] = -sin(zR) * 0;
	Xc[3] = -cos(zR) * deltaTable[3][0];

	double Yc[4];                 //End point Y pos WARRING this positions start at (0 , 0 , 0)
	Yc[0] = -sin(zR) * 0;
	Yc[1] = cos(zR) * deltaTable[1][1];
	Yc[2] = -cos(zR) * deltaTable[2][1];
	Yc[3] = sin(zR) * 0;

	cout << "if(ID != IDlimit){" << endl;
	cout << "stroke(" << "255" << "," << "105" << "," << "180" << ");" << endl;
	cout << "point(" << (Xc[0]) << "," << (Yc[0]) << "," << (Zp[0]) << ");" << endl;
	cout << "stroke(" << "255" << "," << "0" << "," << "0" << ");" << endl;
	cout << "point(" << (Xc[1]) << "," << (Yc[1]) << "," << (Zp[1]) << ");" << endl;
	cout << "stroke(" << "0" << "," << "255" << "," << "0" << ");" << endl;
	cout << "point(" << (Xc[2]) << "," << (Yc[2]) << "," << (Zp[2]) << ");" << endl;
	cout << "stroke(" << "0" << "," << "0" << "," << "255" << ");" << endl;
	cout << "point(" << (Xc[3]) << "," << (Yc[3]) << "," << (Zp[3]) << ");" << endl << endl;


	cout << "ID = ID + 1;" << endl;
	cout << "}else if(loopV == true){" << endl;
	cout << "tableV[1][1] = " << Xc[0] << ";" << endl;
	cout << "tableV[2][1] = " << Xc[1] << ";" << endl;
	cout << "tableV[3][1] = " << Xc[2] << ";" << endl;
	cout << "tableV[4][1] = " << Xc[3] << ";" << endl;

	cout << "tableV[1][2] = " << Yc[0] << ";" << endl;
	cout << "tableV[2][2] = " << Yc[1] << ";" << endl;
	cout << "tableV[3][2] = " << Yc[2] << ";" << endl;
	cout << "tableV[4][2] = " << Yc[3] << ";" << endl;

	cout << "tableV[1][3] = " << Zp[0] << ";" << endl;
	cout << "tableV[2][3] = " << Zp[1] << ";" << endl;
	cout << "tableV[3][3] = " << Zp[2] << ";" << endl;
	cout << "tableV[4][3] = " << Zp[3] << ";" << endl;

	cout << "loopV = false;" << endl;
	cout << "}" << endl;
}

void setDefaultPosMain(double x, double y, double z)
{
	Leg[0][0] = x;
	Leg[0][1] = 0;
	Leg[0][2] = z;

	Leg[1][0] = 0;
	Leg[1][1] = y;
	Leg[1][2] = z;

	Leg[2][0] = 0;
	Leg[2][1] = -y;
	Leg[2][2] = z;

	Leg[3][0] = -x;
	Leg[3][1] = 0;
	Leg[3][2] = z;
}






int main()
{
	do
	{
		cout << '\n' << "Press a key to continue...";
	} while (cin.get() != '\n');

	cout << '\n' << "Input Radius ...";
	cin  >> input[0];
	if (input[0] == 'd') {
		input[0] = 0;
	}
	input[0] = Radius;

	cout << '\n' << "Input x rotation orgin...";
	cin  >> input[1];
	if (input[1] == 'd') {
		input[1] = 0;
	}
	cout << '\n' << "Input x rotation endpoint...";
	cin  >> input[2];
	if (input[2] == 'd') {
		input[2] = 0;
	}
	cout << '\n' << "Input y rotation orgin...";
	cin  >> input[3];
	if (input[3] == 'd') {
		input[3] = 0;
	}
	cout << '\n' << "Input y rotation endpoint...";
	cin  >> input[4];
	if (input[4] == 'd') {
		input[4] = 0;
	}
	cout << '\n' << "Input z rotation orgin...";
	cin  >> input[5];
	if (input[5] == 'd') {
		input[5] = 0;
	}
	cout << '\n' << "Input z rotation endpoint...";
	cin  >> input[6];
	if (input[6] == 'd') {
		input[6] = 0;
	}

	getline(cin, mystr);


	cout << input[0] << endl;
	cout << input[1] << endl;
	cout << input[2] << endl;
	cout << input[3] << endl;
	cout << input[4] << endl;
	cout << input[5] << endl;
	cout << input[6] << endl;

	//cin >> input;

	double xVal;
	double yVal;
	double zVal;

	double multiplicationVal[3];
	double BaseVal[3];
	BaseVal[0] = input[2] - input[1];
	BaseVal[1] = input[4] - input[3];
	BaseVal[2] = input[6] - input[5];

	setDefaultPosMain(100, 100, 100);

	for (int p = 0; p < 100; p++) {
		cout << "//New_Master nr - " << (p) << " - ******" << endl << endl;

		multiplicationVal[0] = BaseVal[0] / 100.00 * p;
		multiplicationVal[1] = BaseVal[1] / 100.00 * p;
		multiplicationVal[2] = BaseVal[2] / 100.00 * p;

		xVal = multiplicationVal[0] + input[1];
		yVal = multiplicationVal[1] + input[1];
		zVal = multiplicationVal[2] + input[1];

		rotate3axed(xVal, yVal, zVal);
	}

	
	while (1);
}
