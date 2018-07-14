#include "VirtualModel.h"
#include "Runner.h"

ServoLX servos1(7, 8);
ServoLX servos2(2, 3);

int stableTable[3][2];

long staL11 = 438.00; long staL12 = 197.00; long staL13 = 426.00;//
long staL21 = 493.00; long staL22 = 460.00; long staL23 = 427.00;//
long staL31 = 450.00;  long staL32 = 344.00; long staL33 = 437.00;//

long staP11 = 442.00; long staP12 = 491.00; long staP13 = 400.00;//
long staP21 = 390.00; long staP22 = 482.00; long staP23 = 169.00;//
long staP31 = 200.00;  long staP32 = 516.00; long staP33 = 339.00;//

Runner RunnerP1(P11, P12, P13, staP11, staP12, staP13, 1, 1);
Runner RunnerP2(P21, P22, P23, staP21, staP22, staP23, 1, 2);

Runner RunnerL1(L11, L12, L13, staL11, staL12, staL13, 0, 3);
Runner RunnerL2(L21, L22, L23, staL21, staL22, staL23, 0, 4);


struct orintationTable {
	double Xgyro;
	double Ygyro;
	double Xaccel;
	double Yaccel;
};

struct position {
	double Leg[3][2];
	double xRoll;
	double yRoll;
	double zRoll;

	bool absolut = false;
};

boolean defSel = true;

position legged;
position leggedDefault;
position leggedselective;

orintationTable Main;
orintationTable Cal1;
orintationTable Cal2;

position getAbsolutPosition(position rawPosition ,double Radius) {
	if (rawPosition.absolut == false) {
		rawPosition.Leg[0][0] = rawPosition.Leg[0][0] + Radius;
		rawPosition.Leg[1][1] = rawPosition.Leg[1][1] + Radius;
		rawPosition.Leg[2][1] = rawPosition.Leg[2][1] + Radius;
		rawPosition.Leg[3][0] = rawPosition.Leg[3][0] + Radius;

		rawPosition.absolut = true;
		return rawPosition;
	}
}

position unDoAbsolutPosition(position rawPosition, double Radius) {
	if (rawPosition.absolut == true) {
		rawPosition.Leg[0][0] = rawPosition.Leg[0][0] - Radius;
		rawPosition.Leg[1][1] = rawPosition.Leg[1][1] - Radius;
		rawPosition.Leg[2][1] = rawPosition.Leg[2][1] - Radius;
		rawPosition.Leg[3][0] = rawPosition.Leg[3][0] - Radius;

		rawPosition.absolut = false;
		return rawPosition;
	}
}

void VirtualModel::serialFlush() {
	while (Serial.available() > 0) {
		char t = Serial.read();
	}
}

void VirtualModel::enableGyro(char which)
{
	if (which == 1) {
		digitalWrite(8, LOW);
		digitalWrite(9, HIGH);
		digitalWrite(10, HIGH);
	}
	else if (which == 2) {
		digitalWrite(8, HIGH);
		digitalWrite(9, LOW);
		digitalWrite(10, HIGH);
	}
	else if (which == 3) {
		digitalWrite(8, HIGH);
		digitalWrite(9, HIGH);
		digitalWrite(10, LOW);
	}
	else {
		digitalWrite(8, HIGH);
		digitalWrite(9, HIGH);
		digitalWrite(10, HIGH);
	}
	delay(10);
}

/*
Main.Xgyro = kalmanY.update(accPitch, gyr.YAxis);
Main.Ygyro = kalmanX.update(accRoll, gyr.XAxis);

getOrientationCal2
*/
void VirtualModel::getOrientationMain()
{
	enableGyro(1);
	float accPitch = 0;
	float accRoll = 0;

	float kalPitch = 0;
	float kalRoll = 0;

	Vector acc = mpu1.readNormalizeAccel();
	Vector gyr = mpu1.readNormalizeGyro();

	// Calculate Pitch & Roll from accelerometer (deg)
	Main.Xaccel = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0) / M_PI;
	Main.Yaccel = (atan2(acc.YAxis, acc.ZAxis)*180.0) / M_PI;

	// Kalman filter
	Main.Xgyro = kalmanY.update(accPitch, gyr.YAxis);
	Main.Ygyro = kalmanX.update(accRoll, gyr.XAxis);
	//enableGyro(4);

}

/*Cal1.Xgyro = kalmanY.update(accPitch, gyr.YAxis);
	Cal1.Ygyro = kalmanX.update(accRoll, gyr.XAxis);*/
void VirtualModel::getOrientationCal1()
{
	enableGyro(2);
	float accPitch = 0;
	float accRoll = 0;

	float kalPitch = 0;
	float kalRoll = 0;

	Vector acc = mpu2.readNormalizeAccel();
	Vector gyr = mpu2.readNormalizeGyro();

	// Calculate Pitch & Roll from accelerometer (deg)
	Cal1.Xaccel = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0) / M_PI;
	Cal1.Yaccel = (atan2(acc.YAxis, acc.ZAxis)*180.0) / M_PI;

	// Kalman filter
	Cal1.Xgyro = kalmanY.update(accPitch, gyr.YAxis);
	Cal1.Ygyro = kalmanX.update(accRoll, gyr.XAxis);
	//enableGyro(4);

}

/*
Cal2.Xgyro = kalmanY.update(accPitch, gyr.YAxis);
	Cal2.Ygyro = kalmanX.update(accRoll, gyr.XAxis);
	*/
void VirtualModel::getOrientationCal2()
{
	enableGyro(3);
	float accPitch = 0;
	float accRoll = 0;

	float kalPitch = 0;
	float kalRoll = 0;

	Vector acc = mpu3.readNormalizeAccel();
	Vector gyr = mpu3.readNormalizeGyro();

	// Calculate Pitch & Roll from accelerometer (deg)
	Cal2.Xaccel = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0) / M_PI;
	Cal2.Yaccel = (atan2(acc.YAxis, acc.ZAxis)*180.0) / M_PI;

	// Kalman filter
	Cal2.Xgyro = kalmanY.update(accPitch, gyr.YAxis);
	Cal2.Ygyro = kalmanX.update(accRoll, gyr.XAxis);
	//enableGyro(4);

}

short int VirtualModel::getMeasurent() {
	VL53L0X_RangingMeasurementData_t measure;

	lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

	if (measure.RangeStatus != 4) {  // phase failures have incorrect data
		return(measure.RangeMilliMeter);
	}
	else {
		return(-1);
	}

}


VirtualModel::VirtualModel(double radius)
{

	Serial.begin(115200);
	radius = Radius;

	servos1.begin();
	servos2.begin();
	servos1.disable();
	servos2.disable();

	enableGyro(1);
	while (!mpu1.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
	{
		delay(500);
	}
	mpu1.calibrateGyro();
	enableGyro(2);
	while (!mpu2.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
	{
		delay(500);
	}
	mpu2.calibrateGyro();
	enableGyro(3);
	while (!mpu3.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
	{
		delay(500);
	}
	mpu3.calibrateGyro();
	enableGyro(4);

	if (!lox.begin()) {
		Serial.println(F("Failed to boot VL53L0X"));
		while (1);
	}
}


void VirtualModel::rotate2axed(double xPos, double yPos, double zPos, double azymuth, double increase, int valZero, int valZero1)
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


	A1 = azymuth + 0.785398;
	A2 = 0.785398 - azymuth;

#ifdef anal
	Serial.println("");
	Serial.print("A1 :  "); Serial.println(A1);
	Serial.print("A2 :  "); Serial.println(A2);
	Serial.println("");
#endif

	a = Radius * sin(A1);
	c = -Radius * sin(A2);

#ifdef anal
	Serial.println("");
	Serial.print("a :  "); Serial.println(a);
	Serial.print("c :  "); Serial.println(c);
	Serial.println("");
#endif

	b = Radius * cos(A1);
	d = -Radius * cos(A2);

#ifdef anal
	Serial.println("");
	Serial.print("b :  "); Serial.println(b);
	Serial.print("d :  "); Serial.println(d);
	Serial.println("");
#endif

	//H1a = sin(increase) * a;
	//H1b = sin(increase) * b;

	//H1 = H1a + yPos;
	//H2 = H2a + yPos;

	vRu = sqrt((xPos*xPos) + (zPos*zPos)) + Radius;
	vR = sqrt((vRu*vRu) + (yPos*yPos));
	//vRk = acos(((vRu*vRu)+(vR*vR) - (yPos*yPos))/ (2*vRu * vR));
	vRk = asin(yPos / vR);
#ifdef anal
	Serial.println("");
	Serial.print("vRu :  "); Serial.println(vRu);
	Serial.print("vR :  "); Serial.println(vR);
	Serial.print("vRk :  "); Serial.println(vRk);
	Serial.println("");
#endif

	//Lz1 = sqrt((H1*H1)+(a*a)+(c*c));
	//Lz2 = sqrt((H2*H2)+(b*b)+(d*d));
	KR1 = sqrt((a*a) + (b*b));
	KR2 = sqrt((c*c) + (d*d));

#ifdef anal
	Serial.println("");
	Serial.print("KR1 :  "); Serial.println(KR1);
	Serial.print("KR2 :  "); Serial.println(KR2);
	Serial.println("");
#endif

	Kx1 = (sin(increase)*a) / 2;
	Kx2 = (sin(increase)*c) / 2;

#ifdef anal
	Serial.println("");
	Serial.print("Kx1 :  "); Serial.println(Kx1);
	Serial.print("Kx2 :  "); Serial.println(Kx2);
	Serial.println("");
#endif

	increaseBis1 = acos(1 - (Kx1 * Kx1) / 14400.06);
	increaseBis2 = acos(1 - (Kx2 * Kx2) / 14400.06);

#ifdef anal
	Serial.println("");
	Serial.print("increaseBis1 :  "); Serial.println(increaseBis1);
	Serial.print("increaseBis2 :  "); Serial.println(increaseBis2);
	Serial.println("");
#endif


	K1 = vRk + increaseBis1 * valZero;
	K2 = vRk + increaseBis2 * valZero;
	K3 = vRk - increaseBis2 * valZero;
	K4 = vRk - increaseBis1 * valZero;

	/*
	if(azymuth < 0){
	K1 = vRk - increaseBis1;
	K2 = vRk - increaseBis2;
	K3 = vRk + increaseBis2;
	K4 = vRk + increaseBis1;
	}
	*/

#ifdef anal
	Serial.println("");
	Serial.print("K1 :  "); Serial.println(K1);
	Serial.print("K2 :  "); Serial.println(K2);
	Serial.print("K3 :  "); Serial.println(K3);
	Serial.print("K4 :  "); Serial.println(K4);
	Serial.println("");
#endif

	L1 = cos(K1)*vR;
	L2 = cos(K2)*vR;
	L3 = cos(K3)*vR;
	L4 = cos(K4)*vR;

#ifdef anal
	Serial.println("");
	Serial.print("L1 :  "); Serial.println(L1);
	Serial.print("L2 :  "); Serial.println(L2);
	Serial.print("L3 :  "); Serial.println(L3);
	Serial.print("L4 :  "); Serial.println(L4);
	Serial.println("");
#endif

	newXZ1 = L1 - Radius;
	newXZ2 = L2 - Radius;
	newXZ3 = L3 - Radius;
	newXZ4 = L4 - Radius;

#ifdef anal
	Serial.println("");
	Serial.print("newXZ1 :  "); Serial.println(newXZ1);
	Serial.print("newXZ2 :  "); Serial.println(newXZ2);
	Serial.print("newXZ3 :  "); Serial.println(newXZ3);
	Serial.print("newXZ4 :  "); Serial.println(newXZ4);
	Serial.println("");
#endif

	yBis1 = sin(K1)*vR;
	yBis2 = sin(K2)*vR;
	yBis3 = sin(K3)*vR;
	yBis4 = sin(K4)*vR;

#ifdef anal
	Serial.println("");
	Serial.print("yBis1 :  "); Serial.println(yBis1);
	Serial.print("yBis2 :  "); Serial.println(yBis2);
	Serial.print("yBis3 :  "); Serial.println(yBis3);
	Serial.print("yBis4 :  "); Serial.println(yBis4);
	Serial.println("");
#endif



	yBis1 = sqrt((vR*vR) - (L1*L1));
	yBis2 = sqrt((vR*vR) - (L2*L2));
	yBis3 = sqrt((vR*vR) - (L3*L3));
	yBis4 = sqrt((vR*vR) - (L4*L4));

	//Serial.print(yBis1); Serial.print(" ");
	//Serial.println(yBis2);

#ifdef anal
	Serial.println("");
	Serial.print("yBis1s :  "); Serial.println(yBis1);
	Serial.print("yBis2s :  "); Serial.println(yBis2);
	Serial.print("yBis3s :  "); Serial.println(yBis3);
	Serial.print("yBis4s :  "); Serial.println(yBis4);
	Serial.println("");
#endif

	xBis1 = newXZ1 / sqrt(2);
	xBis2 = newXZ2 / sqrt(2);
	xBis3 = newXZ3 / sqrt(2);
	xBis4 = newXZ4 / sqrt(2);



#ifdef anal
	Serial.println("");
	Serial.print("xBis1 :  "); Serial.println(xBis1);
	Serial.print("xBis2 :  "); Serial.println(xBis2);
	Serial.print("xBis3 :  "); Serial.println(xBis3);
	Serial.print("xBis4 :  "); Serial.println(xBis4);
	Serial.println("");
#endif

	zBis1 = newXZ1 / sqrt(2);
	zBis2 = newXZ2 / sqrt(2);
	zBis3 = newXZ3 / sqrt(2);
	zBis4 = newXZ4 / sqrt(2);

#ifdef anal
	Serial.println("");
	Serial.print("zBis1 :  "); Serial.println(zBis1);
	Serial.print("zBis2 :  "); Serial.println(zBis2);
	Serial.print("zBis3 :  "); Serial.println(zBis3);
	Serial.print("zBis4 :  "); Serial.println(zBis4);
	Serial.println("");
#endif
	//Serial.println("WUT ZE FUK");


	if (valZero1 == 1) {

		posTableVirualMode[1][1] = xBis1;
		posTableVirualMode[1][2] = yBis1;
		posTableVirualMode[1][3] = zBis1;

		posTableVirualMode[2][1] = xBis2;
		posTableVirualMode[2][2] = yBis2;
		posTableVirualMode[2][3] = zBis2;

		posTableVirualMode[3][1] = xBis3;
		posTableVirualMode[3][2] = yBis3;
		posTableVirualMode[3][3] = zBis3;

		posTableVirualMode[4][1] = xBis4;
		posTableVirualMode[4][2] = yBis4;
		posTableVirualMode[4][3] = zBis4;

		/*
		RunnerP1.Step(xBis1, yBis1, zBis1);
		RunnerP2.Step(xBis2, yBis2, zBis2);

		RunnerL1.Step(xBis3, yBis3, zBis3);
		RunnerL2.Step(xBis4, yBis4, zBis4);
		*/
		//Serial.println("WUT ZE FUK");
	}
	else {
		posTableVirualMode[2][1] = xBis1;
		posTableVirualMode[2][2] = yBis1;
		posTableVirualMode[2][3] = zBis1;

		posTableVirualMode[4][1] = xBis2;
		posTableVirualMode[4][2] = yBis2;
		posTableVirualMode[4][3] = zBis2;

		posTableVirualMode[1][1] = xBis3;
		posTableVirualMode[1][2] = yBis3;
		posTableVirualMode[1][3] = zBis3;

		posTableVirualMode[3][1] = xBis4;
		posTableVirualMode[3][2] = yBis4;
		posTableVirualMode[3][3] = zBis4;

		/*
		RunnerP2.Step(xBis1, yBis1, zBis1);
		RunnerL2.Step(xBis2, yBis2, zBis2);

		RunnerP1.Step(xBis3, yBis3, zBis3);
		RunnerL1.Step(xBis4, yBis4, zBis4);
		*/
	}


}

void VirtualModel::rotate2axedNew(double xR, double yR)
{

}
/*
#define SELECTIVE_MODE 1  move legs in selective mode
#define ALL_MOVE_MODE 2   move legs in default move
*/
void VirtualModel::rotate2axedNew(double xR, double yR, char type)
{
	
		double deltaTable[3][2];

		deltaTable[0][0] = sin(xR) * Radius;
		deltaTable[0][2] = cos(xR) * Radius;

		deltaTable[1][1] = sin(yR) * Radius;
		deltaTable[1][2] = cos(yR) * Radius;

		deltaTable[2][1] = sin(-yR) * Radius;
		deltaTable[2][2] = cos(-yR) * Radius;

		deltaTable[3][0] = sin(-xR) * Radius;
		deltaTable[3][2] = cos(-xR) * Radius;

		
			if (defSel == true) {
				for (int n = 3; n >= 0; n--) {
					for (int i = 2; i >= 0; i--) {
						deltaTable[n][i] = leggedDefault.Leg[n][i] + deltaTable[n][i];
					}
				}
			}
			else {
				for (int n = 3; n >= 0; n--) {
					for (int i = 2; i >= 0; i--) {
						deltaTable[n][i] = leggedselective.Leg[n][i] + deltaTable[n][i];
					}
				}
			}
		


		double D[4];
		D[0] = hypot(deltaTable[0][0], deltaTable[0][2]);
		D[1] = hypot(deltaTable[1][1], deltaTable[1][2]);
		D[2] = hypot(deltaTable[2][1], deltaTable[2][2]);
		D[3] = hypot(deltaTable[3][0], deltaTable[3][2]);

		double KT[3];
		KT[0] = cos(deltaTable[0][0] / D[0]) - xR;
		KT[1] = cos(deltaTable[1][1] / D[1]) - yR;
		KT[2] = cos(deltaTable[2][1] / D[2]) + yR;
		KT[3] = cos(deltaTable[3][0] / D[3]) + xR;

		double Zp[3];                 //End point Z pos WARRING this positions start at (0 , 0 , 0)
		Zp[0] = cos(KT[0]) * D[0];
		Zp[1] = cos(KT[1]) * D[1];
		Zp[2] = cos(KT[2]) * D[2];
		Zp[3] = cos(KT[3]) * D[3];

		double Sp[3];                  //End point X & Y pos WARRING this positions start at (0 , 0 , 0)
		Sp[0] = sin(KT[0]) * D[0];     //Points 1.2 ; 4.2 ; 2.1 ; 3.1 Are default - no change was applied
		Sp[1] = sin(KT[1]) * D[1];
		Sp[2] = sin(KT[2]) * D[2];
		Sp[3] = sin(KT[3]) * D[3];

	
}

void VirtualModel::tranzitSelectiveToDefault(int literation, int timeIntegral) {

	if (defSel == true) {
			for (int n = 3; n >= 0; n--) {
				for (int i = 2; i >= 0; i--) {
					legged.Leg[n][i] = map(timeIntegral, 0, literation,
						leggedDefault.Leg[n][i], leggedselective.Leg[n][i]);
				}
			}
	}
	else {
		for (int n = 3; n >= 0; n--) {
			for (int i = 2; i >= 0; i--) {
				legged.Leg[n][i] = map(timeIntegral, 0, literation,
					leggedselective.Leg[n][i], leggedDefault.Leg[n][i]);
			}
		}
	}
}

boolean changeMode() {
	defSel != defSel;
	return defSel;
}


void VirtualModel::rotate3axed(double xR, double yR , double zR)
{
	double deltaTable[3][2];

	deltaTable[0][0] = sin(xR) * Radius;
	deltaTable[0][2] = cos(xR) * Radius;

	deltaTable[1][1] = sin(yR) * Radius;
	deltaTable[1][2] = cos(yR) * Radius;

	deltaTable[2][1] = sin(-yR) * Radius;
	deltaTable[2][2] = cos(-yR) * Radius;

	deltaTable[3][0] = sin(-xR) * Radius;
	deltaTable[3][2] = cos(-xR) * Radius;

	if (defSel == true) {
		for (int n = 3; n >= 0; n--) {
			for (int i = 2; i >= 0; i--) {
				deltaTable[n][i] = leggedDefault.Leg[n][i] + deltaTable[n][i];
			}
		}
	}
	else {
		for (int n = 3; n >= 0; n--) {
			for (int i = 2; i >= 0; i--) {
				deltaTable[n][i] = leggedselective.Leg[n][i] + deltaTable[n][i];
			}
		}
	}
	double D[4];
	D[0] = hypot(deltaTable[0][0], deltaTable[0][2]);
	D[1] = hypot(deltaTable[1][1], deltaTable[1][2]);
	D[2] = hypot(deltaTable[2][1], deltaTable[2][2]);
	D[3] = hypot(deltaTable[3][0], deltaTable[3][2]);

	double KT[3];
	KT[0] = cos(deltaTable[0][0] / D[0]) - xR;
	KT[1] = cos(deltaTable[1][1] / D[1]) - yR;
	KT[2] = cos(deltaTable[2][1] / D[2]) + yR;
	KT[3] = cos(deltaTable[3][0] / D[3]) + xR;

	double Zp[3];                 //End point Z pos WARRING this positions start at (0 , 0 , 0)
	Zp[0] = cos(KT[0]) * D[0];
	Zp[1] = cos(KT[1]) * D[1];
	Zp[2] = cos(KT[2]) * D[2];
	Zp[3] = cos(KT[3]) * D[3];

	double Sp[3];
	Sp[0] = sin(KT[0]) * D[0];
	Sp[1] = sin(KT[1]) * D[1];
	Sp[2] = sin(KT[2]) * D[2];
	Sp[3] = sin(KT[3]) * D[3];

	double Xc[3];                 //End point X pos WARRING this positions start at (0 , 0 , 0)
	Xc[0] = cos(zR) * Sp[0];
	Xc[1] = sin(zR) * Sp[1];
	Xc[2] = -sin(zR) * Sp[2];
	Xc[3] = -cos(zR) * Sp[3];

	double Yc[3];                 //End point Y pos WARRING this positions start at (0 , 0 , 0)
	Yc[0] = -sin(zR) * Sp[0];
	Yc[1] = cos(zR) * Sp[1];
	Yc[2] = -cos(zR) * Sp[2];
	Yc[3] = sin(zR) * Sp[3];

}

void VirtualModel::returnPointZero()
{
	legged = leggedDefault;
}

void VirtualModel::setDefaultPosMain(double x, double y, double z)
{
	leggedDefault.Leg[1][1] = x;
	leggedDefault.Leg[1][2] = y;
	leggedDefault.Leg[1][3] = z;

	leggedDefault.Leg[2][1] = x;
	leggedDefault.Leg[2][2] = y;
	leggedDefault.Leg[2][3] = z;

	leggedDefault.Leg[3][1] = x;
	leggedDefault.Leg[3][2] = y;
	leggedDefault.Leg[3][3] = z;

	leggedDefault.Leg[4][1] = x;
	leggedDefault.Leg[4][2] = y;
	leggedDefault.Leg[4][3] = z;

	/*
	legged.Leg[1][1] = x;
	legged.Leg[1][2] = y;
	legged.Leg[1][3] = z;

	legged.Leg[2][1] = x;
	legged.Leg[2][2] = y;
	legged.Leg[2][3] = z;

	legged.Leg[3][1] = x;
	legged.Leg[3][2] = y;
	legged.Leg[3][3] = z;

	legged.Leg[4][1] = x;
	legged.Leg[4][2] = y;
	legged.Leg[4][3] = z;
	*/

}

void VirtualModel::setDefaultPosLeg(char which, double x, double y, double z)
{

}

void VirtualModel::translate(double rollX, double rollY, double rollZ)
{
}

#define debugMode
#define marginOfError 0.2
#define repetytions 4
#define beginingMove 10

/*int repeatMeasure(){
				for (int i; i < repetytions; i++) {
					t = baseVal + 10;
					while ((Main.Xgyro - Cal1.Xgyro > 90 - marginOfError
						&& Main.Xgyro - Cal1.Xgyro < 90 + marginOfError)) {

						servos1.move((which * 10) + who, t);
						getOrientationMain();
						getOrientationCal1;
						t--;
						delay(1);
					}


				}
			//}*/

int VirtualModel::calibrateLeg(char which , char who , boolean why)
{
	if (why == true) {
		if (which <= 2) {
			float positionC = servos1.position((which * 10) + who);
			getOrientationMain();
			getOrientationCal1;
			float previousPos;

			previousPos = Main.Xgyro - Cal1.Xgyro;

			if (Main.Xgyro > 45) {
#ifdef debugMode
				Serial.print("Good calibration angle"); Serial.println(Main.Xgyro);
#endif
				for (int i = 0; i < 500; i++) {
					servos1.move((which * 10) + who, positionC + (50 / i));
					delay(1);
				}
				getOrientationMain();
				getOrientationCal1;
				int t = beginingMove;
				int baseVal; //= positionC - t;

				if (Main.Xgyro - Cal1.Xgyro > 90) {
					if (Main.Xgyro - Cal1.Xgyro > previousPos) {
						//Wrong we need to turn counterClockWise
						while ((Main.Xgyro - Cal1.Xgyro > 90 - marginOfError
							&& Main.Xgyro - Cal1.Xgyro < 90 + marginOfError)) {

							servos1.move((which * 10) + who, positionC - t);
							getOrientationMain();
							getOrientationCal1;
							t++;
							delay(1);
						}


					}
					else {
						//Right we need to continue to turn clockWise
						while ((Main.Xgyro - Cal1.Xgyro > 90 - marginOfError
							&& Main.Xgyro - Cal1.Xgyro < 90 + marginOfError)) {

							servos1.move((which * 10) + who, positionC - t);
							getOrientationMain();
							getOrientationCal1;
							t--;
							delay(1);
						}
					}
				}
				else {
					if (Main.Xgyro - Cal1.Xgyro < previousPos) {
						//Right
						while ((Main.Xgyro - Cal1.Xgyro > 90 - marginOfError
							&& Main.Xgyro - Cal1.Xgyro < 90 + marginOfError)) {

							servos1.move((which * 10) + who, positionC - t);
							getOrientationMain();
							getOrientationCal1;
							t--;
							delay(1);
						}
					}
					else {
						//Wrong
						while ((Main.Xgyro - Cal1.Xgyro > 90 - marginOfError
							&& Main.Xgyro - Cal1.Xgyro < 90 + marginOfError)) {

							servos1.move((which * 10) + who, positionC - t);
							getOrientationMain();
							getOrientationCal1;
							t++;
							delay(1);
						}
					}
				}


				baseVal = positionC - t;
				int baseValTable[repetytions];
				int sum;
				for (int i = 0; i < repetytions; i++) {
					t = baseVal + 10;
					while ((Main.Xgyro - Cal1.Xgyro > 90 - marginOfError
						&& Main.Xgyro - Cal1.Xgyro < 90 + marginOfError)) {

						servos1.move((which * 10) + who, t);
						getOrientationMain();
						getOrientationCal1;
						t--;
						delay(1);
					}
					sum = sum + t;

				}
				return(sum / repetytions);

			}
			else {
#ifdef debugMode
				Serial.print("Bad calibration angle"); Serial.println(Main.Xgyro);
#endif
			}

		}

		if (which > 2) {
			float positionC = servos2.position((which * 10) + who);
			getOrientationMain();
			getOrientationCal1;
			float previousPos;

			previousPos = Main.Xgyro - Cal1.Xgyro;

			if (Main.Xgyro > 45) {
#ifdef debugMode
				Serial.print("Good calibration angle"); Serial.println(Main.Xgyro);
#endif
				for (int i = 0; i < 500; i++) {
					servos2.move((which * 10) + who, positionC + (50 / i));
					delay(1);
				}
				getOrientationMain();
				getOrientationCal1;
				int t = beginingMove;
				int baseVal; //= positionC - t;

				if (Main.Xgyro - Cal1.Xgyro > 90) {
					if (Main.Xgyro - Cal1.Xgyro > previousPos) {
						//Wrong we need to turn counterClockWise
						while ((Main.Xgyro - Cal1.Xgyro > 90 - marginOfError
							&& Main.Xgyro - Cal1.Xgyro < 90 + marginOfError)) {

							servos2.move((which * 10) + who, positionC - t);
							getOrientationMain();
							getOrientationCal1;
							t++;
							delay(1);
						}


					}
					else {
						//Right we need to continue to turn clockWise
						while ((Main.Xgyro - Cal1.Xgyro > 90 - marginOfError
							&& Main.Xgyro - Cal1.Xgyro < 90 + marginOfError)) {

							servos2.move((which * 10) + who, positionC - t);
							getOrientationMain();
							getOrientationCal1;
							t--;
							delay(1);
						}
					}
				}
				else {
					if (Main.Xgyro - Cal1.Xgyro < previousPos) {
						//Right
						while ((Main.Xgyro - Cal1.Xgyro > 90 - marginOfError
							&& Main.Xgyro - Cal1.Xgyro < 90 + marginOfError)) {

							servos2.move((which * 10) + who, positionC - t);
							getOrientationMain();
							getOrientationCal1;
							t--;
							delay(1);
						}
					}
					else {
						//Wrong
						while ((Main.Xgyro - Cal1.Xgyro > 90 - marginOfError
							&& Main.Xgyro - Cal1.Xgyro < 90 + marginOfError)) {

							servos2.move((which * 10) + who, positionC - t);
							getOrientationMain();
							getOrientationCal1;
							t++;
							delay(1);
						}
					}
				}


				baseVal = positionC - t;
				int baseValTable[repetytions];
				int sum;
				for (int i = 0; i < repetytions; i++) {
					t = baseVal + 10;
					while ((Main.Xgyro - Cal1.Xgyro > 90 - marginOfError
						&& Main.Xgyro - Cal1.Xgyro < 90 + marginOfError)) {

						servos2.move((which * 10) + who, t);
						getOrientationMain();
						getOrientationCal1;
						t--;
						delay(1);
					}
					sum = sum + t;

				}
				return(sum / repetytions);

			}
			else {
#ifdef debugMode
				Serial.print("Bad calibration angle"); Serial.println(Main.Xgyro);
#endif
			}
		}
	}
else {
	if (which <= 2) {
		float positionC = servos1.position((which * 10) + who);
		getOrientationMain();
		getOrientationCal1;
		float previousPos;

		previousPos = Main.Ygyro - Cal1.Ygyro;

		if (Main.Ygyro > 45) {
#ifdef debugMode
			Serial.print("Good calibration angle"); Serial.println(Main.Ygyro);
#endif
			for (int i = 0; i < 500; i++) {
				servos1.move((which * 10) + who, positionC + (50 / i));
				delay(1);
			}
			getOrientationMain();
			getOrientationCal1;
			int t = beginingMove;
			int baseVal; //= positionC - t;

			if (Main.Ygyro - Cal1.Ygyro > 90) {
				if (Main.Ygyro - Cal1.Ygyro > previousPos) {
					//Wrong we need to turn counterClockWise
					while ((Main.Ygyro - Cal1.Ygyro > 90 - marginOfError
						&& Main.Ygyro - Cal1.Ygyro < 90 + marginOfError)) {

						servos1.move((which * 10) + who, positionC - t);
						getOrientationMain();
						getOrientationCal1;
						t++;
						delay(1);
					}


				}
				else {
					//Right we need to continue to turn clockWise
					while ((Main.Ygyro - Cal1.Ygyro > 90 - marginOfError
						&& Main.Ygyro - Cal1.Ygyro < 90 + marginOfError)) {

						servos1.move((which * 10) + who, positionC - t);
						getOrientationMain();
						getOrientationCal1;
						t--;
						delay(1);
					}
				}
			}
			else {
				if (Main.Ygyro - Cal1.Ygyro < previousPos) {
					//Right
					while ((Main.Ygyro - Cal1.Ygyro > 90 - marginOfError
						&& Main.Ygyro - Cal1.Ygyro < 90 + marginOfError)) {

						servos1.move((which * 10) + who, positionC - t);
						getOrientationMain();
						getOrientationCal1;
						t--;
						delay(1);
					}
				}
				else {
					//Wrong
					while ((Main.Ygyro - Cal1.Ygyro > 90 - marginOfError
						&& Main.Ygyro - Cal1.Ygyro < 90 + marginOfError)) {

						servos1.move((which * 10) + who, positionC - t);
						getOrientationMain();
						getOrientationCal1;
						t++;
						delay(1);
					}
				}
			}


			baseVal = positionC - t;
			int baseValTable[repetytions];
			int sum;
			for (int i = 0; i < repetytions; i++) {
				t = baseVal + 10;
				while ((Main.Ygyro - Cal1.Ygyro > 90 - marginOfError
					&& Main.Ygyro - Cal1.Ygyro < 90 + marginOfError)) {

					servos1.move((which * 10) + who, t);
					getOrientationMain();
					getOrientationCal1;
					t--;
					delay(1);
				}
				sum = sum + t;

			}
			return(sum / repetytions);

		}
		else {
#ifdef debugMode
			Serial.print("Bad calibration angle"); Serial.println(Main.Ygyro);
#endif
		}

	}

	if (which > 2) {
		float positionC = servos2.position((which * 10) + who);
		getOrientationMain();
		getOrientationCal1;
		float previousPos;

		previousPos = Main.Ygyro - Cal1.Ygyro;

		if (Main.Ygyro > 45) {
#ifdef debugMode
			Serial.print("Good calibration angle"); Serial.println(Main.Ygyro);
#endif
			for (int i = 0; i < 500; i++) {
				servos2.move((which * 10) + who, positionC + (50 / i));
				delay(1);
			}
			getOrientationMain();
			getOrientationCal1;
			int t = beginingMove;
			int baseVal; //= positionC - t;

			if (Main.Ygyro - Cal1.Ygyro > 90) {
				if (Main.Ygyro - Cal1.Ygyro > previousPos) {
					//Wrong we need to turn counterClockWise
					while ((Main.Ygyro - Cal1.Ygyro > 90 - marginOfError
						&& Main.Ygyro - Cal1.Ygyro < 90 + marginOfError)) {

						servos2.move((which * 10) + who, positionC - t);
						getOrientationMain();
						getOrientationCal1;
						t++;
						delay(1);
					}


				}
				else {
					//Right we need to continue to turn clockWise
					while ((Main.Ygyro - Cal1.Ygyro > 90 - marginOfError
						&& Main.Ygyro - Cal1.Ygyro < 90 + marginOfError)) {

						servos2.move((which * 10) + who, positionC - t);
						getOrientationMain();
						getOrientationCal1;
						t--;
						delay(1);
					}
				}
			}
			else {
				if (Main.Ygyro - Cal1.Ygyro < previousPos) {
					//Right
					while ((Main.Ygyro - Cal1.Ygyro > 90 - marginOfError
						&& Main.Ygyro - Cal1.Ygyro < 90 + marginOfError)) {

						servos2.move((which * 10) + who, positionC - t);
						getOrientationMain();
						getOrientationCal1;
						t--;
						delay(1);
					}
				}
				else {
					//Wrong
					while ((Main.Ygyro - Cal1.Ygyro > 90 - marginOfError
						&& Main.Ygyro - Cal1.Ygyro < 90 + marginOfError)) {

						servos2.move((which * 10) + who, positionC - t);
						getOrientationMain();
						getOrientationCal1;
						t++;
						delay(1);
					}
				}
			}


			baseVal = positionC - t;
			int baseValTable[repetytions];
			int sum;
			for (int i = 0; i < repetytions; i++) {
				t = baseVal + 10;
				while ((Main.Ygyro - Cal1.Ygyro > 90 - marginOfError
					&& Main.Ygyro - Cal1.Ygyro < 90 + marginOfError)) {

					servos2.move((which * 10) + who, t);
					getOrientationMain();
					getOrientationCal1;
					t--;
					delay(1);
				}
				sum = sum + t;

			}
			return(sum / repetytions);

		}
		else {
#ifdef debugMode
			Serial.print("Bad calibration angle"); Serial.println(Main.Ygyro);
#endif
		}
	}
}
}

void VirtualModel::calibrateBody() {
	stableTable[0][1] = calibrateLeg(1 , 2, true);
	stableTable[1][1] = calibrateLeg(2 , 2, false);
	stableTable[2][1] = calibrateLeg(3 , 2, false);
	stableTable[3][1] = calibrateLeg(4 , 2, true);

	stableTable[0][2] = calibrateLeg(1, 3, true);
	stableTable[1][2] = calibrateLeg(2, 3, false);
	stableTable[2][2] = calibrateLeg(3, 3, false);
	stableTable[3][2] = calibrateLeg(4, 3, true);
}

void VirtualModel::linearMove(double x, double y, double z)
{
}

void VirtualModel::moveAll(double XposT, double YposT, double ZposT, double XstaT, double YstaT, double ZstaT)
{
}

void VirtualModel::screwUrSelfWoof(int Xdefault, int Ydefault, int Zdefault, double substractAngle, double heightPlus)
{
}
