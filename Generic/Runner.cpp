#pragma once
#include "Runner.h"
#include "math.h"
#include "ServoLX.h"


Runner::Runner(int Pin1, int Pin2, int Pin3, double st1, double st2, double st3, bool side, int lp) {

	pin1 = Pin1;
	pin2 = Pin2;
	pin3 = Pin3;

	L0 = Leg0;
	L1 = Leg1;
	L2 = Leg2;

	St1 = st1;
	St2 = st2;
	St3 = st3;

	Side = side;

	Lp = lp;


	
}


void Runner::Step(double x, double y, double z) {


	if ((y > 20) && (z > 17) && (x > 17) && (y < 250) && (z < 350) && (x < 350)) {
		bool error = false;
		X = x;
		Y = y;
		Z = z;

		double XZsafeFactor = sqrt(91809 - (Y*Y));

#ifdef ShowCals
		//Serial.print("Safe Factor XZ - ");
		//Serial.println(XZsafeFactor); 
#endif

		double sqrtZX = hypot(X , Z);
		if (sqrt(sqrtZX) > XZsafeFactor) {
			//Serial.println("XZ Safe factor ERROR");

#ifdef ShowCals
			//Serial.println(sqrtZX);
			//Serial.println("WTF");
#endif
		}
		if (Y > 300) {
			//Serial.println("Y Safe factor ERROR");
			//Serial.println("WTF?");
		}
		T1 = atan(Z / X);
		L5 = sqrt(sqrtZX);
		L4 = sqrt((y*y) + (L5*L5));
		K1 = acos(L5 / L4);
		double PowL2L4, PowL1, subL2L4L1;
		PowL2L4 = ((L2*L2) + (L4*L4));
		PowL1 = L1*L1;
		subL2L4L1 = PowL2L4 - PowL1;
		K2 = acos(subL2L4L1 / (2 * L4*L2));
		K3 = K1 + K2;
		K5 = acos(((L1*L1) + (L2*L2) - (L4*L4)) / (2 * L1*L2));
		K6 = K3;
		T2 = 3.14159 - (K5 + K6);
		T3 = 3.14159 - K5;

		double TD1 = T1*radian*3.333;
		double TD2 = T2*radian*3.333;
		double TD3 = T3*radian*3.333;

		double pos1;
		double pos2;
		double pos3;


		/*
		if(isnan(pos1) == false || isnan(pos2) == true || isnan(pos3) == true){
		Serial.print("ERROR LEG"); Serial.print(Lp); Serial.println("IS NAN");
		}
		*/

		//pos1 = St1 - TD1;    // To jest zamienione na IF
		pos2 = St2 - TD2;
		if (Lp == 2) {
			pos3 = St3 + TD3;
		}
		else {
			pos3 = St3 - TD3;
		}

		if (Lp == 4 || Lp == 1) {
			pos1 = St1 + TD1;
		}
		else {
			pos1 = St1 - TD1;
		}
		//pos3 = St3 - TD3;



#ifdef errorAnalysis

		if (Lp == 1) {
			float exportValue1 = float(pos1);
			float exportValue2 = float(pos2);
			float exportValue3 = float(pos3);
			//Serial.println(exportValue1);
			//Serial.println(exportValue2);
			//Serial.println(exportValue3);
		}
		//Serial.print(Lp);

		//Serial.println(pos2);
		//Serial.println(pos3);
		if (isnan(pos1) == true) {
			Serial.print("ERROR_LEG 1_"); Serial.print(Lp); Serial.println("_IS NAN");
			error = true;
		}
		if (isnan(pos2) == true) {
			Serial.print("ERROR_LEG 2_"); Serial.print(Lp); Serial.println("_IS NAN");
			error = true;
		}
		if (isnan(pos3) == true) {
			Serial.print("ERROR_LEG 3_"); Serial.print(Lp); Serial.println("_IS NAN");
			error = true;
		}
#endif

#ifdef ShowCals
		Serial.println("");
		Serial.println("-----------NEW CLASS-----------");
		Serial.print("Leg Nr : "); Serial.println(Lp);
		Serial.println("**********************");
		Serial.print("pos1 = "); Serial.println(pos1);
		Serial.print("pos2 = "); Serial.println(pos2);
		Serial.print("pos3 = "); Serial.println(pos3);
		Serial.println("**********************");
		Serial.print("T1 = "); Serial.println(T1*radian);
		Serial.print("T2 = "); Serial.println(T2*radian);
		Serial.print("T3 = "); Serial.println(T3*radian);
		Serial.println("-----------Support-----------");
		Serial.print("sqrt = "); Serial.println(sqrtZX);
		//Serial.print("LT = "); Serial.println(LT);
		Serial.print("TD1 = "); Serial.println(TD1);
		Serial.print("TD2 = "); Serial.println(TD2);
		Serial.print("TD3 = "); Serial.println(TD3);
		Serial.print("PowL2L4 = "); Serial.println(PowL2L4);
		Serial.print("PowL1 = "); Serial.println(PowL1);
		Serial.print("subL2L4L1 = "); Serial.println(subL2L4L1);
		Serial.print("K2 = "); Serial.println(K2*radian);
		Serial.print("K3 = "); Serial.println(K3*radian);
		Serial.print("K5 = "); Serial.println(K5*radian);
		Serial.print("K6 = "); Serial.println(K6*radian);

		Serial.println("----------------------");
		Serial.println("");
#endif


		if (error == false) {
			table[0] = pos1;
			table[1] = pos2;
			table[2] = pos3;

			/*
			if ((Lp == 1) || (Lp == 2)) {
				servos1.move(pin1, pos1);
				servos1.move(pin2, pos2);
				servos1.move(pin3, pos3);
			}
			if ((Lp == 3) || (Lp == 4)) {
				servos2.move(pin1, pos1);
				servos2.move(pin2, pos2);
				servos2.move(pin3, pos3);
			}
			*/

			//  pwm.setPWM(pin3, 0,pos3 );
		}
		else {
			//Serial.print("ErrorCode = "); Serial.println(error);
		}
	}
	else {
		/*
		Serial.print("DATA_ERROR LEG_NR :   "); Serial.println(Lp);
		Serial.print("X :   "); Serial.println(x);
		Serial.print("Y :   "); Serial.println(y);
		Serial.print("Z :   "); Serial.println(z);
		*/
	}
}

int Runner::calibrate(int repetytions, int calSpeed)
{
	return 0;
}

void Runner::getFeedback()
{
	
}


