//#include "HardwareSerial.h"
#include "zweiAxedCommun.h"

VirtualModel _VirtualModel(84.853);

zweiAxedCommun::zweiAxedCommun(char TAG) : Commun(TAG)
{
	tag = TAG;
}

void zweiAxedCommun::proceed(char t_tag)
{
	if (t_tag == tag) {

		double Increase1;
		double Increase2;
		double Increase3;

		double Azymuth;
		double Azymuth1;
		double Increase;

		int valZero;
		int valZero1;

		double maxAngle = 0.645772;

		void readData();

		Azymuth = valC[1] * 2 * M_PI;
		Azymuth = Azymuth / 1000;

		if ((Azymuth >= 0) && (Azymuth < 1.5708)) {
			Increase1 = valC[2] - 500;
			Increase2 = Increase1 / 500;
			Increase = Increase2 * maxAngle;

			valZero = 1;
			valZero1 = 1;
			Azymuth = Azymuth - 0.78;
			_VirtualModel.rotate2axed(120, valC[3] + 100, 120, Azymuth, Increase, valZero, valZero1);


		}
		else if ((Azymuth >= 1.57) && (Azymuth < 3.14)) {

			Increase1 = valC[2] - 500;
			Increase2 = Increase1 / 500;
			Increase = Increase2 * maxAngle;

			valZero = -1;
			valZero1 = -1;
			Azymuth = Azymuth - 2.35619;
			_VirtualModel.rotate2axed(120, valC[3] + 100, 120, Azymuth, Increase, valZero, valZero1);

		}
		else if ((Azymuth >= 3.14) && (Azymuth < 4.71)) {

			Increase1 = valC[2] - 500;
			Increase2 = Increase1 / 500;
			Increase = Increase2 * maxAngle;

			valZero = -1;
			valZero1 = 1;
			Azymuth = Azymuth - 3.92699;
			_VirtualModel.rotate2axed(120, valC[3] + 100, 120, Azymuth, Increase, valZero, valZero1);

		}
		else if ((Azymuth >= 4.71) && (Azymuth <= 6.28)) {

			Increase1 = valC[2] - 500;
			Increase2 = Increase1 / 500;
			Increase = Increase2 * maxAngle;

			valZero = 1;
			valZero1 = -1;
			Azymuth = Azymuth - 5.49;
			_VirtualModel.rotate2axed(120, valC[3] + 100, 120, Azymuth, Increase, valZero, valZero1);

		}
		else {
			//INVALID ANGLE
		}

	}
}



DreiAxedCommun::DreiAxedCommun(char TAG) : Commun(TAG)
{
	tag = TAG;
}
void DreiAxedCommun::proceed(char t_tag)
{
	if (t_tag == tag) {

	}
}


DemoCommun::DemoCommun(char TAG) : Commun(TAG)
{
	tag = TAG;
}
void DemoCommun::proceed(char t_tag)
{
	if (t_tag == tag) {

	}
}
