/*
 Name:		Generic.ino
 Created:	10.05.2018 20:48:47
 Author:	Mic
*/

#include <vl53l0x_types.h>
#include <vl53l0x_tuning.h>
#include <vl53l0x_platform_log.h>
#include <vl53l0x_platform.h>
#include <vl53l0x_interrupt_threshold_settings.h>
#include <vl53l0x_i2c_platform.h>
#include <vl53l0x_device.h>
#include <vl53l0x_def.h>
#include <vl53l0x_api_strings.h>
#include <vl53l0x_api_ranging.h>
#include <vl53l0x_api_core.h>
#include <vl53l0x_api_calibration.h>
#include <vl53l0x_api.h>
#include <Adafruit_VL53L0X.h>
#include <KalmanFilter.h>



#include "zweiAxedCommun.h"
//#include "MathModel.h"
#include "Runner.h"
//#include "VirtualModel.h"



struct translationTable {
	double T_Azym;
	double T_Increse;
	double T_Screw;
};
struct possiotion {
	double x[3];
	double y[3];
	double z[3];
};







const char HEADER = 'H';
const char A_TAG = 'M';
const char B_TAG = 'X';
const char C_TAG = 'O';
const char D_TAG = 'L';
const int  TOTAL_BYTES = 16;

//#include "ServoLX.h"




//MathModel mathModel(84.853);

// the setup function runs once when you press reset or power the board


void setup(){
	possiotion x[1];
	translationTable T_Azym;

//	mpu1.calibrateGyro();
}



// the loop function runs over and over again until power down or reset

zweiAxedCommun _zweiAxedCommun(A_TAG);
DreiAxedCommun _DreiAxedCommun(B_TAG);
DemoCommun _DemoCommun(C_TAG);

void loop() {
	if (Serial.available() >= TOTAL_BYTES)
	{
		if (Serial.read() == HEADER) {
			char tagH = Serial.read();

			_zweiAxedCommun.proceed(tagH);
			_DreiAxedCommun.proceed(tagH);
			_DemoCommun.proceed(tagH);
		}



	}
}
