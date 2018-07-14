#pragma once
#include "HardwareSerial.h"
class Commun
{
protected:
	char tag;
	int valC[5];
public:
	Commun(const char TAG);

	void readData();
};




