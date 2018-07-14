#include "Commun.h"



Commun::Commun(const char TAG)
{
	tag = TAG;
}

void Commun::readData()
{
	valC[1] = Serial.read() * 256;
	valC[1] = valC[1] + Serial.read();
	valC[2] = Serial.read() * 256;
	valC[2] = valC[2] + Serial.read();
	valC[3] = Serial.read() * 256;
	valC[3] = valC[3] + Serial.read();
	valC[4] = Serial.read() * 256;
	valC[4] = valC[4] + Serial.read();
	valC[5] = Serial.read() * 256;
	valC[5] = valC[5] + Serial.read();
}

