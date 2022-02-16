#include <MIDI.h>
#include <SoftwareSerial.h>
#include <MotionSensor.h>
#include <SweetMaker.h>
#include <FastLED.h>
#include <Wire.h>
#include <EEPROM.h>
#include "StrawberryString.h"

//StrawberryString myStrStr;
SoftwareSerial softSerial(9, 10); // Rx, Tx
MIDI_CREATE_INSTANCE(SoftwareSerial, softSerial, midiSerial);


void sendCommand(const char * command) {
	Serial.print("Command send :");
	Serial.println(command);
	softSerial.println(command);
	//wait some time
	delay(100);

	char reply[100];
	int i = 0;
	while (softSerial.available()) {
		reply[i] = softSerial.read();
		i += 1;
	}
	//end the string
	reply[i] = '\0';
	Serial.print(reply);
	Serial.println("Reply end");
}

void setup()
{
	pinMode(7, OUTPUT);
	pinMode(8, OUTPUT);
	digitalWrite(8, LOW);
	digitalWrite(7, HIGH);

//	myStrStr.init();

	Serial.begin(9600);
	softSerial.begin(9600);
//	midiSerial.begin();

	sendCommand("AT");
	sendCommand("AT+ROLE1");
	sendCommand("AT+UUID0xFFE0");
	sendCommand("AT+CHAR0xFFE1");
	sendCommand("AT+NAMEpageTurner");

}

void loop()
{
	if (Serial.available())
	{
		softSerial.write(Serial.read());
	}

	if (softSerial.available())
	{
		Serial.write(softSerial.read());
	}

//	myStrStr.update();
}