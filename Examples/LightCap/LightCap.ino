/*******************************************************************************
NewProject.ino - A 'blank' project to jumpstart your FizzyMint development

Copyright(C) 2017 Howard James May

This program is free software : you can redistribute it and / or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.If not, see <http://www.gnu.org/licenses/>.

Contact me at sweet.maker@outlook.com

*******************************************************************************/
#include <Wire.h>
#include <EEPROM.h>
#include <SweetMaker.h>
#include <MotionSensor.h>
#include "StrawberryString.h"
#include "EepromUtility.h"
#include "FrictionSpinner.h"

using namespace SweetMaker;

StrawberryString strStr;

static const uint16_t EVENT_CALLIBRATE = IEventHandler::USER + 0;
static const uint16_t EVENT_SET_ROT_OFF = IEventHandler::USER + 1;
static const uint16_t EVENT_PRINT_POSITION = IEventHandler::USER + 2;

void myEventHandler(uint16_t eventId, uint8_t src, uint16_t eventInfo);

static const SigGen::SAMPLE flash[] PROGMEM = { 255, 0 };
SigGen flasher;
ColourHSV myColourHSV[StrawberryString::num_lights];
Timer myTimer;
StaticGen satStatic;
StaticGen hueStatic;
FrictionSpinner spinner;

int16_t forceScale;

bool diagnosticsOn;

void processReadingBasic(void);
void processReadingSpinner(void);
void processWhiteLightning(void);


void setup()
{
	/* Start Serial at a speed (Baud rate) of 112500 Bytes per second */
	Serial.begin(112500);

	flasher.configSamples(flash, NUM_SAM(flash), 400, SigGen::INTERPOLATE_OFF);
	flasher.start(10);

	strStr.configEventHandlerCallback(myEventHandler);
	strStr.init();

	for(int i=0;i<strStr.num_lights;i++)
	  myColourHSV[i].setColour(123, 255, 255);

	satStatic.configDuty_256(210);
	satStatic.configPeriod_ms(10);

	hueStatic.configDuty_256(245);
	hueStatic.configPeriod_ms(250);

	/*
	* Flush Serial and start timer looking for user input in first 5 seconds
	*/
	while (Serial.available())
		Serial.read();

	Serial.println("LightCap");

	forceScale = 8;

  strStr.motionSensor.clearOffsetRotation();

  {
    RotationQuaternion_16384 ry((float)45, 0, 16384, 0);
    RotationQuaternion_16384 rz((float)45, 0, 0, 16384);
    ry.crossProduct(&rz);
    strStr.configOffsetRotation(&ry);
  }
}


void loop()
{
	PerfMon::getPerfMon()->intervalStop();
	PerfMon::getPerfMon()->intervalStart();

	for (uint8_t i = 0; i < StrawberryString::num_lights; i++)	{
		ColourConverter::ConvertToRGB(myColourHSV + i, strStr.ledStrip + i);
	}

	if (Serial.available()) {
		char c = Serial.read();
		switch (c) {
		case 'c':
			myEventHandler(EVENT_CALLIBRATE, 0, 0);
			break;
    case 'p':
      myEventHandler(EVENT_PRINT_POSITION, 0, 0);
      break;
		case 'd':
			diagnosticsOn = true;
			break;
		case 's':
			forceScale++;
			break;
		}
	}

	strStr.update();
}


void myEventHandler(uint16_t eventId, uint8_t eventRef, uint16_t eventInfo)
{
	switch (eventId)
	{

	case EVENT_CALLIBRATE:	{
		StrawberryString::EEPROM_DATA eeprom_data;
		Serial.println("Starting Self Cal");
		strStr.recalibrateMotionSensor();
		Serial.println("Writing to EEPROM");
		break;
	}

  case EVENT_PRINT_POSITION: {
    Serial.println("Print Position");
    int16_t roll = strStr.motionSensor.rotQuat.getSinRotX();
    int16_t pitch = strStr.motionSensor.rotQuat.getSinRotY();
    int16_t yaw_16384 = strStr.motionSensor.rotQuat.getSinRotZ();
    Serial.print(roll); Serial.print(" ");
    Serial.print(pitch); Serial.print(" ");
    Serial.print(yaw_16384); Serial.println();

    strStr.motionSensor.rotQuatDelta.printQ();

    break;
  }

	case TimerTickMngt::TIMER_TICK_S: 
	  break;

	case TimerTickMngt::TIMER_TICK_10S: // Generated once every ten seconds
		break;

	case MotionSensor::MOTION_SENSOR_INIT_ERROR:
		Serial.println("MOTION_SENSOR_INIT_ERROR: ");
		break;

	case MotionSensor::MOTION_SENSOR_READY:
		Serial.println("MOTION_SENSOR_READY: ");
		break;

	case MotionSensor::MOTION_SENSOR_RUNTIME_ERROR:
		Serial.println("Motion Sensor Error");
		break;

	case MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY:
	{
    //		processReadingBasic();
    //    processReadingSpinner();
    processWhiteLightning();
	}
	break;


	case SigGen::SIG_GEN_STOPPED: // A Signal Generator has been stopped
	case TimerTickMngt::TIMER_TICK_UPDATE: // Generated every time fizzyMint is updated - could be every 500us (micro seconds) e.g. less than a millisecond
	case TimerTickMngt::TIMER_FREQ_GEN: // Generated a certain number of times a seconds
	case TimerTickMngt::TIMER_EXPIRED: // A timer has expired - eventInfo from timerId
	case SigGen::SIG_GEN_STARTED: // A Signal Generator has been started
	case SigGen::SIG_GEN_FINISHED: // A Signal Generator has finished 
		break;

	case TimerTickMngt::TIMER_TICK_100MS: // Generated ten times a second
    if (diagnosticsOn) {
      RotationQuaternion_16384* rq = &strStr.motionSensor.rotQuatDelta;
      uint32_t ang_vel = (uint32_t)rq->x * (uint32_t)rq->x + (uint32_t)rq->y * (uint32_t)rq->y + (uint32_t)rq->z * (uint32_t)rq->z;
      rq->printQ();
      Serial.println(ang_vel);
		}
		break;
	}
}


/*
 * processReadingBasic
 *
 * Sets the hue to a fixed value which can be changed by tilting about 'Y' followed by
 * rotating about 'Z'
 * This sets the saturation to a static value
 * The brightness is set to 
 */
void processReadingBasic(void)
{
	static boolean rolling = false;
	int16_t roll = strStr.motionSensor.rotQuat.getSinRotX();
	int16_t pitch = strStr.motionSensor.rotQuat.getSinRotY();
	int16_t yaw_16384 = strStr.motionSensor.rotQuat.getSinRotZ();

	uint8_t bri = 32;
	/*
	* Set Brightness
	*/
	{
		if ((pitch > 0) && (pitch < 4500))
			bri = map(pitch, 0, 4500, 32, 255);
		if (pitch >= 4500)
			bri = 255;

		myColourHSV[0].value = bri;
	}

	/*
	* Set Saturation
	*/
	{
		if (satStatic.readValue())
			myColourHSV[0].saturation = 245;
		else
			myColourHSV[0].saturation = 230;

		if (rolling)
			myColourHSV[0].saturation = 255;
	}

	/*
	* Set Hue
	*/
	{
		static int16_t roll_yaw_start;
		static uint8_t roll_hue_start;
		static uint8_t myHue;

		if ((roll < -8192) || (roll > 8192))
		{
			if (!rolling) {
				rolling = true;
				roll_yaw_start = yaw_16384;
				roll_hue_start = myColourHSV[0].hue;
			}

			int16_t new_hue = yaw_16384 - roll_yaw_start;
			new_hue = map(new_hue, 0, 9000, 0, 255);
			new_hue += (int16_t)roll_hue_start;
			myHue = (uint8_t)new_hue;
		}
		else {
			rolling = false;
		}

		myColourHSV[0].hue = myHue;
		if (!hueStatic.readValue())
			myColourHSV[0].hue += (255 / 3);
	}
}


/*
* processReadingSpinner: This sets colour based on the yaw of some virtual spinning
*                        arrow which spins about the Z axis. The arrow acts like an 
*                        inertial body subject to static and dynamic friction.
*                        This resists starting to move and tends to make it stop 
*                        Torque is applie proportional to the angular velocity
*                        of the motion sensor about 'Z'
*/ 
void processReadingSpinner(void)
{
	static int16_t last_yaw_16384;
	int16_t yaw_16384 = strStr.motionSensor.rotQuat.getSinRotZ();
	int16_t deltaYaw = yaw_16384 - last_yaw_16384;
	last_yaw_16384 = yaw_16384;
	uint8_t hue;
	uint8_t spread;

	spinner.applyTorque_512(deltaYaw * forceScale);

	spread = (uint8_t)strStr.motionSensor.rotQuat.getSinRotY()/(int16_t)1000;
	
	hue = (uint8_t)(spinner.rotation_100000 / (int32_t)512);
	applyHueSpread(hue, 2);
}


/*
* processWhiteLightning: 
*/
void processWhiteLightning(void)
{
  RotationQuaternion_16384* rq = &strStr.motionSensor.rotQuatDelta;
  uint32_t ang_vel = (uint32_t)rq->x * (uint32_t)rq->x + (uint32_t)rq->y * (uint32_t)rq->y + (uint32_t)rq->z * (uint32_t)rq->z;

  ang_vel = 5 + (ang_vel >> 10);
  uint8_t saturation = 100;

  for (int i = 0; i < strStr.num_lights; i++) {
    myColourHSV[i].hue = 128;
    myColourHSV[i].saturation = saturation;
    myColourHSV[i].value = (uint8_t)ang_vel;
  }

}



void applyHueSpread(uint8_t _hue, uint8_t spread)
{
	_hue -= spread;
	_hue -= spread;

	for (int i = 0; i < strStr.num_lights; i++) {
		myColourHSV[i].hue = _hue;
		_hue += spread;
	}
}