/*******************************************************************************
  StrawberryString Implementation - takes a Nano, an MPU6050 and some RGB leds
                and makes some magic. Integrated with the SweetMaker framework. 

  Copyright(C) 2017-2022  Howard James May

  This file is part of the SweetMaker family of libraries

  The SweetMaker SDK is free software: you can redistribute it and / or
  modify it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  The SweetMaker SDK is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.If not, see <http://www.gnu.org/licenses/>.

  Contact me at sweet.maker@outlook.com

********************************************************************************
  Release     Date                        Change Description
--------|-------------|--------------------------------------------------------|
   1      06-Mar-2019   Initial release
   2      24-Feb-2022   Updated initialisation to include rotation offset
*******************************************************************************/

#include "StrawberryString.h"

using namespace SweetMaker;

/*
 * On the Nano the led strip is mounted on three pins of the ICSP 
 * socket. The MPU6050 is connected piggyback on the Nano.
 */
#ifdef ARDUINO_ARCH_AVR
const static uint8_t ms_scl = 19;    // A5+
const static uint8_t ms_sda = 18;    // A4

const static uint8_t ledStripSigPin = 11;
#endif

#ifdef ARDUINO_ARCH_ESP32
const static uint8_t ms_scl = 21;    // A5+
const static uint8_t ms_sda = 22;    // A4
const static uint8_t ms_5v_pin = 17; // A3
const static uint8_t ms_0v_pin = 16; // A2

const static uint8_t ledStripSigPin = 27;
#endif

/*
 * StrawberryString instantiation
 */
StrawberryString::StrawberryString()
{
	/*
	 * Set Digital Pin output for powering the MPU6050 on the ESP32
	 */
#ifdef ARDUINO_ARCH_ESP32
  pinMode(ms_5v_pin, OUTPUT);
	digitalWrite(ms_5v_pin, HIGH);
	pinMode(ms_0v_pin, OUTPUT);
	digitalWrite(ms_0v_pin, LOW);
#endif

  /*
   * SweetMaker user event handlers
   */
	userEventHandlerCallback = NULL;
	userEventHandlerObject = NULL;

  /*
   * Register this StrawberryString as eventHandler for
   * SweetMaker events - at present StrawberryString does 
   * nothing with these events other than forward to the user
   */
	EventMngr::getMngr()->configCallBack(this);

  /*
   * Configure LED string signal pin.
   */
	pinMode(ledStripSigPin, OUTPUT);
	digitalWrite(ledStripSigPin, LOW);

  /*
   * Set some initial values for good measure
   */
	ledStrip[0] = 0x00BFFF;
	ledStrip[1] = 0xff1493;
	ledStrip[2] = 0x7cfc00;
	ledStrip[3] = 0xffd700;
	ledStrip[4] = 0xf0f8ff;
}

/*
 * init - following instantiation this function performs additional setup.
 *        In particular this includes recovering MotionSensor calibration and
 *        rotation offset values from EEPROM. This avoids trying to manage 
 *        deployment specific values in code.
 */
int StrawberryString::init()
{
  /*
   * Create a LED strip driver dependent on Arduino architecture
   */
#ifdef ARDUINO_ARCH_AVR
	ledStripDriver = new AVRLedStripDriver(ledStripSigPin);
#elif ARDUINO_ARCH_ESP32
	ledStripDriver = new Esp32LedStripDriver(ledStripSigPin, 5);
#endif

	/*
	* Read EEPROM Config then initialise motionSensor
	*/
  EEPROM_DATA eeprom_data;
  if (this->getEepromData(&eeprom_data) == 0)	{
		printEepromData(&eeprom_data);
    while (motionSensor.init(&eeprom_data.ms_calibration) != 0)
			// Try Again
			;
    motionSensor.setOffsetRotation(&eeprom_data.ms_offsetRotation);
	}
	else {
		Serial.println("Resetting EEPROM Data");
		resetEepromData();
		while (motionSensor.init() != 0)
			// Try Again
			;
	}

  /*
   * The TimerTickMngt needs a kick to make it start.
   */
	TimerTickMngt::getTimerMngt()->update(0);
	
  /*
   * Record the current time
   */
	this->lastUpdateTime_ms = millis();
	return(0);
}

/*
 * configEventHandlerCallback - user registers a callback function for handling
 *                              SweetMaker events;
 */
void StrawberryString::configEventHandlerCallback(EventHandler callbackFunction)
{
	this->userEventHandlerCallback = callbackFunction;
}

/*
 * configEventHandlerCallback - user registers a callback object for handling
 *                              SweetMaker events;
 */
void StrawberryString::configEventHandlerCallback(IEventHandler * callbackObject)
{
	this->userEventHandlerObject = callbackObject;
}

/*
 * Calibrate Motion Sensor and stores values in EEPROM. Motion Sensor should remain
 * stationary and horizontal (Z facing upwards) until complete.
 *
 * The calibration values will be automatically retrieved and applied next time.
 */
int StrawberryString::recalibrateMotionSensor()
{
  EEPROM_DATA eeprom_data;
  getEepromData(&eeprom_data);
  printEepromData(&eeprom_data);

  motionSensor.runSelfCalibrate(&eeprom_data.ms_calibration);

  printEepromData(&eeprom_data);
  setEepromData(&eeprom_data);
  return (0);
}


/*
 * configOffsetRotation - this autoLevels the MPU6050 and stores the offset in EEPROM
 */
void StrawberryString::configOffsetRotation()
{
  motionSensor.clearOffsetRotation();
  RotationQuaternion_16384 offsetQ;

  Quaternion_16384 gq;
  motionSensor.rotQuat.getGravity(&gq);

  Quaternion_16384 zAxis(0, 0, 0, 16384);
  offsetQ.findOffsetRotation(&zAxis, &gq);

  configOffsetRotation(&offsetQ);
}

/*
 * configOffsetRotation - this autolevels the MPU6050 and also applies a rotation
 *                        about the Z axis as offset. Stores in EEPROM.
 */
void StrawberryString::configOffsetRotation(float rotation_z)
{
  motionSensor.clearOffsetRotation();
  RotationQuaternion_16384 offsetQ;

  Quaternion_16384 gq;
  motionSensor.rotQuat.getGravity(&gq);

  Quaternion_16384 zAxis(0, 0, 0, 16384);
  offsetQ.findOffsetRotation(&zAxis, &gq);

  RotationQuaternion_16384 rotZ(rotation_z, 0, 0, 16384);
  offsetQ.crossProduct(&rotZ);

  configOffsetRotation(&offsetQ);
}

/*
 * configOffsetRotation - Applies an arbitrary offset rotation. Stores in EEPROM
 */
void StrawberryString::configOffsetRotation(RotationQuaternion_16384 *rotQuat)
{
	int retVal;
	EEPROM_DATA data;

	retVal = getEepromData(&data);
	if (retVal != 0) {
		Serial.println("Resetting EEPROM");
		resetEepromData();
		getEepromData(&data);
	}

	printEepromData(&data);

	data.ms_offsetRotation.r = rotQuat->r;
	data.ms_offsetRotation.x = rotQuat->x;
	data.ms_offsetRotation.y = rotQuat->y;
	data.ms_offsetRotation.z = rotQuat->z;

	printEepromData(&data);
	setEepromData(&data);

	motionSensor.setOffsetRotation(rotQuat);
}

/*
 * Called by user in main loop(). This calculates
 * elapsed time and calls the SweetMaker framework 
 * update function.
 *
 * Also updates the LED strip being careful to disable interrupts
 * as it does so.
 */
void StrawberryString::update()
{
	unsigned long thisTime_ms = millis();
	unsigned long elapsedTime_ms = thisTime_ms - lastUpdateTime_ms;

	AutoUpdateMngr::getUpdater()->update(elapsedTime_ms);

	cli();
	this->ledStripDriver->show(ledStrip, num_lights);
	sei();

	lastUpdateTime_ms = thisTime_ms;
}

/*
 * updateDelay - similar to update but promises not to return for
 *               a period of time in milliseconds. This is 
 *               like a blocking function and allows synchronous 
 *               coding. Note: event callbacks will continue to 
 *               occur from the SweetMaker framework to the user.
 */
void StrawberryString::updateDelay(uint16_t duration_ms)
{
	unsigned long finishTime_ms = millis() + duration_ms;

	while (millis() < finishTime_ms)
		this->update();
}

/*
 * handleEvent - receives events from the SweetMaker framework and forwards them to the User
 */
void StrawberryString::handleEvent(uint16_t eventId, uint8_t sourceInst, uint16_t eventInfo)
{
	if (userEventHandlerCallback)
		userEventHandlerCallback(eventId, sourceInst, eventInfo);

	if (userEventHandlerObject)
		userEventHandlerObject->handleEvent(eventId, sourceInst, eventInfo);
}

/*
 * Reads EEPROM data and validates CRC. 
 */
int StrawberryString::getEepromData(EEPROM_DATA * data)
{
	uint16_t storedCrc;
	uint16_t calculatedCrc;
	EepromUtility::EepromReader eepromReader(eeprom_data_offset, eeprom_data_length);
	/* Calculate and check CRC */
	calculatedCrc = EepromUtility::calculateCrc(StrawberryString::eeprom_data_offset, StrawberryString::eeprom_data_length - 2);

	data->lightStringType = eepromReader.readU8();

	data->ms_offsetRotation.r = eepromReader.readS16();
	data->ms_offsetRotation.x = eepromReader.readS16();
	data->ms_offsetRotation.y = eepromReader.readS16();
	data->ms_offsetRotation.z = eepromReader.readS16();

	data->ms_calibration.accelXoffset = eepromReader.readS16();
	data->ms_calibration.accelYoffset = eepromReader.readS16();
	data->ms_calibration.accelZoffset = eepromReader.readS16();
	data->ms_calibration.gyroXoffset = eepromReader.readS16();
	data->ms_calibration.gyroYoffset = eepromReader.readS16();
	data->ms_calibration.gyroZoffset = eepromReader.readS16();

	data->hardwareVersion = eepromReader.readU8();
	data->softwareVersion = eepromReader.readU8();
	data->serialNumber = eepromReader.readU32();

	storedCrc = eepromReader.readU16();
	Serial.print("Recovered CRC:");
	Serial.println(storedCrc);

	if (storedCrc != calculatedCrc)
		return (-1);
	return (0);
}

/*
 * Sets EEPROM data
 */
int StrawberryString::setEepromData(EEPROM_DATA * data)
{
	uint16_t crc_value;
	EepromUtility::EepromWriter eepromWriter(StrawberryString::eeprom_data_offset, eeprom_data_length);

	/*
	 * LED String Type
	 */
	eepromWriter.writeU8(1);

	/*
	 * Motion Sensor Rotation Offset
	 */
	eepromWriter.writeS16(data->ms_offsetRotation.r);
	eepromWriter.writeS16(data->ms_offsetRotation.x);
	eepromWriter.writeS16(data->ms_offsetRotation.y);
	eepromWriter.writeS16(data->ms_offsetRotation.z);

	/*
	 * Motion Sensor Calibration
	 */
	eepromWriter.writeS16(data->ms_calibration.accelXoffset);
	eepromWriter.writeS16(data->ms_calibration.accelYoffset);
	eepromWriter.writeS16(data->ms_calibration.accelZoffset);

	eepromWriter.writeS16(data->ms_calibration.gyroXoffset);
	eepromWriter.writeS16(data->ms_calibration.gyroYoffset);
	eepromWriter.writeS16(data->ms_calibration.gyroZoffset);

	/*
	 * Hardware and Software and Serial Number!
	 */
	eepromWriter.writeU8(data->hardwareVersion);
	eepromWriter.writeU8(data->softwareVersion);
	eepromWriter.writeU32(data->serialNumber);

	/*
	 * CRC value
	 */
	crc_value = EepromUtility::calculateCrc(StrawberryString::eeprom_data_offset, StrawberryString::eeprom_data_length - 2);
	eepromWriter.writeU16(crc_value);
	return 0;
}

/*
 * Resets EEPROM data
 */
int StrawberryString::resetEepromData()
{
	int retVal;
	EEPROM_DATA data;
	memset(&data, 0, sizeof(EEPROM_DATA));
	data.hardwareVersion = 1;
	data.softwareVersion = 1;
	data.lightStringType = 1;
	data.serialNumber = 1;
	
	/*
	 * MS calibration 
	 */
	data.ms_calibration.accelXoffset = 0;
	data.ms_calibration.accelYoffset = 0;
	data.ms_calibration.accelZoffset = 0;
	data.ms_calibration.gyroXoffset = 0;
	data.ms_calibration.gyroYoffset = 0;
	data.ms_calibration.gyroZoffset = 0;

	/*
	 * Rotation of zero
	 */
	data.ms_offsetRotation.r = 1; 
	data.ms_offsetRotation.x = 0;
	data.ms_offsetRotation.y = 0;
	data.ms_offsetRotation.z = 1;
	data.ms_offsetRotation.normalize();

	retVal = setEepromData(&data);
	return (retVal);
}

/*
 * Prints EEPROM data
 */
void StrawberryString::printEepromData(EEPROM_DATA * data)
{
//	Serial.print("Hardware Version:\t");
//	Serial.println(data->hardwareVersion);
//	Serial.print("Software Version:\t");
//	Serial.println(data->softwareVersion);
//	Serial.print(data->lightStringType);
//	Serial.print(data->serialNumber);

	Serial.print("Motion Sensor Calibration:\t");
	Serial.print(data->ms_calibration.accelXoffset);
	Serial.print("\t");
	Serial.print(data->ms_calibration.accelYoffset);
	Serial.print("\t");
	Serial.print(data->ms_calibration.accelZoffset);
	Serial.print("\t");
	Serial.print(data->ms_calibration.gyroXoffset);
	Serial.print("\t");
	Serial.print(data->ms_calibration.gyroYoffset);
	Serial.print("\t");
	Serial.println(data->ms_calibration.gyroZoffset);

	Serial.print("Motion Sensor Rotation Offset:\t");
	Serial.print(data->ms_offsetRotation.r);
	Serial.print("\t");
	Serial.print(data->ms_offsetRotation.x);
	Serial.print("\t");
	Serial.print(data->ms_offsetRotation.y);
	Serial.print("\t");
	Serial.print(data->ms_offsetRotation.z);
	Serial.println("\t");
}

/*
 * Retrieves and Prints EEPROM Data
 */
void StrawberryString::printEepromData()
{
	EEPROM_DATA data;
	getEepromData(&data);
	printEepromData(&data);
}


