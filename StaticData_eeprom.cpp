/*******************************************************************************
StaticData_eeprom.h Function for accessing Strawberry String EEPROM Static Data 

Copyright(C) 2024  Howard James May

This file is part of the SweetMaker project

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
   1      20-Jun-2024   Initial Version
*******************************************************************************/
#ifndef ARDUINO_ESP32C3_DEV

#include "StaticData.h"

static const uint16_t EEPROM_DATA_LEN_V2 = 97; // includes 2 bytes for crc
static const uint16_t EEPROM_DATA_LEN_V1 = 29; // includes 2 bytes for crc
static const uint16_t EEPROM_SIZE = 0x400;
static const uint8_t MAX_INSTANCE_NAME_LEN = 64;
static const uint8_t DEFAULT_NUM_LIGHTS = 5;

/*
 * Reads EEPROM data and validates CRC.
 */
int _getStaticData(StrawberryString::STATIC_DATA* data)
{
	uint16_t storedCrc;
	uint16_t calculatedCrc;
	bool crc_validated = false;
	EepromUtility::EepromReader* eepromReader;

	storedCrc = EepromUtility::EepromReader::readU16(EepromUtility::maxEepromLen - 2);
	Serial.print("Recovered CRC: ");
	Serial.println(storedCrc);

	/* Calculate and check CRC V2*/
	calculatedCrc = EepromUtility::calculateCrc(EepromUtility::maxEepromLen - EEPROM_DATA_LEN_V2, EEPROM_DATA_LEN_V2 - 2);
	if (calculatedCrc == storedCrc) {
		Serial.println("V2 CRC Check PASSED");

		crc_validated = true;
		eepromReader = new EepromUtility::EepromReader(EepromUtility::maxEepromLen - EEPROM_DATA_LEN_V2, EEPROM_DATA_LEN_V2 - 2);

		/* Read fields introduced in V2 */
		data->numLights = eepromReader->readU8();
		data->ms_calibration.accelXFineGain = eepromReader->readU8();
		data->ms_calibration.accelYFineGain = eepromReader->readU8();
		data->ms_calibration.accelZFineGain = eepromReader->readU8();
		eepromReader->memcpy((void*)data->instanceName, MAX_INSTANCE_NAME_LEN);
	}
	else {
		Serial.println("V2 CRC Check FAILED");
	}

	if (!crc_validated) {
		/* V2 CRC failed - try V1*/
		calculatedCrc = EepromUtility::calculateCrc(EepromUtility::maxEepromLen - EEPROM_DATA_LEN_V1, EEPROM_DATA_LEN_V1 - 2);
		if (calculatedCrc != storedCrc) {
			Serial.println("V1 CRC Check FAILED");
			return(-1);
		}
		Serial.println("V1 CRC Check PASSED");
		eepromReader = new EepromUtility::EepromReader(EepromUtility::maxEepromLen - EEPROM_DATA_LEN_V1, EEPROM_DATA_LEN_V1 - 2);

		/* Default the fields introduced in V2 */
		data->numLights = DEFAULT_NUM_LIGHTS;
		//		data->ms_calibration.accelXFineGain = motionSensor.mpu6050.getXFineGain();
		//		data->ms_calibration.accelYFineGain = motionSensor.mpu6050.getYFineGain();
		//		data->ms_calibration.accelZFineGain = motionSensor.mpu6050.getZFineGain();
		memset(data->instanceName, 0, MAX_INSTANCE_NAME_LEN);
		strcpy(data->instanceName, "SweetMaker-StrawberryString");
	}

	data->lightStringType = eepromReader->readU8();

	data->ms_offsetRotation.r = eepromReader->readS16();
	data->ms_offsetRotation.x = eepromReader->readS16();
	data->ms_offsetRotation.y = eepromReader->readS16();
	data->ms_offsetRotation.z = eepromReader->readS16();

	data->ms_calibration.accelXoffset = eepromReader->readS16();
	data->ms_calibration.accelYoffset = eepromReader->readS16();
	data->ms_calibration.accelZoffset = eepromReader->readS16();
	data->ms_calibration.gyroXoffset = eepromReader->readS16();
	data->ms_calibration.gyroYoffset = eepromReader->readS16();
	data->ms_calibration.gyroZoffset = eepromReader->readS16();

	data->hardwareVersion = eepromReader->readU8();
	data->softwareVersion = eepromReader->readU8();
	data->serialNumber = eepromReader->readU32();

	delete eepromReader;

	return (0);
}

/*
 * Sets EEPROM data - always sets V2
 */
int _setStaticData(StrawberryString::STATIC_DATA* data)
{
	uint16_t crc_value;
	EepromUtility::EepromWriter eepromWriter(EEPROM_SIZE - EEPROM_DATA_LEN_V1, EEPROM_DATA_LEN_V1);
	Serial.println("setEepromData");

	//	eepromWriter.writeU8(data->numLights);
	//	eepromWriter.writeU8(data->ms_calibration.accelXFineGain);
	//	eepromWriter.writeU8(data->ms_calibration.accelYFineGain);
	//	eepromWriter.writeU8(data->ms_calibration.accelZFineGain);
	//	eepromWriter.memcpy(data->instanceName, MAX_INSTANCE_NAME_LEN);

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
	crc_value = EepromUtility::calculateCrc(EEPROM_SIZE - EEPROM_DATA_LEN_V1, EEPROM_DATA_LEN_V1 - 2);
	Serial.println(crc_value);
	eepromWriter.writeU16(crc_value);
	return 0;
}

#endif