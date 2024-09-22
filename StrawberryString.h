 /*******************************************************************************
StrawberryString.h Class for controlling a StrawberryString Device

Copyright(C) 2019-2022  Howard James May

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
   1      06-Mar-2019   Initial release
--------|-------------|--------------------------------------------------------|
   2      24-Feb-2022   Updated initialisation to include rotation offset
*******************************************************************************/


#ifndef __STRAWBERRY_STRING_H__
#define __STRAWBERRY_STRING_H__

#include <Arduino.h>
#include "EEPROM.h"

#include <Wire.h>

#include <SweetMaker.h>
#include <MotionSensor.h>

#include "EepromUtility.h"

namespace SweetMaker
{

	/*
	* Enable this for Serial Debug
	*/
	//#define SWEET_MAKER_DEBUG

	class StrawberryString : public IEventHandler
	{

		/*
		* These class methods provide the public interface to StrawberryString which you
		* use in your program.
		*/
	public:
		/*
		* Use this to create and setup StrawberryString once at startup
		*/
		StrawberryString();

		/*
		* StrawberryString uses a part of the EEPROM memory located at the end of the address space.
		*/
		static const uint8_t MAX_INSTANCE_NAME_LEN = 64;
		static const uint8_t DEFAULT_NUM_LIGHTS = 5;

		typedef struct staticData {
			uint8_t hardwareVersion;
			uint8_t softwareVersion;
			uint32_t serialNumber;

			char instanceName[MAX_INSTANCE_NAME_LEN];
			
			uint8_t lightStringType;
			uint8_t numLights;

			RotationQuaternion_16384 ms_offsetRotation;
			MotionSensor::CALIBRATION ms_calibration;
		}STATIC_DATA;

		void configEventHandlerCallback(EventHandler callbackFunction);
		void configEventHandlerCallback(IEventHandler * callbackObject);

    /*
     * The MPU6050 mounting may not be level - this provides an offset to make 
	 * the sensor appear as if it is currently flat. This is stored in EEPROM and reused 
	 * on startup 
	 * 
	 * Note: no offset for rotation about vertical is made.
     */
    void autoLevelAndStore();

	int init();
		/*
		* Call one of these frequently to allow StrawberryString to work properly
		*/
		void update();
		void updateDelay(uint16_t duration_ms); // wait for 'duration' milliseconds.

    /*
     * calibration is essential for the proper working of the MPU6050
     * the result is stored in EEPROM and so this can be a one time task
     */
		int recalibrateMotionSensor();

		int getStaticData(STATIC_DATA * data);
		int setStaticData(STATIC_DATA * data);
		int resetStaticData();

		void printStaticData(STATIC_DATA * data);
		void printStaticData();

		uint8_t num_lights = DEFAULT_NUM_LIGHTS;

    /*
     * the motionSensor - has it's own API for accessing readings
     */
		MotionSensor motionSensor;

    /*
     * The leds - an array of RGBs
     */
		ColourRGB * ledStrip;
		ILedStripDriver * ledStripDriver;

	protected:
		unsigned long lastUpdateTime_ms;

	private:

		/*
		* These class methods are only for use by StrawberryString itself
		*/
		uint8_t hardwareVersion;

		EventHandler userEventHandlerCallback;
		IEventHandler * userEventHandlerObject;

		void handleEvent(uint16_t eventId, uint8_t sourceInst, uint16_t eventInfo);
		void storeOffsetRotation(RotationQuaternion_16384* rotQuat);

	};
}

#endif


