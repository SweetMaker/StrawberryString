/*
https://docs.espressif.com/projects/arduino-esp32/en/latest/api/preferences.html

*/


#define ARDUINO_ESP32C3_DEV
#ifdef ARDUINO_ESP32C3_DEV
#include <Preferences.h>
#include "StaticData.h"

#define RW_MODE false
#define RO_MODE true

int _setStaticData(StrawberryString::STATIC_DATA* data)
{
    Serial.println("Set Static Data Esp32");

    Preferences preferences;

    if (!preferences.begin("SweetMaker", RW_MODE)) {
        Serial.println("Failed to open namespace");
        return (-1);
    }

    preferences.clear();
    preferences.putUChar("hardwareVersion", data->hardwareVersion);
    preferences.putString("instanceName", data->instanceName);
    preferences.putUChar("lightStringType", data->lightStringType);
    preferences.putUChar("numLights", data->numLights);
    preferences.putUInt("serialNumber", data->serialNumber);
    preferences.putUChar("softwareVersion", data->softwareVersion);

    preferences.putShort("offsetRotationr", data->ms_offsetRotation.r);
    preferences.putShort("offsetRotationx", data->ms_offsetRotation.x);
    preferences.putShort("offsetRotationy", data->ms_offsetRotation.y);
    preferences.putShort("offsetRotationz", data->ms_offsetRotation.z);

    preferences.putShort("accelXoffset", data->ms_calibration.accelXoffset);
    preferences.putShort("accelYoffset", data->ms_calibration.accelYoffset);
    preferences.putShort("accelZoffset", data->ms_calibration.accelZoffset);
    preferences.putShort("gyroXoffset", data->ms_calibration.gyroXoffset);
    preferences.putShort("gyroYoffset", data->ms_calibration.gyroYoffset);
    preferences.putShort("gyroZoffset", data->ms_calibration.gyroZoffset);

    preferences.putChar("accelXFineGain", data->ms_calibration.accelXFineGain);
    preferences.putChar("accelYFineGain", data->ms_calibration.accelYFineGain);
    preferences.putChar("accelZFineGain", data->ms_calibration.accelZFineGain);

    preferences.end();
    return (0);
}

int _getStaticData(StrawberryString::STATIC_DATA* data)
{
    Serial.println("Get Static Data Esp32");

    Preferences preferences;
    if (!preferences.begin("SweetMaker", RO_MODE)) {
        Serial.println("Failed to open namespace");
        return (-1);
    }

    bool staticDataExists = preferences.isKey("hardwareVersion");
    if (!staticDataExists) {
        Serial.print("Static Data doesn't exist: freeEntries: ");
        Serial.println(preferences.freeEntries());
        preferences.end();
        return (-1);
    }

    try {
        data->hardwareVersion = preferences.getUChar("hardwareVersion");
        preferences.getString("instanceName", data->instanceName, StrawberryString::MAX_INSTANCE_NAME_LEN);
        data->lightStringType = preferences.getUChar("lightStringType");
        data->numLights = preferences.getUChar("numLights");
        data->serialNumber = preferences.getUInt("serialNumber");
        data->softwareVersion = preferences.getUChar("softwareVersion");

        data->ms_offsetRotation.r = preferences.getShort("offsetRotationr");
        data->ms_offsetRotation.x = preferences.getShort("offsetRotationx");
        data->ms_offsetRotation.y = preferences.getShort("offsetRotationy");
        data->ms_offsetRotation.z = preferences.getShort("offsetRotationz");

        data->ms_calibration.accelXoffset = preferences.getShort("accelXoffset");
        data->ms_calibration.accelYoffset = preferences.getShort("accelYoffset");
        data->ms_calibration.accelZoffset = preferences.getShort("accelZoffset");
        data->ms_calibration.gyroXoffset = preferences.getShort("gyroXoffset");
        data->ms_calibration.gyroYoffset = preferences.getShort("gyroYoffset");
        data->ms_calibration.gyroZoffset = preferences.getShort("gyroZoffset");

        data->ms_calibration.accelXFineGain = MotionSensor::GAIN_UNDEFINED;
        if(preferences.isKey("accelXFineGain"))
          data->ms_calibration.accelXFineGain = preferences.getChar("accelXFineGain");

        data->ms_calibration.accelYFineGain = MotionSensor::GAIN_UNDEFINED;
        if (preferences.isKey("accelYFineGain"))
            data->ms_calibration.accelYFineGain = preferences.getChar("accelYFineGain");

        data->ms_calibration.accelZFineGain = MotionSensor::GAIN_UNDEFINED;
        if (preferences.isKey("accelZFineGain"))
            data->ms_calibration.accelZFineGain = preferences.getChar("accelZFineGain");
    }
    catch (...) {
        Serial.println("Get Static Data Esp32 Exception!!");
        preferences.end();
        return (-1);
    }

    preferences.end();
    return 0;
}

#endif
