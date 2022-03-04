// The setup() function runs once each time the micro-controller starts
#include <Wire.h>
#include <SweetMaker.h>
#include <StrawberryString.h>
#include <MotionSensor.h>

using namespace SweetMaker;

StrawberryString myStrStr;

void myEventHandlerCallback(uint16_t event, uint8_t source, uint16_t value);
void handleSerialInput();

enum RunMode { IDLE, TRICOLOUR } runMode;

void setup()
{
  Serial.println("Quaternion Tutorial");
  myStrStr.init();
  myStrStr.configEventHandlerCallback(myEventHandlerCallback);
  Serial.println("Setup complete");
  runMode = IDLE;
}

// Add the main program code into the continuous loop() function
void loop()
{
  handleSerialInput();
  myStrStr.update();
}


void myEventHandlerCallback(uint16_t event, uint8_t source, uint16_t value)
{
  switch (event)
  {
  case MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY:
    if (runMode == TRICOLOUR)
    {
      Quaternion_16384 gq;
      myStrStr.motionSensor.rotQuat.getGravity(&gq);
      gq.printQ();

      Quaternion_16384 red_v = { 0, 0, 0, 0x4000 };
      Quaternion_16384 green_v = { 0,9459,9459,9459 };
      Quaternion_16384 blue_v = { 0,9459,-9459,9459 };

      int16_t red = red_v.dotProduct(&gq);
      int16_t green = green_v.dotProduct(&gq);
      int16_t blue = blue_v.dotProduct(&gq);

      red = Quaternion_16384::asr(red, 6);
      green = Quaternion_16384::asr(green, 6);
      blue = Quaternion_16384::asr(blue, 6);

      if (red < 0) red = 0;
      if (red > 255) red = 255;
      if (green < 0) green = 0;
      if (green > 255) green = 255;
      if (blue < 0) blue = 0;
      if (blue > 255) blue = 255;

      Serial.print(red); Serial.print("\t");
      Serial.print(green); Serial.print("\t");
      Serial.print(blue); Serial.println();

      for (int i = 0; i < 5; i++) {
        myStrStr.ledStrip[i].setColour(red, green, blue);
      }
    }
    break;

  default:
    break;
  }

}


void handleSerialInput()
{
  if (!Serial.available())
    return;

  char inChar = Serial.read();
  switch (inChar)
  {
  case 'r':
    Serial.println("ClearOffset");
    myStrStr.motionSensor.clearOffsetRotation();
    break;

  case 'a':
  {
    int16_t num1 = myStrStr.motionSensor.rotQuat.asr(0x12345678, 14);
    Serial.println(num1);
    int16_t num2 = myStrStr.motionSensor.rotQuat.asr(-0x12345678, 14);
    Serial.println(num2);
  }
  break;

  case 'p':
    myStrStr.motionSensor.rotQuat.printQ();
    break;

  case 's':
  {
    Serial.println("Sine: Yaw, Pitch, Roll");
    unsigned long start_us = micros();
    int16_t roll = myStrStr.motionSensor.rotQuat.getSinRotX();
    int16_t pitch = myStrStr.motionSensor.rotQuat.getSinRotY();
    int16_t yaw = myStrStr.motionSensor.rotQuat.getSinRotZ();
    unsigned long stop_us = micros();

    Serial.print(stop_us - start_us); Serial.print(" ");
    Serial.print(yaw); Serial.print(" ");
    Serial.print(pitch); Serial.print(" ");
    Serial.print(roll); Serial.println();
  }
  break;

  case 'y':
  {
    Serial.println("Angle: Yaw, Pitch, Roll");
    unsigned long start_us = micros();
    int16_t roll = myStrStr.motionSensor.rotQuat.getRotX();
    int16_t pitch = myStrStr.motionSensor.rotQuat.getRotY();
    int16_t yaw = myStrStr.motionSensor.rotQuat.getRotZ();
    unsigned long stop_us = micros();

    Serial.print(stop_us - start_us); Serial.print(" ");
    Serial.print(yaw); Serial.print(" ");
    Serial.print(pitch); Serial.print(" ");
    Serial.print(roll); Serial.println();

  }
  break;

  case 'v':
  {
    Quaternion_16384 z = { 0,0,0,16384 };
    Quaternion_16384 v_forwards = { 0,11586,0,11586 };
    Quaternion_16384 v_backwards = { 0,-11586,0,11586 };

    Serial.print("Rotation Q: ");  myStrStr.motionSensor.rotQuat.printQ();

    float z_angle = (float)myStrStr.motionSensor.rotQuat.getRotZ();
    Serial.print("z_angle raw: "); Serial.println(z_angle);

    z_angle = z_angle * 90 / 16384;
    Serial.print("z_angle degree: "); Serial.println(z_angle);

    RotationQuaternion_16384 z_offset = RotationQuaternion_16384(z_angle, 0, 0, 16384);
    z_offset.conjugate();
    Serial.print("Z_offset: "); z_offset.printQ();

    z_angle = (float)z_offset.getRotZ();
    Serial.print("z_angle raw offset: "); Serial.println(z_angle);

    myStrStr.motionSensor.rotQuat.crossProduct(&z_offset);

    Serial.print("Rotation Q Offset: ");  myStrStr.motionSensor.rotQuat.printQ();
    z_angle = (float)myStrStr.motionSensor.rotQuat.getRotZ();
    Serial.print("z_angle should be zero: "); Serial.println(z_angle);

    Serial.print("Rotated Z: ");
    myStrStr.motionSensor.rotQuat.rotate(&z);

    z.printQ();
    Serial.print("Rotation about Y: "); Serial.println(myStrStr.motionSensor.rotQuat.getRotY());

    int16_t forwards = z.dotProduct(&v_forwards);

    Serial.print("Dot Product: RotatedZ, Z: ");
    Serial.println(forwards);
  }
  break;

  case 'u':
  {
    Serial.print("Rotation Q: ");  myStrStr.motionSensor.rotQuat.printQ();

    int32_t vx = myStrStr.motionSensor.rotQuat.getSinRotY();
    int32_t vy = myStrStr.motionSensor.rotQuat.getSinRotX();
    int32_t vz = myStrStr.motionSensor.rotQuat.getCosRotZ();

    Quaternion_16384 newZPos = { 0, -vx, -vy, vz };
    newZPos.printQ();

    Quaternion_16384 v_forwards = { 0,11586,0,11586 };
    Quaternion_16384 v_backwards = { 0,-11586,0,11586 };
    int16_t forwards = v_forwards.dotProduct(&newZPos);
    int16_t backwards = v_backwards.dotProduct(&newZPos);

    Serial.print("Dot Product: Rotated Z, Z: ");
    Serial.print(forwards); Serial.print(":"); Serial.println(backwards);
  }
  break;

  case 'g':
  {
    Serial.print("Rotation Q: ");  myStrStr.motionSensor.rotQuat.printQ();
    Quaternion_16384 gq;
    myStrStr.motionSensor.rotQuat.getGravity(&gq);
    gq.printQ();

    Quaternion_16384 red_v = { 0, 0, 0, 0x4000 };
    Quaternion_16384 green_v = { 0,9459,9459,9459 };
    Quaternion_16384 blue_v = { 0,9459,-9459,9459 };

    int16_t red = red_v.dotProduct(&gq);
    int16_t green = green_v.dotProduct(&gq);
    int16_t blue = blue_v.dotProduct(&gq);

    red = Quaternion_16384::asr(red, 6);
    green = Quaternion_16384::asr(green, 6);
    blue = Quaternion_16384::asr(blue, 6);

    if (red < 0) red = 0;
    if (red > 255) red = 255;
    if (green < 0) green = 0;
    if (green > 255) green = 255;
    if (blue < 0) blue = 0;
    if (blue > 255) blue = 255;

    Serial.print(red); Serial.print("\t");
    Serial.print(green); Serial.print("\t");
    Serial.print(blue); Serial.println();

    for (int i = 0; i < 5; i++) {
      myStrStr.ledStrip[i].setColour(red, green, blue);
    }
  }
  break;

  case 't':
    if (runMode == TRICOLOUR)
      runMode = IDLE;
    else
      runMode = TRICOLOUR;
    break;

    /* This autogenerates an offset quaternion */
  case 'o':
  {
    Serial.println("Setting offset");
    myStrStr.configOffsetRotation(45);

    Quaternion_16384 gq;
    myStrStr.motionSensor.rotQuat.getGravity(&gq);
    gq.printQ();
  }
  break;

  default:
    break;
  }
}


