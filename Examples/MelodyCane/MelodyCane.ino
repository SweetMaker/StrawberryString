#include <MotionSensor.h>
#include <SweetMaker.h>
#include <FastLED.h>
#include <Wire.h>
#include <EEPROM.h>
#include "StrawberryString.h"
#include "MelodyCane.h"
#include "Colour.h"
#include "TuneLib.h"

using namespace SweetMaker;

/*
 * ToneGen: This is a special tone generator for this project
 *          It generates the requested frequency and the next two lower
 *          octaves of the same note. Output across two pins which
 *          maintain opposite polarity to achieve higher volume.
 *
 *          The generator offers the ability to find the time till the
 *          next interrupt is expected. Also the ability to inhibit interrupts.
 *          This clears the interrupt register causing interrupts to be skipped
 *          Rather than delayed.
 */
volatile uint32_t last_interrupt_us;

MelodyCane myMelodyCane;

MelodyCane::MelodyCane()
{
}


void setup()
{
	Serial.begin(112500);
	Serial.println("MelodyCane");

	myMelodyCane.setup();
}

void loop()
{
	myMelodyCane.update();
}

/*
 * Setup - Setup Melody Cane
 */
void MelodyCane::setup()
{
	strStr.configEventHandlerCallback(&evntPrePro);
	evntPrePro.configEventHandler(this);

	toneGen.setup();

	piezzo.configEventHandler(EventMngr::getMngr(), 0);
	piezzo.configToneGen(&myMelodyCane.toneGen);
	piezzo.configListeningPin(TONE_GEN_PIN_1B);

	tunePlayer.configToneGen(&toneGen);
	tunePlayer.configEventHandler(this);

	pinMode(TONE_GEN_PIN_1A, OUTPUT);
	pinMode(TONE_GEN_PIN_1B, OUTPUT);
	pinMode(TONE_GEN_PIN_2A, OUTPUT);
	pinMode(TONE_GEN_PIN_2B, OUTPUT);
	pinMode(TONE_GEN_PIN_3A, OUTPUT);
	pinMode(TONE_GEN_PIN_3B, OUTPUT);

	digitalWrite(TONE_GEN_PIN_1A, LOW);

	strStr.init();

	lastUpdateTime_ms = millis();


	ledRefreshTime_us = strStr.ledStringDriver.getRefreshTime_us(
		strStr.num_lights
	);

	state = STATE_STARTUP;

	tunePlayer.playTune(imperialMarchNotes, IMPERIAL_MARCH_NOTE_COUNT, IMPERIAL_MARCH_TEMPO);
}

/*
 * update: Calculates elapsed time and updates elements.
 *         This tries to minimise the impact of the led string update
 *         time on the piezzo driver.
 */
void MelodyCane::update(void)
{
	unsigned long thisTime_ms = millis();
	unsigned long elapsedTime_ms = thisTime_ms - myMelodyCane.lastUpdateTime_ms;

	AutoUpdateMngr::getUpdater()->update(elapsedTime_ms);

	if ((thisTime_ms - myMelodyCane.strStr.ledStringDriver.lastUpdateTime_ms) > 50)
	{
		myMelodyCane.toneGen.mute(true);
		myMelodyCane.strStr.ledStringDriver.show(myMelodyCane.strStr.ledStrip, myMelodyCane.strStr.num_lights);
		myMelodyCane.toneGen.mute(false);
	}
	else if (toneGen.timeTillNextInterrupt_us() > ledRefreshTime_us)
	{
		strStr.ledStringDriver.show(strStr.ledStrip, strStr.num_lights);
	}

	lastUpdateTime_ms = thisTime_ms;
}

/*
 * handleEvent
 */
void MelodyCane::handleEvent(uint16_t eventId, uint8_t eventRef, uint16_t eventInfo)
{
	switch (eventId)
	{
	case TimerTickMngt::TIMER_TICK_S:
		if (melodyMode == 1)
		{
			Serial.print(evntPrePro.rotY_16384);
			Serial.print("\t");
			Serial.print(stepper.currentStep);
			Serial.println();
		}
		break;

	case TimerTickMngt::TIMER_TICK_10S:
		//		PerfMon::getPerfMon()->print();
		//		PerfMon::getPerfMon()->reset();
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

	case EVENT_MUTE:
		mute = !mute;
		piezzo.stop();
		break;

	case TimerTickMngt::TIMER_TICK_UPDATE: // Generated ten times a second
	case TimerTickMngt::TIMER_TICK_100MS: // Generated ten times a second
	case MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY: // Generated 100 times a second
		break;


	default:
	case TimerTickMngt::TIMER_EXPIRED: // A timer has expired - eventInfo from timerId
	case SigGen::SIG_GEN_STARTED: // A Signal Generator has been started
	case SigGen::SIG_GEN_FINISHED: // A Signal Generator has finished 
	case SigGen::SIG_GEN_STOPPED: // A Signal Generator has been stopped
	case TimerTickMngt::TIMER_FREQ_GEN: // Generated a certain number of times a seconds

		Serial.print("New Event: "); Serial.println(eventId);
		break;
	}


	switch (state)
	{
	case STATE_STARTUP:
	{
		switch (eventId)
		{
		case EVENT_STARTUP_TIMER_EXPIRY:
		case TunePlayer::TUNE_ENDED:
			Serial.println("Entering State Runmode");
			state = STATE_RUN_MODE;
			break;

		case EVENT_CALLIBRATION_REQUEST:
			callibrateMotionSensor();
			restartStartupTimer();
			break;

		case TunePlayer::TUNE_NEXT_NOTE:
			static const uint32_t colours[StrawberryString::num_lights] = { 0xff0000, 0x00ff00, 0x0000ff, 0x00ffff, 0xff00ff };
			uint8_t i5 = eventInfo % strStr.num_lights;

			for (int i = 0; i < strStr.num_lights; i++) {
				strStr.ledStrip[i] = colours[i5];
			}
			break;
		}
	}
	break; // STATE_STARTUP

	case STATE_RUN_MODE:
	{
		switch (eventId)
		{
		case  EVENT_POINTING_UP:
			state = STATE_SELECT_MODE;
			startSelectModeTimer();
			Serial.println("Entering State Select Mode");
			piezzo.stop();
			piezzo.playPeriod_us(NOTE_C5_US, 200);
			break;

		case MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY:
		{
			switch (melodyMode) {
			case MM_CONT:
				updateMMCont();
				break;

			case MM_TWELVE:
				updateMMTwelve();
				break;

			case MM_C_MAJOR:
				updateMMCMajor();
				break;

			case MM_C_MINOR:
				updateMMCMinor();
				break;
			}

		}
		break;

		}
	}
	break; // STATE_RUN_MODE

	case STATE_SELECT_MODE:
	{
		switch (eventId)
		{
		case EVENT_SELECT_TIMER_EXPIRY:
			if (evntPrePro.orientation_state == EventPreProcessor::MCO_HORIZONTAL)
			{
				initMMMode(melodyMode);
				state = STATE_RUN_MODE;
				Serial.println("Entering State Run Mode");
			}
			break;

		case EVENT_POINTING_HORIZONTALY:
			restartSelectModeTimer();
			break;

		case EVENT_POINTING_DOWN:
			--melodyMode %= MM_NUM_MODES;
			Serial.print("New Melody Mode: "); Serial.println(melodyMode);
			piezzo.playPeriod_us(NOTE_C5_US, 200);
			break;

		case EVENT_POINTING_UP:
			++melodyMode %= MM_NUM_MODES;
			Serial.print("New Melody Mode: "); Serial.println(melodyMode);
			piezzo.playPeriod_us(NOTE_C5_US, 200);
			break;
		}
	}
	break; // STATE_SELECT_MODE

	}
}


void MelodyCane::startStartupTimer()
{
	timer.startTimer(START_TIMER_DURATATION_MS, STARTUP_TIMER_REF);
}

void MelodyCane::restartStartupTimer(void)
{
	timer.stopTimer();
	timer.startTimer(START_TIMER_DURATATION_MS, STARTUP_TIMER_REF);
}

void MelodyCane::startSelectModeTimer(void)
{
	timer.startTimer(SELECT_TIMER_DURATION_MS, SELECT_TIMER_REF);
}

void MelodyCane::restartSelectModeTimer(void)
{
	timer.stopTimer();
	timer.startTimer(SELECT_TIMER_DURATION_MS, SELECT_TIMER_REF);
}

void MelodyCane::stopSelectModeTimer(void)
{
	timer.stopTimer();
}

void MelodyCane::updateMMCont(void)
{
	int16_t rotY = evntPrePro.rotY_16384;
	int16_t myNote = (uint16_t)map((long)rotY, -12000, 12000, 50, 7500);

	if (myNote < 50)
		myNote = 50;
	if (myNote > 7500)
		myNote = 7500;

	if (!mute)
	{
		if (evntPrePro.rotX_16384 < 10)
			piezzo.playFrequency_hz(myNote, 0);
		else
			piezzo.stop();
	}

	uint8_t hue = (uint8_t)map(myNote, 50, 7500, 0, 255);
	ColourHSV myHSV(hue, evntPrePro.saturation, 255);

	for (int i = 0; i < strStr.num_lights; i++)
		ColourConverter::ConvertToRGB(&myHSV, strStr.ledStrip + i);
}

void MelodyCane::updateMMTwelve(void)
{
	PROGMEM uint32_t notes[12] = { NOTE_C5_US, NOTE_CS5_US, NOTE_D5_US, NOTE_DS5_US, NOTE_E5_US, NOTE_F5_US,
		NOTE_FS5_US, NOTE_G5_US, NOTE_GS5_US, NOTE_A5_US, NOTE_AS5_US, NOTE_B5_US };

	PROGMEM uint32_t colours[12] = { RED, GREEN, BLUE,  RED, GREEN, BLUE,  RED, GREEN, BLUE,  RED, GREEN, BLUE };

	int16_t rotY = evntPrePro.rotY_16384;
	if (stepper.update(rotY)) {
		if (!mute)
			piezzo.playPeriod_us(notes[stepper.currentStep], 0);
	}

	ColourHSV myHSV(stepper.currentStep * 12, evntPrePro.saturation, 255);
	for (int i = 0; i < strStr.num_lights; i++)
		ColourConverter::ConvertToRGB(&myHSV, strStr.ledStrip + i);
}

void MelodyCane::updateMMCMajor(void)
{
	PROGMEM uint32_t notes[14] = { NOTE_C5_US, NOTE_D5_US, NOTE_E5_US, NOTE_F5_US,
		NOTE_G5_US, NOTE_A5_US, NOTE_B5_US, NOTE_C6_US, NOTE_D6_US, NOTE_E6_US, NOTE_F6_US,
		NOTE_G6_US, NOTE_A6_US, NOTE_B6_US };

	PROGMEM uint32_t colours[14] = { RED, GREEN, BLUE,  RED, GREEN, BLUE,  RED, GREEN, BLUE,  RED, GREEN, BLUE, RED, GREEN };

	int16_t rotY = evntPrePro.rotY_16384;
	if (stepper.update(rotY)) {
		if (!mute)
			piezzo.playPeriod_us(notes[stepper.currentStep], 0);
	}

	ColourHSV myHSV(colours[stepper.currentStep], evntPrePro.saturation, 255);
	for (int i = 0; i < strStr.num_lights; i++)
		ColourConverter::ConvertToRGB(&myHSV, strStr.ledStrip + i);
}


void MelodyCane::updateMMCMinor(void)
{
	PROGMEM uint32_t notes[14] = { NOTE_C5_US, NOTE_D5_US, NOTE_DS5_US, NOTE_F5_US,
		NOTE_G5_US, NOTE_GS5_US, NOTE_AS5_US, NOTE_C6_US, NOTE_D6_US, NOTE_DS6_US, NOTE_F6_US,
		NOTE_G6_US, NOTE_GS6_US, NOTE_AS6_US };

	PROGMEM uint32_t colours[14] = { RED, GREEN, BLUE,  RED, GREEN, BLUE,  RED, GREEN, BLUE,  RED, GREEN, BLUE, RED, GREEN };

	int16_t rotY = evntPrePro.rotY_16384;
	if (stepper.update(rotY)) {
		if (!mute)
			piezzo.playPeriod_us(notes[stepper.currentStep], 0);
	}

	ColourHSV myHSV(colours[stepper.currentStep], evntPrePro.saturation, 255);
	for (int i = 0; i < strStr.num_lights; i++)
		ColourConverter::ConvertToRGB(&myHSV, strStr.ledStrip + i);
}

void MelodyCane::initMMMode(uint8_t melodyMode)
{
	switch (melodyMode)
	{
	case MM_CONT:
		break;

	case MM_TWELVE:
		stepper.configSteps(-12383, 12384, 12);
		break;

	case MM_C_MAJOR:
		stepper.configSteps(-12383, 12384, 14);
		break;

	case MM_C_MINOR:
		stepper.configSteps(-12383, 12384, 14);
		break;
	}
}


void MelodyCane::callibrateMotionSensor()
{
	StrawberryString::EEPROM_DATA eeprom_data;
	strStr.getEepromData(&eeprom_data);
	Serial.println("Starting Self Cal");

	strStr.motionSensor.runSelfCalibrate(&eeprom_data.ms_calibration);

	Serial.println("Writing to EEPROM");
	Serial.println(eeprom_data.ms_calibration.accelXoffset);
	Serial.println(eeprom_data.ms_calibration.accelYoffset);
	Serial.println(eeprom_data.ms_calibration.accelZoffset);
	Serial.println(eeprom_data.ms_calibration.gyroXoffset);
	Serial.println(eeprom_data.ms_calibration.gyroYoffset);
	Serial.println(eeprom_data.ms_calibration.gyroZoffset);

	eeprom_data.hardwareVersion = 0;
	eeprom_data.serialNumber = 0;
	eeprom_data.lightStringType = 0;

	strStr.setEepromData(&eeprom_data);
}


/**************************************************************************

Event PreProcessor Implementation

**************************************************************************/

MelodyCane::EventPreProcessor::EventPreProcessor()
{
	orientation_state = MCO_HORIZONTAL;
}

void MelodyCane::EventPreProcessor::configEventHandler(IEventHandler * eh)
{
	eventHandler = eh;
}

void MelodyCane::EventPreProcessor::handleEvent(uint16_t eventId, uint8_t sourceInst, uint16_t eventInfo)
{
	switch (eventId)
	{
	case TimerTickMngt::TIMER_EXPIRED:
	{
		if (eventInfo == MelodyCane::STARTUP_TIMER_REF)
			eventHandler->handleEvent(MelodyCane::EVENT_STARTUP_TIMER_EXPIRY, 0, 0);
		else if (eventInfo == MelodyCane::SELECT_TIMER_REF)
			eventHandler->handleEvent(MelodyCane::EVENT_SELECT_TIMER_EXPIRY, 0, 0);
	}
	break;

	case TimerTickMngt::TIMER_TICK_UPDATE:
		if (Serial.available())
		{
			char c = Serial.read();
			if (c == 'c')
				eventHandler->handleEvent(MelodyCane::EVENT_CALLIBRATION_REQUEST, 0, 0);

			if (c == 'm')
				eventHandler->handleEvent(MelodyCane::EVENT_MUTE, 0, 0);
		}
		break;

	case MotionSensor::MOTION_SENSOR_NEW_SMPL_RDY:
		processMotionSensorSample();
		eventHandler->handleEvent(eventId, sourceInst, eventInfo);
		break;

	default:
		eventHandler->handleEvent(eventId, sourceInst, eventInfo);
		break;
	}
}


/*
 * processMotionSensorSample - this performs calculations on the motion sensor output
 *                             detects certain events and makes results available to
 *                             Melody Cane state machine
 */
void MelodyCane::EventPreProcessor::processMotionSensorSample(void)
{
	RotationQuaternion_16384 * msRotq = &myMelodyCane.strStr.motionSensor.rotQuat;
	Quaternion_16384 qy_rotated(0, 0, 16384, 0);
	Quaternion_16384 qz(0, 0, 0, 16384);

	msRotq->rotate(&qy_rotated);
	cosAngleToVertical_16384 = qy_rotated.dotProduct(&qz);

	switch (orientation_state)
	{
	case MCO_UP:
		// cosAngle will be negative
		if (cosAngleToVertical_16384 > (cosVerticalHistoresis - cosVerticalThreshold_16384))
		{
			orientation_state = MCO_HORIZONTAL;
			eventHandler->handleEvent(MelodyCane::EVENT_POINTING_HORIZONTALY, 0, 0);
		}
		break;

	case MCO_HORIZONTAL:
		if (cosAngleToVertical_16384 > cosVerticalThreshold_16384)
		{
			orientation_state = MCO_DOWN;
			eventHandler->handleEvent(MelodyCane::EVENT_POINTING_DOWN, 0, 0);
		}
		else if (cosAngleToVertical_16384 < -cosVerticalThreshold_16384)
		{
			orientation_state = MCO_UP;
			eventHandler->handleEvent(MelodyCane::EVENT_POINTING_UP, 0, 0);
		}
		else if (cosAngleToVertical_16384 > cosVerticalSatThreshold_16384)
		{
			saturation = (uint8_t) map((long)cosAngleToVertical_16384,
				(long)cosVerticalSatThreshold_16384, (long)cosVerticalThreshold_16384,
				(long)255, (long)0);
		}
		else if (cosAngleToVertical_16384 < -cosVerticalSatThreshold_16384)
		{
			saturation = (uint8_t)map((long)cosAngleToVertical_16384,
				(long)-cosVerticalSatThreshold_16384, (long)-cosVerticalThreshold_16384,
				(long)255, (long)0);
		}
		else
			saturation = 255;

		break;

	case MCO_DOWN:
		// cosAngle will be positive
		if (cosAngleToVertical_16384 < (cosVerticalThreshold_16384 - cosVerticalHistoresis))
		{
			orientation_state = MCO_HORIZONTAL;
			eventHandler->handleEvent(MelodyCane::EVENT_POINTING_HORIZONTALY, 0, 0);
		}
		break;
	}

	double rotX = msRotq->getSinRotX();
	double rotY = msRotq->getSinRotY();
	double rotZ = msRotq->getSinRotZ();

	rotX /= 16384;
	rotY /= 16384;
	rotZ /= 16384;

	rotX_16384 = (int16_t)(asin(rotX) * 0x8000 / M_PI);
	rotY_16384 = (int16_t)(asin(rotY) * 0x8000 / M_PI);
	rotZ_16384 = (int16_t)(asin(rotZ) * 0x8000 / M_PI);
}


/**************************************************************************
 
  Tone Gen Implementation

 **************************************************************************/

MelodyCane::ToneGen::ToneGen()
{
}

void MelodyCane::ToneGen::setup()
{
	AvrTimer1::setWaveformGenerationMode(
		AvrTimer1::PWM_PHASE_AND_FREQUENCY_CORRECT_ICR,
		AvrTimer1::PIN_DISCONNECTED,
		AvrTimer1::PIN_DISCONNECTED
	);
}



/*
* This configures the timer prescaler and TOP values to achieve the
* requested frequency. The timer then automatically starts and will
* toggle OC1A and OC1B
*
* The Master clock frequency is F_CPU and expected to be 16M or 8M
* At 16M the lowest frequecy without prescaling is 16000000 / 0xffff = 244.14
* With the use of PRESCALING_8 this allows us to reach 30.5Hz which allows
* support for NOTE_B0 is 31Hz.
*
*/
void MelodyCane::ToneGen::playFrequency_hz(uint16_t frequency, uint16_t duration_ms)
{
	uint32_t _period_us;
	if (frequency == 0)
	{
		this->stop();
		return;
	}
	_period_us = 1000000 / frequency;
	playPeriod_us(_period_us, duration_ms);
}

void MelodyCane::ToneGen::playPeriod_us(uint32_t _period_us, uint16_t duration_ms)
{
	uint32_t ocra;
	/*
	* If period is 0 or less than NOTE_B8_US (127)
	* Then don't play anything
	*/
	if (_period_us < NOTE_B8_US)
	{
		AvrTimer1::setInterruptMaskRegister(0);
		AvrTimer1::setClockSelectMode(AvrTimer1::NO_CLOCK_SOURCE);
		return;
	}


	AvrTimer1::setWaveformGenerationMode(
		AvrTimer1::PWM_PHASE_AND_FREQUENCY_CORRECT_ICR,
		AvrTimer1::PIN_DISCONNECTED,
		AvrTimer1::PIN_DISCONNECTED
	);

	DDRC = 0x0c;

	AvrTimer1::setPeriod_us(AvrTimer1::PWM_PHASE_AND_FREQUENCY_CORRECT_ICR, _period_us);
	AvrTimer1::setInterruptMaskRegister(AvrTimer1::OUTPUT_COMPARE_A_MATCH_INTERRUPT_ENABLE);

	// set the duty cycle
	ocra = AvrTimer1::getInputCaptureRegister() >> 1;
	AvrTimer1::setOutputCompareRegisterA(ocra);

	ToneGen::period_us = _period_us;
}

void MelodyCane::ToneGen::writeValue(int32_t _period_us)
{
	playPeriod_us(_period_us, 0);
}

void MelodyCane::ToneGen::stop(void)
{
	pinMode(MelodyCane::TONE_GEN_PIN_2A, OUTPUT);
	digitalWrite(MelodyCane::TONE_GEN_PIN_2A, LOW);
	pinMode(MelodyCane::TONE_GEN_PIN_2B, INPUT_PULLUP);

	AvrTimer1::setClockSelectMode(AvrTimer1::NO_CLOCK_SOURCE);
	AvrTimer1::setInterruptMaskRegister(0);
}

void MelodyCane::ToneGen::mute(bool on)
{
	if (on)
		AvrTimer1::setInterruptMaskRegister(0);
	else
	{
		AvrTimer1::CLOCK_SELECT_MODES csm = AvrTimer1::getClockSelectMode();
		if (csm != AvrTimer1::NO_CLOCK_SOURCE)
			AvrTimer1::setInterruptMaskRegister(AvrTimer1::OUTPUT_COMPARE_A_MATCH_INTERRUPT_ENABLE);
	}
}

void MelodyCane::ToneGen::setVolume(uint8_t volume)
{
}

uint32_t MelodyCane::ToneGen::timeTillNextInterrupt_us()
{
	AvrTimer1::CLOCK_SELECT_MODES csm = AvrTimer1::getClockSelectMode();
	if (csm == AvrTimer1::NO_CLOCK_SOURCE)
		return(-1);

	uint32_t elapsed_us = micros() - last_interrupt_us;
	uint32_t timeTillNext_us = (period_us >> 1) - elapsed_us;
	return(timeTillNext_us);
}

ISR(TIMER1_COMPA_vect)
{
	static uint8_t toneIter = 0;
	static uint8_t toneRegVals[] = {
		0b01010100,
		0b01011000,
		0b01100100,
		0b01101000,
		0b10010100,
		0b10011000,
		0b10100100,
		0b10101000
	};

	if (toneIter == 8)
		toneIter = 0;
	PORTD = toneRegVals[toneIter++];

	last_interrupt_us = micros();
}