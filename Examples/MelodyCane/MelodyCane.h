

#include "Stepper.h"

class MelodyCane : public IEventHandler
{
public:

	MelodyCane();
	void setup();
	void handleEvent(
		uint16_t eventId,
		uint8_t sourceReference,
		uint16_t eventInfo);
	void update(void);

	typedef enum {
		STARTUP_TIMER_REF = 0,
		SELECT_TIMER_REF = 2
	}TIMER_REF;

	typedef enum {
		MM_CONT = 0,
		MM_TWELVE = 1,
		MM_C_MAJOR = 2,
		MM_C_MINOR = 3,
		MM_NUM_MODES = 4
	}MELODY_MODE;

	const static uint8_t STATE_STARTUP = 0;
	const static uint8_t STATE_SELECT_MODE = 1;
	const static uint8_t STATE_RUN_MODE = 2;

	const static uint16_t EVENT_STARTUP = IEventHandler::USER + 0;
	const static uint16_t EVENT_STARTUP_TIMER_EXPIRY = IEventHandler::USER + 1;
	const static uint16_t EVENT_CALLIBRATION_REQUEST = IEventHandler::USER + 2;
	const static uint16_t EVENT_POINTING_UP = IEventHandler::USER + 3;
	const static uint16_t EVENT_POINTING_DOWN = IEventHandler::USER + 4;
	const static uint16_t EVENT_POINTING_HORIZONTALY = IEventHandler::USER + 5;
	const static uint16_t EVENT_SELECT_TIMER_EXPIRY = IEventHandler::USER + 6;
	const static uint16_t EVENT_MUTE = IEventHandler::USER + 7;

	const static uint8_t TONE_GEN_PIN_1A = 2;
	const static uint8_t TONE_GEN_PIN_1B = 3;
	const static uint8_t TONE_GEN_PIN_2A = 4;
	const static uint8_t TONE_GEN_PIN_2B = 5;
	const static uint8_t TONE_GEN_PIN_3A = 6;
	const static uint8_t TONE_GEN_PIN_3B = 7;

	const static uint16_t SELECT_TIMER_DURATION_MS = 3000;
	const static uint16_t START_TIMER_DURATATION_MS = 10000;

	class EventPreProcessor : public IEventHandler
	{
	public:
		EventPreProcessor();
		void configEventHandler(IEventHandler *);
		void handleEvent(
			uint16_t eventId,
			uint8_t sourceReference,
			uint16_t eventInfo);

		int16_t cosAngleToVertical_16384;
		int16_t rotX_16384;
		int16_t rotY_16384;
		int16_t rotZ_16384;

		uint8_t saturation;

		IEventHandler * eventHandler;

		typedef enum {
			MCO_UP = 0,
			MCO_DOWN = 1,
			MCO_HORIZONTAL = 2
		}MC_ORIENTATION;

		MC_ORIENTATION orientation_state;

	private:
		void processMotionSensorSample(void);
		const int16_t cosVerticalThreshold_16384 = 15395; // 20 degrees
		const int16_t cosVerticalSatThreshold_16384 = 12551; // 40  degrees
		const int16_t cosVerticalHistoresis = 200;
	};

	EventPreProcessor evntPrePro;

private:
	StrawberryString strStr;
	PiezoDriver  piezzo;
	Timer timer;
	Stepper stepper;
	TunePlayer tunePlayer;

	uint8_t state;
	uint8_t melodyMode = MM_CONT;
	boolean mute = false;


	unsigned long lastUpdateTime_ms;
	uint16_t ledRefreshTime_us;

	void callibrateMotionSensor(void);
	void startStartupTimer(void);
	void restartStartupTimer(void);
	void startSelectModeTimer(void);
	void restartSelectModeTimer(void);
	void stopSelectModeTimer(void);
	void updateMMCont(void);
	void updateMMTwelve(void);
	void updateMMCMajor(void);	
	void updateMMCMinor(void);
    void initMMMode(uint8_t melodyMode);

	class ToneGen : public SweetMaker::IToneGen {
	public:
		ToneGen();
		void setup();

		void playPeriod_us(uint32_t period_us, uint16_t duration_ms);
		void playFrequency_hz(uint16_t frequency_hz, uint16_t duration_ms);
		void writeValue(int32_t period_us);
		void stop(void);
		void setVolume(uint8_t volume);
		void mute(boolean on);

		uint32_t timeTillNextInterrupt_us();

		uint32_t period_us;

	private:
		uint16_t Cv;
	};

	ToneGen toneGen;
};
