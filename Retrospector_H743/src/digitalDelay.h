#include "initialisation.h"

extern int32_t readPos;
extern int32_t oldReadPos;
extern int32_t writePos;
extern int16_t delayChanged;
extern uint16_t delayCrossfade;
extern int32_t currentDelay;
extern int32_t dampedDelay;
extern int32_t ns, s_n, ls, rp, nrp;

extern uint16_t adcZeroOffset;

class digitalDelay {
public:
	int16_t samples[SAMPLE_BUFFER_LENGTH];
//	int32_t readPos;
//	int32_t targetReadPos;
//	int32_t writePos;
//	int16_t delayChanged;
//	uint16_t delayCrossfade;
//	uint32_t oldReadPos;
//	int32_t currentDelay;
//	int32_t dampedDelay;

	int32_t lastSample;
	bool sampleUp;

	bool sampleClock = false;
	const int16_t delayHysteresis = 100;
	const int16_t crossFade = 1000;

	int32_t calcSample();
};
