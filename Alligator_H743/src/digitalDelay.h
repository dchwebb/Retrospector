#include "initialisation.h"

class digitalDelay {
public:
	int16_t samples[SAMPLE_BUFFER_LENGTH];
	int32_t readPos;
	int32_t writePos;
	int16_t delayChanged;
	uint8_t delayCrossfade;

	int32_t currentDelay;
	int32_t dampedDelay;

	int32_t lastSample;

	bool sampleClock = false;
	const int16_t delayHysteresis = 100;
	const int16_t crossFade = 128;

	int32_t calcSample();
};
