#include "initialisation.h"

class samples {
public:
	int16_t samples[SAMPLE_BUFFER_LENGTH];
	uint32_t readPosition = 10;
	uint32_t writePosition = 0;
	bool sampleClock = false;
	const int16_t delayHysteresis = 100;
	const int16_t crossFade = 32;
};
