#include "digitaldelay.h"
#include <limits>

extern uint32_t clockInterval;
extern bool clockValid;
extern int16_t samples[2][SAMPLE_BUFFER_LENGTH];


int32_t digitalDelay::calcSample(digitalDelay::sampleLR LOrR) {
	int32_t nextSample;

	// Cross fade if moving playback position
	if (delayCrossfade[LOrR] > 0) {
		float scale = static_cast<float>(delayCrossfade[LOrR]) / static_cast<float>(crossFade);
		nextSample = static_cast<float>(samples[LOrR][readPos[LOrR]]) * (1.0f - scale) + static_cast<float>(samples[LOrR][oldReadPos[LOrR]]) * (scale);
		--delayCrossfade[LOrR];
	} else {
		nextSample = samples[LOrR][readPos[LOrR]];
	}
	lastSample[LOrR] = nextSample;

	// Move write and read heads one sample forwards
	if (++writePos[LOrR] == SAMPLE_BUFFER_LENGTH) 		writePos[LOrR] = 0;
	if (++readPos[LOrR] == SAMPLE_BUFFER_LENGTH)		readPos[LOrR] = 0;
	if (++oldReadPos[LOrR] == SAMPLE_BUFFER_LENGTH)		oldReadPos[LOrR] = 0;

	// Get delay time from ADC and average over 32 readings to smooth
	uint32_t delayClkCV = clockValid ? (clockInterval * 48) : static_cast<uint32_t>(ADC_array[LOrR == digitalDelay::channelL ? ADC_Delay_Pot_L : ADC_Delay_Pot_R]);
	//uint32_t delayClkCV = clockValid ? (clockInterval * 48) : static_cast<uint32_t>(ADC_array[ADC_Delay_Pot_L]);
	dampedDelay[LOrR] = std::max((31 * dampedDelay[LOrR] + delayClkCV) >> 5, 0UL);

	// Change delay times after a pause to avoid pitched artifacts
	if (std::abs(dampedDelay[LOrR] - currentDelay[LOrR]) > delayHysteresis) {
		if (delayChanged[LOrR] == 0)
			delayChanged[LOrR] = 1000;
		currentDelay[LOrR] = dampedDelay[LOrR];
	}

	if (delayChanged[LOrR] > 0 && delayCrossfade[LOrR] == 0) {
		if (delayChanged[LOrR] == 1) {
			oldReadPos[LOrR] = readPos[LOrR];
			readPos[LOrR] = writePos[LOrR] - dampedDelay[LOrR];
			while (readPos[LOrR] < 0) 		readPos[LOrR] += SAMPLE_BUFFER_LENGTH;
			delayCrossfade[LOrR] = crossFade;
		}
		--delayChanged[LOrR];
	}


	return nextSample;
}
