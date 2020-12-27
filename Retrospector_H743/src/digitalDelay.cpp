#include "digitaldelay.h"
#include <limits>

extern uint32_t clockInterval;
extern bool clockValid;
extern int16_t samples[2][SAMPLE_BUFFER_LENGTH];
extern int16_t filterBuffer[2][FIRTAPS];

int32_t digitalDelay::calcSample(digitalDelay::sampleLR LOrR, int16_t adcSample) {
	int32_t nextSample;

	// Cross fade if moving playback position
	if (delayCrossfade[LOrR] > 0) {
		float scale = static_cast<float>(delayCrossfade[LOrR]) / static_cast<float>(crossFade);
		nextSample = static_cast<float>(samples[LOrR][readPos[LOrR]]) * (1.0f - scale) + static_cast<float>(samples[LOrR][oldReadPos[LOrR]]) * (scale);
		--delayCrossfade[LOrR];
	} else {
		nextSample = samples[LOrR][readPos[LOrR]];
	}

	// Filter output - use a separate filter buffer for the calculations as this will use SRAM which much faster than SDRAM
	filterBuffer[LOrR][filterBuffPos[LOrR]] = nextSample;
	if (activateFilter) {
		float outputSample = 0.0;
		int16_t pos = filterBuffPos[LOrR];
		for (uint16_t i = 0; i < FIRTAPS; ++i) {
			if (++pos == FIRTAPS) pos = 0;
			outputSample += firCoeff[activeFilter][i] * filterBuffer[LOrR][pos];
		}
		nextSample = static_cast<int32_t>(outputSample);
	}

	int32_t temp = (static_cast<int32_t>(ADC_audio[LOrR]) - adcZeroOffset) +
			(static_cast<float>(ADC_array[ADC_Feedback_Pot]) / 65536.0f * static_cast<float>(nextSample));

	samples[LOrR][writePos[LOrR]] = std::max(std::min(temp, 23000L), -23000L);

	// Move write and read heads one sample forwards
	if (++writePos[LOrR] == SAMPLE_BUFFER_LENGTH) 		writePos[LOrR] = 0;
	if (++readPos[LOrR] == SAMPLE_BUFFER_LENGTH)		readPos[LOrR] = 0;
	if (++oldReadPos[LOrR] == SAMPLE_BUFFER_LENGTH)		oldReadPos[LOrR] = 0;
	if (++filterBuffPos[LOrR] == FIRTAPS) 				filterBuffPos[LOrR] = 0;


	// Get delay time from ADC and average over 32 readings to smooth
	volatile uint32_t delayClkCV = clockValid ? (clockInterval * 48) : static_cast<uint32_t>(ADC_array[LOrR == digitalDelay::channelL ? ADC_Delay_Pot_L : ADC_Delay_Pot_R]);
	dampedDelay[LOrR] = std::max((31 * dampedDelay[LOrR] + delayClkCV) >> 5, 0UL);

	// Change delay times after a pause to avoid pitched artifacts
	if (std::abs(dampedDelay[LOrR] - currentDelay[LOrR]) > delayHysteresis) {
		if (delayChanged[LOrR] == 0)
			delayChanged[LOrR] = 1;
		currentDelay[LOrR] = dampedDelay[LOrR];
	}

	if (delayChanged[LOrR] > 0 && delayCrossfade[LOrR] == 0) {
		if (delayChanged[LOrR] == 1) {
			oldReadPos[LOrR] = readPos[LOrR];
			readPos[LOrR] = writePos[LOrR] - dampedDelay[LOrR] - 1;
			while (readPos[LOrR] < 0) 		readPos[LOrR] += SAMPLE_BUFFER_LENGTH;
			delayCrossfade[LOrR] = crossFade;
		}
		--delayChanged[LOrR];
	}




	return nextSample;
}
