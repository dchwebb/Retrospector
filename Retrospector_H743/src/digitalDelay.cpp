#include "digitaldelay.h"
#include <limits>

int32_t digitalDelay::calcSample() {
	int32_t nextSample;

	// Cross fade if moving playback position
	if (delayCrossfade > 0) {
		float scale = static_cast<float>(delayCrossfade) / static_cast<float>(crossFade);
		nextSample = static_cast<float>(samples[readPos]) * (1.0f - scale) + static_cast<float>(samples[oldReadPos]) * (scale);
		--delayCrossfade;
		if (SysTickVal > 2000) {
			int susp = 1;
		}
		if (delayCrossfade == 0) {
			GPIOB->ODR &= ~GPIO_ODR_OD7;
		}
	} else {
		nextSample = samples[readPos];
	}
	lastSample = nextSample;


	// Move write and read heads one sample forwards
	if (++writePos == SAMPLE_BUFFER_LENGTH) 		writePos = 0;
	if (++readPos == SAMPLE_BUFFER_LENGTH)			readPos = 0;
	if (++oldReadPos == SAMPLE_BUFFER_LENGTH)		oldReadPos = 0;

	// Get delay time from ADC and average over 32 readings to smooth
	dampedDelay = std::max((31 * dampedDelay + (static_cast<int32_t>(ADC_array[ADC_Delay_Pot_L] - 150))) >> 5, 0L);

	// Change delay times after a pause to avoid pitched artifacts
	if (std::abs(dampedDelay - currentDelay) > delayHysteresis) {
		if (delayChanged == 0)
			delayChanged = 1000;
		currentDelay = dampedDelay;
	}

	if (delayChanged > 0 && delayCrossfade == 0) {
		if (delayChanged == 1) {
			oldReadPos = readPos;
			readPos = writePos - dampedDelay;
			if (readPos < 0) 		readPos += SAMPLE_BUFFER_LENGTH;
			delayCrossfade = crossFade;
			GPIOB->ODR |= GPIO_ODR_OD7;
		}
		--delayChanged;
	}


	return nextSample;
}
