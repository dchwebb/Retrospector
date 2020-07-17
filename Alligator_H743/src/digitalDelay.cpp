#include "digitaldelay.h"

int32_t digitalDelay::calcSample() {
	int32_t nextSample = samples[readPos];

	// Cross fade if moving playback position
	if (delayCrossfade > 0) {		// && SysTickVal > 2000
		int32_t diff = nextSample - lastSample;
		if (std::abs(diff) > 400) {
			GPIOB->ODR |= GPIO_ODR_OD7;
			nextSample = lastSample + (diff) / 32;
			if (SysTickVal > 2000) {
				int susp = 1;
			}
			--delayCrossfade;
		} else {
			GPIOB->ODR &= ~GPIO_ODR_OD7;
			delayCrossfade = 0;
		}
	}
	lastSample = nextSample;

	// Move write and read heads one sample forwards
	if (++writePos == SAMPLE_BUFFER_LENGTH) 	writePos = 0;
	if (++readPos == SAMPLE_BUFFER_LENGTH)		readPos = 0;

	// Get delay time from ADC and average over 32 readings to smooth
	dampedDelay = std::max((31 * dampedDelay + ((int32_t)ADC_array[2] - 150)) >> 5, 0L);

	// Change delay times after a pause to avoid pitched artifacts
	if (std::abs(dampedDelay - currentDelay) > delayHysteresis) {
		if (delayChanged == 0)
			delayChanged = 1000;
		currentDelay = dampedDelay;
	}

	if (delayChanged > 0) {
		if (delayChanged == 1) {
			delayCrossfade = crossFade;
			readPos = writePos - dampedDelay;
			if (readPos < 0) {
				readPos = SAMPLE_BUFFER_LENGTH + readPos;
			}
		}
		--delayChanged;
	}
	return nextSample;
}
