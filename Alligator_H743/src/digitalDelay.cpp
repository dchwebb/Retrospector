#include "digitaldelay.h"
#include <limits>

int32_t digitalDelay::calcSample() {
	int32_t nextSample = samples[readPos];
	sampleUp = nextSample > lastSample;

	// Cross fade if moving playback position
	if (delayCrossfade > 0) {
		int32_t diff = nextSample - lastSample;
		if (std::abs(diff) > 400) {

			nextSample = lastSample + (diff) / 8;

			--delayCrossfade;
		} else {
			GPIOB->ODR &= ~GPIO_ODR_OD7;
			delayCrossfade = 0;
		}

		//--delayCrossfade;
	}

	// Move write and read heads one sample forwards
	if (++writePos == SAMPLE_BUFFER_LENGTH) 		writePos = 0;
	if (++readPos == SAMPLE_BUFFER_LENGTH)			readPos = 0;
	//if (++targetReadPos == SAMPLE_BUFFER_LENGTH)	targetReadPos = 0;

	// Get delay time from ADC and average over 32 readings to smooth
	dampedDelay = std::max((31 * dampedDelay + (static_cast<int32_t>(ADC_array[2] - 150))) >> 5, 0L);

	// Change delay times after a pause to avoid pitched artifacts
	if (std::abs(dampedDelay - currentDelay) > delayHysteresis) {
		if (delayChanged == 0)
			delayChanged = 1000;
		currentDelay = dampedDelay;
	}

	if (delayChanged > 0) {
		if (delayChanged == 1) {
			int newReadPos = writePos - dampedDelay;
			if (newReadPos < 0) 		newReadPos = SAMPLE_BUFFER_LENGTH + newReadPos;
			GPIOB->ODR |= GPIO_ODR_OD7;
			// Check that the difference between the current and target sample is small
			//if (std::abs(nextSample - samples[newReadPos]) < 100 && newReadPos - 1 > 0 && sampleUp == (samples[newReadPos] > samples[newReadPos - 1]) ) {
			if (std::abs(samples[newReadPos] - lastSample) < 100 && (samples[newReadPos] >= lastSample) == (nextSample >= lastSample))  {
				ns = nextSample;
				s_n = samples[newReadPos];
				ls = lastSample;
				rp = readPos;
				nrp = newReadPos;

				if (SysTickVal > 2000) {
					int susp = 2;
				}

				//delayCrossfade = crossFade;
				GPIOB->ODR &= ~GPIO_ODR_OD7;
				readPos = newReadPos;
				nextSample = samples[newReadPos];
				delayChanged = 0;
			}
		} else {
			--delayChanged;
		}
	}

	lastSample = nextSample;
	return nextSample;
}
