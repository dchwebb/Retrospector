#include "digitaldelay.h"
#include <limits>

int32_t digitalDelay::calcSample() {
	int32_t nextSample = samples[readPos];
	sampleUp = nextSample > lastSample;

	// Cross fade if moving playback position
	if (delayCrossfade > 0 && SysTickVal > 2000) {
		// Move from the current read position to the target read position to try and find a place to cross fade

		// if either the readPos or targetReadPos is before the writePos (but not both) add the buffer length to the earlier position so relative distance can be calculated
		GPIOB->ODR |= GPIO_ODR_OD7;
		int tempTarg = targetReadPos, tempRead = readPos;
		if (readPos < writePos && targetReadPos > writePos) {
			tempRead = readPos + SAMPLE_BUFFER_LENGTH;
		}
		if (targetReadPos < writePos && readPos > writePos) {
			tempTarg = targetReadPos + SAMPLE_BUFFER_LENGTH;
		}
		int diff = std::numeric_limits<int>::max();		// Difference of position between the readPos and a provisional target read pos
		int newPos = -1;
		while (tempRead != tempTarg) {
			int r2 = (tempRead >= SAMPLE_BUFFER_LENGTH) ? 0 : tempRead;		// loop ring buffer
			if (std::abs(samples[r2] - nextSample) < 100 && std::abs(tempTarg - tempRead) < diff) {		// found a close target sample that is closer to the target position
				diff =  std::abs(tempTarg - tempRead);
				newPos = r2;
			}

			tempRead += tempTarg > tempRead ? 1 : -1;
		}

		// Found a better read position - move read head
		if (newPos != -1) {
			readPos = newPos;
		}
		GPIOB->ODR &= ~GPIO_ODR_OD7;
		/*int32_t diff = nextSample - lastSample;
		if (std::abs(diff) > 400) {
			GPIOB->ODR |= GPIO_ODR_OD7;
			nextSample = lastSample + (diff) / 32;

			--delayCrossfade;
		} else {
			GPIOB->ODR &= ~GPIO_ODR_OD7;
			delayCrossfade = 0;
		}
*/

		--delayCrossfade;
	}
	lastSample = nextSample;

	// Move write and read heads one sample forwards
	if (++writePos == SAMPLE_BUFFER_LENGTH) 		writePos = 0;
	if (++readPos == SAMPLE_BUFFER_LENGTH)			readPos = 0;
	if (++targetReadPos == SAMPLE_BUFFER_LENGTH)	targetReadPos = 0;

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
			delayCrossfade = crossFade;
			targetReadPos = writePos - dampedDelay;
			if (targetReadPos < 0) {
				targetReadPos = SAMPLE_BUFFER_LENGTH + targetReadPos;
			}
		}
		--delayChanged;
	}


	return nextSample;
}
