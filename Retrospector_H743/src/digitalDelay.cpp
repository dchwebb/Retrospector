#include "digitaldelay.h"


uint32_t debugDuration = 0;
uint32_t filterDuration = 0;
volatile bool checkFilt = 0;

void digitalDelay::calcSample(channel LR) {
	int32_t delayClkCV;
	static int16_t leftWriteSample;
	float nextSample, pingSample;
	bool reverse = (mode() == modeReverse);
	int32_t recordSample = static_cast<int32_t>(ADC_audio[LR]);		// Capture recording sample here to avoid jitter
	StereoSample readSamples = {samples[readPos[LR]]};				// Get read samples as interleaved stereo

	channel RL = (LR == left) ? right : left;		// Get other channel for use in ping-pong calculations

	TIM3->CNT = 0;		// Debug

	// Cross fade if moving playback position
	if (delayCrossfade[LR] > 0) {
		StereoSample oldreadSamples = {samples[oldReadPos[LR]]};
		float scale = static_cast<float>(delayCrossfade[LR]) / static_cast<float>(crossfade);
		nextSample = static_cast<float>(readSamples.sample[LR]) * (1.0f - scale) + static_cast<float>(oldreadSamples.sample[LR]) * (scale);
		pingSample = static_cast<float>(readSamples.sample[RL]) * (1.0f - scale) + static_cast<float>(oldreadSamples.sample[RL]) * (scale);
		--delayCrossfade[LR];
	} else {
		nextSample = static_cast<float>(readSamples.sample[LR]);
		pingSample = static_cast<float>(readSamples.sample[RL]);
	}

	// Add in a scaled amount of the sample from the opposite stereo channel
	if (pingPong) {
		nextSample += pingSample * ADC_array[ADC_Delay_CV_L] / 65536.0f;
	}


	// Filter output - use a separate filter buffer for the calculations as this will use SRAM which much faster than SDRAM
	if (activateFilter) {			// For debug
		if (filter.filterType == IIR) {
			nextSample = filter.CalcIIRFilter(nextSample, LR);
		} else {
			//nextSample = filter.CalcFIRFilter(nextSample, LR);

			//volatile uint8_t pos;

			filter.filterBuffer[LR][filter.filterBuffPos[LR]] = nextSample;
			if (filter.currentCutoff == 1.0f) {		// If not filtering take middle most sample to account for FIR group delay when filtering active (gives more time for main loop when filter inactive)

				uint8_t mid = filter.filterBuffPos[LR] - (uint8_t)(FIRTAPS / 2);
				//uint8_t mid = filter.filterBuffPos[LR];
				//if (pos < 0) pos += FIRTAPS;
				nextSample = filter.filterBuffer[LR][mid];
			} else {
				float outputSample = 0.0;
				volatile uint8_t pos;
				volatile uint8_t revpos = filter.filterBuffPos[LR];

				if (!checkFilt) {
					pos = (uint8_t)(filter.filterBuffPos[LR] - FIRTAPS);
					for (uint8_t i = 0; i < FIRTAPS; ++i) {
						//if (++pos == FIRTAPS) pos = 0;
						pos++;
						outputSample += filter.firCoeff[filter.activeFilter][i] * filter.filterBuffer[LR][pos];
					}
				} else {
					pos = filter.filterBuffPos[LR] - FIRTAPS;
					for (uint8_t i = 0; i < FIRTAPS / 2; ++i) {
						//if (++pos == FIRTAPS) pos = 0;
						pos++;
						// Folded FIR structure - as coefficients are symmetrical we can multiple the sample 1 + sample N by the 1st coefficient, sample 2 + sample N - 1 by 2nd coefficient etc
						outputSample += filter.firCoeff[filter.activeFilter][i] * (filter.filterBuffer[LR][pos] + filter.filterBuffer[LR][revpos]);

						--revpos;
						//if (--revpos == -1)	revpos = FIRTAPS - 1;
					}
					//if (++pos == FIRTAPS) pos = 0;
					pos++;
					outputSample += filter.firCoeff[filter.activeFilter][FIRTAPS / 2] * filter.filterBuffer[LR][pos];
				}
				nextSample = outputSample;
			}


			++filter.filterBuffPos[LR];		// FIXME - probably need only one position, incremented on right sample

//			if (++filter.filterBuffPos[LR] == FIRTAPS)
//				filter.filterBuffPos[LR] = 0;

		}
	}

	// Timing Debug
	volatile uint32_t time = TIM3->CNT;
	if (time > filterDuration) {
		filterDuration = time;
		if (time > 200) {
			volatile int susp = 1;
			susp++;
		}
	}

	// Compression
	if (nextSample > threshold || nextSample < -threshold) {
		int8_t thresholdSign = (nextSample < -threshold ? -1 : 1);
		int32_t excess = abs(nextSample - thresholdSign * threshold);
		nextSample = (threshold + (ratio * excess) / (ratio + excess)) * thresholdSign;
	}


	// Put next output sample in I2S buffer
	SPI2->TXDR = std::clamp(static_cast<int32_t>(nextSample), -32767L, 32767L);


	// Add the current sample and the delayed sample scaled by the feedback control
	int32_t feedbackSample = (recordSample - adcZeroOffset[LR]) +
			(static_cast<float>(ADC_array[ADC_Feedback_Pot]) / 65536.0f * nextSample);


	// Hold the left sample in a temporary variable and write both left and right samples when processing right signal
	if (LR == left) {
		leftWriteSample = static_cast<uint16_t>(std::clamp(feedbackSample, -32767L, 32767L));
	} else {
		StereoSample writeSample;
		writeSample.sample[left] = leftWriteSample;
		writeSample.sample[right] = static_cast<uint16_t>(std::clamp(feedbackSample, -32767L, 32767L));
		samples[writePos[LR]] = writeSample.bothSamples;
	}



	// Move write and read heads one sample forwards
	if (++writePos[LR] == SAMPLE_BUFFER_LENGTH) 		writePos[LR] = 0;
	if (reverse) {
		if (--readPos[LR] < 0)							readPos[LR] = SAMPLE_BUFFER_LENGTH - 1;
		if (--oldReadPos[LR] < 0)						oldReadPos[LR] = SAMPLE_BUFFER_LENGTH - 1;
	} else {
		if (++readPos[LR] == SAMPLE_BUFFER_LENGTH)		readPos[LR] = 0;
		if (++oldReadPos[LR] == SAMPLE_BUFFER_LENGTH)	oldReadPos[LR] = 0;
	}


	// Get delay time from ADC or tempo clock and average over 32 readings to smooth
	delayClkCV = static_cast<int32_t>(ADC_array[(LR == left) ? ADC_Delay_Pot_L : ADC_Delay_Pot_R]);
	if (clockValid) {
		if (abs(delayPotVal[LR] - delayClkCV) > tempoHysteresis) {
			delayPotVal[LR] = delayClkCV;											// Store value for hysteresis checking
			delayMult[LR] = tempoMult[tempoMult.size() * delayClkCV / 65536];		// get tempo multiplier from lookup
		}
		calcDelay[LR] = delayMult[LR] * clockInterval * SAMPLE_RATE / 1000;

	} else {
		if (mode() != modeShort)
			delayClkCV *= SAMPLE_BUFFER_LENGTH / 65356;
		calcDelay[LR] = std::max((31 * calcDelay[LR] + delayClkCV) >> 5, 0L);
	}


	// If reversing delay samples read head works backwards from write position until delay time is reached and then jumps back to write position (with crossfade)
	if (reverse) {
		// Adjust read and write positions to handle circular buffer complications
		int32_t wp = writePos[LR];
		int32_t rp = readPos[LR];
		if (rp > wp) {
			if (wp - (calcDelay[LR] * 2) < 0)
				wp += (SAMPLE_BUFFER_LENGTH - 1);
			else
				rp -= (SAMPLE_BUFFER_LENGTH - 1);
		}
		int32_t remainingDelay = rp + (calcDelay[LR] * 2) - wp;

		// Scale the LED so it fades in with the timing of the reverse delay
		reverseLED(LR, remainingDelay);

		// check if read position is less than write position less delay then reset read position to write position
		if (remainingDelay < 0 && delayCrossfade[LR] == 0) {
			ledOffTime[LR] = SysTickVal + 50;
			oldReadPos[LR] = readPos[LR];
			readPos[LR] = writePos[LR] - 1;
			if (readPos[LR] < 0)	readPos[LR] = SAMPLE_BUFFER_LENGTH - 1;
			delayCrossfade[LR] = crossfade;
		}

	} else {
		// If delay time has changed trigger crossfade from old to new read position
		if (delayCrossfade[LR] == 0 && std::abs(calcDelay[LR] - currentDelay[LR]) > delayHysteresis) {
			oldReadPos[LR] = readPos[LR];
			readPos[LR] = writePos[LR] - calcDelay[LR] - 1;
			while (readPos[LR] < 0) 		readPos[LR] += SAMPLE_BUFFER_LENGTH;
			delayCrossfade[LR] = crossfade;
			currentDelay[LR] = calcDelay[LR];
		}
		updateLED(LR);
	}


//	time = TIM3->CNT;
//	if (time > debugDuration)
//		debugDuration = time;		// Debug


}


void digitalDelay::init() {
	calcDelay[left] = ADC_array[ADC_Delay_Pot_L];
	calcDelay[right] = ADC_array[ADC_Delay_Pot_R];
	calcSample(left);
	calcSample(right);
	delayCrossfade[left] = 0;
	delayCrossfade[right] = 0;
}

void digitalDelay::updateLED(channel c) {
	// Turns on LED delay time indicator, locking as accurately as possible to external tempo clock
	++ledCounter[c];
	if (clockValid) {
		// If using external clock try to sync LEDs to tempo, allowing for drift in both directions
		if (delayMult[c] < 1.0f) {
			if (SysTickVal - lastClock < 1 && ledOffTime[c] < SysTickVal) {		// Always trigger on the clock, unless LED already on
				ledFraction[c] = 0;
				ledOn(c);
			} else if (delayMult[c] * ledFraction[c] < 1 && ledCounter[c] > calcDelay[c]) {	// Handle fractional times
				ledFraction[c]++;
				ledOn(c);
			}
		} else if (SysTickVal - lastClock < 5 && ledCounter[c] > calcDelay[c] - 1000) { 		// 1000 ~= 20.8ms: handle tempos slower than 1
			ledOn(c);
		}
	} else {
		if (ledCounter[c] > calcDelay[c]) {
			ledOn(c);
		}
	}

	// Turn off delay LED after duration expired
	if (SysTickVal > ledOffTime[c]) {
		ledOff(c);
	}
}

// Scale the LED so it fades in with the timing of the reverse delay
void digitalDelay::reverseLED(channel c, int32_t remainingDelay)
{
	float rc = (float)remainingDelay / (calcDelay[c] * 2);
	ledCounter[c]++;
	if (ledCounter[c] >= pow(rc, 2.0f) * 120) {
		ledOn(c);
	} else {
		ledOff(c);
	}
}

// Switch LED on and set off time
void digitalDelay::ledOn(channel c) {
	if (c == left) {
		GPIOC->ODR |= GPIO_ODR_OD10;
//		GPIOC->ODR |= GPIO_ODR_OD12;		// Debug
	}
	if (c == right) {
		GPIOC->ODR |= GPIO_ODR_OD11;
	}
	ledCounter[c] = 0;
	ledOffTime[c] = SysTickVal + 50;		// In ms
}

// Directly switch LED on or off
void digitalDelay::ledOff(channel c) {
	if (c == left) {
		GPIOC->ODR &= ~GPIO_ODR_OD10;
//		GPIOC->ODR &= ~GPIO_ODR_OD12;		// Debug
	}
	if (c == right) {
		GPIOC->ODR &= ~GPIO_ODR_OD11;
	}
	ledOffTime[c] = 0;
}

delay_mode digitalDelay::mode() {
	// Return setting of mode switch
	if ((GPIOE->IDR & GPIO_IDR_ID2) == 0)
		return modeReverse;
	if ((GPIOE->IDR & GPIO_IDR_ID3) == 0)
		return modeShort;
	return modeLong;
}
