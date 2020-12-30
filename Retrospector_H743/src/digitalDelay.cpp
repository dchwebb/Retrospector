#include "digitaldelay.h"

extern uint32_t clockInterval;
extern bool clockValid;
extern int16_t samples[2][SAMPLE_BUFFER_LENGTH];
extern int16_t filterBuffer[2][FIRTAPS];

//volatile int32_t wp;
//volatile int32_t rp;

int32_t digitalDelay::calcSample(channel LR) {
	int32_t nextSample, delayClkCV;
	bool reverse = (mode() == modeReverse);

	// Cross fade if moving playback position
	if (delayCrossfade[LR] > 0) {
		if (reverse)	LED(LR, true);		// In reverse mode crossfade happens each sample loop so also indicates delay length
		float scale = static_cast<float>(delayCrossfade[LR]) / static_cast<float>(crossfade);
		nextSample = static_cast<float>(samples[LR][readPos[LR]]) * (1.0f - scale) + static_cast<float>(samples[LR][oldReadPos[LR]]) * (scale);
		--delayCrossfade[LR];
	} else {
		if (reverse)	LED(LR, false);
		nextSample = samples[LR][readPos[LR]];
	}

	// Filter output - use a separate filter buffer for the calculations as this will use SRAM which much faster than SDRAM
	filterBuffer[LR][filterBuffPos[LR]] = nextSample;
	if (activateFilter) {			// For debug
		if (currentCutoff == 1.0f) {		// If not filtering take middle most sample to account for FIR group delay when filtering active (gives more time for main loop when filter inactive)
			int16_t pos = filterBuffPos[LR] - (FIRTAPS / 2);
			if (pos < 0) pos += FIRTAPS;
			nextSample = filterBuffer[LR][pos];
		} else {
			float outputSample = 0.0;
			int16_t pos = filterBuffPos[LR];
			for (uint16_t i = 0; i < FIRTAPS; ++i) {
				if (++pos == FIRTAPS) pos = 0;
				outputSample += firCoeff[activeFilter][i] * filterBuffer[LR][pos];
			}
			nextSample = static_cast<int32_t>(outputSample);
		}
	}

	// Add the current sample and the delayed sample scaled by the feedback control
	int32_t feedbackSample = (static_cast<int32_t>(ADC_audio[LR]) - adcZeroOffset[LR]) +
			(static_cast<float>(ADC_array[ADC_Feedback_Pot]) / 65536.0f * static_cast<float>(nextSample));

	samples[LR][writePos[LR]] = clamp(feedbackSample, -32767L, 32767L);

	// Move write and read heads one sample forwards
	if (++writePos[LR] == SAMPLE_BUFFER_LENGTH) 			writePos[LR] = 0;
	if (reverse) {
		if (--readPos[LR] < 0)							readPos[LR] = SAMPLE_BUFFER_LENGTH - 1;
		if (--oldReadPos[LR] < 0)							oldReadPos[LR] = SAMPLE_BUFFER_LENGTH - 1;
	} else {
		if (++readPos[LR] == SAMPLE_BUFFER_LENGTH)		readPos[LR] = 0;
		if (++oldReadPos[LR] == SAMPLE_BUFFER_LENGTH)		oldReadPos[LR] = 0;
	}
	if (++filterBuffPos[LR] == FIRTAPS) 					filterBuffPos[LR] = 0;


	// Get delay time from ADC or tempo clock and average over 32 readings to smooth
	if (clockValid) {
		delayClkCV = clockInterval * 48;
	} else {
		delayClkCV = static_cast<int32_t>(ADC_array[(LR == left) ? ADC_Delay_Pot_L : ADC_Delay_Pot_R]) * ((mode() == modeShort) ? 1 : SAMPLE_BUFFER_LENGTH / 65356);
	}
	dampedDelay[LR] = std::max((31 * dampedDelay[LR] + delayClkCV) >> 5, 0L);

	// If reversing delay samples read head works backwards from write position until delay time is reached and then jumps back to write position (with crossfade)
	if (reverse) {
		// Adjust read and write positions to handle circular buffer complications
		int32_t wp = writePos[LR];
		int32_t rp = readPos[LR];
		if (rp > wp) {
			if (wp - dampedDelay[LR] < 0)
				wp += (SAMPLE_BUFFER_LENGTH - 1);
			else
				rp -= (SAMPLE_BUFFER_LENGTH - 1);
		}

		// check if read position is less than write position less delay then reset read position to write position
		if (rp < wp - dampedDelay[LR] && delayCrossfade[LR] == 0) {
			oldReadPos[LR] = readPos[LR];
			readPos[LR] = writePos[LR] - 1;
			if (readPos[LR] < 0)	readPos[LR] = SAMPLE_BUFFER_LENGTH - 1;
			delayCrossfade[LR] = crossfade;
		}

	} else {
		// If delay time has changed trigger crossfade from old to new read position
		if (delayCrossfade[LR] == 0 && std::abs(dampedDelay[LR] - currentDelay[LR]) > delayHysteresis) {
			oldReadPos[LR] = readPos[LR];
			readPos[LR] = writePos[LR] - dampedDelay[LR] - 1;
			while (readPos[LR] < 0) 		readPos[LR] += SAMPLE_BUFFER_LENGTH;
			delayCrossfade[LR] = crossfade;
			currentDelay[LR] = dampedDelay[LR];
		}
	}

	// LED display
	if (++ledCounter[LR] > dampedDelay[LR]) {
		ledCounter[LR] = 0;
	}
	LED(LR, (ledCounter[LR] < crossfade));

	nextSample = clamp(nextSample, -32767L, 32767L);
	return nextSample;
}


#ifdef orig
int32_t digitalDelay::calcSample(channel LOrR) {
	int32_t nextSample, delayClkCV;

	// Cross fade if moving playback position
	if (delayCrossfade[LOrR] > 0) {
		float scale = static_cast<float>(delayCrossfade[LOrR]) / static_cast<float>(crossfade);
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

	// Add the current sample and the delayed sample scaled by the feedback control
	int32_t feedbackSample = (static_cast<int32_t>(ADC_audio[LOrR]) - adcZeroOffset[LOrR]) +
			(static_cast<float>(ADC_array[ADC_Feedback_Pot]) / 65536.0f * static_cast<float>(nextSample));

	samples[LOrR][writePos[LOrR]] = clamp(feedbackSample, -32767L, 32767L);		// Digital distortion when setting these limits much higher with high feeback (theoretically should limit to 32,767)

	// Move write and read heads one sample forwards
	if (++writePos[LOrR] == SAMPLE_BUFFER_LENGTH) 		writePos[LOrR] = 0;
	if (++readPos[LOrR] == SAMPLE_BUFFER_LENGTH)		readPos[LOrR] = 0;
	if (++oldReadPos[LOrR] == SAMPLE_BUFFER_LENGTH)		oldReadPos[LOrR] = 0;
	if (++filterBuffPos[LOrR] == FIRTAPS) 				filterBuffPos[LOrR] = 0;


	// Get delay time from ADC or tempo clock and average over 32 readings to smooth
	if (clockValid) {
		delayClkCV = clockInterval * 48;
	} else {
		delayClkCV = static_cast<int32_t>(ADC_array[(LOrR == left) ? ADC_Delay_Pot_L : ADC_Delay_Pot_R]) * ((delayMode == 1) ? 1 : SAMPLE_BUFFER_LENGTH / 65356);
	}
	dampedDelay[LOrR] = std::max((31 * dampedDelay[LOrR] + delayClkCV) >> 5, 0L);

	// If delay time has changed trigger crossfade from old to new read position
	if (delayCrossfade[LOrR] == 0 && std::abs(dampedDelay[LOrR] - currentDelay[LOrR]) > delayHysteresis) {
		oldReadPos[LOrR] = readPos[LOrR];
		readPos[LOrR] = writePos[LOrR] - dampedDelay[LOrR] - 1;
		while (readPos[LOrR] < 0) 		readPos[LOrR] += SAMPLE_BUFFER_LENGTH;
		delayCrossfade[LOrR] = crossfade;
		currentDelay[LOrR] = dampedDelay[LOrR];
	}
	nextSample = clamp(nextSample, -32767L, 32767L);
	return nextSample;
}
#endif

void digitalDelay::init() {
	dampedDelay[left] = ADC_array[ADC_Delay_Pot_L];
	dampedDelay[right] = ADC_array[ADC_Delay_Pot_R];
	calcSample(left);
	calcSample(right);
	delayCrossfade[left] = 0;
	delayCrossfade[right] = 0;
}

delay_mode digitalDelay::mode() {
	// Return setting of mode switch
	if ((GPIOE->IDR & GPIO_IDR_ID2) == 0)
		return modeReverse;
	if ((GPIOE->IDR & GPIO_IDR_ID3) == 0)
		return modeShort;
	return modeLong;
}
