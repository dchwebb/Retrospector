#include "digitaldelay.h"

extern uint32_t clockInterval;
extern bool clockValid;
extern int16_t samples[2][SAMPLE_BUFFER_LENGTH];
extern int16_t filterBuffer[2][FIRTAPS];

volatile int32_t wp;
volatile int32_t rp;

int32_t digitalDelay::calcSample(channel LOrR) {
	int32_t nextSample, delayClkCV;
	bool reverse = (mode() == modeReverse);

	// Cross fade if moving playback position
	if (delayCrossfade[LOrR] > 0) {
		if (reverse)	LED(LOrR, true);		// In reverse mode crossfade happens each sample loop so also indicates delay length
		float scale = static_cast<float>(delayCrossfade[LOrR]) / static_cast<float>(crossFade);
		nextSample = static_cast<float>(samples[LOrR][readPos[LOrR]]) * (1.0f - scale) + static_cast<float>(samples[LOrR][oldReadPos[LOrR]]) * (scale);
		--delayCrossfade[LOrR];
	} else {
		if (reverse)	LED(LOrR, false);
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

	samples[LOrR][writePos[LOrR]] = clamp(feedbackSample, -32767L, 32767L);

	// Move write and read heads one sample forwards
	if (++writePos[LOrR] == SAMPLE_BUFFER_LENGTH) 			writePos[LOrR] = 0;
	if (reverse) {
		if (--readPos[LOrR] < 0)							readPos[LOrR] = SAMPLE_BUFFER_LENGTH - 1;
		if (--oldReadPos[LOrR] < 0)							oldReadPos[LOrR] = SAMPLE_BUFFER_LENGTH - 1;
	} else {
		if (++readPos[LOrR] == SAMPLE_BUFFER_LENGTH)		readPos[LOrR] = 0;
		if (++oldReadPos[LOrR] == SAMPLE_BUFFER_LENGTH)		oldReadPos[LOrR] = 0;
	}
	if (++filterBuffPos[LOrR] == FIRTAPS) 					filterBuffPos[LOrR] = 0;


	// Get delay time from ADC or tempo clock and average over 32 readings to smooth
	if (clockValid) {
		delayClkCV = clockInterval * 48;
	} else {
		delayClkCV = static_cast<int32_t>(ADC_array[(LOrR == left) ? ADC_Delay_Pot_L : ADC_Delay_Pot_R]) * ((mode() == modeShort) ? 1 : SAMPLE_BUFFER_LENGTH / 65356);
	}
	dampedDelay[LOrR] = std::max((31 * dampedDelay[LOrR] + delayClkCV) >> 5, 0L);

	// If reversing delay samples read head works backwards from write position until delay time is reached and then jumps back to write position (with crossfade)
	if (reverse) {
		// Adjust read and write positions to handle circular buffer complications
		wp = writePos[LOrR];
		rp = readPos[LOrR];
		if (rp > wp) {
			if (wp - dampedDelay[LOrR] < 0)
				wp += (SAMPLE_BUFFER_LENGTH - 1);
			else
				rp -= (SAMPLE_BUFFER_LENGTH - 1);
		}

		// check if read position is less than write position less delay then reset read position to write position
		if (rp < wp - dampedDelay[LOrR] && delayCrossfade[LOrR] == 0) {
			oldReadPos[LOrR] = readPos[LOrR];
			readPos[LOrR] = writePos[LOrR] - 1;
			if (readPos[LOrR] < 0)	readPos[LOrR] = SAMPLE_BUFFER_LENGTH - 1;
			delayCrossfade[LOrR] = crossFade;
		}

	} else {
		// If delay time has changed trigger crossfade from old to new read position
		if (delayCrossfade[LOrR] == 0 && std::abs(dampedDelay[LOrR] - currentDelay[LOrR]) > delayHysteresis) {
			oldReadPos[LOrR] = readPos[LOrR];
			readPos[LOrR] = writePos[LOrR] - dampedDelay[LOrR] - 1;
			while (readPos[LOrR] < 0) 		readPos[LOrR] += SAMPLE_BUFFER_LENGTH;
			delayCrossfade[LOrR] = crossFade;
			currentDelay[LOrR] = dampedDelay[LOrR];
		}
	}

	nextSample = clamp(nextSample, -32767L, 32767L);
	return nextSample;
}


#ifdef orig
int32_t digitalDelay::calcSample(channel LOrR) {
	int32_t nextSample, delayClkCV;

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
		delayCrossfade[LOrR] = crossFade;
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
