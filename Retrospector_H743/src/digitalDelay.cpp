#include "digitaldelay.h"

extern uint32_t clockInterval;
extern bool clockValid;
extern int16_t samples[2][SAMPLE_BUFFER_LENGTH];
extern int16_t filterBuffer[2][FIRTAPS];

//volatile int32_t wp;
//volatile int32_t rp;

void digitalDelay::calcSample(channel LR) {
	int32_t nextSample, delayClkCV;
	bool reverse = (mode() == modeReverse);
	int32_t recordSample = static_cast<int32_t>(ADC_audio[LR]);		// Capture recording sample here to avoid jitter

	// Cross fade if moving playback position
	if (delayCrossfade[LR] > 0) {
		float scale = static_cast<float>(delayCrossfade[LR]) / static_cast<float>(crossfade);
		nextSample = static_cast<float>(samples[LR][readPos[LR]]) * (1.0f - scale) + static_cast<float>(samples[LR][oldReadPos[LR]]) * (scale);
		--delayCrossfade[LR];
	} else {
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

	// Put next output sample in I2S buffer
	SPI2->TXDR = clamp(nextSample, -32767L, 32767L);

	// Add the current sample and the delayed sample scaled by the feedback control
	int32_t feedbackSample = (recordSample - adcZeroOffset[LR]) +
			(static_cast<float>(ADC_array[ADC_Feedback_Pot]) / 65536.0f * static_cast<float>(nextSample));

	samples[LR][writePos[LR]] = clamp(feedbackSample, -32767L, 32767L);

	// Move write and read heads one sample forwards
	if (++writePos[LR] == SAMPLE_BUFFER_LENGTH) 		writePos[LR] = 0;
	if (reverse) {
		if (--readPos[LR] < 0)							readPos[LR] = SAMPLE_BUFFER_LENGTH - 1;
		if (--oldReadPos[LR] < 0)						oldReadPos[LR] = SAMPLE_BUFFER_LENGTH - 1;
	} else {
		if (++readPos[LR] == SAMPLE_BUFFER_LENGTH)		readPos[LR] = 0;
		if (++oldReadPos[LR] == SAMPLE_BUFFER_LENGTH)	oldReadPos[LR] = 0;
	}
	if (++filterBuffPos[LR] == FIRTAPS) 				filterBuffPos[LR] = 0;


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
			if (wp - calcDelay[LR] < 0)
				wp += (SAMPLE_BUFFER_LENGTH - 1);
			else
				rp -= (SAMPLE_BUFFER_LENGTH - 1);
		}

		// check if read position is less than write position less delay then reset read position to write position
		if (rp < wp - calcDelay[LR] && delayCrossfade[LR] == 0) {
			LedOffTime[LR] = SysTickVal + 50;
			oldReadPos[LR] = readPos[LR];
			readPos[LR] = writePos[LR] - 1;
			if (readPos[LR] < 0)	readPos[LR] = SAMPLE_BUFFER_LENGTH - 1;
			delayCrossfade[LR] = crossfade;
			LED(LR);
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
	}


	// Turn off delay LED after duration expired
	if (SysTickVal > LedOffTime[LR])			LED(LR, false);

	// LED display
	if (!reverse) {
		// If using external clock try to sync LEDs to tempo, allowing for drift in both directions
		++ledCounter[LR];
		if (!clockValid) {
			if (ledCounter[LR] > calcDelay[LR]) {
				ledCounter[LR] = 0;
				LED(LR);
			}
		} else {
			if (SysTickVal - lastClock < 5 && ledCounter[LR] > calcDelay[LR] - 1000) {
				ledCounter[LR] = 0;
				LED(LR);
			}
		}
	}

}


void digitalDelay::init() {
	calcDelay[left] = ADC_array[ADC_Delay_Pot_L];
	calcDelay[right] = ADC_array[ADC_Delay_Pot_R];
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

// Directly switch LED on or off
void digitalDelay::LED(channel c, bool on) {
	if (c == left) {
		if (on)	GPIOC->ODR |= GPIO_ODR_OD10;
		else {
			GPIOC->ODR &= ~GPIO_ODR_OD10;
			GPIOC->ODR &= ~GPIO_ODR_OD12;		// Debug
			LedOffTime[c] = 0;
		}
	}
	if (c == right) {
		if (on)	GPIOC->ODR |= GPIO_ODR_OD11;
		else {
			GPIOC->ODR &= ~GPIO_ODR_OD11;
			LedOffTime[c] = 0;
		}
	}
}

// Switch LED on and set off time for processing in main loop
void digitalDelay::LED(channel c) {
	if (c == left) {
		GPIOC->ODR |= GPIO_ODR_OD10;
		GPIOC->ODR |= GPIO_ODR_OD12;		// Debug
	}
	if (c == right) {
		GPIOC->ODR |= GPIO_ODR_OD11;
	}
	LedOffTime[c] = SysTickVal + 50;		// In ms
}
