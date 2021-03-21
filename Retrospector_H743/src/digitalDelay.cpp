#include "DigitalDelay.h"

extern float DACLevel;

uint32_t debugDuration = 0;


void DigitalDelay::CalcSample()
{
	static int16_t leftWriteSample;									// Holds the left sample in a temp so both writes can be done at once
	float nextSample, pingSample;									// nextSample is the delayed sample to be played (pingSample form the opposite side)
	bool reverse = (Mode() == modeReverse);

	LR = (channel)!(bool)LR;
	int32_t recordSample = static_cast<int32_t>(ADC_audio[LR]) - adcZeroOffset[LR];		// Capture recording sample here to avoid jitter
	StereoSample readSamples = {samples[readPos[LR]]};				// Get read samples as interleaved stereo
	channel RL = (LR == left) ? right : left;						// Get other channel for use in ping-pong calculations

	// If chorussing, 'dry' sample is current sample + LFO delayed chorus sample
	if (chorusMode) {

		// Capture samples for L and R channels each time to get double speed sampling
		chorusSamples[left][chorusWrite++] = ADC_audio[left];
		chorusSamples[right][chorusWrite] = ADC_audio[right];

		// Get the LFO delayed sample from the chorus buffer
		chorusLFO[LR] += chorusAdd[LR];
		if (chorusLFO[LR] > CHORUS_MAX || chorusLFO[LR] < CHORUS_MIN) {
			chorusAdd[LR] *= -1;
		}
		uint16_t chorusRead = chorusWrite - static_cast<uint16_t>(chorusLFO[LR]) - 1;

		// Low pass filter the LFO delayed sample
		float filteredChorus = chorusFilter[LR].FilterSample(chorusSamples[LR][chorusRead] - adcZeroOffset[LR]);

		recordSample = 0.8f * (recordSample + filteredChorus);		// FIXME - scaling factor needs to be more scientific
	}


//	TIM3->CNT = 0;		// Debug

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
		// FIXME levels are somewhat arbitrary
		nextSample = 0.75f * nextSample + 0.4f * pingSample;
	}

	// Filter output
	nextSample = filter.CalcFilter(nextSample, LR);

	// Compression
	if (nextSample > threshold || nextSample < -threshold) {
		int8_t thresholdSign = (nextSample < -threshold ? -1 : 1);
		int32_t excess = abs(nextSample - thresholdSign * threshold);
		nextSample = (threshold + (ratio * excess) / (ratio + excess)) * thresholdSign;
	}


	// Put next output sample in I2S buffer (OutputMix will set VCA mix levels in normal mode or apply a software mix for chorus mode)
	SPI2->TXDR = OutputMix(recordSample, nextSample);

	// Add the current sample and the delayed sample scaled by the feedback control
	int32_t feedbackSample = recordSample +	(static_cast<float>(ADC_array[ADC_Feedback_Pot]) / 65536.0f * nextSample);

	// Hold the left sample in a temporary variable and write both left and right samples when processing right signal
	if (LR == left) {
		leftWriteSample = static_cast<uint16_t>(std::clamp(feedbackSample, -32767L, 32767L));
	} else {
		StereoSample writeSample;
		writeSample.sample[left] = leftWriteSample;
		writeSample.sample[right] = static_cast<uint16_t>(std::clamp(feedbackSample, -32767L, 32767L));
		samples[writePos] = writeSample.bothSamples;

		if (++writePos == SAMPLE_BUFFER_LENGTH) 		writePos = 0;

	}



	// Move write and read heads one sample forwards
	if (reverse) {
		if (--readPos[LR] < 0)							readPos[LR] = SAMPLE_BUFFER_LENGTH - 1;
		if (--oldReadPos[LR] < 0)						oldReadPos[LR] = SAMPLE_BUFFER_LENGTH - 1;
	} else {
		if (++readPos[LR] == SAMPLE_BUFFER_LENGTH)		readPos[LR] = 0;
		if (++oldReadPos[LR] == SAMPLE_BUFFER_LENGTH)	oldReadPos[LR] = 0;
	}

	// Check if clock received
	if ((GPIOA->IDR & GPIO_IDR_ID7) == GPIO_IDR_ID7) {
		if (!clockHigh) {
			clockError = delayCounter - (lastClock + clockInterval);
			clockInterval = delayCounter - lastClock - 85;			// FIXME constant found by trial and error - probably relates to filtering group delay
			lastClock = delayCounter;
			clockHigh = true;
		}
	} else {
		clockHigh = false;
	}
	clockValid = (delayCounter - lastClock < (SAMPLE_RATE * 2));			// Valid clock interval is within a second
	++delayCounter;

	// Get delay time from ADC or tempo clock and average over 32 readings to smooth
	int32_t delayClkCV = static_cast<int32_t>(ADC_array[(LR == left) ? ADC_Delay_Pot_L : ADC_Delay_Pot_R]);
	if (clockValid) {
		if (abs(delayPotVal[LR] - delayClkCV) > tempoHysteresis) {
			delayPotVal[LR] = delayClkCV;											// Store value for hysteresis checking
			delayMult[LR] = tempoMult[tempoMult.size() * delayClkCV / 65536];		// get tempo multiplier from lookup
		}
		calcDelay[LR] = delayMult[LR] * (clockInterval / 2);

	} else {
		if (Mode() != modeShort)
			delayClkCV *= SAMPLE_BUFFER_LENGTH / 65356;
		calcDelay[LR] = std::max((31 * calcDelay[LR] + delayClkCV) >> 5, 0L);
	}


	// If reversing delay samples read head works backwards from write position until delay time is reached and then jumps back to write position (with crossfade)
	if (reverse) {
		// Create temporary read and write positions that handle circular buffer complications
		int32_t wp = writePos;
		int32_t rp = readPos[LR];
		if (rp > wp) {
			if (wp - (calcDelay[LR] * 2) < 0)
				wp += (SAMPLE_BUFFER_LENGTH - 1);
			else
				rp -= (SAMPLE_BUFFER_LENGTH - 1);
		}
		int32_t remainingDelay = rp + (calcDelay[LR] * 2) - wp;

		// Scale the LED so it fades in with the timing of the reverse delay
		ReverseLED(LR, remainingDelay);

		// check if read position is less than write position less delay then reset read position to write position
		if (remainingDelay < 0 && delayCrossfade[LR] == 0) {
			ledOffTime[LR] = SysTickVal + 50;
			oldReadPos[LR] = readPos[LR];
			readPos[LR] = writePos - 1;
			if (readPos[LR] < 0)	readPos[LR] = SAMPLE_BUFFER_LENGTH - 1;
			delayCrossfade[LR] = crossfade;
		}

	} else {
		// If delay time has changed trigger crossfade from old to new read position
		if (delayCrossfade[LR] == 0 && std::abs(calcDelay[LR] - currentDelay[LR]) > delayHysteresis) {
			oldReadPos[LR] = readPos[LR];
			readPos[LR] = writePos - calcDelay[LR] - 1;
			while (readPos[LR] < 0) 		readPos[LR] += SAMPLE_BUFFER_LENGTH;
			delayCrossfade[LR] = crossfade;
			currentDelay[LR] = calcDelay[LR];
		}
		UpdateLED(LR);
	}


//	time = TIM3->CNT;
//	if (time > debugDuration)
//		debugDuration = time;		// Debug

}


int32_t DigitalDelay::OutputMix(float drySample, float wetSample)
{
	// Output mix level
	float dryLevel = static_cast<float>(ADC_array[ADC_Mix]) / 65536.0f;		// Convert 16 bit int to float 0 -> 1
	float outputSample;

	// As mixing is done in software in chorus mode add current chorused sample to delay sample
	if (chorusMode) {
		float multWet = 1.0f - std::pow(dryLevel * 1.05, 2.0f);
		float multDry = 1.0f - std::pow(1 - (dryLevel * 1.05), 2.0f);

		outputSample = (multWet * wetSample) + (multDry * drySample);
	} else {
		DAC1->DHR12R2 = (1.0f - dryLevel) * 4095.0f;		// Wet level
		DAC1->DHR12R1 = dryLevel * 4095.0f;					// Dry level

		outputSample = wetSample;
	}
	return std::clamp(static_cast<int32_t>(outputSample), -32767L, 32767L);
}


void DigitalDelay::ChorusMode(bool on)
{
	// chorus mixing is handled in software so set VCA to fully wet
	if (on) {
		DAC1->DHR12R2 = 4095;								// Wet level
		DAC1->DHR12R1 = 0;									// Dry level
	}
	chorusMode = on;
}


void DigitalDelay::Init()
{
	// Clear sample buffers
	std::fill_n(samples, SAMPLE_BUFFER_LENGTH, 0);
	std::fill_n(chorusSamples[left], 65536, adcZeroOffset[left]);
	std::fill_n(chorusSamples[right], 65536, adcZeroOffset[right]);

	// Calculate delays once to avoid delays when starting I2S interrupts
	calcDelay[left] = ADC_array[ADC_Delay_Pot_L];
	calcDelay[right] = ADC_array[ADC_Delay_Pot_R];
	CalcSample();
	CalcSample();
	delayCrossfade[left] = 0;
	delayCrossfade[right] = 0;
}


void DigitalDelay::UpdateLED(channel c)
{
	// Turns on LED delay time indicator, locking as accurately as possible to external tempo clock
	++ledCounter[c];
	if (clockValid) {
		// If using external clock try to sync LEDs to tempo, allowing for drift in both directions
		if (delayMult[c] < 1.0f) {
			if (delayCounter - lastClock < 1000 && ledOffTime[c] < SysTickVal) {		// Always trigger on the clock, unless LED already on
				ledFraction[c] = 1;
				LedOn(c);
			} else if (delayMult[c] * ledFraction[c] < 1.0f && ledCounter[c] > calcDelay[c]) {	// Handle fractional times
				ledFraction[c]++;
				LedOn(c);
			}
		} else if (delayCounter - lastClock < 5000 && ledCounter[c] > calcDelay[c] - 1000) { 		// 1000 ~= 20.8ms: handle tempos slower than 1
			LedOn(c);
		}
	} else if (ledCounter[c] > calcDelay[c]) {
		LedOn(c);
	}

	// Turn off delay LED after duration expired
	if (SysTickVal > ledOffTime[c]) {
		LedOff(c);
	}
}


// Scale the LED so it fades in with the timing of the reverse delay
void DigitalDelay::ReverseLED(channel c, int32_t remainingDelay)
{
	float rc = (float)remainingDelay / (calcDelay[c] * 2);
	ledCounter[c]++;
	if (ledCounter[c] >= pow(rc, 2.0f) * 120) {
		LedOn(c);
	} else {
		LedOff(c);
	}
}


// Switch LED on and set off time
void DigitalDelay::LedOn(channel c)
{
	if (c == left) {
		GPIOC->ODR |= GPIO_ODR_OD10;
	}
	if (c == right) {
		GPIOC->ODR |= GPIO_ODR_OD11;
	}
	ledCounter[c] = 0;
	ledOffTime[c] = SysTickVal + 50;		// In ms
}


// Directly switch LED on or off
void DigitalDelay::LedOff(channel c)
{
	if (c == left) {
		GPIOC->ODR &= ~GPIO_ODR_OD10;
	}
	if (c == right) {
		GPIOC->ODR &= ~GPIO_ODR_OD11;
	}
	ledOffTime[c] = 0;
}


delay_mode DigitalDelay::Mode()
{
	// Return setting of Mode switch
	if ((GPIOE->IDR & GPIO_IDR_ID2) == 0)
		return modeReverse;
	if ((GPIOE->IDR & GPIO_IDR_ID3) == 0)
		return modeShort;
	return modeLong;
}
