#include "DigitalDelay.h"

void DigitalDelay::CalcSample()
{
	StereoSample readSamples, oldReadSamples;						// Delayed samples as interleaved stereo, second var for crossfading
	float nextSample, oppositeSample = 0.0f;						// nextSample is the delayed sample to be played (oppositeSample from the opposite side)
	static int16_t leftWriteSample;									// Holds the left sample in a temp so both writes can be done at once
	LR = static_cast<channel>(!static_cast<bool>(LR));
	channel RL = (LR == left) ? right : left;						// Get other channel for use in stereo widening calculations
	delay_mode delayMode = Mode();									// Long, short or reverse
	int32_t recordSample = GateSample();							// Capture recording sample
	int32_t modReadPos, modReadPosNext;								// Positions of two samples (for interpolation) when using modulated delay

	if (modulatedDelay) {
		// Using modulated delay - update offset accounting for circular buffer wrapping
		modOffset[LR] += modOffsetAdd[LR];
		if (modOffset[LR] > modOffsetMax || modOffset[LR] < 1.0f) {
			modOffsetAdd[LR] *= -1;
		}
		modReadPos = readPos[LR] - static_cast<int32_t>(modOffset[LR]);
		if (modReadPos < 0) {
			modReadPos = modReadPos + SAMPLE_BUFFER_LENGTH;
		}
		readSamples = {samples[modReadPos]};
		modReadPosNext = modReadPos + 1;
		if (modReadPosNext == SAMPLE_BUFFER_LENGTH) {
			modReadPosNext = 0;
		}
	} else {
		readSamples = {samples[readPos[LR]]};
	}

	// Test modes
	if (testMode != TestMode::none) {
		RunTest(recordSample);
		return;
	}


	// Cross fade if moving playback position
	if (delayCrossfade[LR] > 0) {
		if (modulatedDelay) {
			modReadPos = oldReadPos[LR] - static_cast<int32_t>(modOffset[LR]);
			if (modReadPos < 0) {
				modReadPos = modReadPos + SAMPLE_BUFFER_LENGTH;
			}
			oldReadSamples = {samples[modReadPos]};
		} else {
			oldReadSamples = {samples[oldReadPos[LR]]};
		}
		float scale = static_cast<float>(delayCrossfade[LR]) / static_cast<float>(crossfade);
		nextSample = static_cast<float>(readSamples.sample[LR]) * (1.0f - scale) + static_cast<float>(oldReadSamples.sample[LR]) * (scale);
		oppositeSample = static_cast<float>(readSamples.sample[RL]) * (1.0f - scale) + static_cast<float>(oldReadSamples.sample[RL]) * (scale);
		--delayCrossfade[LR];
	} else {
		if (modulatedDelay) {
			StereoSample readSamples2 = {samples[modReadPosNext]};			// Get next sample for interpolation
			float offsetFraction = modOffset[LR] - std::round(modOffset[LR]);
			nextSample = static_cast<float>(readSamples.sample[LR]) * (1.0f - offsetFraction) + static_cast<float>(readSamples2.sample[LR]) * offsetFraction;
		} else {
			nextSample = static_cast<float>(readSamples.sample[LR]);
			oppositeSample = static_cast<float>(readSamples.sample[RL]);
		}
	}

	// Add in a scaled amount of the sample from the opposite stereo channel - Levels are somewhat arbitrary
	if (stereoWide) {
		nextSample = 0.75f * nextSample + 0.4f * oppositeSample;
	}


	nextSample = filter.CalcFilter(nextSample, LR);

	// Compression
	if (tanhCompression) {
		nextSample = FastTanh(nextSample / 40000.0) * 40000.0;
	} else 	if (nextSample > threshold || nextSample < -threshold) {
		int8_t thresholdSign = (nextSample < -threshold ? -1 : 1);
		int32_t excess = abs(nextSample - thresholdSign * threshold);
		nextSample = (threshold + (ratio * excess) / (ratio + excess)) * thresholdSign;
	}


	SPI2->TXDR = OutputMix(nextSample);


	// Add the current sample and the delayed sample scaled by the feedback control
	int32_t feedbackSample = recordSample +	(static_cast<float>(1.1f * (ADC_array[ADC_Feedback_Pot]) / 65536.0f) * nextSample);

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
	if (delayMode == modeReverse) {
		if (--readPos[LR] < 0)							readPos[LR] = SAMPLE_BUFFER_LENGTH - 1;
		if (--oldReadPos[LR] < 0)						oldReadPos[LR] = SAMPLE_BUFFER_LENGTH - 1;
	} else {
		if (++readPos[LR] == SAMPLE_BUFFER_LENGTH)		readPos[LR] = 0;
		if (++oldReadPos[LR] == SAMPLE_BUFFER_LENGTH)	oldReadPos[LR] = 0;
	}

	// Check if clock received
	if ((GPIOB->IDR & GPIO_IDR_ID2) == GPIO_IDR_ID2) {
		if (!clockHigh) {
			clockInterval = delayCounter - lastClock - 85;			// FIXME constant found by trial and error - probably relates to filtering group delay
			lastClock = delayCounter;
			clockHigh = true;
		}
	} else {
		clockHigh = false;
	}
	clockValid = (delayCounter - lastClock < (SAMPLE_RATE * 2));					// Valid clock interval is within a second
	++delayCounter;



	// Calculate delay times - either clocked or not, with link button making right delay a multiple of left delay/clock
	//int32_t delayClkCV = static_cast<int32_t>(ADC_array[(LR == left) ? ADC_Delay_Pot_L : ADC_Delay_Pot_R]);
	int32_t delayClkCV = DelayCV(LR);												// Pot and CV combined
	float hysteresisDivider = (delayMode != modeShort) ? 8 : 1;						// Hysteresis on delay time changes must be scaled to avoid firing continually on long delays
	if (clockValid) {
		if (!linkLR && LR == right) {
			if (delayMode != modeShort) {
				delayClkCV *= longDelMult;
			}
			calcDelay[LR] = std::max((31 * calcDelay[LR] + delayClkCV) >> 5, 0L);	// Average over 32 readings to smooth

		} else if (abs(delayPotVal[LR] - delayClkCV) > tempoHysteresis) {
			delayPotVal[LR] = delayClkCV;											// Store value for hysteresis checking
			delayMult[LR] = tempoMult[tempoMult.size() * delayClkCV / 65536];		// get tempo multiplier from lookup
			if (delayMode != modeShort) {
				delayMult[LR] *= longDelMult;
			}
			calcDelay[LR] = delayMult[LR] * (clockInterval / 2);
			hysteresisDivider = delayMult[LR];
		}

	} else {
		// If link tempo button active, right delay is multiple of left delay
		if (linkLR && LR == right) {
			//delayMult[left] = tempoMult[tempoMult.size() * ADC_array[ADC_Delay_Pot_L] / 65536];		// Get the equivalent multiplier for Left delay
			delayMult[left] = tempoMult[tempoMult.size() * DelayCV(left) / 65536];		// Get the equivalent multiplier for Left delay
			int32_t leftDelScaled = currentDelay[left] / delayMult[left];			// Normalise the left delay
			delayMult[right] = tempoMult[tempoMult.size() * delayClkCV / 65536];	// Get the multiplier for the right delay
			calcDelay[right] = delayMult[right] * leftDelScaled;					// Apply the right delay multiplier to the normalised left delay
			hysteresisDivider *= delayMult[right] / delayMult[left];
		} else {
			if (delayMode != modeShort) {
				delayClkCV *= longDelMult;
			}
			calcDelay[LR] = std::max((31 * calcDelay[LR] + delayClkCV) >> 5, 0L);
		}
	}


	// If reversing delay samples read head works backwards from write position until delay time is reached and then jumps back to write position (with crossfade)
	if (delayMode == modeReverse) {
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
		UpdateLED(LR, true, std::max(remainingDelay, 0L));

		// check if read position is less than write position less delay then reset read position to write position
		if (remainingDelay < 0 && delayCrossfade[LR] == 0) {
			oldReadPos[LR] = readPos[LR];
			readPos[LR] = writePos - 1;
			if (readPos[LR] < 0)	readPos[LR] = SAMPLE_BUFFER_LENGTH - 1;
			delayCrossfade[LR] = crossfade;
		}

	} else {
		// If delay time has changed trigger crossfade from old to new read position
		if (delayCrossfade[LR] == 0 && std::abs(calcDelay[LR] - currentDelay[LR]) / hysteresisDivider > delayHysteresis) {
			oldReadPos[LR] = readPos[LR];
			readPos[LR] = writePos - calcDelay[LR] - 1;
			while (readPos[LR] < 0) 		readPos[LR] += SAMPLE_BUFFER_LENGTH;
			delayCrossfade[LR] = crossfade;
			currentDelay[LR] = calcDelay[LR];
		}
		UpdateLED(LR, false);
	}
}



int32_t DigitalDelay::OutputMix(float wetSample)
{
	// Output mix level
	float dryLevel = static_cast<float>(ADC_array[ADC_Mix]) / 65536.0f;		// Convert 16 bit int to float 0 -> 1

	DAC1->DHR12R2 = (1.0f - dryLevel) * 4095.0f;		// Wet level
	DAC1->DHR12R1 = dryLevel * 4095.0f;					// Dry level

	int16_t outputSample = std::clamp(static_cast<int32_t>(wetSample), -32767L, 32767L);

	return outputSample;
}



int32_t DigitalDelay::GateSample()
{
	int32_t recordSample = static_cast<int32_t>(ADC_array[LR]) - adcZeroOffset[LR];
	if (gateThreshold == 0) {
		return recordSample;
	}

	if (std::abs(recordSample) < gateThreshold) {
		overThreshold[LR] = 0;
		if (belowThresholdCount[LR] > gateHoldCount) {				// there have been enough samples around zero
			gateShut[LR] = gateStatus::closed;
			return 0;
		} else {
			if (belowThresholdCount[LR] > gateHoldCount / 2) {		// Halfway through the fade out count start closing the gate
				float fadeout = 2.0f - (2.0f * belowThresholdCount[LR]) / gateHoldCount;
				recordSample = static_cast<float>(recordSample) * fadeout;
				gateShut[LR] = gateStatus::closing;
			}
			belowThresholdCount[LR]++;
		}

	} else  {
		if (overThreshold[LR] > 0 && std::abs(recordSample) > gateThreshold + 100) {		// Last two samples have exceeded threshold in same direction
			belowThresholdCount[LR] = 0;
			gateShut[LR] = gateStatus::open;
		} else {
			if (std::abs(recordSample) > gateThreshold + 100) {		// Use hysteresis to avoid gate continually opening and closing on continous low level signals
				overThreshold[LR] = recordSample;
			}
			if (belowThresholdCount[LR] > gateHoldCount) {			// there have been enough samples around zero
				gateShut[LR] = gateStatus::closed;
				return 0;
			}
		}
	}

	return recordSample;
}



void DigitalDelay::Init()
{
	std::fill_n(samples, SAMPLE_BUFFER_LENGTH, 0);					// Clear sample buffers

	// Calculate delays once to avoid delays when starting I2S interrupts
	calcDelay[left] = ADC_array[ADC_Delay_Pot_L];
	calcDelay[right] = ADC_array[ADC_Delay_Pot_R];
	CalcSample();
	CalcSample();
	delayCrossfade[left] = 0;
	delayCrossfade[right] = 0;
	modOffsetAdd[left] = modOffsetInc;
	modOffsetAdd[right] = -1 * modOffsetInc;
	modOffset[left] = modOffsetMax / 2;
	modOffset[right] = modOffsetMax / 2;
}

float brightness;

void DigitalDelay::UpdateLED(channel c, bool reverse, int32_t remainingDelay)
{
	ledOnTimer++;

	// Carry out counter calculations each loop even if not sending
	if (reverse) {
		brightness = std::pow(1.0f - static_cast<float>(remainingDelay) / (calcDelay[c] * 2), 4.0f);
	} else {
		// Turns on LED delay time indicator, locking as accurately as possible to external tempo clock
		++ledCounter[c];
		if (clockValid && (linkLR || LR == left)) {
			// If using external clock try to sync LEDs to tempo, allowing for drift in both directions
			if (delayMult[c] < 1.0f) {
				if (delayCounter - lastClock < 500) {				// Always trigger on the clock, unless LED already on
					ledFraction[c] = 1;
					ledCounter[c] = 0;
				} else if (delayMult[c] * ledFraction[c] < 1.0f && ledCounter[c] > calcDelay[c]) {	// Handle fractional times
					ledFraction[c]++;
					ledCounter[c] = 0;
				}
			} else if (delayCounter - lastClock < 5000 && ledCounter[c] > calcDelay[c] - 1000) { 	// handle tempos slower than 1 (1000 ~= 20.8ms)
				ledCounter[c] = 0;
			}
		} else if (ledCounter[c] > calcDelay[c]) {
			ledCounter[c] = 0;
		}

		/* Debug - outputs pulse for checking synch to external clock input
		if (c == left) {
			if (ledCounter[c] < 50) {
				GPIOB->ODR |= GPIO_ODR_OD8;			// Toggle LED for debugging
			} else if (ledCounter[c] > 200) {
				GPIOB->ODR &= ~GPIO_ODR_OD8;		// Toggle LED for debugging
			}
		}*/

		// For very short delays just display a continuous colour
		if (calcDelay[c] < 1000) {
			brightness = 0.06f;
		} else {
			brightness = 1.0f - std::pow(2.0f * ledCounter[c] / calcDelay[c], 0.1f);					// Exponential fade out
		}

	}

	enum ledColours {
		ledColourLeft1 = 0xFF1206,
		ledColourLeft2 = 0xF42204,
		ledColourRight1 = 0x44FF12,
		ledColourRight2 = 0x05BB22,
		ledColourClock1 = 0x00FF33,
		ledColourClock2 = 0x0033FF,
	};

	// Pause to give LEDs enough time to receive SPI packet
	if (ledOnTimer > 8) {

		float lengthScale = std::min(1.0f, static_cast<float>(calcDelay[c]) / 65536.0f);
		if (clockValid) {
			if (c == right && !linkLR) {
				led.LEDColour(c == left ? ledDelL : ledDelR, ledColourLeft1, ledColourLeft2, lengthScale, brightness);
			} else {
				led.LEDColour(c == left ? ledDelL : ledDelR, ledColourClock1, ledColourClock2, lengthScale, brightness);
			}
		} else {
			if (c == right && !linkLR) {
				led.LEDColour(ledDelR, ledColourRight1, ledColourRight2, lengthScale, brightness);
			} else {
				led.LEDColour(c == left ? ledDelL : ledDelR, ledColourLeft1, ledColourLeft2, lengthScale, brightness);
			}
		}

		// Filter LED can be configured to display status of gate
		if (gateLED) {
			switch (gateShut[left]) {
			case gateStatus::open:
				led.LEDColour(ledFilter, 0x00FF00);
				break;
			case gateStatus::closed:
				led.LEDColour(ledFilter, 0xFF0000);
				break;
			case gateStatus::closing:
				led.LEDColour(ledFilter, 0xFFAA00);
				break;
			}
		}

		led.LEDSend();
		ledOnTimer = 0;
	}
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



void DigitalDelay::CheckSwitches()
{
	static uint32_t linkBtnTest;
	static bool linkButton;

	// Implement modulated delay (PG10)/stereo wide (PC12) switch, and link button (PB4) for delay LR timing
	if (((GPIOG->IDR & GPIO_IDR_ID10) == 0) != modulatedDelay) {
		modulatedDelay = !modulatedDelay;

		// Trigger a cross-fade when switching in and out of modulated delay to avoid pops and clicks
		if (modulatedDelay) {
			oldReadPos[left] = readPos[left];
			oldReadPos[right] = readPos[LR];
		} else {
			oldReadPos[left] = readPos[left] - static_cast<int32_t>(modOffset[left]);
			if (oldReadPos[left] < 0) 		oldReadPos[left] = oldReadPos[left] + SAMPLE_BUFFER_LENGTH;
			oldReadPos[right] = readPos[right] - static_cast<int32_t>(modOffset[right]);
			if (oldReadPos[right] < 0) 		oldReadPos[right] = oldReadPos[right] + SAMPLE_BUFFER_LENGTH;
		}
		delayCrossfade[right] = crossfade;
		delayCrossfade[left] = crossfade;
	}
	if (((GPIOC->IDR & GPIO_IDR_ID12) == 0) != stereoWide) {
		stereoWide = !stereoWide;
	}
	if (SysTickVal > linkBtnTest + 1) {			// Test if link button pressed with some debouncing
		if ((GPIOB->IDR & GPIO_IDR_ID4) == 0) {
			if (!linkButton) {
				linkLR = !linkLR;
				linkButton = true;
			}
		} else {
			linkButton = false;
		}
		linkBtnTest = SysTickVal;
	}
}



// Algorithm source: https://varietyofsound.wordpress.com/2011/02/14/efficient-tanh-computation-using-lamberts-continued-fraction/
float DigitalDelay::FastTanh(float x)
{
	float x2 = x * x;
	float a = x * (135135.0f + x2 * (17325.0f + x2 * (378.0f + x2)));
	float b = 135135.0f + x2 * (62370.0f + x2 * (3150.0f + x2 * 28.0f));
	return a / b;
}


// Combined calculation for delay potentiometer and CV inputs
inline int32_t DigitalDelay::DelayCV(channel c) {
	if (c == left) {
		return std::min(static_cast<int32_t>(ADC_array[ADC_Delay_Pot_L]) + 65536L - ADC_array[ADC_Delay_CV_L], 65535L);
	} else {
		return std::min(static_cast<int32_t>(ADC_array[ADC_Delay_Pot_R]) + 65536L - ADC_array[ADC_Delay_CV_R], 65535L);
	}
}


// Runs audio tests (audio loopback and 1kHz saw wave)
void DigitalDelay::RunTest(int32_t sample)
{
	static int16_t testSample;
	switch (testMode) {
	case TestMode::none:
		break;
	case TestMode::loop: {
		// Capture the samples to the buffer
		StereoSample writeSample;
		writeSample.sample[left] = static_cast<int32_t>(ADC_array[left]) - adcZeroOffset[left];
		writeSample.sample[right] = static_cast<int32_t>(ADC_array[right]) - adcZeroOffset[right];
		samples[writePos] = writeSample.bothSamples;
		if (++writePos == SAMPLE_BUFFER_LENGTH) 		writePos = 0;

		SPI2->TXDR = OutputMix(sample);
		break;
	}
	case TestMode::saw:
		testSample += 683;		// 65536/96000 * 1kHz
		SPI2->TXDR = OutputMix(testSample);
		break;
	}
}


