#include "Filter.h"

bool calculatingFilter = false;			// Debug

GpioPin switchLP	{GPIOC, 10, GpioPin::Type::Input};			// PC10: LP switch, low when in LP mode
GpioPin switchHP	{GPIOC, 11, GpioPin::Type::Input};			// PC11: HP switch

void Filter::Init()
{
	filter.FIRFilterWindow();
	filter.Update(true);
}

void Filter::Update(bool reset)
{
	// In the middle of changing filter type using soft switching
	if (newFilterType != filterType) {
		return;
	}

	// Check if filter mode has been changed [PC10 = 0: LP; PC11 = 0: HP; PC10 and PC11 = 1: FIR Sweep]
	if (switchLP.IsLow()) {
		if (filterControl != LP) {
			newFilterType = IIR;
			newFilterControl = LP;
		}
	} else if (switchHP.IsLow()) {
		if (filterControl != HP) {
			newFilterType = IIR;
			newFilterControl = HP;
		}
	} else if (filterControl != Both) {
		newFilterType = FIR;
		newFilterControl = Both;
	}

	// get filter values from pot and CV and smooth through fixed IIR filter
	dampedADC = filterADC.FilterSample(std::min(static_cast<uint32_t>(ADC_array[ADC_Filter_Pot]) + (65535UL - ADC_array[ADC_Filter_CV]), 65535UL));

	if (newFilterType != filterType || reset || std::abs(dampedADC - previousADC) > hysteresis) {
		calculatingFilter = true;

		previousADC = dampedADC;
		if (newFilterType == IIR) {
			InitIIRFilter(dampedADC);
		} else {
			InitFIRFilter(dampedADC);
		}
		calculatingFilter = false;

		// If not changing filter type switch active filter
		if (newFilterType == filterType) {
			activeFilter = !activeFilter;
		} else {
			softSwitchTime = softSwitchDefault;
		}

		// Update filter LED
		if (newFilterControl == Both) {
			if (dampedADC < cfg.potCentre) {	// LP
				float colourMult = std::pow(dampedADC / cfg.potCentre, 2.0f);
				led.LEDColour(ledFilter, 0xFF99BB, 0xFF0000, colourMult, 1.0f);
			} else {
				float colourMult = std::pow((dampedADC - cfg.potCentre) / cfg.potCentre, 0.5f);
				led.LEDColour(ledFilter, 0x0000FF, 0xBB99FF, colourMult, 1.0f);
			}
		} else 	if (newFilterControl == LP) {
			float colourMult = std::pow(dampedADC / 65535.0f, 4.0f);
			led.LEDColour(ledFilter, 0xFF5555, 0xFF0000, colourMult, 1.0f);
		} else {
			float colourMult = std::pow(1.0f - (dampedADC / 65535.0f), 4.0f);
			led.LEDColour(ledFilter, 0x5555FF, 0x1100FF, colourMult, 1.0f);
		}
	}
}


void Filter::SwitchFilter()
{
	filterType = newFilterType;
	filterControl = newFilterControl;

	if (filterControl == HP) {
		passType = HighPass;
		iirHPReg[left].Init();
		iirHPReg[right].Init();
	}
	if (filterControl == LP) {
		passType = LowPass;
		iirLPReg[left].Init();
		iirLPReg[right].Init();
	}
	activeFilter = !activeFilter;
}


float Filter::CalcFilter(float sample, channel c)
{
	float outputSample;

	if (activateFilter) {
		if (filterType == IIR) {
			// Store the sample to the filter buffer so that switching is smoother
			filterBuffer[c][filterBuffPos[c]] = sample;
			++filterBuffPos[c];

			outputSample = static_cast<float>(filter.CalcIIRFilter(sample, c));
		} else {
			outputSample = filter.CalcFIRFilter(sample, c);
		}

		// Handle soft-switching to cross fade between old and new filter types to avoid pops and clicks
		if (softSwitchTime > 0) {
			float softSwitchProp = static_cast<float>(softSwitchTime) / softSwitchDefault;
			if (softSwitchProp > 0.5f) {											// Fade out old filter
				outputSample = ((softSwitchProp * 2.0f) - 1.0f) * outputSample;
			} else {
				if (softSwitchProp == 0.5f) {										// Activate new filter
					SwitchFilter();
				}
				outputSample = (1.0f - (softSwitchProp * 2.0f)) * outputSample;		// Fade in new filter
			}
			--softSwitchTime;
		}


		return outputSample;
	} else {
		return sample;
	}
}


// Rectangular FIR
void Filter::InitFIRFilter(float tone)
{
	float omega;
	int8_t arg;

	// Pass in smoothed ADC reading - generate appropriate omega sweeping from Low pass to High Pass
	if (tone < cfg.potCentre - 1000) {		// Low Pass
		passType = LowPass;
		omega = 1.0f - std::pow((cfg.potCentre - tone) / 33000.0f, 0.2f);
	} else if (tone > cfg.potCentre + 1000) {
		passType = HighPass;
		omega = 1.0f - std::pow((tone - cfg.potCentre)  / 75000.0f, 3.0f);
	} else {
		passType = FilterOff;
		omega = 1.0f;
	}

	// cycle between two sets of coefficients so one can be changed without affecting the other
	bool inactiveFilter = !activeFilter;

	if (passType == LowPass) {
		for (int8_t j = 0; j < cfg.firTaps / 2 + 1; ++j) {
			arg = j - cfg.firTaps / 2;
			firCoeff[inactiveFilter][j] = omega * Sinc(omega * arg * M_PI) * winCoeff[j];
		}
	} else if (passType == HighPass)  {
		int8_t sign = 1;
		for (int8_t j = 0; j < cfg.firTaps / 2 + 1; ++j) {
			arg = j - cfg.firTaps / 2;
			firCoeff[inactiveFilter][j] = sign * omega * Sinc(omega * arg * M_PI) * winCoeff[j];
			sign = sign * -1;
		}
	}

	currentCutoff = omega;
}


// Convolution routine for delayed samples - takes current sample, buffers, convolves and returns filtered sample
float Filter::CalcFIRFilter(float sample, channel c)
{
	float outputSample = 0.0;

	filterBuffer[c][filterBuffPos[c]] = sample;
	if (currentCutoff == 1.0f) {		// If not filtering take middle most sample to account for FIR group delay when filtering active (gives more time for main loop when filter inactive)
		uint8_t mid = filterBuffPos[c] - (cfg.firTaps / 2);
		outputSample = filterBuffer[c][mid];
	} else {
		uint8_t pos, revpos;

		pos = filterBuffPos[c] - cfg.firTaps + 1;		// position of sample 1, 2, 3 etc
		revpos = filterBuffPos[c];					// position of sample N, N-1, N-2 etc

		for (uint8_t i = 0; i < cfg.firTaps / 2; ++i) {
			// Folded FIR structure - as coefficients are symmetrical we can multiple the sample 1 + sample N by the 1st coefficient, sample 2 + sample N - 1 by 2nd coefficient etc
			outputSample += firCoeff[activeFilter][i] * (filterBuffer[c][pos++] + filterBuffer[c][revpos--]);
		}

		outputSample += firCoeff[activeFilter][cfg.firTaps / 2] * filterBuffer[c][pos];
	}

	++filterBuffPos[c];		// FIXME - probably need only one position, incremented on right sample
	return outputSample;
}


float Filter::Sinc(float x)
{
	if (x > -1.0E-5 && x < 1.0E-5)
		return(1.0);
	return (std::sin(x) / x);
}


void Filter::FIRFilterWindow()
{
	float arg;
	constexpr float beta = 0.4f;			// between 0.0 and 10.0

	// Kaiser window
	for (uint8_t j = 0; j < cfg.firTaps; j++) {
		arg = beta * sqrt(1.0 - pow( (static_cast<float>(2 * j) + 1 - cfg.firTaps) / (cfg.firTaps + 1), 2.0) );
		winCoeff[j] = Bessel(arg) / Bessel(beta);
	}
}


// Used for Kaiser window calculations
float Filter::Bessel(float x)
{
	float sum = 0.0, xPower;
	int factorial;
	for (uint8_t i = 1; i < 10; ++i) {
		xPower = pow(x / 2.0, static_cast<float>(i));
		factorial = 1;
		for (uint8_t j = 1; j <= i; ++j) {
			factorial *= j;
		}
		sum += pow(xPower / static_cast<float>(factorial), 2.0);
	}
	return(1.0 + sum);
}


void Filter::InitIIRFilter(iirdouble_t tone)
{
	iirdouble_t cutoff;
	constexpr iirdouble_t LPMax = 0.995;
	constexpr iirdouble_t HPMin = 0.001;

	bool inactiveFilter = !activeFilter;

	if (newFilterControl == HP) {				// Want a sweep from 0.03 to 0.99 with most travel at low end
		//newPassType = HighPass;
		cutoff = pow((tone / 100000.0), 3.0) + HPMin;
		iirHPFilter[inactiveFilter].CalcCoeff(cutoff);
	} else {		// Want a sweep from 0.001 to 0.2-0.3
		//newPassType = LowPass;
		cutoff = std::min(0.03 + pow(tone / 65536.0, 2.0), LPMax);
		iirLPFilter[inactiveFilter].CalcCoeff(cutoff);
	}

	currentCutoff = cutoff;
}


//	Take a new sample and return filtered value
iirdouble_t Filter::CalcIIRFilter(iirdouble_t sample, channel c)
{
	if (filterControl == HP) {
		return iirHPFilter[activeFilter].FilterSample(sample, iirHPReg[c]);
	} else {
		return iirLPFilter[activeFilter].FilterSample(sample, iirLPReg[c]);
	}

}

// Edit damping of individual sections
void Filter::CustomiseIIR(uint8_t section, iirdouble_t dampAmt)
{
	customDamping = true;									// Set to true if using custom damping coefficients (otherwise will default to Butterworth)
	for (auto& iir : iirLPFilter) {
		iir.UpdateProto(section, dampAmt);
	}
	for (auto& iir : iirHPFilter) {
		iir.UpdateProto(section, dampAmt);
	}
}

// Edit number of sections
void Filter::CustomiseIIR(uint8_t poleCount)
{
	for (auto& iir : iirLPFilter) {
		iir.UpdateProto(poleCount);
	}
	for (auto& iir : iirHPFilter) {
		iir.UpdateProto(poleCount);
	}
	DefaultIIR();											// Reset coefficients to Butterworth (According to pole count)
}

void Filter::DefaultIIR()
{
	customDamping = false;									// Set to true if using custom damping coefficients (otherwise will default to Butterworth)
	for (auto& iir : iirLPFilter) {
		iir.DefaultProto();
	}
	for (auto& iir : iirHPFilter) {
		iir.DefaultProto();
	}
	Update(true);											// Force recalculation of coefficients
}


void Filter::UpdateConfig()
{
	if (filter.cfg.iirNumPoles != filter.defaultPoles && filter.cfg.iirNumPoles > 0 && filter.cfg.iirNumPoles < MAX_POLES) {
		filter.CustomiseIIR(filter.cfg.iirNumPoles);
	}

	if (filter.customDamping) {
		for (uint8_t i = 0; i < 4; ++i) {
			filter.CustomiseIIR(i, filter.customDamping);
		}
	}
}
