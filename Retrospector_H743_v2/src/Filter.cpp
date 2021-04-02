#include "Filter.h"

// Debug
bool calculatingFilter = false;

void Filter::Init()
{
	filter.FIRFilterWindow();
	filter.Update(true);
}


void Filter::Update(bool reset)
{

	// Debug: create tests of variation in damping techniques
	static uint16_t testCounter;
	static uint16_t dampMin[2], dampMax[2];
	if (testCounter == 0) {
		dampDiff[0] = dampMax[0] - dampMin[0];
		dampMin[0] = dampedADC;
		dampMax[0] = dampedADC;
		dampDiff[1] = dampMax[1] - dampMin[1];
		dampMin[1] = dampedADC2;
		dampMax[1] = dampedADC2;
	} else {
		if (dampedADC > dampMax[0]) {
			dampMax[0] = dampedADC;
		} else if (dampedADC < dampMin[0]) {
			dampMin[0] = dampedADC;
		}
		if (dampedADC2 > dampMax[1]) {
			dampMax[1] = dampedADC2;
		} else if (dampedADC2 < dampMin[1]) {
			dampMin[1] = dampedADC2;
		}
	}
	++testCounter;

	TIM3->CNT = 0;		// Debug

	//dampedADC = std::max((127L * dampedADC + std::min((int)ADC_array[ADC_Tone] + (65535 - ADC_array[ADC_Delay_CV_L]), 65535)) >> 7, 0L);		// FIXME - don't yet have CV input for Filter
	dampedADC = filterADC.FilterSample(std::min((int)ADC_array[ADC_Filter_Pot] + (65535 - ADC_array[ADC_Filter_CV]), 65535));
	//dampedTone = std::max((31L * dampedTone + ADC_array[ADC_Tone]) >> 5, 0L);		// FIXME - don't yet have CV input for Filter

	extern uint32_t debugDuration;
	debugDuration = TIM3->CNT;

	if (reset || std::abs(dampedADC - previousADC) > hysteresis) {
		calculatingFilter = true;
		previousADC = dampedADC;
		if (filterType == IIR) {
			InitIIRFilter(dampedADC);
		} else {
			InitFIRFilter(dampedADC);
		}
		calculatingFilter = false;
	}
}

float Filter::CalcFilter(float sample, channel c)
{
	if (activateFilter) {
		if (filterType == IIR) {
			return static_cast<float>(filter.CalcIIRFilter(sample, c));
		} else {
			return filter.CalcFIRFilter(sample, c);
		}
	} else {
		return sample;
	}
}

// Rectangular FIR
void Filter::InitFIRFilter(uint16_t tone)
{
	float omega;
	int8_t arg;

	// Pass in smoothed ADC reading - generate appropriate omega sweeping from Low pass to High Pass
	if (tone < filterPotCentre - 1000) {		// Low Pass
		passType = LowPass;
		omega = 1.0f - std::pow(((float)filterPotCentre - tone) / 34000.0f, 0.2f);
	} else if (tone > filterPotCentre + 1000) {
		passType = HighPass;
		omega = 1.0f - std::pow(((float)tone - filterPotCentre)  / 75000.0f, 3.0f);
	} else {
		passType = FilterOff;
		omega = 1.0f;
	}

	// cycle between two sets of coefficients so one can be changed without affecting the other
	uint8_t inactiveFilter = (activeFilter == 0) ? 1 : 0;

	if (passType == LowPass) {
		for (int8_t j = 0; j < firTaps / 2 + 1; ++j) {
			arg = j - firTaps / 2;
			firCoeff[inactiveFilter][j] = omega * Sinc(omega * arg * M_PI) * winCoeff[j];
		}
	} else if (passType == HighPass)  {
		int8_t sign = 1;
		for (int8_t j = 0; j < firTaps / 2 + 1; ++j) {
			arg = j - firTaps / 2;
			firCoeff[inactiveFilter][j] = sign * omega * Sinc(omega * arg * M_PI) * winCoeff[j];
			sign = sign * -1;
		}
	}

	activeFilter = inactiveFilter;
	currentCutoff = omega;
}


// Convolution routine for delayed samples - takes current sample, buffers, convolves and returns filtered sample
float Filter::CalcFIRFilter(float sample, channel c)
{
	float outputSample = 0.0;

	filterBuffer[c][filterBuffPos[c]] = sample;
	if (currentCutoff == 1.0f) {		// If not filtering take middle most sample to account for FIR group delay when filtering active (gives more time for main loop when filter inactive)
		uint8_t mid = filterBuffPos[c] - (firTaps / 2);
		outputSample = filterBuffer[c][mid];
	} else {
		uint8_t pos, revpos;

		pos = filterBuffPos[c] - firTaps + 1;		// position of sample 1, 2, 3 etc
		revpos = filterBuffPos[c];					// position of sample N, N-1, N-2 etc

		for (uint8_t i = 0; i < firTaps / 2; ++i) {
			// Folded FIR structure - as coefficients are symmetrical we can multiple the sample 1 + sample N by the 1st coefficient, sample 2 + sample N - 1 by 2nd coefficient etc
			outputSample += firCoeff[activeFilter][i] * (filterBuffer[c][pos++] + filterBuffer[c][revpos--]);
		}

		outputSample += firCoeff[activeFilter][firTaps / 2] * filterBuffer[c][pos];
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
	for (uint8_t j = 0; j < firTaps; j++) {
		arg = beta * sqrt(1.0 - pow( ((float)(2 * j) + 1 - firTaps) / (firTaps + 1), 2.0) );
		winCoeff[j] = Bessel(arg) / Bessel(beta);
	}
}


// Used for Kaiser window calculations
float Filter::Bessel(float x)
{
	float sum = 0.0, xPower;
	int factorial;
	for (uint8_t i = 1; i < 10; ++i) {
		xPower = pow(x / 2.0, (float)i);
		factorial = 1;
		for (uint8_t j = 1; j <= i; ++j) {
			factorial *= j;
		}
		sum += pow(xPower / (float)factorial, 2.0);
	}
	return(1.0 + sum);
}


void Filter::InitIIRFilter(uint16_t tone)
{
	iirdouble_t cutoff;
	constexpr iirdouble_t LPMax = 0.995;
	constexpr iirdouble_t HPMin = 0.001;

	uint8_t inactiveFilter = (activeFilter == 0) ? 1 : 0;

	if (filterControl == HP) {				// Want a sweep from 0.03 to 0.99 with most travel at low end
		passType = HighPass;
		cutoff = pow(((iirdouble_t)tone / 100000.0), 3.0) + HPMin;
		iirLPFilter[inactiveFilter].CalcCoeff(LPMax);
		iirHPFilter[inactiveFilter].CalcCoeff(cutoff);
	} else {		// Want a sweep from 0.001 to 0.2-0.3
		passType = LowPass;
		cutoff = std::min(0.03 + pow((iirdouble_t)tone / 65536.0, 2.0), LPMax);
		iirHPFilter[inactiveFilter].CalcCoeff(HPMin);
		iirLPFilter[inactiveFilter].CalcCoeff(cutoff);
	}

	activeFilter = inactiveFilter;
	currentCutoff = cutoff;		// For debug
}


//	Take a new sample and return filtered value
iirdouble_t Filter::CalcIIRFilter(iirdouble_t sample, channel c)
{
	if (passType == HighPass) {
		return iirHPFilter[activeFilter].FilterSample(sample, iirHPReg[c]);
	} else {
		return iirLPFilter[activeFilter].FilterSample(sample, iirLPReg[c]);
	}

}


//	Take a new sample and return filtered value
iirdouble_t IIRFilter::FilterSample(iirdouble_t sample, IIRRegisters& registers)
{
	iirdouble_t y = CalcSection(0, sample, registers);
	for (uint8_t k = 1; k < numSections; k++) {
		y = CalcSection(k, y, registers);
	}
	return y;
}

// Calculates each stage of a multi-section IIR filter (eg 8 pole is constructed from four 2-pole filters)
iirdouble_t IIRFilter::CalcSection(int k, iirdouble_t x, IIRRegisters& registers)
{
	iirdouble_t y, CenterTap;
	static iirdouble_t MaxRegVal = 1.0E-12;

	// Zero the registers on an overflow condition
	if (MaxRegVal > 1.0E6) {

		MaxRegVal = 1.0E-12;
		for (uint8_t i = 0; i < MAX_SECTIONS; i++) {
			registers.X1[i] = 0.0;
			registers.X2[i] = 0.0;
			registers.Y1[i] = 0.0;
			registers.Y2[i] = 0.0;
		}
	}

	CenterTap = x * iirCoeff.b0[k] + iirCoeff.b1[k] * registers.X1[k] + iirCoeff.b2[k] * registers.X2[k];
	y = iirCoeff.a0[k] * CenterTap - iirCoeff.a1[k] * registers.Y1[k] - iirCoeff.a2[k] * registers.Y2[k];

	registers.X2[k] = registers.X1[k];
	registers.X1[k] = x;
	registers.Y2[k] = registers.Y1[k];
	registers.Y1[k] = y;

	// MaxRegVal is used to prevent overflow. Note that CenterTap regularly exceeds 100k but y maxes out at about 65k
	if (std::abs(CenterTap) > MaxRegVal) {
		MaxRegVal = std::abs(CenterTap);
	}
	if (std::abs(y) > MaxRegVal) {
		MaxRegVal = std::abs(y);
	}

	return y;
}



/*
 Calculate the z-plane coefficients for IIR filters from 2nd order S-plane coefficients
 H(s) = ( Ds^2 + Es + F ) / ( As^2 + Bs + C )
 H(z) = ( b2z^2 + b1z + b0 ) / ( a2z^2 + a1z + a0 )
 See http://www.iowahills.com/A4IIRBilinearTransform.html
 function originally CalcIIRFilterCoeff()
 */
void IIRFilter::CalcCoeff(iirdouble_t omega)
{
	int j;
	iirdouble_t A, B, C, D, E, F, T, arg;

	if (cutoffFreq == omega)		// Avoid recalculating coefficients when already found
		return;
	else
		cutoffFreq = omega;

	// Init the IIR structure.
	for (j = 0; j < MAX_SECTIONS; j++)	{
		iirCoeff.a0[j] = 0.0;  iirCoeff.b0[j] = 0.0;
		iirCoeff.a1[j] = 0.0;  iirCoeff.b1[j] = 0.0;
		iirCoeff.a2[j] = 0.0;  iirCoeff.b2[j] = 0.0;
	}

	// Set the number of IIR filter sections we will be generating.
	numSections = (numPoles + 1) / 2;
	 if (passType == BandPass)
		 numSections = numPoles;

	// T sets the IIR filter's corner frequency, or center freqency.
	// The Bilinear transform is defined as:  s = 2/T * tan(Omega/2) = 2/T * (1 - z)/(1 + z)
	T = 2.0 * tan(omega * M_PI / 2);

	// Calc the IIR coefficients. SPlaneCoeff.NumSections is the number of 1st and 2nd order s plane factors.
	for (j = 0; j < numSections; j++) {
		A = iirProto.Coeff.D2[j];			// Use A - F to make the code easier to read.
		B = iirProto.Coeff.D1[j];
		C = iirProto.Coeff.D0[j];
		D = iirProto.Coeff.N2[j];
		E = iirProto.Coeff.N1[j];			// N1 is always zero, except for the all pass. Consequently, the equations below can be simplified a bit by removing E.
		F = iirProto.Coeff.N0[j];

		// b's are the numerator  a's are the denominator
		if (passType == LowPass) {
			if (A == 0.0 && D == 0.0) {					// 1 pole case
				arg = (2.0 * B + C * T);
				iirCoeff.a2[j] = 0.0;
				iirCoeff.a1[j] = (-2.0 * B + C * T) / arg;
				iirCoeff.a0[j] = 1.0;

				iirCoeff.b2[j] = 0.0;
				iirCoeff.b1[j] = (-2.0 * E + F * T) / arg * C/F;
				iirCoeff.b0[j] = ( 2.0 * E + F * T) / arg * C/F;
			} else {									// 2 poles

				arg = (4.0 * A + 2.0 * B * T + C * T * T);
				iirCoeff.a2[j] = (4.0 * A - 2.0 * B * T + C * T * T) / arg;
				iirCoeff.a1[j] = (2.0 * C * T * T - 8.0 * A) / arg;
				iirCoeff.a0[j] = 1.0;

				// With all pole filters, LPF numerator is (z+1)^2, so all Z Plane zeros are at -1
				iirCoeff.b2[j] = (4.0 * D - 2.0 * E * T + F * T * T) / arg * C/F;
				iirCoeff.b1[j] = (2.0 * F * T * T - 8.0 * D) / arg * C/F;
				iirCoeff.b0[j] = (4*D + F * T * T + 2.0 * E * T) / arg * C/F;
			}
		}

		if (passType == HighPass) {						// High Pass
			if (A == 0.0 && D == 0.0) {					// 1 pole
				arg = 2.0 * C + B * T;
				iirCoeff.a2[j] = 0.0;
				iirCoeff.a1[j] = (B * T - 2.0 * C) / arg;
				iirCoeff.a0[j] = 1.0;

				iirCoeff.b2[j] = 0.0;
				iirCoeff.b1[j] = (E * T - 2.0 * F) / arg * C/F;
				iirCoeff.b0[j] = (E * T + 2.0 * F) / arg * C/F;
			} else {									// 2 poles
				arg = A * T * T + 4.0 * C + 2.0 * B * T;
				iirCoeff.a2[j] = (A * T * T + 4.0 * C - 2.0 * B * T) / arg;
				iirCoeff.a1[j] = (2.0 * A * T * T - 8.0 * C) / arg;
				iirCoeff.a0[j] = 1.0;

				// With all pole filters, our HPF numerator is (z-1)^2, so all our Z Plane zeros are at 1
				iirCoeff.b2[j] = (D * T * T - 2.0 * E * T + 4.0 * F) / arg * C/F;
				iirCoeff.b1[j] = (2.0 * D * T * T - 8.0 * F) / arg * C/F;
				iirCoeff.b0[j] = (D * T * T + 4.0 * F + 2.0 * E * T) / arg * C/F;
			}
		}
	}

}


// Calculate the S Plane Coefficients:  H(s) = (N2*s^2 + N1*s + N0) / (D2*s^2 + D1*s + D0)
void IIRPrototype::CalcLowPassProtoCoeff()
{
	std::array<complex_t, MAX_POLES> Poles;

	for (uint8_t j = 0; j < MAX_POLES; j++) {
		Coeff.N2[j] = 0.0;
		Coeff.N1[j] = 0.0;
		Coeff.N0[j] = 1.0;
		Coeff.D2[j] = 0.0;
		Coeff.D1[j] = 0.0;
		Coeff.D0[j] = 1.0;
	}

	// A one pole filter is simply 1/(s+1)
	if (numPoles == 1) {
		Coeff.D1[0] = 1.0;
	} else {				// Always use Butterworth
		Poles[0] = complex_t(0.0, 0.0);
		ButterworthPoly(Poles);
		GetFilterCoeff(Poles);
	}

	numSections = numPoles;
}


// Calculate the roots for a Butterworth filter: fill the array Roots[]
void IIRPrototype::ButterworthPoly(std::array<complex_t, MAX_POLES> &Roots)
{
	int n = 0;
	iirdouble_t theta;

	for (uint8_t j = 0; j < numPoles / 2; j++) {
		theta = M_PI * (iirdouble_t)(2 * j + numPoles + 1) / (iirdouble_t)(2 * numPoles);
		Roots[n++] = complex_t(cos(theta), sin(theta));
		Roots[n++] = complex_t(cos(theta), -sin(theta));
	}
	if (numPoles % 2 == 1)
		Roots[n++] = complex_t(-1.0, 0.0);		// The real root for odd pole counts

}


// Create the 2nd order polynomials with coefficients A2, A1, A0.
void IIRPrototype::GetFilterCoeff(std::array<complex_t, MAX_POLES> &Roots)
{
	int polyCount, j;

	// Sort the roots with the most negative real part first
	std::sort(Roots.begin(), Roots.end(), [](const complex_t &lhs, const complex_t &rhs) {
		return lhs.real() < rhs.real();
	});

	// This forms the 2nd order coefficients from the root value. Ignore roots in the Right Hand Plane.
	polyCount = 0;
	for (j = 0; j < numPoles; j++) {
		if (Roots[j].real() > 0.0)
			continue;							// Right Hand Plane
		if (Roots[j].real() == 0.0 && Roots[j].imag() == 0.0)
			continue;							// At the origin.  This should never happen.

		if (Roots[j].real() == 0.0) {			// Imag Root (A poly zero)
			Coeff.D2[polyCount] = 1.0;
			Coeff.D1[polyCount] = 0.0;
			Coeff.D0[polyCount] = Roots[j].imag() * Roots[j].imag();
			j++;
			polyCount++;
		} else if (Roots[j].imag() == 0.0) {	// Real Pole
			Coeff.D2[polyCount] = 0.0;
			Coeff.D1[polyCount] = 1.0;
			Coeff.D0[polyCount] = -Roots[j].real();
			polyCount++;
		} else { 								// Complex Pole
			Coeff.D2[polyCount] = 1.0;
			Coeff.D1[polyCount] = -2.0 * Roots[j].real();
			Coeff.D0[polyCount] = Roots[j].real() * Roots[j].real() + Roots[j].imag() * Roots[j].imag();
			j++;
			polyCount++;
		}
	}
}

iirdouble_t FixedFilter::FilterSample(iirdouble_t sample)
{
	return filter.FilterSample(sample, iirReg);
}
