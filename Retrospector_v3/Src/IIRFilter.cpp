#include "IIRFilter.h"


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
 H(s) = 1 / ( As^2 + Bs + C )
 H(z) = ( b2 z^-2 + b1 z^-1 + b0 ) / ( a2 z^-2 + a1 z^-1 + a0 )
 See http://www.iowahills.com/A4IIRBilinearTransform.html
 function originally CalcIIRFilterCoeff()
 */
void IIRFilter::CalcCoeff(iirdouble_t omega)
{
	int j;
	iirdouble_t A, B, C, T, arg;

	if (cutoffFreq == omega)		// Avoid recalculating coefficients when already found
		return;
	else
		cutoffFreq = omega;

	// Set the number of IIR filter sections we will be generating.
	numSections = (numPoles + 1) / 2;

	// T sets the IIR filter's corner frequency, or center frequency
	// The Bilinear transform is defined as:  s = 2/T * tan(Omega/2) = 2/T * (1 - z^-1)/(1 + z^-1)
	T = 2.0 * tan(omega * M_PI / 2);

	// Calc the IIR coefficients. SPlaneCoeff.NumSections is the number of 1st and 2nd order s plane factors.
	for (j = 0; j < numSections; j++) {
		A = iirProto.Coeff.D2[j];				// Always one
		B = iirProto.Coeff.D1[j];
		C = iirProto.Coeff.D0[j];				// Always one

		// b's are the numerator, a's are the denominator
		if (passType == LowPass) {
			if (A == 0.0) {						// 1 pole case
				arg = (2.0 * B + C * T);
				iirCoeff.a2[j] = 0.0;
				iirCoeff.a1[j] = (-2.0 * B + C * T) / arg;
				iirCoeff.a0[j] = 1.0;

				iirCoeff.b2[j] = 0.0;
				iirCoeff.b1[j] = T / arg * C;
				iirCoeff.b0[j] = T / arg * C;
			} else {							// 2 poles

				arg = (4.0 * A + 2.0 * B * T + C * T * T);
				iirCoeff.a2[j] = (4.0 * A - 2.0 * B * T + C * T * T) / arg;
				iirCoeff.a1[j] = (2.0 * C * T * T - 8.0 * A) / arg;
				iirCoeff.a0[j] = 1.0;

				// With all pole filters, LPF numerator is (z+1)^2, so all Z Plane zeros are at -1
				iirCoeff.b2[j] = (T * T) / arg * C;
				iirCoeff.b1[j] = (2.0 * T * T) / arg * C;
				iirCoeff.b0[j] = (T * T) / arg * C;
			}
		}

		if (passType == HighPass) {				// High Pass
			if (A == 0.0) {						// 1 pole
				arg = 2.0 * C + B * T;
				iirCoeff.a2[j] = 0.0;
				iirCoeff.a1[j] = (B * T - 2.0 * C) / arg;
				iirCoeff.a0[j] = 1.0;

				iirCoeff.b2[j] = 0.0;
				iirCoeff.b1[j] = -2.0 / arg * C;
				iirCoeff.b0[j] = 2.0 / arg * C;
			} else {							// 2 poles
				arg = A * T * T + 4.0 * C + 2.0 * B * T;
				iirCoeff.a2[j] = (A * T * T + 4.0 * C - 2.0 * B * T) / arg;
				iirCoeff.a1[j] = (2.0 * A * T * T - 8.0 * C) / arg;
				iirCoeff.a0[j] = 1.0;

				// With all pole filters, HPF numerator is (z-1)^2, so all Z Plane zeros are at 1
				iirCoeff.b2[j] = 4.0 / arg * C;
				iirCoeff.b1[j] = -8.0 / arg * C;
				iirCoeff.b0[j] = 4.0 / arg * C;
			}
		}
	}

}


void IIRFilter::UpdateProto(uint8_t poleCount)
{
	numPoles = poleCount;
	iirProto.numPoles = poleCount;
	if (numPoles == 1) {
		iirProto.Coeff.D2[0] = 0.0;
	} else {
		iirProto.Coeff.D2[0] = 1.0;
	}
}

void IIRFilter::UpdateProto(uint8_t section, iirdouble_t dampAmt)
{
	customDamping = true;
	damping[section] = dampAmt;
	if (iirProto.Coeff.D2[section] == 0.0) {		// One pole
		iirProto.Coeff.D1[section] = dampAmt;
	} else {
		iirProto.Coeff.D1[section] = 2 * dampAmt;
	}
}


void IIRFilter::DefaultProto()
{
	customDamping = false;
	iirProto.DefaultProtoCoeff();
}

// Calculate the default S Plane Coefficients (Butterworth):  H(s) = 1 / (D2*s^2 + D1*s + D0)
void IIRPrototype::DefaultProtoCoeff()
{
	std::array<complex_t, MAX_POLES> Poles;

	// A one pole filter is simply 1/(s+1)
	if (numPoles == 1) {
		Coeff.D0[0] = 1.0;
		Coeff.D1[0] = 1.0;
		Coeff.D2[0] = 0.0;
	} else {
		Poles[0] = complex_t(0.0, 0.0);
		ButterworthPoly(Poles);
		GetFilterCoeff(Poles);
	}
}


// Calculate the roots for a Butterworth filter: fill the array Roots[]
void IIRPrototype::ButterworthPoly(std::array<complex_t, MAX_POLES> &Roots)
{
	int n = 0;
	iirdouble_t theta;

	for (uint8_t j = 0; j < numPoles / 2; j++) {
		theta = M_PI * static_cast<iirdouble_t>(2 * j + numPoles + 1) / static_cast<iirdouble_t>(2 * numPoles);
		Roots[n++] = complex_t(cos(theta), sin(theta));
		Roots[n++] = complex_t(cos(theta), -sin(theta));
	}
	if (numPoles % 2 == 1) {
		Roots[n++] = complex_t(-1.0, 0.0);		// The real root for odd pole counts
	}

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


