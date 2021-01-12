#include "filter.h"

/*
 * CalcIIRFilterCoeff
 * 	CalcLowPassProtoCoeff
 *  	ButterworthPoly
 *  	GetFilterCoeff
 *  		SortRootsByZeta
 *  			HeapIndexSort
 */

// FIR data
float firCoeff[2][FIRTAPS];
float winCoeff[FIRTAPS];
int16_t filterBuffer[2][FIRTAPS];		// Ring buffer containing most recent playback samples for quicker filtering from SRAM

// Debug
bool calculatingFilter = false;
bool activateFilter = true;
bool activateWindow = true;
float currentCutoff;

bool debugSort = false;

// Rectangular FIR
void filter::InitFIRFilter(uint16_t tone)
{
	float arg, omegaC;

	// Pass in smoothed ADC reading - generate appropriate omega sweeping from Low pass to High Pass (settings optimised for 81 filter taps)
	if (tone < filterPotCentre - 1000) {		// Low Pass
		passType = LowPass;
		omegaC = 1.0f - std::pow(((float)filterPotCentre - tone) / 34000.0f, 0.2f);
	} else if (tone > filterPotCentre + 1000) {
		passType = HighPass;
		omegaC = 1.0f - std::pow(((float)tone - filterPotCentre)  / 75000.0f, 3.0f);
	} else {
		passType = FilterOff;
		omegaC = 1.0f;
	}

	// cycle between two sets of coefficients so one can be changed without affecting the other
	//uint8_t inactiveFilter = activeFilter;
	uint8_t inactiveFilter = (activeFilter == 0) ? 1 : 0;

	if (passType == LowPass) {
		for (int8_t j = 0; j < FIRTAPS; ++j) {
			arg = (float)j - (float)(FIRTAPS - 1) / 2.0;
			firCoeff[inactiveFilter][j] = omegaC * Sinc(omegaC * arg * M_PI) * (activateWindow ? winCoeff[j] : 1.0);
		}
	} else if (passType == HighPass)  {
		int8_t sign = 1;
		for (int8_t j = 0; j < FIRTAPS; ++j) {
			arg = (float)j - (float)(FIRTAPS - 1) / 2.0;

			firCoeff[inactiveFilter][j] = sign * omegaC * Sinc(omegaC * arg * M_PI) * (activateWindow ? winCoeff[j] : 1.0);
			sign = sign * -1;
		}
	}
	activeFilter = inactiveFilter;
	currentCutoff = omegaC;
}


void filter::FIRFilterWindow(float beta)		// example has beta = 4.0 (value between 0.0 and 10.0)
{
	float arg;

	// Kaiser window
	for (uint8_t j = 0; j < FIRTAPS; j++) {
		arg = beta * sqrt(1.0 - pow( ((float)(2 * j) + 1 - FIRTAPS) / (FIRTAPS + 1), 2.0) );
		winCoeff[j] = Bessel(arg) / Bessel(beta);
	}
}

float filter::Sinc(float x)
{
	if (x > -1.0E-5 && x < 1.0E-5)
		return(1.0);
	return (std::sin(x) / x);
}

// Used for Kaiser window calculations
float filter::Bessel(float x)
{
	float Sum = 0.0, XtoIpower;
	int Factorial;
	for (uint8_t i = 1; i < 10; ++i) {
		XtoIpower = pow(x / 2.0, (float)i);
		Factorial = 1;
		for (uint8_t j = 1; j <= i; ++j) {
			Factorial *= j;
		}
		Sum += pow(XtoIpower / (float)Factorial, 2.0);
	}
	return(1.0 + Sum);
}




void filter::InitIIRFilter(uint16_t tone) {

	iirdouble cutoff;

	//activateFilter = true;
	if (filterControl == LP) {			// Want a sweep from 0.03 to 0.999 with most travel at low end
		iirSettings.NumPoles = 8;
		passType = LowPass;
		cutoff = std::min(0.03 + pow((iirdouble)tone / 65536.0, 2.0), 0.999);
	} else if (filterControl == HP) {		// Want a sweep from 0.001 to 0.2-0.3
		passType = HighPass;
		iirSettings.NumPoles = 4;
		cutoff = 0.001 + pow(((iirdouble)tone / 100000.0), 3.0);
	} else {
		if (tone <= filterPotCentre) {		// Low Pass
			iirSettings.NumPoles = 8;
			passType = LowPass;
			cutoff = std::min(pow(((iirdouble)tone + 15000) / 32000.0, 2.0), 0.999);
		} else if (tone > filterPotCentre) {
			passType = HighPass;
			iirSettings.NumPoles = 4;
			cutoff = 0.001 + pow((((iirdouble)tone - filterPotCentre) / 50000.0), 3.0);
		}
	}

	uint8_t inactiveFilter = (activeFilter == 0) ? 1 : 0;

	CalcIIRFilterCoeff(cutoff, passType, IIRCoeff[inactiveFilter]);

	activeFilter = inactiveFilter;
	currentCutoff = cutoff;
}



//---------------------------------------------------------------------------
/*
 This calculates the coefficients for IIR filters from a set of 2nd order s plane coefficients
 which are obtained by calling CalcLowPassProtoCoeff() in LowPassPrototypes.cpp.
 The s plane filters are frequency scaled so their 3 dB frequency is at s = omega = 1 rad/sec.
 The poles and zeros are also ordered in a manner appropriate for IIR filters.
 For a derivation of the formulas used here, see the IIREquationDerivations.pdf
 This shows how the various poly coefficients are defined.
 H(s) = ( Ds^2 + Es + F ) / ( As^2 + Bs + C )
 H(z) = ( b2z^2 + b1z + b0 ) / ( a2z^2 + a1z + a0 )
 */
void filter::CalcIIRFilterCoeff(iirdouble omegaC, PassType passType, TIIRCoeff &iirCoeff)
{
	int j;

	iirdouble A, B, C, D, E, F, T, Arg;

	// Get the low pass prototype 2nd order s plane coefficients.
	iirPrototype iirProto = GetPrototype(iirSettings.NumPoles);

	// Init the IIR structure.
	for (j = 0; j < ARRAY_DIM; j++)	{
		iirCoeff.a0[j] = 0.0;  iirCoeff.b0[j] = 0.0;
		iirCoeff.a1[j] = 0.0;  iirCoeff.b1[j] = 0.0;
		iirCoeff.a2[j] = 0.0;  iirCoeff.b2[j] = 0.0;
		iirCoeff.a3[j] = 0.0;  iirCoeff.b3[j] = 0.0;
		iirCoeff.a4[j] = 0.0;  iirCoeff.b4[j] = 0.0;
	}

	// Set the number of IIR filter sections we will be generating.
	iirCoeff.NumSections = (iirSettings.NumPoles + 1) / 2;

	// T sets the IIR filter's corner frequency, or center freqency.
	// The Bilinear transform is defined as:  s = 2/T * tan(Omega/2) = 2/T * (1 - z)/(1 + z)
	T = 2.0 * tan(omegaC * M_PI / 2);

	// Calc the IIR coefficients. SPlaneCoeff.NumSections is the number of 1st and 2nd order s plane factors.
	for (j = 0; j < iirProto.Coeff.NumSections; j++) {
		A = iirProto.Coeff.D2[j];			// We use A - F to make the code easier to read.
		B = iirProto.Coeff.D1[j];
		C = iirProto.Coeff.D0[j];
		D = iirProto.Coeff.N2[j];
		E = iirProto.Coeff.N1[j];			// N1 is always zero, except for the all pass. Consequently, the equations below can be simplified a bit by removing E.
		F = iirProto.Coeff.N0[j];

		// b's are the numerator  a's are the denominator
		if (passType == LowPass) {
			if (A == 0.0 && D == 0.0) {					// 1 pole case
				Arg = (2.0 * B + C * T);
				iirCoeff.a2[j] = 0.0;
				iirCoeff.a1[j] = (-2.0 * B + C * T) / Arg;
				iirCoeff.a0[j] = 1.0;

				iirCoeff.b2[j] = 0.0;
				iirCoeff.b1[j] = (-2.0 * E + F * T) / Arg * C/F;
				iirCoeff.b0[j] = ( 2.0 * E + F * T) / Arg * C/F;
			} else {									// 2 poles

				Arg = (4.0 * A + 2.0 * B * T + C * T * T);
				iirCoeff.a2[j] = (4.0 * A - 2.0 * B * T + C * T * T) / Arg;
				iirCoeff.a1[j] = (2.0 * C * T * T - 8.0 * A) / Arg;
				iirCoeff.a0[j] = 1.0;

				// With all pole filters, our LPF numerator is (z+1)^2, so all our Z Plane zeros are at -1
				iirCoeff.b2[j] = (4.0 * D - 2.0 * E * T + F * T * T) / Arg * C/F;
				iirCoeff.b1[j] = (2.0 * F * T * T - 8.0 * D) / Arg * C/F;
				iirCoeff.b0[j] = (4*D + F * T * T + 2.0 * E * T) / Arg * C/F;
			}
		}

		if (passType == HighPass) { // High Pass
			if (A == 0.0 && D == 0.0) { // 1 pole
				Arg = 2.0 * C + B * T;
				iirCoeff.a2[j] = 0.0;
				iirCoeff.a1[j] = (B * T - 2.0 * C) / Arg;
				iirCoeff.a0[j] = 1.0;

				iirCoeff.b2[j] = 0.0;
				iirCoeff.b1[j] = (E * T - 2.0 * F) / Arg * C/F;
				iirCoeff.b0[j] = (E * T + 2.0 * F) / Arg * C/F;
			} else {  // 2 poles
				Arg = A * T * T + 4.0 * C + 2.0 * B * T;
				iirCoeff.a2[j] = (A * T * T + 4.0 * C - 2.0 * B * T) / Arg;
				iirCoeff.a1[j] = (2.0 * A * T * T - 8.0 * C) / Arg;
				iirCoeff.a0[j] = 1.0;

				// With all pole filters, our HPF numerator is (z-1)^2, so all our Z Plane zeros are at 1
				iirCoeff.b2[j] = (D * T * T - 2.0 * E * T + 4.0 * F) / Arg * C/F;
				iirCoeff.b1[j] = (2.0 * D * T * T - 8.0 * F) / Arg * C/F;
				iirCoeff.b0[j] = (D * T * T + 4.0 * F + 2.0 * E * T) / Arg * C/F;
			}
		}
	}

}



//	Take a new sample and return filtered value
iirdouble filter::IIRFilter(iirdouble sample, channel c)
{
	iirdouble y = SectCalc(0, sample, c);
	for (uint8_t k = 1; k < IIRCoeff[activeFilter].NumSections; k++) {
		y = SectCalc(k, y, c);
	}
	return y;
}


// This gets used with the function above, FilterWithIIR()
iirdouble filter::SectCalc(int k, iirdouble x, channel c)
{
	iirdouble y, CenterTap;
	static iirdouble RegX1[2][ARRAY_DIM], RegX2[2][ARRAY_DIM], RegY1[2][ARRAY_DIM], RegY2[2][ARRAY_DIM];
	static iirdouble MaxRegVal = 1.0E-12;

	// Zero the registers on the 1st call or on an overflow condition. The overflow limit used
	// here is small for iirdouble variables, but a filter that reaches this threshold is broken.
	int j = 1;
	if ((j == 0 && k == 0) || MaxRegVal > OVERFLOW_LIMIT) {

		MaxRegVal = 1.0E-12;
		for (int i = 0; i < ARRAY_DIM; i++) {
			RegX1[c][i] = 0.0;
			RegX2[c][i] = 0.0;
			RegY1[c][i] = 0.0;
			RegY2[c][i] = 0.0;
		}
	}

	CenterTap = x * IIRCoeff[activeFilter].b0[k] + IIRCoeff[activeFilter].b1[k] * RegX1[c][k] + IIRCoeff[activeFilter].b2[k] * RegX2[c][k];
	y = IIRCoeff[activeFilter].a0[k] * CenterTap - IIRCoeff[activeFilter].a1[k] * RegY1[c][k] - IIRCoeff[activeFilter].a2[k] * RegY2[c][k];

	RegX2[c][k] = RegX1[c][k];
	RegX1[c][k] = x;
	RegY2[c][k] = RegY1[c][k];
	RegY1[c][k] = y;

	// MaxRegVal is used to prevent overflow.  Overflow seldom occurs, but will
	// if the filter has faulty coefficients. MaxRegVal is usually less than 100.0
	if (std::abs(CenterTap) > MaxRegVal)
		MaxRegVal = std::abs(CenterTap);
	if (std::abs(y) > MaxRegVal)
		MaxRegVal = std::abs(y);
	return(y);
}

iirPrototype& filter::GetPrototype(uint8_t poles) {
	// Check if prototype exists in map and create if not
	if (IIRPrototypes.find(poles) == IIRPrototypes.end()) {
		IIRPrototypes.emplace(poles, iirPrototype(poles));
	}
	return IIRPrototypes.at(poles);
}

void iirPrototype::CalcLowPassProtoCoeff()
{
	std::array<std::complex<double>, ARRAY_DIM> Poles;

	// Init the S Plane Coeff. H(s) = (N2*s^2 + N1*s + N0) / (D2*s^2 + D1*s + D0)
	for (uint8_t j = 0; j < ARRAY_DIM; j++) {
		Coeff.N2[j] = 0.0;
		Coeff.N1[j] = 0.0;
		Coeff.N0[j] = 1.0;
		Coeff.D2[j] = 0.0;
		Coeff.D1[j] = 0.0;
		Coeff.D0[j] = 1.0;
	}

	// A one pole filter is simply 1/(s+1)
	if (NumPoles == 1) {
		Coeff.D1[0] = 1.0;
	} else {				// Always use Butterworth
		Poles[0] =  std::complex<double>(0.0, 0.0) ;
		ButterworthPoly(Poles);
		GetFilterCoeff(Poles, Coeff.D2, Coeff.D1, Coeff.D0);
	}

	Coeff.NumSections = NumPoles;
}


// Calculate the roots for a Butterworth filter: fill the array Roots[]
//void iirPrototype::ButterworthPoly(std::complex<double> *Roots)
void iirPrototype::ButterworthPoly(std::array<std::complex<double>, ARRAY_DIM> &Roots)
{
	int n = 0;
	iirdouble theta;

	for (uint8_t j = 0; j < NumPoles / 2; j++) {
		theta = M_PI * (iirdouble)(2 * j + NumPoles + 1) / (iirdouble)(2 * NumPoles);
		Roots[n++] = std::complex<double>(cos(theta), sin(theta));
		Roots[n++] = std::complex<double>(cos(theta), -sin(theta));
	}
	if (NumPoles % 2 == 1)
		Roots[n++] = std::complex<double>(-1.0, 0.0);		// The real root for odd pole counts

}


// Some of the Polys generate both left hand and right hand plane roots.
// We use this function to get the left hand plane poles and imag axis zeros to
// create the 2nd order polynomials with coefficients A2, A1, A0.
// We return the Polynomial count.

// We first sort the roots according the the real part (a zeta sort). Then all the left
// hand plane roots are grouped and in the correct order for IIR and Opamp filters.
// We then check for duplicate roots, and set an inconsequential real or imag part to zero.
// Then the 2nd order coefficients are calculated.
void iirPrototype::GetFilterCoeff(std::array<std::complex<double>, ARRAY_DIM> &Roots, iirdouble *A2, iirdouble *A1, iirdouble *A0)
{
	int PolyCount, j;

	//SortRootsByZeta(Roots, NumPoles);			// Places the most negative real part 1st.

	// Sort the roots with the most negative real part first
	std::sort(Roots.begin(), Roots.end(), [](const std::complex<double> &lhs, const std::complex<double> &rhs) {
		return lhs.real() < rhs.real();
	});


//	// Check for duplicate roots and set to a RHP value
//	for (j = 0; j < NumPoles-1; j++) {
//		for (k = j + 1; k < NumPoles; k++) {
//			if (std::abs(Roots[j].real() - Roots[k].real()) < 1.0E-3 && std::abs(Roots[j].imag() - Roots[k].imag()) < 1.0E-3) {
//				Roots[k] = std::complex<double>((iirdouble)k, 0.0); // RHP roots are ignored below, Use k is to prevent duplicate checks for matches.
//				debugCount++;
//			}
//		}
//	}

	// This forms the 2nd order coefficients from the root value. Ignore roots in the Right Hand Plane.
	PolyCount = 0;
	for (j = 0; j < NumPoles; j++) {
		if ((double)Roots[j].real() > 0.0)
			continue;							// Right Hand Plane
		if (Roots[j].real() == 0.0 && Roots[j].imag() == 0.0)
			continue;							// At the origin.  This should never happen.

		if (Roots[j].real() == 0.0) {			// Imag Root (A poly zero)
			A2[PolyCount] = 1.0;
			A1[PolyCount] = 0.0;
			A0[PolyCount] = Roots[j].imag() * Roots[j].imag();
			j++;
			PolyCount++;
		} else if (Roots[j].imag() == 0.0) {	// Real Pole
			A2[PolyCount] = 0.0;
			A1[PolyCount] = 1.0;
			A0[PolyCount] = -Roots[j].real();
			PolyCount++;
		} else { 								// Complex Pole
			A2[PolyCount] = 1.0;
			A1[PolyCount] = -2.0 * Roots[j].real();
			A0[PolyCount] = Roots[j].real() * Roots[j].real() + Roots[j].imag() * Roots[j].imag();
			j++;
			PolyCount++;
		}
	}

}

/*
volatile int debugCount = 0;

//---------------------------------------------------------------------------
// This sorts on the real part if the real part of the 1st root != 0 (a Zeta sort)
// else we sort on the imag part. If SortType == stMin for both the poles and zeros, then
// the poles and zeros will be properly matched.
// This also sets an inconsequential real or imag part to zero.
// A matched pair of z plane real roots, such as +/- 1, don't come out together.
// Used above in GetFilterCoeff and the FIR zero plot.
void iirPrototype::SortRootsByZeta(std::array<std::complex<double>, ARRAY_DIM> &Roots, int Count)
{
	int j, k, RootJ[ARRAY_DIM];
	iirdouble SortValue[ARRAY_DIM];
	std::complex<double> TempRoots[ARRAY_DIM];

	// Set an inconsequential real or imag part to zero. FIXME - not sure this is working or necessary
	for (j = 0; j < Count; j++)	{
		if (std::abs(Roots[j].real()) * 1.0E3 < std::abs(Roots[j].imag()) ) {
			Roots[j].real(0.0);
			debugCount++;
		}
		if (std::abs(Roots[j].imag()) * 1.0E3 < std::abs(Roots[j].real()) ) {
			Roots[j].imag(0.0);
			debugCount++;
		}
	}

	// Sort the roots.
	for (j = 0; j < Count; j++) {
		RootJ[j] = j;							// Needed for HeapIndexSort
	}

	if (Roots[0].real() != 0.0 ) {				// Sort on Real part
		for (j = 0; j < Count; j++)
			SortValue[j] = Roots[j].real();
	} else {  									// Sort on Imag part
		for (j = 0; j < Count; j++)
			SortValue[j] = std::abs(Roots[j].imag());
	}
	HeapIndexSort(SortValue, RootJ, Count);		// most negative root on top

	for (j = 0; j < Count; j++)	{
		k = RootJ[j];							// RootJ is the sort index
		TempRoots[j] = Roots[k];
	}
	for (j = 0; j < Count; j++)	{
		Roots[j] = TempRoots[j];
	}

}



// Remember to set the Index array to 0, 1, 2, 3, ... N-1
void iirPrototype::HeapIndexSort(iirdouble *Data, int *Index, int N)
{
	int i, j, k, m, IndexTemp;
	long long FailSafe, NSquared; // need this for big sorts

	NSquared = (long long)N * (long long)N;
	m = N / 2;
	k = N - 1;
	for (FailSafe = 0; FailSafe < NSquared; FailSafe++) { // typical FailSafe value on return is N*log2(N)

		if (m > 0)
			IndexTemp = Index[--m];
		else {
			IndexTemp = Index[k];
			Index[k] = Index[0];
			if (--k == 0) {
				Index[0] = IndexTemp;
				return;
			}
		}

		i = m + 1;
		j = 2 * i;

		while (j < k + 2) {
			FailSafe++;
			if (j <= k && Data[Index[j-1]] < Data[Index[j]])
				j++;
			if (Data[IndexTemp] < Data[Index[j-1]]) {
				Index[i-1] = Index[j-1];
				i = j;
				j += i;
			}
			else break;
		}

		Index[i-1] = IndexTemp;
	}
}
*/
