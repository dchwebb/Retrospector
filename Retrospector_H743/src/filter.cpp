#include "filter.h"

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
		XtoIpower = pow(x/2.0, (float)i);
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

	if (tone <= filterPotCentre) {		// Low Pass
		iirSettings.NumPoles = 4;
		passType = LowPass;
		cutoff = std::min(pow(((iirdouble)tone + 15000) / 37000.0, 5.0), 0.999);
	} else if (tone > filterPotCentre) {
		passType = HighPass;
		iirSettings.NumPoles = 1;
		cutoff = pow((((iirdouble)tone - filterPotCentre) / 65536.0), 4.0);
	}

	uint8_t inactiveFilter = (activeFilter == 0) ? 1 : 0;

	CalcIIRFilterCoeff(cutoff, passType, IIRCoeff[inactiveFilter]);

	activeFilter = inactiveFilter;
	currentCutoff = cutoff;
}


//---------------------------------------------------------------------------

// We calculate the roots for a Butterworth filter directly. (No root finder needed)
// We fill the array Roots[] and return the number of roots.
int ButterworthPoly(int NumPoles, CplxD *Roots)
{
	int j, n;
	iirdouble Theta;

	n = 0;
	for (j = 0; j < NumPoles / 2; j++) {
		Theta = M_PI * (iirdouble)(2 * j + NumPoles + 1) / (iirdouble)(2 * NumPoles);
		Roots[n++] = CplxD(cos(Theta), sin(Theta));
		Roots[n++] = CplxD(cos(Theta), -sin(Theta));
	}
	if (NumPoles % 2 == 1)
		Roots[n++] = CplxD(-1.0, 0.0); // The real root for odd pole counts.
	return NumPoles;
}

// This code was described in "Elliptic Functions for Filter Design"
// H J Orchard and Alan N Willson  IEE Trans on Circuits and Systems April 97
// The equation numbers in the comments are from the paper.
// As the stop band attenuation -> infinity, the Elliptic -> Chebyshev.
int EllipticPoly(int FiltOrder, double Ripple, double DesiredSBdB, CplxD *EllipPoles, CplxD *EllipZeros, int *ZeroCount)
{
	int j, k, n, LastK;
	double K[ELLIPARRAYSIZE], G[ELLIPARRAYSIZE], Epsilon[ELLIPARRAYSIZE];
	double A, D, SBdB, dBErr, RealPart, ImagPart;
	double DeltaK, PrevErr, Deriv;
	CplxD C;

	for (j = 0; j < ELLIPARRAYSIZE; j++)
		K[j] = G[j] = Epsilon[j] = 0.0;
	if (Ripple < 0.001)
		Ripple = 0.001;
	if (Ripple > 1.0)
		Ripple = 1.0;
	Epsilon[0] = sqrt(pow(10.0, Ripple / 10.0) - 1.0);

	// Estimate K[0] to get the algorithm started.
	K[0] = (double)(FiltOrder-2) * 0.1605 + 0.016;
	if (K[0] < 0.01)
		K[0] = 0.01;
	if (K[0] > 0.7)
		K[0] = 0.7;

	// This loop calculates K[0] for the desired stopband attenuation. It typically loops < 5 times.
	for (j = 0; j<MAX_ELLIP_ITER; j++) {
		// Compute K with a forward Landen Transformation.
		for (k=1; k<10; k++) {
			K[k] = pow(K[k-1] / (1.0 + sqrt(1.0 - K[k-1]*K[k-1]) ), 2.0);   // eq. 10
			if (K[k] <= 1.0E-6)
				break;
		}
		LastK = k;

		// Compute G with a backwards Landen Transformation.
		G[LastK] = 4.0 * pow( K[LastK] / 4.0, (double)FiltOrder);
		for (k=LastK; k>=1; k--)
		{
			G[k-1] = 2.0 * sqrt(G[k]) / (1.0 + G[k]) ;  // eq. 9
		}

		if (G[0] <= 0.0)
			G[0] = 1.0E-10;
		SBdB = 10.0 * log10(1.0 + pow(Epsilon[0]/G[0], 2.0)); // Current stopband attenuation dB
		dBErr = DesiredSBdB - SBdB;

		if (fabs(dBErr) < 0.1)
			break;
		if (j == 0) { // Do this on the 1st loop so we can calc a derivative.
			if (dBErr > 0)
				DeltaK = 0.005;
			else DeltaK = -0.005;
			PrevErr = dBErr;
		} else {
			// Use Newtons Method to adjust K[0].
			Deriv = (PrevErr - dBErr)/DeltaK;
			PrevErr = dBErr;
			if (Deriv == 0.0)
				break; // This happens when K[0] hits one of the limits set below.
			DeltaK = dBErr/Deriv;
			if (DeltaK > 0.1)
				DeltaK = 0.1;
			if (DeltaK < -0.1)
				DeltaK = -0.1;
		}
		K[0] -= DeltaK;
		if (K[0] < 0.001)
			K[0] = 0.001;  // must not be < 0.0
		if (K[0] > 0.990)
			K[0] = 0.990;  // if > 0.990 we get a pole in the RHP. This means we were unable to set the stop band atten to the desired level (the Ripple is too large for the Pole Count).
	}


	// Epsilon[0] was calulated above, now calculate Epsilon[LastK] from G
	for (j = 1; j <= LastK; j++) {
		A = (1.0 + G[j]) * Epsilon[j-1] / 2.0;  // eq. 37
		Epsilon[j] = A + sqrt(A*A + G[j]);
	}

	// Calulate the poles and zeros.
	ImagPart = log((1.0 + sqrt(1.0 + Epsilon[LastK]*Epsilon[LastK])) / Epsilon[LastK] ) / (double)FiltOrder;  // eq. 22
	n = 0;
	for (j=1; j<=FiltOrder/2; j++) {
		RealPart = (double)(2*j - 1) * (M_PI / 2) / (double)FiltOrder;   // eq. 19
		C = CplxD(0.0, -1.0) / cos(CplxD(-RealPart, ImagPart));      // eq. 20
		D = 1.0 / cos(RealPart);
		for (k=LastK; k>=1; k--) {
			C = (C - K[k]/C) / (1.0 + K[k]);  // eq. 36
			D = (D + K[k]/D) / (1.0 + K[k]);
		}

		EllipPoles[n] = 1.0 / C;
		EllipPoles[n+1] = EllipPoles[n].conj();
		EllipZeros[n] = CplxD(0.0, D/K[0]);
		EllipZeros[n+1] = EllipZeros[n].conj();
		n+=2;
	}
	*ZeroCount = n; // n is the num zeros

	if (FiltOrder%2 == 1)   // The real pole for odd pole counts
	{
		A = 1.0 / sinh(ImagPart);
		for (k=LastK; k>=1; k--)
		{
			A = (A - K[k]/A) / (1.0 + K[k]);      // eq. 38
		}
		EllipPoles[n] = CplxD(-1.0/A, 0.0);
		n++;
	}

	return(n); // n is the num poles. There will be 1 more pole than zeros for odd pole counts.

}



// Remember to set the Index array to 0, 1, 2, 3, ... N-1
void HeapIndexSort(iirdouble *Data, int *Index, int N)
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

//---------------------------------------------------------------------------
// This sorts on the real part if the real part of the 1st root != 0 (a Zeta sort)
// else we sort on the imag part. If SortType == stMin for both the poles and zeros, then
// the poles and zeros will be properly matched.
// This also sets an inconsequential real or imag part to zero.
// A matched pair of z plane real roots, such as +/- 1, don't come out together.
// Used above in GetFilterCoeff and the FIR zero plot.
void SortRootsByZeta(CplxD *Roots, int Count)
{
	int j, k, RootJ[P51_ARRAY_SIZE];
	iirdouble SortValue[P51_ARRAY_SIZE];
	CplxD TempRoots[P51_ARRAY_SIZE];

	// Set an inconsequential real or imag part to zero. FIXME - not sure this is working or necessary
	for (j = 0; j < Count; j++)	{
		if (fabs(Roots[j].re) * 1.0E3 < fabs(Roots[j].im) ) {
			Roots[j].re = 0.0;
		}
		if (fabs(Roots[j].im) * 1.0E3 < fabs(Roots[j].re) ) {
			Roots[j].im = 0.0;
		}
	}

	// Sort the roots.
	for (j = 0; j < Count; j++)
		RootJ[j] = j;  // Needed for HeapIndexSort

	if (Roots[0].re != 0.0 ) {				// Sort on Real part
		for (j = 0; j < Count; j++)
			SortValue[j] = Roots[j].re;
	} else {  								// Sort on Imag part
		for (j = 0; j < Count; j++)
			SortValue[j] = fabs(Roots[j].im);
	}
	HeapIndexSort(SortValue, RootJ, Count);  // most negative root on top

	for (j = 0; j < Count; j++)	{
		k = RootJ[j];   // RootJ is the sort index
		TempRoots[j] = Roots[k];
	}
	for (j = 0; j < Count; j++)	{
		Roots[j] = TempRoots[j];
	}

}

//---------------------------------------------------------------------------

// Some of the Polys generate both left hand and right hand plane roots.
// We use this function to get the left hand plane poles and imag axis zeros to
// create the 2nd order polynomials with coefficients A2, A1, A0.
// We return the Polynomial count.

// We first sort the roots according the the real part (a zeta sort). Then all the left
// hand plane roots are grouped and in the correct order for IIR and Opamp filters.
// We then check for duplicate roots, and set an inconsequential real or imag part to zero.
// Then the 2nd order coefficients are calculated.
int filter::GetFilterCoeff(int RootCount, CplxD *Roots, iirdouble *A2, iirdouble *A1, iirdouble *A0)
{
	int PolyCount, j, k;

	SortRootsByZeta(Roots, RootCount);   // Places the most negative real part 1st.

	// Check for duplicate roots. The Inv Cheby generates duplicate imag roots, and the
	// Elliptic generates duplicate real roots. We set duplicates to a RHP value.
	for (j = 0; j < RootCount-1; j++) {
		for (k = j + 1; k < RootCount; k++) {
			if (fabs(Roots[j].re - Roots[k].re) < 1.0E-3 && fabs(Roots[j].im - Roots[k].im) < 1.0E-3) {
				Roots[k] = CplxD((iirdouble)k, 0.0); // RHP roots are ignored below, Use k is to prevent duplicate checks for matches.
			}
		}
	}

	// This forms the 2nd order coefficients from the root value.
	// We ignore roots in the Right Hand Plane.
	PolyCount = 0;
	for (j = 0; j < RootCount; j++) {
		if (Roots[j].re > 0.0)
			continue;						// Right Hand Plane
		if (Roots[j].re == 0.0 && Roots[j].im == 0.0)
			continue;						// At the origin.  This should never happen.

		if (Roots[j].re == 0.0) {			// Imag Root (A poly zero)
			A2[PolyCount] = 1.0;
			A1[PolyCount] = 0.0;
			A0[PolyCount] = Roots[j].im * Roots[j].im;
			j++;
			PolyCount++;
		} else if (Roots[j].im == 0.0) {	// Real Pole
			A2[PolyCount] = 0.0;
			A1[PolyCount] = 1.0;
			A0[PolyCount] = -Roots[j].re;
			PolyCount++;
		} else { 							// Complex Pole
			A2[PolyCount] = 1.0;
			A1[PolyCount] = -2.0 * Roots[j].re;
			A0[PolyCount] = Roots[j].re * Roots[j].re + Roots[j].im * Roots[j].im;
			j++;
			PolyCount++;
		}
	}

	return(PolyCount);

}

//---------------------------------------------------------------------------

// TLowPassParams defines the low pass prototype (NumPoles, Ripple, etc.).
// We return SPlaneCoeff filled with the 2nd order S plane coefficients.
TSPlaneCoeff filter::CalcLowPassProtoCoeff()
{
	int j, DenomCount, NumerCount, NumRoots;
	CplxD Poles[ARRAY_DIM];
	TSPlaneCoeff Coeff;  // The return value.

	// Init the S Plane Coeff. H(s) = (N2*s^2 + N1*s + N0) / (D2*s^2 + D1*s + D0)
	for (j = 0; j < ARRAY_DIM; j++) {
		Coeff.N2[j] = 0.0;
		Coeff.N1[j] = 0.0;
		Coeff.N0[j] = 1.0;
		Coeff.D2[j] = 0.0;
		Coeff.D1[j] = 0.0;
		Coeff.D0[j] = 1.0;
	}
	Coeff.NumSections = 0;


	// We need to range check the various argument values here.
	// These are the practical limits the max number of poles.
	if (iirSettings.NumPoles < 1)
		iirSettings.NumPoles = 1;
	if (iirSettings.NumPoles > MAX_POLE_COUNT)
		iirSettings.NumPoles = MAX_POLE_COUNT;
	if (iirSettings.ProtoType == ELLIPTIC || iirSettings.ProtoType == INVERSE_CHEBY) {
		if (iirSettings.NumPoles > 15)
			iirSettings.NumPoles = 15;
	}

	// Ripple is used by the Chebyshev and Elliptic
	if (iirSettings.Ripple < 0.0001)
		iirSettings.Ripple = 0.0001;
	if (iirSettings.Ripple > 1.0)
		iirSettings.Ripple = 1.0;


	// StopBanddB is used by the Inverse Chebyshev and the Elliptic
	// It is given in positive dB values.
	if (iirSettings.StopBanddB < 20.0)
		iirSettings.StopBanddB = 20.0;
	if (iirSettings.StopBanddB > 120.0)
		iirSettings.StopBanddB = 120.0;


	// There isn't such a thing as a 1 pole Chebyshev, or 1 pole Bessel, etc.
	// A one pole filter is simply 1/(s+1).
	NumerCount = 0; // init
	DenomCount = 1;
	if (iirSettings.NumPoles == 1) {
		Coeff.D1[0] = 1.0;
		DenomCount = 1;    // DenomCount is the number of denominator factors (1st or 2nd order).
	} else if (iirSettings.ProtoType == BUTTERWORTH) {
		NumRoots   = ButterworthPoly(iirSettings.NumPoles, Poles);

		if (debugSort) {
			for (int i = 0; i < NumRoots; ++i) {
				usb.SendString(std::to_string(i) + ": re=" + std::to_string(Poles[i].re) + " im=" + std::to_string(Poles[i].im).append("\n").c_str());
			}
		}

		DenomCount = GetFilterCoeff(NumRoots, Poles, Coeff.D2, Coeff.D1, Coeff.D0);
		// A Butterworth doesn't require frequncy scaling with SetCornerFreq().
	}  else if (iirSettings.ProtoType == ELLIPTIC) {
//		NumRoots   = EllipticPoly(iirSettings.NumPoles, iirSettings.Ripple, iirSettings.StopBanddB, Poles, Zeros, &ZeroCount);
//		DenomCount = GetFilterCoeff(NumRoots, Poles, Coeff.D2, Coeff.D1, Coeff.D0);
//		NumerCount = GetFilterCoeff(ZeroCount, Zeros, Coeff.N2, Coeff.N1, Coeff.N0);
//		SetCornerFreq(DenomCount, Coeff.D2, Coeff.D1, Coeff.D0, Coeff.N2, Coeff.N1, Coeff.N0);
	}


	Coeff.NumSections = DenomCount;

	// If we have an odd pole count, there will be 1 less zero than poles, so we need to shift the
	// zeros down in the arrays so the 1st zero (which is zero) and aligns with the real pole.
	if (NumerCount != 0 && iirSettings.NumPoles % 2 == 1) {
		for (j = NumerCount; j >= 0; j--) {
			Coeff.N2[j+1] = Coeff.N2[j];  // Coeff.N1's are always zero
			Coeff.N0[j+1] = Coeff.N0[j];
		}
		Coeff.N2[0] = 0.0;   // Set the 1st zero to zero for odd pole counts.
		Coeff.N0[0] = 1.0;
	}

	return(Coeff);

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
void filter::CalcIIRFilterCoeff(iirdouble OmegaC, PassType PassType, TIIRCoeff &iirCoeff)
{
	int j;

	iirdouble A, B, C, D, E, F, T, Q, Arg;

	TSPlaneCoeff SPlaneCoeff;     // Filled by the CalcLowPassProtoCoeff() function.

	// Get the low pass prototype 2nd order s plane coefficients.
	SPlaneCoeff = CalcLowPassProtoCoeff();


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
	T = 2.0 * tan(OmegaC * M_PI / 2);
	Q = 1.0 + OmegaC;             // Q is used for band pass and notch filters.
	if (Q > 1.95)
		Q = 1.95;
	Q = 0.8 * tan(Q * M_PI / 4);            // This is a correction factor for Q.
	Q = OmegaC / iirSettings.BW / Q;  // This is the corrected Q.


	// Calc the IIR coefficients.
	// SPlaneCoeff.NumSections is the number of 1st and 2nd order s plane factors.

	for (j = 0; j < SPlaneCoeff.NumSections; j++) {
		A = SPlaneCoeff.D2[j]; // We use A - F to make the code easier to read.
		B = SPlaneCoeff.D1[j];
		C = SPlaneCoeff.D0[j];
		D = SPlaneCoeff.N2[j];
		E = SPlaneCoeff.N1[j]; // N1 is always zero, except for the all pass. Consequently, the equations below can be simplified a bit by removing E.
		F = SPlaneCoeff.N0[j];

		// b's are the numerator  a's are the denominator
		if (PassType == LowPass) {
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

		if (PassType == HighPass) { // High Pass
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
	iirdouble y;
	int k = 0;

	y = SectCalc(0, sample, c);
	for (k = 1; k < IIRCoeff[activeFilter].NumSections; k++) {
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
	static bool MessageShown = false;

	// Zero the registers on the 1st call or on an overflow condition. The overflow limit used
	// here is small for iirdouble variables, but a filter that reaches this threshold is broken.
	int j = 1;
	if ((j == 0 && k == 0) || MaxRegVal > OVERFLOW_LIMIT) {
		if (MaxRegVal > OVERFLOW_LIMIT && !MessageShown) {
			// ShowMessage("ERROR: Math Over Flow in IIR Section Calc. \nThe register values exceeded 1.0E20 \n");
			MessageShown = true; // So this message doesn't get shown thousands of times.
		}

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
	if (fabs(CenterTap) > MaxRegVal)
		MaxRegVal = fabs(CenterTap);
	if (fabs(y) > MaxRegVal)
		MaxRegVal = fabs(y);
	return(y);
}




