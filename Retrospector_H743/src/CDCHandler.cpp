#include "CDCHandler.h"
#include <stdio.h>

extern volatile bool sampleClock;
extern bool activateFilter;
extern uint16_t currentTone;

volatile bool CmdPending = false;
std::string ComCmd;

void suspendI2S() {
	SPI2->CR1 |= SPI_CR1_CSUSP;
	while ((SPI2->SR & SPI_SR_SUSP) == 0);
}

void resumeI2S() {
	sampleClock = true;

	// to allow resume from debugging ensure suspended then clear buffer underrun flag
	SPI2->CR1 |= SPI_CR1_CSUSP;
	while ((SPI2->SR & SPI_SR_SUSP) == 0);
	SPI2->IFCR |= SPI_IFCR_UDRC;

	// Clear suspend state and resume
	SPI2->IFCR |= SPI_IFCR_SUSPC;
	while ((SPI2->SR & SPI_SR_SUSP) != 0);
	SPI2->CR1 |= SPI_CR1_CSTART;
}

bool CDCCommand(const std::string ComCmd) {

	if (ComCmd.compare("help\n") == 0) {
		usb.SendString("Mountjoy Retrospector - supported commands:\n\n\r"
				"help      -  Shows this information\n\r"
				"dl        -  Dump samples for left channel\n\r"
				"dr        -  Dump samples for right channel\n\r"
				"f         -  Filter on/off\n\r"
				"lp        -  Filter is low pass\n\r"
				"hp        -  Filter is high pass\n\r"
				"both      -  Filter sweeps from low pass to high pass\n\r"
				"fd        -  Dump filter coefficients\n\r"
				"fdl       -  Dump left filter buffer\n\r"
				"wd        -  Dump filter window coefficients\n\r"
				"imp       -  IIR impulse response\n\r"
				"iirs      -  IIR square wave test\n\r"
				"iir       -  Activate IIR filter\n\r"
				"fir       -  Activate FIR filter\n\r"
		);

	} else if (ComCmd.compare("fir\n") == 0) {		// Activate FIR

		if (filter.filterType == IIR) {
			usb.SendString(std::string("FIR Filter Activated\n").c_str());
			filter.filterType = FIR;
			currentTone = 0;
		}

		char buf[50];
		sprintf(buf, "%0.10f", currentCutoff);		// 10dp
		usb.SendString(std::string("FIR Filter: ").append(std::string(buf)).append("\n").c_str());

	} else if (ComCmd.compare("iir\n") == 0) {		// Activate IIR

		if (filter.filterType == FIR) {
			filter.filterType = IIR;
			currentTone = 0;			// Force reset
			usb.SendString("IIR Filter Activated\n");
		}

		// Output coefficients
		IIRFilter& activeFilter = (filter.passType == LowPass) ? filter.iirLPFilter[filter.activeFilter] : filter.iirHPFilter[filter.activeFilter];
		for (int i = 0; i < activeFilter.numSections; ++i) {
			usb.SendString(std::to_string(i) + ": b0=" + std::to_string(activeFilter.iirCoeff.b0[i]) + " b1=" + std::to_string(activeFilter.iirCoeff.b1[i]) + " b2=" + std::to_string(activeFilter.iirCoeff.b2[i]).append("\n").c_str());
			usb.SendString(std::to_string(i) + ": a0=" + std::to_string(activeFilter.iirCoeff.a0[i]) + " a1=" + std::to_string(activeFilter.iirCoeff.a1[i]) + " a2=" + std::to_string(activeFilter.iirCoeff.a2[i]).append("\n").c_str());
		}


	} else if (ComCmd.compare("imp\n") == 0) {		// IIR Filter test
		suspendI2S();

		float out;
		int i;

		//out = Filter.IIRFilter(500, left);		// Impulse
		usb.SendString(std::to_string(out).append("\n").c_str());

		for (i = 1; i < 500; ++i) {
			//out = Filter.IIRFilter(0, left);
			usb.SendString(std::to_string(out).append("\n").c_str());
		}

		resumeI2S();

	} else if (ComCmd.compare("iirs\n") == 0) {		// IIR Filter test on square wave
		suspendI2S();
		/*
		float out;
		int i;

		for (i = 1; i < 1000; ++i) {
			if (i < 50) {
				out = Filter.IIRFilter(0, left);
			} else if (i < 100) {
				out = Filter.IIRFilter(-30000, left);
			} else if (i < 150) {
				out = Filter.IIRFilter(30000, left);
			} else if (i < 200) {
				out = Filter.IIRFilter(-30000, left);
			} else if (i < 250) {
				out = Filter.IIRFilter(30000, left);
			} else if (i < 300) {
				out = Filter.IIRFilter(-30000, left);
			} else if (i < 350) {
				out = Filter.IIRFilter(30000, left);
			} else {
				out = Filter.IIRFilter(0, left);
			}

			usb.SendString(std::to_string(out).append("\n").c_str());
		}
*/
		resumeI2S();

	} else if (ComCmd.compare("dl\n") == 0 || ComCmd.compare("dr\n") == 0) {		// Dump sample buffer for L or R output
		suspendI2S();

		channel LOrR = ComCmd.compare("dl\n") == 0 ? left : right;
		usb.SendString("Read Pos: " + std::to_string(DigitalDelay.readPos[LOrR]) + "; Write Pos: " + std::to_string(DigitalDelay.writePos[LOrR]) + "\n");
		for (int s = 0; s < SAMPLE_BUFFER_LENGTH; ++s) {
			usb.SendString(std::to_string(samples[LOrR][s]).append("\n").c_str());
		}

		resumeI2S();

	} else if (ComCmd.compare("f\n") == 0) {		// Activate filter

		activateFilter = !activateFilter;
		if (activateFilter) {
			usb.SendString(std::string("Filter on\n").c_str());
		} else {
			usb.SendString(std::string("Filter off\n").c_str());
		}

	} else if (ComCmd.compare("lp\n") == 0) {		// Activate filter

		filter.filterControl = LP;
		usb.SendString(std::string("Low Pass\n").c_str());

	} else if (ComCmd.compare("hp\n") == 0) {		// Activate filter

		filter.filterControl = HP;
		usb.SendString(std::string("High Pass\n").c_str());

	} else if (ComCmd.compare("both\n") == 0) {		// Activate filter

		filter.filterControl = Both;
		usb.SendString(std::string("Low Pass to High Pass\n").c_str());

	} else if (ComCmd.compare("w\n") == 0) {		// Activate filter window

		activateWindow = !activateWindow;
		extern uint16_t currentTone;
		currentTone = 0;		// force filter recalculation

		if (activateWindow) {
			usb.SendString(std::string("Window on\n").c_str());
		} else {
			usb.SendString(std::string("Window off\n").c_str());
		}

	} else if (ComCmd.compare("fd\n") == 0) {		// Dump filter coefficients
		suspendI2S();

		/* NB to_string not working. Use sprintf with following: The float formatting support is not enabled, check your MCU Settings from "Project Properties > C/C++ Build > Settings > Tool Settings",
		or add manually "-u _printf_float" in linker flags */
		char buf[50];
		for (int f = 0; f < FIRTAPS; ++f) {
			sprintf(buf, "%0.10f", filter.firCoeff[filter.activeFilter][f]);		// 10dp
			std::string ts = std::string(buf);
			usb.SendString(ts.append("\n").c_str());
		}

		resumeI2S();

	} else if (ComCmd.compare("wd\n") == 0) {		// Dump filter window
		suspendI2S();

		/* NB to_string not working. Use sprintf with following: The float formatting support is not enabled, check your MCU Settings from "Project Properties > C/C++ Build > Settings > Tool Settings",
		or add manually "-u _printf_float" in linker flags */
		char buf[50];
		for (int f = 0; f < FIRTAPS; ++f) {
			sprintf(buf, "%0.10f", filter.winCoeff[f]);		// 10dp
			std::string ts = std::string(buf);
			usb.SendString(ts.append("\n").c_str());
		}

		resumeI2S();

	} else if (ComCmd.compare("fdl\n") == 0) {		// Dump left filter buffer
		suspendI2S();

		uint16_t pos = DigitalDelay.filterBuffPos[0];
		for (int f = 0; f < FIRTAPS; ++f) {
			usb.SendString(std::to_string(filterBuffer[0][pos]).append("\n").c_str());
			if (++pos == FIRTAPS) pos = 0;
		}

		resumeI2S();

	} else {
		return false;
	}

	return true;
}


void CDCHandler(uint8_t* data, uint32_t length) {
	static bool newCmd = true;
	if (newCmd) {
		ComCmd = std::string((char*)data, length);
		newCmd = false;
	} else {
		ComCmd.append((char*)data, length);
	}
	if (*ComCmd.rbegin() == '\r')
		*ComCmd.rbegin() = '\n';

	if (*ComCmd.rbegin() == '\n') {
		CmdPending = true;
		newCmd = true;
	}
}

