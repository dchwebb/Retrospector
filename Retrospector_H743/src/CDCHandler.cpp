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
	SPI2->IFCR |= SPI_IFCR_SUSPC;
	while ((SPI2->SR & SPI_SR_SUSP) != 0);
	SPI2->CR1 |= SPI_CR1_CSTART;
}

bool CDCCommand(const std::string ComCmd) {

	if (ComCmd.compare("help\n") == 0) {
		usb.SendString("Mountjoy Retrospector - supported commands:\n\n"
				"help      -  Shows this information\n"
				"dl        -  Dump samples for left channel\n"
				"dr        -  Dump samples for right channel\n"
				"f         -  Filter on/off\n"
				"fd        -  Dump filter coefficients\n"
				"fdl       -  Dump left filter buffer\n"
				"wd        -  Dump filter window coefficients\n"
				"imp       -  IIR impulse response\n"
				"iirs      -  IIR square wave test\n"
				"iir       -  Activate IIR filter\n"
				"fir       -  Activate FIR filter\n"
		);

	} else if (ComCmd.compare("fir\n") == 0) {		// Activate FIR

		if (iirFilter) {
			usb.SendString(std::string("FIR Filter Activated\n").c_str());
			iirFilter = false;
			currentTone = 0;
		}

		char buf[50];
		sprintf(buf, "%0.10f", currentCutoff);		// 10dp
		usb.SendString(std::string("FIR Filter: ").append(std::string(buf)).append("\n").c_str());

	} else if (ComCmd.compare("iir\n") == 0) {		// Activate IIR

		if (!iirFilter) {
			iirFilter = true;
			currentTone = 0;			// Force reset
			usb.SendString("IIR Filter Activated\n");
		}

		// Output coefficients
		for (int i = 0; i < Filter.IIRCoeff[activeFilter].NumSections; ++i) {
			usb.SendString(std::to_string(i) + ": b0=" + std::to_string(Filter.IIRCoeff[activeFilter].b0[i]) + " b1=" + std::to_string(Filter.IIRCoeff[activeFilter].b1[i]) + " b2=" + std::to_string(Filter.IIRCoeff[activeFilter].b2[i]).append("\n").c_str());
			usb.SendString(std::to_string(i) + ": a0=" + std::to_string(Filter.IIRCoeff[activeFilter].a0[i]) + " a1=" + std::to_string(Filter.IIRCoeff[activeFilter].a1[i]) + " a2=" + std::to_string(Filter.IIRCoeff[activeFilter].a2[i]).append("\n").c_str());
		}

	} else if (ComCmd.compare("iirsort\n") == 0) {		// IIR Sort test
		suspendI2S();

		debugSort = true;
		for (int tone = 0; tone < 65536; tone += 1000) {
			Filter.InitIIRFilter(tone);
		}

		resumeI2S();


	} else if (ComCmd.compare("imp\n") == 0) {		// IIR Filter test
		suspendI2S();

		float out;
		int i;

		out = Filter.IIRFilter(IIRSAMPLECOUNT, left);		// Impulse
		usb.SendString(std::to_string(out).append("\n").c_str());

		for (i = 1; i < IIRSAMPLECOUNT; ++i) {
			out = Filter.IIRFilter(0, left);
			usb.SendString(std::to_string(out).append("\n").c_str());
		}

		resumeI2S();

	} else if (ComCmd.compare("iirs\n") == 0) {		// IIR Filter test on square wave
		suspendI2S();

		float out;
		int i;

		for (i = 1; i < IIRSAMPLECOUNT; ++i) {
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
			sprintf(buf, "%0.10f", firCoeff[activeFilter][f]);		// 10dp
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
			sprintf(buf, "%0.10f", winCoeff[f]);		// 10dp
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

