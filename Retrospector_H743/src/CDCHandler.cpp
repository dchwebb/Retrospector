#include "CDCHandler.h"
#include <stdio.h>

extern volatile bool sampleClock;
extern bool activateFilter;
extern uint16_t currentTone;
extern int32_t dampedTone;
extern volatile bool checkFilt;

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
		usb.SendString("Mountjoy Retrospector - supported commands:\r\n\r\n"
				"help      -  Shows this information\r\n"
				"dl        -  Dump samples for left channel\r\n"
				"dr        -  Dump samples for right channel\r\n"
				"f         -  Filter on/off\r\n"
				"lp        -  Filter is low pass\r\n"
				"hp        -  Filter is high pass\r\n"
				"both      -  Filter sweeps from low pass to high pass\r\n"
				"fd        -  Dump filter coefficients\r\n"
				"fdl       -  Dump left filter buffer\r\n"
				"wd        -  Dump filter window coefficients\r\n"
				"imp       -  IIR impulse response\r\n"
				"iirs      -  IIR square wave test\r\n"
				"iir       -  Activate IIR filter\r\n"
				"fir       -  Activate FIR filter\r\n"
				"resume    -  Resume I2S after debugging\r\n"
				"pp        -  Turn ping pong mode on/off\r\n"
		);
	} else if (ComCmd.compare("cf\n") == 0) {		// Resume I2S after debugging

		checkFilt = !checkFilt;
		usb.SendString(checkFilt ? "Folded FIR on\r\n" : "Folded FIR off\r\n");

	} else if (ComCmd.compare("resume\n") == 0) {		// Resume I2S after debugging
		resumeI2S();

	} else if (ComCmd.compare("pp\n") == 0) {		// Resume I2S after debugging
		DigitalDelay.pingPong = !DigitalDelay.pingPong;
		usb.SendString(DigitalDelay.pingPong ? "PingPong on\r\n" : "PingPong off\r\n");

	} else if (ComCmd.compare("fir\n") == 0) {		// Activate FIR

		if (filter.filterType == IIR) {
			usb.SendString(std::string("FIR Filter Activated\r\n").c_str());
			filter.filterType = FIR;
			currentTone = 0;
		}

		char buf[50];
		sprintf(buf, "%0.10f", filter.currentCutoff);		// 10dp
		usb.SendString(std::string("FIR Filter: ").append(std::string(buf)).append("\r\n").c_str());

	} else if (ComCmd.compare("iir\n") == 0) {		// Activate IIR

		if (filter.filterType == FIR) {
			filter.filterType = IIR;
			currentTone = 0;			// Force reset
			usb.SendString("IIR Filter Activated\r\n");
		}

		usb.SendString((filter.passType == LowPass) ? "Low Pass\r\n" : "High Pass\r\n");

		// Output coefficients
		IIRFilter& activeFilter = (filter.passType == LowPass) ? filter.iirLPFilter[filter.activeFilter] : filter.iirHPFilter[filter.activeFilter];
		for (int i = 0; i < activeFilter.numSections; ++i) {
			usb.SendString(std::to_string(i) + ": b0=" + std::to_string(activeFilter.iirCoeff.b0[i]) + " b1=" + std::to_string(activeFilter.iirCoeff.b1[i]) + " b2=" + std::to_string(activeFilter.iirCoeff.b2[i]).append("\r\n").c_str());
			usb.SendString(std::to_string(i) + ": a0=" + std::to_string(activeFilter.iirCoeff.a0[i]) + " a1=" + std::to_string(activeFilter.iirCoeff.a1[i]) + " a2=" + std::to_string(activeFilter.iirCoeff.a2[i]).append("\r\n").c_str());
		}


	} else if (ComCmd.compare("imp\n") == 0) {		// IIR Filter test
		suspendI2S();

		float out;
		int i;

		//out = Filter.IIRFilter(500, left);		// Impulse
		usb.SendString(std::to_string(out).append("\r\n").c_str());

		for (i = 1; i < 500; ++i) {
			//out = Filter.IIRFilter(0, left);
			usb.SendString(std::to_string(out).append("\r\n").c_str());
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

			usb.SendString(std::to_string(out).append("\r\n").c_str());
		}
*/
		resumeI2S();

	} else if (ComCmd.compare("dl\n") == 0 || ComCmd.compare("dr\n") == 0) {		// Dump sample buffer for L or R output
		suspendI2S();

		channel LOrR = ComCmd.compare("dl\n") == 0 ? left : right;
		usb.SendString("Read Pos: " + std::to_string(DigitalDelay.readPos[LOrR]) + "; Write Pos: " + std::to_string(DigitalDelay.writePos[LOrR]) + "\r\n");
		for (int s = 0; s < SAMPLE_BUFFER_LENGTH; ++s) {
			//usb.SendString(std::to_string(samples[LOrR][s]).append("\r\n").c_str());
		}

		resumeI2S();

	} else if (ComCmd.compare("f\n") == 0) {		// Activate filter

		activateFilter = !activateFilter;
		if (activateFilter) {
			usb.SendString(std::string("Filter on\r\n").c_str());
		} else {
			usb.SendString(std::string("Filter off\r\n").c_str());
		}

	} else if (ComCmd.compare("lp\n") == 0) {		// Activate filter

		filter.filterType = IIR;
		filter.filterControl = LP;
		currentTone = dampedTone + 1000;			// to force update
		usb.SendString(std::string("Low Pass\r\n").c_str());

	} else if (ComCmd.compare("hp\n") == 0) {		// Activate filter

		filter.filterType = IIR;
		filter.filterControl = HP;
		currentTone = dampedTone + 1000;			// to force update
		usb.SendString(std::string("High Pass\r\n").c_str());

	} else if (ComCmd.compare("both\n") == 0) {		// Activate filter

		filter.filterType = FIR;
		filter.filterControl = Both;
		currentTone = dampedTone + 1000;			// to force update
		usb.SendString(std::string("Low Pass to High Pass\r\n").c_str());

	} else if (ComCmd.compare("w\n") == 0) {		// Activate filter window

		activateWindow = !activateWindow;
		extern uint16_t currentTone;
		currentTone = 0;		// force filter recalculation

		if (activateWindow) {
			usb.SendString(std::string("Window on\r\n").c_str());
		} else {
			usb.SendString(std::string("Window off\r\n").c_str());
		}

	} else if (ComCmd.compare("fd\n") == 0) {		// Dump filter coefficients
		suspendI2S();

		/* NB to_string not working. Use sprintf with following: The float formatting support is not enabled, check your MCU Settings from "Project Properties > C/C++ Build > Settings > Tool Settings",
		or add manually "-u _printf_float" in linker flags */
		char buf[50];
		for (int f = 0; f < FIRTAPS; ++f) {
			sprintf(buf, "%0.10f", filter.firCoeff[filter.activeFilter][f]);		// 10dp
			std::string ts = std::string(buf);
			usb.SendString(ts.append("\r\n").c_str());
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
			usb.SendString(ts.append("\r\n").c_str());
		}

		resumeI2S();

	} else if (ComCmd.compare("fdl\n") == 0) {		// Dump left filter buffer
		suspendI2S();

		uint16_t pos = filter.filterBuffPos[0];
		for (int f = 0; f < FIRTAPS; ++f) {
			usb.SendString(std::to_string(filter.filterBuffer[0][pos]).append("\r\n").c_str());
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

