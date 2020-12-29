#include "CDCHandler.h"
#include <stdio.h>

extern volatile bool sampleClock;
extern bool activateFilter;

volatile bool CmdPending = false;
std::string ComCmd;

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
		);
	} else if (ComCmd.compare("dl\n") == 0 || ComCmd.compare("dr\n") == 0) {		// Dump sample buffer for L or R output
		// Suspend I2S
		SPI2->CR1 |= SPI_CR1_CSUSP;
		while ((SPI2->SR & SPI_SR_SUSP) == 0);

		channel LOrR = ComCmd.compare("dl\n") == 0 ? left : right;

		usb.SendString("Read Pos: " + std::to_string(DigitalDelay.readPos[LOrR]) + "; Write Pos: " + std::to_string(DigitalDelay.writePos[LOrR]) + "\n");

		for (int s = 0; s < SAMPLE_BUFFER_LENGTH; ++s) {
			usb.SendString(std::to_string(samples[LOrR][s]).append("\n").c_str());
		}

		// Resume I2S
		sampleClock = true;
		SPI2->IFCR |= SPI_IFCR_SUSPC;
		while ((SPI2->SR & SPI_SR_SUSP) != 0);
		SPI2->CR1 |= SPI_CR1_CSTART;

	} else if (ComCmd.compare("f\n") == 0) {		// Activate filter

		activateFilter = !activateFilter;

		char buf[50];
		sprintf(buf, "%0.10f", currentCutoff);		// 10dp


		if (activateFilter) {
			//GPIOC->ODR |= GPIO_ODR_OD11;			// Toggle LED for testing
			usb.SendString(std::string("Filter on: ").append(std::string(buf)).append("\n").c_str());
		} else {
			usb.SendString(std::string("Filter off\n").c_str());
			//GPIOC->ODR &= ~GPIO_ODR_OD11;			// Toggle LED for testing
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
		// Suspend I2S
		SPI2->CR1 |= SPI_CR1_CSUSP;
		while ((SPI2->SR & SPI_SR_SUSP) == 0);

		/* NB to_string not working. Use sprintf with following: The float formatting support is not enabled, check your MCU Settings from "Project Properties > C/C++ Build > Settings > Tool Settings",
		or add manually "-u _printf_float" in linker flags */


		char buf[50];
		for (int f = 0; f < FIRTAPS; ++f) {
			sprintf(buf, "%0.10f", firCoeff[activeFilter][f]);		// 10dp
			std::string ts = std::string(buf);
			usb.SendString(ts.append("\n").c_str());
		}

		// Resume I2S
		sampleClock = true;
		SPI2->IFCR |= SPI_IFCR_SUSPC;
		while ((SPI2->SR & SPI_SR_SUSP) != 0);

		SPI2->CR1 |= SPI_CR1_CSTART;

	} else if (ComCmd.compare("wd\n") == 0) {		// Dump filter window
		// Suspend I2S
		SPI2->CR1 |= SPI_CR1_CSUSP;
		while ((SPI2->SR & SPI_SR_SUSP) == 0);

		/* NB to_string not working. Use sprintf with following: The float formatting support is not enabled, check your MCU Settings from "Project Properties > C/C++ Build > Settings > Tool Settings",
		or add manually "-u _printf_float" in linker flags */

		char buf[50];
		for (int f = 0; f < FIRTAPS; ++f) {
			sprintf(buf, "%0.10f", winCoeff[f]);		// 10dp
			std::string ts = std::string(buf);
			usb.SendString(ts.append("\n").c_str());
		}

		// Resume I2S
		sampleClock = true;
		SPI2->IFCR |= SPI_IFCR_SUSPC;
		while ((SPI2->SR & SPI_SR_SUSP) != 0);

		SPI2->CR1 |= SPI_CR1_CSTART;

	} else if (ComCmd.compare("fdl\n") == 0) {		// Dump left filter buffer
		// Suspend I2S
		SPI2->CR1 |= SPI_CR1_CSUSP;
		while ((SPI2->SR & SPI_SR_SUSP) == 0);

		uint16_t pos = DigitalDelay.filterBuffPos[0];
		for (int f = 0; f < FIRTAPS; ++f) {
			usb.SendString(std::to_string(filterBuffer[0][pos]).append("\n").c_str());
			if (++pos == FIRTAPS) pos = 0;
		}

		// Resume I2S
		sampleClock = true;
		SPI2->IFCR |= SPI_IFCR_SUSPC;
		while ((SPI2->SR & SPI_SR_SUSP) != 0);
		SPI2->CR1 |= SPI_CR1_CSTART;


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

