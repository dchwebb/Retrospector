#include "SerialHandler.h"

#include <stdio.h>

extern DigitalDelay delay;
extern Config config;
extern Bootloader bootloader;

SerialHandler::SerialHandler(USB& usbObj)
{
	usb = &usbObj;

	// bind the usb's CDC caller to the CDC handler in this class
	usb->cdcDataHandler = std::bind(&SerialHandler::Handler, this, std::placeholders::_1, std::placeholders::_2);
}

void SerialHandler::suspendI2S()
{
	SPI2->CR1 |= SPI_CR1_CSUSP;
	while ((SPI2->SR & SPI_SR_SUSP) == 0);
}

void SerialHandler::resumeI2S()
{
	delay.LR = left;

	// to allow resume from debugging ensure suspended then clear buffer underrun flag
	SPI2->CR1 |= SPI_CR1_CSUSP;
	while ((SPI2->SR & SPI_SR_SUSP) == 0);
	SPI2->IFCR |= SPI_IFCR_UDRC;

	// Clear suspend state and resume
	SPI2->IFCR |= SPI_IFCR_SUSPC;
	while ((SPI2->SR & SPI_SR_SUSP) != 0);
	SPI2->CR1 |= SPI_CR1_CSTART;
}


// Check if a command has been received from USB, parse and action as required
bool SerialHandler::Command()
{
	if (!CmdPending) {
		return false;
	}

	// Provide option to switch to USB DFU mode - this allows the MCU to be programmed with STM32CubeProgrammer in DFU mode
	if (dfuConfirm) {
		if (ComCmd.compare("y\n") == 0 || ComCmd.compare("Y\n") == 0) {
			usb->SendString("Switching to DFU Mode ...\r\n");
			uint32_t old = SysTickVal;
			while (SysTickVal < old + 100) {};		// Give enough time to send the message
			void BootDFU();
			BootDFU();
		} else {
			dfuConfirm = false;
			usb->SendString("Upgrade cancelled\r\n");
		}

	} else if (ComCmd.compare("info\n") == 0) {		// Print diagnostic information

		usb->SendString("Mountjoy Retrospector - Current Settings:\r\n"
				"Chorus: " + std::string(delay.chorusMode ? "on" : "off") +
				"  PingPong: " + std::string(delay.pingPong ? "on\r\n" : "off\r\n"));

		char buf[50];

		usb->SendString(!filter.activateFilter ? "Filter: Off\r\n" : (filter.filterType == IIR) ? "Filter: IIR" : "Filter: FIR");
		if (filter.activateFilter) {
			sprintf(buf, "%0.10f", filter.currentCutoff);		// 10dp
			usb->SendString(std::string((filter.passType == LowPass) ? " Low Pass" : " High Pass") + ";  Cutoff: " + std::string(buf).append("\r\n"));
		}

		extern uint16_t adcZeroOffset[2];
		usb->SendString("Delay Times L: " + std::to_string(delay.calcDelay[left] / 48) + " ms, R: " + std::to_string(delay.calcDelay[right] / 48) + " ms\r\n" +
				std::string((delay.clockValid ? "Clock On": "Clock Off")) + ": interval: " + std::to_string(delay.clockInterval / 96) + " ms, " +
				std::to_string(delay.clockInterval) + " samples; Mult L: " + std::to_string(delay.delayMult[left]) + " R: " + std::to_string(delay.delayMult[right]) +"\r\n" +
				"ADC Zero offset L: " + std::to_string(adcZeroOffset[left]) + " R: " + std::to_string(adcZeroOffset[right]) + "\r\n" +
				"\r\n");

	} else if (ComCmd.compare("help\n") == 0) {

		usb->SendString("Mountjoy Retrospector\r\n"
				"\r\nSupported commands:\r\n"
				"help       -  Shows this information\r\n"
				"info       -  Show diagnostic information\r\n"
				"f          -  Filter on/off\r\n"
				"lp         -  Low pass IIR filter mode\r\n"
				"hp         -  High pass IIR filter mode\r\n"
				"both       -  Low pass to high pass FIR filter mode\r\n"
				"pp         -  Turn ping pong mode on/off\r\n"
				"c          -  Chorus on/off\r\n"
				"resume     -  Resume I2S after debugging\r\n"
				"dfu        -  USB firmware upgrade\r\n"
				"boot       -  Bootloader test\r\n"
				"save       -  Save calibration\r\n"
				"\r\nDebug Data Dump:\r\n"
				"dl         -  Left delay samples\r\n"
				"dr         -  Right delay samples\r\n"
				"fir        -  FIR coefficients\r\n"
				"iir        -  IIR coefficients\r\n"
				"fdl        -  Left filter buffer\r\n"
				"wd         -  FIR window coefficients\r\n"
				"imp        -  IIR impulse response\r\n"
				"cb         -  Chorus samples\r\n"
				"\r\n"
		);



	} else if (ComCmd.compare("dfu\n") == 0) {		// USB DFU firmware upgrade
		usb->SendString("Start DFU upgrade mode? Press 'y' to confirm.\r\n");
		dfuConfirm = true;

	} else if (ComCmd.compare("boot\n") == 0) {		// Test bootloader code
		bootloader.Receive();
		//void Bootloader();
		//Bootloader();

	} else if (ComCmd.compare("save\n") == 0) {		// Save calibration information
		suspendI2S();
		config.SaveConfig();
		resumeI2S();

	} else if (ComCmd.compare("resume\n") == 0) {	// Resume I2S after debugging
		resumeI2S();

	} else if (ComCmd.compare("f\n") == 0) {		// Activate filter

		filter.activateFilter = !filter.activateFilter;
		if (filter.activateFilter) {
			usb->SendString(std::string("Filter on\r\n").c_str());
		} else {
			usb->SendString(std::string("Filter off\r\n").c_str());
		}

	} else if (ComCmd.compare("lp\n") == 0) {		// Low Pass (IIR)

		filter.filterType = IIR;
		filter.filterControl = LP;
		filter.Update(true);
		usb->SendString(std::string("Low Pass\r\n").c_str());

	} else if (ComCmd.compare("hp\n") == 0) {		// High Pass (IIR)

		filter.filterType = IIR;
		filter.filterControl = HP;
		filter.Update(true);
		usb->SendString(std::string("High Pass\r\n").c_str());

	} else if (ComCmd.compare("both\n") == 0) {		// Low to High Pass sweep (FIR)

		filter.filterType = FIR;
		filter.filterControl = Both;
		filter.Update(true);
		usb->SendString(std::string("Low Pass to High Pass\r\n").c_str());

	} else if (ComCmd.compare("c\n") == 0) {		// Activate chorus mode
		delay.ChorusMode(!delay.chorusMode);
		usb->SendString(delay.chorusMode ? "Chorus on\r\n" : "Chorus off\r\n");

	} else if (ComCmd.compare("pp\n") == 0) {		// Activate ping pong mode
		delay.pingPong = !delay.pingPong;
		usb->SendString(delay.pingPong ? "PingPong on\r\n" : "PingPong off\r\n");

	} else if (ComCmd.compare("iir\n") == 0) {		// Show IIR Coefficients

		usb->SendString((filter.passType == LowPass) ? "Low Pass\r\n" : "High Pass\r\n");

		// Output coefficients
		IIRFilter& activeFilter = (filter.passType == LowPass) ? filter.iirLPFilter[filter.activeFilter] : filter.iirHPFilter[filter.activeFilter];
		for (int i = 0; i < activeFilter.numSections; ++i) {
			usb->SendString(std::to_string(i) + ": b0=" + std::to_string(activeFilter.iirCoeff.b0[i]) + " b1=" + std::to_string(activeFilter.iirCoeff.b1[i]) + " b2=" + std::to_string(activeFilter.iirCoeff.b2[i]).append("\r\n").c_str());
			usb->SendString(std::to_string(i) + ": a0=" + std::to_string(activeFilter.iirCoeff.a0[i]) + " a1=" + std::to_string(activeFilter.iirCoeff.a1[i]) + " a2=" + std::to_string(activeFilter.iirCoeff.a2[i]).append("\r\n").c_str());
		}


	} else if (ComCmd.compare("imp\n") == 0) {		// IIR Filter Print impulse response
		suspendI2S();

		IIRRegisters iirImpReg;		// Create a temporary set of shift registers for the filter
		IIRFilter& currentFilter = (filter.passType == LowPass) ? filter.iirLPFilter[0] : filter.iirHPFilter[0];

		float out = currentFilter.FilterSample(500, iirImpReg);		// Impulse
		usb->SendString(std::to_string(out).append("\r\n").c_str());

		for (int i = 1; i < 500; ++i) {
			out = currentFilter.FilterSample(0, iirImpReg);
			usb->SendString(std::to_string(out).append("\r\n").c_str());
		}

		resumeI2S();


	} else if (ComCmd.compare("cb\n") == 0) {		// Dump chorus buffer for L or R output
		suspendI2S();
		TIM2->CR1 &= ~TIM_CR1_CEN;

		usb->SendString("Write Pos: " + std::to_string(delay.chorusWrite) + "\r\n");
		for (int s = 0; s < 65536; ++s) {
			usb->SendString(std::to_string(chorusSamples[0][s]).append("\r\n").c_str());
		}

		TIM2->CR1 |= TIM_CR1_CEN;
		resumeI2S();


	} else if (ComCmd.compare("dl\n") == 0 || ComCmd.compare("dr\n") == 0) {		// Dump sample buffer for L or R output
		suspendI2S();

		channel LR = ComCmd.compare("dl\n") == 0 ? left : right;
		usb->SendString("Read Pos: " + std::to_string(delay.readPos[LR]) + "; Write Pos: " + std::to_string(delay.writePos) + "\r\n");
		for (int s = 0; s < 1000; ++s) {
			StereoSample samp = {samples[s]};
			usb->SendString(std::to_string(samp.sample[LR]).append("\r\n").c_str());
		}

		resumeI2S();

	} else if (ComCmd.compare("fir\n") == 0) {		// Dump FIR filter coefficients
		suspendI2S();

		// NB to_string not working. Use sprintf with following: The float formatting support is not enabled, check your MCU Settings from "Project Properties > C/C++ Build > Settings > Tool Settings",
		// or add manually "-u _printf_float" in linker flags
		char buf[50];
		for (int f = 0; f < Filter::firTaps; ++f) {
			sprintf(buf, "%0.10f", filter.firCoeff[filter.activeFilter][f]);		// 10dp
			std::string ts = std::string(buf);
			usb->SendString(ts.append("\r\n").c_str());
		}

		resumeI2S();

	} else if (ComCmd.compare("wd\n") == 0) {		// Dump filter window
		suspendI2S();

		char buf[50];
		for (int f = 0; f < Filter::firTaps; ++f) {
			sprintf(buf, "%0.10f", filter.winCoeff[f]);		// 10dp
			std::string ts = std::string(buf);
			usb->SendString(ts.append("\r\n").c_str());
		}

		resumeI2S();

	} else if (ComCmd.compare("fdl\n") == 0) {		// Dump left filter buffer
		suspendI2S();

		uint16_t pos = filter.filterBuffPos[0];
		for (int f = 0; f < Filter::firTaps; ++f) {
			usb->SendString(std::to_string(filter.filterBuffer[0][pos]).append("\r\n").c_str());
			if (++pos == Filter::firTaps) pos = 0;
		}

		resumeI2S();

	} else {
		usb->SendString("Unrecognised command: " + ComCmd + "Type 'help' for supported commands\r\n");
	}

	CmdPending = false;
	return true;
}


void SerialHandler::Handler(uint8_t* data, uint32_t length)
{
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

