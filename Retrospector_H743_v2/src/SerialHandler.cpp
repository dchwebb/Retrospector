#include "SerialHandler.h"

#include <stdio.h>

extern DigitalDelay delay;
extern Config config;
extern Bootloader bootloader;

int16_t ParseInt(const std::string cmd, const char precedingChar) {
	uint16_t val = -1;
	int8_t pos = cmd.find(precedingChar);		// locate position of character preceding
	if (pos >= 0 && std::strspn(cmd.substr(pos + 1).c_str(), "0123456789") > 0) {
		val = stoi(cmd.substr(pos + 1));
	}
	return val;
}

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
	if (state == serialState::dfuConfirm) {
		if (ComCmd.compare("y\n") == 0 || ComCmd.compare("Y\n") == 0) {
			usb->SendString("Switching to DFU Mode ...\r\n");
			uint32_t old = SysTickVal;
			while (SysTickVal < old + 100) {};		// Give enough time to send the message
			bootloader.BootDFU();
		} else {
			state = serialState::pending;
			usb->SendString("Upgrade cancelled\r\n");
		}

	} else if (state == serialState::calibConfirm) {
		if (ComCmd.compare("y\n") == 0 || ComCmd.compare("Y\n") == 0) {
			config.Calibrate();
			resumeI2S();
		} else {
			usb->SendString("Calibration cancelled\r\n");
		}
		state = serialState::pending;

	} else if (state == serialState::cancelAudioTest && ComCmd.compare("dl\n") != 0 && ComCmd.compare("dr\n") != 0) {
		delay.testMode = delay.TestMode::none;
		usb->SendString("Audio test cancelled\r\n");
		state = serialState::pending;

	} else if (ComCmd.compare("info\n") == 0) {		// Print diagnostic information

		usb->SendString("Mountjoy Retrospector - Current Settings:\r\n"
				"Chorus: " + std::string(delay.chorusMode ? "on" : "off") +
				"  Stereo wide: " + std::string(delay.stereoWide ? "on\r\n" : "off\r\n"));

		char buf[50];

		usb->SendString(!filter.activateFilter ? "Filter: Off\r\n" : (filter.filterType == IIR) ? "Filter: IIR" : "Filter: FIR");
		if (filter.activateFilter) {
			sprintf(buf, "%0.10f", filter.currentCutoff);		// 10dp
			usb->SendString(std::string((filter.passType == LowPass) ? " Low Pass" : " High Pass") + ";  Cutoff: " + std::string(buf).append("\r\n"));
		}

		usb->SendString("Delay Times L: " + std::to_string(delay.calcDelay[left] / 48) + " ms, R: " + std::to_string(delay.calcDelay[right] / 48) + " ms\r\n" +
				std::string((delay.clockValid ? "Clock On": "Clock Off")) + ": interval: " + std::to_string(delay.clockInterval / 96) + " ms, " +
				std::to_string(delay.clockInterval) + " samples; Mult L: " + std::to_string(delay.delayMult[left]) + " R: " + std::to_string(delay.delayMult[right]) +"\r\n" +
				"ADC Zero offset L: " + std::to_string(adcZeroOffset[left]) + " R: " + std::to_string(adcZeroOffset[right]) + "\r\n" +
				"LEDs Filter R:" + std::to_string(led.colour[6]) + " G: " + std::to_string(led.colour[7]) + " B: " + std::to_string(led.colour[8]) + "\r\n" +
				"Gate threshold: " + std::to_string(delay.gateThreshold) + " Activation time: " + std::to_string(delay.gateHoldCount)+ "\r\n" +
				"\r\n");

	} else if (ComCmd.compare("help\n") == 0) {

		usb->SendString("Mountjoy Retrospector\r\n"
				"\r\nSupported commands:\r\n"
				"help        -  Shows this information\r\n"
				"info        -  Show diagnostic information\r\n"
				"f           -  Filter on/off\r\n"
				"led         -  LEDs on/off\r\n"
				"resume      -  Resume I2S after debugging\r\n"
				"dfu         -  USB firmware upgrade\r\n"
				"boot        -  Bootloader test\r\n"
				"calib       -  Calibrate device\r\n"
				"threshold:x -  Configure gate threshold to x (default 100)\r\n"
				"gateact:x   -  Configure gate activate time to x samples (default 20000)\r\n"
				"gateled     -  Show gate status on filter LED\r\n"
				"save        -  Save calibration\r\n"
				"\r\nRun Tests:\r\n"
				"mem16       -  Start/stop Memory Test of lower 16MB\r\n"
				"mem32       -  Start/stop Memory Test of all 32MB\r\n"
				"loop        -  Run an audio loopback test\r\n"
				"saw         -  Generate a 1kHz saw tooth wave\r\n"
				"\r\nDebug Data Dump:\r\n"
				"dl          -  Most recent Left delay samples\r\n"
				"dr          -  Most recent Right delay samples\r\n"
				"fir         -  FIR coefficients\r\n"
				"iir         -  IIR coefficients\r\n"
				"fdl         -  Left filter buffer\r\n"
				"wd          -  FIR window coefficients\r\n"
				"imp         -  IIR impulse response\r\n"
				"cb          -  Chorus samples\r\n"
				"\r\n"
		);

	} else if (ComCmd.compare("gateled\n") == 0) {	// Configure gate LED
		delay.gateLED = !delay.gateLED;
		usb->SendString("Toggle Filter LED displaying gate status\r\n");
		if (!delay.gateLED) {
			filter.Update(true);
		}

	} else if (ComCmd.compare(0, 10, "threshold:") == 0) {	// Configure gate threshold
		uint16_t threshold = ParseInt(ComCmd, ':');
		delay.gateThreshold = threshold;
		usb->SendString("Gate threshold set to: " + std::to_string(delay.gateThreshold) + "\r\n");
		suspendI2S();
		config.SaveConfig();
		resumeI2S();

	} else if (ComCmd.compare(0, 8, "gateact:") == 0) {	// Configure gate threshold
		uint16_t gate = ParseInt(ComCmd, ':');
		delay.gateHoldCount = gate;
		usb->SendString("Gate activate time set to: " + std::to_string(delay.gateHoldCount) + "\r\n");
		suspendI2S();
		config.SaveConfig();
		resumeI2S();

	} else if (ComCmd.compare("loop\n") == 0) {		// Audio loopback test
		usb->SendString("Starting audio loopback test. Press any key to cancel.\r\n");
		delay.testMode = delay.TestMode::loop;
		state = serialState::cancelAudioTest;

	} else if (ComCmd.compare("saw\n") == 0) {		// Audio loopback test
		usb->SendString("Generating saw wave. Press any key to cancel\r\n");
		delay.testMode = delay.TestMode::saw;
		state = serialState::cancelAudioTest;

	} else if (ComCmd.compare("dfu\n") == 0) {		// USB DFU firmware upgrade
		usb->SendString("Start DFU upgrade mode? Press 'y' to confirm.\r\n");
		state = serialState::dfuConfirm;

	} else if (ComCmd.compare("boot\n") == 0) {		// Test bootloader code
		bootloader.Receive();
		//void Bootloader();
		//Bootloader();

	} else if (ComCmd.compare("calib\n") == 0) {	// Calibrate filter pot center and audio offsets

		usb->SendString("Remove cables from audio inputs and set filter knob to centre position. Proceed (y/n)?\r\n");
		state = serialState::calibConfirm;

	} else if (ComCmd.compare("save\n") == 0) {		// Save calibration information
		suspendI2S();
		config.SaveConfig();
		resumeI2S();

	} else if (ComCmd.compare("resume\n") == 0) {	// Resume I2S after debugging
		resumeI2S();

	} else if (ComCmd.compare("mem16\n") == 0 || ComCmd.compare("mem32\n") == 0) {		// Memory test
		extern bool runMemTest;
		if (!runMemTest) {
			suspendI2S();
			usb->SendString("Entering memory test mode - clears, writes and reads external RAM. Type 'mem' again to stop\r\n");
			CmdPending = false;
			MemoryTest(ComCmd.compare("mem16\n") == 0);
		} else {
			usb->SendString("Memory test complete\r\n");
			delay.Init();
			resumeI2S();
			runMemTest = false;
		}

	} else if (ComCmd.compare("f\n") == 0) {		// Activate filter

		filter.activateFilter = !filter.activateFilter;
		if (filter.activateFilter) {
			usb->SendString(std::string("Filter on\r\n").c_str());
		} else {
			usb->SendString(std::string("Filter off\r\n").c_str());
		}

	} else if (ComCmd.compare("led\n") == 0) {		// LEDs on/off
		activateLEDs = !activateLEDs;
		if (activateLEDs) {
			usb->SendString("LEDs on\r\n");
		} else {
			usb->SendString("LEDs off\r\n");
		}


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

		int32_t dumpCount = 2000;
		channel LR = ComCmd.compare("dl\n") == 0 ? left : right;
		usb->SendString("Samples: " + std::to_string(dumpCount) + "; Read Pos: " + std::to_string(delay.readPos[LR]) + "; Write Pos: " + std::to_string(delay.writePos) + "\r\n");

		int32_t wp;

		if (delay.writePos < dumpCount)
			wp = delay.writePos + SAMPLE_BUFFER_LENGTH - dumpCount;
		else
			wp = delay.writePos - dumpCount;

		for (int s = 0; s < dumpCount; ++s) {
			StereoSample samp = {samples[wp]};
			usb->SendString(std::to_string(wp) + ": " + std::to_string(samp.sample[LR]) + "\r\n");
			if (++wp == SAMPLE_BUFFER_LENGTH)
				wp = 0;
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
		ComCmd = std::string(reinterpret_cast<char*>(data), length);
		newCmd = false;
	} else {
		ComCmd.append(reinterpret_cast<char*>(data), length);
	}
	if (*ComCmd.rbegin() == '\r')
		*ComCmd.rbegin() = '\n';

	if (*ComCmd.rbegin() == '\n') {
		CmdPending = true;
		newCmd = true;
	}

}

