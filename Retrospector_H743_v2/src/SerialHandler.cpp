#include "SerialHandler.h"

#include <stdio.h>

extern DigitalDelay delay;
extern Config config;
extern Bootloader bootloader;

int32_t SerialHandler::ParseInt(const std::string cmd, const char precedingChar, int low = 0, int high = 0) {
	int32_t val = -1;
	int8_t pos = cmd.find(precedingChar);		// locate position of character preceding
	if (pos >= 0 && std::strspn(cmd.substr(pos + 1).c_str(), "0123456789-") > 0) {
		val = stoi(cmd.substr(pos + 1));
	}
	if (high > low && (val > high || val < low)) {
		usb->SendString("Must be a value between " + std::to_string(low) + " and " + std::to_string(high) + "\r\n");
		return low - 1;
	}
	return val;
}

float SerialHandler::ParseFloat(const std::string cmd, const char precedingChar, float low = 0.0, float high = 0.0) {
	float val = -1.0f;
	int8_t pos = cmd.find(precedingChar);		// locate position of character preceding
	if (pos >= 0 && std::strspn(cmd.substr(pos + 1).c_str(), "0123456789.") > 0) {
		val = stof(cmd.substr(pos + 1));
	}
	if (high > low && (val > high || val < low)) {
		usb->SendString("Must be a value between " + std::to_string(low) + " and " + std::to_string(high) + "\r\n");
		return low - 1.0f;
	}
	return val;
}

SerialHandler::SerialHandler(USB& usbObj)
{
	usb = &usbObj;

	// bind the usb's CDC caller to the CDC handler in this class
	usb->cdcDataHandler = std::bind(&SerialHandler::Handler, this, std::placeholders::_1, std::placeholders::_2);
}



// Check if a command has been received from USB, parse and action as required
bool SerialHandler::Command()
{
	char buf[50];

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

		usb->SendString("Mountjoy Retrospector v2.0 - Current Settings:\r\n\r\n" +
				std::string(delay.modulatedDelay ? "Modulated delay: Max: " + std::to_string(static_cast<uint32_t>(delay.modOffsetMax)) + " Inc: " + std::to_string(delay.modOffsetInc) + "\r\n" : "") +
				std::string(delay.stereoWide ? "Stereo wide on\r\n" : ""));

		if (filter.activateFilter) {
			if (filter.filterType == IIR) {
				usb->SendString(std::to_string(filter.iirLPFilter[0].numPoles) + " Pole IIR " + std::string(filter.filterControl == LP ? "Low" : "High") + " Pass Filter: ");
			} else {
				usb->SendString(std::string((filter.passType == LowPass) ? "Low Pass " : "High Pass ") + std::to_string(filter.firTaps) + " Tap FIR Filter: ");
			}

			sprintf(buf, "%0.10f", filter.currentCutoff);		// 10dp
			usb->SendString("Cutoff: " + std::string(buf).append("\r\n"));
		} else {
			usb->SendString("Filter: Off\r\n");
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
				"info        -  Show diagnostic information\r\n"
				"led         -  LEDs on/off\r\n"
				"resume      -  Resume I2S after debugging\r\n"
				"dfu         -  USB firmware upgrade\r\n"
				"boot        -  Audio Bootloader\r\n"
				"calib       -  Calibrate device\r\n"
				"save        -  Save calibration\r\n"
				"\r\nDynamics config:\r\n"
				"threshold:x -  Configure gate threshold to x (default 200, 0 to deactivate)\r\n"
				"gateact:x   -  Configure gate activate time to x samples (default 30000)\r\n"
				"gateled     -  Show gate status on filter LED\r\n"
				"tanh        -  Toggle between fast tanh and linear compression\r\n"
				"\r\nFilter config:\r\n"
				"f           -  Filter on/off\r\n"
				"firtaps:x   -  x FIR taps for LP > HP filter (multiple of 4, >= 4, <= 92)\r\n"
				"iirdefault  -  Reset coefficients of IIR filter to default Butterworth\r\n"
				"dampx:y     -  Set damping value of xth stage of IIR filter to y (> 0.2, < 2.0)\r\n"
				"poles:x     -  Configure number of IIR poles (>= 1, <= 8)\r\n"
				"\r\nModulated Delay Settings:\r\n"
				"mdlength:x  -  Configure sweep length in samples (default 180)\r\n"
				"mdinc:x     -  Configure sweep increment (default 0.00375)\r\n"
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
				"\r\n"
#if (USB_DEBUG)
				"usbdebug    -  Start USB debugging\r\n"
				"\r\n"
#endif
		);

#if (USB_DEBUG)
	} else if (ComCmd.compare("usbdebug\n") == 0) {				// Configure gate LED
		USBDebug = true;
		usb->SendString("Press link button to dump output\r\n");
#endif

	} else if (ComCmd.compare("gateled\n") == 0) {				// Configure gate LED
		delay.gateLED = !delay.gateLED;
		usb->SendString("Toggle Filter LED displaying gate status\r\n");
		if (!delay.gateLED) {
			filter.Update(true);
		}

	} else if (ComCmd.compare(0, 9, "mdlength:") == 0) {		// Modulated Delay length
		uint16_t val = ParseInt(ComCmd, ':', 1, 65535);
		if (val > 0) {
			delay.modOffsetMax = val;
			delay.modOffset[left] = delay.modOffsetMax / 2;
			delay.modOffset[right] = delay.modOffsetMax / 2;
			config.SaveConfig();
		}
		usb->SendString("Modulated delay length set to: " + std::to_string(delay.modOffsetMax) + "\r\n");

	} else if (ComCmd.compare(0, 6, "mdinc:") == 0) {			// Modulated Delay increment
		float val = ParseFloat(ComCmd, ':', 0.000001, 2.0);
		if (val > 0.0) {
			delay.modOffsetInc = val;
			delay.modOffsetAdd[left] = delay.modOffsetInc;
			delay.modOffsetAdd[right] = -1 * delay.modOffsetInc;
			config.SaveConfig();
		}
		usb->SendString("Modulated delay increment set to: " + std::to_string(delay.modOffsetInc) + "\r\n");


	} else if (ComCmd.compare(0, 10, "threshold:") == 0) {		// Configure gate threshold
		uint16_t threshold = ParseInt(ComCmd, ':');
		delay.gateThreshold = threshold;
		usb->SendString("Gate threshold set to: " + std::to_string(delay.gateThreshold) + "\r\n");
		config.SaveConfig();

	} else if (ComCmd.compare(0, 8, "gateact:") == 0) {			// Configure gate activation time
		uint16_t gate = ParseInt(ComCmd, ':');
		delay.gateHoldCount = gate;
		usb->SendString("Gate activate time set to: " + std::to_string(delay.gateHoldCount) + "\r\n");
		config.SaveConfig();

	} else if (ComCmd.compare(0, 8, "firtaps:") == 0) {			// Configure fir taps
		uint16_t taps = ParseInt(ComCmd, ':', 4, 92);
		if (taps > 0) {
			taps = (taps / 4) * 4;								// taps must be a multiple of four
			filter.firTaps = taps;
			filter.Init();										// forces recalculation of coefficients and window
			usb->SendString("FIR taps set to: " + std::to_string(filter.firTaps) + "\r\n");
			config.SaveConfig();
		}

	} else if (ComCmd.compare("iirdefault\n") == 0) {			// Reset IIR filter to default Butterworth coefficients
		filter.DefaultIIR();
		usb->SendString("Filter coefficients set to Butterworth defaults\r\n");
		filter.Update(true);		// forces recalculation of coefficients

	} else if (ComCmd.compare(0, 6, "poles:") == 0) {			// Configure number of iir poles
		uint8_t poles = ParseInt(ComCmd, ':');
		filter.CustomiseIIR(poles);
		usb->SendString("Poles set to: " + std::to_string(poles) + "\r\n");
		config.SaveConfig();

	} else if (ComCmd.compare(0, 4, "damp") == 0) {				// Configure iir zeta damping factor (format dampx:y where x is section and y is amount)
		int section = ParseInt(ComCmd, 'p', 1, 4);
		if (section > 0) {
			float zeta = ParseFloat(ComCmd, ':', 0.000001, 2.0);
			if (zeta > 0.0) {
				filter.CustomiseIIR(section - 1, zeta);
				filter.Update(true);		// forces recalculation of coefficients
				usb->SendString("Damping (zeta) set to: " + std::to_string(zeta) + "\r\n");
				config.SaveConfig();
			}
		}

	} else if (ComCmd.compare("loop\n") == 0) {					// Audio loopback test
		usb->SendString("Starting audio loopback test. Press any key to cancel.\r\n");
		delay.testMode = delay.TestMode::loop;
		state = serialState::cancelAudioTest;

	} else if (ComCmd.compare("saw\n") == 0) {					// Audio loopback test
		usb->SendString("Generating saw wave. Press any key to cancel\r\n");
		delay.testMode = delay.TestMode::saw;
		state = serialState::cancelAudioTest;

	} else if (ComCmd.compare("dfu\n") == 0) {					// USB DFU firmware upgrade
		usb->SendString("Start DFU upgrade mode? Press 'y' to confirm.\r\n");
		state = serialState::dfuConfirm;

	} else if (ComCmd.compare("boot\n") == 0) {					// Test bootloader code
		bootloader.Receive();
		//void Bootloader();
		//Bootloader();

	} else if (ComCmd.compare("calib\n") == 0) {				// Calibrate filter pot center and audio offsets
		usb->SendString("Remove cables from audio inputs and set filter knob to centre position. Proceed (y/n)?\r\n");
		state = serialState::calibConfirm;

	} else if (ComCmd.compare("save\n") == 0) {					// Save calibration information
		config.SaveConfig();

	} else if (ComCmd.compare("resume\n") == 0) {				// Resume I2S after debugging
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

	} else if (ComCmd.compare("f\n") == 0) {					// Activate filter
		filter.activateFilter = !filter.activateFilter;
		usb->SendString("Filter " + std::string(filter.activateFilter ? "on" : "off") + "\r\n");

	} else if (ComCmd.compare("tanh\n") == 0) {					// tanh compression
		delay.tanhCompression = !delay.tanhCompression;
		usb->SendString("Tanh Compression " + std::string(delay.tanhCompression ? "on" : "off") + "\r\n");

	} else if (ComCmd.compare("led\n") == 0) {					// LEDs on/off
		if (ledState == ledOn) {
			ledState = ledTurnOff;
		} else {
			ledState = ledOn;
			filter.Update(true);								// Force the Filter LED to refresh
		}
		usb->SendString("LEDs " + std::string(ledState == ledOn ? "on" : "off") + "\r\n");


	} else if (ComCmd.compare("iir\n") == 0) {					// Show IIR Coefficients

		IIRFilter& activeFilter = (filter.passType == LowPass) ? filter.iirLPFilter[filter.activeFilter] : filter.iirHPFilter[filter.activeFilter];
		usb->SendString(std::to_string(activeFilter.numPoles) + " Pole " + std::string((filter.passType == LowPass) ? "Low Pass\r\n" : "High Pass\r\n"));

		// Output coefficients
		for (int i = 0; i < activeFilter.numSections; ++i) {
			usb->SendString("Stage " + std::to_string(i+1) + ": Cutoff: " + std::to_string(activeFilter.cutoffFreq)
				+ "; Damping: " + std::to_string(activeFilter.iirProto.Coeff.D1[i] / (activeFilter.iirProto.Coeff.D2[i] == 0.0 ? 1.0 : 2.0)) + "\r\n");
			usb->SendString("       Y(z)   " + std::to_string(activeFilter.iirCoeff.b2[i]) + " z^-2 + " + std::to_string(activeFilter.iirCoeff.b1[i]) + " z^-1 + " + std::to_string(activeFilter.iirCoeff.b0[i]) + "\r\n");
			usb->SendString("H(z) = ---- = -----------------------------------------\r\n");
			usb->SendString("       X(z)   " + std::to_string(activeFilter.iirCoeff.a2[i]) + " z^-2 + " + std::to_string(activeFilter.iirCoeff.a1[i]) + " z^-1 + " + std::to_string(activeFilter.iirCoeff.a0[i]) + "\r\n\r\n");
		}


	} else if (ComCmd.compare("imp\n") == 0) {					// IIR Filter Print impulse response
		suspendI2S();

		IIRRegisters iirImpReg;									// Create a temporary set of shift registers for the filter
		IIRFilter& currentFilter = (filter.passType == LowPass) ? filter.iirLPFilter[0] : filter.iirHPFilter[0];

		float out = currentFilter.FilterSample(500, iirImpReg);	// Impulse
		usb->SendString(std::to_string(out) + "\r\n");

		for (int i = 1; i < 500; ++i) {
			out = currentFilter.FilterSample(0, iirImpReg);
			usb->SendString(std::to_string(out) + "\r\n");
		}

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
			usb->SendString(std::to_string(samp.sample[LR]) + "\r\n");
			if (++wp == SAMPLE_BUFFER_LENGTH)
				wp = 0;
		}

		resumeI2S();

	} else if (ComCmd.compare("fir\n") == 0) {					// Dump FIR filter coefficients
		suspendI2S();

		// NB to_string not working. Use sprintf with following: The float formatting support is not enabled, check your MCU Settings from "Project Properties > C/C++ Build > Settings > Tool Settings",
		// or add manually "-u _printf_float" in linker flags
		for (int f = 0; f < filter.firTaps; ++f) {
			if (f > filter.firTaps / 2) {						// Using a folded FIR structure so second half of coefficients is a reflection of the first
				sprintf(buf, "%0.10f", filter.firCoeff[filter.activeFilter][filter.firTaps - f]);		// 10dp
			} else {
				sprintf(buf, "%0.10f", filter.firCoeff[filter.activeFilter][f]);
			}
			usb->SendString(std::string(buf) + "\r\n");
		}
		resumeI2S();

	} else if (ComCmd.compare("wd\n") == 0) {					// Dump filter window
		suspendI2S();
		for (int f = 0; f < filter.firTaps; ++f) {
			sprintf(buf, "%0.10f", filter.winCoeff[f]);			// 10dp
			usb->SendString(std::string(buf) + "\r\n");
		}
		resumeI2S();

	} else if (ComCmd.compare("fdl\n") == 0) {					// Dump left filter buffer
		suspendI2S();
		uint16_t pos = filter.filterBuffPos[0];
		for (int f = 0; f < filter.firTaps; ++f) {
			usb->SendString(std::to_string(filter.filterBuffer[0][pos]) + "\r\n");
			if (++pos == filter.firTaps)
				pos = 0;
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

