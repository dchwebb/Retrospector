#include "USB.h"
#include "CDCHandler.h"
#include "config.h"
#include "DigitalDelay.h"
#include "sdram.h"
#include <stdio.h>
#include <charconv>

uint32_t flashBuff[8192];

// Check if a command has been received from USB, parse and action as required
void CDCHandler::ProcessCommand()
{
	if (!cmdPending) {
		return;
	}

	std::string_view cmd {comCmd};


	cmdPending = false;

	// Provide option to switch to USB DFU mode - this allows the MCU to be programmed with STM32CubeProgrammer in DFU mode
	if (state == serialState::dfuConfirm) {
		if (cmd.compare("y") == 0 || cmd.compare("Y") == 0) {
			usb->SendString("Switching to DFU Mode ...\r\n");
			uint32_t old = SysTickVal;
			while (SysTickVal < old + 100) {};		// Give enough time to send the message
			BootDFU();
		} else {
			state = serialState::pending;
			usb->SendString("Upgrade cancelled\r\n");
		}

	} else if (state == serialState::calibConfirm) {
		if (cmd.compare("y") == 0 || cmd.compare("Y") == 0) {
			config.Calibrate();
			resumeI2S();
		} else {
			usb->SendString("Calibration cancelled\r\n");
		}
		state = serialState::pending;

	} else if (state == serialState::cancelAudioTest && cmd.compare("dl\n") != 0 && cmd.compare("dr\n") != 0) {
		delay.testMode = delay.TestMode::none;
		usb->SendString("Audio test cancelled\r\n");
		state = serialState::pending;

	} else if (cmd.compare("info") == 0) {		// Print diagnostic information

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

	} else if (cmd.compare("help") == 0) {

		usb->SendString("Mountjoy Retrospector\r\n"
				"\r\nSupported commands:\r\n"
				"info        -  Show diagnostic information\r\n"
				"led         -  LEDs on/off\r\n"
				"resume      -  Resume I2S after debugging\r\n"
				"dfu         -  USB firmware upgrade\r\n"
				"calib       -  Calibrate device\r\n"
				"save        -  Save calibration\r\n"
				"\r\nDynamics config:\r\n"
				"threshold:x -  Configure gate threshold to x (default 200, 0 to deactivate)\r\n"
				"gateact:x   -  Configure gate activate time to x samples (default 30000)\r\n"
				"gateled     -  Show gate status on filter LED\r\n"
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
	} else if (cmd.compare("usbdebug") == 0) {				// Configure gate LED
		USBDebug = true;
		usb->SendString("Press link button to dump output\r\n");
#endif

	} else if (cmd.compare("gateled") == 0) {				// Configure gate LED
		delay.gateLED = !delay.gateLED;
		usb->SendString("Toggle Filter LED displaying gate status\r\n");
		if (!delay.gateLED) {
			filter.Update(true);
		}

	} else if (cmd.compare(0, 9, "mdlength:") == 0) {		// Modulated Delay length
		uint16_t val = ParseInt(cmd, ':', 1, 65535);
		if (val > 0) {
			delay.modOffsetMax = val;
			delay.modOffset = delay.modOffsetMax / 2;
			config.SaveConfig();
		}
		usb->SendString("Modulated delay length set to: " + std::to_string(delay.modOffsetMax) + "\r\n");

	} else if (cmd.compare(0, 6, "mdinc:") == 0) {			// Modulated Delay increment
		float val = ParseFloat(cmd, ':', 0.000001, 2.0);
		if (val > 0.0) {
			delay.modOffsetInc = val;
			config.SaveConfig();
		}
		usb->SendString("Modulated delay increment set to: " + std::to_string(delay.modOffsetInc) + "\r\n");


	} else if (cmd.compare(0, 10, "threshold:") == 0) {		// Configure gate threshold
		uint16_t threshold = ParseInt(cmd, ':');
		delay.gateThreshold = threshold;
		usb->SendString("Gate threshold set to: " + std::to_string(delay.gateThreshold) + "\r\n");
		config.SaveConfig();

	} else if (cmd.compare(0, 8, "gateact:") == 0) {			// Configure gate activation time
		uint16_t gate = ParseInt(cmd, ':');
		delay.gateHoldCount = gate;
		usb->SendString("Gate activate time set to: " + std::to_string(delay.gateHoldCount) + "\r\n");
		config.SaveConfig();

	} else if (cmd.compare(0, 8, "firtaps:") == 0) {			// Configure fir taps
		uint16_t taps = ParseInt(cmd, ':', 4, 92);
		if (taps > 0) {
			taps = (taps / 4) * 4;								// taps must be a multiple of four
			filter.firTaps = taps;
			filter.Init();										// forces recalculation of coefficients and window
			usb->SendString("FIR taps set to: " + std::to_string(filter.firTaps) + "\r\n");
			config.SaveConfig();
		}

	} else if (cmd.compare("iirdefault") == 0) {			// Reset IIR filter to default Butterworth coefficients
		filter.DefaultIIR();
		usb->SendString("Filter coefficients set to Butterworth defaults\r\n");
		filter.Update(true);		// forces recalculation of coefficients

	} else if (cmd.compare(0, 6, "poles:") == 0) {			// Configure number of iir poles
		uint8_t poles = ParseInt(cmd, ':');
		filter.CustomiseIIR(poles);
		usb->SendString("Poles set to: " + std::to_string(poles) + "\r\n");
		config.SaveConfig();

	} else if (cmd.compare(0, 4, "damp") == 0) {				// Configure iir zeta damping factor (format dampx:y where x is section and y is amount)
		int section = ParseInt(cmd, 'p', 1, 4);
		if (section > 0) {
			float zeta = ParseFloat(cmd, ':', 0.000001, 2.0);
			if (zeta > 0.0) {
				filter.CustomiseIIR(section - 1, zeta);
				filter.Update(true);		// forces recalculation of coefficients
				usb->SendString("Damping (zeta) set to: " + std::to_string(zeta) + "\r\n");
				config.SaveConfig();
			}
		}

	} else if (cmd.compare("loop") == 0) {					// Audio loopback test
		usb->SendString("Starting audio loopback test. Press any key to cancel.\r\n");
		delay.testMode = delay.TestMode::loop;
		state = serialState::cancelAudioTest;

	} else if (cmd.compare("saw") == 0) {					// Audio loopback test
		usb->SendString("Generating saw wave. Press any key to cancel\r\n");
		delay.testMode = delay.TestMode::saw;
		state = serialState::cancelAudioTest;

	} else if (cmd.compare("dfu") == 0) {					// USB DFU firmware upgrade
		usb->SendString("Start DFU upgrade mode? Press 'y' to confirm.\r\n");
		state = serialState::dfuConfirm;

	} else if (cmd.compare("calib") == 0) {				// Calibrate filter pot center and audio offsets
		usb->SendString("Remove cables from audio inputs and set filter knob to centre position. Proceed (y/n)?\r\n");
		state = serialState::calibConfirm;

	} else if (cmd.compare("save") == 0) {					// Save calibration information
		config.SaveConfig();

	} else if (cmd.compare("resume") == 0) {				// Resume I2S after debugging
		resumeI2S();

	} else if (cmd.compare("mem16") == 0 || cmd.compare("mem32") == 0) {		// Memory test
		extern bool runMemTest;
		if (!runMemTest) {
			suspendI2S();
			usb->SendString("Entering memory test mode - clears, writes and reads external RAM. Type 'mem' again to stop\r\n");
			cmdPending = false;
			MemoryTest(cmd.compare("mem16") == 0);
		} else {
			usb->SendString("Memory test complete\r\n");
			delay.Init();
			resumeI2S();
			runMemTest = false;
		}

	} else if (cmd.compare("f") == 0) {					// Activate filter
		filter.activateFilter = !filter.activateFilter;
		usb->SendString("Filter " + std::string(filter.activateFilter ? "on" : "off") + "\r\n");

	} else if (cmd.compare("led") == 0) {					// LEDs on/off
		if (ledState == ledOn) {
			ledState = ledTurnOff;
		} else {
			ledState = ledOn;
			filter.Update(true);								// Force the Filter LED to refresh
		}
		usb->SendString("LEDs " + std::string(ledState == ledOn ? "on" : "off") + "\r\n");


	} else if (cmd.compare("iir") == 0) {					// Show IIR Coefficients

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


	} else if (cmd.compare("imp") == 0) {					// IIR Filter Print impulse response
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


	} else if (cmd.compare("dl") == 0 || cmd.compare("dr") == 0) {		// Dump sample buffer for L or R output
		suspendI2S();

		int32_t dumpCount = 2000;
		channel LR = cmd.compare("dl") == 0 ? left : right;
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

	} else if (cmd.compare("fir") == 0) {					// Dump FIR filter coefficients
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

	} else if (cmd.compare("wd") == 0) {					// Dump filter window
		suspendI2S();
		for (int f = 0; f < filter.firTaps; ++f) {
			sprintf(buf, "%0.10f", filter.winCoeff[f]);			// 10dp
			usb->SendString(std::string(buf) + "\r\n");
		}
		resumeI2S();

	} else if (cmd.compare("fdl") == 0) {					// Dump left filter buffer
		suspendI2S();
		uint16_t pos = filter.filterBuffPos[0];
		for (int f = 0; f < filter.firTaps; ++f) {
			usb->SendString(std::to_string(filter.filterBuffer[0][pos]) + "\r\n");
			if (++pos == filter.firTaps)
				pos = 0;
		}
		resumeI2S();

	} else {
		printf("Unrecognised command: %s\r\nType 'help' for supported commands\r\n", cmd.data());
	}

	cmdPending = false;
}


void CDCHandler::DataIn()
{
	if (inBuffSize > 0 && inBuffSize % USB::ep_maxPacket == 0) {
		inBuffSize = 0;
		EndPointTransfer(Direction::in, inEP, 0);				// Fixes issue transmitting an exact multiple of max packet size (n x 64)
	}
}


// As this is called from an interrupt assign the command to a variable so it can be handled in the main loop
void CDCHandler::DataOut()
{
	// Check if sufficient space in command buffer
	const uint32_t newCharCnt = std::min(outBuffCount, maxCmdLen - 1 - buffPos);

	strncpy(&comCmd[buffPos], (char*)outBuff, newCharCnt);
	buffPos += newCharCnt;

	// Check if cr has been sent yet
	if (comCmd[buffPos - 1] == 13 || comCmd[buffPos - 1] == 10 || buffPos == maxCmdLen - 1) {
		comCmd[buffPos - 1] = '\0';
		cmdPending = true;
		buffPos = 0;
	}
}


void CDCHandler::ActivateEP()
{
	EndPointActivate(USB::CDC_In,   Direction::in,  EndPointType::Bulk);			// Activate CDC in endpoint
	EndPointActivate(USB::CDC_Out,  Direction::out, EndPointType::Bulk);			// Activate CDC out endpoint
	EndPointActivate(USB::CDC_Cmd,  Direction::in,  EndPointType::Interrupt);		// Activate Command IN EP

	EndPointTransfer(Direction::out, USB::CDC_Out, USB::ep_maxPacket);
}


void CDCHandler::ClassSetup(usbRequest& req)
{
	if (req.RequestType == DtoH_Class_Interface && req.Request == GetLineCoding) {
		SetupIn(req.Length, (uint8_t*)&lineCoding);
	}

	if (req.RequestType == HtoD_Class_Interface && req.Request == SetLineCoding) {
		// Prepare to receive line coding data in ClassSetupData
		usb->classPendingData = true;
		EndPointTransfer(Direction::out, 0, req.Length);
	}
}


void CDCHandler::ClassSetupData(usbRequest& req, const uint8_t* data)
{
	// ClassSetup passes instruction to set line coding - this is the data portion where the line coding is transferred
	if (req.RequestType == HtoD_Class_Interface && req.Request == SetLineCoding) {
		lineCoding = *(LineCoding*)data;
	}
}

int32_t CDCHandler::ParseInt(const std::string_view cmd, const char precedingChar, const int32_t low, const int32_t high) {
	int32_t val = -1;
	const int8_t pos = cmd.find(precedingChar);		// locate position of character preceding
	if (pos >= 0 && std::strspn(&cmd[pos + 1], "0123456789-") > 0) {
		val = std::stoi(&cmd[pos + 1]);
	}
	if (high > low && (val > high || val < low)) {
		printf("Must be a value between %ld and %ld\r\n", low, high);
		return low - 1;
	}
	return val;
}


float CDCHandler::ParseFloat(const std::string_view cmd, const char precedingChar, const float low = 0.0f, const float high = 0.0f) {
	float val = -1.0f;
	const int8_t pos = cmd.find(precedingChar);		// locate position of character preceding
	if (pos >= 0 && std::strspn(&cmd[pos + 1], "0123456789.") > 0) {
		val = std::stof(&cmd[pos + 1]);
	}
	if (high > low && (val > high || val < low)) {
		printf("Must be a value between %f and %f\r\n", low, high);
		return low - 1.0f;
	}
	return val;
}


// Descriptor definition here as requires constants from USB class
const uint8_t CDCHandler::Descriptor[] = {
	// IAD Descriptor - Interface association descriptor for CDC class
	0x08,									// bLength (8 bytes)
	USB::IadDescriptor,						// bDescriptorType
	USB::CDCCmdInterface,					// bFirstInterface
	0x02,									// bInterfaceCount
	0x02,									// bFunctionClass (Communications and CDC Control)
	0x02,									// bFunctionSubClass
	0x01,									// bFunctionProtocol
	USB::CommunicationClass,				// String Descriptor

	// Interface Descriptor
	0x09,									// bLength: Interface Descriptor size
	USB::InterfaceDescriptor,				// bDescriptorType: Interface
	USB::CDCCmdInterface,					// bInterfaceNumber: Number of Interface
	0x00,									// bAlternateSetting: Alternate setting
	0x01,									// bNumEndpoints: 1 endpoint used
	0x02,									// bInterfaceClass: Communication Interface Class
	0x02,									// bInterfaceSubClass: Abstract Control Model
	0x01,									// bInterfaceProtocol: Common AT commands
	USB::CommunicationClass,				// iInterface

	// Header Functional Descriptor
	0x05,									// bLength: Endpoint Descriptor size
	USB::ClassSpecificInterfaceDescriptor,	// bDescriptorType: CS_INTERFACE
	0x00,									// bDescriptorSubtype: Header Func Desc
	0x10,									// bcdCDC: spec release number
	0x01,

	// Call Management Functional Descriptor
	0x05,									// bFunctionLength
	USB::ClassSpecificInterfaceDescriptor,	// bDescriptorType: CS_INTERFACE
	0x01,									// bDescriptorSubtype: Call Management Func Desc
	0x00,									// bmCapabilities: D0+D1
	0x01,									// bDataInterface: 1

	// ACM Functional Descriptor
	0x04,									// bFunctionLength
	USB::ClassSpecificInterfaceDescriptor,	// bDescriptorType: CS_INTERFACE
	0x02,									// bDescriptorSubtype: Abstract Control Management desc
	0x02,									// bmCapabilities

	// Union Functional Descriptor
	0x05,									// bFunctionLength
	USB::ClassSpecificInterfaceDescriptor,	// bDescriptorType: CS_INTERFACE
	0x06,									// bDescriptorSubtype: Union func desc
	0x00,									// bMasterInterface: Communication class interface
	0x01,									// bSlaveInterface0: Data Class Interface

	// Endpoint 2 Descriptor
	0x07,									// bLength: Endpoint Descriptor size
	USB::EndpointDescriptor,				// bDescriptorType: Endpoint
	USB::CDC_Cmd,							// bEndpointAddress
	USB::Interrupt,							// bmAttributes: Interrupt
	0x08,									// wMaxPacketSize
	0x00,
	0x10,									// bInterval

	//---------------------------------------------------------------------------

	// Data class interface descriptor
	0x09,									// bLength: Endpoint Descriptor size
	USB::InterfaceDescriptor,				// bDescriptorType:
	USB::CDCDataInterface,					// bInterfaceNumber: Number of Interface
	0x00,									// bAlternateSetting: Alternate setting
	0x02,									// bNumEndpoints: Two endpoints used
	0x0A,									// bInterfaceClass: CDC
	0x00,									// bInterfaceSubClass:
	0x00,									// bInterfaceProtocol:
	0x00,									// iInterface:

	// Endpoint OUT Descriptor
	0x07,									// bLength: Endpoint Descriptor size
	USB::EndpointDescriptor,				// bDescriptorType: Endpoint
	USB::CDC_Out,							// bEndpointAddress
	USB::Bulk,								// bmAttributes: Bulk
	LOBYTE(USB::ep_maxPacket),				// wMaxPacketSize:
	HIBYTE(USB::ep_maxPacket),
	0x00,									// bInterval: ignore for Bulk transfer

	// Endpoint IN Descriptor
	0x07,									// bLength: Endpoint Descriptor size
	USB::EndpointDescriptor,				// bDescriptorType: Endpoint
	USB::CDC_In,							// bEndpointAddress
	USB::Bulk,								// bmAttributes: Bulk
	LOBYTE(USB::ep_maxPacket),				// wMaxPacketSize:
	HIBYTE(USB::ep_maxPacket),
	0x00,									// bInterval: ignore for Bulk transfer
};


uint32_t CDCHandler::GetInterfaceDescriptor(const uint8_t** buffer) {
	*buffer = Descriptor;
	return sizeof(Descriptor);
}
