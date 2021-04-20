#pragma once

#include "DigitalDelay.h"
#include "initialisation.h"
#include "USB.h"
#include "Config.h"
#include "Bootloader.h"
#include "sdram.h"



class SerialHandler {
public:
	SerialHandler(USB& usb);
	bool Command();
	void Handler(uint8_t* data, uint32_t length);
	void suspendI2S();
	void resumeI2S();

private:
	// State machine for multi-stage commands
	enum class serialState {pending, dfuConfirm, calibConfirm, cancelAudioTest, configureGate};
	serialState state = serialState::pending;

	bool CmdPending = false;
	std::string ComCmd;
	USB* usb;
};

extern SerialHandler serial;
