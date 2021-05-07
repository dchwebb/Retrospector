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

private:
	int32_t ParseInt(const std::string cmd, const char precedingChar, int low, int high);
	float ParseFloat(const std::string cmd, const char precedingChar, float low, float high);

	// State machine for multi-stage commands
	enum class serialState {pending, dfuConfirm, calibConfirm, cancelAudioTest};
	serialState state = serialState::pending;

	bool CmdPending = false;
	std::string ComCmd;
	USB* usb;
};

extern SerialHandler serial;
