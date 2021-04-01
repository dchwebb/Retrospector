#pragma once

#include "DigitalDelay.h"
#include "initialisation.h"
#include "USB.h"
#include "Config.h"
#include "Bootloader.h"


class SerialHandler {
public:
	SerialHandler(USB& usb);
	bool Command();
	void Handler(uint8_t* data, uint32_t length);
	void suspendI2S();
	void resumeI2S();

private:
	bool CmdPending = false;
	std::string ComCmd;
	bool dfuConfirm = false;		// Used to allow confirmation before entering USB DFU mode
	USB* usb;

};

