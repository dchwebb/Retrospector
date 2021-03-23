#pragma once

#include "DigitalDelay.h"
#include "initialisation.h"
#include "USB.h"
#include "Config.h"



class SerialHandler {
public:
	SerialHandler(USB& usb);
	bool Command();
	void Handler(uint8_t* data, uint32_t length);
private:
	bool CmdPending = false;
	std::string ComCmd;
	bool dfuConfirm = false;		// Used to allow confirmation before entering USB DFU mode
	USB* usb;

	void suspendI2S();
	void resumeI2S();

};

