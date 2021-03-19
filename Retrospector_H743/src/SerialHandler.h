#include "DigitalDelay.h"
#include "initialisation.h"
#include "USB.h"

extern DigitalDelay delay;


class SerialHandler {
public:
	bool CmdPending = false;
	std::string ComCmd;
	bool dfuConfirm = false;		// Used to allow confirmation before entering USB DFU mode
	USB* usb;
	SerialHandler(USB& usb);

	bool Command();
	void Handler(uint8_t* data, uint32_t length);
};
