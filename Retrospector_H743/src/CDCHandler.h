#include "DigitalDelay.h"
#include "initialisation.h"
#include "USB.h"

extern DigitalDelay delay;


class CDCHandler {
public:
	bool CmdPending = false;
	std::string ComCmd;
	USB* usb;
	CDCHandler(USB& usb);

	bool Command();
	void Handler(uint8_t* data, uint32_t length);
};

