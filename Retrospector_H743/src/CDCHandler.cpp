#include "CDCHandler.h"


bool CDCCommand(const std::string ComCmd) {
	std::stringstream ss;

	if (ComCmd.compare("help\n") == 0) {
		usb.SendString("Mountjoy Retrospector - supported commands:\n\n"
				"help      -  Shows this information\n"
		);

	} else {
		return false;
	}

	return true;
}
