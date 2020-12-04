#include "initialisation.h"
#include "USB.h"
//#include <sstream>

#define CDC_CMD_LEN 20

extern USB usb;

bool CDCCommand(const std::string ComCmd);
void CDCHandler(const uint8_t* data, uint32_t length);
