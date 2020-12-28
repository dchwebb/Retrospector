#include "initialisation.h"
#include "USB.h"
#include "DigitalDelay.h"
#include "filter.h"

extern USB usb;
extern digitalDelay DigitalDelay;
extern int16_t samples[2][SAMPLE_BUFFER_LENGTH];
extern volatile bool CmdPending;
extern std::string ComCmd;

bool CDCCommand(const std::string ComCmd);
void CDCHandler(uint8_t* data, uint32_t length);
