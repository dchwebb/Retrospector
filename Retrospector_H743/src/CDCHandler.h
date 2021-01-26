#include "DigitalDelay.h"
#include "initialisation.h"
#include "USB.h"
#include "Filter.h"

extern USB usb;
extern DigitalDelay delay;
extern int32_t samples[SAMPLE_BUFFER_LENGTH];
extern volatile bool CmdPending;
extern std::string ComCmd;

bool CDCCommand(const std::string ComCmd);
void CDCHandler(uint8_t* data, uint32_t length);
