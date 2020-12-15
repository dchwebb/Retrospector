#include "initialisation.h"
#include "USB.h"
#include "DigitalDelay.h"

extern USB usb;
extern digitalDelay DigitalDelay;
extern int16_t samples[2][SAMPLE_BUFFER_LENGTH];

bool CDCCommand(const std::string ComCmd);
