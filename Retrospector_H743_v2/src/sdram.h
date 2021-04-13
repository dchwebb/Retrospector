#pragma once

#include "USB.h"
#include "SerialHandler.h"

enum sdramModes: uint16_t {BurstLength1 = 0x0000, BurstLength2 = 0x0001, BurstLength4 = 0x0002, BurstLength8 = 0x0004,
	BurstTypeSequential = 0x0000, BurstTypeInterleaved = 0x0008,
	CASLatency2 = 0x0020, CASLatency3 = 0x0030,
	OperatingModeStandard = 0x0000,
	WriteburstModeProgrammed = 0x0000, WriteburstModeSingle = 0x0200
};

void InitSDRAM_16160(void);
void MemoryTest(bool test16MB);
