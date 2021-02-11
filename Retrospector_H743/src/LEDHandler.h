#pragma once
#include "initialisation.h"

enum ledSelection {
	ledR0  = 0b00000010,
	ledG0  = 0b00000100,
	ledB0  = 0b00000110,
	ledR1  = 0b00001000,
	ledG1  = 0b00001010,
	ledB1  = 0b00001100,
	ledR2  = 0b00001110,
	ledG2  = 0b00010000,
	ledB2  = 0b00010010,
	ledAll = 0b00100000,
	ledSeq = 0b01100000,
};

union LEDControl {
	uint8_t data[14];
	struct {
		const uint8_t spacer = 0;			// See Errata in data sheet - does not work correctly without spacer
		const uint8_t start = 0xFF;
		const uint8_t slaveAddress = 3;
		ledSelection led = ledSeq;
		uint8_t brightness[9];
		const uint8_t stop = 0x81;
		//const uint16_t padding = 0;
	};
};

class LEDHandler {
public:
	LEDControl control;
	void LEDSet(ledSelection l, uint8_t b);
	void LEDSend();
};

