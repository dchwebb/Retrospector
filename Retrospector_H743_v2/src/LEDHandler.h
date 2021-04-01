#pragma once
#include "initialisation.h"

// LED Handler class used with Toshiba 9 channel LED IC TB62781FNG. All 9 values transmitted as SPI using DMA

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


class LEDHandler {
public:
	//uint8_t spacer0 = 0;			// See Errata in data sheet - does not work correctly without spacer
	uint8_t spacer = 0;			// See Errata in data sheet - does not work correctly without spacer
	uint8_t start = 0xFF;
	uint8_t slaveAddress = 3;
	ledSelection led = ledSeq;
	uint8_t brightness[9];
	uint8_t stop = 0x81;
	uint8_t stopSpacer = 0xFF;

	void Init();
	void LEDSet(ledSelection l, uint8_t b);
	void LEDColour(uint8_t g, uint32_t b);
	void LEDSend();
	void TestPattern();

};



