#pragma once
#include "initialisation.h"

enum ledSelection {
	ledR0 = 0b00000010,
	ledG0 = 0b00000100,
	ledB0 = 0b00000110,
	ledR1 = 0b00001000,
	ledG1 = 0b00001010,
	ledB1 = 0b00001100,
	ledR2 = 0b00001110,
	ledG2 = 0b00010000,
	ledB2 = 0b00010010,
};

union LEDControl {
	LEDControl(ledSelection l, uint8_t b) : led(l), brightness(b) {};

	uint32_t data[2];
	struct {
		const uint8_t spacer = 0;
		const uint8_t start = 0xFF;
		const uint8_t slaveAddress = 3;
		ledSelection led;
		uint8_t brightness;
		const uint8_t stop = 0x81;
		const uint16_t padding = 0;
	};
};

void LEDPacket(ledSelection l, uint8_t b);
