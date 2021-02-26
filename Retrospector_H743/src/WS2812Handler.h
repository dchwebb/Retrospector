#pragma once
#include "initialisation.h"

enum ledName { R0, G0, B0, R1, G1, B1, R2, G2, B2 };
class WS2812Handler {
public:
	uint8_t transmit[29];		// 3 RGB values are padded to three bits wide with a spacer bit at front
	uint8_t colours[9];
	uint8_t oldColours[9];

	void Init();
	void LEDSet(ledName l, uint8_t b);
	void LEDColour(uint8_t g, uint32_t b);
	void LEDSend();
};



