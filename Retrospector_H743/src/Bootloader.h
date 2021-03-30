#pragma once

#include "initialisation.h"
#include "USB.h"
#include "SerialHandler.h"

class Bootloader {
public:
	enum class RecordState {waiting, triggered, sampling, finished} state;

	uint16_t bitsCaptured = 0;
	uint16_t bytesCaptured = 0;
	uint32_t sampleCounter = 0;
	int16_t recordSample;
	uint8_t captureByte;

	void Receive();
	void GetSample();
	void CopyToFlash();
};
