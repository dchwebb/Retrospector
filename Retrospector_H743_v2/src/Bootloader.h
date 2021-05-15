#pragma once

#include "initialisation.h"
#include "USB.h"
#include "SerialHandler.h"

struct Bootloader {
public:
	enum class RecordState {waiting, triggered, sampling, finished} recordState;
	enum class BitState {setup, start, header, checksum, processing} bitState;

	uint16_t bitsCaptured = 0;
	uint16_t bytesCaptured = 0;
	uint32_t sampleCounter = 0;
	int16_t recordSample;
	uint8_t captureByte;
	uint32_t fileSize = 0;
	std::string usbResult;

	void Receive();
	void DebugBootloader();

	void GetSample();
	void ProcessBit(uint8_t bit);
	void CopyToFlash();
	void BootDFU();

};
