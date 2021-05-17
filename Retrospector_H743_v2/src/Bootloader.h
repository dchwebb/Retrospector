#pragma once

#include "initialisation.h"
#include "USB.h"
#include "SerialHandler.h"

struct Bootloader {
public:
	enum class RecordState {setup, waiting, triggered, sampling, finished, install, error} recordState;
	enum class BitState {setup, start, header, checksum, processing} bitState;

	uint32_t bitsCaptured = 0;
	uint32_t bytesCaptured = 0;
	uint32_t sampleCounter = 0;
	int16_t recordSample;
	uint8_t captureByte;
	uint32_t fileSize = 0;
	uint8_t versionMajor;
	uint8_t versionMinor;
	std::string usbResult;

	void Receive();
	void Install();
	void GetSample();
	void ProcessBit(uint8_t bit);
	void LaunchBootloader();
	void BootDFU();

};
