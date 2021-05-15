#include "Bootloader.h"

extern USB usb;
extern SerialHandler serial;

#define BL_SAMPLE_SIZE 20000
//uint8_t __attribute__((section (".sdramSection"))) bootloaderSamples[BL_SAMPLE_SIZE];
// FIXME - changed for memory testing
uint8_t bootloaderSamples[BL_SAMPLE_SIZE];

// Enter DFU bootloader - store a custom word at a known RAM address. The startup file checks for this word and jumps to bootloader in RAM if found
// STM32CubeProgrammer can be used to upload an .elf in this mode (v2.6.0 tested)
void  Bootloader::BootDFU()
{
	SCB_DisableDCache();
	__disable_irq();
	*reinterpret_cast<unsigned long *>(0x20000000) = 0xDEADBEEF; 	// Use DTCM RAM for DFU flag as this is not cleared at restart
	__DSB();
	NVIC_SystemReset();
}

void __attribute__((optimize("O0"))) Bootloader::DebugBootloader()
{
	uint8_t DummyArray[] = {0xAA, 0xAA, 0x00, 0xAA, 0x00, 0x00, 0x06, 0x80, 0x02, 0xDD, 0x00};
	for (uint8_t i = 0; i < 11; ++i) {
		for (uint8_t b = 0; b < 8; ++b) {
			ProcessBit((DummyArray[i] >> (7 - b)) & 0x01);
		}
	}

}

void Bootloader::Receive()
{
	suspendI2S();
	usb.SendString("Ready to receive audio ...\r\n");
//	DebugBootloader();
	InitBootloaderTimer();
	recordState = RecordState::waiting;
	fileSize = 0;
}

void __attribute__((optimize("O0"))) Bootloader::GetSample()
{
	static bool potentiallyLate;
	static bool potentiallyEarly;
	static int highestSample = 0;
	static int16_t lastSample = 0;
	static uint8_t bitStuffCount = 0;
	recordSample = static_cast<int16_t>(adcZeroOffset[left]) - ADC_array[left];

	// Signal starts with a high low pattern - wait for highest value to get offset
	switch (recordState) {

	case RecordState::waiting:
		if (recordSample > 1000) {				// Once the incoming audio signal crosses threshold start capture
			recordState = RecordState::triggered;
			highestSample = recordSample;
			bitsCaptured = 0;
		}
		break;

	case RecordState::triggered:
		if (recordSample > highestSample) {		// each bit will be 8 audio samples; locate the position of the highest signal as the sampling point
			highestSample = recordSample;
		} else {
			sampleCounter = 6;					// sampling point found - begin main sampling
			bitsCaptured = 1;
			recordState = RecordState::sampling;
			//captureByte = 1;
		}
		break;


	case RecordState::sampling:
		if (bitsCaptured < BL_SAMPLE_SIZE) {
			if (sampleCounter == 0) {

				// FIXME timing correction mechanism not yet implemented for early sampling
				// Confirm a potential late sample (transitioned from 1 to 0 and previous capture was a sample late)
				if (recordSample < 0 && potentiallyLate) {
					GPIOB->ODR |= GPIO_ODR_OD7;			// Debug
					sampleCounter = 6;			// We could try altering the sampling speed but currently just reducing the interval of the next capture
				} else if (recordSample > 0 && potentiallyEarly) {
					GPIOB->ODR |= GPIO_ODR_OD7;			// Debug
					sampleCounter = 6;
				} else {
					sampleCounter = 7;
				}

				// Check if we are potentially sampling late (ie capturing on a falling edge when transitioning from 1 to 0)
				potentiallyLate =  (recordSample > 0 && lastSample > recordSample);
				potentiallyEarly = (recordSample < 0 && lastSample < recordSample);	// FIXME this is NOT early - just another test for late

				if (bitState == BitState::processing) {
					if (recordSample < 0) {
						bitStuffCount++;
					} else {
						if (bitStuffCount > 8) {		// After 8 bits of zero a one will be bit-stuffed so ignore
							bitStuffCount = 0;
							return;
						} else {
							bitStuffCount = 0;
						}
					}
				}

				ProcessBit(recordSample > 0);

			} else {
				lastSample = recordSample;
				GPIOB->ODR &= ~GPIO_ODR_OD7;
				sampleCounter--;
			}
		} else {
			recordState = RecordState::finished;
			usb.SendString("Samples Captured ...\r\n");
		}

	default:
		break;
	}
}


void Bootloader::ProcessBit(uint8_t bit)
{
	static uint16_t setupCount;
	static uint8_t lastBit;
	static bool foundCheckSum = false;

	if (bitState == BitState::setup) {		// Setup is a stream of 1010101..
		if (bit != lastBit) {
			setupCount++;
		} else {
			if (setupCount > 6 && bit == 0 && lastBit == 0) {	// check we have at least 8 setup bits and then look for the start of 0x00 byte
				bitState = BitState::start;
			}
			setupCount = 0;
		}
	}

	if (bitState == BitState::start) {		// 0x00 0xAA
		if (setupCount == 16) {
			bitsCaptured = 1;
			captureByte = bit;
			bitState = BitState::header;
			return;
		} else if ((bit == 1 && setupCount < 8) || (setupCount >= 8 && bit == lastBit)) {		// Check for error state
			bitState = BitState::setup;
		}
		setupCount++;
	}


	if (bitsCaptured == 8) {
		if (bitState == BitState::header) {
			if (bytesCaptured < 4) {
				fileSize |= (captureByte << ((3 - bytesCaptured) * 8));
				bytesCaptured++;
			} else {
				usbResult = "Bootloader Size: " + std::to_string(fileSize) + "\r\n";
				usb.SendString(usbResult.c_str());
				bytesCaptured = 0;
				bitState = BitState::processing;
			}
		}

		if (bitState == BitState::processing) {
			// Check if byte is checksum
			if ((bytesCaptured + 1) % 100 == 0 && bytesCaptured > 0 && !foundCheckSum) {
				foundCheckSum = true;
				usbResult = "Checksum: " + std::to_string(captureByte) + "\r\n";
				usb.SendString(usbResult.c_str());
			} else {
				foundCheckSum = false;
				bootloaderSamples[bytesCaptured++] = captureByte;	// store the previously captured byte
			}

			if (bytesCaptured == fileSize) {
				recordState = RecordState::finished;

				usbResult = std::to_string(fileSize) + " Samples captured\r\n";
				usb.SendString(usbResult.c_str());

			}
		}

		bitsCaptured = 0;
		captureByte = bit;									// Clear the capture byte and start shifting in the next value
	} else {
		captureByte = (captureByte << 1) | bit;				// Shift in the next bit value into the capture byte
	}
	bitsCaptured++;

	lastBit = bit;

}


void Bootloader::CopyToFlash() {
	SCB_DisableDCache();
	__disable_irq();
	*reinterpret_cast<unsigned long *>(0x20000000) = 0xABBACAFE; 	// Use DTCM RAM for DFU flag as this is not cleared at restart
	__DSB();
	NVIC_SystemReset();
}
