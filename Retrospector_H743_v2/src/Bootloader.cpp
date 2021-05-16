#include "Bootloader.h"

extern USB usb;
extern SerialHandler serial;

#define BL_SAMPLE_SIZE 100000
//uint8_t __attribute__((section (".sdramSection"))) bootloaderSamples[BL_SAMPLE_SIZE];
// FIXME - changed for memory testing
uint8_t bootloaderSamples[BL_SAMPLE_SIZE];
uint16_t timeAdjust[2000];
uint16_t timeAdjustCount = 0;

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


void Bootloader::Receive()
{
	suspendI2S();
	usb.SendString("Ready to receive audio ...\r\n");
	recordState = RecordState::setup;
	bitState = BitState::setup;
	InitBootloaderTimer();
}

void __attribute__((optimize("O0"))) Bootloader::GetSample()
{
	static bool potentiallyLate;
	static int highestSample = 0;
	static int16_t lastSample = 0;
	recordSample = static_cast<int16_t>(adcZeroOffset[left]) - ADC_array[left];

	// Signal starts with a high low pattern - wait for highest value to get offset
	switch (recordState) {

	case RecordState::setup:
		recordSample = 0;
		recordState = RecordState::waiting;
		highestSample = 0;
		bitsCaptured = 0;
		break;

	case RecordState::waiting:
		if (recordSample > 1000) {				// Once the incoming audio signal crosses threshold start capture
			recordState = RecordState::triggered;
			highestSample = recordSample;
		}
		break;

	case RecordState::triggered:
		if (recordSample > highestSample) {		// each bit will be 8 audio samples; locate the position of the highest signal as the sampling point
			highestSample = recordSample;
		} else {
			sampleCounter = 6;					// sampling point found - begin main sampling
			bitsCaptured = 1;
			recordState = RecordState::sampling;
		}
		break;


	case RecordState::sampling:
		if (sampleCounter == 0) {

			// FIXME timing correction mechanism not yet implemented for early sampling
			// Confirm a potential late sample (transitioned from 1 to 0 and previous capture was a sample late)
			if (recordSample < 0 && potentiallyLate) {
				GPIOB->ODR |= GPIO_ODR_OD7;			// Debug
				sampleCounter = 6;			// We could try altering the sampling speed but currently just reducing the interval of the next capture
				timeAdjust[timeAdjustCount++] = bytesCaptured;
			} else {
				sampleCounter = 7;
			}

			// Check if we are potentially sampling late (ie capturing on a falling edge when transitioning from 1 to 0)
			potentiallyLate = (recordSample > 0 && lastSample > recordSample);

			ProcessBit(recordSample > 0);

		} else {
			lastSample = recordSample;
			GPIOB->ODR &= ~GPIO_ODR_OD7;
			sampleCounter--;
		}
		break;

	case RecordState::finished: {
		recordState = RecordState::install;
		break;
	}

	case RecordState::install: {
		DisableBootloaderTimer();

		volatile uint32_t time = SysTickVal;
		while (SysTickVal  < time + 1000) {};

		Install();
	}


	case RecordState::error: {
		DisableBootloaderTimer();
		resumeI2S();
	}

	default:
		break;
	}
}


void __attribute__((section(".itcm_text"))) Bootloader::Install()
{
	extern Config config;

	// Calculate how many banks we need to erase
	uint8_t eraseBanks = std::ceil(static_cast<float>(fileSize) / 0x1FFFF);
	__disable_irq();

	config.FlashUnlock(1);							// Unlock Bank 1
	FLASH->SR2 = FLASH_ALL_ERRORS;					// Clear error flags in Status Register

	for (uint8_t b = 0; b < eraseBanks; ++b) {
		config.FlashEraseSector(b, 1);				// Erase sector b, Bank 1 (See p152 of manual)
		uint32_t* bankAddr = reinterpret_cast<uint32_t*>(0x08000000 + (0x20000 * b));
		bool result = config.FlashProgram(bankAddr, reinterpret_cast<uint32_t*>(&bootloaderSamples), fileSize);		// FIXME Need to increment bootloaderSamples address
	}

	config.FlashLock(1);							// Lock Bank 1 Flash
	__DSB();
	SCB->AIRCR = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos) | (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) | SCB_AIRCR_SYSRESETREQ_Msk);
	__DSB();

}


void Bootloader::ProcessBit(uint8_t bit)
{
	static uint16_t setupCount;
	static uint8_t lastBit;
	static bool foundCheckSum = false;
	static uint32_t checkSum = 0;

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
			bytesCaptured = 0;
			bitsCaptured = 1;
			checkSum = 0;
			captureByte = bit;
			fileSize = 0;
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
			// Check if byte is checksum (every 500 bytes or at end of file)
			if (((bytesCaptured + 1) % 500 == 0 && bytesCaptured > 0 && !foundCheckSum) || bytesCaptured == fileSize) {
				usbResult = std::to_string(bytesCaptured + 1) + " Bytes received. Checksum: " + std::to_string(captureByte) + ((checkSum & 0xFF) == captureByte ? " OK" : " Error") + "\r\n";
				if ((checkSum & 0xFF) != captureByte) {
					usbResult += "Error receiving file. Cancelling upgrade ...\r\n";
					recordState = RecordState::error;
				} else if (bytesCaptured == fileSize) {
					usbResult += "Installing. Do not switch off ...\r\n";
					recordState = RecordState::finished;
				}
				usb.SendString(usbResult.c_str());

				foundCheckSum = true;
				captureByte = 0;
				checkSum = 0;
			} else {
				foundCheckSum = false;
				checkSum += captureByte;
				bootloaderSamples[bytesCaptured++] = captureByte ^ 0xCC;	// store the captured byte, XORd with check sum (to ensure clock recovery transitions)
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
