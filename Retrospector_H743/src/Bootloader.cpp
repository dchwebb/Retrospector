#include "Bootloader.h"

extern USB usb;
extern SerialHandler serial;
extern uint16_t adcZeroOffset[2];

#define BL_SAMPLE_SIZE 20000
uint8_t __attribute__((section (".sdramSection"))) bootloaderSamples[BL_SAMPLE_SIZE];


void Bootloader::Receive()
{
	serial.suspendI2S();
	InitBootloaderTimer();
	state = RecordState::waiting;
	usb.SendString("Ready to receive audio ...\r\n");
}

void __attribute__((optimize("O0"))) Bootloader::GetSample()
{
	static bool potentiallyLate;
	static int highestSample = 0;
	static int16_t lastSample = 0;
	recordSample = static_cast<int16_t>(adcZeroOffset[left]) - ADC_audio[left];

	// Signal starts with a high low pattern - wait for highest value to get offset
	switch (state) {

	case RecordState::waiting:
		if (recordSample > 1000) {				// Once the incoming audio signal crosses threshold start capture
			state = RecordState::triggered;
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
			state = RecordState::sampling;
			captureByte = 1;
		}
		break;


	case RecordState::sampling:
		if (bitsCaptured < BL_SAMPLE_SIZE) {
			if (sampleCounter == 0) {

				// FIXME timing correction mechanism not yet implemented for early sampling
				// Confirm a potential late sample (transitioned from 1 to 0 and previous capture was a sample late)
				if (recordSample < 0 && potentiallyLate) {
					GPIOC->ODR |= GPIO_ODR_OD12;			// Debug
					sampleCounter = 6;			// We could try altering the sampling speed but currently just reducing the interval of the next capture
				} else {
					sampleCounter = 7;
				}

				// Check if we are potentially sampling late (ie capturing on a falling edge when transitioning from 1 to 0)
				potentiallyLate = (recordSample > 0 && lastSample > recordSample);

				if (bitsCaptured % 8 == 0) {
					bootloaderSamples[bytesCaptured++] = captureByte;		// store the previously captured byte
					captureByte = (recordSample > 0 ? 1 : 0);				// Clear the capture byte and start shifting in the next value
				} else {
					captureByte = (captureByte << 1) | (recordSample > 0 ? 1 : 0);		// Shift in the next bit value into the capture byte
				}
				bitsCaptured++;

			} else {
				lastSample = recordSample;
				GPIOC->ODR &= ~GPIO_ODR_OD12;
				sampleCounter--;
			}
		} else {
			state = RecordState::finished;
			usb.SendString("Samples Captured ...\r\n");
		}

	default:
		break;
	}
}


void Bootloader::CopyToFlash() {
	SCB_DisableDCache();
	__disable_irq();
	*((unsigned long *)0x20000000) = 0xABBACAFE; 	// Use DTCM RAM for DFU flag as this is not cleared at restart
	__DSB();
	NVIC_SystemReset();
}
