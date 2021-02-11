#include "LEDHandler.h"

void LEDHandler::LEDSet(ledSelection l, uint8_t b) {
	// Brightness is from 0 to 127, left shifted one
	control.brightness[(l >> 1) - 1] = b << 1;
}

void LEDHandler::LEDSend() {
	// Test send PWM is send as value from 0 to 127 left shifted by 1

	SPI5->CR1 |= SPI_CR1_CSTART;

	for (uint8_t i = 0; i < 14; ++i) {
		while ((SPI5->SR & SPI_SR_TXP) == 0 && (SPI5->SR & SPI_SR_TXC) == 0) {};

		uint8_t* SPI_DR = (uint8_t*)&(SPI5->TXDR);		// cast the data register address as an 8 bit pointer - otherwise data gets packed wtih an extra byte of zeros
		*SPI_DR = control.data[i];


		//SPI5->TXDR = control.data[i];
	}
	while ((SPI5->SR & SPI_SR_TXC) == 0) {};		// Wait for transmission to complete

}
