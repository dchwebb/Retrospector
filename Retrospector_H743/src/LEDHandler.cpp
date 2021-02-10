#include "LEDHandler.h"

void LEDPacket(ledSelection l, uint8_t b) {
	// Test send PWM is send as value from 0 to 127 left shifted by 1
//	SPI5->CR1 |= SPI_CR1_CSTART;
//	SPI5->TXDR = 0x00FF0302;		// 00 is dummy frame FF is start command; 03 is slave address; 02 means sub address is RGB R0
//	SPI5->TXDR = 0x8081;			// 80 is half PWM; 81 is stop packet (period command)

	LEDControl led {l, b };
	while ((SPI5->SR & SPI_SR_TXP) == 0) {};
	SPI5->TXDR = led.data[0];
	SPI5->TXDR = led.data[1];
	SPI5->CR1 |= SPI_CR1_CSTART;
	while ((SPI5->SR & SPI_SR_TXC) == 0) {};

}
