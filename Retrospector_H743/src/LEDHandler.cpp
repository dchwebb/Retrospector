#include "LEDHandler.h"

void LEDPacket() {
	// Test send PWM is send as value from 0 to 127 left shifted by 1
	SPI5->CR1 |= SPI_CR1_CSTART;
	SPI5->TXDR = 0x00FF0302;		// 00 is dummy frame FF is start command; 03 is slave address; 02 means sub address is RGB R0
	SPI5->TXDR = 0x8081;			// 80 is half PWM; 81 is stop packet (period command)


}
