#include "LEDHandler.h"

void LEDHandler::LEDSet(ledSelection l, uint8_t b) {
	// Brightness is from 0 to 127, left shifted one
	control.brightness[(l >> 1) - 1] = b << 1;
}

void LEDHandler::LEDSend() {
	// Test send PWM is send as value from 0 to 127 left shifted by 1

	struct ledData {
		uint8_t spacer;			// See Errata in data sheet - does not work correctly without spacer
		uint8_t start;
		uint8_t slaveAddress;
		uint8_t led;
		uint8_t brightness[9];
		uint8_t stop;
	};


	struct ledData led = {.spacer = 0, .start = 0xFF, .slaveAddress = 3, .led = 0b01100000, .stop = 0x81};
	DMA1_Stream5->M0AR = (uint32_t)(&led);		// Configure the memory data register address

	// Clear DAM errors
	DMA1->HIFCR = DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CFEIF5;

	//SPI5->CR1 |= SPI_CR1_CSTART;
	while ((SPI5->SR & SPI_SR_TXP) == 0 && (SPI5->SR & SPI_SR_TXC) == 0) {};

	DMA1_Stream5->NDTR = 14;						// Number of data items to transfer (ie size of LED sequence control)
	// PAR, M0AR
	// MODIFY_REG(((DMA_Stream_TypeDef   *)hdma->Instance)->CR, (DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_HT), (DMA_IT_TC | DMA_IT_TE | DMA_IT_DME));
	// BDMA_CCR_HTIE
	DMA1_Stream5->CR |= DMA_SxCR_EN;					// Enable DMA and wait

	SPI5->CR2 |= 14;
	SPI5->CFG1 |= SPI_CFG1_TXDMAEN;					// Tx DMA stream enable
	SPI5->CR1 |= SPI_CR1_SPE;

	SPI5->CR1 |= SPI_CR1_CSTART;

/*
	for (uint8_t i = 0; i < 14; ++i) {
		while ((SPI5->SR & SPI_SR_TXP) == 0 && (SPI5->SR & SPI_SR_TXC) == 0) {};

		uint8_t* SPI_DR = (uint8_t*)&(SPI5->TXDR);		// cast the data register address as an 8 bit pointer - otherwise data gets packed wtih an extra byte of zeros
		*SPI_DR = control.data[i];

		//SPI5->TXDR = control.data[i];
	}
	while ((SPI5->SR & SPI_SR_TXC) == 0) {};		// Wait for transmission to complete
*/
}
