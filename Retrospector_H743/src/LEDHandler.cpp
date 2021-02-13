#include "LEDHandler.h"

void LEDHandler::LEDSet(ledSelection l, uint8_t b) {
	// Brightness is from 0 to 127, left shifted one
	brightness[(l >> 1) - 1] = b << 1;
}

void LEDHandler::LEDSend() {

	// Clear DMA errors and transfer complete status flags
	DMA1->HIFCR = DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CFEIF5;
	SPI5->IFCR |= SPI_IFCR_TXTFC;

	// Check SPI is not sending
	while ((SPI5->SR & SPI_SR_TXP) == 0 && (SPI5->SR & SPI_SR_TXC) == 0) {};

	SPI5->CR1 &= ~SPI_CR1_SPE;						// Disable SPI
	DMA1_Stream5->NDTR = 14;						// Number of data items to transfer (ie size of LED sequence control)
	DMA1_Stream5->CR |= DMA_SxCR_EN;				// Enable DMA and wait
	SPI5->CR1 |= SPI_CR1_SPE;						// Enable SPI
	SPI5->CR1 |= SPI_CR1_CSTART;					// Start SPI
}

void LEDHandler::Init() {
	// Need to initialise values here as placing object in non-default RAM means initial values not being set on object
	spacer = 0;			// See Errata in data sheet - does not work correctly without spacer
	start = 0xFF;
	slaveAddress = 3;
	led = ledSeq;
	std::fill(std::begin(brightness), std::begin(brightness) + 9, 0);
	stop = 0x81;

	SPI5->CR2 |= 14;								// Set the number of items to transfer
	SPI5->CFG1 |= SPI_CFG1_TXDMAEN;					// Tx DMA stream enable
	SPI5->CR1 |= SPI_CR1_SPE;						// Enable SPI
	DMA1_Stream5->M0AR = (uint32_t)(this);			// Configure the memory data register address

}
