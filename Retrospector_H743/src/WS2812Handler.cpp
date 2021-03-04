#include "WS2812Handler.h"

// Set an individual led brightness
void WS2812Handler::LEDSet(ledName l, uint8_t b)
{
	colours[l] = b;
	if (colours[l] != oldColours[l]) {
		uint32_t tmp = (colours[l] & 0xF) | ((colours[l] & 0xF0) << 8);
		tmp = (tmp & 0x3003) | ((tmp & 0xC00C) << 4);
		tmp = (tmp & 0x41041) | ((tmp & 0x82082) << 2);
		tmp = (tmp << 1) | 0x924924;		// insert the 1s so the pattern is 1x0 1x0 1x0 1x0 (where x is actual data bit)

		// Move the stuffed bytes into the transmit buffer, reversing byte order
		transmit[l * 3 + 3] = tmp & 0xFF;
		transmit[l * 3 + 2] = (tmp & 0xFF00) >> 8;
		transmit[l * 3 + 1] = (tmp & 0xFF0000) >> 16;
	}
}

// Set colour of RGB led group
void WS2812Handler::LEDColour(uint8_t leds, uint32_t rgb)
{
	/* Use bit manipulation trickery to get bits in correct sequence to send as pseudo NRZ encoding
	 * Technique effectively uses fact that a bit in NRZ format can be treated as 3 SPI bits: bits 'abc' become '1a0 1b0 1c0'
	 * Then manipulate the bits so they can be encoded in a 32 bit word for extraction into the transmit buffer
	*/

	// Bit reverse samples
	uint32_t bitReverse;
	asm("rbit %[result], %[value]" : [result] "=r" (bitReverse) : [value] "r" (rgb));

	uint32_t encoded = ((bitReverse & 0x2A2A2A00) << 2) |
					   ((bitReverse & 0x54545400) >> 2) |
					   ((bitReverse & 0x01010100) << 6) |
					   ((bitReverse & 0x80808000) >> 6);

	// 24 bit data orderL G7 G6 G5 G4 G3 G2 G1 G0 R7 R6 R5 R4 R3 R2 R1 R0 B7 B6 B5 B4 B3 B2 B1 B0

	// Split out colours - FIXME might be better to store original colours in GRB format
	uint8_t green = encoded >> 16;
	uint8_t red   = encoded >> 8;
	uint8_t blue  = encoded >> 24;

	transmit[leds * 9 + 3] = (green & 0x92) | 0x24;
	transmit[leds * 9 + 2] = (green & 0x24) | 0x49;
	transmit[leds * 9 + 1] = (green & 0x49) | 0x92;

	transmit[leds * 9 + 6] = (red   & 0x92) | 0x24;
	transmit[leds * 9 + 5] = (red   & 0x24) | 0x49;
	transmit[leds * 9 + 4] = (red   & 0x49) | 0x92;

	transmit[leds * 9 + 9] = (blue  & 0x92) | 0x24;
	transmit[leds * 9 + 8] = (blue  & 0x24) | 0x49;
	transmit[leds * 9 + 7] = (blue  & 0x49) | 0x92;

}

void WS2812Handler::LEDSend()
{
	// Clear DMA errors and transfer complete status flags
	DMA1->HIFCR = DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTCIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CFEIF5;
	SPI5->IFCR |= SPI_IFCR_TXTFC;

	// Check SPI is not sending
	while ((SPI5->SR & SPI_SR_TXP) == 0 && (SPI5->SR & SPI_SR_TXC) == 0) {};
	SPI5->CR1 &= ~SPI_CR1_SPE;						// Disable SPI
	DMA1_Stream5->NDTR = TRANSMIT_SIZE;				// Number of data items to transfer (ie size of LED sequence control)
	DMA1_Stream5->CR |= DMA_SxCR_EN;				// Enable DMA and wait
	SPI5->CR1 |= SPI_CR1_SPE;						// Enable SPI
	SPI5->CR1 |= SPI_CR1_CSTART;					// Start SPI
}

void WS2812Handler::Init()
{
	// SPI MOSI line can idle high or low so add a load of padding bits to the end of the transmit sequence as workaround
	std::fill(std::begin(transmit), std::begin(transmit) + TRANSMIT_SIZE, 0);

	SPI5->CR2 |= TRANSMIT_SIZE;						// Set the number of items to transfer
	SPI5->CFG1 |= SPI_CFG1_TXDMAEN;					// Tx DMA stream enable
	SPI5->CR1 |= SPI_CR1_SPE;						// Enable SPI
	DMA1_Stream5->M0AR = (uint32_t)(this);			// Configure the memory data register address
}


