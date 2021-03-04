#include "LEDHandler.h"

// Set an individual led brightness
void LEDHandler::LEDSet(ledSelection l, uint8_t b)
{
	// Brightness is from 0 to 127, left shifted one
	brightness[(l >> 1) - 1] = b << 1;
}

// Set colour of RGB led group
void LEDHandler::LEDColour(uint8_t g, uint32_t rgb)
{
	// Brightness is from 0 to 255, with lowest bit cleared
	rgb &= ~0x010101;				// Clear lowest bit
	brightness[g * 3] = rgb >> 16;
	brightness[g * 3 + 1] = (rgb >> 8) & 0xFF;
	brightness[g * 3 + 2] = rgb & 0xFF;
}

void LEDHandler::LEDSend()
{
	// Clear DMA errors and transfer complete status flags
	SPI6->IFCR |= SPI_IFCR_TXTFC;

	// Check SPI is not sending
	while ((SPI6->SR & SPI_SR_TXP) == 0 && (SPI6->SR & SPI_SR_TXC) == 0) {};

	SPI6->CR1 &= ~SPI_CR1_SPE;						// Disable SPI
	BDMA_Channel5->CCR &= ~BDMA_CCR_EN;				// Disable BDMA
	BDMA_Channel5->CNDTR = 15;						// Number of data items to transfer (ie size of LED sequence control)
	BDMA_Channel5->CCR |= BDMA_CCR_EN;				// Enable DMA and wait
	SPI6->CR1 |= SPI_CR1_SPE;						// Enable SPI
	SPI6->CR1 |= SPI_CR1_CSTART;					// Start SPI
}

void LEDHandler::Init()
{
	// Need to initialise values here as placing object in non-default RAM means initial values not being set on object
	//spacer0 = 0x0;
	spacer = 0x0;			// See Errata in data sheet - does not work correctly without spacer
	start = 0xFF;
	slaveAddress = 3;
	led = ledSeq;
	std::fill(std::begin(brightness), std::begin(brightness) + 9, 0);
	stop = 0x81;
	stopSpacer = 0x0;

	SPI6->CR2 |= 15;								// Set the number of items to transfer
	SPI6->CFG1 |= SPI_CFG1_TXDMAEN;					// Tx DMA stream enable
	SPI6->CR1 |= SPI_CR1_SPE;						// Enable SPI
	BDMA_Channel5->CM0AR = (uint32_t)(this);		// Configure the memory data register address
}


void LEDHandler::TestPattern()
{


	static uint32_t lastLED = 0;
	static uint16_t ledCounter[3];
	static uint32_t ledTarg[3] = {0xFF0000, 0x00FF00, 0x0000FF};
	static uint32_t ledPrev[3];
	static uint8_t ledPos[3] = {0, 2, 4};


	static bool ledCycle = true;
	static const uint16_t transition = 1000;
	static const uint32_t colourCycle[] = {
			0xFF0000,
			0x00FF00,
			0x0000FF,
			0xFF3300,
			0x0000FF,
			0x555555 };

	if (SysTickVal > lastLED + 3) {
		for (uint8_t i = 0; i < 3; ++i) {

			++ledCounter[i];

			float mult = (float)ledCounter[i] / transition;

			// Interpolate colours between previous and target
			uint8_t oldR = ledPrev[i] >> 16;
			uint8_t newR = ledTarg[i] >> 16;
			uint8_t oldG = (ledPrev[i] >> 8) & 0xFF;
			uint8_t newG = (ledTarg[i] >> 8) & 0xFF;
			uint8_t oldB = ledPrev[i] & 0xFF;
			uint8_t newB = ledTarg[i] & 0xFF;

			newR = (oldR + (float)(newR - oldR) * mult);
			newG = (oldG + (float)(newG - oldG) * mult);
			newB = (oldB + (float)(newB - oldB) * mult);
			uint32_t setColour = (newR << 16) + (newG << 8) + newB;


			if (ledCounter[i] < transition) {
				LEDColour(i, setColour);
			} else {
				ledPrev[i] = ledTarg[i];

				if (ledCycle) {
					ledTarg[i] = colourCycle[++ledPos[i]];
					if (ledPos[i] == 5)
						ledPos[i] = 0;
				}
				ledCounter[i] = 0;
			}
		}
		lastLED = SysTickVal;
		LEDSend();
	}
}
