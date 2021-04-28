#include "LEDHandler.h"

ledStatus ledState = ledOn;

// Set an individual led brightness
void LEDHandler::LEDSet(ledSelection l, uint8_t b)
{
	// Brightness is from 0 to 127, left shifted one
	colour[(l >> 1) - 1] = b << 1;
}

// Set colour of RGB led group
void LEDHandler::LEDColour(ledType l, uint32_t rgb)
{
	// Brightness is from 0 to 255, with lowest bit cleared
	rgb &= ~0x010101;				// Clear lowest bit
	colour[l * 3] = rgb >> 16;
	colour[l * 3 + 1] = (rgb >> 8) & 0xFF;
	colour[l * 3 + 2] = rgb & 0xFF;
}

// Set colour of RGB led group, applying a fractional reduction to each colour separately
void LEDHandler::LEDColour(ledType l, uint32_t rgb, float fract)
{
	// Brightness is from 0 to 255, with lowest bit cleared
	rgb &= ~0x010101;				// Clear lowest bit
	colour[l * 3] = static_cast<uint8_t>(fract * (rgb >> 16));
	colour[l * 3 + 1] = static_cast<uint8_t>(fract * ((rgb >> 8) & 0xFF));
	colour[l * 3 + 2] = static_cast<uint8_t>(fract * (rgb & 0xFF));
}

// Set colour of RGB led group, blending from/to colour and darkening by applying a fractional reduction to each colour separately
void LEDHandler::LEDColour(ledType l, uint32_t rgbFrom, uint32_t rgbTo, float blend, float brightness)
{
	// Interpolate from to colours
	float r = brightness * ((blend * (rgbFrom >> 16)) + ((1.0f - blend) * (rgbTo >> 16)));
	float g = brightness * ((blend * ((rgbFrom >> 8) & 0xFF)) + ((1.0f - blend) * ((rgbTo >> 8) & 0xFF)));
	float b = brightness * ((blend * (rgbFrom & 0xFF)) + ((1.0f - blend) * (rgbTo & 0xFF)));

	// Brightness is from 0 to 255, with lowest bit cleared
	colour[l * 3] = static_cast<uint8_t>(r) & 0xFE;
	colour[l * 3 + 1] = static_cast<uint8_t>(g) & 0xFE;
	colour[l * 3 + 2] = static_cast<uint8_t>(b) & 0xFE;
}

void LEDHandler::LEDColour(ledType l, uint8_t r, uint8_t g, uint8_t b) {
	colour[l * 3] = r & 0xFE;
	colour[l * 3 + 1] = g & 0xFE;
	colour[l * 3 + 2] = b & 0xFE;
}

void LEDHandler::LEDSend()
{
	// LED disabling handling through simple state machine for timing reasons
	if (ledState == ledTurnOff) {
		LEDColour(ledDelL, 0);
		LEDColour(ledDelR, 0);
		LEDColour(ledFilter, 0);
	} else if (ledState == ledOff) {
		return;
	}

	// Clear DMA errors and transfer complete status flags
	SPI6->IFCR |= SPI_IFCR_TXTFC;

	// Check SPI is not sending
	while ((SPI6->SR & SPI_SR_TXP) == 0 && (SPI6->SR & SPI_SR_TXC) == 0) {};

	SPI6->CR1 &= ~SPI_CR1_SPE;						// Disable SPI
	BDMA_Channel0->CCR &= ~BDMA_CCR_EN;				// Disable BDMA
	BDMA_Channel0->CNDTR = sizeof(*this);			// Number of data items to transfer (ie size of LED sequence control)
	BDMA_Channel0->CCR |= BDMA_CCR_EN;				// Enable DMA and wait
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
	ledSel = ledSeq;
	std::fill(std::begin(colour), std::begin(colour) + 9, 0);
	stop = 0x81;
	stopSpacer = 0x0;

	SPI6->CR2 |= sizeof(*this);						// Set the number of items to transfer
	SPI6->CFG1 |= SPI_CFG1_TXDMAEN;					// Tx DMA stream enable
	SPI6->CR1 |= SPI_CR1_SPE;						// Enable SPI
	BDMA_Channel0->CM0AR = reinterpret_cast<uint32_t>(this);		// Configure the memory data register address
}

void LEDHandler::TimedSend()
{
	static uint32_t lastSend = 0;
	if (SysTickVal > lastSend + 1) {
		lastSend = SysTickVal;
		LEDSend();
	}
}


void LEDHandler::TestPattern()
{
	static uint32_t lastLED = 0;
	static uint16_t ledCounter[3];
	static uint32_t ledTarg[3] = {0xFF0000, 0x00FF00, 0x0000FF};
	static uint32_t ledPrev[3];
	static uint8_t ledPos[3] = {0, 2, 4};


	static bool ledCycle = true;
	static const uint16_t transition = 500;
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

			float mult = static_cast<float>(ledCounter[i]) / transition;

			// Interpolate colours between previous and target
			uint8_t oldR = ledPrev[i] >> 16;
			uint8_t newR = ledTarg[i] >> 16;
			uint8_t oldG = (ledPrev[i] >> 8) & 0xFF;
			uint8_t newG = (ledTarg[i] >> 8) & 0xFF;
			uint8_t oldB = ledPrev[i] & 0xFF;
			uint8_t newB = ledTarg[i] & 0xFF;

			newR = (oldR + static_cast<float>(newR - oldR) * mult);
			newG = (oldG + static_cast<float>(newG - oldG) * mult);
			newB = (oldB + static_cast<float>(newB - oldB) * mult);
			uint32_t setColour = (newR << 16) + (newG << 8) + newB;


			if (ledCounter[i] < transition) {
				LEDColour(static_cast<ledType>(i), setColour);
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
