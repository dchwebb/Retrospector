#include "CDCHandler.h"



bool CDCCommand(const std::string ComCmd) {

	if (ComCmd.compare("help\n") == 0) {
		usb.SendString("Mountjoy Retrospector - supported commands:\n\n"
				"help      -  Shows this information\n"
				"d1        -  Dump samples for channel 1\n"
				"o         -  Dump debug samples (for loopback testing)\n"
		);
	} else if (ComCmd.compare("d1\n") == 0) {		// Dump sample buffer for L output
		// Suspend I2S
		SPI2->CR1 |= SPI_CR1_CSUSP;
		while ((SPI2->SR & SPI_SR_SUSP) == 0);
		usb.SendString("Read Pos: " + std::to_string(DigitalDelay.readPos[0]) + "; Write Pos: " + std::to_string(DigitalDelay.writePos[0]) + "\n");

		for (int s = 0; s < SAMPLE_BUFFER_LENGTH; ++s) {

			usb.SendString(std::to_string(samples[0][s]).append("\n").c_str());
		}

		// Resume I2S
		extern volatile bool sampleClock;
		sampleClock = true;

		SPI2->IFCR |= SPI_IFCR_SUSPC;
		while ((SPI2->SR & SPI_SR_SUSP) != 0);

		SPI2->CR1 |= SPI_CR1_CSTART;

	} else if (ComCmd.compare("o\n") == 0) {		// Dump sample output buffer

		extern int16_t samplesOut[SAMPLE_BUFFER_LENGTH];
		extern int16_t samplesMeasure[SAMPLE_BUFFER_LENGTH];
		extern int32_t sampleOutCount;

		// Suspend I2S
		SPI2->CR1 |= SPI_CR1_CSUSP;
		while ((SPI2->SR & SPI_SR_SUSP) == 0);
		usb.SendString("Read Pos: " + std::to_string(sampleOutCount) + "\n");

		for (int s = 0; s < SAMPLE_BUFFER_LENGTH; ++s) {

			usb.SendString(std::to_string(samplesMeasure[s]).append("\n").c_str());
		}

		// Resume I2S
		SPI2->IFCR |= SPI_IFCR_SUSPC;
		while ((SPI2->SR & SPI_SR_SUSP) != 0);

		SPI2->CR1 |= SPI_CR1_CSTART;
	} else {
		return false;
	}

	return true;
}
