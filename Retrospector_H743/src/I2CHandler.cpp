#include "I2CHandler.h"

/* SPI Pins:
 * Preferred:
 * PG14 (129) SPI6_MOSI
 * PG13 (128) SPI6_SCK
 *
 * PF7 (19) SPI5_SCK
 * PF9 (21) SPI5_MOSI
 *
 * PE2  SPI4_SCK (Currently mode pin)
 * PE5  SPI4_ MOSI
 *
 */

uint8_t I2C::SetAddress(uint8_t addr) {

	// In 7-bit addressing mode (ADD10 = 0):SADD[7:1] is 7-bit slave address. SADD[9],	SADD[8] and SADD[0] are don't care.
	I2C1->CR2 |= I2C_CR2_STOP;
	I2C1->CR2 = addr << (I2C_CR2_SADD_Pos + 1);			// Set slave address
	I2C1->CR2 &= ~I2C_CR2_RD_WRN;						// Set direction to write
	I2C1->CR2 |= 0x1 << I2C_CR2_NBYTES_Pos;				// Number of bytes to send
	I2C1->CR2 |= I2C_CR2_START;

	while ((I2C1->ISR & I2C_ICR_STOPCF) == 0);

	// Check I2S_ISR_NACKF
	if ((I2C1->ISR & I2C_ISR_NACKF) > 0) {
		I2C1->ICR |= I2C_ICR_NACKCF;					// Clear NACK
		I2C1->ICR |= I2C_ICR_STOPCF;					// Clear stop bit
	} else {
		return addr;
	}

	return 0;
}

uint8_t I2C::FindAddress() {

	// Test I2C Send
	for (volatile uint8_t addr = 0x0; addr < 255; ++addr) {

		// In 7-bit addressing mode (ADD10 = 0):SADD[7:1] is 7-bit slave address. SADD[9],	SADD[8] and SADD[0] are don't care.
		I2C1->CR2 |= I2C_CR2_STOP;
		I2C1->CR2 = addr << (I2C_CR2_SADD_Pos + 1);			// Set slave address
		I2C1->CR2 &= ~I2C_CR2_RD_WRN;						// Set direction to write
		I2C1->CR2 |= 0x1 << I2C_CR2_NBYTES_Pos;				// Number of bytes to send
		I2C1->CR2 |= I2C_CR2_START;

		while ((I2C1->ISR & I2C_ICR_STOPCF) == 0);

		// Check I2S_ISR_NACKF
		if ((I2C1->ISR & I2C_ISR_NACKF) > 0) {
			I2C1->ICR |= I2C_ICR_NACKCF;					// Clear NACK
			I2C1->ICR |= I2C_ICR_STOPCF;					// Clear stop bit
		} else {
			return addr;
		}
	}
	return 0;
}


void I2C::LEDTest() {

	// Test I2C Send
	// In 7-bit addressing mode (ADD10 = 0):SADD[7:1] is 7-bit slave address. SADD[9],	SADD[8] and SADD[0] are don't care.

	// First packet must be all ones - send a dummy read to address 255
	I2C1->CR2 |= I2C_CR2_STOP;
	I2C1->CR2 = 255 << (I2C_CR2_SADD_Pos + 1);			// Set slave address
	//I2C1->CR2 &= ~I2C_CR2_RD_WRN;						// Set direction to write
	I2C1->CR2 |= I2C_CR2_RD_WRN;						// Set direction to read
	I2C1->CR2 |= 0x1 << I2C_CR2_NBYTES_Pos;				// Number of bytes to send
	I2C1->CR2 |= I2C_CR2_START;

	while ((I2C1->ISR & I2C_ICR_STOPCF) == 0);

	// Check I2S_ISR_NACKF
	if ((I2C1->ISR & I2C_ISR_NACKF) > 0) {
		I2C1->ICR |= I2C_ICR_NACKCF;					// Clear NACK
		I2C1->ICR |= I2C_ICR_STOPCF;					// Clear stop bit
	}


	// Second packet is slave address - set to 3
	I2C1->CR2 |= I2C_CR2_STOP;
	I2C1->CR2 = 3 << (I2C_CR2_SADD_Pos + 1);			// Set slave address
	I2C1->CR2 &= ~I2C_CR2_RD_WRN;						// Set direction to write
	//I2C1->CR2 |= I2C_CR2_RD_WRN;						// Set direction to read
	I2C1->CR2 |= 0x1 << I2C_CR2_NBYTES_Pos;				// Number of bytes to send
	I2C1->CR2 |= I2C_CR2_START;

	while ((I2C1->ISR & I2C_ICR_STOPCF) == 0);

	// Check I2S_ISR_NACKF
	if ((I2C1->ISR & I2C_ISR_NACKF) > 0) {
		I2C1->ICR |= I2C_ICR_NACKCF;					// Clear NACK
		I2C1->ICR |= I2C_ICR_STOPCF;					// Clear stop bit
	}

}

void I2C::Write(uint8_t byte) {
	I2C1->CR2 &= ~I2C_CR2_NBYTES_Msk;					// Clear Number of bytes to send
	I2C1->CR2 |= 0x1 << I2C_CR2_NBYTES_Pos;				// Number of bytes to send = 1

	while ((I2C1->ISR & I2C_ISR_TXE) == 0);				// Wait until transmit buffer is empty

	I2C1->TXDR = byte;
	I2C1->CR2 |= I2C_CR2_START;
}


void I2C::DMATransfer(uint8_t* byte, uint8_t count) {
	while (DMA1_Stream0->NDTR > 0 || (I2C1->ISR & I2C_ISR_BUSY) == I2C_ISR_BUSY);

	I2C1->CR2 |= I2C_CR2_AUTOEND;
	I2C1->CR2 &= ~I2C_CR2_NBYTES_Msk;					// Clear Number of bytes to send
	I2C1->CR2 |= count << I2C_CR2_NBYTES_Pos;			// Number of bytes to send
	I2C1->CR2 |= I2C_CR2_START;

	while ((I2C1->ISR & I2C_ISR_STOPF) == 0);

	if ((I2C1->ISR & I2C_ISR_NACKF) == I2C_ISR_NACKF) {
		I2C1->ICR |= I2C_ICR_NACKCF;					// Clear NACK
		I2C1->CR2 |= I2C_CR2_START;
	}

	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF0;		// Channel 0 Clear synchronization overrun event flag
	DMA1->LIFCR = 0x3F << DMA_LIFCR_CFEIF0_Pos;			// clear all five interrupts for this stream

	DMA1_Stream0->NDTR |= count;						// Number of data items to transfer (ie size of ADC buffer)
	DMA1_Stream0->PAR = (uint32_t)(&(I2C1->TXDR));		// Configure the peripheral data register address 0x40022040
	DMA1_Stream0->M0AR = (uint32_t)(byte);				// Configure the memory address (note that M1AR is used for double-buffer mode) 0x24000040

	DMA1_Stream0->CR |= DMA_SxCR_EN;					// Enable DMA and wait

}

void I2C::Write(uint8_t* byte, uint8_t count) {
	while ((I2C1->ISR & I2C_ISR_BUSY) == I2C_ISR_BUSY);

	I2C1->CR2 |= I2C_CR2_AUTOEND;
	I2C1->CR2 &= ~I2C_CR2_NBYTES_Msk;					// Clear Number of bytes to send
	I2C1->CR2 |= count << I2C_CR2_NBYTES_Pos;			// Number of bytes to send
	I2C1->CR2 |= I2C_CR2_START;

	while ((I2C1->ISR & I2C_ISR_STOPF) == 0);

	if ((I2C1->ISR & I2C_ISR_NACKF) == I2C_ISR_NACKF) {
		I2C1->ICR |= I2C_ICR_NACKCF;					// Clear NACK
		I2C1->CR2 |= I2C_CR2_START;
	}
	I2C1->TXDR = byte[0];

	for (uint8_t i = 1; i < count; ++i) {
		while ((I2C1->ISR & I2C_ISR_TXE) == 0);
		I2C1->TXDR = byte[i];
	}
}



void I2C::OLED_init(uint8_t addr) {
	// keywords:
	// SEG = COL = segment = column byte data on a page
	// Page = 8 pixel tall row. Has 128 SEGs and 8 COMs
	// COM = row
	I2C1->CR2 |= I2C_CR2_STOP;
	I2C1->CR2 = addr << (I2C_CR2_SADD_Pos + 1);			// Set slave address
	I2C1->CR2 &= ~I2C_CR2_RD_WRN;						// Set direction to write

	// Follow instructions on pg.64 of the dataSheet for software configuration of the SSD1306
	uint8_t oledInit[] = {
			OLED_CONTROL_BYTE_CMD_STREAM,
			OLED_CMD_DISPLAY_OFF,
			OLED_CMD_SET_MUX_RATIO, 0x3F, 		// Set mux ration tp select max number of rows - 64
			OLED_CMD_SET_DISPLAY_OFFSET, 0x00,
			OLED_CMD_SET_DISPLAY_START_LINE,	// Display start line to 0
			OLED_CMD_SET_SEGMENT_REMAP,			// Mirror the x-axis. In case you set it up such that the pins are north.// Write(0xA0 - in case pins are south - default
			OLED_CMD_SET_COM_SCAN_MODE,			// Mirror the y-axis. In case you set it up such that the pins are north. Write(0xC0 - in case pins are south - default
			OLED_CMD_SET_COM_PIN_MAP, 0x12,		// Default - alternate COM pin map
			OLED_CMD_SET_CONTRAST, 0x7F,
			OLED_CMD_DISPLAY_RAM,				// Set display to enable rendering from GDDRAM (Graphic Display Data RAM)
			OLED_CMD_DISPLAY_NORMAL,
			OLED_CMD_SET_DISPLAY_CLK_DIV, 0x80,	// Default oscillator clock
			OLED_CMD_SET_CHARGE_PUMP, 0x14,		// Enable the charge pump
			OLED_CMD_SET_PRECHARGE,	0x22,		// Set precharge cycles to high cap type
			OLED_CMD_SET_VCOMH_DESELCT,	0x30,	// Set the V_COMH deselect volatage to max
			OLED_CMD_SET_MEMORY_ADDR_MODE, 0x02,// Page addressing mode (0x00 = Horizonatal addressing mode)
			OLED_CMD_DISPLAY_ON
	};
	//Write(oledInit, sizeof(oledInit));
	DMATransfer(oledInit, sizeof(oledInit));

	// Write pattern
	uint8_t oledPageTop[] = {
			OLED_CONTROL_BYTE_CMD_STREAM,
			OLED_CMD_SET_MEMORY_ADDR_MODE, 0x02,		// PAGE Mode
			0x00,		// set left most column lower nibble (ie start at column 0)
			0x10		// set left most column higher nibble
	};

	uint8_t blank[] = {
			OLED_CONTROL_BYTE_DATA_STREAM,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	};
	DMATransfer(oledPageTop, sizeof(oledPageTop));
	for (uint8_t row = 0; row < 8; row++) { 			//row - top to bottom
		uint8_t oledRow[] = {
				OLED_CONTROL_BYTE_CMD_STREAM,
				static_cast<uint8_t>(0xB0 + row)		// 0x00 - 0x07 row to start on (called 'page' in docs)
		};
		DMATransfer(oledRow, sizeof(oledRow));
		for (uint16_t i = 0; i < 8; i++) {				// column - left to right
			DMATransfer(blank, sizeof(blank));
		}
	}



	// Cross weave pattern (duplicated 8 times to fill a whole row)
	uint8_t pattern2[] = {
			OLED_CONTROL_BYTE_DATA_STREAM,
			0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81,
			0x00, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x00,
			0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81,
			0x00, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x00,
			0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81,
			0x00, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x00,
			0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81,
			0x00, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x00,
			0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81,
			0x00, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x00,
			0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81,
			0x00, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x00,
			0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81,
			0x00, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x00,
			0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81,
			0x00, 0x7E, 0x42, 0x42, 0x42, 0x42, 0x7E, 0x00,
	};

	DMATransfer(oledPageTop, sizeof(oledPageTop));
	for (uint8_t row = 0; row < 8; row++) { 			//row - top to bottom
		uint8_t oledRow[] = {
				OLED_CONTROL_BYTE_CMD_STREAM,
				static_cast<uint8_t>(0xB0 + row)		// 0x00 - 0x07 row to start on (called 'page' in docs)
		};
		DMATransfer(oledRow, sizeof(oledRow));
		DMATransfer(pattern2, sizeof(pattern2));
	}

	// As patterns are on stack need to wait until finished or memory will be corrupted before DMA as finished
	while (DMA1_Stream0->NDTR > 0 || (I2C1->ISR & I2C_ISR_BUSY) == I2C_ISR_BUSY);

}
