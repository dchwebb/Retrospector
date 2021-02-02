#pragma once
#include "initialisation.h"

#define OLED_I2C_ADDRESS   0x3C

// The SSD1306 datasheet (pg.20) says that a control byte has to be sent before sending a command
// Control byte consists of
// bit 7		: Co   : Continuation bit - If 0, then it assumes all the next bytes are data (no more control bytes).
//				:		 You can send a stream of data, ie: gRAM dump - if Co=0
//				:        For Command, you'd prolly wanna set this - one at a time. Hence, Co=1 for commands
//				:		 For Data stream, Co=0 :)
// bit 6    	: D/C# : Data/Command Selection bit, Data=1/Command=0
// bit [5-0] 	: lower 6 bits have to be 0
#define OLED_CONTROL_BYTE_CMD_SINGLE	0x80
#define OLED_CONTROL_BYTE_CMD_STREAM	0x00
#define OLED_CONTROL_BYTE_DATA_STREAM	0x40

// Fundamental commands (pg.28)
#define OLED_CMD_SET_CONTRAST			0x81	// follow with 0x7F
#define OLED_CMD_DISPLAY_RAM			0xA4
#define OLED_CMD_DISPLAY_ALLON			0xA5
#define OLED_CMD_DISPLAY_NORMAL			0xA6
#define OLED_CMD_DISPLAY_INVERTED 		0xA7
#define OLED_CMD_DISPLAY_OFF			0xAE
#define OLED_CMD_DISPLAY_ON				0xAF

// Addressing Command Table (pg.30)
#define OLED_CMD_SET_MEMORY_ADDR_MODE	0x20	// follow with 0x00 = HORZ mode ; 0x01 = VERT ; 0x02 = PAGE
#define OLED_CMD_SET_COLUMN_RANGE		0x21	// can be used only in HORZ/VERT mode - follow with 0x00 + 0x7F = COL127
#define OLED_CMD_SET_PAGE_RANGE			0x22	// can be used only in HORZ/VERT mode - follow with 0x00 + 0x07 = PAGE7

// Hardware Config (pg.31)
#define OLED_CMD_SET_DISPLAY_START_LINE	0x40
#define OLED_CMD_SET_SEGMENT_REMAP		0xA1
#define OLED_CMD_SET_MUX_RATIO			0xA8	// follow with 0x3F = 64 MUX
#define OLED_CMD_SET_COM_SCAN_MODE		0xC8
#define OLED_CMD_SET_DISPLAY_OFFSET		0xD3	// follow with 0x00
#define OLED_CMD_SET_COM_PIN_MAP		0xDA	// follow with 0x12

// Timing and Driving Scheme (pg.32)
#define OLED_CMD_SET_DISPLAY_CLK_DIV	0xD5	// follow with 0x80
#define OLED_CMD_SET_PRECHARGE			0xD9	// follow with 0x22
#define OLED_CMD_SET_VCOMH_DESELCT		0xDB	// follow with 0x30

// Charge Pump (pg.62)
#define OLED_CMD_SET_CHARGE_PUMP		0x8D	// follow with 0x14

// NOP
#define OLED_CMD_NOP 					0xE3

class I2C {
public:
	uint8_t FindAddress();
	uint8_t SetAddress(uint8_t addr);
	void OLED_init(uint8_t addr);
	void Write(uint8_t byte);
	void Write(uint8_t* byte, uint8_t count);
	void DMATransfer(uint8_t* byte, uint8_t count);
};
