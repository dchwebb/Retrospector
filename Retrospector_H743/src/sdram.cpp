#include <stdint.h>
#include "sdram.h"
#include "initialisation.h"

// SDRAM attached to bank 2 (ie use FMC_SDCKE1 and FMC_SDNE1 rather than 0 equivalents)
// Timing parameters computed for a 168Mhz clock (N prescaler = 336 for 168MHz; 360 for 180MHz)
/*-- GPIOs Configuration - use Alternate Function 12
 +-------------------+--------------------+--------------------+--------------------+
 +                       SDRAM pins assignment                                      +
 +-------------------+--------------------+--------------------+--------------------+
 | PD0  <-> FMC_D2   | PE0  <-> FMC_NBL0  | PF0  <-> FMC_A0    | PG0  <-> FMC_A10   |
 | PD1  <-> FMC_D3   | PE1  <-> FMC_NBL1  | PF1  <-> FMC_A1    | PG1  <-> FMC_A11   |
 | PD8  <-> FMC_D13  | PE7  <-> FMC_D4    | PF2  <-> FMC_A2    | PG4  <-> FMC_BA0   |
 | PD9  <-> FMC_D14  | PE8  <-> FMC_D5    | PF3  <-> FMC_A3    | PG5  <-> FMC_BA1   |
 | PD10 <-> FMC_D15  | PE9  <-> FMC_D6    | PF4  <-> FMC_A4    | PG8  <-> FMC_SDCLK |
 | PD14 <-> FMC_D0   | PE10 <-> FMC_D7    | PF5  <-> FMC_A5    | PG15 <-> FMC_NCAS  |
 | PD15 <-> FMC_D1   | PE11 <-> FMC_D8    | PF11 <-> FMC_NRAS  |--------------------+
 +-------------------| PE12 <-> FMC_D9    | PF12 <-> FMC_A6    |
                     | PE13 <-> FMC_D10   | PF13 <-> FMC_A7    |
                     | PE14 <-> FMC_D11   | PF14 <-> FMC_A8    |
                     | PE15 <-> FMC_D12   | PF15 <-> FMC_A9    |
 +-------------------+--------------------+--------------------+
 | PB5 <-> FMC_SDCKE1|
 | PB6 <-> FMC_SDNE1 |
 | PC0 <-> FMC_SDNWE |
 +-------------------+
*/

void InitRamPin(GPIO_TypeDef* bank, std::initializer_list<uint8_t> pins) {
	for (auto pin : pins) {
		// Set the output speed to highest
		bank->OSPEEDR |= (0b11 << (2 * pin));

		// Set GPIO mode to alternate function (00: Input (reset state)	01: General purpose output mode	10: Alternate function mode	11: Analog mode)
		bank->MODER |= (0b10 << (2 * pin));
		bank->MODER &= ~(0b01 << (2 * pin));

		// Set alternate function to AF12
		if (pin < 8)
			bank->AFR[0] |= 12 << (4 * pin);
		else
			bank->AFR[1] |= 12 << (4 * (pin - 8));
	}
}

void InitSDRAM(void) {

	// Initialise clock for SDRAM pins
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN;
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN;
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOFEN;
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOGEN;

	InitRamPin(GPIOB, {5, 6});
	InitRamPin(GPIOC, {0});
	InitRamPin(GPIOD, {0, 1, 8, 9, 10, 14, 15});
	InitRamPin(GPIOE, {0, 1, 7, 8, 9, 10, 11, 12, 13, 14, 15});
	InitRamPin(GPIOF, {0, 1, 2, 3, 4, 5, 11, 12, 13, 14, 15});
	InitRamPin(GPIOG, {0, 1, 4, 5, 8, 15});

	// Enable the SDRAM Controller
	RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;

	// Clock configuration - FMC clock (fmc_ker_ck) selected with RCC_D1CCIPR_FMCSEL register. Defaults to D1 domain AHB prescaler (RCC_D1CFGR_HPRE_3) ie main clock /2 = 140MHz
	RCC->D1CCIPR |= RCC_D1CCIPR_FMCSEL_1;					// 10: pll2_r_ck clock selected as kernel peripheral clock (286MHz)

	// Memory maximum clock speed is 140MHz at CAS latency = 3

	// SDCLK, RPIPE, RBURST in Control register and TRP, TRC in Timing register are shared and must be configured against bank 1 even though using bank 2
	FMC_Bank5_6_R->SDCR[0] = FMC_SDCRx_RPIPE_0 |			// Read pipe: 01: One HCLK clock cycle delay
			               FMC_SDCRx_RBURST |				// Burst read: 1: single read requests are always managed as bursts
	                       FMC_SDCRx_SDCLK_1;				// SDRAM clock configuration 10: SDCLK period = 2 x HCLK periods (@286MHz = 3.5ns period; 2 x period = 7ns)

	FMC_Bank5_6_R->SDCR[1] = FMC_SDCRx_CAS_Msk |			// CAS Latency in number of memory clock cycles: 11: 3 cycles
	                       FMC_SDCRx_NB |					// Number of internal banks: 1: Four internal Banks (2M x16 x4 Banks)
	                       FMC_SDCRx_MWID_0 |				// Memory data bus width.	00: 8 bits	01: 16 bits	10: 32 bits
	                       FMC_SDCRx_NR_0 |					// Number of row address bits 01: 12 bits = 4096 - See RAM datasheet p2 Device Overview
						   FMC_SDCRx_NC_0;					// Number of column address bits 01: 9 bits = 512

	// ISSI IS42S16800 SDRAM Timings on p17 of datasheet. All settings below in cycles minus one
	FMC_Bank5_6_R->SDTR[0] = 2 << FMC_SDTRx_TRP_Pos |		// Row precharge delay - requires 15ns (3 cycles)
	                       8 << FMC_SDTRx_TRC_Pos;			// Row Cycle Delay - requires 60ns (9 cycles) in data sheet: trc Command Period (REF to REF / ACT to ACT)

	FMC_Bank5_6_R->SDTR[1] = 2 << FMC_SDTRx_TRCD_Pos |		// Row to column delay - requires 15ns (3 cycles)
	                       1 << FMC_SDTRx_TWR_Pos |			// Write Recovery Time - requires 14ns (2 cycles) - in dataheet Input Data To Precharge/Command Delay time
	                       5 << FMC_SDTRx_TRAS_Pos |		// Self Refresh Time - minimum 37ns (6 cycles)
	                       9 << FMC_SDTRx_TXSR_Pos |		// Exit Self Refresh Time - 67ns (10 cycles)
	                       1 << FMC_SDTRx_TMRD_Pos;			// Load to Active Delay - requires 14ns (2 cycle)

	FMC_Bank1_R->BTCR[0] |= FMC_BCR1_FMCEN;					// Enable the FMC Controller (NB datasheet implies only needed for PSRAM/SRAM but SDRAM also requires from testing)

	// p1664  Set MODE to 001 and configure Target Bank 2 to start delivering the clock to the memory (SDCKE is driven high)
	FMC_Bank5_6_R->SDCMR = FMC_SDCMR_CTB2 | 0b001 << FMC_SDCMR_MODE_Pos;		// Command target bank 2 | Clock enable command

	// sleep at least 100uS
	uint32_t time = SysTickVal;
	while (time == SysTickVal) {};

	// Set MODE to 010 and configure Target Bank 2 to issue a "Precharge All" command
	FMC_Bank5_6_R->SDCMR = FMC_SDCMR_CTB2 | 0b010 << FMC_SDCMR_MODE_Pos;		// Command target bank 2 | Precharge All command

	// Set MODE to 011 to configure number of consecutive Auto-refresh commands (NRFS) in the FMC_SDCMR register. RAM Datashseet says at least two are required (p20)
	FMC_Bank5_6_R->SDCMR = FMC_SDCMR_CTB2 |					// Command target bank (bank 2)
	                     0b011 << FMC_SDCMR_MODE_Pos |		// Auto refresh command
	                     3 << FMC_SDCMR_NRFS_Pos;			// Number of auto-refresh less one (ie 4)

	// These parameters are specified in the SDRAM datasheet (p14 for ISSI SDRAM)
#define SDRAM_MODE_BURST_LENGTH_1				((uint16_t)0x0000)
#define SDRAM_MODE_BURST_LENGTH_2				((uint16_t)0x0001)
#define SDRAM_MODE_BURST_LENGTH_4				((uint16_t)0x0002)
#define SDRAM_MODE_BURST_LENGTH_8				((uint16_t)0x0004)
#define SDRAM_MODE_BURST_TYPE_SEQUENTIAL		((uint16_t)0x0000)
#define SDRAM_MODE_BURST_TYPE_INTERLEAVED		((uint16_t)0x0008)
#define SDRAM_MODE_CAS_LATENCY_2				((uint16_t)0x0020)
#define SDRAM_MODE_CAS_LATENCY_3				((uint16_t)0x0030)
#define SDRAM_MODE_OPERATING_MODE_STANDARD		((uint16_t)0x0000)
#define SDRAM_MODE_WRITEBURST_MODE_PROGRAMMED	((uint16_t)0x0000)
#define SDRAM_MODE_WRITEBURST_MODE_SINGLE		((uint16_t)0x0200)

	// Set MODE to 100 to issue a "Load Mode Register" command in order to program the SDRAM
	uint32_t tr_tmp = SDRAM_MODE_BURST_LENGTH_4 |		// was SDRAM_MODE_BURST_LENGTH_2
				SDRAM_MODE_BURST_TYPE_SEQUENTIAL |
				SDRAM_MODE_CAS_LATENCY_3 |
				SDRAM_MODE_OPERATING_MODE_STANDARD |
				SDRAM_MODE_WRITEBURST_MODE_SINGLE;


	//while (FMC_Bank5_6_R->SDSR & FMC_SDSR_BUSY_Msk);
	FMC_Bank5_6_R->SDCMR = FMC_SDCMR_CTB2 |					// Command target bank (bank 2)
	                     0b100 << FMC_SDCMR_MODE_Pos |		// Clock enable command
	                     tr_tmp << FMC_SDCMR_MRD_Pos;		// Mode register direction

	/* Set the refresh counter to trigger an auto refresh often enough to prevent data loss.
	 	Datasheet is contradictory and unclear in its example (p877) - best guess is SDRAM clock frequency = SDCLK period as set above
		Refresh rate = ((SDRAM refresh period / Number of rows) × SDRAM clock frequency) - 20

		Eg COUNT = (64ms / 4096) = 15.625uS
		Refresh rate = (15.625uS * 143MHz) - 20 = 2,214.375
	*/
	FMC_Bank5_6_R->SDRTR = 2214 << FMC_SDRTR_COUNT_Pos;

	// 16 megabytes of ram now available at address 0xD0000000 - 0xD1000000 (for SDRAM Bank2 See manual p129)
}

// SDRAM testing variables
using memSize = uint32_t;
uint32_t testAddr;
memSize* tempAddr;
constexpr uint32_t maxAddr = 0x1000000 / sizeof(memSize);
uint32_t MemTestDuration;
uint32_t MemTestCount = 0;
uint32_t MemTestErrors = 0;

// Tell linker script to store buffer in SDRAM
uint32_t __attribute__((section (".sdramSection"))) SDRAMBuffer[maxAddr];

// Test SDRAM
uint32_t MemoryTest() {
	// Blank
	uint32_t testStart = SysTickVal;
	for (testAddr = 0; testAddr < maxAddr; ++testAddr) {
		tempAddr = (memSize *)(0xD0000000 + (testAddr * sizeof(memSize)));
		*tempAddr = 0;
	}

	// Write
	for (testAddr = 0; testAddr < maxAddr; ++testAddr) {
		tempAddr = (memSize *)(0xD0000000 + (testAddr * sizeof(memSize)));
		*tempAddr = static_cast<memSize>(testAddr);
		if (*tempAddr == 0x00080000) {
			int susp = 1;
		}
	}

	// Read
	int err = 0;
	for (testAddr = 0; testAddr < maxAddr; ++testAddr) {
		memSize val = SDRAMBuffer[testAddr];
		if (val != static_cast<memSize>(testAddr))
			++err;
	}

	MemTestDuration = SysTickVal - testStart;		// For 32 bit words > 6632 using PLL2 at 143MHz, 6309 at 220MHz, 6385 (2813 with DCache, 2347 with DCache and ICache) at 100MHz
	++MemTestCount;
	return err;
}
