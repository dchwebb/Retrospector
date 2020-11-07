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
void sdram_init(void) {

	// Initialise clock for SDRAM pins
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;			// reset and clock control - advanced high performance bus - GPIO port B
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN;			// reset and clock control - advanced high performance bus - GPIO port C
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;			// reset and clock control - advanced high performance bus - GPIO port D
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN;			// reset and clock control - advanced high performance bus - GPIO port E
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOFEN;			// reset and clock control - advanced high performance bus - GPIO port F
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOGEN;			// reset and clock control - advanced high performance bus - GPIO port G

	// It appears it is necessary to set the output speed even though GPIO is configured as Alternate function
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED5  | GPIO_OSPEEDR_OSPEED6;
	GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED0;
	GPIOD->OSPEEDR |= GPIO_OSPEEDR_OSPEED0  | GPIO_OSPEEDR_OSPEED1  | GPIO_OSPEEDR_OSPEED8  | GPIO_OSPEEDR_OSPEED9 | GPIO_OSPEEDR_OSPEED10 | GPIO_OSPEEDR_OSPEED14 | GPIO_OSPEEDR_OSPEED15;
	GPIOE->OSPEEDR |= GPIO_OSPEEDR_OSPEED0  | GPIO_OSPEEDR_OSPEED1  | GPIO_OSPEEDR_OSPEED7  | GPIO_OSPEEDR_OSPEED8 | GPIO_OSPEEDR_OSPEED9  | GPIO_OSPEEDR_OSPEED10 | GPIO_OSPEEDR_OSPEED11 |
			          GPIO_OSPEEDR_OSPEED12 | GPIO_OSPEEDR_OSPEED13 | GPIO_OSPEEDR_OSPEED14 | GPIO_OSPEEDR_OSPEED15;
	GPIOF->OSPEEDR |= GPIO_OSPEEDR_OSPEED0  | GPIO_OSPEEDR_OSPEED1  | GPIO_OSPEEDR_OSPEED2  | GPIO_OSPEEDR_OSPEED3 | GPIO_OSPEEDR_OSPEED4  | GPIO_OSPEEDR_OSPEED5  | GPIO_OSPEEDR_OSPEED11 |
			          GPIO_OSPEEDR_OSPEED12 | GPIO_OSPEEDR_OSPEED13 | GPIO_OSPEEDR_OSPEED14 | GPIO_OSPEEDR_OSPEED15;
	GPIOG->OSPEEDR |= GPIO_OSPEEDR_OSPEED0  | GPIO_OSPEEDR_OSPEED1  | GPIO_OSPEEDR_OSPEED4  | GPIO_OSPEEDR_OSPEED5 | GPIO_OSPEEDR_OSPEED8  | GPIO_OSPEEDR_OSPEED15;

	// Set GPIO mode to alternate function 10: Alternate function mode
	GPIOB->MODER |= GPIO_MODER_MODE5_1  | GPIO_MODER_MODE6_1;
	GPIOC->MODER |= GPIO_MODER_MODE0_1;
	GPIOD->MODER |= GPIO_MODER_MODE0_1  | GPIO_MODER_MODE1_1  | GPIO_MODER_MODE8_1  | GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1 | GPIO_MODER_MODE14_1 | GPIO_MODER_MODE15_1;
	GPIOE->MODER |= GPIO_MODER_MODE0_1  | GPIO_MODER_MODE1_1  | GPIO_MODER_MODE7_1  | GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1  | GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1 |
			        GPIO_MODER_MODE12_1 | GPIO_MODER_MODE13_1 | GPIO_MODER_MODE14_1 | GPIO_MODER_MODE15_1;
	GPIOF->MODER |= GPIO_MODER_MODE0_1  | GPIO_MODER_MODE1_1  | GPIO_MODER_MODE2_1  | GPIO_MODER_MODE3_1 | GPIO_MODER_MODE4_1  | GPIO_MODER_MODE5_1  | GPIO_MODER_MODE11_1 |
			        GPIO_MODER_MODE12_1 | GPIO_MODER_MODE13_1 | GPIO_MODER_MODE14_1 | GPIO_MODER_MODE15_1;
	GPIOG->MODER |= GPIO_MODER_MODE0_1  | GPIO_MODER_MODE1_1  | GPIO_MODER_MODE4_1  | GPIO_MODER_MODE5_1 | GPIO_MODER_MODE8_1  | GPIO_MODER_MODE15_1;

	// Set alternate function to AF12
	GPIOB->AFR[0] |= 12 << GPIO_AFRL_AFSEL5_Pos  | 12 << GPIO_AFRL_AFSEL6_Pos;
	GPIOC->AFR[0] |= 12 << GPIO_AFRL_AFSEL0_Pos;
	GPIOD->AFR[0] |= 12 << GPIO_AFRL_AFSEL0_Pos  | 12 << GPIO_AFRL_AFSEL1_Pos;
	GPIOD->AFR[1] |= 12 << GPIO_AFRH_AFSEL8_Pos  | 12 << GPIO_AFRH_AFSEL9_Pos  | 12 << GPIO_AFRH_AFSEL10_Pos | 12 << GPIO_AFRH_AFSEL14_Pos | 12 << GPIO_AFRH_AFSEL15_Pos;
	GPIOE->AFR[0] |= 12 << GPIO_AFRL_AFSEL0_Pos  | 12 << GPIO_AFRL_AFSEL1_Pos  | 12 << GPIO_AFRL_AFSEL7_Pos;
	GPIOE->AFR[1] |= 12 << GPIO_AFRH_AFSEL8_Pos  | 12 << GPIO_AFRH_AFSEL9_Pos  | 12 << GPIO_AFRH_AFSEL10_Pos | 12 << GPIO_AFRH_AFSEL11_Pos | 12 << GPIO_AFRH_AFSEL12_Pos | 12 << GPIO_AFRH_AFSEL13_Pos | 12 << GPIO_AFRH_AFSEL14_Pos | 12 << GPIO_AFRH_AFSEL15_Pos;
	GPIOF->AFR[0] |= 12 << GPIO_AFRL_AFSEL0_Pos  | 12 << GPIO_AFRL_AFSEL1_Pos  | 12 << GPIO_AFRL_AFSEL2_Pos  | 12 << GPIO_AFRL_AFSEL3_Pos  | 12 << GPIO_AFRL_AFSEL4_Pos  | 12 << GPIO_AFRL_AFSEL5_Pos;
	GPIOF->AFR[1] |= 12 << GPIO_AFRH_AFSEL11_Pos | 12 << GPIO_AFRH_AFSEL12_Pos | 12 << GPIO_AFRH_AFSEL13_Pos | 12 << GPIO_AFRH_AFSEL14_Pos | 12 << GPIO_AFRH_AFSEL15_Pos;
	GPIOG->AFR[0] |= 12 << GPIO_AFRL_AFSEL0_Pos  | 12 << GPIO_AFRL_AFSEL1_Pos  | 12 << GPIO_AFRL_AFSEL4_Pos  | 12 << GPIO_AFRL_AFSEL5_Pos;
	GPIOG->AFR[1] |= 12 << GPIO_AFRH_AFSEL8_Pos  | 12 << GPIO_AFRH_AFSEL15_Pos;

	// Enable the SDRAM Controller
	RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;
	//while (FMC_Bank5_6_R->SDSR & FMC_SDSR_BUSY_Msk);

	// SDCLK, RPIPE, RBURST in Control register and TRP, TRC in Timing register are shared and must be configured against bank 1 even though using bank 2
	FMC_Bank5_6_R->SDCR[0] = FMC_SDCRx_RPIPE_0 |			// Read pipe: 01: One HCLK clock cycle delay
			               FMC_SDCRx_RBURST |				// Burst read: 1: single read requests are always managed as bursts
	                       FMC_SDCRx_SDCLK_1;				// SDRAM clock configuration 10: SDCLK period = 2 x HCLK periods (@168MHz = 5.95ns period; 2 x period = 11.90ns)

	FMC_Bank5_6_R->SDCR[1] = FMC_SDCRx_CAS_Msk |			// CAS Latency in number of memory clock cycles: 11: 3 cycles
	                       FMC_SDCRx_NB |					// Number of internal banks: 1: Four internal Banks
	                       FMC_SDCRx_MWID_0 |				// Memory data bus width.	00: 8 bits	01: 16 bits	10: 32 bits
	                       FMC_SDCRx_NR_0;					// Number of row address bits 01: 12 bits (FMC_SDCR2_NC: number of column address bits defaults to 8 bits)

	// ISSI IS42S16400J SDRAM Timings on p16 of datasheet. All settings below in cycles minus one (@180MHz = 5.55ns period; 2 x period = 11.11ns)
	FMC_Bank5_6_R->SDTR[0] = 1 << FMC_SDTRx_TRP_Pos |		// Row precharge delay - requires 15ns (2 cycles)
	                       5 << FMC_SDTRx_TRC_Pos;			// Row Cycle Delay - requires 63ns (6 cycles)

	FMC_Bank5_6_R->SDTR[1] = 1 << FMC_SDTRx_TRCD_Pos |		// Row to column delay - requires 15ns (2 cycles)
	                       1 << FMC_SDTRx_TWR_Pos |			// Write Recovery Time - requires 2 cycles
	                       3 << FMC_SDTRx_TRAS_Pos |		// Self Refresh Time - minimum 42ns (4 cycles)
	                       6 << FMC_SDTRx_TXSR_Pos |		// Exit Self Refresh Time - 70ns (7 cycles)
	                       1 << FMC_SDTRx_TMRD_Pos;			// Load to Active Delay - requires 2 cycles

	FMC_Bank1_R->BTCR[0] |= FMC_BCR1_FMCEN;		// FROM Example

	// p1664  Set MODE to 001 and configure Target Bank 2 to start delivering the clock to the memory (SDCKE is driven high)
	//while (FMC_Bank5_6_R->SDSR & FMC_SDSR_BUSY_Msk);
	FMC_Bank5_6_R->SDCMR = FMC_SDCMR_CTB2 | 0b001 << FMC_SDCMR_MODE_Pos;		// Command target bank 2 | Clock enable command

	// sleep at least 100uS
	uint32_t time = SysTickVal;
	while (time == SysTickVal) {};

	// Set MODE to 010 and configure Target Bank 2 to issue a "Precharge All" command
	//while (FMC_Bank5_6_R->SDSR & FMC_SDSR_BUSY_Msk);
	FMC_Bank5_6_R->SDCMR = FMC_SDCMR_CTB2 | 0b010 << FMC_SDCMR_MODE_Pos;		// Command target bank 2 | Precharge All command

	// Set MODE to 011 to configure number of consecutive Auto-refresh commands (NRFS) in the FMC_SDCMR register. Typical number is 8
	//while (FMC_Bank5_6_R->SDSR & FMC_SDSR_BUSY_Msk);
	FMC_Bank5_6_R->SDCMR = FMC_SDCMR_CTB2 |					// Command target bank (bank 2)
	                     0b011 << FMC_SDCMR_MODE_Pos |		// Auto refresh command
	                     3 << FMC_SDCMR_NRFS_Pos;			// Number of auto-refresh less one (ie 4)

	// These parameters are specified in the SDRAM datasheet (p19 for ISSI SDRAM)
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

	// Set the refresh counter to trigger an auto refresh often enough to prevent data loss.
	FMC_Bank5_6_R->SDRTR = 683 << FMC_SDRTR_COUNT_Pos;

	// 8 megabytes of ram now available at address 0xD0000000 (for SDRAM Bank2 See manual p805)
}
