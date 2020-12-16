#include "stm32h743xx.h"
#include "initialisation.h"

#if CPUCLOCK == 320
// 8MHz (HSE) / 2 (M) * 160 (N) / 2 (P) = 320MHz
#define PLL_M1 2
#define PLL_N1 160
#define PLL_P1 1			// 0000001: pll1_p_ck = vco1_ck / 2
#define PLL_Q1 2			// This is used for I2S clock: 8MHz (HSE) / 2 (M) * 160 (N) / 2 (Q) = 320MHz
#define PLL_R1 2

#elif CPUCLOCK == 280
// 8MHz (HSE) / 2 (M) * 140 (N) / 2 (P) = 280MHz
#define PLL_M1 2
#define PLL_N1 140
#define PLL_P1 1			// 0000001: pll1_p_ck = vco1_ck / 2
#define PLL_Q1 2			// This is used for I2S clock: 8MHz (HSE) / 2 (M) * 140 (N) / 2 (Q) = 280MHz
#define PLL_R1 2
#elif CPUCLOCK == 240
// 8MHz (HSE) / 2 (M) * 120 (N) / 2 (P) = 240MHz
#define PLL_M1 2
#define PLL_N1 120
#define PLL_P1 1			// 0000001: pll1_p_ck = vco1_ck / 2
#define PLL_Q1 2			// This is used for I2S clock: 8MHz (HSE) / 2 (M) * 120 (N) / 2 (Q) = 240MHz
#define PLL_R1 2
#elif CPUCLOCK == 200
// 8MHz (HSE) / 2 (M) * 100 (N) / 2 (P) = 200MHz
#define PLL_M1 2
#define PLL_N1 100
#define PLL_P1 1			// 0000001: pll1_p_ck = vco1_ck / 2
#define PLL_Q1 2			// This is used for I2S clock: 8MHz (HSE) / 2 (M) * 100 (N) / 2 (Q) = 200MHz FIXME - problems using this PLL divider and FMC
#define PLL_R1 2
#else
// 8MHz (HSE) / 2 (M) * 240 (N) / 2 (P) = 480MHz
#define PLL_M1 2
#define PLL_N1 240
#define PLL_P1 1			// 0000001: pll1_p_ck = vco1_ck / 2
#define PLL_Q1 4			// This is used for I2S clock: 8MHz (HSE) / 2 (M) * 240 (N) / 4 (Q) = 240MHz
#define PLL_R1 2
#endif

// Second PLL used for SDRAM clock
// 8MHz (HSE) / 4 (M) * 200 (N) / 2 (R) = 200MHz
#define PLL_M2 4
#define PLL_N2 200
#define PLL_R2 1			// 1 means Div by 2

void SystemClock_Config()
{

	RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;			// Enable System configuration controller clock

	// Voltage scaling - see Datasheet page 113. VOS1 > 300MHz; VOS2 > 200MHz; VOS3 < 200MHz
	PWR->CR3 &= ~PWR_CR3_SCUEN;						// Supply configuration update enable - this must be deactivated or VOS ready does not come on
#if CPUCLOCK <= 200
	PWR->D3CR |= PWR_D3CR_VOS_0;					// Configure voltage scaling level 2 (0b10 = VOS2)
	PWR->D3CR &= ~PWR_D3CR_VOS_1;
#elif CPUCLOCK <= 280
	PWR->D3CR |= PWR_D3CR_VOS_1;					// Configure voltage scaling level 2 (0b10 = VOS2)
	PWR->D3CR &= ~PWR_D3CR_VOS_0;
#else
	PWR->D3CR |= PWR_D3CR_VOS;						// Configure voltage scaling level 1 before engaging overdrive (0b11 = VOS1)
#endif
	while ((PWR->CSR1 & PWR_CSR1_ACTVOSRDY) == 0);	// Check Voltage ready 1= Ready, voltage level at or above VOS selected level

#if CPUCLOCK == 480
	SYSCFG->PWRCR |= SYSCFG_PWRCR_ODEN;				// Activate the LDO regulator overdrive mode. Must be written only in VOS1 voltage scaling mode.
	while ((SYSCFG->PWRCR & SYSCFG_PWRCR_ODEN) == 0);
#endif

	RCC->CR |= RCC_CR_HSEON;						// Turn on external oscillator
	while ((RCC->CR & RCC_CR_HSERDY) == 0);			// Wait till HSE is ready

	// Clock source to High speed external and main (M) dividers
	RCC->PLLCKSELR = RCC_PLLCKSELR_PLLSRC_HSE |
	                 PLL_M1 << RCC_PLLCKSELR_DIVM1_Pos |
					 PLL_M2 << RCC_PLLCKSELR_DIVM2_Pos;

	// PLL 1 dividers
	RCC->PLL1DIVR = (PLL_N1 - 1) << RCC_PLL1DIVR_N1_Pos |
			        PLL_P1 << RCC_PLL1DIVR_P1_Pos |
			        (PLL_Q1 - 1) << RCC_PLL1DIVR_Q1_Pos |
			        (PLL_R1 - 1) << RCC_PLL1DIVR_R1_Pos;

	RCC->PLLCFGR |= RCC_PLLCFGR_PLL1RGE_2;			// 10: The PLL1 input (ref1_ck) clock range frequency is between 4 and 8 MHz (Will be 4MHz for 8MHz clock divided by 2)
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLL1VCOSEL;		// 0: Wide VCO range:192 to 836 MHz (default after reset)	1: Medium VCO range:150 to 420 MHz
	RCC->PLLCFGR |= RCC_PLLCFGR_DIVP1EN | RCC_PLLCFGR_DIVQ1EN;		// Enable divider outputs

	RCC->CR |= RCC_CR_PLL1ON;						// Turn on main PLL
	while ((RCC->CR & RCC_CR_PLL1RDY) == 0);		// Wait till PLL is ready

	// PLL 2 dividers
	RCC->PLL2DIVR = (PLL_N2 - 1) << RCC_PLL2DIVR_N2_Pos |
			        PLL_R2 << RCC_PLL2DIVR_R2_Pos;

	RCC->PLLCFGR |= RCC_PLLCFGR_PLL2RGE_1;			// 01: The PLL2 input (ref1_ck) clock range frequency is between 2 and 4 MHz (Will be 2MHz for 8MHz clock divided by 4)
	RCC->PLLCFGR |= RCC_PLLCFGR_PLL2VCOSEL;			// 1: Medium VCO range:150 to 420 MHz
	RCC->PLLCFGR |= RCC_PLLCFGR_DIVR2EN;			// Enable R divider output for SDRAM clock

	RCC->CR |= RCC_CR_PLL2ON;						// Turn on PLL 2
	while ((RCC->CR & RCC_CR_PLL2RDY) == 0);		// Wait till PLL is ready

	// Peripheral scalers
	RCC->D1CFGR |= RCC_D1CFGR_HPRE_3;				// D1 domain AHB prescaler - divide by 480MHz by 2 for 240MHz - this is then divided for all APB clocks below
	RCC->D1CFGR |= RCC_D1CFGR_D1PPRE_2; 			// Clock divider for APB3 clocks - set to 4 for 120MHz: 100: hclk / 2
	RCC->D2CFGR |= RCC_D2CFGR_D2PPRE1_2;			// Clock divider for APB1 clocks - set to 4 for 120MHz: 100: hclk / 2
	RCC->D2CFGR |= RCC_D2CFGR_D2PPRE2_2;			// Clock divider for APB2 clocks - set to 4 for 120MHz: 100: hclk / 2
	RCC->D3CFGR |= RCC_D3CFGR_D3PPRE_2;				// Clock divider for APB4 clocks - set to 4 for 120MHz: 100: hclk / 2



	RCC->CFGR |= RCC_CFGR_SW_PLL1;					// System clock switch: 011: PLL1 selected as system clock
	while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != (RCC_CFGR_SW_PLL1 << RCC_CFGR_SWS_Pos));		// Wait until PLL has been selected as system clock source

	// By default Flash latency is set to 7 wait states - set to 4 for now but may need to increase
	FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_4WS;
	while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != FLASH_ACR_LATENCY_4WS);

	// Note that the peripherals can use different clock sources - eg for UART configured thus:
	// RCC->D2CCIP2R |= RCC_D2CCIP2R_USART28SEL;
}


void InitCache()
{
	// Use the Memory Protection Unit (MPU) to set up a region of memory with data caching disabled for use with DMA buffers
	MPU->RNR = 0;									// Memory region number
	MPU->RBAR = reinterpret_cast<uint32_t>(&ADC_array);	// Store the address of the ADC_array into the region base address register

	MPU->RASR = (0b11  << MPU_RASR_AP_Pos)   |		// All access permitted
				(0b001 << MPU_RASR_TEX_Pos)  |		// Type Extension field: See truth table on p228 of CortexM7 programming manual
				(1     << MPU_RASR_S_Pos)    |		// Shareable: provides data synchronization between bus masters. Eg a processor with a DMA controller
				(0     << MPU_RASR_C_Pos)    |		// Cacheable
				(0     << MPU_RASR_B_Pos)    |		// Bufferable (ignored for non-cacheable configuration)
				(17    << MPU_RASR_SIZE_Pos) |		// 256KB - D3 is actually 288K (size is log 2(mem size) - 1 ie 2^18 = 256k)
				(1     << MPU_RASR_ENABLE_Pos);		// Enable MPU region
	MPU->CTRL |= (1 << MPU_CTRL_PRIVDEFENA_Pos) |	// Enable PRIVDEFENA - this allows use of default memory map for memory areas other than those specific regions defined above
				 (1 << MPU_CTRL_ENABLE_Pos);		// Enable the MPU

	// Enable data and instruction caches
	SCB_EnableDCache();

	SCB_EnableICache();
}


void InitSysTick()
{
	SysTick_Config(SystemCoreClock / 1000UL);		// gives 1ms
	NVIC_SetPriority(SysTick_IRQn, 0);
}


void InitDAC()
{
	// DAC1_OUT2 on PA5
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;			// GPIO port clock
	RCC->APB1LENR |= RCC_APB1LENR_DAC12EN;			// Enable DAC Clock

	DAC1->CR |= DAC_CR_EN1;							// Enable DAC using PA4 (DAC_OUT1)
	DAC1->MCR &= DAC_MCR_MODE1_Msk;					// Mode = 0 means Buffer activated, Connected to external pin

	DAC1->CR |= DAC_CR_EN2;							// Enable DAC using PA5 (DAC_OUT2)
	DAC1->MCR &= DAC_MCR_MODE2_Msk;					// Mode = 0 means Buffer activated, Connected to external pin
}


void InitAdcPins(std::initializer_list<uint8_t> channels) {
	uint8_t sequence = 1;

	for (auto channel: channels) {
		// NB reset mode of GPIO pins is 0b11 = analog mode so shouldn't need to change
		ADC1->PCSEL |= 1 << channel;					// ADC channel preselection register

		// Set conversion sequence to order ADC channels are passed to this function
		if (sequence < 5) {
			ADC1->SQR1 |= channel << (sequence * 6);
		} else if (sequence < 10) {
			ADC1->SQR2 |= channel << ((sequence - 5) * 6);
		} else if (sequence < 15)  {
			ADC1->SQR3 |= channel << ((sequence - 10) * 6);
		} else {
			ADC1->SQR4 |= channel << ((sequence - 15) * 6);
		}

		// 000: 1.5 ADC clock cycles; 001: 2.5 cycles; 010: 8.5 cycles;	011: 16.5 cycles; 100: 32.5 cycles; 101: 64.5 cycles; 110: 387.5 cycles; 111: 810.5 cycles
		if (channel < 10)
			ADC1->SMPR1 |= 0b000 << (3 * channel);
		else
			ADC1->SMPR2 |= 0b000 << (3 * (channel - 10));

		sequence++;
	}
}

void InitADC()
{
	// Configure clocks
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;			// GPIO port clock
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN;

	RCC->AHB1ENR |= RCC_AHB1ENR_ADC12EN;
	// FIXME - currently ADC clock is set to HSI - might be more accurate to use HSE
	RCC->D3CCIPR |= RCC_D3CCIPR_ADCSEL_1;			// SAR ADC kernel clock source selection: 10: per_ck clock (hse_ck, hsi_ker_ck or csi_ker_ck according to CKPERSEL in RCC->D1CCIPR p.353)
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

	//ADC12_COMMON->CCR |= ADC_CCR_PRESC_0;

	// Initialize ADC peripheral
	DMA1_Stream1->CR &= ~DMA_SxCR_EN;
	DMA1_Stream1->CR |= DMA_SxCR_CIRC;				// Circular mode to keep refilling buffer
	DMA1_Stream1->CR |= DMA_SxCR_MINC;				// Memory in increment mode
	DMA1_Stream1->CR |= DMA_SxCR_PSIZE_0;			// Peripheral size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA1_Stream1->CR |= DMA_SxCR_MSIZE_0;			// Memory size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA1_Stream1->CR |= DMA_SxCR_PL_0;				// Priority: 00 = low; 01 = Medium; 10 = High; 11 = Very High

	DMA1_Stream1->FCR &= ~DMA_SxFCR_FTH;			// Disable FIFO Threshold selection
	DMA1->LIFCR = 0x3FUL << 6;						// clear interrupts for this stream

	DMAMUX1_Channel1->CCR |= 0x9; 					// DMA request MUX input 9 = adc1_dma (See p.695)
	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF1; // Channel 1 Clear synchronization overrun event flag

	//NVIC_SetPriority(DMA1_Stream1_IRQn, 1);
	//NVIC_EnableIRQ(DMA1_Stream1_IRQn);

	ADC1->CR &= ~ADC_CR_DEEPPWD;					// Deep power down: 0: ADC not in deep-power down	1: ADC in deep-power-down (default reset state)
	ADC1->CR |= ADC_CR_ADVREGEN;					// Enable ADC internal voltage regulator

	// Wait until voltage regulator settled
	volatile uint32_t wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL) {
		wait_loop_index--;
	}
	while ((ADC1->CR & ADC_CR_ADVREGEN) != ADC_CR_ADVREGEN) {}

	ADC12_COMMON->CCR |= ADC_CCR_PRESC_0;
	ADC1->CFGR |= ADC_CFGR_CONT;					// 1: Continuous conversion mode for regular conversions
	ADC1->CFGR |= ADC_CFGR_OVRMOD;					// Overrun Mode 1: ADC_DR register is overwritten with the last conversion result when an overrun is detected.
	ADC1->CFGR |= ADC_CFGR_DMNGT;					// Data Management configuration 11: DMA Circular Mode selected

	// Initialise 8x oversampling
//	ADC1->CFGR2 |= (8-1) << ADC_CFGR2_OVSR_Pos;		// Number of oversamples = 8
//	ADC1->CFGR2 |= 3 << ADC_CFGR2_OVSS_Pos;			// Divide oversampled total via bit shift of 3 - ie divide by 8
//	ADC1->CFGR2 |= ADC_CFGR2_ROVSE;

	// Oversampling 2x (Smoother with no oversampling)
//	ADC1->CFGR2 |= (2-1) << ADC_CFGR2_OVSR_Pos;		// Number of oversamples = 2
//	ADC1->CFGR2 |= 1 << ADC_CFGR2_OVSS_Pos;			// Divide oversampled total via bit shift of 1 - ie divide by 2
//	ADC1->CFGR2 |= ADC_CFGR2_ROVSE;


	// Boost mode 1: Boost mode on. Must be used when ADC clock > 20 MHz.
	ADC1->CR |= ADC_CR_BOOST_1;						// Note this sets reserved bit according to SFR - HAL has notes about silicon revision

	// For scan mode: set number of channels to be converted
	ADC1->SQR1 |= (ADC_BUFFER_LENGTH - 1);

	// Start calibration
	ADC1->CR |= ADC_CR_ADCAL;
	while ((ADC1->CR & ADC_CR_ADCAL) == ADC_CR_ADCAL) {};

	/*--------------------------------------------------------------------------------------------
	Configure ADC Channels to be converted:
		0	PA2 ADC12_INP14		AUDIO_IN_L
		1	PA3 ADC12_INP15 	AUDIO_IN_R
		2	PC5 ADC12_INP8		WET_DRY_MIX
		3	PB1 ADC12_INP5		DELAY_POT_L
		4	PA1 ADC1_INP17		DELAY_POT_R
		5	PA0 ADC1_INP16		DELAY_CV_SCALED_L
		6	PA6 ADC12_INP3 		DELAY_CV_SCALED_R
		7	PB0 ADC12_INP9		FEEDBACK_POT
		8	PC4 ADC12_INP4		FEEDBACK_CV_SCALED
		9	PC1 ADC123_INP11 	TONE_POT
	*/
	InitAdcPins({14, 15, 8, 5, 17, 16, 3, 9, 4, 11});

	// Enable ADC
	ADC1->CR |= ADC_CR_ADEN;
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {}

	// With DMA, overrun event is always considered as an error even if hadc->Init.Overrun is set to ADC_OVR_DATA_OVERWRITTEN. Therefore, ADC_IT_OVR is enabled.
	//ADC1->IER |= ADC_IER_OVRIE;

	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF1; // Channel 1 Clear synchronization overrun event flag
	DMA1->LIFCR = 0x3FUL << 6;		// clear interrupts for this stream

	DMA1_Stream1->NDTR |= ADC_BUFFER_LENGTH;		// Number of data items to transfer (ie size of ADC buffer)
	DMA1_Stream1->PAR = (uint32_t)(&(ADC1->DR));	// Configure the peripheral data register address 0x40022040
	DMA1_Stream1->M0AR = (uint32_t)(ADC_array);		// Configure the memory address (note that M1AR is used for double-buffer mode) 0x24000040

	DMA1_Stream1->CR |= DMA_SxCR_EN;				// Enable DMA and wait
	wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL) {
	  wait_loop_index--;
	}

	ADC1->CR |= ADC_CR_ADSTART;						// Start ADC
}


void InitI2S() {
	/* Available I2S2 pins on AF5
	PA9  I2S2_CK
	PA11 I2S2_WS
	PA12 I2S2_CK
*	PB9  I2S2_WS
	PB10 I2S2_CK
	PB12 I2S2_WS
x	PB13 I2S2_CK		on nucleo jumpered to Ethernet and not working
	PB15 I2S2_SDO
	PC1  I2S2_SDO
*	PC3  I2S2_SDO
*	PD3  I2S2_CK
	*/

	//	Enable GPIO and SPI clocks
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;			// GPIO port clock
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN;			// GPIO port clock
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;			// GPIO port clock
	RCC->APB1LENR |= RCC_APB1LENR_SPI2EN;

	// PB9: I2S2_WS [alternate function AF5]
	GPIOB->MODER &= ~GPIO_MODER_MODE9_0;			// 10: Alternate function mode
	GPIOB->AFR[1] |= 5 << GPIO_AFRH_AFSEL9_Pos;		// Alternate Function 5 (I2S2)

	// PD3 I2S2_CK [alternate function AF5]
	GPIOD->MODER &= ~GPIO_MODER_MODE3_0;			// 00: Input (reset state)	01: General purpose output mode	10: Alternate function mode	11: Analog mode
	GPIOD->AFR[0] |= 5 << GPIO_AFRL_AFSEL3_Pos;		// Alternate Function 5 (I2S2)

	// PC3 I2S2_SDO [alternate function AF5]
	GPIOC->MODER &= ~GPIO_MODER_MODE3_0;			// 00: Input (reset state)	01: General purpose output mode	10: Alternate function mode	11: Analog mode
	GPIOC->AFR[0] |= 5 << GPIO_AFRL_AFSEL3_Pos;		// Alternate Function 5 (I2S2)


	// Configure SPI (Shown as SPI2->CGFR in SFR)
	SPI2->I2SCFGR |= SPI_I2SCFGR_I2SMOD;			// I2S Mode
	SPI2->I2SCFGR |= SPI_I2SCFGR_I2SCFG_1;			// I2S configuration mode: 00=Slave transmit; 01=Slave receive; 10=Master transmit; 11=Master receive

	// Use a 16bit data length but pack into the upper 16 bits of a 32 bit word
	SPI2->I2SCFGR &= ~SPI_I2SCFGR_DATLEN;			// Data Length 00=16-bit; 01=24-bit; 10=32-bit
	SPI2->I2SCFGR |= SPI_I2SCFGR_CHLEN;				// Channel Length = 32bits

	/* I2S Clock
	000: pll1_q_ck clock selected as SPI/I2S1,2 and 3 kernel clock (default after reset)
	001: pll2_p_ck clock selected as SPI/I2S1,2 and 3 kernel clock
	010: pll3_p_ck clock selected as SPI/I2S1,2 and 3 kernel clock
	011: I2S_CKIN clock selected as SPI/I2S1,2 and 3 kernel clock
	100: per_ck clock selected as SPI/I2S1,2 and 3 kernel clock
	//RCC->D2CCIP1R |= RCC_D2CCIP1R_SPI123SEL;

	I2S Prescaler Clock calculations:
	FS = I2SxCLK / [(32*2)*((2*I2SDIV)+ODD))]

	I2S Clock = 200MHz:			200000000 / (32*2  * ((2 * 32) + 1)) = 48076.92
	I2S Clock = 240MHz:			240000000 / (32*2  * ((2 * 39) + 0)) = 48076.92
	I2S Clock = 300MHz:			280000000 / (32*2  * ((2 * 45) + 1)) = 48076.92
	I2S Clock = 320MHz: 		320000000 / (32*2  * ((2 * 52) + 0)) = 48076.92
	PER_CLK = 64MHz				64000000  / (32*2  * ((2 * 10) + 1)) = 47619.05

	Note timing problems experienced using both pll1_q_ck and pll2_p_ck
	*/
#define I2S_PER_CLK
#ifdef I2S_PLL2P_CLK
	// Use PLL2 clock - configured to 200MHz
	RCC->D2CCIP1R |= RCC_D2CCIP1R_SPI123SEL_0;
	SPI2->I2SCFGR |= (32 << SPI_I2SCFGR_I2SDIV_Pos);// Set I2SDIV to 45 with Odd factor prescaler
	SPI2->I2SCFGR |= SPI_I2SCFGR_ODD;
#endif

#ifdef I2S_PER_CLK
	// Use peripheral clock - configured to internal HSI at 64MHz
	RCC->D2CCIP1R |= RCC_D2CCIP1R_SPI123SEL_2;
	SPI2->I2SCFGR |= (10 << SPI_I2SCFGR_I2SDIV_Pos);// Set I2SDIV to 45 with Odd factor prescaler
	SPI2->I2SCFGR |= SPI_I2SCFGR_ODD;
#endif

#ifdef I2S_DEFAULT
#if CPUCLOCK == 200
	SPI2->I2SCFGR |= (32 << SPI_I2SCFGR_I2SDIV_Pos);// Set I2SDIV to 45 with Odd factor prescaler
	SPI2->I2SCFGR |= SPI_I2SCFGR_ODD;
#elif CPUCLOCK == 280
	SPI2->I2SCFGR |= (45 << SPI_I2SCFGR_I2SDIV_Pos);// Set I2SDIV to 45 with Odd factor prescaler
	SPI2->I2SCFGR |= SPI_I2SCFGR_ODD;
#elif CPUCLOCK == 320
	SPI2->I2SCFGR |= (52 << SPI_I2SCFGR_I2SDIV_Pos);// Set I2SDIV to 52 with no Odd factor prescaler
#else
	SPI2->I2SCFGR |= (39 << SPI_I2SCFGR_I2SDIV_Pos);// Set I2SDIV to 39 with no Odd factor prescaler
#endif
#endif

	// Enable interrupt when TxFIFO threshold reached
	SPI2->IER |= SPI_IER_TXPIE;
	NVIC_SetPriority(SPI2_IRQn, 2);					// Lower is higher priority
	NVIC_EnableIRQ(SPI2_IRQn);

	SPI2->CR1 |= SPI_CR1_SPE;						// Enable I2S
	SPI2->CR1 |= SPI_CR1_CSTART;					// Start I2S
}


void InitTempoClock()
{
	// Fire interrupt when clock pulse is received on PA7 - See manual p770
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;			// GPIO port clock
	GPIOA->MODER &= ~GPIO_MODER_MODE7_Msk;			// 00: Input mode
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI7_PA;	// Select Pin PA7 which uses External interrupt 2
	EXTI->RTSR1 |= EXTI_RTSR1_TR7;					// Enable rising edge trigger
	EXTI->IMR1 |= EXTI_IMR1_IM7;					// Activate interrupt using mask register

	NVIC_SetPriority(EXTI9_5_IRQn, 4);				// Lower is higher priority
	NVIC_EnableIRQ(EXTI9_5_IRQn);
}


void InitClockTimer()
{
	// Configure timer to use Capture and Compare mode on external clock to time duration between pulses (not using)

	// FIXME Production will use PA7 - temporarily using PB5 as also configured as TIM3_CH2
	// See manual page 1670 for info on Capture and Compare Input mode
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;			// GPIO port clock
	GPIOB->MODER &= ~GPIO_MODER_MODE5_0;			// Alternate function is Mode 0b10 (defaults to 0b11)
	GPIOB->AFR[0] |= 2 << GPIO_AFRL_AFSEL5_Pos;		// Alternate Function 2 for TIM3_CH2 on PB5 and PA7
	RCC->APB1LENR |= RCC_APB1LENR_TIM3EN;
	TIM3->ARR = 65535;
	TIM3->PSC = 2000;

	// TISEL Register is used to select which channel is routed to TI1, TI2, TI3 and TI4 inputs. By default CH2 is routed to TI2 so use that
	// TIM3->TISEL |= 0 << TIM_TISEL_TI2SEL_Pos;

	// There are two capture and compare registers. Configure the input to CCR1 to be CH2
	TIM3->CCMR1 |= TIM_CCMR1_CC1S_1;				// 10: CC1 channel is configured as input, IC1 is mapped on TI2

	// Configure the digital filter to allow time for the input signal to stabilise - initially set to 0
	// Note that the number of units depends on the filter clock - this is either the main timer or a subdivision set by the DTS (See CKD in CR1)
	TIM3->CCMR1 |= TIM_CCMR1_IC1F_0 & TIM_CCMR1_IC1F_1;

	// TIMx_CCER register controls detection on rising (default) or falling edge of input
	TIM3->CCER |= TIM_CCER_CC1E;					// Enable capture on 1

	// Configure the timer to reset when a rising edge is detected
	TIM3->SMCR |= 0b110 << TIM_SMCR_TS_Pos;			// 00110: Filtered Timer Input 2 (TI2FP2)
	TIM3->SMCR |= TIM_SMCR_SMS_2;					// 0100: Reset Mode - Rising edge of the selected trigger input (TRGI) reinitializes the counter

	TIM3->CR1 |= TIM_CR1_CEN;

}


void InitLEDs()
{
	// Initialise timing LEDs on PC10 and PC11
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN;			// GPIO port clock
	GPIOC->MODER &= ~GPIO_MODER_MODE10_1;			// 00: Input (reset state)	01: General purpose output mode	10: Alternate function mode	11: Analog mode
	GPIOC->MODER &= ~GPIO_MODER_MODE11_1;			// 00: Input (reset state)	01: General purpose output mode	10: Alternate function mode	11: Analog mode
}
