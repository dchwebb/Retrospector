#include "stm32h743xx.h"
#include "initialisation.h"
//#include "initialisation.h"


#define PLL_M 2
#define PLL_N 240
#define PLL_P 1		//  0000001: pll1_p_ck = vco1_ck / 2
#define PLL_Q 4
#define PLL_R 2

void SystemClock_Config()
{

	RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;			// Enable System configuration controller clock

	PWR->CR3 &= ~PWR_CR3_SCUEN;						// Supply configuration update enable - this must be deactivated or VOS ready does not come on
	PWR->D3CR |= PWR_D3CR_VOS;						// Configure voltage scaling level 1 before engaging overdrive (0b11 = VOS1)
	while ((PWR->CSR1 & PWR_CSR1_ACTVOSRDY) == 0);	// Check Voltage ready 1= Ready, voltage level at or above VOS selected level

	SYSCFG->PWRCR |= SYSCFG_PWRCR_ODEN;				// Activate the LDO regulator overdrive mode. Must be written only in VOS1 voltage scaling mode.
	while ((SYSCFG->PWRCR & SYSCFG_PWRCR_ODEN) == 0);

	RCC->CR |= RCC_CR_HSEON;						// Turn on external oscillator
	while ((RCC->CR & RCC_CR_HSERDY) == 0);			// Wait till HSE is ready

/*	RCC->CR |= RCC_CR_HSI48ON;						// Enable Internal High Speed oscillator for USB
	while ((RCC->CR & RCC_CR_HSI48RDY) == 0);		// Wait till internal USB oscillator is ready
*/

	// Clock source to High speed external and main (M) divider
	RCC->PLLCKSELR = RCC_PLLCKSELR_PLLSRC_HSE |
	                 PLL_M << RCC_PLLCKSELR_DIVM1_Pos;

	// PLL dividers
	RCC->PLL1DIVR = (PLL_N - 1) << RCC_PLL1DIVR_N1_Pos |
			        PLL_P << RCC_PLL1DIVR_P1_Pos |
			        (PLL_Q - 1) << RCC_PLL1DIVR_Q1_Pos |
			        (PLL_R - 1) << RCC_PLL1DIVR_R1_Pos;

	RCC->PLLCFGR |= RCC_PLLCFGR_PLL1RGE_2;			// 10: The PLL1 input (ref1_ck) clock range frequency is between 4 and 8 MHz (Will be 4MHz for 8MHz clock divided by 2)
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLL1VCOSEL;		// 0: Wide VCO range:192 to 836 MHz (default after reset)	1: Medium VCO range:150 to 420 MHz
	RCC->PLLCFGR |= RCC_PLLCFGR_DIVP1EN | RCC_PLLCFGR_DIVQ1EN | RCC_PLLCFGR_DIVR1EN;		// Enable divider outputs
	//RCC->PLLCFGR |= RCC_PLLCFGR_PLL1FRACEN;		// FIXME enables fractional divider - not sure if this is necessary as not using

	RCC->CR |= RCC_CR_PLL1ON;						// Turn on main PLL
	while ((RCC->CR & RCC_CR_PLL1RDY) == 0);		// Wait till PLL is ready

	RCC->D1CFGR |= RCC_D1CFGR_HPRE_3;				// D1 domain AHB prescaler - set to 8 for 240MHz - this is then divided for all APB clocks below
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


void InitADC()
{
	// PA6 ADC12_INP3 | PC0 ADC123_INP10 | PA3 ADC12_INP15 | PB1 ADC12_INP5

	// Enable instruction and data cache - core_cm7.h
	// See https://community.st.com/s/article/FAQ-DMA-is-not-working-on-STM32H7-devices
	//SCB_EnableICache();
	//SCB_EnableDCache();

	// Configure clocks
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;			// GPIO port clock
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN;

	RCC->AHB1ENR |= RCC_AHB1ENR_ADC12EN;
	// FIXME - currently ADC clock is set to HSI - might be more accurate to use HSE
	RCC->D3CCIPR |= RCC_D3CCIPR_ADCSEL_1;			// SAR ADC kernel clock source selection: 10: per_ck clock (hse_ck, hsi_ker_ck or csi_ker_ck according to CKPERSEL in RCC->D1CCIPR p.353)
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

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
	ADC1->CFGR2 |= (8-1) << ADC_CFGR2_OVSR_Pos;		// Number of oversamples = 8
	ADC1->CFGR2 |= 3 << ADC_CFGR2_OVSS_Pos;			// Divide oversampled total via bit shift of 3 - ie divide by 8
	ADC1->CFGR2 |= ADC_CFGR2_ROVSE;

	// Boost mode 1: Boost mode on. Must be used when ADC clock > 20 MHz.
	ADC1->CR |= ADC_CR_BOOST_1;						// Note this sets reserved bit according to SFR - HAL has notes about silicon revision

	// For scan mode: set number of channels to be converted
	ADC1->SQR1 |= (ADC_BUFFER_LENGTH - 1);

	// Start calibration
	ADC1->CR |= ADC_CR_ADCAL;
	while ((ADC1->CR & ADC_CR_ADCAL) == ADC_CR_ADCAL) {};

	/*--------------------------------------------------------------------------------------------*/
	// Configure ADC Channels to be converted: PA6 ADC12_INP3 | PC1 ADC123_INP11 | PA3 ADC12_INP15 | PB1 ADC12_INP5
	// NB reset mode of GPIO pins is 0b11 = analog mode so shouldn't need to change
	ADC1->PCSEL |= ADC_PCSEL_PCSEL_3;				// ADC channel preselection register
	ADC1->SQR1 |= 3  << ADC_SQR1_SQ1_Pos;			// Set 1st conversion in sequence
	ADC1->PCSEL |= ADC_PCSEL_PCSEL_11;				// ADC channel preselection register
	ADC1->SQR1 |= 10 << ADC_SQR1_SQ2_Pos;			// Set 2nd conversion in sequence
	ADC1->PCSEL |= ADC_PCSEL_PCSEL_15;				// ADC channel preselection register
	ADC1->SQR1 |= 15 << ADC_SQR1_SQ3_Pos;			// Set 3rd conversion in sequence
	ADC1->PCSEL |= ADC_PCSEL_PCSEL_5;				// ADC channel preselection register
	ADC1->SQR1 |= 5  << ADC_SQR1_SQ4_Pos;			// Set 4th conversion in sequence

	// 000: 1.5 ADC clock cycles; 001: 2.5 cycles; 010: 8.5 cycles;	011: 16.5 cycles; 100: 32.5 cycles; 101: 64.5 cycles; 110: 387.5 cycles; 111: 810.5 cycles
	ADC1->SMPR1 |= 0b010 << ADC_SMPR1_SMP3_Pos;		// Set conversion speed
	ADC1->SMPR2 |= 0b010 << ADC_SMPR2_SMP11_Pos;	// Set conversion speed
	ADC1->SMPR2 |= 0b010 << ADC_SMPR2_SMP15_Pos;	// Set conversion speed
	ADC1->SMPR1 |= 0b010 << ADC_SMPR1_SMP5_Pos;		// Set conversion speed

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

/*
	// PB10 I2S2_CK [alternate function AF5]
	GPIOB->MODER &= ~GPIO_MODER_MODE10_0;			// 00: Input (reset state)	01: General purpose output mode	10: Alternate function mode	11: Analog mode
	GPIOB->AFR[1] |= 5 << GPIO_AFRH_AFSEL10_Pos;	// Alternate Function 5 (I2S2)

	// PB15 I2S2_SDO [alternate function AF5]
	GPIOB->MODER &= ~GPIO_MODER_MODE15_0;			// 00: Input (reset state)	01: General purpose output mode	10: Alternate function mode	11: Analog mode
	GPIOB->AFR[1] |= 5 << GPIO_AFRH_AFSEL15_Pos;	// Alternate Function 5 (I2S2)
*/

	// PC3 I2S2_SDO [alternate function AF5]
	GPIOC->MODER &= ~GPIO_MODER_MODE3_0;			// 00: Input (reset state)	01: General purpose output mode	10: Alternate function mode	11: Analog mode
	GPIOC->AFR[0] |= 5 << GPIO_AFRL_AFSEL3_Pos;		// Alternate Function 5 (I2S2)


	// Configure SPI (Shown as SPI2->CGFR in SFR)
	SPI2->I2SCFGR |= SPI_I2SCFGR_I2SMOD;			// I2S Mode
	SPI2->I2SCFGR |= SPI_I2SCFGR_I2SCFG_1;			// I2S configuration mode: 00=Slave transmit; 01=Slave receive; 10=Master transmit; 11=Master receive

	// Use a 16bit data length but pack into the upper 16 bits of a 32 bit word
	SPI2->I2SCFGR &= ~SPI_I2SCFGR_DATLEN;			// Data Length 00=16-bit; 01=24-bit; 10=32-bit
	SPI2->I2SCFGR |= SPI_I2SCFGR_CHLEN;				// Channel Length = 32bits

	/* I2S Clock currently 240MHz
	000: pll1_q_ck clock selected as SPI/I2S1,2 and 3 kernel clock (default after reset)
	001: pll2_p_ck clock selected as SPI/I2S1,2 and 3 kernel clock
	010: pll3_p_ck clock selected as SPI/I2S1,2 and 3 kernel clock
	011: I2S_CKIN clock selected as SPI/I2S1,2 and 3 kernel clock
	100: per_ck clock selected as SPI/I2S1,2 and 3 kernel clock
	//RCC->D2CCIP1R |= RCC_D2CCIP1R_SPI123SEL;

	I2S Prescaler Clock calculations:
	FS = I2SxCLK / [(32*2)*((2*I2SDIV)+ODD))]					Eg  240000000 / (32*2  * ((2 * 39) + 0)) = 48076.92
	*/
	SPI2->I2SCFGR |= (39 << SPI_I2SCFGR_I2SDIV_Pos);// Set I2SDIV to 39 with no Odd factor prescaler

	// Enable interrupt when TxFIFO threshold reached
	SPI2->IER |= SPI_IER_TXPIE;
	NVIC_SetPriority(SPI2_IRQn, 2);					// Lower is higher priority
	NVIC_EnableIRQ(SPI2_IRQn);
	GPIOB->MODER &= ~GPIO_MODER_MODE7_1;			// FIXME Use LED on PB7 for testing interrupt


	SPI2->CR1 |= SPI_CR1_SPE;						// Enable I2S
	SPI2->CR1 |= SPI_CR1_CSTART;					// Start I2S
}


void InitClock()
{
	// Fire interrupt when clock pulse is received
	// FIXME Production will use PA7 - temporarily using PB5 as also configured as TIM3_CH2
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;			// GPIO port clock
	GPIOB->MODER &= ~GPIO_MODER_MODE5_Msk;			// 00: Input mode
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PB;	// Select Pin PB5 which uses External interrupt 2
	EXTI->RTSR1 |= EXTI_RTSR1_TR5;					// Enable rising edge trigger
	EXTI->IMR1 |= EXTI_IMR1_IM5;					// Activate interrupt using mask register

	NVIC_SetPriority(EXTI9_5_IRQn, 4);				// Lower is higher priority
	NVIC_EnableIRQ(EXTI9_5_IRQn);

}


void InitClockTimer()
{
	// Configure timer to use Capture and Compare mode on external clock to time duration between pulses

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
