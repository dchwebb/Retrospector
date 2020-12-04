#include "stm32f429xx.h"
#include "initialisation.h"

#define PLL_M 8
#define PLL_N 360	// Change 360 to 336 for 168MHz
#define PLL_P 2		// Main PLL (PLL) division factor for main system clock can be 2 (PLL_P = 0), 4 (PLL_P = 1), 6 (PLL_P = 2), 8 (PLL_P = 3)
#define PLL_Q 7

void SystemClock_Config() {

	RCC->APB1ENR |= RCC_APB1ENR_PWREN;			// Enable Power Control clock
	PWR->CR |= PWR_CR_VOS_0;					// Enable VOS voltage scaling - allows maximum clock speed

	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));// CPACR register: set full access privileges for coprocessors

	RCC->CR |= RCC_CR_HSEON;					// HSE ON
	while ((RCC->CR & RCC_CR_HSERDY) == 0);		// Wait till HSE is ready
	RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) | (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);

	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;			// HCLK = SYSCLK / 1
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;			// PCLK2 = HCLK / 2
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;			// PCLK1 = HCLK / 4
	RCC->CR |= RCC_CR_PLLON;					// Enable the main PLL
	while((RCC->CR & RCC_CR_PLLRDY) == 0);		// Wait till the main PLL is ready

	// Configure Flash prefetch, Instruction cache, Data cache and wait state
	FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;

	// Select the main PLL as system clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	// Wait till the main PLL is used as system clock source
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

}



void InitSysTick()
{
	SysTick_Config(SystemCoreClock / 1000UL);		// gives 1ms
	NVIC_SetPriority(SysTick_IRQn, 0);
}


void InitDAC()
{
	// DAC1_OUT2 on PA5
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;			// GPIO port clock
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;			// Enable DAC Clock

	DAC->CR |= DAC_CR_EN1;							// Enable DAC using PA4 (DAC_OUT1)
	DAC->CR |= DAC_CR_BOFF1;						// Enable DAC channel output buffer to reduce the output impedance
	DAC->CR |= DAC_CR_TEN1;							// DAC 1 enable trigger
	DAC->CR |= DAC_CR_TSEL1;						// Set trigger to software (0b111: Software trigger)

	DAC->CR |= DAC_CR_EN2;							// Enable DAC using PA5 (DAC_OUT2)
	DAC->CR |= DAC_CR_BOFF2;						// Enable DAC channel output buffer
	DAC->CR |= DAC_CR_TEN2;							// DAC 2 enable trigger
	DAC->CR |= DAC_CR_TSEL2;						// Set trigger to software (0b111: Software trigger)
}

#ifdef h473
void InitADC()
{
	// Configure clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;			// GPIO port clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

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

	// NB reset mode of GPIO pins is 0b11 = analog mode so shouldn't need to change
	ADC1->PCSEL |= ADC_PCSEL_PCSEL_14;				// ADC channel preselection register
	ADC1->PCSEL |= ADC_PCSEL_PCSEL_15;				// ADC channel preselection register
	ADC1->PCSEL |= ADC_PCSEL_PCSEL_8;				// ADC channel preselection register
	ADC1->PCSEL |= ADC_PCSEL_PCSEL_5;				// ADC channel preselection register
	ADC1->PCSEL |= ADC_PCSEL_PCSEL_17;				// ADC channel preselection register
	ADC1->PCSEL |= ADC_PCSEL_PCSEL_16;				// ADC channel preselection register
	ADC1->PCSEL |= ADC_PCSEL_PCSEL_3;				// ADC channel preselection register
	ADC1->PCSEL |= ADC_PCSEL_PCSEL_9;				// ADC channel preselection register
	ADC1->PCSEL |= ADC_PCSEL_PCSEL_4;				// ADC channel preselection register
	ADC1->PCSEL |= ADC_PCSEL_PCSEL_11;				// ADC channel preselection register

	ADC1->SQR1 |= 14 << ADC_SQR1_SQ1_Pos;			// Set 1st conversion in sequence
	ADC1->SQR1 |= 15 << ADC_SQR1_SQ2_Pos;			// Set 2nd conversion in sequence
	ADC1->SQR1 |= 8  << ADC_SQR1_SQ3_Pos;			// Set 3rd conversion in sequence
	ADC1->SQR1 |= 5  << ADC_SQR1_SQ4_Pos;			// Set 4th conversion in sequence
	ADC1->SQR2 |= 17 << ADC_SQR2_SQ5_Pos;			// Set 5th conversion in sequence
	ADC1->SQR2 |= 16 << ADC_SQR2_SQ6_Pos;			// Set 6th conversion in sequence
	ADC1->SQR2 |= 3  << ADC_SQR2_SQ7_Pos;			// Set 7th conversion in sequence
	ADC1->SQR2 |= 9  << ADC_SQR2_SQ8_Pos;			// Set 8th conversion in sequence
	ADC1->SQR2 |= 4  << ADC_SQR2_SQ9_Pos;			// Set 9th conversion in sequence
	ADC1->SQR3 |= 11 << ADC_SQR3_SQ10_Pos;			// Set 10th conversion in sequence

	// 000: 1.5 ADC clock cycles; 001: 2.5 cycles; 010: 8.5 cycles;	011: 16.5 cycles; 100: 32.5 cycles; 101: 64.5 cycles; 110: 387.5 cycles; 111: 810.5 cycles
	ADC1->SMPR2 |= 0b010 << ADC_SMPR2_SMP14_Pos;	// Set conversion speed
	ADC1->SMPR2 |= 0b010 << ADC_SMPR2_SMP15_Pos;	// Set conversion speed
	ADC1->SMPR1 |= 0b010 << ADC_SMPR1_SMP8_Pos;		// Set conversion speed
	ADC1->SMPR1 |= 0b010 << ADC_SMPR1_SMP5_Pos;		// Set conversion speed
	ADC1->SMPR2 |= 0b010 << ADC_SMPR2_SMP17_Pos;	// Set conversion speed
	ADC1->SMPR2 |= 0b010 << ADC_SMPR2_SMP16_Pos;	// Set conversion speed
	ADC1->SMPR1 |= 0b010 << ADC_SMPR1_SMP3_Pos;		// Set conversion speed
	ADC1->SMPR1 |= 0b010 << ADC_SMPR1_SMP9_Pos;		// Set conversion speed
	ADC1->SMPR1 |= 0b010 << ADC_SMPR1_SMP4_Pos;		// Set conversion speed
	ADC1->SMPR2 |= 0b010 << ADC_SMPR2_SMP11_Pos;	// Set conversion speed

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
#endif

void InitAdcPin(GPIO_TypeDef* bank, uint8_t pin, uint8_t channel) {

	// Set conversion sequence to order ADC channels are passed to this function
	static uint8_t sequence = 0;
	if (sequence < 6) {
		ADC1->SQR3 |= channel << (sequence * 5);
	} else if (sequence < 12) {
		ADC1->SQR2 |= channel << ((sequence - 6) * 5);
	} else {
		ADC1->SQR1 |= channel << ((sequence - 12) * 5);
	}

	// Pin mode to analog
	bank->MODER |= (0b11 << (2 * pin));

	// Set to 56 cycles (0b11) sampling speed (SMPR2 Left shift speed 3 x ADC_INx up to input 9; use SMPR1 from 0 for ADC_IN10+)
	// 000: 3 cycles; 001: 15 cycles; 010: 28 cycles; 011: 56 cycles; 100: 84 cycles; 101: 112 cycles; 110: 144 cycles; 111: 480 cycles
	if (channel < 10)
		ADC1->SMPR2 |= 0b101 << (3 * channel);
	else
		ADC1->SMPR1 |= 0b101 << (3 * (channel - 10));

	sequence++;
}

void InitADC(void) {
	//	Setup Timer 2 to trigger ADC
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;				// Enable Timer 2 clock
	TIM2->CR2 |= TIM_CR2_MMS_2;						// 100: Compare - OC1REF signal is used as trigger output (TRGO)
	TIM2->PSC = 20 - 1;								// Prescaler
	TIM2->ARR = 50 - 1;								// Auto-reload register (ie reset counter) divided by 100
	TIM2->CCR1 = 50 - 1;							// Capture and compare - ie when counter hits this number PWM high
	TIM2->CCER |= TIM_CCER_CC1E;					// Capture/Compare 1 output enable
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 |TIM_CCMR1_OC1M_2;		// 110 PWM Mode 1
	TIM2->CR1 |= TIM_CR1_CEN;

	// Enable ADC1 and GPIO clock sources
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;


	ADC1->CR1 |= ADC_CR1_SCAN;						// Activate scan mode
	ADC1->SQR1 = (ADC_BUFFER_LENGTH - 1) << ADC_SQR1_L_Pos;	// Number of conversions in sequence (set to 3, getting multiple samples for each channel to average)

	/*--------------------------------------------------------------------------------------------
	Configure ADC Channels to be converted:
		0	PA2 ADC12_INP2		AUDIO_IN_L
		1	PA3 ADC12_INP3 		AUDIO_IN_R
		2	PC5 ADC12_INP15		WET_DRY_MIX
		3	PB1 ADC12_INP9		DELAY_POT_L
		4	PA1 ADC1_INP1		DELAY_POT_R
		5	PA0 ADC1_INP0		DELAY_CV_SCALED_L
		6	PA6 ADC12_INP6 		DELAY_CV_SCALED_R
		7	PB0 ADC12_INP8		FEEDBACK_POT
		8	PC4 ADC12_INP14		FEEDBACK_CV_SCALED
		9	PC1 ADC123_INP11 	TONE_POT
	*/
	InitAdcPin(GPIOA, 2, 2);
	InitAdcPin(GPIOA, 3, 3);
	InitAdcPin(GPIOC, 5, 15);
	InitAdcPin(GPIOB, 1, 9);
	InitAdcPin(GPIOA, 1, 1);
	InitAdcPin(GPIOA, 0, 0);
	InitAdcPin(GPIOA, 6, 6);
	InitAdcPin(GPIOB, 0, 8);
	InitAdcPin(GPIOC, 4, 14);
	InitAdcPin(GPIOC, 1, 11);

	ADC1->CR2 |= ADC_CR2_EOCS;						// The EOC bit is set at the end of each regular conversion. Overrun detection is enabled.
	ADC1->CR2 |= ADC_CR2_EXTEN_0;					// ADC hardware trigger 00: Trigger detection disabled; 01: Trigger detection on the rising edge; 10: Trigger detection on the falling edge; 11: Trigger detection on both the rising and falling edges
	ADC1->CR2 |= ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2;	// ADC External trigger: 0110 = TIM2_TRGO event

	// Enable DMA - DMA2, Channel 0, Stream 0  = ADC1 (Manual p207)
	ADC1->CR2 |= ADC_CR2_DMA;						// Enable DMA Mode on ADC1
	ADC1->CR2 |= ADC_CR2_DDS;						// DMA requests are issued as long as data are converted and DMA=1
	RCC->AHB1ENR|= RCC_AHB1ENR_DMA2EN;

	DMA2_Stream4->CR &= ~DMA_SxCR_DIR;				// 00 = Peripheral-to-memory
	DMA2_Stream4->CR |= DMA_SxCR_PL_1;				// Priority: 00 = low; 01 = Medium; 10 = High; 11 = Very High
	DMA2_Stream4->CR |= DMA_SxCR_PSIZE_0;			// Peripheral size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA2_Stream4->CR |= DMA_SxCR_MSIZE_0;			// Memory size: 8 bit; 01 = 16 bit; 10 = 32 bit
	DMA2_Stream4->CR &= ~DMA_SxCR_PINC;				// Peripheral not in increment mode
	DMA2_Stream4->CR |= DMA_SxCR_MINC;				// Memory in increment mode
	DMA2_Stream4->CR |= DMA_SxCR_CIRC;				// circular mode to keep refilling buffer
	DMA2_Stream4->CR &= ~DMA_SxCR_DIR;				// data transfer direction: 00: peripheral-to-memory; 01: memory-to-peripheral; 10: memory-to-memory

	DMA2_Stream4->NDTR |= ADC_BUFFER_LENGTH;		// Number of data items to transfer (ie size of ADC buffer)
	DMA2_Stream4->PAR = (uint32_t)(&(ADC1->DR));	// Configure the peripheral data register address
	DMA2_Stream4->M0AR = (uint32_t)(ADC_array);		// Configure the memory address (note that M1AR is used for double-buffer mode)
	DMA2_Stream4->CR &= ~DMA_SxCR_CHSEL;			// channel select to 0 for ADC1

	DMA2_Stream4->CR |= DMA_SxCR_EN;				// Enable DMA2
	ADC1->CR2 |= ADC_CR2_ADON;						// Activate ADC

}

void InitI2S()
{
	//	Enable GPIO and SPI clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;			// reset and clock control - advanced high performance bus - GPIO port B
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

	//	Enable GPIO and SPI clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;			// GPIO port clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;			// GPIO port clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;			// GPIO port clock
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

	// PB9: I2S2_WS [alternate function AF5]
	GPIOB->MODER |= GPIO_MODER_MODE9_1;				// 10: Alternate function mode
	GPIOB->AFR[1] |= 5 << GPIO_AFRH_AFSEL9_Pos;		// Alternate Function 5 (I2S2)

	// PD3 I2S2_CK [alternate function AF5]
	GPIOD->MODER |= GPIO_MODER_MODE3_1;				// 00: Input (reset state)	01: General purpose output mode	10: Alternate function mode	11: Analog mode
	GPIOD->AFR[0] |= 5 << GPIO_AFRL_AFSEL3_Pos;		// Alternate Function 5 (I2S2)

	// PC3 I2S2_SDO [alternate function AF5]
	GPIOC->MODER |= GPIO_MODER_MODE3_1;				// 00: Input (reset state)	01: General purpose output mode	10: Alternate function mode	11: Analog mode
	GPIOC->AFR[0] |= 5 << GPIO_AFRL_AFSEL3_Pos;		// Alternate Function 5 (I2S2)

	// Configure SPI
	SPI2->I2SCFGR |= SPI_I2SCFGR_I2SMOD;			// I2S Mode
	SPI2->I2SCFGR |= SPI_I2SCFGR_I2SCFG_1;			// I2S configuration mode: 00=Slave transmit; 01=Slave receive; 10=Master transmit; 11=Master receive
//	SPI2->I2SCFGR |= SPI2_I2SCFGR_I2SSTD;			// I2S standard selection:	00=Philips; 01=MSB justified; 10=LSB justified; 11=PCM
//	SPI2->I2SCFGR |= SPI_I2SCFGR_DATLEN_1;			// Data Length 00=16-bit; 01=24-bit; 10=32-bit
	SPI2->I2SCFGR |= SPI_I2SCFGR_CHLEN;				// Channel Length = 32bits

	/* RCC Clock calculations:
	f[VCO clock] = f[PLLI2S clock input] x (PLLI2SN / PLLM)		Eg (8MHz osc) * (192 / 4) = 384MHz
	f[PLL I2S clock output] = f[VCO clock] / PLLI2SR			Eg 384 / 5 = 76.8MHz

	RCC->DCKCFGR
	00: I2S2 clock frequency = f(PLLI2S_R);
	01: I2S2 clock frequency = I2S_CKIN Alternate function input frequency
	10: I2S2 clock frequency = f(PLL_R)
	11: I2S2 clock frequency = HSI/HSE depends on PLLSRC bit (PLLCFGR[22])

	RCC->PLLI2SCFGR
	Try: M = 8; N = 154; R = 2		ie 8Mz / 8 * 154 / 2 = 77MHz
	*/
	RCC->CFGR &= ~RCC_CFGR_I2SSRC;					// Set I2S PLL source to internal
	RCC->PLLI2SCFGR = (RCC_PLLI2SCFGR_PLLI2SN_Msk & (77 << RCC_PLLI2SCFGR_PLLI2SN_Pos)) | (RCC_PLLI2SCFGR_PLLI2SR_Msk & (2 << RCC_PLLI2SCFGR_PLLI2SR_Pos));
	RCC->CR |= RCC_CR_PLLI2SON;

	/* I2S Prescaler Clock calculations:
	FS = I2SxCLK / [(32*2)*((2*I2SDIV)+ODD))]					Eg  77 / (64 * ((2 * 12) + 1)) = 48,125Hz
	NB This tests out by a factor of 2 so divide 12 by 2 = 6
	*/
	SPI2->I2SPR = (SPI_I2SPR_I2SDIV_Msk & 6) | SPI_I2SPR_ODD;		// Set Linear prescaler to 12 and enable Odd factor prescaler

	// Enable interrupt when TxFIFO threshold reached
	SPI2->CR2 |= SPI_CR2_TXEIE;
	NVIC_SetPriority(SPI2_IRQn, 2);					// Lower is higher priority
	NVIC_EnableIRQ(SPI2_IRQn);


	SPI2->I2SCFGR |= SPI_I2SCFGR_I2SE;				// Enable I2S
}

void sendI2SData(uint32_t data) {
	//while (((SPI2->SR & SPI_SR_TXE) == 0) | ((SPI2->SR & SPI_SR_BSY) == SPI_SR_BSY) );
	while ((SPI2->SR & SPI_SR_TXE) == 0);
	SPI2->DR = 0x5533;
	while ((SPI2->SR & SPI_SR_TXE) == 0);
	SPI2->DR = 0xAABB;
	while ((SPI2->SR & SPI_SR_TXE) == 0);
	SPI2->DR = 0x5533;
	while ((SPI2->SR & SPI_SR_TXE) == 0);
	SPI2->DR = 0xAABB;

}


void InitTempoClock()
{
	// Fire interrupt when clock pulse is received on PA7 - See manual p383
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;			// GPIO port clock
	GPIOA->MODER &= ~GPIO_MODER_MODE7_Msk;			// 00: Input mode
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI7_PA;	// Select Pin PA7 which uses External interrupt 2
	EXTI->RTSR |= EXTI_RTSR_TR7;					// Enable rising edge trigger
	EXTI->IMR |= EXTI_IMR_IM7;					// Activate interrupt using mask register

	NVIC_SetPriority(EXTI9_5_IRQn, 4);				// Lower is higher priority
	NVIC_EnableIRQ(EXTI9_5_IRQn);
}


void InitClockTimer()
{
	// Configure timer to use Capture and Compare mode on external clock to time duration between pulses (not using)

	// FIXME Production will use PA7 - temporarily using PB5 as also configured as TIM3_CH2
	// See manual page 1670 for info on Capture and Compare Input mode
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;			// GPIO port clock
	GPIOB->MODER &= ~GPIO_MODER_MODE5_0;			// Alternate function is Mode 0b10 (defaults to 0b11)
	GPIOB->AFR[0] |= 2 << GPIO_AFRL_AFSEL5_Pos;		// Alternate Function 2 for TIM3_CH2 on PB5 and PA7
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
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
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;			// GPIO port clock
	GPIOC->MODER |= GPIO_MODER_MODE10_0;			// 00: Input (reset state)	01: General purpose output mode	10: Alternate function mode	11: Analog mode
	GPIOC->MODER |= GPIO_MODER_MODE11_0;			// 00: Input (reset state)	01: General purpose output mode	10: Alternate function mode	11: Analog mode
}
