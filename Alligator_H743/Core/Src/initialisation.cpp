#include "stm32h743xx.h"
#include "initialisation.h"

#define PLL_M 2
#define PLL_N 240
#define PLL_P 1		//  0000001: pll1_p_ck = vco1_ck / 2
#define PLL_Q 4
#define PLL_R 2

void SystemClock_Config() {

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


void InitSysTick() {
	SysTick_Config(SystemCoreClock / 1000UL);		// gives 1ms
	NVIC_SetPriority(SysTick_IRQn, 0);
}


void InitUART() {
	// H743 Nucelo STLink connects virtual COM port to PD8 (USART3 TX) and PD9 (USART3 RX)

	RCC->APB1LENR |= RCC_APB1LENR_USART3EN;			// USART3 clock enable
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;			// GPIO port D

	GPIOD->MODER &= ~GPIO_MODER_MODE8_0;			// Set alternate function on PD8
	GPIOD->AFR[1] |= 7 << GPIO_AFRH_AFSEL8_Pos;		// Alternate function on PD8 for USART3_TX is AF7
	GPIOD->MODER &= ~GPIO_MODER_MODE9_0;			// Set alternate function on PA10
	GPIOD->AFR[1] |= 7 << GPIO_AFRH_AFSEL9_Pos;		// Alternate function on PD9 for USART3_RX is AF7

	// By default clock source is muxed to peripheral clock 1 which is system clock / 4 (change clock source in RCC->D2CCIP2R)
	// Calculations depended on oversampling mode set in CR1 OVER8. Default = 0: Oversampling by 16
	int USARTDIV = (SystemCoreClock / 4) / 230400;	//clk / desired_baud
	USART3->BRR |= USARTDIV & USART_BRR_DIV_MANTISSA_Msk;
	USART3->CR1 &= ~USART_CR1_M;					// 0: 1 Start bit, 8 Data bits, n Stop bit; 	1: 1 Start bit, 9 Data bits, n Stop bit
	USART3->CR1 |= USART_CR1_RE;					// Receive enable
	USART3->CR1 |= USART_CR1_TE;					// Transmitter enable

	// Set up interrupts
	USART3->CR1 |= USART_CR1_RXNEIE;
	NVIC_SetPriority(USART3_IRQn, 3);				// Lower is higher priority
	NVIC_EnableIRQ(USART3_IRQn);

	USART3->CR1 |= USART_CR1_UE;					// USART Enable
}


void uartSendChar(char c) {
	while ((USART3->ISR & USART_ISR_TXE_TXFNF) == 0);
	USART3->TDR = c;
}

void uartSendString(const std::string& s) {
	for (char c : s) {
		while ((USART3->ISR & USART_ISR_TXE_TXFNF) == 0);
		USART3->TDR = c;
	}

}
/*
// 743 - see p695
void InitADC() {
	//	Setup Timer 2 to trigger ADC
	RCC->APB1LENR |= RCC_APB1LENR_TIM2EN;			// Enable Timer 2 clock
	TIM2->CR2 |= TIM_CR2_MMS_2;						// 100: Compare - OC1REF signal is used as trigger output (TRGO)
	TIM2->PSC = 20 - 1;								// Prescaler
	TIM2->ARR = 50 - 1;								// Auto-reload register (ie reset counter) divided by 100
	TIM2->CCR1 = 50 - 1;							// Capture and compare - ie when counter hits this number PWM high
	TIM2->CCER |= TIM_CCER_CC1E;					// Capture/Compare 1 output enable
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 |TIM_CCMR1_OC1M_2;		// 110 PWM Mode 1
	TIM2->CR1 |= TIM_CR1_CEN;

	// Enable ADC1 and GPIO clock sources
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN;			// GPIO port clock
	RCC->AHB1ENR |= RCC_AHB1ENR_ADC12EN;

	// Enable ADC - PC0 ADC123_INP10
	GPIOC->MODER |= GPIO_MODER_MODER10;				// Set to Analog mode (0b11)
	ADC1->PCSEL |= ADC_PCSEL_PCSEL_10;				// ADC channel preselection register
	//GPIOA->MODER |= GPIO_MODER_MODER5;			// Set PA5 to Analog mode (0b11)
	//GPIOA->MODER |= GPIO_MODER_MODER0;			// Set PA0 to Analog mode (0b11)

	//ADC1->CR1 |= ADC_CR1_SCAN;					// Activate scan mode
	ADC1->SQR1 = (1 - 1) << 20;						// Number of conversions in sequence (set to 3, getting multiple samples for each channel to average)
	ADC1->SQR1 |= 10 << ADC_SQR1_SQ1_Pos;			// Set 1st conversion in sequence
	//ADC1->SQR3 |= 5 << 5;							// Set 2nd conversion in sequence
	//ADC1->SQR3 |= 0 << 10;							// Set 3rd conversion in sequence

	// Set to 56 cycles (0b11) sampling speed (SMPR2 Left shift speed 3 x ADC_INx up to input 9; use SMPR1 from 0 for ADC_IN10+)
	// 000: 1.5 ADC clock cycles; 001: 2.5 cycles; 010: 8.5 cycles;	011: 16.5 cycles; 100: 32.5 cycles; 101: 64.5 cycles; 110: 387.5 cycles; 111: 810.5 cycles
	ADC1->SMPR2 |= 0b110 << ADC_SMPR2_SMP10_Pos;		// Set conversion speed

	ADC1->CFGR |= ADC_CFGR_EOCS;					// The EOC bit is set at the end of each regular conversion. Overrun detection is enabled.
	ADC1->CFGR |= ADC_CFGR_EXTEN_0;					// ADC hardware trigger 00: Trigger detection disabled; 01: Trigger detection on the rising edge; 10: Trigger detection on the falling edge; 11: Trigger detection on both the rising and falling edges
	ADC1->CFGR |= ADC_CFGR_EXTSEL_0 | ADC_CFGR_EXTSEL_1 | ADC_CFGR_EXTSEL_3;	// ADC External trigger: 1011: Timer 2 TRGO

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
	ADC1->CR |= ADC_CR_ADEN;						// Activate ADC

}
*/

void InitADC() {
	//#define ADCx_CHANNEL_PIN                GPIO_PIN_6
	//#define ADCx_CHANNEL_GPIO_PORT          GPIOA

	// Enable instruction and data cache - core_cm7.h
	SCB_EnableICache();

	/* Enable D-Cache */
	SCB_EnableDCache();

	//SET_BIT(hadc->Instance->CFGR, ADC_CFGR_JQM);
	/*Clears flags in ISR register
	__HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_AWD3  | ADC_FLAG_AWD2 | ADC_FLAG_AWD1 |
								ADC_FLAG_JQOVF | ADC_FLAG_OVR  |
								ADC_FLAG_JEOS  | ADC_FLAG_JEOC |
								ADC_FLAG_EOS   | ADC_FLAG_EOC  |
								ADC_FLAG_EOSMP | ADC_FLAG_RDY));

	SET_BIT(hadc->Instance->CR, ADC_CR_DEEPPWD);
	SET_BIT(hadc->Instance->CFGR, ADC_CFGR_JQDIS);

	*/
	// Reset peripherals
	RCC->AHB1RSTR |= RCC_AHB1RSTR_ADC12RST;
	RCC->AHB1RSTR &= ~ (RCC_AHB1RSTR_ADC12RST);
	RCC->AHB1ENR &= ~ (RCC_AHB1ENR_ADC12EN);

	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_ADC12EN;
	RCC->D3CCIPR |= RCC_D3CCIPR_ADCSEL_1;		// SAR ADC kernel clock source selection
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

	/* ### - 1 - Initialize ADC peripheral #################################### */
	/*
	  DmaHandle.Instance                 = DMA1_Stream1;
	  DmaHandle.Init.Request             = DMA_REQUEST_ADC1;
	  DmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
	  DmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
	  DmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
	  DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	  DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
	  DmaHandle.Init.Mode                = DMA_CIRCULAR;
	  DmaHandle.Init.Priority            = DMA_PRIORITY_MEDIUM;
	*/
	//DMA1_Stream1->CR ??
	DMA1_Stream1->CR &= ~DMA_SxCR_EN;

	// Deinit code
	// DMA Mux Channel: DMAMUX1->C1CR (0x40020804)
	// channel status: DMAMUX1_ChannelStatus
	// DMAmuxChannelStatusMask: 1 << 1

	/* Clear the DMAMUX synchro overrun flag */
	// hdma->DMAmuxChannelStatus->CFR = hdma->DMAmuxChannelStatusMask;

	// 0001 0010 1101 0000 0000
	// DMA1->S1CR should be 0x12d00 afterwards

	DMA1_Stream1->CR |= DMA_SxCR_CIRC;
	DMA1_Stream1->CR |= DMA_SxCR_MINC;
	DMA1_Stream1->CR |= DMA_SxCR_PSIZE_0;
	DMA1_Stream1->CR |= DMA_SxCR_MSIZE_0;
	DMA1_Stream1->CR |= DMA_SxCR_PL_0;

	// Disable FIFO Threshold selection
	DMA1_Stream1->FCR &= ~DMA_SxFCR_FTH;

	//hdma->StreamIndex = 6		This is index of DMA1->LIFCR CFEIF1 - CTCIF1 - ie clear interrupts for this stream
	//hdma->StreamBaseAddress = 0x40020000 (DMA1)
	DMA1->LIFCR = 0x3FUL << 6;		// clear interrupts for this stream
	//DMAMUX1->C1CR |= 0x9; 			// DMA request MUX input 9 = adc1_dma (See p.695)
	DMAMUX1_Channel1->CCR |= 0x9; 			// DMA request MUX input 9 = adc1_dma (See p.695)

	/* Clear the DMAMUX synchro overrun flag */
	//DMAMUX1->CFR = 1 << 1;
	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF1; // Channel 1 Clear synchronization overrun event flag

	/* Associate the DMA handle */
	//__HAL_LINKDMA(hadc, DMA_Handle, DmaHandle);

	NVIC_SetPriority(DMA1_Stream1_IRQn, 1);
	NVIC_EnableIRQ(DMA1_Stream1_IRQn);

	ADC1->CR &= ~ADC_CR_DEEPPWD;

	/* Enable ADC internal voltage regulator */
	ADC1->CR |= ADC_CR_ADVREGEN;

	/* Note: Variable divided by 2 to compensate partially              */
	/*       CPU processing cycles, scaling in us split to not          */
	/*       exceed 32 bits register capacity and handle low frequency. */

	volatile uint32_t wait_loop_index = (SystemCoreClock / (100000UL * 2UL));
	while (wait_loop_index != 0UL)
	{
	  wait_loop_index--;
	}

	while ((ADC1->CR & ADC_CR_ADVREGEN) != ADC_CR_ADVREGEN) {}

	//LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(hadc->Instance), hadc->Init.ClockPrescaler);
	ADC12_COMMON->CCR |= ADC_CCR_PRESC_0;
	ADC1->CFGR |= ADC_CFGR_CONT;
	ADC1->CFGR |= ADC_CFGR_OVRMOD;

	// ADC1->CFGR should be 0x80003000
	ADC1->CFGR |= ADC_CFGR_DMNGT;				// Data Management configuration 11: DMA Circular Mode selected
	//0x80003003

	// oversampling set here?: ADC1->CFGR2 |= ADC_CFGR2_ROVSE

	// ADC_CCR_PRESC_Pos  =18
	//freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_ADC);	// Gives 64,000,000 (HSI Clock)
	//freq /= (1 << 1UL);		// Gives 32,000,000
	// Boost mode 1: Boost mode on. Must be used when ADC clock > 20 MHz.
	ADC1->CR |= ADC_CR_BOOST_1;		// Note this is not legal according to SFR - HAL has notes about silicon revision

	// For scan mode:
	// MODIFY_REG(hadc->Instance->SQR1, ADC_SQR1_L, (hadc->Init.NbrOfConversion - (uint8_t)1));

	/* ### - 2 - Start calibration ############################################ */
	ADC1->CR |= ADC_CR_ADCAL;

	// Wait until calibration has finished
	while ((ADC1->CR & ADC_CR_ADCAL) == ADC_CR_ADCAL) {};

	/*
	sConfig.Channel      = ADCx_CHANNEL;                /* Sampled channel number = 3 (ADC12_INP3)
	sConfig.Rank         = ADC_REGULAR_RANK_1;          /* Rank of sampled channel number ADCx_CHANNEL
	sConfig.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;   /* Sampling time (number of clock cycles unit)
	sConfig.SingleDiff   = ADC_SINGLE_ENDED;            /* Single-ended input channel
	sConfig.OffsetNumber = ADC_OFFSET_NONE;             /* No offset subtraction
	sConfig.Offset = 0;                                 /* Parameter discarded because offset correction is disabled
	*/
	ADC1->PCSEL |= ADC_PCSEL_PCSEL_3;
	ADC1->SQR1 |= 3 << ADC_SQR1_SQ1_Pos;			// Set 1st conversion in sequence

	// 000: 1.5 ADC clock cycles; 001: 2.5 cycles; 010: 8.5 cycles;	011: 16.5 cycles; 100: 32.5 cycles; 101: 64.5 cycles; 110: 387.5 cycles; 111: 810.5 cycles
	ADC1->SMPR1 |= 0b010 << ADC_SMPR1_SMP3_Pos;		// Set conversion speed

/*
	ALIGN_32BYTES (static uint16_t   aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]);
	#define ALIGN_32BYTES(buf)  buf __attribute__ ((aligned (32)))
	#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  32)    Size of array aADCxConvertedData[]
*/

	/* ### - 4 - Start conversion in DMA mode ################################# */
	// HAL_ADC_Start_DMA(&AdcHandle, (uint32_t *)aADCxConvertedData, ADC_CONVERTED_DATA_BUFFER_SIZE)
	ADC1->CR |= ADC_CR_ADEN;

	// Wait til ready
	while (ADC1->ISR & ADC_ISR_ADRDY == 0) {}

	/*
	 *         /* Set the DMA transfer complete callback
			hadc->DMA_Handle->XferCpltCallback = ADC_DMAConvCplt;

			/* Set the DMA half transfer complete callback
			hadc->DMA_Handle->XferHalfCpltCallback = ADC_DMAHalfConvCplt;

			/* Set the DMA error callback
			hadc->DMA_Handle->XferErrorCallback = ADC_DMAError;
	 */

	/* With DMA, overrun event is always considered as an error even if
	   hadc->Init.Overrun is set to ADC_OVR_DATA_OVERWRITTEN. Therefore,
	   ADC_IT_OVR is enabled. */
	ADC1->IER |= ADC_IER_OVRIE;

	//  MODIFY_REG(ADCx->CFGR, ADC_CFGR_DMNGT, DataTransferMode);		// Already set to 3??

	/* Start the DMA channel */
	// tmp_hal_status = HAL_DMA_Start_IT(hadc->DMA_Handle, (uint32_t)&hadc->Instance->DR, (uint32_t)pData, Length);

	/* Clear the DMAMUX synchro overrun flag */
//	DMAMUX1->CFR = 1 << 1;
	DMAMUX1_ChannelStatus->CFR |= DMAMUX_CFR_CSOF1; // Channel 1 Clear synchronization overrun event flag

	DMA1->LIFCR = 0x3FUL << 6;		// clear interrupts for this stream

	/* Clear DBM bit */
	//((DMA_Stream_TypeDef *)hdma->Instance)->CR &= (uint32_t)(~DMA_SxCR_DBM);

	DMA1_Stream1->NDTR |= ADC_BUFFER_LENGTH;		// Number of data items to transfer (ie size of ADC buffer)
	DMA1_Stream1->PAR = (uint32_t)(&(ADC1->DR));	// Configure the peripheral data register address 0x40022040
	DMA1_Stream1->M0AR = (uint32_t)(ADC_array);		// Configure the memory address (note that M1AR is used for double-buffer mode) 0x24000040

	DMA1_Stream1->CR |= DMA_SxCR_DMEIE;		// Direct mode error interrupt enable
	DMA1_Stream1->CR |= DMA_SxCR_TEIE;		// Transfer error interrupt enable
	DMA1_Stream1->CR |= DMA_SxCR_TCIE;		// Transfer complete interrupt enable

	// S1CR should be 0x12d16
	DMA1_Stream1->CR |= DMA_SxCR_HTIE;		// Half transfer interrupt enable
	//0x12d1e

	DMA1_Stream1->CR |= DMA_SxCR_EN;
	//0x12d1f

	ADC1->CR |= ADC_CR_ADSTART;
}
