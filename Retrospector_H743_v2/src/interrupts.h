void OTG_FS_IRQHandler(void) {
	usb.USBInterruptHandler();
}

void __attribute__((optimize("O0"))) TinyDelay() {
	for (int x = 0; x < 2; ++x);
}

// I2S Interrupt
void SPI2_IRQHandler() {

	GPIOB->ODR |= GPIO_ODR_OD7;		// Toggle red for debugging
	if (calculatingFilter) {
		//GPIOB->ODR |= GPIO_ODR_OD7;
	}

	delay.CalcSample();

	// FIXME - it appears we need something here to add a slight delay or the interrupt sometimes fires twice
	TinyDelay();

	GPIOB->ODR &= ~GPIO_ODR_OD7;  	// Clear red to show calc sample has finished
	GPIOB->ODR |= GPIO_ODR_OD8;		// Activate blue to show audio ADC started
	ADC1->CR |= ADC_CR_ADSTART;
	DMA1->LIFCR |= DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1;
	DMA1_Stream1->CR |= DMA_SxCR_EN;
}

// Debug tests for EOC timing
void ADC_IRQHandler()
{
	ADC1->ISR |= ADC_ISR_EOS;
	GPIOB->ODR &= ~GPIO_ODR_OD8;
//
//	static bool adcDebug = false;
//	adcDebug = !adcDebug;
//	if (adcDebug) {
//		GPIOB->ODR |= GPIO_ODR_OD8;		// Toggle blue for debugging
//	} else {
//		GPIOB->ODR &= ~GPIO_ODR_OD8;
//	}
}

// Bootloader timer
void TIM2_IRQHandler() {
	TIM2->SR &= ~TIM_SR_UIF;				// clear Update Interrupt Flag
	bootloader.GetSample();
}


// System interrupts
void NMI_Handler(void) {}

void HardFault_Handler(void) {
	while (1) {}
}
void MemManage_Handler(void) {
	while (1) {}
}
void BusFault_Handler(void) {
	while (1) {}
}
void UsageFault_Handler(void) {
	while (1) {}
}

void SVC_Handler(void) {}

void DebugMon_Handler(void) {}

void PendSV_Handler(void) {}

void SysTick_Handler(void) {
	++SysTickVal;
}

/*
// USART Decoder
void USART3_IRQHandler() {
	//if ((USART3->ISR & USART_ISR_RXNE_RXFNE) != 0 && !uartCmdRdy) {
	if (!uartCmdRdy) {
		uartCmd[uartCmdPos] = USART3->RDR; 				// accessing RDR automatically resets the receive flag
		if (uartCmd[uartCmdPos] == 10) {
			uartCmdRdy = true;
			uartCmdPos = 0;
		} else {
			uartCmdPos++;
		}
	}
}*/
