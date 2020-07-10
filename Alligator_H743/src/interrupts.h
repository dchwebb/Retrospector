

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
}

// I2S Interrupt
void SPI2_IRQHandler() {

	sampleClock = !sampleClock;

	if (sampleClock) {
		DigitalDelay.samples[DigitalDelay.writePos] = (int16_t)ADC_array[1] - adcZeroOffset;
		SPI2->TXDR = DigitalDelay.calcSample();		// Left Channel
	} else {
		SPI2->TXDR = 32768;				// Right Channel
	}

	// FIXME - it appears we need something here to add a slight delay or the interrupt sometimes fires twice

	if ((GPIOB->ODR & GPIO_ODR_OD8) == 0)
		GPIOB->ODR |= GPIO_ODR_OD8;
	else
		GPIOB->ODR &= ~GPIO_ODR_OD8;


	nextSample = true;		// request next sample be prepared
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
