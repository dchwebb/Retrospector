void OTG_FS_IRQHandler(void) {
	usb.USBInterruptHandler();
}

/*// USART Decoder
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


void __attribute__((optimize("O0"))) TinyDelay() {
	for (int x = 0; x < 2; ++x);
}
// I2S Interrupt
void SPI2_IRQHandler() {
	/*
	 * 1.4uS constant delay rate; 1.77uS longest with crossfade
	 * 1.47uS constant sdram; 2.14uS crossfade sdram
	 * Opt -O1; sdram, constant DR: 1.06uS; variable DR: 1.53uS
	 * Opt -O0: sdram, constant DR: 1.64uS; variable DR: 2.35uS
	 *
	 */
	GPIOC->ODR |= GPIO_ODR_OD11;			// Toggle LED for testing

	sampleClock = !sampleClock;

	if (sampleClock) {
		//samples[digitalDelay::channelL][DigitalDelay.writePos[ADC_Audio_L]] = (int16_t)ADC_audio[ADC_Audio_L] - adcZeroOffset;
		SPI2->TXDR = DigitalDelay.calcSample(digitalDelay::channelL, ADC_audio[ADC_Audio_L]);		// Left Channel
	} else {
		//samples[digitalDelay::channelR][DigitalDelay.writePos[ADC_Audio_R]] = (int16_t)ADC_audio[ADC_Audio_R] - adcZeroOffset;
		SPI2->TXDR = DigitalDelay.calcSample(digitalDelay::channelR, ADC_audio[ADC_Audio_R]);		// Left Channel
	}

	// FIXME - it appears we need something here to add a slight delay or the interrupt sometimes fires twice
	TinyDelay();

	nextSample = true;		// request next sample be prepared

	GPIOC->ODR &= ~GPIO_ODR_OD11;

}


void EXTI9_5_IRQHandler(void) {

	// Handle incoming clock pulse
	if (EXTI->PR1 & EXTI_PR1_PR7) {
		clockInterval = SysTickVal - lastClock;
		lastClock = SysTickVal;
		newClock = true;
		EXTI->PR1 |= EXTI_PR1_PR7;							// Clear interrupt pending
	}


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
