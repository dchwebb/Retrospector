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

	//if (calculatingFilter)
		GPIOC->ODR |= GPIO_ODR_OD12;			// Toggle LED for debugging

	sampleClock = !sampleClock;

	if (sampleClock) {
		DigitalDelay.calcSample(left);			// Left Channel
	} else {
		DigitalDelay.calcSample(right);			// Right Channel
	}

	// FIXME - it appears we need something here to add a slight delay or the interrupt sometimes fires twice
	TinyDelay();

	GPIOC->ODR &= ~GPIO_ODR_OD12;
}


void EXTI9_5_IRQHandler(void) {

	// Handle incoming clock pulse
	if (EXTI->PR1 & EXTI_PR1_PR7) {
		clockInterval = SysTickVal - lastClock;
		lastClock = SysTickVal;
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
