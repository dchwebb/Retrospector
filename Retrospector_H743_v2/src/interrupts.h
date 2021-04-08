void OTG_FS_IRQHandler(void) {
	usb.USBInterruptHandler();
}

void __attribute__((optimize("O0"))) TinyDelay() {
	for (int x = 0; x < 2; ++x);
}

// I2S Interrupt
void SPI2_IRQHandler() {

	//if (calculatingFilter)
		//GPIOB->ODR |= GPIO_ODR_OD8;		// Toggle LED for debugging

	delay.CalcSample();

	// FIXME - it appears we need something here to add a slight delay or the interrupt sometimes fires twice
	TinyDelay();

	//GPIOB->ODR &= ~GPIO_ODR_OD8;
}


// Handle incoming clock pulse
//void EXTI9_5_IRQHandler(void) {
//	if (EXTI->PR1 & EXTI_PR1_PR7) {
//		clockInterval = SysTickVal - lastClock;
//		lastClock = SysTickVal;
//		EXTI->PR1 |= EXTI_PR1_PR7;			// Clear interrupt pending
//	}
//}

// Bootloader timer
void TIM2_IRQHandler() {
	TIM2->SR &= ~TIM_SR_UIF;				// clear Update Interrupt Flag

	bootloader.GetSample();

}

//void I2C1_EV_IRQHandler() {
////	I2C1->TXIS
//	volatile int txint = 0;
//	txint++;
//}

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
