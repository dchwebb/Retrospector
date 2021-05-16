void OTG_FS_IRQHandler(void) {
	usb.USBInterruptHandler();
}

void TIM2_IRQHandler() {
	TIM2->SR &= ~TIM_SR_UIF;				// clear UIF flag

	if ((GPIOB->ODR & GPIO_ODR_OD8) == GPIO_ODR_OD8) {
		GPIOB->ODR &= ~GPIO_ODR_OD8;
	} else {
		GPIOB->ODR |= GPIO_ODR_OD8;
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
