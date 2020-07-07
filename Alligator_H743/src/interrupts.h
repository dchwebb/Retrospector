#include "initialisation.h"

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
	if (sampleClock)
		SPI2->TXDR = (int32_t)(1.45f * (float)(ADC_array[1] - 32768));		// Left Channel
	else
		SPI2->TXDR = 32768;													// Right Channel

	if ((GPIOB->ODR & GPIO_ODR_OD7) == 0)
		GPIOB->ODR |= GPIO_ODR_OD7;
	else
		GPIOB->ODR &= ~GPIO_ODR_OD7;
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
