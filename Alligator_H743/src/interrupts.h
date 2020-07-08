#include "initialisation.h"
#include <algorithm>

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
		SPI2->TXDR = (int32_t)(samples[readPos]);		// Left Channel
		samples[writePos] = (int16_t)ADC_array[1] - 32767;
		samplePlayback = samples[readPos];
		if (++writePos == SAMPLE_BUFFER_LENGTH) 	writePos = 0;
		if (++readPos == SAMPLE_BUFFER_LENGTH)	readPos = 0;

		dampedDelay = std::max((31 * dampedDelay + ((int32_t)ADC_array[2] - 100)) >> 5, 0);
		newReadPos = writePos - dampedDelay;

		if (newReadPos < 0) {
			newReadPos = 65536 + newReadPos;
		}

		// Work out if new position has changed enough (hysteresis) to move read head (factoring in circular buffer size)
#define delayHysteresis 100
		if ((newReadPos > readPos && SAMPLE_BUFFER_LENGTH - newReadPos + readPos > delayHysteresis && newReadPos - readPos > delayHysteresis) ||
			(newReadPos < readPos && SAMPLE_BUFFER_LENGTH - readPos + newReadPos > delayHysteresis && readPos - newReadPos > delayHysteresis)) {

			// Debug data
			currentDelay = dampedDelay;
			nrp = newReadPos;
			rp = readPos;

			readPos = newReadPos;

		}


	} else {
		SPI2->TXDR = 32768;													// Right Channel
	}

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
