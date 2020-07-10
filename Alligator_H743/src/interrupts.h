

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
		Samples.samples[writePos] = (int16_t)ADC_array[1] - 32767;

		// Cross fade if moving playback position
		int32_t playSample = Samples.samples[readPos];
		if (delayCrossfade > 0) {		// && SysTickVal > 2000
			int32_t diff = playSample - lastSample;
			if (std::abs(diff) > 500) {
				playSample = lastSample + (diff) / Samples.crossFade;
				--delayCrossfade;
			} else {
				delayCrossfade = 0;
			}
		}
		SPI2->TXDR = playSample;		// Left Channel
		lastSample = playSample;

		if (++writePos == SAMPLE_BUFFER_LENGTH) 	writePos = 0;
		if (++readPos == SAMPLE_BUFFER_LENGTH)		readPos = 0;

		dampedDelay = std::max((31 * dampedDelay + ((int32_t)ADC_array[2] - 150)) >> 5, 0L);

		if (std::abs(dampedDelay - currentDelay) > Samples.delayHysteresis) {
			delayChanged = 1000;
			currentDelay = dampedDelay;
		}

		if (delayChanged > 0) {
			if (delayChanged == 1) {
				delayCrossfade = Samples.crossFade;
				readPos = writePos - dampedDelay;
				if (readPos < 0) {
					readPos = 65536 + readPos;
				}
			}
			--delayChanged;
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
