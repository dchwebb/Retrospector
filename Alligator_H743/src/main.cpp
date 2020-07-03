#include "initialisation.h"

volatile uint32_t SysTickVal;
extern uint32_t SystemCoreClock;

volatile uint8_t uartCmdPos = 0;
volatile char uartCmd[50];
volatile bool uartCmdRdy = false;
char pendingCmd[50];

volatile uint16_t ADC_array[ADC_BUFFER_LENGTH] __attribute__ ((aligned (32)));

int16_t i2sInc = -10;
int32_t i2sOut = 0;

extern "C" {
#include "interrupts.h"
}

int main(void) {
	//NVIC_SetPriorityGrouping(3);

	SystemClock_Config();					// Configure the clock and PLL
	SystemCoreClockUpdate();				// Update SystemCoreClock (system clock frequency)
	InitSysTick();
	InitUART();
	InitADC();
	InitI2S();

	while (1) {

		if ((SPI2->SR & SPI_SR_TXP) == SPI_SR_TXP) {		// TXP: Tx-packet space available
			i2sOut += i2sInc;
/*
			if (i2sOut > 32000 || i2sOut < -32000) {
				inc *= -1;
			}
*/
			if (i2sOut < -32000) {
				i2sOut = 32000;
			}
			SPI2->TXDR = i2sOut;
		}

		// Check if a UART command has been received and copy to pending command
		if (uartCmdRdy) {
			for (uint8_t c = 0; c < 50; ++c) {
				if (uartCmd[c] == 10) {
					pendingCmd[c] = 0;
					break;
				}
				else
					pendingCmd[c] = uartCmd[c];
			}
			uartCmdRdy = false;
		}

		if (pendingCmd[0]) {
			uartSendString("Received: ");
			uartSendString(pendingCmd);
			uartSendString("\n");
			pendingCmd[0] = 0;
		}

	}
}
