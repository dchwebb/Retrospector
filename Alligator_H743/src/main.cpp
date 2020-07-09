#include "initialisation.h"
#include "samples.h"

volatile uint32_t SysTickVal;
extern uint32_t SystemCoreClock;

volatile uint8_t uartCmdPos = 0;
volatile char uartCmd[50];
volatile bool uartCmdRdy = false;

//int16_t samples[SAMPLE_BUFFER_LENGTH];
int32_t readPos;
int32_t writePos = 5;

int32_t newReadPos;
int32_t dampedDelay;
uint8_t delayCrossfade;
int32_t currentDelay;
int32_t lastSample;

int32_t debugVal;
int32_t debugC;
int32_t debugD;

bool sampleClock = false;

char pendingCmd[50];

volatile uint16_t ADC_array[ADC_BUFFER_LENGTH] __attribute__ ((aligned (32)));


samples Samples;

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

	dampedDelay = ADC_array[2];

	InitI2S();

	while (1) {


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

