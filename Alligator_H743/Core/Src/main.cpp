#include "initialisation.h"

volatile uint32_t SysTickVal;
extern uint32_t SystemCoreClock;

volatile uint8_t uartCmdPos = 0;
volatile char uartCmd[50];
volatile bool uartCmdRdy = false;
std::string pendingCmd;

volatile int dummy = 0;

extern "C" {
#include "interrupts.h"
}

int main(void) {
	//NVIC_SetPriorityGrouping(3);

	SystemClock_Config();					// Configure the clock and PLL
	SystemCoreClockUpdate();				// Update SystemCoreClock (system clock frequency)
	InitSysTick();
	InitUART();

	while (1) {
		USART3->ICR |= USART_ICR_TCCF;
		//USART3->ICR |= USART_ICR_FECF;

		if (dummy == 1) {
			uartSendChar('a');
		}
		for (int x; x < 1000000; ++x);
	}
}
