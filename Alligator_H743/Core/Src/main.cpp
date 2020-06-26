#include "initialisation.h"

volatile uint32_t SysTickVal;
extern uint32_t SystemCoreClock;

int main(void) {
	//SystemInit();							// Activates floating point coprocessor and resets clock
	NVIC_SetPriorityGrouping(3);
	SysTick_Config(64000);

	SystemClock_Config();					// Configure the clock and PLL
	SystemCoreClockUpdate();				// Update SystemCoreClock (system clock frequency) derived from settings of

	while (1) {

	}
}
