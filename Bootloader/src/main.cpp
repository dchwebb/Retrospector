#include "initialisation.h"
#include "USB.h"
#include "sdram.h"

volatile uint32_t SysTickVal;
extern uint32_t SystemCoreClock;

extern "C" {
#include "interrupts.h"
}

USB usb;

int main() {

	SystemClock_Config();			// Configure the clock and PLL
	SystemCoreClockUpdate();		// Update SystemCoreClock (system clock frequency)
	InitSysTick();
	InitSDRAM_16160();				// Initialise 32MB SDRAM

	// Relocate the vector table to the flash memory location (moved to startup script)
	//SCB->VTOR = 0x08100000;

	// Initialise timing LEDs on PC10 and PC11
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;			// GPIO port clock
	GPIOB->MODER &= ~GPIO_MODER_MODE7_1;			// PB7: debug pin
	GPIOB->MODER &= ~GPIO_MODER_MODE8_1;			// PB8: debug pin

	// Init Timer
	RCC->APB1LENR |= RCC_APB1LENR_TIM2EN;
	for (int x = 0; x < 200000; ++x) {};

	TIM2->PSC = 100;									// [prescaler is PSC + 1]
	TIM2->ARR = 65535;								// Set auto reload register
	TIM2->DIER |= TIM_DIER_UIE;						// DMA/interrupt enable register
	NVIC_SetPriority(TIM2_IRQn, 3);					// Lower is higher priority
	NVIC_EnableIRQ(TIM2_IRQn);
	TIM2->CR1 |= TIM_CR1_CEN;
	//TIM2->EGR |= TIM_EGR_UG;						//  Re-initializes counter and generates update of registers

	while(1) {
		for (int x = 0; x < 200000; ++x) {
			GPIOB->ODR |= GPIO_ODR_OD7;
		}
		for (int x = 0; x < 200000; ++x) {
			GPIOB->ODR &= ~GPIO_ODR_OD7;
		}

	}

}
/*
  ldr r3, =0xe000ed00
  ldr r2, =0x80e0000
  str r2, [r3, #8]
 */
