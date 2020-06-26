#include "stm32h743xx.h"
#include "initialisation.h"

#define PLL_M 2
#define PLL_N 240
#define PLL_P 1		//  0000001: pll1_p_ck = vco1_ck / 2
#define PLL_Q 4
#define PLL_R 2

void SystemClock_Config() {

	RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;			// Enable System configuration controller clock

	PWR->D3CR |= PWR_D3CR_VOS;						// Configure voltage scaling level 1 before engaging overdrive (0b11 = VOS1)
	while ((PWR->CSR1 & PWR_CSR1_ACTVOSRDY) == 0);	// Check Voltage ready 1= Ready, voltage level at or above VOS selected level

	SYSCFG->PWRCR |= SYSCFG_PWRCR_ODEN;				// Activate the LDO regulator overdrive mode. Must be written only in VOS1 voltage scaling mode.
	while ((SYSCFG->PWRCR & SYSCFG_PWRCR_ODEN) == 0);

	RCC->CR |= RCC_CR_HSEON;						// Turn on external oscillator
	while ((RCC->CR & RCC_CR_HSERDY) == 0);			// Wait till HSE is ready

/*	RCC->CR |= RCC_CR_HSI48ON;						// Enable Internal High Speed oscillator for USB
	while ((RCC->CR & RCC_CR_HSI48RDY) == 0);		// Wait till internal USB oscillator is ready
*/

	// Clock source to High speed external and main (M) divider
	RCC->PLLCKSELR = RCC_PLLCKSELR_PLLSRC_HSE |
	                 PLL_M << RCC_PLLCKSELR_DIVM1_Pos;

	// PLL dividers
	RCC->PLL1DIVR = (PLL_N - 1) << RCC_PLL1DIVR_N1_Pos |
			        PLL_P << RCC_PLL1DIVR_P1_Pos |
			        (PLL_Q - 1) << RCC_PLL1DIVR_Q1_Pos |
			        (PLL_R - 1) << RCC_PLL1DIVR_R1_Pos;

	RCC->PLLCFGR |= RCC_PLLCFGR_PLL1RGE_2;			// 10: The PLL1 input (ref1_ck) clock range frequency is between 4 and 8 MHz (Will be 4MHz for 8MHz clock divided by 2)
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLL1VCOSEL;		// 0: Wide VCO range:192 to 836 MHz (default after reset)	1: Medium VCO range:150 to 420 MHz
	RCC->PLLCFGR |= RCC_PLLCFGR_DIVP1EN | RCC_PLLCFGR_DIVQ1EN | RCC_PLLCFGR_DIVR1EN;		// Enable divider outputs
	//RCC->PLLCFGR |= RCC_PLLCFGR_PLL1FRACEN;		// FIXME enables fractional divider - not sure if this is necessary as not using

	RCC->CR |= RCC_CR_PLL1ON;						// Turn on main PLL
	while ((RCC->CR & RCC_CR_PLL1RDY) == 0);		// Wait till PLL is ready

	RCC->D1CFGR |= RCC_D1CFGR_HPRE_3;				// D1 domain AHB prescaler - set to 8 for 240MHz - this is then divided for all APB clocks below
	RCC->D1CFGR |= RCC_D1CFGR_D1PPRE_2; 			// Clock divider for APB3 clocks - set to 4 for 120MHz: 100: hclk / 2
	RCC->D2CFGR |= RCC_D2CFGR_D2PPRE1_2;			// Clock divider for APB1 clocks - set to 4 for 120MHz: 100: hclk / 2
	RCC->D2CFGR |= RCC_D2CFGR_D2PPRE2_2;			// Clock divider for APB2 clocks - set to 4 for 120MHz: 100: hclk / 2
	RCC->D3CFGR |= RCC_D3CFGR_D3PPRE_2;				// Clock divider for APB4 clocks - set to 4 for 120MHz: 100: hclk / 2

	RCC->CFGR |= RCC_CFGR_SW_PLL1;					// System clock switch: 011: PLL1 selected as system clock
	while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != (RCC_CFGR_SW_PLL1 << RCC_CFGR_SWS_Pos));		// Wait until PLL has been selected as system clock source

	// By default Flash latency is set to 7 wait states - set to 4 for now but may need to increase
	FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_4WS;
	while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != FLASH_ACR_LATENCY_4WS);

	// Note that the peripherals can use different clock sources - eg for UART configured thus:
	// RCC->D2CCIP2R |= RCC_D2CCIP2R_USART28SEL;
}


void InitSysTick() {
	SysTick_Config(SystemCoreClock / 1000UL);		// gives 1ms
	NVIC_SetPriority(SysTick_IRQn, 0);
}


void InitUART() {
	// H743 Nucelo STLink connects virtual COM port to PD8 (USART3 TX) and PD9 (USART3 RX)

	RCC->APB1LENR |= RCC_APB1LENR_USART3EN;			// USART3 clock enable
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;			// GPIO port D

	GPIOD->MODER &= ~GPIO_MODER_MODE8_0;			// Set alternate function on PD8
	GPIOD->AFR[1] |= 7 << GPIO_AFRH_AFSEL8_Pos;		// Alternate function on PD8 for USART3_TX is AF7
	GPIOD->MODER &= ~GPIO_MODER_MODE9_0;			// Set alternate function on PA10
	GPIOD->AFR[1] |= 7 << GPIO_AFRH_AFSEL9_Pos;		// Alternate function on PD9 for USART3_RX is AF7

	// By default clock source is muxed to peripheral clock 1 which is system clock / 4 (change clock source in RCC->D2CCIP2R)
	// Calculations depended on oversampling mode set in CR1 OVER8. Default = 0: Oversampling by 16
	int USARTDIV = (SystemCoreClock / 4) / 230400;	//clk / desired_baud
	USART3->BRR |= USARTDIV & USART_BRR_DIV_MANTISSA_Msk;
	USART3->CR1 &= ~USART_CR1_M;					// 0: 1 Start bit, 8 Data bits, n Stop bit; 	1: 1 Start bit, 9 Data bits, n Stop bit
	USART3->CR1 |= USART_CR1_RE;					// Receive enable
	USART3->CR1 |= USART_CR1_TE;					// Transmitter enable

	// Set up interrupts
	USART3->CR1 |= USART_CR1_RXNEIE;
	NVIC_SetPriority(USART3_IRQn, 3);				// Lower is higher priority
	NVIC_EnableIRQ(USART3_IRQn);

	USART3->CR1 |= USART_CR1_UE;					// USART Enable
}


void uartSendChar(char c) {
	while ((USART3->ISR & USART_ISR_TXE_TXFNF) == 0);
	USART3->TDR = c;
}

void uartSendString(const std::string& s) {
	for (char c : s) {
		while ((USART3->ISR & USART_ISR_TXE_TXFNF) == 0);
		USART3->TDR = c;
	}

}
