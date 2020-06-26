#include "stm32h743xx.h"

#define PLL_M 2
#define PLL_N 240
#define PLL_P 1		//  0000001: pll1_p_ck = vco1_ck / 2
#define PLL_Q 4
#define PLL_R 2

void SystemClock_Config() {
	//  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);


	RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;		// Enable System configuration controller clock

	//PWR->CR3 &= ~(PWR_CR3_SCUEN | PWR_CR3_LDOEN | PWR_CR3_BYPASS);
	//PWR->CR3 |= PWR_CR3_LDOEN;
	//Clear mask: (PWR_CR3_SCUEN | PWR_CR3_LDOEN | PWR_CR3_BYPASS)
	//Set mask: PWR_CR3_LDOEN
	PWR->D3CR |= PWR_D3CR_VOS;
	while ((PWR->CSR1 & PWR_CSR1_ACTVOSRDY) == 0U);
//while (PWR->D3CR & PWR_D3CR_VOSRDY)    != PWR_D3CR_VOSRDY

   /* if((__REGULATOR__) == PWR_REGULATOR_VOLTAGE_SCALE0)                      \
    {                                                                        \
       Configure the Voltage Scaling 1                                   \
      MODIFY_REG(PWR->D3CR, PWR_D3CR_VOS, PWR_REGULATOR_VOLTAGE_SCALE1);     \
       Delay after setting the voltage scaling                           \
      tmpreg = READ_BIT(PWR->D3CR, PWR_D3CR_VOS);                            \
       Enable the PWR overdrive                                          \
      SET_BIT(SYSCFG->PWRCR, SYSCFG_PWRCR_ODEN);                             \
       Delay after setting the syscfg boost setting                      \
      tmpreg = READ_BIT(SYSCFG->PWRCR, SYSCFG_PWRCR_ODEN); */
	SYSCFG->PWRCR |= SYSCFG_PWRCR_ODEN;		// Activate the LDO regulator overdrive mode. Must be written only in VOS1 voltage scaling mode.

	// Check Voltage ready 1= Ready, voltage level at or above VOS selected level
	while ((PWR->D3CR & PWR_D3CR_VOSRDY) != PWR_D3CR_VOSRDY);

	RCC->CR |= RCC_CR_HSEON;					// Turn on external oscillator
	while ((RCC->CR & RCC_CR_HSERDY) == 0);		// Wait till HSE is ready

/*	RCC->CR |= RCC_CR_HSI48ON;					// Enable Internal High Speed oscillator for USB
	while ((RCC->CR & RCC_CR_HSI48RDY) == 0);	// Wait till internal USB oscillator is ready
*/

	// Clock source to High speed external and main (M) divider
	RCC->PLLCKSELR = RCC_PLLCKSELR_PLLSRC_HSE |
	                 PLL_M << RCC_PLLCKSELR_DIVM1_Pos;

	// PLL dividers
	RCC->PLL1DIVR = (PLL_N - 1) << RCC_PLL1DIVR_N1_Pos |
			        PLL_P << RCC_PLL1DIVR_P1_Pos |
			        (PLL_Q - 1) << RCC_PLL1DIVR_Q1_Pos |
			        (PLL_R - 1) << RCC_PLL1DIVR_R1_Pos;

	RCC->PLLCFGR |= RCC_PLLCFGR_PLL1RGE_2;		// 10: The PLL1 input (ref1_ck) clock range frequency is between 4 and 8 MHz (Will be 4MHz for 8MHz clock divided by 2)
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLL1VCOSEL;	// 0: Wide VCO range:192 to 836 MHz (default after reset)	1: Medium VCO range:150 to 420 MHz
	RCC->PLLCFGR |= RCC_PLLCFGR_DIVP1EN | RCC_PLLCFGR_DIVQ1EN | RCC_PLLCFGR_DIVR1EN;		// Enable divider outputs
	//RCC->PLLCFGR |= RCC_PLLCFGR_PLL1FRACEN;	// FIXME enables fractional divider - not sure if this is necessary as not using

	RCC->CR |= RCC_CR_PLL1ON;					// Turn on main PLL
	while ((RCC->CR & RCC_CR_PLL1RDY) == 0);	// Wait till PLL is ready

	RCC->D1CFGR |= RCC_D1CFGR_HPRE_3;			// D1 domain AHB prescaler - set to 8 for 240MHz - this is then divided for all APB clocks below
	RCC->D1CFGR |= RCC_D1CFGR_D1PPRE_2; 		// Clock divider for APB3 clocks - set to 4 for 120MHz: 100: hclk / 2
	RCC->D2CFGR |= RCC_D2CFGR_D2PPRE1_2;		// Clock divider for APB1 clocks - set to 4 for 120MHz: 100: hclk / 2
	RCC->D2CFGR |= RCC_D2CFGR_D2PPRE2_2;		// Clock divider for APB2 clocks - set to 4 for 120MHz: 100: hclk / 2
	RCC->D3CFGR |= RCC_D3CFGR_D3PPRE_2;			// Clock divider for APB4 clocks - set to 4 for 120MHz: 100: hclk / 2

	RCC->CFGR |= RCC_CFGR_SW_PLL1;				// System clock switch: 011: PLL1 selected as system clock
	while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != (RCC_CFGR_SW_PLL1 << RCC_CFGR_SWS_Pos));		// Wait until PLL has been selected as system clock source

	// By default Flash latency is set to 7 wait states - set to 4 for now but may need to increase
	FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_4WS;
	while ((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != FLASH_ACR_LATENCY_4WS);

	// SysTick_Config(TicksNumb) TicksNumb = SystemCoreClock / 1000UL gives 1ms
	// NVIC_SetPriority(SysTick_IRQn, 0);

	// Note that the peripherals can use different clock sources - eg for UART configured thus:
	// RCC->D2CCIP2R |= RCC_D2CCIP2R_USART28SEL;
}
