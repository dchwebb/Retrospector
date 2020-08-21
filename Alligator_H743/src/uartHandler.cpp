#include "uartHandler.h"

volatile uint8_t uartCmdPos = 0;
volatile char uartCmd[100];
volatile bool uartCmdRdy = false;

// Manages communication to ST Link debugger UART
void InitUART() {
	// H743 Nucelo STLink connects virtual COM port to PD8 (USART3 TX) and PD9 (USART3 RX)

	RCC->APB1LENR |= RCC_APB1LENR_USART3EN;			// USART3 clock enable
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIODEN;			// GPIO port D

	GPIOD->MODER &= ~GPIO_MODER_MODE8_0;			// Set alternate function on PD8
	GPIOD->AFR[1] |= 7 << GPIO_AFRH_AFSEL8_Pos;		// Alternate function on PD8 for USART3_TX is AF7
	GPIOD->MODER &= ~GPIO_MODER_MODE9_0;			// Set alternate function on PD9
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

	// configure PC13 user button on nucleo board
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN;			// GPIO port clock
	GPIOC->MODER &= ~GPIO_MODER_MODE13_Msk;

}

std::string IntToString(const int32_t& v) {
	std::stringstream ss;
	ss << v;
	return ss.str();
}

std::string HexToString(const uint32_t& v, const bool& spaces) {
	std::stringstream ss;
	ss << std::uppercase << std::setfill('0') << std::setw(8) << std::hex << v;
	if (spaces) {
		//std::string s = ss.str();
		return ss.str().insert(2, " ").insert(5, " ").insert(8, " ");
	}
	return ss.str();
}

std::string HexByte(const uint16_t& v) {
	std::stringstream ss;
	ss << std::uppercase << std::setfill('0') << std::setw(2) << std::hex << v;
	return ss.str();
}

void uartSendChar(char c) {
	while ((USART3->ISR & USART_ISR_TXE_TXFNF) == 0);
	USART3->TDR = c;
}

void uartSendString(const char* s) {
	char c = s[0];
	uint8_t i = 0;
	while (c) {
		while ((USART3->ISR & USART_ISR_TXE_TXFNF) == 0);
		USART3->TDR = c;
		c = s[++i];
	}
}

void uartSendString(const std::string& s) {
	for (char c : s) {
		while ((USART3->ISR & USART_ISR_TXE_TXFNF) == 0);
		USART3->TDR = c;
	}
}

extern "C" {

// USART Decoder
void USART3_IRQHandler() {
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
}
