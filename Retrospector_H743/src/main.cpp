#include "initialisation.h"
#include "digitalDelay.h"
#include "USB.h"
#include "CDCHandler.h"
#include "uartHandler.h"		// FIXME Only needed for dev boards with ST Link UART debugging available
#include "sdram.h"
#include <cmath>

volatile uint32_t SysTickVal;
extern uint32_t SystemCoreClock;

// As this is called from an interrupt assign the command to a variable so it can be handled in the main loop
bool CmdPending = false;
std::string ComCmd;

void CDCHandler(uint8_t* data, uint32_t length) {
	ComCmd = std::string((char*)data, length);
	CmdPending = true;
}

// Enter DFU bootloader - store a custom word at a known RAM address. The startup file checks for this word and jumps to bootloader in RAM if found
void BootDFU() {
	//SCB_DisableDCache();
	__disable_irq();
	*((unsigned long *)0x2407FFF0) = 0xDEADBEEF; // 512KB STM32H7xx
	__DSB();
	NVIC_SystemReset();
}

//int16_t samples[SAMPLE_BUFFER_LENGTH];
uint16_t adcZeroOffset = 34067;		// 0V ADC reading


//int32_t newReadPos;
//int32_t playSample;
int32_t readPos;
int32_t oldReadPos;
int32_t writePos;
int16_t delayChanged;
uint16_t delayCrossfade;
int32_t currentDelay;
int32_t dampedDelay;

int32_t lastClock = 0;
int32_t clockInterval = 0;

int32_t ns, s_n, ls, rp, nrp;

int16_t testOutput = 0;
int32_t debugVal;
int32_t debugC;
int32_t debugD;
int32_t debugTimer = 0;
int32_t debugCCR = 0;
uint16_t adcTest;

volatile int32_t debugClkInt = 0;
float DACLevel;

bool USBDebug;

volatile bool sampleClock = false;
bool nextSample = false;

char pendingCmd[50];

volatile uint16_t __attribute__((section (".dma_buffer"))) ADC_array[ADC_BUFFER_LENGTH] __attribute__ ((aligned (32)));

USB usb;
digitalDelay DigitalDelay;

extern "C" {
#include "interrupts.h"
}

// SDRAM testing variables
using memSize = uint32_t;
uint32_t testAddr;
memSize* tempAddr;
constexpr uint32_t maxAddr = 0x1000000 / sizeof(memSize);
uint32_t MemTestDuration;
uint32_t MemTestCount = 0;
uint32_t MemTestErrors = 0;


uint32_t volDebug = 0;
uint32_t volDebugLevel = 1024;

// Tell linker script to store buffer in SDRAM
uint32_t __attribute__((section (".sdramSection"))) SDRAMBuffer[maxAddr];

// Test SDRAM
uint32_t MemoryTest() {
	// Blank
	uint32_t testStart = SysTickVal;
	for (testAddr = 0; testAddr < maxAddr; ++testAddr) {
		tempAddr = (memSize *)(0xD0000000 + (testAddr * sizeof(memSize)));
		*tempAddr = 0;
	}

	// Write
	for (testAddr = 0; testAddr < maxAddr; ++testAddr) {
		tempAddr = (memSize *)(0xD0000000 + (testAddr * sizeof(memSize)));
		*tempAddr = static_cast<memSize>(testAddr);
		if (*tempAddr == 0x00080000) {
			int susp = 1;
		}
	}

	// Read
	int err = 0;
	for (testAddr = 0; testAddr < maxAddr; ++testAddr) {
		memSize val = SDRAMBuffer[testAddr];
		if (val != static_cast<memSize>(testAddr))
			++err;
	}

	MemTestDuration = SysTickVal - testStart;		// For 32 bit words > 6632 using PLL2 at 143MHz, 6309 at 220MHz, 6385 (2813 with DCache, 2347 with DCache and ICache) at 100MHz
	++MemTestCount;
	return err;
}

int main(void) {
	SystemClock_Config();					// Configure the clock and PLL
	SystemCoreClockUpdate();				// Update SystemCoreClock (system clock frequency)
	InitSysTick();
	//InitUART();
	InitADC();
	InitDAC();
	InitClock();
	InitSDRAM();

	//usb.InitUSB();
	//usb.cdcDataHandler = std::bind(CDCHandler, std::placeholders::_1, std::placeholders::_2);

	InitI2S();

	InitCache();		// Configure MPU to not cache RAM_D3 where the ADC DMA memory resides NB - not currently working

	DAC1->DHR12R2 = 1048; //Pins 3 & 6 on VCA (MIX_WET_CTL)
	DAC1->DHR12R1 = 2048; //Pins 11 & 14 on VCA (MIX_DRY_CTL)

	while (1) {
		// DAC signals that it is ready for the next sample
/*		if (nextSample) {
			nextSample = false;
			if (!sampleClock) {
				playSample = DigitalDelay.calcSample();
			}

		}*/

		// Code to invalidate data cache for ADC_array
		//SCB_InvalidateDCache_by_Addr((uint32_t*)(((uint32_t)ADC_array) & ~(uint32_t)0x1F), (ADC_BUFFER_LENGTH * 2) + 32);
		adcTest = ADC_array[0];

		//MemTestErrors += MemoryTest();

		debugTimer = TIM3->CNT;
		debugCCR = TIM3->CCR1;

		// Output mix level
		DACLevel = (static_cast<float>(ADC_array[ADC_Mix]) / 65536.0f);		// Convert 16 bit int to float 0 -> 1
		volDebug += 1;
		if (volDebug > 1000000) {
			//DAC1->DHR12R2 = std::pow(DACLevel, 5) * 4096;
			//DAC1->DHR12R1 = std::pow((1.0f - DACLevel), 5) * 4096.0f;
			volDebugLevel += 1024;
			if (volDebugLevel > 4000) {
				volDebugLevel = 0;
			}
			volDebug = 0;
			DAC1->DHR12R2 = volDebugLevel; //Pins 3 & 6 on VCA (MIX_WET_CTL)
			DAC1->DHR12R1 = volDebugLevel; //Pins 11 & 14 on VCA (MIX_DRY_CTL)

		}

		//DAC1->DHR12R2 = std::pow(DACLevel, 5) * 4096;
		//DAC1->DHR12R1 = std::pow((1.0f - DACLevel), 5) * 4096.0f;

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

		// Handle UART Commands from STLink
		if (pendingCmd[0]) {
			if (strcmp(pendingCmd, "dfu") == 0) {
				BootDFU();
			}
			uartSendString("Received: ");
			uartSendString(pendingCmd);
			uartSendString("\n");
			pendingCmd[0] = 0;
		}

		// Check for incoming CDC commands
		if (CmdPending) {
			if (!CDCCommand(ComCmd)) {
				usb.SendString("Unrecognised command. Type 'help' for supported commands\n");
			}
			CmdPending = false;
		}

#if (USB_DEBUG)
		if ((GPIOC->IDR & GPIO_IDR_ID13) == GPIO_IDR_ID13 && !USBDebug) {
			USBDebug = true;
			usb.OutputDebug();
		} else {
			USBDebug = false;
		}
#endif

	}
}

