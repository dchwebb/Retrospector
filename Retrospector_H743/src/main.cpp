#include "initialisation.h"
#include "digitalDelay.h"
#include "USB.h"
#include "CDCHandler.h"
#include "uartHandler.h"		// FIXME Only needed for dev boards with ST Link UART debugging available
#include "sdram.h"
#include "filter.h"
#include <cmath>

volatile uint32_t SysTickVal;
extern uint32_t SystemCoreClock;

bool USBDebug;

// As this is called from an interrupt assign the command to a variable so it can be handled in the main loop
volatile bool CmdPending = false;
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

uint16_t adcZeroOffset = 33800;		// 0V ADC reading

//int32_t readPos;
//int32_t oldReadPos;
//int32_t writePos;
//int16_t delayChanged;
//uint16_t delayCrossfade;
//int32_t currentDelay;
//int32_t dampedDelay;

uint32_t lastClock = 0;
uint32_t clockInterval = 0;
bool ledL, ledR;
bool newClock, clockValid;
int16_t testOutput = 0;
float DACLevel;							// Cross fade value
volatile bool sampleClock = false;		// Records whether outputting left or right channel on I2S
bool nextSample = false;

volatile uint16_t __attribute__((section (".dma_buffer"))) ADC_audio[2];
volatile uint16_t __attribute__((section (".dma_buffer"))) ADC_array[ADC_BUFFER_LENGTH];

USB usb;
digitalDelay DigitalDelay;

//int16_t __attribute__((section (".sdramSection"))) samples[2][SAMPLE_BUFFER_LENGTH];
int16_t samples[2][SAMPLE_BUFFER_LENGTH];

// FIR data
bool activateFilter = false;
uint8_t activeFilter = 0;		// choose which set of coefficients to use
float firCoeff[2][FIRTAPS];
uint16_t currentTone = 0;
int32_t dampedTone = 0;
uint16_t toneHysteresis = 40;
float currentCutoff;
FilterType filterType = LowPass;


extern "C" {
#include "interrupts.h"
}

int main(void) {
	SystemClock_Config();					// Configure the clock and PLL
	SystemCoreClockUpdate();				// Update SystemCoreClock (system clock frequency)
	InitSysTick();
	InitADCAudio();
	InitADCControls();
	InitDAC();
	InitTempoClock();
	InitSDRAM();
	InitLEDs();

	currentTone = ADC_array[ADC_Tone];
	dampedTone = currentTone;
	float expTone;
	if (filterType == LowPass)
		expTone = 1.0f - std::pow((float)currentTone / 65536.0f, 0.2f);
	else
		expTone = 1.0f - std::pow((float)currentTone / 65536.0f, 5.0f);
	InitFilter(expTone);

	//InitFilter(0.04);			// OmegaC: cut off divided by nyquist; ie 1000 / 24000 for 1kHz cut off at 48kHz sample rate

	usb.InitUSB();
	usb.cdcDataHandler = std::bind(CDCHandler, std::placeholders::_1, std::placeholders::_2);

	InitI2S();

	InitCache();		// Configure MPU to not cache RAM_D3 where the ADC DMA memory resides NB - not currently working

	DAC1->DHR12R2 = 2048; //Pins 3 & 6 on VCA (MIX_WET_CTL)
	DAC1->DHR12R1 = 2048; //Pins 11 & 14 on VCA (MIX_DRY_CTL)

	while (1) {
		//MemoryTest();

		// Hacky delay before activating filter as this can hang the I2S interrupt with longer filter taps and low compiler optimisation
		if (SysTickVal > 500 && SysTickVal < 600) {
			activateFilter = true;
		}

		if (newClock) {
			if (SysTickVal - lastClock < 10)
				GPIOC->ODR |= GPIO_ODR_OD10;
			else {
				GPIOC->ODR &= ~GPIO_ODR_OD10;
				newClock = false;
			}
		}

		clockValid = (SysTickVal - lastClock < 1000);		// Valid clock interval is within a second

		// Output mix level
		DACLevel = (static_cast<float>(ADC_array[ADC_Mix]) / 65536.0f);		// Convert 16 bit int to float 0 -> 1
		DAC1->DHR12R2 = DACLevel * 4095.0f;					// Wet level
		DAC1->DHR12R1 = (1.0f - DACLevel) * 4095.0f;		// Dry level

		// Check if filter coefficients need to be updated
		dampedTone = std::max((31 * dampedTone + ADC_array[ADC_Tone]) >> 5, 0L);

		if (std::abs(dampedTone - currentTone) > toneHysteresis) {
			currentTone = dampedTone;
			float expTone;

			if (currentTone < 32768) {		// Low Pass
				filterType = LowPass;
				expTone = 1.0f - std::pow((32768.0f - currentTone) / 33000.0f, 0.2f);
			} else {
				filterType = HighPass;
				expTone = 1.0f - std::pow(((float)currentTone - 32768.0f)  / 40000.0f, 7.0f);
			}
			InitFilter(expTone);
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

