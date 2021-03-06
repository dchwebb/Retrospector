#include "initialisation.h"
#include "USB.h"
#include "CDCHandler.h"
#include "DigitalDelay.h"
#include "Filter.h"
#include "sdram.h"
//#include "I2CHandler.h"
#include "LEDHandler.h"
#include "WS2812Handler.h"

/* TODO
 * Increase tempo Multiplier times for Long Delay
 * USB hangs when sending over CDC and client disconnects
 * Accuracy of tempo (add negative jitter?)
 * DFU
 *
 *
 * Electrical/Hardware
 * CV control for filter?
 * Power supply ripple and current final tests
 * Control for panoramic mode
 */

volatile uint32_t SysTickVal;
extern uint32_t SystemCoreClock;


bool USBDebug;

// Enter DFU bootloader - store a custom word at a known RAM address. The startup file checks for this word and jumps to bootloader in RAM if found
void BootDFU() {
	//SCB_DisableDCache();
	__disable_irq();
	*((unsigned long *)0x2407FFF0) = 0xDEADBEEF; 	// 512KB STM32H7xx
	__DSB();
	NVIC_SystemReset();
}

uint16_t adcZeroOffset[2] = {33800, 33800};			// 0V ADC reading
uint32_t newOffset[2] = {33800, 33800};
uint32_t offsetCounter[2];

// Settings for tempo clock input
uint32_t lastClock = 0;
uint32_t clockInterval = 0;
bool clockValid;

volatile bool sampleClock = false;		// Records whether outputting left or right channel on I2S

// ADC arrays - place in separate memory area with caching disabled
volatile uint16_t __attribute__((section (".dma_buffer"))) ADC_audio[2];
volatile uint16_t __attribute__((section (".dma_buffer"))) ADC_array[ADC_BUFFER_LENGTH];

__attribute__((section (".led_buffer"))) LEDHandler led;			// led handler in RAM_D3 as SPI6 uses BDMA which only works on this memory region

// Place delay sample buffers in external SDRAM and chorus samples in RAM_D1 (slower, but more space)
int32_t __attribute__((section (".sdramSection"))) samples[SAMPLE_BUFFER_LENGTH];
uint16_t __attribute__((section (".chorus_data"))) chorusSamples[2][65536];		// Place in RAM_D1 as no room in DTCRAM

USB usb;
CDCHandler cdc(usb);
DigitalDelay delay;
Filter filter;

extern "C" {
#include "interrupts.h"
}


extern "C" int myadd(int a, int b);




int main(void) {

	SystemClock_Config();			// Configure the clock and PLL
	SystemCoreClockUpdate();		// Update SystemCoreClock (system clock frequency)
	InitSysTick();

	InitADCAudio();					// Initialise ADC to capture audio samples
	InitADCControls();				// Initialise ADC to capture knob and CV data
	InitDAC();						// DAC used to output Wet/Dry mix levels
	InitTempoClock();				// Timer to handle incoming tempo clock
	InitSDRAM();
	InitCache();					// Configure MPU to not cache RAM_D3 where the ADC DMA memory resides
	InitIO();						// Initialise switches and LEDs
	InitDebugTimer();
	filter.Init();					// Initialise filter coefficients, windows etc
	usb.InitUSB();
	delay.Init();					// clear sample buffers and preset delay timings
	InitLEDSPI();					// Initialise SPI/DAM for LED controller
	//Init_WS2812_SPI();
	led.Init();
	InitI2S();						// Initialise I2S which will start main sample interrupts

	while (1) {



		//MemoryTest();

		// When silence is detected for a long enough time recalculate ADC offset
		for (channel lr : {left, right}) {
 			if (ADC_audio[lr] > 33000 && ADC_audio[lr] < 34500) {
				newOffset[lr] = (ADC_audio[lr] + (127 * newOffset[lr])) >> 7;
				if (offsetCounter[lr] == 3000000) {
					adcZeroOffset[lr] = newOffset[lr];
					offsetCounter[lr] = 0;
				}
				offsetCounter[lr]++;

			} else {
				offsetCounter[lr] = 0;
			}
		}

		clockValid = (SysTickVal - lastClock < 1000);			// Valid clock interval is within a second
		filter.Update();			// Check if filter coefficients need to be updated

		cdc.Command();				// Check for incoming CDC commands

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

