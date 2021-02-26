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
 * Explore LED options for filter control
 * Increase tempo Multiplier times for Long Delay
 * USB hangs when sending over CDC and client disconnects
 * Accuracy of tempo (add negative jitter?)
 * Thump when activating chorus mode from startup with silence
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

__attribute__((section (".dma_buffer"))) WS2812Handler ledWS;			// led handler in dma section as DMA cannot operate with DTCMRAM
__attribute__((section (".led_buffer"))) LEDHandler led;			// led handler in RAM_D3 as SPI6 uses BDMA which only works on this memory region

// Place delay sample buffers in external SDRAM and chorus samples in RAM_D1 (slower, but more space)
int32_t __attribute__((section (".sdramSection"))) samples[SAMPLE_BUFFER_LENGTH];
uint16_t __attribute__((section (".chorus_data"))) chorusSamples[2][65536];		// Place in RAM_D1 as no room in DTCRAM

USB usb;
CDCHandler cdc(usb);
DigitalDelay delay;
Filter filter;
//WS2812Handler ledWS;

extern "C" {
#include "interrupts.h"
}


extern "C" int myadd(int a, int b);

uint32_t lastLED = 0;
uint16_t ledCounter[3];
uint32_t ledTarg[3] = {0xFF0000, 0x00FF00, 0x0000FF};
uint32_t ledPrev[3];
uint8_t ledPos[3] = {0, 2, 4};


bool ledCycle = true;
const uint16_t transition = 1000;
const uint32_t colourCycle[] = {
		0xFF0000,
		0x00FF00,
		0x0000FF,
		0xFF3300,
		0x0000FF,
		0x555555 };


int main(void) {

	SystemClock_Config();			// Configure the clock and PLL
	SystemCoreClockUpdate();		// Update SystemCoreClock (system clock frequency)
	InitSysTick();

/*
	InitADCAudio();					// Initialise ADC to capture audio samples
	InitADCControls();				// Initialise ADC to capture knob and CV data
	InitDAC();						// DAC used to output Wet/Dry mix levels
	InitTempoClock();				// Timer to handle incoming tempo clock
	InitSDRAM();
	InitCache();					// Configure MPU to not cache RAM_D3 where the ADC DMA memory resides
	InitIO();						// Initialise switches and LEDs
	InitDebugTimer();
	filter.Init();					// Initialise filter coefficients, windows etc
*/
//	usb.InitUSB();
/*	delay.Init();					// clear sample buffers and preset delay timings
	InitI2S();						// Initialise I2S which will start main sample interrupts
*/
	Init_WS2812_SPI();
	InitLEDSPI();
	led.Init();
	ledWS.Init();
	//int c = myadd(12, 14);			// Assembly test

	while (1) {
		// LED Test
		if (SysTickVal > lastLED + 3) {
			for (uint8_t i = 0; i < 3; ++i) {

				++ledCounter[i];

				float mult = (float)ledCounter[i] / transition;

				// Interpolate colours between previous and target
				uint8_t oldR = ledPrev[i] >> 16;
				uint8_t newR = ledTarg[i] >> 16;
				uint8_t oldG = (ledPrev[i] >> 8) & 0xFF;
				uint8_t newG = (ledTarg[i] >> 8) & 0xFF;
				uint8_t oldB = ledPrev[i] & 0xFF;
				uint8_t newB = ledTarg[i] & 0xFF;

				newR = (oldR + (float)(newR - oldR) * mult);
				newG = (oldG + (float)(newG - oldG) * mult);
				newB = (oldB + (float)(newB - oldB) * mult);
				uint32_t setColour = (newR << 16) + (newG << 8) + newB;


				if (ledCounter[i] < transition) {
					if (i == 2) {
						ledWS.LEDColour(i, 0);
					} else {
						ledWS.LEDColour(i, setColour);
					}
				} else {
					ledPrev[i] = ledTarg[i];

					if (ledCycle) {
						ledTarg[i] = colourCycle[++ledPos[i]];
						if (ledPos[i] == 5)
							ledPos[i] = 0;
					}
					ledCounter[i] = 0;
				}
			}
			lastLED = SysTickVal;
			ledWS.LEDSend();
		}

		//MemoryTest();
/*
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
*/
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

