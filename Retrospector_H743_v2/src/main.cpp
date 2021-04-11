#include "initialisation.h"
#include "USB.h"
#include "DigitalDelay.h"
#include "Filter.h"
#include "sdram.h"
#include "LEDHandler.h"
#include "SerialHandler.h"
#include "config.h"
#include "Bootloader.h"

/* TODO
 * USB hangs when sending over CDC and client disconnects
 * Store config/calibration values to Flash (colours, filter slope)
 * Investigate R channel zero offset
 * Investigate background noise and interference every 20ms
 * Link button to unlink right delay in clocked mode
 */

volatile uint32_t SysTickVal;
extern uint32_t SystemCoreClock;


bool USBDebug;



int32_t adcZeroOffset[2] = {33791, 33791};			// 0V ADC reading
int32_t newOffset[2] = {33870, 34000};
uint32_t offsetCounter[2];
bool linkButton;
uint32_t linkBtnTest;
bool activateLEDs = true;

// ADC arrays - place in separate memory area with caching disabled
volatile uint16_t __attribute__((section (".dma_buffer"))) ADC_audio[2];
volatile uint16_t __attribute__((section (".dma_buffer"))) ADC_array[ADC_BUFFER_LENGTH];

__attribute__((section (".led_buffer"))) LEDHandler led;			// led handler in RAM_D3 as SPI6 uses BDMA which only works on this memory region

// Place delay sample buffers in external SDRAM and chorus samples in RAM_D1 (slower, but more space)
int32_t __attribute__((section (".sdramSection"))) samples[SAMPLE_BUFFER_LENGTH];
//int32_t  samples[SAMPLE_BUFFER_LENGTH];
uint16_t __attribute__((section (".chorus_data"))) chorusSamples[2][65536];		// Place in RAM_D1 as no room in DTCRAM

USB usb;
SerialHandler serial(usb);
DigitalDelay delay;
Filter filter;
Config config;
Bootloader bootloader;

extern "C" {
#include "interrupts.h"
}


int main(void) {

	SystemClock_Config();			// Configure the clock and PLL
	SystemCoreClockUpdate();		// Update SystemCoreClock (system clock frequency)
	InitSysTick();

	InitADCAudio();					// Initialise ADC to capture audio samples
	InitADCControls();				// Initialise ADC to capture knob and CV data
	InitDAC();						// DAC used to output Wet/Dry mix levels
	InitSDRAM_16160();
	InitCache();					// Configure MPU to not cache RAM_D3 where the ADC DMA memory resides
	InitLEDSPI();					// Initialise SPI/DAM for LED controller
	led.Init();
	InitIO();						// Initialise switches and LEDs
//	InitDebugTimer();
	filter.Init();					// Initialise filter coefficients, windows etc
	usb.InitUSB();
	delay.Init();					// clear sample buffers and preset delay timings
	InitI2S();						// Initialise I2S which will start main sample interrupts

	while (1) {


		// When silence is detected for a long enough time recalculate ADC offset
		for (channel lr : {left, right}) {
			if (ADC_audio[lr] > 33000 && ADC_audio[lr] < 34500) {
				newOffset[lr] = (ADC_audio[lr] + (127 * newOffset[lr])) >> 7;
				if (offsetCounter[lr] == 1000000) {
					if (adcZeroOffset[lr] > newOffset[lr])
						adcZeroOffset[lr]--;
					else
						adcZeroOffset[lr]++;
					//adcZeroOffset[lr] = newOffset[lr];
					offsetCounter[lr] = 0;
				}
				offsetCounter[lr]++;

			} else {
				offsetCounter[lr] = 0;
			}
		}

		// Implement chorus (PG10)/stereo wide (PC12) switch, and link button (PB4) for delay LR timing
		if (((GPIOG->IDR & GPIO_IDR_ID10) == 0) != delay.chorusMode) {
			delay.ChorusMode(!delay.chorusMode);
		}
		if (((GPIOC->IDR & GPIO_IDR_ID12) == 0) != delay.stereoWide) {
			delay.stereoWide = !delay.stereoWide;
		}
		if (SysTickVal > linkBtnTest + 1) {			// Test if link button pressed with some debouncing
			if ((GPIOB->IDR & GPIO_IDR_ID4) == 0) {
				if (!linkButton) {
					delay.linkLR = !delay.linkLR;
					linkButton = true;
				}
			} else {
				linkButton = false;
			}
			linkBtnTest = SysTickVal;
		}

		filter.Update();			// Check if filter coefficients need to be updated

		serial.Command();			// Check for incoming CDC commands

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

