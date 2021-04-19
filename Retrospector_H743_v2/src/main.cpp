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
 * config to adjust: slope of filters; length multiplier of long delay and reverse
 * Increase speed of LED SPI
 * Investigate R channel zero offset
 * Investigate background noise and interference every 20ms
 * Bug where very short delay times do not activate delay LED correctly
 * USB hangs when sending over CDC and cable removed and reinserted
 */

volatile uint32_t SysTickVal;
extern uint32_t SystemCoreClock;

int32_t adcZeroOffset[2] = {33673, 33660};			// 0V ADC reading
bool linkButton;
uint32_t linkBtnTest;

volatile int adcErrs = 0;		// Debug
volatile int adcErrsPerSec = 0;		// Debug
volatile int adcErrsTemp = 0;		// Debug
volatile int adcErrsTime = 0;		// Debug
volatile bool adcReady = true;

// Store buffers that need to live in special memory areas
volatile uint16_t __attribute__((section (".dma_buffer"))) ADC_audio[AUDIO_BUFFER_LENGTH];		// Place in separate memory area with caching disabled
volatile uint16_t __attribute__((section (".dma_buffer"))) ADC_array[ADC_BUFFER_LENGTH];
__attribute__((section (".led_buffer"))) LEDHandler led;						// led handler in RAM_D3 as SPI6 uses BDMA which only works on this memory region
int32_t __attribute__((section (".sdramSection"))) samples[SAMPLE_BUFFER_LENGTH];	// Place delay sample buffers in external SDRAM
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

//	InitADCAudio();					// Initialise ADC to capture audio samples
	InitADCAudioADC1();

//	InitADCControls();				// Initialise ADC to capture knob and CV data
	InitDAC();						// DAC used to output Wet/Dry mix levels
	InitSDRAM_16160();				// Initialise larger SDRAM
	InitCache();					// Configure MPU to not cache RAM_D3 where the ADC DMA memory resides
	InitLEDSPI();					// Initialise SPI/DAM for LED controller
	led.Init();						// Initialise LED SPI packet
	InitIO();						// Initialise switches and LEDs
	config.RestoreConfig();			// Restore configuration settings (ADC offsets etc)
	filter.Init();					// Initialise filter coefficients, windows etc
	usb.InitUSB();
	delay.Init();					// clear sample buffers and preset delay timings
	InitI2S();						// Initialise I2S which will start main sample interrupts

	while (1) {
/*
		// Test ADC code
		GPIOB->ODR |= GPIO_ODR_OD8;		// Toggle LED for debugging
		ADC2->CR |= ADC_CR_ADSTART;						// Start ADC
		while ((ADC2->ISR & ADC_ISR_EOC) == 0) {};
		ADC_audio[0] = ADC2->DR;
		ADC2->CR |= ADC_CR_ADSTART;						// Start ADC
		while ((ADC2->ISR & ADC_ISR_EOC) == 0) {};
		ADC_audio[1] = ADC2->DR;
		GPIOB->ODR &= ~GPIO_ODR_OD8;
*/
//		if (adcReady) {
//			GPIOB->ODR |= GPIO_ODR_OD8;		// Toggle LED for debugging
//			ADC2->CR |= ADC_CR_ADSTART;
//		}

		if (abs(ADC_audio[left] - adcZeroOffset[left]) > 40 || abs(ADC_audio[1] - adcZeroOffset[right]) > 40) {
			adcErrs++;
			adcErrsTemp++;
			if (adcErrsTime + 1000 < SysTickVal) {
				adcErrsTime = SysTickVal;
				adcErrsPerSec = adcErrsTemp;
				adcErrsTemp = 0;
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

