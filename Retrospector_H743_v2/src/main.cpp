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
 * Config to adjust: length multiplier of long delay and reverse
 */


volatile uint32_t SysTickVal;
extern uint32_t SystemCoreClock;

int32_t adcZeroOffset[2] = {ADC_OFFSET_DEFAULT, ADC_OFFSET_DEFAULT};				// 0V ADC reading

// Store buffers that need to live in special memory areas
volatile uint16_t __attribute__((section (".dma_buffer"))) ADC_array[ADC1_BUFFER_LENGTH + ADC2_BUFFER_LENGTH];
__attribute__((section (".led_buffer"))) LEDHandler led;							// led handler in RAM_D3 as SPI6 uses BDMA which only works on this memory region
int32_t __attribute__((section (".sdramSection"))) samples[SAMPLE_BUFFER_LENGTH];	// Place delay sample buffers in external SDRAM

USB usb;
SerialHandler serial(usb);
DigitalDelay delay;
Filter filter;
Config config;
Bootloader bootloader;

extern "C" {
#include "interrupts.h"
}

uint32_t lastVal;

int main(void) {

	SystemClock_Config();			// Configure the clock and PLL
	SystemCoreClockUpdate();		// Update SystemCoreClock (system clock frequency)
	InitSysTick();

	InitADC();
	InitDAC();						// DAC used to output Wet/Dry mix levels
	InitSDRAM_16160();				// Initialise 32MB SDRAM
	InitCache();					// Configure MPU to not cache memory regions where DMA beffers reside
	InitLEDSPI();					// Initialise SPI/DAM for LED controller
	led.Init();						// Initialise LED SPI packet
	InitIO();						// Initialise switches and LEDs
	config.RestoreConfig();			// Restore configuration settings (ADC offsets etc)
	filter.Init();					// Initialise filter coefficients, windows etc
	usb.InitUSB();
	delay.Init();					// Clear sample buffers and preset delay timings
	InitI2S();						// Initialise I2S which will start main sample interrupts
	CopyToITCMRAM();				// Copy bootloader code to instruction RAM so it can update Flash

	while (1) {
		config.AutoZeroOffset();	// Automatically adjust ADC zero offset during quiet sections
		delay.CheckSwitches();		// Check values of switches and detect link button press
		filter.Update();			// Check if filter coefficients need to be updated
		serial.Command();			// Check for incoming CDC commands

#if (USB_DEBUG)
		if ((GPIOB->IDR & GPIO_IDR_ID4) == 0 && USBDebug) {
			USBDebug = false;
			usb.OutputDebug();
		}
#endif

	}
}

