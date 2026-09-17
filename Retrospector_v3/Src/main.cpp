#include "initialisation.h"
#include "USB.h"
#include "DigitalDelay.h"
#include "LEDHandler.h"
#include "configManager.h"
#include "Calib.h"

volatile uint32_t SysTickVal;
extern uint32_t SystemCoreClock;

// Store buffers that need to live in special memory areas
volatile uint16_t __attribute__((section (".dma_buffer"))) ADC_array[ADC1_BUFFER_LENGTH + ADC2_BUFFER_LENGTH];
__attribute__((section (".led_buffer"))) LEDHandler led;							// led handler in RAM_D3 as SPI6 uses BDMA which only works on this memory region
#ifdef NOEXTRAM
int32_t __attribute__((section (".ram_d1_data"))) samples[SAMPLE_BUFFER_LENGTH];	// Place delay sample buffers in external SDRAM
#else
int32_t __attribute__((section (".sdramSection"))) samples[SAMPLE_BUFFER_LENGTH];	// Place delay sample buffers in external SDRAM
#endif


extern "C" {
#include "interrupts.h"
}

Calib calib;
Filter filter;
DigitalDelay delay;

Config config{&calib.configSaver, &filter.configSaver, &delay.configSaver};			// Construct config handler with list of configSavers

uint32_t lastVal;

int main()
{

	InitHardware();
	led.Init();						// Initialise LED SPI packet
	config.RestoreConfig();			// Restore configuration settings (ADC offsets etc)
	filter.Init();					// Initialise filter coefficients, windows etc
	usb.Init(false);
	delay.Init();					// Clear sample buffers and preset delay timings
	InitI2S();						// Initialise I2S which will start main sample interrupts
	CopyToITCMRAM();				// Copy bootloader code to instruction RAM so it can update Flash

	while (1) {
		calib.AutoZeroOffset();		// Automatically adjust ADC zero offset during quiet sections
		delay.CheckSwitches();		// Check values of switches and detect link button press
		filter.Update();			// Check if filter coefficients need to be updated
		usb.cdc.ProcessCommand();	// Check for incoming USB serial commands
		config.SaveConfig();		// Save any scheduled changes

#if (USB_DEBUG)
		if ((GPIOB->IDR & GPIO_IDR_ID4) == 0 && USBDebug) {
			USBDebug = false;
			usb.OutputDebug();
		}
#endif

	}
}

