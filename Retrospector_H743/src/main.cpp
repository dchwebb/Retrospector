#include "initialisation.h"
#include "digitalDelay.h"
#include "USB.h"
#include "CDCHandler.h"
#include "sdram.h"
#include "filter.h"

/* TODO
 * Look at handling distortions better (eg loud sine waves with repeats)
 * Elliptic IIR Filter option
 * Use C++ complex library
 * IIR Filters for ADC smoothing
 * IIR prototype code called every time coefficients recalculated
 * CV control over cutoff
 * Options for filter to operate in LP, HP and Both mode
 * Explore LED options for filter control
 * implement ADC CV controls
 * Increase tempo Multiplier times for Long Delay
 *
 * Electrical
 * CV control for filter?
 * Power supply ripple and current final tests
 */

volatile uint32_t SysTickVal;
extern uint32_t SystemCoreClock;


bool USBDebug;

// Enter DFU bootloader - store a custom word at a known RAM address. The startup file checks for this word and jumps to bootloader in RAM if found
void BootDFU() {
	//SCB_DisableDCache();
	__disable_irq();
	*((unsigned long *)0x2407FFF0) = 0xDEADBEEF; // 512KB STM32H7xx
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

// Tone settings to control filter
uint16_t currentTone = 0;
int32_t dampedTone = 0;
uint16_t toneHysteresis = 200;

float DACLevel;							// Wet/Dry Cross fade value
volatile bool sampleClock = false;		// Records whether outputting left or right channel on I2S
//bool ledL, ledR;

// ADC arrays - place in separate memory area with caching disabled
volatile uint16_t __attribute__((section (".dma_buffer"))) ADC_audio[2];
volatile uint16_t __attribute__((section (".dma_buffer"))) ADC_array[ADC_BUFFER_LENGTH];

USB usb;
digitalDelay DigitalDelay;
filter Filter;


int16_t __attribute__((section (".sdramSection"))) samples[2][SAMPLE_BUFFER_LENGTH];
//int16_t samples[2][SAMPLE_BUFFER_LENGTH];

// Debug
char usbBuf[8 * FIRTAPS + 1];
bool sendVals = false;


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
	InitIO();

	// Initialise filter
	FIRFilterWindow(4.0);
	currentTone = ADC_array[ADC_Tone];
	dampedTone = currentTone;
	Filter.InitFIRFilter(currentTone);
	Filter.InitIIRFilter(currentTone);

	usb.InitUSB();
	usb.cdcDataHandler = std::bind(CDCHandler, std::placeholders::_1, std::placeholders::_2);

	InitCache();				// Configure MPU to not cache RAM_D3 where the ADC DMA memory resides
	std::fill_n(samples[0], SAMPLE_BUFFER_LENGTH, 0);
	std::fill_n(samples[1], SAMPLE_BUFFER_LENGTH, 0);

	DAC1->DHR12R2 = 2048;		// Pins 3 & 6 on VCA (MIX_WET_CTL)
	DAC1->DHR12R1 = 2048; 		// Pins 11 & 14 on VCA (MIX_DRY_CTL)

	DigitalDelay.init();
	InitI2S();


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


		clockValid = (SysTickVal - lastClock < 1000);		// Valid clock interval is within a second

		// Output mix level
		DACLevel = (static_cast<float>(ADC_array[ADC_Mix]) / 65536.0f);		// Convert 16 bit int to float 0 -> 1
		DAC1->DHR12R2 = DACLevel * 4095.0f;					// Wet level
		DAC1->DHR12R1 = (1.0f - DACLevel) * 4095.0f;		// Dry level

		// Check if filter coefficients need to be updated
		dampedTone = std::max((31 * dampedTone + ADC_array[ADC_Tone]) >> 5, 0L);

		if (std::abs(dampedTone - currentTone) > toneHysteresis) {
			calculatingFilter = true;
			currentTone = dampedTone;
			if (iirFilter) {
				Filter.InitIIRFilter(currentTone);
			} else {
				Filter.InitFIRFilter(currentTone);
			}
			calculatingFilter = false;
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

