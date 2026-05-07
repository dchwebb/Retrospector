#include <config.h>


extern DigitalDelay delay;

void Config::Calibrate()
{
	usb.SendString("Calibrating ...\r\n");

	int32_t audioOffsetL = ADC_array[left];
	int32_t audioOffsetR = ADC_array[right];
	int32_t filterCenter = ADC_array[ADC_Filter_Pot];

	for (int32_t i = 0; i < 10000000; ++i) {
		audioOffsetL = std::round((static_cast<float>(ADC_array[left]) + (63.0f * audioOffsetL)) / 64.0f);
		audioOffsetR = std::round((static_cast<float>(ADC_array[right]) + (63.0f * audioOffsetR)) / 64.0f);
		filterCenter = std::round((static_cast<float>(ADC_array[ADC_Filter_Pot]) + (63.0f * filterCenter)) / 64.0f);
	}

	usb.SendString("Calibration Audio L: " + std::to_string(audioOffsetL) + "; Audio R: " + std::to_string(audioOffsetR) + "; Filter: " + std::to_string(filterCenter) + "\r\n");

	if (audioOffsetL < 33000 || audioOffsetL > 34500) {
		usb.SendString("Calibration failed. Audio L out of range\r\n");
		return;
	}
	if (audioOffsetR < 33000 || audioOffsetR > 34500) {
		usb.SendString("Calibration failed. Audio R out of range\r\n");
		return;
	}
	if (filterCenter < 30000 || filterCenter > 35000) {
		usb.SendString("Calibration failed. Filter pot out of range\r\n");
		return;
	}

	// If calibration in acceptable range save to config
	adcZeroOffset[left] = audioOffsetL;
	adcZeroOffset[right] = audioOffsetR;
	filter.potCentre = filterCenter;
	SaveConfig();
}

void Config::AutoZeroOffset()
{
	// When silence is detected for a long enough time recalculate ADC offset
	for (channel lr : {left, right}) {
		if (ADC_array[lr] > adcZeroOffset[lr] - 500 && ADC_array[lr] < adcZeroOffset[lr] + 500) {
			newOffset[lr] = (static_cast<float>(ADC_array[lr]) + (799.0f * newOffset[lr])) / 800.0f;
			if (offsetCounter[lr] == 200000) {
				if (newOffset[lr] > adcZeroOffset[lr] + 10) {
					adcZeroOffset[lr]++;
				} else if (newOffset[lr] < adcZeroOffset[lr] - 10) {
					adcZeroOffset[lr]--;
				}
				offsetCounter[lr] = 0;
			}
			offsetCounter[lr]++;
		} else {
			offsetCounter[lr] = 0;
		}
	}
}


// called whenever a config setting is changed to schedule a save after waiting to see if any more changes are being made
void Config::ScheduleSave()
{
	scheduleSave = true;
	saveBooked = SysTickVal;
}


// Write calibration settings to Flash memory (H743 see programming manual p152 for sector layout)
bool Config::SaveConfig()
{
	scheduleSave = false;
	suspendI2S();

	configValues cv;
	SetConfig(cv);

	__disable_irq();					// Disable Interrupts
	FlashUnlock(2);						// Unlock Flash memory for writing
	FLASH->SR2 = FLASH_ALL_ERRORS;		// Clear error flags in Status Register
	FlashEraseSector(7, 2);				// Erase sector 7, Bank 2 (See p152 of manual)
	bool result = FlashProgram(ADDR_FLASH_SECTOR_7, reinterpret_cast<uint32_t*>(&cv), sizeof(cv));
	FlashLock(2);						// Lock Bank 2 Flash
	__enable_irq(); 					// Enable Interrupts

	resumeI2S();
	return result;
}


void Config::SetConfig(configValues &cv)
{
	cv.audio_offset_left = adcZeroOffset[left];
	cv.audio_offset_right = adcZeroOffset[right];

	cv.delay_gate_threshold = delay.gateThreshold;
	cv.delay_gate_activate = delay.gateHoldCount;
	cv.delay_modOffsetMax = delay.modOffsetMax;
	cv.delay_modOffsetInc = delay.modOffsetInc;

	cv.filter_pot_center = filter.potCentre;
	cv.filter_fir_taps = filter.firTaps;
	cv.filter_num_poles = filter.iirLPFilter[0].numPoles;
	cv.filter_custom_damping = filter.customDamping;
	for (uint8_t i = 0; i < 4; ++i) {
		cv.filter_damping[i] = filter.iirLPFilter[0].damping[i];
	}
}


// Restore configuration settings from flash memory
void Config::RestoreConfig()
{
	// create temporary copy of settings from memory to check if they are valid
	configValues cv;
	memcpy(reinterpret_cast<uint32_t*>(&cv), ADDR_FLASH_SECTOR_7, sizeof(cv));

	if (strcmp(cv.StartMarker, "CFG") == 0 && strcmp(cv.EndMarker, "END") == 0 && cv.Version == CONFIG_VERSION) {
		adcZeroOffset[left]  = cv.audio_offset_left;
		adcZeroOffset[right] = cv.audio_offset_right;
		delay.gateThreshold = cv.delay_gate_threshold;
		delay.gateHoldCount = cv.delay_gate_activate;
		delay.modOffsetMax = cv.delay_modOffsetMax;
		delay.modOffsetInc = cv.delay_modOffsetInc;
		filter.potCentre = cv.filter_pot_center;
		filter.firTaps = cv.filter_fir_taps;

		if (cv.filter_num_poles != filter.defaultPoles && cv.filter_num_poles > 0 && cv.filter_num_poles < MAX_POLES) {
			filter.CustomiseIIR(cv.filter_num_poles);
		}
		filter.customDamping = cv.filter_custom_damping;
		if (filter.customDamping) {
			for (uint8_t i = 0; i < 4; ++i) {
				filter.CustomiseIIR(i, cv.filter_damping[i]);
			}
		}
	}

	// Set up averaging values for ongoing ADC offset calibration
	newOffset[0] = adcZeroOffset[0];
	newOffset[1] = adcZeroOffset[1];
}


// Unlock the FLASH control register access
void __attribute__((section(".itcm_text"))) Config::FlashUnlock(uint8_t bank)
{
	volatile uint32_t* bankCR  = &(bank == 1 ? FLASH->CR1 : FLASH->CR2);
	volatile uint32_t* bankKEY = &(bank == 1 ? FLASH->KEYR1 : FLASH->KEYR2);

	if ((*bankCR & FLASH_CR_LOCK) != 0)  {
		*bankKEY = 0x45670123U;					// These magic numbers unlock the bank for programming
		*bankKEY = 0xCDEF89ABU;
	}
}


// Lock the FLASH Bank Registers access
void __attribute__((section(".itcm_text"))) Config::FlashLock(uint8_t bank)
{
	if (bank == 1) {
		FLASH->CR1 |= FLASH_CR_LOCK;
	} else {
		FLASH->CR2 |= FLASH_CR_LOCK;
	}
}


// Erase sector - FLASH_CR_PSIZE_1 means a write size of 32bits (see p163 of manual)
void __attribute__((section(".itcm_text"))) Config::FlashEraseSector(uint8_t Sector, uint32_t bank)
{
	volatile uint32_t* bankCR = &(bank == 1 ? FLASH->CR1 : FLASH->CR2);

	*bankCR &= ~(FLASH_CR_PSIZE | FLASH_CR_SNB);
	*bankCR |= (FLASH_CR_SER |					// Sector erase request
			FLASH_CR_PSIZE_1 |					// Write 32 bits at a time
			(Sector << FLASH_CR_SNB_Pos) |
			FLASH_CR_START);
}


bool __attribute__((section(".itcm_text"))) Config::FlashWaitForLastOperation(uint32_t bank)
{
    // Even if FLASH operation fails, the QW flag will be reset and an error flag will be set
	volatile uint32_t* bankSR  = &(bank == 1 ? FLASH->SR1 : FLASH->SR2);
	volatile uint32_t* bankCCR = &(bank == 1 ? FLASH->CCR1 : FLASH->CCR2);

	if (*bankSR & FLASH_ALL_ERRORS) {					// If any error occurred abort
		*bankSR = FLASH_ALL_ERRORS;						// Clear all errors
		return false;
	}

	while ((*bankSR & FLASH_SR_QW) == FLASH_SR_QW) {	// QW flag set when write or erase operation is pending in the command queue buffer

	}

	if ((*bankSR & FLASH_SR_EOP) == FLASH_SR_EOP) {		// Check End of Operation flag
		*bankCCR = FLASH_CCR_CLR_EOP;
	}

	return true;
}


bool __attribute__((section(".itcm_text"))) Config::FlashProgram(uint32_t* dest_addr, uint32_t* src_addr, size_t size)
{
	uint8_t bank = (reinterpret_cast<uintptr_t>(dest_addr) < FLASH_BANK2_BASE) ? 1 : 2;		// Get which bank we are programming from destination address
	volatile uint32_t* bankCR = &(bank == 1 ? FLASH->CR1 : FLASH->CR2);

	if (Config::FlashWaitForLastOperation(bank)) {
		*bankCR |= FLASH_CR_PG;

		__ISB();
		__DSB();

		// Each write block is 32 bytes
		for (uint16_t b = 0; b < std::ceil(static_cast<float>(size) / 32); ++b) {
			for (uint8_t i = 0; i < FLASH_NB_32BITWORD_IN_FLASHWORD; ++i) {
				*dest_addr = *src_addr;
				dest_addr++;
				src_addr++;
			}

			if (!Config::FlashWaitForLastOperation(bank)) {
				*bankCR &= ~FLASH_CR_PG;				// Clear programming flag
				return false;
			}
		}

		__ISB();
		__DSB();

		*bankCR &= ~FLASH_CR_PG;						// Clear programming flag
	}
	return true;
}


