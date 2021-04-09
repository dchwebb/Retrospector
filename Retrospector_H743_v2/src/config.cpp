#include <config.h>


#define FLASH_ALL_ERRORS (FLASH_SR_WRPERR | FLASH_SR_PGSERR | FLASH_SR_STRBERR | FLASH_SR_INCERR | FLASH_SR_OPERR | FLASH_SR_RDPERR | FLASH_SR_RDSERR | FLASH_SR_SNECCERR | FLASH_SR_DBECCERR | FLASH_SR_CRCRDERR)

// Unlock the FLASH control register access
void FLASH_Unlock(uint8_t bank)
{
	volatile uint32_t* bankCR  = &(bank == 1 ? FLASH->CR1 : FLASH->CR2);
	volatile uint32_t* bankKEY = &(bank == 1 ? FLASH->KEYR1 : FLASH->KEYR2);

	if ((*bankCR & FLASH_CR_LOCK) != 0)  {
		*bankKEY = 0x45670123U;					// These magic numbers unlock the bank for programming
		*bankKEY = 0xCDEF89ABU;
	}
}


// Lock the FLASH Bank Registers access
void FLASH_Lock(uint8_t bank)
{
	if (bank == 1) {
		FLASH->CR1 |= FLASH_CR_LOCK;
	} else {
		FLASH->CR2 |= FLASH_CR_LOCK;
	}
}


// Erase sector - FLASH_CR_PSIZE_1 means a write size of 32bits (see p163 of manual)
void FLASH_EraseSector(uint8_t Sector, uint32_t bank)
{
	volatile uint32_t* bankCR = &(bank == 1 ? FLASH->CR1 : FLASH->CR2);

	*bankCR &= ~(FLASH_CR_PSIZE | FLASH_CR_SNB);
	*bankCR |= (FLASH_CR_SER |					// Sector erase request
			FLASH_CR_PSIZE_1 |					// Write 32 bits at a time
			(Sector << FLASH_CR_SNB_Pos) |
			FLASH_CR_START);
}


bool FLASH_WaitForLastOperation(uint32_t Timeout, uint32_t bank)
{
    // Even if FLASH operation fails, the QW flag will be reset and an error flag will be set

	uint32_t tickstart = SysTickVal;
	volatile uint32_t* bankSR  = &(bank == 1 ? FLASH->SR1 : FLASH->SR2);
	volatile uint32_t* bankCCR = &(bank == 1 ? FLASH->CCR1 : FLASH->CCR2);

	if (*bankSR & FLASH_ALL_ERRORS) {					// If any error occurred abort
		*bankSR = FLASH_ALL_ERRORS;						// Clear all errors
		return false;
	}

	while ((*bankSR & FLASH_SR_QW) == FLASH_SR_QW) {	// QW flag set when write or erase operation is pending in the command queue buffer
		if ((SysTickVal - tickstart) > Timeout) {
			return false;
		}
	}

	if ((*bankSR & FLASH_SR_EOP) == FLASH_SR_EOP) {		// Check End of Operation flag
		*bankCCR = FLASH_CCR_CLR_EOP;
	}

	return true;
}


bool FLASH_Program(uint32_t* dest_addr, uint32_t* src_addr, size_t size)
{
	//uint8_t row_index = FLASH_NB_32BITWORD_IN_FLASHWORD;
	uint8_t bank = ((uintptr_t)dest_addr < FLASH_BANK2_BASE) ? 1 : 2;

	volatile uint32_t* bankCR  = &(bank == 1 ? FLASH->CR1 : FLASH->CR2);

	if (FLASH_WaitForLastOperation(10, bank)) {			// FIXME guess a timeout of 10
		*bankCR |= FLASH_CR_PG;

		__ISB();
		__DSB();

		// Each write block is 32 bytes
		for (uint8_t b = 0; b < std::ceil((float)size / 32); ++b) {

			// Program the flash word (8 * 32 bits)
			for (uint8_t i = 0; i < FLASH_NB_32BITWORD_IN_FLASHWORD; ++i) {
				*dest_addr = *src_addr;
				dest_addr++;
				src_addr++;
			}

			if (!FLASH_WaitForLastOperation(10, bank)) {
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

// called whenever a config setting is changed to schedule a save after waiting to see if any more changes are being made
void Config::ScheduleSave() {
	scheduleSave = true;
	saveBooked = SysTickVal;
}

// Write calibration settings to Flash memory (H743 see programming manual p152 for sector layout)
bool Config::SaveConfig() {
	scheduleSave = false;

	configValues cv;
	SetConfig(cv);

	__disable_irq();					// Disable Interrupts
	FLASH_Unlock(2);					// Unlock Flash memory for writing
	FLASH->SR2 = FLASH_ALL_ERRORS;		// Clear error flags in Status Register
	FLASH_EraseSector(7, 2);			// Erase sector 7, Bank 2
	bool result = FLASH_Program((uint32_t*)ADDR_FLASH_SECTOR_7, (uint32_t*)&cv, sizeof(cv));
	FLASH_Lock(2);						// Lock Bank 2 Flash
	__enable_irq(); 					// Enable Interrupts

	return result;
}


void Config::SetConfig(configValues &cv) {
	cv.filter_pot_center = filter.filterPotCentre;
}


// Restore configuration settings from flash memory
void Config::RestoreConfig()
{
	// create temporary copy of settings from memory to check if they are valid
	configValues cv;
	memcpy((uint32_t*)&cv, (uint32_t*)ADDR_FLASH_SECTOR_7, sizeof(cv));

	if (strcmp(cv.StartMarker, "CFG") == 0 && strcmp(cv.EndMarker, "END") == 0 && cv.Version == 2) {
		filter.filterPotCentre = cv.filter_pot_center;
	}
}


