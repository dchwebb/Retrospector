#include <config.h>

#define FLASH_KEY1                 0x45670123U
#define FLASH_KEY2                 0xCDEF89ABU
#define FLASH_OPT_KEY1             0x08192A3BU
#define FLASH_OPT_KEY2             0x4C5D6E7FU

#define FLASH_BANK_1             0x01U                         /*!< Bank 1   */
#define FLASH_BANK_2             0x02U                         /*!< Bank 2   */
#define FLASH_BANK_BOTH          (FLASH_BANK_1 | FLASH_BANK_2) /*!< Bank1 and Bank2 */

#define FLASH_VOLTAGE_RANGE_1        0x00000000U       /*!< Flash program/erase by 8 bits  */
#define FLASH_VOLTAGE_RANGE_2        FLASH_CR_PSIZE_0  /*!< Flash program/erase by 16 bits */
#define FLASH_VOLTAGE_RANGE_3        FLASH_CR_PSIZE_1  /*!< Flash program/erase by 32 bits */
#define FLASH_VOLTAGE_RANGE_4        FLASH_CR_PSIZE    /*!< Flash program/erase by 64 bits */

#define FLASH_SECTOR_0             0U       /*!< Sector Number 0   */
#define FLASH_SECTOR_1             1U       /*!< Sector Number 1   */
#define FLASH_SECTOR_2             2U       /*!< Sector Number 2   */
#define FLASH_SECTOR_3             3U       /*!< Sector Number 3   */
#define FLASH_SECTOR_4             4U       /*!< Sector Number 4   */
#define FLASH_SECTOR_5             5U       /*!< Sector Number 5   */
#define FLASH_SECTOR_6             6U       /*!< Sector Number 6   */
#define FLASH_SECTOR_7             7U       /*!< Sector Number 7   */

#define FLASH_FLAG_BSY                     FLASH_SR_BSY             /*!< FLASH Busy flag */
#define FLASH_FLAG_WBNE                    FLASH_SR_WBNE            /*!< Write Buffer Not Empty flag */
#define FLASH_FLAG_QW                      FLASH_SR_QW              /*!< Wait Queue on flag */
#define FLASH_FLAG_CRC_BUSY                FLASH_SR_CRC_BUSY        /*!< CRC Busy flag */
#define FLASH_FLAG_EOP                     FLASH_SR_EOP             /*!< End Of Program on flag */
#define FLASH_FLAG_WRPERR                  FLASH_SR_WRPERR          /*!< Write Protection Error on flag */
#define FLASH_FLAG_PGSERR                  FLASH_SR_PGSERR          /*!< Program Sequence Error on flag */
#define FLASH_FLAG_STRBERR                 FLASH_SR_STRBERR         /*!< Strobe Error flag */
#define FLASH_FLAG_INCERR                  FLASH_SR_INCERR          /*!< Inconsistency Error on flag */
#define FLASH_FLAG_RDPERR                  FLASH_SR_RDPERR          /*!< Read Protection Error on flag */
#define FLASH_FLAG_RDSERR                  FLASH_SR_RDSERR          /*!< Read Secured Error on flag */
#define FLASH_FLAG_SNECCERR                FLASH_SR_SNECCERR        /*!< Single ECC Error Correction on flag */
#define FLASH_FLAG_DBECCERR                FLASH_SR_DBECCERR        /*!< Double Detection ECC Error on flag */
#define FLASH_FLAG_CRCEND                  FLASH_SR_CRCEND          /*!< CRC End of Calculation flag */
#define FLASH_FLAG_CRCRDERR                FLASH_SR_CRCRDERR        /*!< CRC Read Error on bank flag */
#define FLASH_ALL_ERRORS (FLASH_SR_WRPERR | FLASH_SR_PGSERR | FLASH_SR_STRBERR | FLASH_SR_INCERR | FLASH_SR_OPERR | FLASH_SR_RDPERR | FLASH_SR_RDSERR | FLASH_SR_SNECCERR | FLASH_SR_DBECCERR | FLASH_SR_CRCRDERR)

// Unlock the FLASH control register access
void FLASH_Unlock(uint8_t bank)
{
	volatile uint32_t* bankCR  = &(bank == FLASH_BANK_1 ? FLASH->CR1 : FLASH->CR2);
	volatile uint32_t* bankKEY = &(bank == FLASH_BANK_1 ? FLASH->KEYR1 : FLASH->KEYR2);

	if ((*bankCR & FLASH_CR_LOCK) != 0)  {
		*bankKEY = FLASH_KEY1;
		*bankKEY = FLASH_KEY2;
	}

	if (bank == FLASH_BANK_1) {
		if ((FLASH->CR1 & FLASH_CR_LOCK) != 0)  {
			FLASH->KEYR1 = FLASH_KEY1;
			FLASH->KEYR1 = FLASH_KEY2;
		}
	} else {
		if ((FLASH->CR2 & FLASH_CR_LOCK) != 0) {
			FLASH->KEYR2 = FLASH_KEY1;
			FLASH->KEYR2 = FLASH_KEY2;
		}
	}
}


// Lock the FLASH Bank Registers access
void FLASH_Lock(uint8_t bank)
{
	if (bank == FLASH_BANK_1) {
		FLASH->CR1 |= FLASH_CR_LOCK;
	} else {
		FLASH->CR2 |= FLASH_CR_LOCK;
	}
}


// Erase sector - FLASH_CR_PSIZE_1 means a write size of 32bits (see p163 of manual)
void FLASH_EraseSector(uint32_t Sector, uint32_t bank)
{
	if (bank == FLASH_BANK_1) {
		FLASH->CR1 &= ~(FLASH_CR_PSIZE | FLASH_CR_SNB);
		FLASH->CR1 |= (FLASH_CR_SER | FLASH_CR_PSIZE_1 | (Sector << FLASH_CR_SNB_Pos) | FLASH_CR_START);
	}

	if (bank == FLASH_BANK_2) {
		FLASH->CR2 &= ~(FLASH_CR_PSIZE | FLASH_CR_SNB);
		FLASH->CR2 |= (FLASH_CR_SER | FLASH_CR_PSIZE_1  | (Sector << FLASH_CR_SNB_Pos) | FLASH_CR_START);
	}
}



bool FLASH_WaitForLastOperation(uint32_t Timeout, uint32_t bank)
{
	// Wait for the FLASH operation to complete by polling on QW flag to be reset.
    // Even if the FLASH operation fails, the QW flag will be reset and an error flag will be set

	uint32_t tickstart = SysTickVal;
	volatile uint32_t* bankSR = &(bank == FLASH_BANK_1 ? FLASH->SR1 : FLASH->SR2);

	if (bank == FLASH_BANK_1) {
		if (FLASH->SR1 & FLASH_ALL_ERRORS) {
			FLASH->SR1 = FLASH_ALL_ERRORS;
			return false;
		}
	} else {
		if (FLASH->SR2 & FLASH_ALL_ERRORS) {
			FLASH->SR2 = FLASH_ALL_ERRORS;
			return false;
		}
	}

	// Fixme timeout for bank 1
	while ((FLASH->SR2 & FLASH_SR_QW) == FLASH_SR_QW) {
		if ((SysTickVal - tickstart) > Timeout) {
			return false;
		}
	}

	// Check FLASH End of Operation flag
	if (bank == FLASH_BANK_1) {
		if ((FLASH->SR1 & FLASH_SR_EOP) == FLASH_SR_EOP) {
			FLASH->CCR1 = FLASH_CCR_CLR_EOP;
		}
	} else {
		if ((FLASH->SR2 & FLASH_SR_EOP) == FLASH_SR_EOP) {
			FLASH->CCR2 = FLASH_CCR_CLR_EOP;
		}
	}

	return true;
}

//  Program flash word (4 * 32 bites) at a specified address
void FLASH_Program(uint32_t FlashAddress, uint32_t DataAddress)
{
	volatile uint32_t* dest_addr = (volatile uint32_t*)FlashAddress;
	volatile uint32_t* src_addr = (volatile uint32_t*)DataAddress;
	uint32_t bank;
	uint8_t row_index = FLASH_NB_32BITWORD_IN_FLASHWORD;

	if (FlashAddress < FLASH_BANK2_BASE) {
		bank = FLASH_BANK_1;
	} else {
		bank = FLASH_BANK_2;
	}

	if (FLASH_WaitForLastOperation(10, bank)) {			// FIXME guess a timeout of 10
		if (bank == FLASH_BANK_1) {
			FLASH->CR1 |= FLASH_CR_PG;
		} else {
			FLASH->CR2 |= FLASH_CR_PG;
		}

		__ISB();
		__DSB();

		// Program the flash word (8 * 32 bits)
		do {
			*dest_addr = *src_addr;
			dest_addr++;
			src_addr++;
			row_index--;
		} while (row_index != 0U);

		__ISB();
		__DSB();

		FLASH_WaitForLastOperation(10, bank);

		if (bank == FLASH_BANK_1) {
			FLASH->CR1 &= ~FLASH_CR_PG;
		} else {
			FLASH->CR2 &= ~FLASH_CR_PG;
		}

	}

}

// called whenever a config setting is changed to schedule a save after waiting to see if any more changes are being made
void Config::ScheduleSave() {
	scheduleSave = true;
	saveBooked = SysTickVal;
}

// Write calibration settings to Flash memory (H743 see programming manual p152 for sector layout)
void Config::SaveConfig() {
	scheduleSave = false;

	configValues cv;
	SetConfig(cv);

	__disable_irq();					// Disable Interrupts
	FLASH_Unlock(FLASH_BANK_2);			// Unlock Flash memory for writing
	FLASH->SR2 = FLASH_ALL_ERRORS;		// Clear error flags in Status Register

	// Erase sector 7 (has to be erased before write - this sets all bits to 1 as write can only switch to 0)
	FLASH_EraseSector(FLASH_SECTOR_7, FLASH_BANK_2);

	FLASH_Program(ADDR_FLASH_SECTOR_7, (uint32_t)&cv);

	FLASH_Lock(FLASH_BANK_2);			// Lock the Flash memory
	__enable_irq(); 					// Enable Interrupts
}


void Config::SetConfig(configValues &cv) {
	cv.filter_pot_center = filter.filterPotCentre;
}


// Restore configuration settings from flash memory
void Config::RestoreConfig()
{
	// create temporary copy of settings from memory to check if they are valid
	configValues cv;
	memcpy(&cv, (uint32_t*)ADDR_FLASH_SECTOR_7, sizeof(cv));

	if (strcmp(cv.StartMarker, "CFG") == 0 && strcmp(cv.EndMarker, "END") == 0 && cv.Version == 2) {
		filter.filterPotCentre = cv.filter_pot_center;
	}
}


