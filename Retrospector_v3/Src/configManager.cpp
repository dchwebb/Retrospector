#include "configManager.h"
#include <cstring>
#include <cstdio>
#include <cmath>

bool Config::SaveConfig(const bool forceSave)
{
	bool result = true;
	if (forceSave || (scheduleSave && SysTickVal > saveBooked + 20000)) {			// 20 seconds between saves
		scheduleSave = false;

		if (currentSettingsOffset == -1) {					// Default = -1 if not first set in RestoreConfig
			currentSettingsOffset = 0;
		} else {
			currentSettingsOffset += settingsSize;
			if ((uint32_t)currentSettingsOffset > flashSectorSize - settingsSize) {
				// Check if another sector is available for use
				for (auto& sector : sectors) {
					if (!sector.dirty && sector.sector != currentSector) {
						if (++currentIndex > configSectorCount) { 	// set new index, wrapping at count of allowed config sectors
							currentIndex = 0;
						}
						currentSector = sector.sector;
						flashConfigAddr = SetCurrentConfigAddr(currentSector, flashConfigBank);
						currentSettingsOffset = 0;
						sector.index = currentIndex;
						sector.dirty = true;
						break;
					}
				}
				if (currentSettingsOffset != 0) {				// No free configurations slots found - abort save
					printf("Error saving config - no space\r\n");
					return false;
				}
			}
		}
		uint32_t* flashPos = flashConfigAddr + currentSettingsOffset / 4;


		uint8_t configBuffer[settingsSize];					// Will hold all the data to be written by config savers
		memcpy(configBuffer, ConfigHeader, 4);
		configBuffer[4] = currentIndex;						// Store the index of the config to identify the active sector
		uint32_t configPos = headerSize;
		for (auto& saver : configSavers) {					// Add individual config settings to buffer after header
			memcpy(&configBuffer[configPos], saver->settingsAddress, saver->settingsSize);
			configPos += saver->settingsSize;
		}

		FlashUnlock(GetBank(flashPos));						// Unlock Flash memory for writing
		FLASH->CCR1 = flashAllErrors;						// Clear error flags in Status Register
		result = FlashProgram(flashPos, reinterpret_cast<uint32_t*>(&configBuffer), settingsSize);
		FlashLock(flashConfigBank);							// Lock Flash

		if (result) {
			printf("Config Saved (%lu bytes at %#010lx)\r\n", settingsSize, (uint32_t)flashPos);
		} else {
			printf("Error saving config\r\n");
		}
	}
	return result;
}


void Config::RestoreConfig()
{
	// Initialise sector array - used to manage which sector contains current config, and which sectors are available for writing when current sector full
	for (uint32_t i = 0; i < configSectorCount; ++i) {
		sectors[i].sector = flashConfigSector + i;
		uint32_t* const addr = flashConfigAddr + i * (flashSectorSize / 4);

		// Check if sector is dirty
		for (uint32_t w = 0; w < (flashSectorSize / 4); ++w) {
			if (addr[w] != 0xFFFFFFFF) {
				sectors[i].dirty = true;
				break;
			}
		}

		// Check if there is a config block at the start of the sector and read the index number if so
		sectors[i].index = (addr[0] == *(uint32_t*)ConfigHeader) ? (uint8_t)addr[1] : 255;
	}

	// Work out which is the active config sector: will be the highest index from the bottom before the sequence jumps
	std::sort(sectors.begin(), sectors.end(), [](const CfgSector& l, const CfgSector& r) { return l.index < r.index; });
	uint32_t index = sectors[0].index;
	if (index == 255) {
		currentSector = flashConfigSector;
		currentIndex = 0;					// Each sector is assigned an index to determine which contains the latest config
	} else {
		currentSector = sectors[0].sector;
		for (uint32_t i = 1; i < configSectorCount; ++i) {
			if (sectors[i].index == index + 1) {
				++index;
				currentSector = sectors[i].sector;
			} else {
				break;
			}
		}
		currentIndex = index;
	}
	flashConfigAddr= SetCurrentConfigAddr(currentSector, flashConfigBank);				// Set the base address of the sector holding the current config

	// Erase any dirty sectors that are not the current one, or do not contain config data
	for (auto& sector : sectors) {
		if (sector.dirty && (sector.sector != currentSector || sector.index == 255)) {
			FlashEraseSector(sector.sector, flashConfigBank);
			sector.index = 255;
			sector.dirty = false;
		}
	}
	std::sort(sectors.begin(), sectors.end(), [](const CfgSector& l, const CfgSector& r) { return l.sector < r.sector; });

	// Locate latest (active) config block
	uint32_t pos = 0;
	while (pos <= flashSectorSize - settingsSize) {
		if (*(flashConfigAddr + pos / 4) == *(uint32_t*)ConfigHeader) {
			currentSettingsOffset = pos;
			pos += settingsSize;
		} else {
			break;			// Either reached the end of the sector or found the latest valid config block
		}
	}

	if (currentSettingsOffset >= 0) {
		const uint8_t* flashConfig = reinterpret_cast<uint8_t*>(flashConfigAddr) + currentSettingsOffset;
		uint32_t configPos = headerSize;		// Position in buffer to retrieve settings from

		// Restore settings
		for (auto saver : configSavers) {
			memcpy(saver->settingsAddress, &flashConfig[configPos], saver->settingsSize);
			if (saver->validateSettings != nullptr) {
				saver->validateSettings();
			}
			configPos += saver->settingsSize;
		}
	} else {		// If no config stored run update settings to initialise config across components
		for (auto saver : configSavers) {
			if (saver->validateSettings != nullptr) {
				saver->validateSettings();
			}
		}
	}

}


void Config::EraseConfig()
{
	for (uint32_t i = 0; i < configSectorCount; ++i) {
		FlashEraseSector(flashConfigSector, flashConfigBank);
		sectors[i].dirty = false;
		sectors[i].index = 255;
	}

	printf("Config Erased\r\n");
}


void Config::ScheduleSave()
{
	// called whenever a config setting is changed to schedule a save after waiting to see if any more changes are being made
	scheduleSave = true;
	saveBooked = SysTickVal;
}


void Config::FlashUnlock(uint8_t bank)
{
	volatile uint32_t* bankCR  = &(bank == 1 ? FLASH->CR1 : FLASH->CR2);
	volatile uint32_t* bankKEY = &(bank == 1 ? FLASH->KEYR1 : FLASH->KEYR2);

	// Unlock the FLASH control register access
	if ((*bankCR & FLASH_CR_LOCK) != 0)  {
		*bankKEY = 0x45670123U;						// These magic numbers unlock the flash for programming
		*bankKEY = 0xCDEF89ABU;
	}
}


void Config::FlashLock(uint8_t bank)
{
	if (bank == 1) {
		FLASH->CR1 |= FLASH_CR_LOCK;
	} else {
		FLASH->CR2 |= FLASH_CR_LOCK;
	}
}


//void Config::FlashEraseSector(uint8_t sector)
//{
//	FlashUnlock(flashConfigBank);										// Unlock Flash memory for writing
//	FLASH->CCR1 = flashAllErrors;						// Clear error flags in Status Register
//
//	FLASH->CR1 &= ~FLASH_CR_SNB_Msk;
//	FLASH->CR1 |= (sector - 1) << FLASH_CR_SNB_Pos;		// Sector number selection
//	FLASH->CR1 |= FLASH_CR_SER;							// Sector erase
//	FLASH->CR1 |= FLASH_CR_START;
//	FlashWaitForLastOperation();
//	FLASH->CR1 &= ~FLASH_CR_SER;
//
//	FlashLock();										// Lock Flash
//}


void Config::FlashEraseSector(uint8_t sector, uint8_t bank)
{
	volatile uint32_t* bankCR = &(bank == 1 ? FLASH->CR1 : FLASH->CR2);

	FlashUnlock(bank);									// Unlock Flash memory for writing
	FLASH->CCR1 = flashAllErrors;						// Clear error flags in Status Register

	*bankCR &= ~(FLASH_CR_PSIZE | FLASH_CR_SNB);
	*bankCR |= (FLASH_CR_SER |							// Sector erase request
			FLASH_CR_PSIZE_1 |							// Write 32 bits at a time
			((sector - 0) << FLASH_CR_SNB_Pos) |
			FLASH_CR_START);

	FlashWaitForLastOperation(bank);
	*bankCR &= ~FLASH_CR_SER;

	FlashLock(bank);									// Lock Flash

}

/*
bool Config::FlashWaitForLastOperation()
{
	if (FLASH->SR1 & flashAllErrors) {					// If any error occurred abort
		FLASH->CCR1 = flashAllErrors;					// Clear error flags in Status Register
		return false;
	}

	while ((FLASH->SR1 & FLASH_SR_BSY) == FLASH_SR_BSY) {}

	if ((FLASH->SR1 & FLASH_SR_EOP) == FLASH_SR_EOP) {	// Check End of Operation flag
		FLASH->SR1 = FLASH_SR_EOP;						// Clear FLASH End of Operation pending bit
	}

	return true;
}
*/

bool Config::FlashWaitForLastOperation(uint8_t bank)
{
    // Even if FLASH operation fails, the QW flag will be reset and an error flag will be set
	volatile uint32_t* bankSR  = &(bank == 1 ? FLASH->SR1 : FLASH->SR2);
	volatile uint32_t* bankCCR = &(bank == 1 ? FLASH->CCR1 : FLASH->CCR2);

	if (*bankSR & flashAllErrors) {					// If any error occurred abort
		*bankSR = flashAllErrors;						// Clear all errors
		return false;
	}

	while ((*bankSR & FLASH_SR_QW) == FLASH_SR_QW) {	// QW flag set when write or erase operation is pending in the command queue buffer

	}

	if ((*bankSR & FLASH_SR_EOP) == FLASH_SR_EOP) {		// Check End of Operation flag
		*bankCCR = FLASH_CCR_CLR_EOP;
	}

	return true;
}


bool Config::FlashProgram(uint32_t* dest_addr, uint32_t* src_addr, size_t size)
{
	uint8_t bank = (reinterpret_cast<uintptr_t>(dest_addr) < FLASH_BANK2_BASE) ? 1 : 2;		// Get which bank we are programming from destination address
	volatile uint32_t* bankCR = &(bank == 1 ? FLASH->CR1 : FLASH->CR2);

	if (!FlashWaitForLastOperation(bank)) {
		return false;
	}
	*bankCR |= FLASH_CR_PG;

	__ISB();
	__DSB();

	// Each write block is up to 256 bits
	for (uint16_t b = 0; b < std::ceil(static_cast<float>(size) / 32); ++b) {
		for (uint8_t i = 0; i < 8; ++i) {
			*dest_addr = *src_addr;
			++dest_addr;
			++src_addr;
		}

		if (!FlashWaitForLastOperation(bank)) {
			*bankCR &= ~FLASH_CR_PG;				// Clear programming flag
			return false;
		}
	}

	__ISB();
	__DSB();

	*bankCR &= ~FLASH_CR_PG;						// Clear programming flag
	return true;
}


