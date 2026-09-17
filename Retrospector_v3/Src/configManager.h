#pragma once

#include "initialisation.h"
#include <vector>
#include <array>

// Struct added to classes that need settings saved
struct ConfigSaver {
	void* settingsAddress;
	uint32_t settingsSize;
	void (*validateSettings)(void);		// function pointer to method that will validate config settings when restored
};


class Config {
	friend class CDCHandler;					// Allow the serial handler access to private data for printing
public:
	static constexpr uint8_t configVersion = 5;
	
	// STM32H743 has 2048k Flash in 2 banks each with 8 sectors of 128k
	static constexpr uint32_t flashConfigBank = 2;			// Each of 2 banks has 8 sectors
	static constexpr uint32_t flashConfigSector = 7;		// Allow 1 sector for config giving a config size of 128k before erase needed
	static constexpr uint32_t flashSectorSize = 131072;
	static constexpr uint32_t configSectorCount = 1;		// Number of sectors after base sector used for config
	uint32_t* flashConfigAddr =	SetCurrentConfigAddr(flashConfigSector, flashConfigBank);

	// Constructor taking multiple config savers: Get total config block size from each saver
	Config(std::initializer_list<ConfigSaver*> initList) : configSavers(initList) {
		for (auto saver : configSavers) {
			settingsSize += saver->settingsSize;
		}
		// Ensure config size (+ 4 byte header + 1 byte index) is aligned to 32 byte boundary
		settingsSize = AlignToBytes(settingsSize + headerSize, 32);
	}

	void ScheduleSave();				// called whenever a config setting is changed to schedule a save after waiting to see if any more changes are being made
	bool SaveConfig(const bool forceSave = false);
	void EraseConfig();					// Erase flash page containing config
	void RestoreConfig();				// gets config from Flash, checks and updates settings accordingly

private:
	static constexpr uint32_t flashAllErrors = FLASH_CCR_CLR_WRPERR | FLASH_CCR_CLR_PGSERR | FLASH_CCR_CLR_STRBERR | FLASH_CCR_CLR_INCERR | FLASH_CCR_CLR_RDPERR | FLASH_CCR_CLR_RDSERR | FLASH_CCR_CLR_SNECCERR | FLASH_CCR_CLR_DBECCERR | FLASH_CCR_CLR_CRCEND | FLASH_CCR_CLR_CRCRDERR;

	bool scheduleSave = false;
	uint32_t saveBooked = false;

	const std::vector<ConfigSaver*> configSavers;
	uint32_t settingsSize = 0;			// Size of all settings from each config saver module + size of config header

	const char ConfigHeader[4] = {'C', 'F', 'G', configVersion};
	static constexpr uint32_t headerSize = sizeof(ConfigHeader) + 1;
	int32_t currentSettingsOffset = -1;	// Offset within flash page to block containing active/latest settings

	uint32_t currentIndex = 0;			// Each config gets a new index to track across multiple sectors
	uint32_t currentSector = flashConfigSector;			// Sector containing current config
	struct CfgSector {
		uint32_t sector;
		uint8_t index;
		bool dirty;
	};
	std::array<CfgSector, configSectorCount> sectors;

	uint32_t* const SetCurrentConfigAddr(uint32_t sector, uint32_t bank) {
		return reinterpret_cast<uint32_t* const>((bank == 1 ? FLASH_BANK1_BASE : FLASH_BANK2_BASE) + flashSectorSize * sector);
	}
	uint8_t GetBank(uint32_t* addr) {		// Get which bank we are programming from destination address
		return (reinterpret_cast<uintptr_t>(addr) < FLASH_BANK2_BASE) ? 1 : 2;
	}


	void FlashUnlock(uint8_t bank);
	void FlashLock(uint8_t bank);
	void FlashEraseSector(uint8_t sector, uint8_t bank);
	bool FlashWaitForLastOperation(uint8_t bank);
	bool FlashProgram(uint32_t* dest_addr, uint32_t* src_addr, size_t size);

	// Aligns to a power of two boundary - eg val = 37, bytes = 32, returns 64
	static const inline uint32_t AlignToBytes(uint32_t val, uint32_t bytes) {
		val += bytes - 1;
		val &= ~(bytes - 1);
		return std::max(val, bytes);
	}
};

extern Config config;
