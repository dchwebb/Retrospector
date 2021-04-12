#pragma once

#include "initialisation.h"
#include "DigitalDelay.h"
#include "USB.h"

#define ADDR_FLASH_SECTOR_7		reinterpret_cast<uint32_t*>(0x081E0000) // Base address of Bank 2 Sector 7, 128 Kbytes

extern USB usb;

struct configValues {
	char StartMarker[4] = "CFG";		// Start Marker

	//	General settings
	uint8_t Version = 1;				// version of saved config struct format

	// Settings
	int16_t filter_pot_center = 0;
	int16_t audio_offset_left = 0;
	int16_t audio_offset_right = 0;
	uint32_t test_value1 = 0xABBACAFE;
	uint32_t test_value2 = 0xDEADBABE;
	uint32_t test_value3 = 0xCEDECAFE;
	uint32_t test_value4 = 0xACEDABBA;
	uint32_t test_value5 = 0xABBACAFE;
	uint32_t test_value6 = 0xDEADBABE;
	uint32_t test_value7 = 0xCEDECAFE;
	uint32_t test_value8 = 0xACEDABBA;

	char EndMarker[4] = "END";			// End Marker
};


// Class used to store calibration settings - note this uses the Standard Peripheral Driver code
class Config {
public:
	bool scheduleSave = false;
	uint32_t saveBooked;

	void Calibrate();
	void ScheduleSave();				// called whenever a config setting is changed to schedule a save after waiting to see if any more changes are being made
	bool SaveConfig();
	void SetConfig(configValues &cv);	// sets properties of class to match current values
	void RestoreConfig();				// gets config from Flash, checks and updates settings accordingly
private:
	void FlashUnlock(uint8_t bank);
	void FlashLock(uint8_t bank);
	void FlashEraseSector(uint8_t Sector, uint32_t bank);
	bool FlashWaitForLastOperation(uint32_t Timeout, uint32_t bank);
	bool FlashProgram(uint32_t* dest_addr, uint32_t* src_addr, size_t size);
};

