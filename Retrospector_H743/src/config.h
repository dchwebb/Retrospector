#pragma once

#include "initialisation.h"
#include "DigitalDelay.h"

#define ADDR_FLASH_SECTOR_7		((uint32_t)0x081E0000) // Base address of Bank 2 Sector 7, 128 Kbytes



struct configValues {
	char StartMarker[4] = "CFG";		// Start Marker

	//	General settings
	uint8_t Version = 1;				// version of saved config struct format

	// Settings
	int16_t filter_pot_center = 0;

	char EndMarker[4] = "END";			// End Marker
};


// Class used to store calibration settings - note this uses the Standard Peripheral Driver code
class Config {
public:
	bool scheduleSave = false;
	uint32_t saveBooked;

	void ScheduleSave();				// called whenever a config setting is changed to schedule a save after waiting to see if any more changes are being made
	void SaveConfig();
	void SetConfig(configValues &cv);	// sets properties of class to match current values
	void RestoreConfig();				// gets config from Flash, checks and updates settings accordingly

};

