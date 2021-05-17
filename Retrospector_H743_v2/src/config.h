#pragma once

#include "initialisation.h"
#include "DigitalDelay.h"
#include "USB.h"
#include "SerialHandler.h"

#define ADDR_FLASH_SECTOR_7		reinterpret_cast<uint32_t*>(0x081E0000) // Base address of Bank 2 Sector 7, 128 Kbytes
#define CONFIG_VERSION 6
#define FLASH_ALL_ERRORS (FLASH_SR_WRPERR | FLASH_SR_PGSERR | FLASH_SR_STRBERR | FLASH_SR_INCERR | FLASH_SR_OPERR | FLASH_SR_RDPERR | FLASH_SR_RDSERR | FLASH_SR_SNECCERR | FLASH_SR_DBECCERR | FLASH_SR_CRCRDERR)

extern USB usb;


struct configValues {
	char StartMarker[4] = "CFG";		// Start Marker

	//	General settings
	uint8_t Version = CONFIG_VERSION;				// version of saved config struct format

	// Settings
	uint16_t audio_offset_left = 0;
	uint16_t audio_offset_right = 0;
	uint16_t delay_gate_threshold = 0;
	uint16_t delay_gate_activate = 0;
	float delay_modOffsetMax = 0.0f;
	float delay_modOffsetInc = 0.0f;


	uint16_t filter_pot_center = 0;
	uint8_t  filter_fir_taps = 0;
	uint8_t  filter_num_poles = 0;
	bool filter_custom_damping = false;
	float filter_damping[4] = {0.0, 0.0, 0.0, 0.0};

	char EndMarker[4] = "END";			// End Marker
};


// Class used to store calibration settings - note this uses the Standard Peripheral Driver code
class Config {
public:
	bool scheduleSave = false;
	uint32_t saveBooked;

	void Calibrate();
	void AutoZeroOffset();				// Automatically adjusts ADC zero offset by averaging low level signals
	void ScheduleSave();				// called whenever a config setting is changed to schedule a save after waiting to see if any more changes are being made
	bool SaveConfig();
	void SetConfig(configValues &cv);	// sets properties of class to match current values
	void RestoreConfig();				// gets config from Flash, checks and updates settings accordingly

	void FlashUnlock(uint8_t bank);
	void FlashLock(uint8_t bank);
	void FlashEraseSector(uint8_t Sector, uint32_t bank);
	bool FlashWaitForLastOperation(uint32_t bank);
	bool FlashProgram(uint32_t* dest_addr, uint32_t* src_addr, size_t size);
private:
	float newOffset[2] = {ADC_OFFSET_DEFAULT, ADC_OFFSET_DEFAULT};
	uint32_t offsetCounter[2];

};

