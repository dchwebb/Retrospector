#pragma once

#include "initialisation.h"
#include "configManager.h"

#define ADC_OFFSET_DEFAULT 33800

class Calib {
public:
	void Calibrate();
	void AutoZeroOffset();
	static void UpdateConfig();

	struct {
		int32_t adcZeroOffset[2] = {ADC_OFFSET_DEFAULT, ADC_OFFSET_DEFAULT};
	} cfg;

	ConfigSaver configSaver = {
		.settingsAddress = &cfg,
		.settingsSize = sizeof(cfg),
		.validateSettings = UpdateConfig
	};

	bool calibrating;			// Triggered by serial console
	enum class State {Waiting0, Waiting1, Octave0, Octave1, PendingSave};
	State state;
private:
	float newOffset[2] = {ADC_OFFSET_DEFAULT, ADC_OFFSET_DEFAULT};
	uint32_t offsetCounter[2];
};

extern Calib calib;
