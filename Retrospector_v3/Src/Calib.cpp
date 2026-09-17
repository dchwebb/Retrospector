#include <Calib.h>
#include <cstdio>
#include "filter.h"
#include "configManager.h"

void Calib::Calibrate()
{
	printf("Calibrating ...\r\n");

	int32_t audioOffsetL = ADC_array[left];
	int32_t audioOffsetR = ADC_array[right];
	int32_t filterCenter = ADC_array[ADC_Filter_Pot];

	for (int32_t i = 0; i < 10000000; ++i) {
		audioOffsetL = std::round((static_cast<float>(ADC_array[left]) + (63.0f * audioOffsetL)) / 64.0f);
		audioOffsetR = std::round((static_cast<float>(ADC_array[right]) + (63.0f * audioOffsetR)) / 64.0f);
		filterCenter = std::round((static_cast<float>(ADC_array[ADC_Filter_Pot]) + (63.0f * filterCenter)) / 64.0f);
	}

	printf("Calibration Audio L: %ld; Audio R: %ld; Filter: %ld\r\n"
		   "Old         Audio L: %ld; Audio R: %ld; Filter: %ld\r\n",
			audioOffsetL,
			audioOffsetR,
			filterCenter,
			cfg.adcZeroOffset[left],
			cfg.adcZeroOffset[right],
			(int32_t)filter.cfg.potCentre
			);

	if (audioOffsetL < 33000 || audioOffsetL > 34500) {
		printf("Calibration failed. Audio L out of range\r\n");
		return;
	}
	if (audioOffsetR < 33000 || audioOffsetR > 34500) {
		printf("Calibration failed. Audio R out of range\r\n");
		return;
	}
	if (filterCenter < 28000 || filterCenter > 36000) {
		printf("Calibration failed. Filter pot out of range\r\n");
		return;
	}

	// If calibration in acceptable range save to config
	cfg.adcZeroOffset[left] = audioOffsetL;
	cfg.adcZeroOffset[right] = audioOffsetR;
	filter.cfg.potCentre = filterCenter;
	config.SaveConfig(true);
}


void Calib::AutoZeroOffset()
{
	// When silence is detected for a long enough time recalculate ADC offset
	for (channel lr : {left, right}) {
		if (ADC_array[lr] > cfg.adcZeroOffset[lr] - 500 && ADC_array[lr] < cfg.adcZeroOffset[lr] + 500) {
			newOffset[lr] = (static_cast<float>(ADC_array[lr]) + (799.0f * newOffset[lr])) / 800.0f;
			if (offsetCounter[lr] == 200000) {
				if (newOffset[lr] > cfg.adcZeroOffset[lr] + 10) {
					cfg.adcZeroOffset[lr]++;
				} else if (newOffset[lr] < cfg.adcZeroOffset[lr] - 10) {
					cfg.adcZeroOffset[lr]--;
				}
				offsetCounter[lr] = 0;
			}
			offsetCounter[lr]++;
		} else {
			offsetCounter[lr] = 0;
		}
	}
}

void Calib::UpdateConfig()
{

}
