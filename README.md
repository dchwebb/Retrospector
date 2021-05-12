# Retrospector
![Image](https://github.com/dchwebb/Retrospector/raw/master/pictures/retrospector.png "icon")

Retrospector is a Eurorack format stereo digital delay. 

Delay times can be set clocked or free-running and left and right channels can be independent or the right channel set as a multiple of the left channel. The delay times are switchable between short, long and reversed with maximum delay times over 5 minutes.

The feedback path can be filtered with settings allowing a sweep from low pass to high pass, or either separately. The delayed signal can be modulated for subtle chorussing effects or left and right channels fed back into each other to widen dual mono sources.

The feedback control allows greater than 100% regeneration allowing the delay to increase the volume of the delayed signal.

A USB port is available on the rear of the module providing configuration, debug and upgrade options via a serial interface.

Technical
![Image](https://github.com/dchwebb/Retrospector/raw/master/pictures/components.png "icon")

A) A switched-mode power supply (Texas Instruments LM2675) converts the 12V rail to 5V which is then converted to 3.3V using an LDO. THe 5V rail also powers the RGB LEDs.

B) The mix control uses a 4 channel linear VCA (Alfa AS3364D) to control the Wet/Dry mix. This keeps the dry signal analog throughout.

C) The Output signals are mixed and inverted (to preserve signal polarity) with a TL074 and a TL072 op-amp.

D) The digital Wet signal is converted to analog using a Texas Instruments PCM5100 DAC.

E) The analog input signals (audio and control voltage) are scaled and mixed using a pair of MCP6004 op-amps.

F) The microcontroller is an STM32H743 running at 400MHz.

G) The delay beffers are stored in an external 32MB RAM module (ISSI IS42S16160J) allowing for over 5 minutes of stereo delay.