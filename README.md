# Retrospector
![Image](https://github.com/dchwebb/Retrospector/raw/master/pictures/retrospector.png "icon")

Retrospector is a Eurorack format stereo digital delay. 

Delay times can be set clocked or free-running and left and right channels can be independent or the right channel set as a multiple of the left channel. The delay times are switchable between short, long and reversed with maximum delay times over 5 minutes.

The feedback path can be filtered with settings allowing a sweep from low pass to high pass, or either separately. The delayed signal can be modulated for subtle chorussing effects or left and right channels fed back into each other to widen dual mono sources.

The feedback control allows greater than 100% regeneration allowing the delay to increase the volume of the delayed signal.

Three RGB LEDs display the delay times (using different colours to indicate clocking and channel linking) and the filter status (white to red with increasing low pass filtering and white to blue for high pass).

A USB port is available on the rear of the module providing configuration, debug and upgrade options via a serial interface.

# Technical
The module is constructed from a sandwich of three PCBs. The front panel is held to the control PCB using the potentiometers and jack sockets with light guides directing the RGB LEDs from the control PCB. The control PCB also houses the LED driver. All other components are on the component PCB and this is attached to the control PCB with standard 2.54mm headers. The component board is a 4 layer PCB; the others have 2 layers.

Retrospector is built around an STM32H743 microcontroller clocked at 400MHz. The MCU uses a pair of multi-channel 16-bit ADCs to digitise the stereo audio input signal and the various potentiometer and control voltage inputs.

The MCU controls the external DAC with I2S at 16 bits. The MCU's internal 12-bit DACs are used to control the linear VCA that carries out stereo mixing of the digital wet signal and analog dry signal.

Delay samples are held in external 32MB SDRAM controlled by the MCU's FMC peripheral. The MCU's internal memory is mainly used for DMA buffers and FIR filter buffers (better performance than the slower external RAM). The MCU also has both data and instruction caches which are used to greatly improve performance.

The three RGB LEDs are controlled by a Toshiba 9 channel LED driver (TB62781FNG) with the MCU setting colour and brightness over SPI and DMA.

The MCU also supports a USB micro-B port to allow upgrades, configuration and debugging via a serial console:

![Image](https://github.com/dchwebb/Retrospector/raw/master/pictures/serial.png "icon")


Annotated component PCB
-----------------------
![Image](https://github.com/dchwebb/Retrospector/raw/master/pictures/components.png "icon")

A) A switched-mode power supply (Texas Instruments LM2675) converts the 12V rail to 5V which is then converted to 3.3V using an LDO. THe 5V rail also powers the RGB LEDs.

B) The mix control uses a 4 channel linear VCA (Alfa AS3364D) to control the Wet/Dry mix. This keeps the dry signal analog throughout.

C) The Output signals are mixed and inverted (to preserve signal polarity) with a TL074 and a TL072 op-amp.

D) The digital Wet signal is converted to analog using a Texas Instruments PCM5100 DAC.

E) The analog input signals (audio and control voltage) are scaled and mixed using a pair of MCP6004 op-amps.

F) The microcontroller is an STM32H743 running at 400MHz.

G) The delay beffers are stored in an external 32MB RAM module (ISSI IS42S16160J) allowing for over 5 minutes of stereo delay.

Errata
------
There were two errors on v2 hardware:

1. OpAmp U3 pin 2 and 3 incorrectly swapped. Bodge is to ground pin 3 of U3, remove capacitor C34, lift U3 pin 2 and connect it with a bodge wire to pad 2 of R24.
2. DRAM address line 12 was not connected. Bodge is to connect DRAM Pin 36 to MCU Pin 87.
3. Swapped C54 (tantalum 22uF) for ceramic 47uF (1206 25V X5R).
