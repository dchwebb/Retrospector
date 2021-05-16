# Boottest_H743
Test running bootloader from different Flash address. Project tests basic blinky functionality in a loop and from a timer interrupt to establish that running from different flash region works.

Flash start location is set in linker script to Bank 2 sector 1:

```
FLASH (rx)     : ORIGIN = 0x08100000, LENGTH = 1024K
```

assembly startup script then takes origin of vector table and updates SCB->VTOR register:

```
  ldr r3, =0xe000ed00    // SCB location
  ldr r2, =g_pfnVectors  // Location of vector table in Flash (set by linker)
  str r2, [r3, #8]       // store vector table location to offset 8 of SCB which is VTOR
```
