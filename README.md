# ATTinyRTC

This repository contains my attempt at a real-time clock (RTC) replacement, specially for the Mac SE/30, untested (yet should work :-P) on others, using the Microchip ATTiny85.

It's specifically aimed at SE/30's that have suffered damage to their RTC circuitry due to battery leaks.

It therefore doesn't require a battery, storing the PRAM contents in the ATTiny's integrated EEPROM.

It also doesn't require a crystal, using the ATTiny's integrated RC oscillator as its clock source.

One unfortunate limitation that I'm aware of, is that it can't be used in macs which have a RTC battery present.
This is due to the system shutdown, appearing to the RTC as the start of a transfer, for which it has no timeout mechanism.
I've previously attempted to add one but timings are already tight and the additional few instructions knocked things out of whack.
If used in a system with an RTC battery, the clock will not increment while the system is powered down and the battery will likely be depleted faster than expected as the microcontroller will be wide awake waiting for the next bit of serial data.

It's based on the good work by other folks:

https://www.reddit.com/r/VintageApple/comments/91e5cf/couldnt_find_a_replacement_for_the_rtcpram_chip/
https://pastebin.com/baPZ4nN4
https://github.com/quorten/macsehw/blob/master/firmware/rtc/MacRTC.c

The ATTiny85 was identified previously as a candidate for a drop-in replacement for some of the classic Mac family of RTC chips.

Previous versions of the firmware focused on the Mac Plus, which has a serial clock of between 1 and 20Khz (based on comments in the code).

The Mac SE/30 has a serial clock of 250Khz, requiring a substantial rewrite to get things working.

See my blog for additional notes on how the software works / its development:

[New Timepiece for a Classic Mac - Part 1](http://www.quantulum.co.uk/new-timepiece-for-a-classic-mac-part-1/)

## Building

1. Ensure avg-gcc, cmake, make and avrdude are installed.

2. Build with cmake
   ```
   mkdir build
   cd build
   cmake ..
   make
   ```

3. Flash
   ```
   make upload
   make upload_eeprom
   ```

4. Set fuses
   ```
   make fuses
   ```

### Note

As part of setting fuses the reset pin is reconfigured as GPIO.

Once this has been done the only way to re-program the chip is by using high voltage programming.

During development, as a workaround I've been re-assigning one of the unused crystal pins to fullfil the role of the PB5, with the help of a small adapter. Thereby allowing standard ICSP programming with no high voltage funny business required.
