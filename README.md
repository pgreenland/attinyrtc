# ATTinyRTC

This repository contains my attempt at a real-time clock (RTC) replacement, specially for the Mac SE/30, untested (yet should work :-P) on others, using the Microchip ATTiny85.

It's specifically aimed at SE/30's that have suffered damage to their RTC circuitry due to battery leaks.

It therefore doesn't require a battery, storing the PRAM contents in the ATTiny's integrated EEPROM.

It also doesn't require a crystal, using the ATTiny's integrated RC oscillator as its clock source.

It should work (although hasn't been tested) in a mac with a working RTC battery. If desired the EEPROM backup of the settings and time can be disabled via commenting out the definition `ENABLE_EEPROM`.

An effort has been made to limit the EEPROM writes. With only updates being written, and update only being considered when the RTC is marked as read-only by the host. However it isn't clear how long the EEPROM will last in the ATTiny's using this approach. Time will tell.

It's based on the good work by other folks:

https://www.reddit.com/r/VintageApple/comments/91e5cf/couldnt_find_a_replacement_for_the_rtcpram_chip/
https://pastebin.com/baPZ4nN4
https://github.com/quorten/macsehw/blob/master/firmware/rtc/MacRTC.c

The ATTiny85 was identified previously as a candidate for a drop-in replacement for some of the classic Mac family of RTC chips.

Previous versions of the firmware focused on the Mac Plus, which has a serial clock of between 1 and 20Khz (based on comments in the code).

The Mac SE/30 has a serial clock of 250Khz, requiring a substantial rewrite to get things working.

See my blog for additional notes on how the software works / its development:

[New Timepiece for a Classic Mac - Part 1](http://www.quantulum.co.uk/blog/new-timepiece-for-a-classic-mac-part-1/)

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

The programmer I'm using is a usbasp programmer, search eBay for USBasp, they all seem pretty much the same. Remember to head over to [USBasp](https://www.fischl.de/usbasp/) to drop them a tip. Alternatively any programmer supported by avrdude can be selected via configuration in the CMake file.

As part of setting fuses the reset pin is reconfigured as GPIO.

Once this has been done the only way to re-program the chip is by using high voltage programming.

During development, as a workaround I've been re-assigning one of the unused crystal pins to fullfil the role of the PB5, with the help of a small adapter. Thereby allowing standard ICSP programming with no high voltage funny business required.

After the initial programming I found [this guide](https://www.electronics-lab.com/recover-bricked-attiny-using-arduino-as-high-voltage-programmer/) helpful. Using a spare Arduino and minimal components to rewrite the fuses using HV programming, such that the reset pin is again enabled and regular ICSP programming can be performed again.
