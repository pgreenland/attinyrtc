/* ------- Config ------- */

/* Enable EEPROM - if defined RTC will backup/restore its data to EEPROM, allowing it to be used in a mac without an RTC battery. Disable for authentic RTC behavior */
#define ENABLE_EEPROM

/* Define to have write only test register readable, returning 0xA5 and 0x5A on alternate reads */
//#define TEST_REG_READ_DUMMY_VALUE

/* Define to output one second line on PB4 rather than PB5 */
//#define ONE_SECOND_ON_PB4

/* Define to output timing pulses on PB3 */
//#define TIMING_ENABLED

/* ---------------------- */

/* Standard libraries */
#include <stdint.h>
#include <stdbool.h>

/* AVR libraries */
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>

/* Add section for simulator */
#ifdef BUILD_FOR_SIM
#include "avr/avr_mcu_section.h"
AVR_MCU(F_CPU, "attiny85");
#endif

/*
RTC Pinout -> ATTiny85 pinout
              __  __
 1SEC (PB5) -|1 \/ 8|- VCC
XTAL1 (PB3) -|2    7|- (PB2) RTC.CLK
XTAL2 (PB4) -|3    6|- (PB1) RTC.DATA
        GND -|4____5|- (PB0) !RTC
*/

/*
One second interrupt with 50% duty cycle:

The GPIO interrupt line needs to toggle every 500ms.
Timer 0 prescaler = 1024.

@ 16Mhz clock:
Timer 0 freq = 16e6 / 1024 = 15625Hz
We enable CTC mode, causing the timer to interrupt and reset on a value of our choice.
Timer 0 compare = 125
15625 / 125 = 125Hz (timer interrupt will fire every 8ms)
We wish to toggle every 500ms aka 2Hz.
125 / 2 = 62.5
We therefore need to toggle the output every 62.5 interrupts.
We can scale that number up to an integer
62.5 * 2 = 125
Each time we receive an interrupt we add 2 to a counter.
Whenever the counter is >= 125 we decrement it by 125 and toggle the output.
*/

/*
Regular command / address is
uVVVVVww
Where
	u = 0 for write and x = 1 for read
	VVVVV = Regular addr (table below)
	ww = 01 for regular command, don't care for extended

If Regular Command Addr = 0x0e or 0x0f, command is extended and next byte is an extended address
	xYYYYYzz
Where
	x = Don't care
	YYYYY = Extended address, least significant 5 bytes
	zz = Don't care

Full extended address may be formed by:
	(Vww << 5) | YYYYY
	That is the lowest order 3 bits from regular command / address and extended address bits

An 8-bit data byte follows the regular / extended command bytes, send by the host (if write) or device (if read)

Classic PRAM is 20 bytes (+ 4 bytes for clock).
	00 = Clock 0
	01 = Clock 1
	02 = Clock 2
	03 = Clock 3
	04 = Clock 0
	05 = Clock 1
	06 = Clock 2
	07 = Clock 3
	08 = RAM Lo
	09 = RAM Lo
	0A = RAM Lo
	0B = RAM Lo
	0C = Test reg
	0D = Write protect
	0E = Extended cmd
	0F = Extended cmd
	10 = RAM Hi
	11 = RAM Hi
	12 = RAM Hi
	13 = RAM Hi
	14 = RAM Hi
	15 = RAM Hi
	16 = RAM Hi
	17 = RAM Hi
	18 = RAM Hi
	19 = RAM Hi
	1A = RAM Hi
	1B = RAM Hi
	1C = RAM Hi
	1D = RAM Hi
	1E = RAM Hi
	1F = RAM Hi

Extended PRAM is 256 bytes (+ 4 bytes for clock):
	0x08 - 0x0B = RAM Lo mirror
	0x10 - 0x1F = RAM Hi mirror
*/

/* Define fuse config */
FUSES = {
	/* Use 16 MHz internal clock via PLL, disabling divide by 8. */
	.low = (((LFUSE_DEFAULT | ~FUSE_CKDIV8) & FUSE_CKSEL3 & FUSE_CKSEL1) | ~FUSE_CKSEL0),

	/* Disable the external RESET pin since it is used for the 1-second interrupt output. */
	.high = (HFUSE_DEFAULT & FUSE_RSTDISBL),

	/* Extended fuse takes default value */
	.extended = EFUSE_DEFAULT
};

/* Types */
typedef union
{
	/* Value for incrementing */
	uint32_t uiValue;

	/* 8-bit AVR is modelled as little endian by GCC at least, access bytes directly */
	uint8_t abyValue[4];

} uSeconds_t;

/* Define pins */
static const uint8_t uiPinEnable = PB0;
static const uint8_t uiPinData = PB1;
static const uint8_t uiPinClock = PB2;
#ifdef ONE_SECOND_ON_PB4
static const uint8_t uiPinOneSec = PB4;
#else
static const uint8_t uiPinOneSec = PB5;
#endif

/* Hijack one of the crystal pins for some logic analyzer snoopage */
#ifdef TIMING_ENABLED
static const uint8_t uiPinTiming = PB3;
#endif

/* Define pin change interrupts */
static const uint8_t uiChangeIntEnable = PCINT0;

/* Check supported clock frequency for timer calculations */
#if F_CPU != 16000000
#error "Unexpected clock frequency"
#endif

/* One second timer divider */
static const uint8_t uiTimerDivider = 125;

/* One second timer counter increment */
static const uint8_t uiCounterInc = 2;

/* One second timer match */
static const uint8_t uiCounterMatch = 125;

/* Macros */

/* Check if inputs are high */
#define mIsEnableHigh() (0 != (PINB & _BV(uiPinEnable)))
#define mIsClockHigh() (0 != (PINB & _BV(uiPinClock)))
#define mIsDataHigh() (0 != (PINB & _BV(uiPinData)))

/* Check if timeout timer has expired */
#define mSerialTimeout() (0 != (WDTCR & _BV(WDIF)))

/* Check if base address is special extended address value */
#define mIsAddrExtended(base) (0x38 == ((base) & 0x78))

/* Check if base address has write flag set */
#define mIsAddrWrite(base) (0 == ((base) & 0x80))

/* Extract address from regular / extended command */
#define mExtractBaseAddr(base) (((base) & 0x7C) >> 2)
#define mExtractExtendedAddr(base, ext) ((((base) & 0x07) << 5) | (((ext) & 0x7C) >> 2))

/* Check if regular address is within clock range */
#define mIsClockAddr(base) ((base) >= 0x00 && (base) <= 0x07)

/* Check if regular address is within low or high RAM range */
#define mIsRAMLoAddr(base) ((base) >= 0x08 && (base) <= 0x0B)
#define mIsRAMHiAddr(base) ((base) >= 0x10 && (base) <= 0x1F)

/* Check if regular address is for (write only) write protect register */
#define mIsWriteProtectAddr(base) (0x0D == (base))

/* Check if write protect data bit is set */
#define mDataByteToWriteProtect(data) (0 != ((data) & 0x80))

/* Check if regular address is for (write only) test register */
#define mIsTestAddr(base) (0x0C == (base))

/* Clock bytes are duplicated, work out which byte is being read or written */
#define mClockByte(base) ((base) & 0x03)

/* Change data pin outputs (single instruction) */
#define mSetDataPinHigh() asm volatile("cbi %[ddrb], %[pin]" : /* no out */ : [ddrb] "I" (_SFR_IO_ADDR(DDRB)), [pin] "I" (uiPinData));
#define mSetDataPinLow() asm volatile("sbi %[ddrb], %[pin]" : /* no out */ : [ddrb] "I" (_SFR_IO_ADDR(DDRB)), [pin] "I" (uiPinData));

/* Change timing pin outputs */
#ifdef TIMING_ENABLED
	#define mTimingPinLow() asm volatile("sbi %[ddrb], %[pin]" : /* no out */ : [ddrb] "I" (_SFR_IO_ADDR(DDRB)), [pin] "I" (uiPinTiming));
	#define mTimingPinHigh() asm volatile("cbi %[ddrb], %[pin]" : /* no out */ : [ddrb] "I" (_SFR_IO_ADDR(DDRB)), [pin] "I" (uiPinTiming));
#else
	#define mTimingPinLow()
	#define mTimingPinHigh()
#endif

/* Define PRAM size */
#define PRAM_SIZE (256) /* bytes */

/* Function prototypes */
static void powerSaveInit(void);
static void gpioInit(void);
static void timerInit(void);
static uint8_t processRegularCmdRead(uint8_t uiAddrBase);
static void processRegularCmdWrite(uint8_t uiAddrBase, uint8_t uiData);
static uint8_t processExtendedCmdRead(uint8_t uiAddrBase, uint8_t uiAddrExt);
static void processExtendedCmdWrite(uint8_t uiAddrBase, uint8_t uiAddrExt, uint8_t uiData);

#ifdef ENABLE_EEPROM

/* PRAM EEPROM buffer */
__attribute__((section(".eeprom")))
static uint8_t abyPRAM[PRAM_SIZE] = {0x00};

/* Clock EEPROM buffer */
__attribute__((section(".eeprom")))
static uint32_t uiStoredSeconds = 0;

#endif

/* PRAM shadow buffer (working copy of EEPROM data in RAM) */
static uint8_t abyPRAMShadow[PRAM_SIZE];

#ifdef ENABLE_EEPROM

/*
** Dirty flag - Set when PRAM shadow write protection applied. Ideally this would
** only be set if a value was written, however this way we get a free flush of the
** clock when shutting down. The EEPROM is only written if it changes so we should
** be ok....maybe....hopefully....
*/
static volatile bool bShadowDirty;

#endif

/* Write protection status */
static bool bWriteProtected = true;

/* Seconds counter */
static volatile uSeconds_t uSeconds;

int main(void)
{
	/* Disable interrupts */
	cli();

	/* Set oscillator calibration */
	//OSCCAL = 0x84;

	/* Disable unused peripherals to save power */
	powerSaveInit();

	/* Init peripherals */
	timerInit();
	gpioInit();

#ifdef ENABLE_EEPROM
	/* Load PRAM shadow from EEPROM */
	eeprom_read_block(abyPRAMShadow, abyPRAM, sizeof(abyPRAMShadow));

	/* Load last time from EEPROM */
	uSeconds.uiValue = eeprom_read_dword(&uiStoredSeconds);
#endif

	/* Enable interrupts */
	sei();

	/* Sleep whenever possible, updating EEPROM if dirty */
	for (;;)
	{
#ifdef ENABLE_EEPROM
		if (bShadowDirty)
		{
			/* Clear dirty flag first in case its set again */
			bShadowDirty = false;

			/* Update PRAM in EEPROM from shadow in RAM */
			eeprom_update_block(abyPRAMShadow, abyPRAM, sizeof(abyPRAMShadow));

			/* Sample clock via repeated read, to avoid having to disable interrupts */
			uint32_t uiTmpSeconds;
			do
			{
				uiTmpSeconds = uSeconds.uiValue;
			} while (uiTmpSeconds != uSeconds.uiValue);

			/* Update clock in EEPROM from RAM */
			eeprom_update_dword(&uiStoredSeconds, uiTmpSeconds);
		}
#endif

		/* Go back to sleep */
		sleep_mode();
	}
}

/* Flag ISR with ISR_NOBLOCK, such that interrupts will be re-enabled allowing higher priority serial interrupt to nest with less critical timer */
ISR(TIM0_COMPA_vect, ISR_NOBLOCK)
{
	static uint8_t uiCounter;

	/* Count match */
	uiCounter += uiCounterInc;

	/* Check counter */
	if (uiCounter >= uiCounterMatch)
	{
		/* Toggle one second line */
		DDRB ^= _BV(uiPinOneSec);

		/* Increment seconds counter every other toggle */
		if (DDRB & _BV(uiPinOneSec))
		{
			uSeconds.uiValue++;
		}

		/* Decrement counter */
		uiCounter -= uiCounterMatch;
	}
}

ISR(PCINT0_vect)
{
	/* Bit counter - count down with while loops allowing gcc to perform one less comparison */
	uint8_t uiBit;

	/* Serial address (base and extended) */
	uint8_t uiAddrBase = 0;
	uint8_t uiAddrExt = 0;

	/* Serial data */
	uint8_t uiData = 0;

	/* Rising edge on enable (chip deselected), stop early */
	if (mIsEnableHigh())
	{
		/* Ensure timing pin reset */
		mTimingPinHigh();

		/* Disable watchdog interrupt (stopping timer) and clear pending flag */
		wdt_reset();
		WDTCR = _BV(WDIF);

		/* Bail */
		return;
	}

	/* Reset watchdog and enable interrupt (to start timer) */
	wdt_reset();
	WDTCR = _BV(WDIE) | _BV(WDIF);

	/* Falling edge on enable (chip selected), receive address bits */
	uiBit = 8;
	while (0 != uiBit)
	{
		/* Sample bit on rising edge, while monitoring enable (except after last bit) */
		while (!mIsClockHigh()) if (mIsEnableHigh() || mSerialTimeout()) return;
		mTimingPinLow();
		uiAddrBase = (uiAddrBase << 1) | mIsDataHigh();
		mTimingPinHigh();
		uiBit--;
		if (0 != uiBit) while (mIsClockHigh()) if (mIsEnableHigh()) return;
	}

	/* Check if extended command */
	if (mIsAddrExtended(uiAddrBase))
	{
		/* Receive extended address */
		uiBit = 8;
		while (mIsClockHigh()) if (mIsEnableHigh()) return;
		while (0 != uiBit)
		{
			/* Sample bit on rising edge, while monitoring enable */
			while (!mIsClockHigh()) if (mIsEnableHigh() || mSerialTimeout()) return;
			mTimingPinLow();
			uiAddrExt = (uiAddrExt << 1) | mIsDataHigh();
			mTimingPinHigh();
			uiBit--;
			if (0 != uiBit) while (mIsClockHigh()) if (mIsEnableHigh()) return;
		}
	}

	/* Check if write */
	if (mIsAddrWrite(uiAddrBase))
	{
		/* Write request, receive data */
		uiBit = 8;
		while (mIsClockHigh()) if (mIsEnableHigh()) return;
		while (0 != uiBit)
		{
			/* Sample bit on rising edge, while monitoring enable */
			while (!mIsClockHigh()) if (mIsEnableHigh() || mSerialTimeout()) return;
			mTimingPinLow();
			uiData = (uiData << 1) | mIsDataHigh();
			mTimingPinHigh();
			uiBit--;
			if (0 != uiBit) while (mIsClockHigh()) if (mIsEnableHigh()) return;
		}

		/* Execute command */
		mTimingPinLow();
		if (mIsAddrExtended(uiAddrBase))
		{
			/* Execute extended command */
			processExtendedCmdWrite(uiAddrBase, uiAddrExt, uiData);
		}
		else
		{
			/* Execute regular command */
			processRegularCmdWrite(uiAddrBase, uiData);
		}
		mTimingPinHigh();
	}
	else
	{
		/* Read request, execute command */
		mTimingPinLow();
		if (mIsAddrExtended(uiAddrBase))
		{
			/* Execute extended command */
			uiData = processExtendedCmdRead(uiAddrBase, uiAddrExt);
		}
		else
		{
			/* Execute regular command */
			uiData = processRegularCmdRead(uiAddrBase);
		}
		mTimingPinHigh();

		/* Send data, cache current DDR value */
		uint8_t uiDDRVal = DDRB;

		/*
		** Invert data bits to assist assembly routine
		** When data bit is 1 we want to set ddr reg bit to 0 and vice versa
		*/
		uiData = ~uiData;

		/* Shift out bits */
		uiBit = 8;
		while (0 != uiBit)
		{
			/* Change data when clock low, before waiting for rising edge, while monitoring enable */
			while (mIsClockHigh()) if (mIsEnableHigh()) break;
			mTimingPinLow();
			asm volatile(
				/* Shift data bit out of data register */
				"bst %[out], 7\n\t"
				"bld %[ddrb_tmp], %[data_pin]\n\t"
				"out %[ddrb], %[ddrb_tmp]\n\t"
				:
					/* Outputs */
					[ddrb_tmp] "+r" (uiDDRVal)
				:
					/* Inputs */
					[out] "r" (uiData),
					[ddrb] "I" (_SFR_IO_ADDR(DDRB)),
					[data_pin] "I" (uiPinData)
				:
					/* Clobbers - none */
			);
			mTimingPinHigh();
			uiData <<= 1;
			uiBit--;
			if (0 != uiBit) while (!mIsClockHigh()) if (mIsEnableHigh() || mSerialTimeout()) break;
		}

		/* Give host a chance to sample last bit, wait for enable to go high */
		while (!mIsEnableHigh() && !mSerialTimeout());

		/* Switch pin to input mode, allowing data line to float after completion / abort */
		mSetDataPinHigh();
	}
}

ISR(WDT_vect)
{
	/* Ignore watchdog interrupt, managed by ISR above */
}

static void powerSaveInit(void)
{
	/* Disable Analog Comparator */
	ACSR |= _BV(ACD);

	/* Disable Timer 1, only using Timer 0 */
	PRR |= _BV(PRTIM1);

	/* Disable Universal Serial Interface */
	PRR |= _BV(PRUSI);

	/* Disable Analog/Digital Converter */
	PRR |= _BV(PRADC);
}

static void gpioInit(void)
{
	/* Enable pin change interrupt for enable */
	GIMSK |= _BV(PCIE);
	PCMSK |= _BV(uiChangeIntEnable);
}

static void timerInit(void)
{
	/* Enable CTC mode (clear timer on compare match) */
	TCCR0A = (1 << WGM01) | (0 << WGM00);
	TCCR0B = (0 << WGM02);

	/* Set prescaler to 1024 */
	TCCR0B |= (1 << CS02) | (0 << CS01) | (1 << CS00);

	/* Divide by uiTimerDivider (matching on limit - 1) */
	OCR0A = uiTimerDivider - 1;

	/* Enable timer 0 output compare match A interrupt */
	TIMSK |= _BV(OCIE0A);
}

static inline uint8_t processRegularCmdRead(uint8_t uiAddrBase)
{
	/* Prepare full address */
	uint8_t uiFullAddr = mExtractBaseAddr(uiAddrBase);

	if (mIsClockAddr(uiFullAddr))
	{
		/* Clock address, decide on byte to get or set (given clock is repeated) */
		uint8_t uiByteIndex = mClockByte(uiFullAddr);

		/* Retrieve byte */
		return uSeconds.abyValue[uiByteIndex];
	}
	else if (	mIsRAMLoAddr(uiFullAddr)
			 || mIsRAMHiAddr(uiFullAddr)
			)
	{
		/* RAM address is within classic low or high range, which maps directly into extended range, retrieve byte */
		return abyPRAMShadow[uiFullAddr];
	}
	#ifdef TEST_REG_READ_DUMMY_VALUE
	else if (mIsTestAddr(uiFullAddr))
	{
		static uint8_t uiTestRegVal = 0x5A;

		/* Invert bits */
		uiTestRegVal = ~uiTestRegVal;

		/* Return value */
		return uiTestRegVal;
	}
	#endif

	/* If we got this far, address was rejected, return bus idle value */
	return 0xFF;
}

static inline void processRegularCmdWrite(uint8_t uiAddrBase, uint8_t uiData)
{
	/* Prepare full address */
	uint8_t uiFullAddr = mExtractBaseAddr(uiAddrBase);

	/* Check if write protected when writing */
	if (	bWriteProtected
		 && !mIsWriteProtectAddr(uiFullAddr)
	   )
	{
		/*
		** Write request while write protected, to a register other than the
		** write protect register, ignore
		*/
		return;
	}

	if (mIsClockAddr(uiFullAddr))
	{
		/* Clock address, decide on byte to get or set (given clock is repeated) */
		uint8_t uiByteIndex = mClockByte(uiFullAddr);

		/* Retrieve byte */
		uSeconds.abyValue[uiByteIndex] = uiData;
	}
	else if (mIsWriteProtectAddr(uiFullAddr))
	{
		/* Extract new write protect status */
		bool bNewWriteProtected = mDataByteToWriteProtect(uiData);

#ifdef ENABLE_EEPROM
		/* Set dirty flag when transitioning from read-write to read-only */
		if (!bWriteProtected && bNewWriteProtected) bShadowDirty = true;
#endif

		/* Update write protect flag */
		bWriteProtected = bNewWriteProtected;
	}
	else if (	mIsRAMLoAddr(uiFullAddr)
			 || mIsRAMHiAddr(uiFullAddr)
			)
	{
		/* RAM address is within classic low or high range, which maps directly into extended range, store byte */
		abyPRAMShadow[uiFullAddr] = uiData;
	}
}

static inline uint8_t processExtendedCmdRead(uint8_t uiAddrBase, uint8_t uiAddrExt)
{
	/* Prepare full address */
	uint8_t uiFullAddr = mExtractExtendedAddr(uiAddrBase, uiAddrExt);

	/* Retrieve data */
	return abyPRAMShadow[uiFullAddr];
}

static inline void processExtendedCmdWrite(uint8_t uiAddrBase, uint8_t uiAddrExt, uint8_t uiData)
{
	/* Prepare full address */
	uint8_t uiFullAddr = mExtractExtendedAddr(uiAddrBase, uiAddrExt);

	/* Prevent write if write protection enabled */
	if (bWriteProtected) return;

	/* Store data */
	abyPRAMShadow[uiFullAddr] = uiData;
}
