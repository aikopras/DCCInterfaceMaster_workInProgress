# Adding a New Hardware Driver

This document explains how to add support for a new processor to this Arduino Library by creating a new hardware driver file.

The DCCHardware code is split into:
- a processor-independent core (DCCHardwareCommon.inc)
- and processor-specific hardware drivers (DCCHardware_*.inc)

Your task, when adding a new processor-specific hardware driver, is to connect timers, interrupts, and GPIO pins of your processor to the existing DCC state machine. You should not modify the processor-independent core (DCCHardwareCommon.inc) itself.


## File Structure and Naming


Each processor or hardware variant is implemented in a single include file:

    DCCHardware_MyProcessor.inc

This file:
- Contains all hardware-specific code
- Defines a fixed set of macros used by the common core
- Ends by including DCCHardwareCommon.inc

A new driver is typically created by copying and adapting an existing one. Good starting points are:
- ATMega2560  -> DCCHardware_atmega_sw_timer1.inc
- AVR DxCore  -> DCCHardware_dxcore_sw_tcb1.inc
- ESP32-C    -> DCCHardware_ESP32C.inc


## Selecting the Driver in DCCHardware.cpp

After creating the new driver file, it must be conditionally included in
DCCHardware.cpp.

Example:

    #if defined(ARDUINO_ARCH_MYPROCESSOR)
      #include "variants/DCCHardware_MyProcessor.inc"
    #endif

Use a preprocessor symbol that uniquely identifies your platform.

## Required Driver Structure


Each driver file follows the same structure:

    Part 1: Pins and output signals
    Part 2: Timer and ISR
    Part 3: Include the common DCC core

Only Part 1 and Part 2 are hardware-specific, and must be adapted.

-----------------------------------------------------------------------
### Part 1: Pins and output signals


This section defines how the DCC outputs are mapped to physical pins and how
they are toggled.

#### 1.1 Output structure


Each output (normal rail, inverted rail, monitor) is represented by a small
structure containing:
- One or more register pointers or GPIO pin numbers
- A bitmask for the pin
- An enable flag used for RailCom gaps and power-down

Example (ATMega):

    typedef struct {
      volatile uint8_t *port;
      uint8_t bit;
      uint8_t enable;
    } DccOut_t;


#### 1.2 Pin initialization

Each driver must implement:

    inline void initDccPin(DccOut_t &pinObj, uint8_t arduinoPin);

Responsibilities:
- Configure the pin as OUTPUT
- Map the Arduino pin to the correct hardware
- Force a safe initial LOW state
- Disable toggling initially

Unused pins are represented by setting the pin number to 0xFF. In these cases, all register pointers are set to nullptr, the bitmask is 0, and the enable flag is false, ensuring the driver will not attempt to toggle the pin.

#### 1..3 Output control macros

Each driver must define the following macros.

Toggle / power off macros:

    #define DCCOUT_TGL
    #define DCCINV_TGL
    #define DCCMON_TGL
    #define DCCOUT_LOW
    #define DCCINV_LOW

The _TGL macros toggle the physical output pins.
They must be fast and deterministic and, if possible, use direct register access.
The _LOW macros are needed to force the rail outputs LOW and are used for emergency stop and short-circuit handling.

Enable / disable macros:

    #define ENABLE_DCCOUT
    #define DISABLE_DCCOUT
    #define ENABLE_DCCINV
    #define DISABLE_DCCINV
    #define ENABLE_DCCMON
    #define DISABLE_DCCMON

These macros do not change pin levels.
They only control whether toggling is allowed.

Enable state queries:

    #define DCCOUT_IS_ENABLED
    #define DCCINV_IS_ENABLED
    #define DCCMON_IS_ENABLED

-----------------------------------------------------------------------
### Part 2: Timer and ISR

This section connects the DCC state machine to the processor’s timer and
interrupt system. Depending on the processor, it is often possible to select a different hardware timer if the default one is already used elsewhere. The chosen timer must support periodic interrupts with microsecond resolution to maintain accurate DCC timing.

#### 2.1 ISR binding macros

Each driver must define:

    #define ISR_START
    #define ISR_PROLOGUE
    #define ISR_EPILOGUE

Example (ATMega):

ATMega2560:

    #define ISR_START    ISR(TIMER1_COMPA_vect)
    #define ISR_PROLOGUE {;}
    #define ISR_EPILOGUE {;}


#### 2.2 Timer timing macros

The common core requests specific DCC timings via these macros:

    #define TMR_ZERO
    #define TMR_ONE
    #define TMR_RAILCOM_STARTBIT

Required timing (per DCC RCN standards):
- ZERO bit           100 µs
- ONE bit            58 µs
- RailCom start bit  29 µs

Each macro must program the timer so that the interrupt occurs after the
required DCC bit time. If the timer can run at 1 µs resolution, the values 29, 58, and 100 can be used directly. If this is not possible due to prescaler or clock limitations, the timer values must be adjusted proportionally to achieve the same effective DCC bit timings.

#### 2.3 Timer control functions

Each driver must implement:

    void enableDccTimer();
    void startDccTimer();
    void disableDccTimer();

Together, these functions configure the hardware timer and its interrupt source, start and stop DCC waveform generation, and clear any pending interrupt flags where required by the processor architecture.

### 3: Include the common DCC core

At the end of the driver file, include the processor-independent logic:

    #include "DCCHardwareCommon.inc"

This instantiates the ISR, connects all macros to the DCC state machine,
and completes the DCC generator.
