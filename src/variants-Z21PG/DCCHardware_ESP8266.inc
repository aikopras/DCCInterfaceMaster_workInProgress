//******************************************************************************
//
// File:            DCCHardware_ESP8266_sw_timer1.inc
// Purpose:         DCC Waveform Generator for ESP8266  processors (bit-banging)
// Copyright 2026:  Aiko Pras
// History:         2026-01-29 / Aiko Pras - First version
//
//
// Historical credits:
// - Wolfgang Kufer (OpenDCC, http://opendcc.de)
//     * Original DCC waveform generation concepts and basic routines
//     * Provides timing calculations for '1' and '0' bits
//
// - Don Goodman-Wilson (Railstars CmdrArduino)
//     * Adapted OpenDCC waveform generation for Arduino
//     * Packet scheduling logic
//     * Initial integration with Arduino timers and interrupts
//
// - Philipp Gahtow (DCCInterfaceMaster, https://github.com/Digital-MoBa/DCCInterfaceMaster)
//     * Rewrote waveform generator for multiple MCU platforms (Arduino/ESP)
//     * Modified ISR and preamble handling
//     * Added support for service mode, RailCom cutouts and 3 DCC output pins
//
// Hardware layer overview:
// This file implements the ESP8266-specific hardware abstraction layer for
// software-based DCC waveform generation using the ESP8266 hardware Timer1.
//
// It connects the processor-independent DCC state machine
// (DCCHardwareCommon.inc) to the ESP8266 hardware by providing:
//   - Configuration and control of ESP8266 Timer1
//     with microsecond resolution for accurate DCC bit timing
//   - Binding of the generic DCC ISR to the Timer1 interrupt
//   - Inline macros for fast, deterministic GPIO pin toggling using
//     direct register access (GPIO0–15)
//   - Initialization, enabling, and disabling of the DCC rail outputs
//     and optional monitor output
//
// The DCC waveform is generated entirely in software (bit-banging).
// All output transitions are executed inside the timer ISR to guarantee
// precise timing independent of the main application code.
//
// Note: GPIO16 (D0) is excluded because it cannot be toggled via normal
//       GPIO registers, so only GPIO0–15 can be used for DCC outputs.
//       Depending on the other software, not all of these pins can be used.
//
// This implementation is specific for ESP8266 processors and uses
// the low-level ESP8266 SDK timer and GPIO functions.
//
//******************************************************************************
#include <Arduino.h>
#include <ets_sys.h>      // timer1, ISR hooks
#include <osapi.h>        // system macros (sometimes required)
#include <gpio.h>         // GPIO registers & macros
#include "../DCCHardware.h"


//******************************************************************************
// Part 1: Pins en output signals
//******************************************************************************
// Direct register access for pin changes
// Create a structure for each output pin. This structure provides
// direct access to the ESP8266 GPIO registers, using a bitmask.
typedef struct {
  uint32_t bit;           // BITmask: (1 << pin)
  uint8_t pin;            // GPIO pin number
  volatile bool enable;   // For RailCom / power-down logics
} DccOut_t;

// Instantiate three objects; one for each pin (output channel)
static DccOut_t dccMonitor;   // Create an instance for the dccMonitor
static DccOut_t dccRail;      // Create an instance for the dccRail
static DccOut_t dccRailInv;   // Create an instance for the dccRailInv

inline void initDccPin(DccOut_t &pinObj, uint8_t arduinoPin) {
  // Only allow valid GPIOs for DCC output (ESP8266 GPIO0–15)
  // 0xFF = no pin; GPIO16 (D0) is excluded because it can't be toggled via normal GPIO registers
  if (arduinoPin != 0xFF && arduinoPin <= 15) {
    pinMode(arduinoPin, OUTPUT);       // Arduino API
    pinObj.pin = arduinoPin;           // Store the Arduino pin number
    pinObj.bit = (1UL << arduinoPin);  // bitmask for the register
    pinObj.enable = false;
    GPOC = pinObj.bit;                 // force LOW at init
  }
  else {
    pinObj.pin = 0xFF;
    pinObj.bit = 0;
    pinObj.enable = false;
    arduinoPin = 0xFF;
  }
}

// #defines to set, clear and toggle the output signals
// Although only some will be used, for completeness all options are given.
#define DCCOUT_HGH     (GPOS = dccRail.bit)
#define DCCOUT_LOW     (GPOC = dccRail.bit)
#define DCCOUT_TGL     (GPO ^= dccRail.bit)
#define DCCINV_HGH     (GPOS = dccRailInv.bit)
#define DCCINV_LOW     (GPOC = dccRailInv.bit)
#define DCCINV_TGL     (GPO ^= dccRailInv.bit)
#define DCCMON_HGH     (GPOS = dccMonitor.bit)
#define DCCMON_LOW     (GPOC = dccMonitor.bit)
#define DCCMON_TGL     (GPO ^= dccMonitor.bit)

// defines to enable / disable the output signals
#define ENABLE_DCCMON       {dccMonitor.enable = true;}
#define DISABLE_DCCMON      {dccMonitor.enable = false;}
#define ENABLE_DCCOUT       {dccRail.enable = true;}
#define DISABLE_DCCOUT      {dccRail.enable = false;}
#define ENABLE_DCCINV       {dccRailInv.enable = true;}
#define DISABLE_DCCINV      {dccRailInv.enable = false;}

#define DCCOUT_IS_ENABLED   dccRail.enable
#define DCCINV_IS_ENABLED   dccRailInv.enable
#define DCCMON_IS_ENABLED   dccMonitor.enable

//******************************************************************************
// Part 2: Timer and ISR
//******************************************************************************
// Forward declaration for the timer ISR (to avoid compile errors)
void dccTimerIsr();

// The ISR triggered by ESP8266 Timer1 hardware
#define ISR_START void ICACHE_RAM_ATTR dccTimerIsr()
#define ISR_PROLOGUE {;}           // Nothing to do
#define ISR_EPILOGUE {;}           // Nothing to do

#define TICKS_PER_US               (F_CPU / 1000000UL)
#define half_one_count             ((uint32_t)TICKS_PER_US * (29))   // RCN 217
#define one_count                  ((uint32_t)TICKS_PER_US * (58))   // RCN 210
#define zero_count                 ((uint32_t)TICKS_PER_US * (100))  // RCN 210

// Load timer for specific DCC timings
#define TMR_ONE                    timer1_write(one_count)
#define TMR_ZERO                   timer1_write(zero_count)
#define TMR_RAILCOM_STARTBIT       timer1_write(half_one_count)

void enableDccTimer() {
  noInterrupts();
  timer1_disable();
  timer1_isr_init();                        // reset previous ISR
  timer1_attachInterrupt(dccTimerIsr);      // Attach the timer to the new ISR
  timer1_write(one_count);                  // timer initial value
  interrupts();
}

inline void startDccTimer() {
  timer1_enable(TIM_DIV1, TIM_EDGE, TIM_LOOP); // enable timer: divider=1, edge-triggered, loop
}

inline void disableDccTimer() {
  timer1_disable();        // Stop the timer
  timer1_isr_init();       // reset ISR (detach)
}


//******************************************************************************
// Part 3: Continue with the processor independent code
//******************************************************************************
#include "DCCHardwareCommon.inc"
