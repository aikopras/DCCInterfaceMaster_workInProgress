//******************************************************************************
//
// File:            DCCHardware_ESP32S.inc
// Purpose:         DCC Waveform Generator for ESP32-S (Xtensa) processors (bit-banging)
// Copyright 2026:  Aiko Pras
// History:         2026-01-31 / Aiko Pras - First version
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
// This file implements the ESP32-S series (Xtensa: ESP32 / S2 / S3) specific
// hardware abstraction layer for software-based DCC waveform generation
// using hw_timer_t.
//
// It connects the processor-independent DCC state machine
// (DCCHardwareCommon.inc) to the ESP32 hardware by providing:
//
//   - Configuration and control of an ESP32 hardware timer (hw_timer_t)
//     with microsecond resolution for accurate DCC bit timing
//   - Binding of the generic DCC ISR to an ESP32 timer interrupt callback
//   - Inline macros for fast, deterministic GPIO pin toggling using
//     direct register access
//   - Initialization, enabling, and disabling of the DCC rail outputs
//     and optional monitor output
//
// The DCC waveform is generated entirely in software (bit-banging).
// All output transitions are executed inside the timer ISR to guarantee
// precise timing independent of the main application code.
//
// Note: This implementation requires Arduino-ESP32 core version 3.0.0
// or later, due to changes in the hw_timer_t API and interrupt handling.
//
//******************************************************************************
// Each variant file should provide the following macros (#defines) to
// implement hardware-specific code for this variant.
// These macros simplify ISR and setup code, providing clear and consistent
// naming for timer counts and pin control across each driver.
//
// ISR specific:
// - ISR_START     : Bind the Timer to an interrupt vector / call-back function
// - ISR_PROLOGUE  : Code needed at the ISR start (like clearing the ISR flag)
// - ISR_EPILOGUE  : Code needed at the ISR end
//
// Timer specific:
// - TMR_ZERO                           : Load timer for a zero bit
// - TMR_ONE                            : Load timer for a one bit
// - TMR_RAILCOM_STARTBIT               : Load timer for RailCom start-bit
//
// Output specific:
// - DCCOUT_TGL                         : Toggle the normal rail output
// - DCCINV_TGL                         : Toggle the inverted rail output
// - DCCMON_TGL                         : Toggle the monitor output
// - ENABLE_DCCOUT / DISABLE_DCCOUT     : Enable or disable normal rail output
// - ENABLE_DCCINV / DISABLE_DCCINV     : Enable or disable inverted rail output
// - ENABLE_DCCMON / DISABLE_DCCMON     : Enable or disable monitor output
//
// Power-off specific:
// - DCCOUT_LOW                         : Power-off the normal DCC rail signal
// - DCCINV_LOW                         : Power-off the inverted DCC rail signal
//
//******************************************************************************
#include <Arduino.h>
#include "esp32-hal-timer.h"
#include "soc/gpio_struct.h"    // <-- Voor GPIO.out_w1ts etc.
#include "../DCCHardware.h"


//******************************************************************************
// Part 1: Pins en output signals
//******************************************************************************
// Direct register access for pin changes
// Create a structure for each output pin. This structure provides
// direct access to the ESP32 GPIO registers, using a bitmask.
typedef struct {
  volatile uint32_t *outset;    // GPIO out set register
  volatile uint32_t *outclr;    // GPIO out clear register
  volatile uint32_t *outtgl;    // GPIO out toggle register
  uint32_t bit;                 // Bitmask for the pin
  uint8_t enable;               // No pin toggle during RailCom gap / Power-down
} DccOut_t;

// Instantiate three objects; one for each pin (output channel)
static DccOut_t dccMonitor;   // Create an instance for the dccMonitor
static DccOut_t dccRail;      // Create an instance for the dccRail
static DccOut_t dccRailInv;   // Create an instance for the dccRailInv

// Create a dummy pointer address, to avoid problems with null pointers
static volatile uint32_t gpio_dummy;

inline void initDccPin(DccOut_t &pinObj, uint8_t arduinoPin) {
  if (arduinoPin != 0xFF) {
    pinMode(arduinoPin, OUTPUT);
    uint8_t gpio = arduinoPin;            // Simply use the Arduino pin
    pinObj.enable = false;
    if (gpio < 32) {
      pinObj.bit    = (1UL << gpio);
      pinObj.outset = &GPIO.out_w1ts;
      pinObj.outclr = &GPIO.out_w1tc;
      pinObj.outtgl = &GPIO.out;           // toggle via XOR
      GPIO.out_w1tc = pinObj.bit;          // force LOW
    } else {
      pinObj.bit    = (1UL << (gpio - 32));
      pinObj.outset = &GPIO.out1_w1ts.val;
      pinObj.outclr = &GPIO.out1_w1tc.val;
      pinObj.outtgl = &GPIO.out1.val;      // toggle via XOR
      GPIO.out1_w1tc.val = pinObj.bit;     // force LOW
    }
  }
  else {
    pinObj.outset = &gpio_dummy;
    pinObj.outclr = &gpio_dummy;
    pinObj.outtgl = &gpio_dummy;
    pinObj.bit    = 0;
    pinObj.enable = false;
  }
}

// #defines to set, clear and toggle the output signals
// Although only some will be used, for completeness all options are given.
#define DCCOUT_HGH          {*dccRail.outset = dccRail.bit;}
#define DCCOUT_LOW          {*dccRail.outclr = dccRail.bit;}
#define DCCOUT_TGL          {*dccRail.outtgl ^= dccRail.bit;}
#define DCCINV_HGH          {*dccRailInv.outset = dccRailInv.bit;}
#define DCCINV_LOW          {*dccRailInv.outclr = dccRailInv.bit;}
#define DCCINV_TGL          {*dccRailInv.outtgl ^= dccRailInv.bit;}
#define DCCMON_HGH          {*dccMonitor.outset = dccMonitor.bit;}
#define DCCMON_LOW          {*dccMonitor.outclr = dccMonitor.bit;}
#define DCCMON_TGL          {*dccMonitor.outtgl ^= dccMonitor.bit;}


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
// Timer #defines using hw_timer_t
static hw_timer_t *dccTimer = nullptr;

// Forward declaration for the timer ISR (to avoid compile errors)
void dccTimerIsr();

// The ISR is a call-back
#define ISR_START void IRAM_ATTR dccTimerIsr()
#define ISR_PROLOGUE {;}             // Nothing to do
#define ISR_EPILOGUE {;}             // Nothing to do

#define half_one_count  29           // RCN 217
#define one_count       58           // RCN 210
#define zero_count      100          // RCN 210

// Set the timer alarms for specific DCC timings (µs resolution)
#define TMR_ONE                timerAlarm(dccTimer, one_count, true, 0)
#define TMR_ZERO               timerAlarm(dccTimer, zero_count, true, 0)
#define TMR_RAILCOM_STARTBIT   timerAlarm(dccTimer, half_one_count, true, 0)

void enableDccTimer() {
  dccTimer = timerBegin(1000000);              // 1 MHz = 1 µs ticks
  //dccTimer = timerBegin(0, 80, true);  // 0=timer0, 80 prescaler → 1 µs ticks
  timerAttachInterrupt(dccTimer, &dccTimerIsr);
  timerAlarm(dccTimer, one_count, true, 0);   // start with "1"-bit
}

inline void startDccTimer() {
  timerStart(dccTimer);
}

void disableDccTimer() {
  if (dccTimer) {
    timerStop(dccTimer);
    timerDetachInterrupt(dccTimer);
    timerEnd(dccTimer);
    dccTimer = nullptr;
  }
}


//******************************************************************************
// TODO: FOR TESTING
#define PIN_TEST1   12
#define PIN_TEST2   13
#define PIN_TEST3   14

#define DEBUG1 digitalWrite(PIN_TEST1, HIGH); digitalWrite(PIN_TEST1, LOW);
#define DEBUG2 digitalWrite(PIN_TEST2, HIGH); digitalWrite(PIN_TEST2, LOW);
#define DEBUG3 digitalWrite(PIN_TEST2, HIGH); digitalWrite(PIN_TEST2, LOW);


//******************************************************************************
// Part 3: Continue with the processor independent code
//******************************************************************************
#include "DCCHardwareCommon.inc"
