//******************************************************************************
//
// File:            DCCHardware_dxcore_sw_tcb1.inc
// Purpose:         DCC Waveform Generator for DxCore processors (bit-banging)
// Copyright 2026:  Aiko Pras
// History:         2026-01-17 / Aiko Pras - First version
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
// This file implements a DxCore-specific hardware layer connecting the
// processor-independent DCC state machine (DCCHardwareCommon.inc) to the
// AVR Dx hardware. It provides:
//   - Configuration of a TCB timer for periodic half-bit interrupts
//   - Binding of the generic ISR to the correct interrupt vector
//   - Inline functions and macros for timing and pin control
//   - Initialization and enabling of the rail and monitor outputs
//
// Pin control and toggling:
// The driver uses software-controlled toggles (bit-banging) via
// direct register access.
// The normal rail, inverted rail, and monitor outputs are each represented
// by a small structure containing the port registers and bitmask. Output
// transitions are triggered explicitly in the ISR for deterministic timing.
//
// NOTE: Arduino Mega 2560 shows periodic multi-microsecond DCC jitter.
// This is caused by the millis() Timer0 ISR, which runs extra correction
// code about every 41 ms. Other drivers, such as the DxCore driver, don't
// have such problems, and therefore generate cleaner DCC signals.
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
#include "../DCCHardware.h"

//******************************************************************************
// Part 1: Pins en output signals
//******************************************************************************
// Direct register access for pin changes
// Create a structure for each output signal. This structure provides
// direct access to the DxCore output registers, using a bitmask.
typedef struct {
  volatile uint8_t *outset;   // Pointer naar PORTx.OUTSET register
  volatile uint8_t *outclr;   // Pointer naar PORTx.OUTCLR register
  volatile uint8_t *outtgl;   // Pointer naar PORTx.OUTTGL register
  uint8_t bit;                // Bitmask voor de pin
  uint8_t enable;             // No pin toggle during RailCom gap / Power-down
} DccOut_t;

// Instantiate three objects; one for each output signal
static DccOut_t dccMonitor;   // Create an instance for the dccMonitor
static DccOut_t dccRail;      // Create an instance for the dccRail
static DccOut_t dccRailInv;   // Create an instance for the dccRailInv

// Initialize a DCC output signal (pin) for direct register access (DxCore)
// Parameters:
//   - pinObj: DccOut_t object holding port registers and bit mask
//   - arduinoPin: Arduino pin number; set to 0xFF if unused
// For valid pins, this function:
//   - Sets the pin as OUTPUT
//   - Maps the Arduino pin to its PORTx registers
//   - Stores pointers to OUTSET, OUTCLR, OUTTGL registers
//   - Stores the pin bit mask in the structure
//   - Disables toggling initially (enable = false)
// If the pin is 0xFF (not used), all pointers are set to nullptr and enable
// is false
inline void initDccPin(DccOut_t &pinObj, uint8_t arduinoPin) {
  if (arduinoPin != 0xFF) {
    pinMode(arduinoPin, OUTPUT);                  // Port is output
    uint8_t port = digitalPinToPort(arduinoPin);  // Mapping from pin to port
    volatile PORT_t *p = portToPortStruct(port);  // Local pointer to the port
    pinObj.outset = &p->OUTSET;                   // Register to set output
    pinObj.outclr = &p->OUTCLR;                   // Register to clear output
    pinObj.outtgl = &p->OUTTGL;                   // Register to toggle output
    pinObj.bit = digitalPinToBitMask(arduinoPin); // pin bitmask
    pinObj.enable = false;                        // Toggle pin not allowed
    *pinObj.outclr = pinObj.bit;                  // Set output pin low
  }
  else {
    pinObj.outset = nullptr;
    pinObj.outclr = nullptr;
    pinObj.outtgl = nullptr;
    pinObj.bit = 0;
    pinObj.enable = false;
  }
}

// #defines to set, clear and toggle the output signals
// Although only some will be used, for completeness all options are given.
#define DCCOUT_HGH          {*dccRail.outset = dccRail.bit;}
#define DCCOUT_LOW          {*dccRail.outclr = dccRail.bit;}
#define DCCOUT_TGL          {*dccRail.outtgl = dccRail.bit;}
#define DCCINV_HGH          {*dccRailInv.outset = dccRailInv.bit;}
#define DCCINV_LOW          {*dccRailInv.outclr = dccRailInv.bit;}
#define DCCINV_TGL          {*dccRailInv.outtgl = dccRailInv.bit;}
#define DCCMON_HGH          {*dccMonitor.outset = dccMonitor.bit;}
#define DCCMON_LOW          {*dccMonitor.outclr = dccMonitor.bit;}
#define DCCMON_TGL          {*dccMonitor.outtgl = dccMonitor.bit;}

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
// Timer #defines using TCB1
// We have to set the right ISR vector, and  clear the interrupt flag
#define ISR_START    ISR(TCB1_INT_vect)
#define ISR_PROLOGUE {TCB1.INTFLAGS = TCB_CAPT_bm; } // Clear the interrupt flag
#define ISR_EPILOGUE {; }                            // Empty, nothing to do

// TICKS_PER_US       : Number of timer ticks per microsecond, based on F_CPU
// half_one_count     : Timer count for a half "1" bit (29 µs typical)
// one_count          : Timer count for a full "1" bit (58 µs typical)
// zero_count         : Timer count for a full "0" bit (100 µs typical)
#define TICKS_PER_US               (F_CPU / 1000000UL)
#define half_one_count             (TICKS_PER_US * (29))     // RCN 217
#define one_count                  (TICKS_PER_US * (58))     // RCN 210
#define zero_count                 (TICKS_PER_US * (100))    // RCN 210

// Set the Compare Match Register
#define TMR_RAILCOM_STARTBIT       { TCB1.CCMP = half_one_count; }
#define TMR_ONE                    { TCB1.CCMP = one_count; }
#define TMR_ZERO                   { TCB1.CCMP = zero_count; }

// We use TCB1 in periodic interrupt mode and change CCMP of needed
// Switching to other TCBs should be trivial
inline void enableDccTimer() {
  // Configure TCB1 in periodic interrupt mode
  TCB1.CTRLB = TCB_CNTMODE_INT_gc;      // periodic interrupt mode
  TCB1.CCMP = one_count;                // start with a "1" bit (preamble)
  TCB1.INTFLAGS = TCB_CAPT_bm;          // clear pending interrupt flag
  TCB1.INTCTRL  = TCB_CAPT_bm;          // enable capture/timeout interrupt
  TCB1.CTRLA = TCB_CLKSEL_CLKDIV1_gc;   // Set the prescaler to 1
  TCB1.CNT = 0;                         // Make sure the counter is zero
}

inline void startDccTimer() {TCB1.CTRLA |= TCB_ENABLE_bm;}
inline void disableDccTimer() {TCB1.CTRLA = 0;}


//******************************************************************************
// TODO: FOR TESTING
#define DEBUG1 digitalWrite(PIN_PB3, HIGH); digitalWrite(PIN_PB3, LOW);
#define DEBUG2 digitalWrite(PIN_PB4, HIGH); digitalWrite(PIN_PB4, LOW);
#define DEBUG3 digitalWrite(PIN_PD7, HIGH); digitalWrite(PIN_PD7, LOW);


//******************************************************************************
// Part 3: Continue with the processor independent code
//******************************************************************************
#include "DCCHardwareCommon.inc"
