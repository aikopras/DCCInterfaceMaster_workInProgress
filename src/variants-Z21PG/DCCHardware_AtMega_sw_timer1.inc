//******************************************************************************
//
// File:            DCCHardware_atmega_sw_timer1.inc
// Purpose:         DCC Waveform Generator for traditional ATMega  processors (bit-banging)
// Copyright 2026:  Aiko Pras
// History:         2026-01-21 / Aiko Pras - First version
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
// Implements the hardware layer for ATMega processors (e.g., 328, 2560).
// Connects the processor-independent DCC state machine (DCCHardwareCommon.inc)
// to the ATMega hardware. It provides:
//   - Timer1 configuration for precise DCC bit-timing interrupts
//   - Binding the ISR to the correct Timer1 vector
//   - Inline functions/macros for timing and pin control
//   - Initialization and enabling of rail and monitor outputs
//
// Pin control and toggling:
// The driver uses software-controlled toggles (bit-banging) via
// direct register access.
// Each output (normal rail, inverted rail, monitor) is represented by a
// small structure, holding the port register pointer and pin bitmask.
// This ensures deterministic timing.
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
// Create a structure for each output pin. This structure provides direct access
// to the AtMega output registers, using a bitmask.
typedef struct {
  volatile uint8_t *port;     // Pointer naar PORTx register
  uint8_t bit;                // Bitmask voor de pin
  uint8_t enable;             // No pin toggle during RailCom gap / Power-down
} DccOut_t;

// Instantiate three objects; one for each pin.
static DccOut_t dccMonitor;   // Create an instance for the dccMonitor
static DccOut_t dccRail;      // Create an instance for the dccRail
static DccOut_t dccRailInv;   // Create an instance for the dccRailInv

// Initialize a DCC output signal (pin) for direct register access (328 / 2560)
// Parameters:
//   - pinObj: DccOut_t object holding port registers and bit mask
//   - arduinoPin: Arduino pin number; set to 0xFF if unused
// For valid pins, this function:
//   - Sets the pin as OUTPUT
//   - Maps the Arduino pin to its PORTx registers
//   - Stores pointers to PORTx registers
//   - Stores the pin bit mask in the structure
//   - Disables toggling initially (enable = false)
// If the pin is 0xFF (not used), all pointers are set to nullptr and enable
// is false
inline void initDccPin(DccOut_t &pinObj, uint8_t arduinoPin) {
  if (arduinoPin != 0xFF) {
    pinMode(arduinoPin, OUTPUT);                  // Port is output
    uint8_t port = digitalPinToPort(arduinoPin);  // Mapping from pin to port
    pinObj.port = portOutputRegister(port);       // Port register
    pinObj.bit = digitalPinToBitMask(arduinoPin); // pin bitmask
    pinObj.enable = false;                        // Toggle pin not allowed
    *pinObj.port &= ~pinObj.bit;                  // Set output pin low
  }
  else {
    pinObj.port = nullptr;
    pinObj.bit = 0;
    pinObj.enable = false;
  }
}

// #defines to set, clear and toggle the output signals
// Although only some will be used, for completeness all options are given.
#define DCCOUT_HGH      { *dccRail.port |=  dccRail.bit; }
#define DCCOUT_LOW      { *dccRail.port &= ~dccRail.bit; }
#define DCCOUT_TGL      { *dccRail.port ^=  dccRail.bit; }
#define DCCINV_HGH      { *dccRailInv.port |=  dccRailInv.bit; }
#define DCCINV_LOW      { *dccRailInv.port &= ~dccRailInv.bit; }
#define DCCINV_TGL      { *dccRailInv.port ^=  dccRailInv.bit; }
#define DCCMON_HGH      { *dccMonitor.port |=  dccMonitor.bit; }
#define DCCMON_LOW      { *dccMonitor.port &= ~dccMonitor.bit; }
#define DCCMON_TGL      { *dccMonitor.port ^=  dccMonitor.bit; }

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
// Timer #defines using TIMER1
// Switching to TIMER2 should be doable
// We have to set the right ISR call and vector
#define ISR_START                  ISR(TIMER1_COMPA_vect)
#define ISR_PROLOGUE {; }          // Empty, no need to do anything
#define ISR_EPILOGUE {; }          // Empty, no need to do anything

// Timer macros for DCC waveform generation
// TICKS_PER_US       : Number of timer ticks per microsecond, based on F_CPU
// half_one_count     : Timer count for a half "1" bit (29 µs typical)
// one_count          : Timer count for a full "1" bit (58 µs typical)
// zero_count         : Timer count for a full "0" bit (100 µs typical)
#define TICKS_PER_US               (F_CPU / 1000000UL)
#define half_one_count             ((uint16_t)TICKS_PER_US * (29))   // RCN 217
#define one_count                  ((uint16_t)TICKS_PER_US * (58))   // RCN 210
#define zero_count                 ((uint16_t)TICKS_PER_US * (100))  // RCN 210

// Set the Compare Match Register
#define TMR_RAILCOM_STARTBIT       { OCR1A = half_one_count; }
#define TMR_ONE                    { OCR1A = one_count; }
#define TMR_ZERO                   { OCR1A = zero_count; }

// We use TIMER1 in CTC mode mode and reload OCR1A each time an match occurs
inline void enableDccTimer() {
  TCCR1A = 0;                          // Normal port operation, OC1A/OC1B disconnected
  TCCR1B = (1 << WGM12);               // CTC mode. Do not start yet
  OCR1A = one_count;                   // start with a "1" bit (preamble)
  TCNT1 = 0;                           // Make sure the counter is zero
  TIMSK1 = (1 << OCIE1A);              // Allow Timer1 Compare Match interrupt
}

inline void startDccTimer() {
  TCCR1B |= (1 << CS10);   // Timer starts (prescaler = 1)
}

inline void disableDccTimer() {
  TCCR1A = 0;   // Normal port operation, disconnect OC1A/OC1B
  TCCR1B = 0;   // Stop the timer by clearing clock select bits
}


//******************************************************************************
// TODO: FOR TESTING
#define PIN_TEST1   8
#define PIN_TEST2   9
#define PIN_TEST3   10

#define DEBUG1 digitalWrite(PIN_TEST1, HIGH); digitalWrite(PIN_TEST1, LOW);
#define DEBUG2 digitalWrite(PIN_TEST2, HIGH); digitalWrite(PIN_TEST2, LOW);
#define DEBUG3 digitalWrite(PIN_TEST2, HIGH); digitalWrite(PIN_TEST2, LOW);


//******************************************************************************
// Part 3: Continue with the processor independent code
//******************************************************************************
#include "DCCHardwareCommon.inc"
