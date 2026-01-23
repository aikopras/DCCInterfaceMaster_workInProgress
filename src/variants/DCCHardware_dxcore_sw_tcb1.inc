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
// Toggling strategy and H-bridge safety:
// - All rail transitions use toggles rather than set/clear to maintain
//   consistent behavior between software- and hardware-generated approaches.
// - When both normal and inverted rail outputs are enabled, their toggles are
//   executed back-to-back to minimize temporal offset and produce the cleanest
//   zero-crossing of the H-bridge output signal.
// - H-bridge safety requirement: both legs must never be high simultaneously
//   to avoid shoot-through.
//   Therefore, the toggle order within each half-bit is critical.
//
//******************************************************************************
#include <Arduino.h>
#include "../DCCHardware.h"


// Direct register access for pin changes
// Create a structure for each output pin. This structure provides direct access
// to the DxCore output registers, using a bitmask.
typedef struct {
  volatile uint8_t *outset;   // Pointer naar PORTx.OUTSET register
  volatile uint8_t *outclr;   // Pointer naar PORTx.OUTCLR register
  volatile uint8_t *outtgl;   // Pointer naar PORTx.OUTTGL register
  uint8_t bit;                // Bitmask voor de pin
  uint8_t enable;             // Flag to decide if a pin toggle is allowed
} DccOut_t;

// Instantiate three objects; one for each pin.
static DccOut_t dccMonitor;   // Create an instance for the dccMonitor
static DccOut_t dccRail;      // Create an instance for the dccRail
static DccOut_t dccRailInv;   // Create an instance for the dccRailInv


// TCB specific #defines to set, clear and toggle the output pins
// Although only some will be used, for completeness all options are given.
#define DCC_OUT_HGH          {*dccRail.outset = dccRail.bit;}
#define DCC_OUT_LOW          {*dccRail.outclr = dccRail.bit;}
#define DCC_OUT_TGL          {*dccRail.outtgl = dccRail.bit;}
#define DCC_OUT_INV_HGH      {*dccRailInv.outset = dccRailInv.bit;}
#define DCC_OUT_INV_LOW      {*dccRailInv.outclr = dccRailInv.bit;}
#define DCC_OUT_INV_TGL      {*dccRailInv.outtgl = dccRailInv.bit;}
#define DCC_OUT_MON_HGH      {*dccMonitor.outset = dccMonitor.bit;}
#define DCC_OUT_MON_LOW      {*dccMonitor.outclr = dccMonitor.bit;}
#define DCC_OUT_MON_TGL      {*dccMonitor.outtgl = dccMonitor.bit;}


//******************************************************************************
// Timer and TCB macros for DCC waveform generation
//
// TICKS_PER_US       : Number of timer ticks per microsecond, based on F_CPU
// half_one_count     : Timer count for a half "1" bit (29 µs typical)
// one_count          : Timer count for a full "1" bit (58 µs typical)
// zero_count         : Timer count for a full "0" bit (100 µs typical)
//
// Timer setup macros:
//   - TMR_RAILCOM_STARTBIT(): Load timer for RailCom start-bit
//   - TMR_ONE()             : Load timer for "1" bit duration
//   - TMR_ZERO()            : Load timer for "0" bit duration
//
// ISR macros:
//   - ISR_START    : Bind the ISR to TCB1 interrupt vector
//   - ISR_PROLOGUE : Clear the interrupt flag at the start of ISR
//
// Pin enable/disable macros:
//   - ENABLE_DCCOUT / DISABLE_DCCOUT  : Enable or disable normal rail output
//   - ENABLE_DCCINV / DISABLE_DCCINV  : Enable or disable inverted rail output
//   - ENABLE_DCCMON / DISABLE_DCCMON  : Enable or disable monitor output
//
// These macros simplify ISR and setup code, providing clear and consistent
// naming for timer counts and pin control across the driver.
//******************************************************************************
#define TICKS_PER_US               (F_CPU / 1000000UL)
#define half_one_count             (TICKS_PER_US * (29))     // RCN 217
#define one_count                  (TICKS_PER_US * (58))     // RCN 210
#define zero_count                 (TICKS_PER_US * (100))    // RCN 210

// We use TCB1 in periodic interrupt mode and reload CCMP each time
// Switching to other TCBs should be trivial
#define TMR_RAILCOM_STARTBIT()     { TCB1.CCMP = half_one_count; }
#define TMR_ONE()                  { TCB1.CCMP = one_count; }
#define TMR_ZERO()                 { TCB1.CCMP = zero_count; }

// We have to set the right ISR call and vector, and  clear the interrupt flag
#define ISR_START                  ISR(TCB1_INT_vect)
#define ISR_PROLOGUE               {TCB1.INTFLAGS = TCB_CAPT_bm; }

// Enable / Disable the various channels
#define ENABLE_DCCMON              {dccMonitor.enable = true;}
#define DISABLE_DCCMON             {dccMonitor.enable = false;}
#define ENABLE_DCCOUT              {dccRail.enable = true;}
#define DISABLE_DCCOUT             {dccRail.enable = false;}
#define ENABLE_DCCINV              {dccRailInv.enable = true;}
#define DISABLE_DCCINV             {dccRailInv.enable = false;}


// TODO: FOR TESTING
#define DEBUG1 digitalWrite(PIN_PB3, HIGH); digitalWrite(PIN_PB3, LOW);
#define DEBUG2 digitalWrite(PIN_PB4, HIGH); digitalWrite(PIN_PB4, LOW);
#define DEBUG3 digitalWrite(PIN_PB5, HIGH); digitalWrite(PIN_PB5, LOW);


//******************************************************************************
// Initialize a DCC output pin for direct register access (DxCore)
// Parameters:
//   - pinObj: DccOut_t object holding port registers and bit mask
//   - arduinoPin: Arduino pin number; set to 0xFF if unused
//
// For valid pins, this function:
//   - Sets the pin as OUTPUT
//   - Maps the Arduino pin to its PORTx registers
//   - Stores pointers to OUTSET, OUTCLR, OUTTGL registers
//   - Stores the pin bit mask in the structure
//   - Disables toggling initially (enable = false)
//
// If the pin is 0xFF (not used), all pointers are set to nullptr and enable
// is false
//******************************************************************************
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
  }
  else {
    pinObj.outset = nullptr;
    pinObj.outclr = nullptr;
    pinObj.outtgl = nullptr;
    pinObj.bit = 0;
    pinObj.enable = false;
  }
}


//******************************************************************************
// Setup DCC waveform generator (DxCore, TCB1)
// This function configures the hardware and pins for DCC output:
//   - Stops TCB1 to safely modify registers
//   - Initializes rail, inverted rail, and monitor pins via initDccPin()
//   - Configures TCB1 in periodic interrupt mode, sets initial compare value
//   - Clears pending interrupts and enables capture/timeout interrupt
//   - Sets the three pins to their initial states (rail LOW, inverted HIGH,
//     monitor LOW)
//   - Enables all outputs for toggling
//   - Starts with a DCC "0" signal
//
// Timing definitions (one/zero/half-bit) are derived from F_CPU
//******************************************************************************
void DccPacketEngine::setupWaveformGenerator() {
  // We must stop the timer, since it is using the same registers as we may
  // subsequently modify. If we don't stop the timer first, the code may crash.
  TCB1.CTRLA = 0;                       // stop + reset control A
  // Initialise the structure with registers and mask for the three pins
  // If a pin is not used (0xFF), the pointers to the registers will be cleared
  initDccPin(dccRail, dccRailPin);
  initDccPin(dccRailInv, dccRailInvPin);
  initDccPin(dccMonitor, dccMonitorPin);
  // DCC_USES_TCB1 - DxCore
  TCB1.CTRLB = TCB_CNTMODE_INT_gc;      // periodic interrupt mode
  TCB1.CCMP  = one_count;               // start with "1" timing
  TCB1.INTFLAGS = TCB_CAPT_bm;          // clear pending interrupt flag
  TCB1.INTCTRL  = TCB_CAPT_bm;          // enable capture/timeout interrupt
  TCB1.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;   // CLK_PER, enable
  // Set the three pins to their start value
  DCC_OUT_LOW;
  DCC_OUT_INV_HGH;
  DCC_OUT_MON_LOW;
  // Enable the three output channels
  dccRail.enable = true;
  dccRailInv.enable = true;
  dccMonitor.enable = true;

  // Start with a DCC low-signal
  TMR_ZERO();
}


//******************************************************************************
// First half-bit toggle
// During the first half-bit, the normal rail output starts LOW and the
// inverted rail output starts HIGH. To maintain proper H-bridge operation
// and clean zero-crossing:
//   - When both rails are enabled, the inverted rail is toggled first,
//     immediately followed by the normal rail.
//   - The monitor output is toggled if enabled.
// This order is critical to avoid shoot-through.
// In hardware-assisted mode (toggle-on-compare-match), this function will be
// empty, as the timer handles the transitions automatically.
//******************************************************************************
inline void dccFirstHalfBitToggle(void) {
  if (dccRailInv.enable && dccRail.enable) {
    DCC_OUT_INV_TGL;
    DCC_OUT_TGL;
  } else {
    if (dccRailInv.enable) DCC_OUT_INV_TGL;
    if (dccRail.enable) DCC_OUT_TGL;
    }
  if (dccMonitor.enable) DCC_OUT_MON_TGL;
}

//******************************************************************************
// Second half-bit toggle
// During the second half-bit, the normal rail output starts HIGH and the
// inverted rail output starts LOW.
// To maintain proper H-bridge operation and clean zero-crossing:
//   - When both rails are enabled, the normal rail is toggled first,
//     immediately followed by the inverted rail.
//   - The monitor output is toggled if enabled.
// This order is critical to avoid shoot-through.
// In hardware-assisted mode (toggle-on-compare-match), this function will be
// empty, as the timer handles the transitions automatically.
//******************************************************************************
inline void dccSecondHalfBitToggle(void) {
  if (dccRailInv.enable && dccRail.enable) {
    DCC_OUT_TGL;
    DCC_OUT_INV_TGL;
  } else {
    if (dccRailInv.enable) DCC_OUT_INV_TGL;
    if (dccRail.enable) DCC_OUT_TGL;;
    }
  if (dccMonitor.enable) DCC_OUT_MON_TGL;
}


//******************************************************************************
// Continue with the processor independent code
//******************************************************************************
#include "DCCHardwareCommon.inc"
