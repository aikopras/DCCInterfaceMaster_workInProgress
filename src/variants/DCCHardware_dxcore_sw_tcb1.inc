//******************************************************************************************************
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
//
// This file implements a DxCore-specific hardware layer for the DCC waveform
// generator. It provides the physical interface between the processor-independent
// DCC state machine and the AVR Dx hardware, using a TCB timer as interrupt source
// and direct port access for rail signal control.
//
// Its responsibilities are strictly hardware-related and include:
//   - configuring the TCB timer to generate periodic half-bit interrupts,
//   - binding the generic ISR to the correct interrupt vector,
//   - providing macros and inline functions for timing control,
//   - providing macros and inline functions for controlling the DCC output pins,
//   - initializing and enabling the selected rail and monitor pins.
//
// All DCC protocol logic, packet flow, Service Mode handling, RailCom timing,
// and state machine behaviour are implemented in the processor-independent file
// ../DCCHardware_common.inc, which is included at the end of this file.
//
// This driver uses software-controlled pin toggling (bit-banging) via direct
// register access. The normal rail, inverted rail and monitor outputs are each
// represented by a small structure holding the required port registers and bit mask.
// Output transitions are triggered explicitly by inline functions called from the ISR.
//
// NOTE ON TIMING AND PIN TOGGLE LATENCY
// ------------------------------------
// In this DxCore variant, the rail outputs are toggled in software from within the ISR.
// Because this involves inline functions, structure dereferencing and enable checks,
// each toggle operation expands into multiple CPU instructions. At 24 MHz, the time
// between toggling the inverted and normal rail outputs is on the order of 20 CPU cycles
// (approximately 800–900 ns).
//
// This delay is relatively large compared to the ideal DCC zero-crossing and causes a
// measurable deformation of the waveform. Functionally this is acceptable, but it is
// not electrically optimal.
//
// Possible optimizations include using hard-coded ports and bit masks, removing runtime
// enable checks, toggling both rail pins simultaneously when located on the same port,
// or delegating output toggling to hardware peripherals such as TCA, EVSYS or CCL.
//
// For this reason, a separate TCA-based driver is provided which uses hardware pin
// toggling and produces an ideal, simultaneous zero-crossing, at the cost of reduced
// flexibility in pin selection.
//
//******************************************************************************************************
#include <Arduino.h>
#include "../DCCHardware.h"


// Direct register access for pin changes
// Create a structure for each output pin. This structure provides direct access
// to the DxCore output registers, using a bitmask. Create three variables for all three pins.
typedef struct {
  volatile uint8_t *outset;   // Pointer naar PORTx.OUTSET register
  volatile uint8_t *outclr;   // Pointer naar PORTx.OUTCLR register
  volatile uint8_t *outtgl;   // Pointer naar PORTx.OUTTGL register
  uint8_t bit;                // Bitmask voor de pin
  uint8_t enable;             // Flag to decide if a pin toggle is allowed
} DccOut_t;

static DccOut_t dccMonitor;   // Create an instance of the register struct for the dccMonitor
static DccOut_t dccRail;      // Create an instance of the register struct for the dccRail
static DccOut_t dccRailInv;   // Create an instance of the register struct for the dccRailInv

// We perform a check if the output is enabled, to allow similar software sructures for
// approaches where the pin output is toggled by software or by hardware
inline void dccMonLow()        { if (dccMonitor.enable)  *dccMonitor.outclr = dccMonitor.bit; }
inline void dccMonHigh()       { if (dccMonitor.enable)  *dccMonitor.outset = dccMonitor.bit; }
inline void dccMonTgl()        { if (dccMonitor.enable)  *dccMonitor.outtgl = dccMonitor.bit; }
inline void dccOutLow()        { if (dccRail.enable)     *dccRail.outclr = dccRail.bit; }
inline void dccOutHigh()       { if (dccRail.enable)     *dccRail.outset = dccRail.bit; }
inline void dccOutTgl()        { if (dccRail.enable)     *dccRail.outtgl = dccRail.bit; }
inline void dccOutInvLow()     { if (dccRailInv.enable)  *dccRailInv.outclr = dccRailInv.bit; }
inline void dccOutInvHigh()    { if (dccRailInv.enable)  *dccRailInv.outset = dccRailInv.bit; }
inline void dccOutInvTgl()     { if (dccRailInv.enable)  *dccRailInv.outtgl = dccRailInv.bit; }

// TCB clock = CLK_PER = F_CPU (typically 24 MHz)
// 1 tick = 1 / F_CPU seconds */
// The timer counts are calculated from the selected clockspeed, and are therefore dynamic
#define TICKS_PER_US               (F_CPU / 1000000UL)
#define half_one_count             (TICKS_PER_US * (29))
#define one_count                  (TICKS_PER_US * (58))
#define zero_count                 (TICKS_PER_US * (116))

// We use TCB1 in periodic interrupt mode and reload CCMP each time
// Switching to other TCBs should be trivial
#define TMR_RAILCOM_STARTBIT()     { TCB1.CCMP = half_one_count; }
#define TMR_ONE()                  { TCB1.CCMP = one_count; }
#define TMR_ZERO()                 { TCB1.CCMP = zero_count; }

#define ISR_START                  ISR(TCB1_INT_vect)
#define ISR_PROLOGUE               {TCB1.INTFLAGS = TCB_CAPT_bm; }  // Clear the interrupt flag


#define DEBUG1 digitalWrite(PIN_PB3, HIGH); digitalWrite(PIN_PB3, LOW);
#define DEBUG2 digitalWrite(PIN_PB4, HIGH); digitalWrite(PIN_PB4, LOW);
#define DEBUG3 digitalWrite(PIN_PB5, HIGH); digitalWrite(PIN_PB5, LOW);


//******************************************************************************
// setup_DCC_waveform_generator
//******************************************************************************
// Initialise a DCC output pin for direct register access (DxCore)
// - pinObj: the DccOut_t object that stores the registers and bit mask
// - arduinoPin: any Arduino pin number, 0xFF if the pin is not set
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
  // Enable the three output channels
  dccRail.enable = true;
  dccRailInv.enable = true;
  dccMonitor.enable = true;
  // Set the three pins to their start value
  dccOutLow();
  dccOutInvHigh();
  dccMonLow();
  //dccOutInvTgl();                       // Inverted DCC signal

  // Start with a DCC low-signal
  TMR_ZERO();
}


//******************************************************************************
// Continue with the processor independent code
//******************************************************************************
#include "../DCCHardware_common.inc"
