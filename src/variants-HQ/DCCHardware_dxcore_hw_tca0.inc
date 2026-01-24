//******************************************************************************************************
//
// File:            DCCHardware_dxcore_hw_tca0.inc
// Purpose:         DCC Waveform Generator for DxCore processors
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
// TODO: AANPASSEN VOOR TCA

// This file implements a DxCore-specific hardware layer for the DCC waveform
// generator. It provides the physical interface between the processor-independent
// DCC state machine and the AVR Dx hardware, using a TCA timer as interrupt source
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
//
// Principle: TCA runs in Normal mode. In this mode the Counter Register (CNT) counts up, until it
// reaches the Compare Register (CMPn). When TOP is reached, the counter wraps to ‘0’.
// TCA is configured in NORMAL mode
//******************************************************************************************************
#include <Arduino.h>
#include "../DCCHardware.h"


// Set the output port pins
// The default is PD0..PD2, since these pins exist on many DxCore variants, and are not used for
// important other peripherals, such as USART0. Changing to another port should be easy, however,
// by changing PORTMUX_TCA0_PORTD_gc to the desired port as well as the three pins.
#define DCC_PORTMUX           PORTMUX_TCA0_PORTD_gc
#define DCC_MONITOR_PIN       PIN_PD0                  // The CMP0 channel
#define DCC_RAIL_PIN          PIN_PD1                  // The CMP1 channel
#define DCC_RAIL_INV_PIN      PIN_PD2                  // The CMP2 channel


// TCA clock = CLK_PER = F_CPU (typically 24 MHz)
// 1 tick = 1 / F_CPU seconds
// The timer counts are calculated from the selected clockspeed, and are therefore dynamic
#define TICKS_PER_US               (F_CPU / 1000000UL)
#define half_one_count             (TICKS_PER_US * (29))     // RCN 217
#define one_count                  (TICKS_PER_US * (58))     // RCN 210
#define zero_count                 (TICKS_PER_US * (100))    // RCN 210

// We can not use TCA in NORMAL mode, since that inhibits pin toggles on
// compare match. We must therefore use TCA0 in SINGLESLOPE PWM mode.
// Switching to TCA1 should be trivial
#define TMR_RAILCOM_STARTBIT()    {TCA0.SINGLE.PER = half_one_count;    \
                                   TCA0.SINGLE.CMP0 = half_one_count;   \
                                   TCA0.SINGLE.CMP1 = half_one_count;   \
                                   TCA0.SINGLE.CMP2 = half_one_count;}
#define TMR_ONE()                 {TCA0.SINGLE.PER = one_count;         \
                                   TCA0.SINGLE.CMP0 = one_count;        \
                                   TCA0.SINGLE.CMP1 = one_count;        \
                                   TCA0.SINGLE.CMP2 = one_count;}
#define TMR_ZERO()                {TCA0.SINGLE.PER = zero_count;       \
                                   TCA0.SINGLE.CMP0 = zero_count;       \
                                   TCA0.SINGLE.CMP1 = zero_count;       \
                                   TCA0.SINGLE.CMP2 = zero_count;}

// We have to set the right ISR call and vector, and  clear the interrupt flag
#define ISR_START                  ISR(TCA0_OVF_vect)
#define ISR_PROLOGUE               {TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm; }


// Enable / Disable the various channels
#define ENABLE_DCCMON     {TCA0.SINGLE.CTRLB |= TCA_SINGLE_CMP0EN_bm;}

//#define DISABLE_DCCMON    {TCA0.SINGLE.CTRLB &= ~ TCA_SINGLE_CMP0EN_bm;}
#define DISABLE_DCCMON    {;}
/*
#define ENABLE_DCCOUT     {TCA0.SINGLE.CTRLB |= TCA_SINGLE_CMP1EN_bm;}
#define DISABLE_DCCOUT    {TCA0.SINGLE.CTRLB &= ~ TCA_SINGLE_CMP1EN_bm;}
#define ENABLE_DCCINV     {TCA0.SINGLE.CTRLB |= TCA_SINGLE_CMP2EN_bm;}
#define DISABLE_DCCINV    {TCA0.SINGLE.CTRLB &= ~ TCA_SINGLE_CMP2EN_bm;}
*/


#define ENABLE_DCCOUT     {TCA0.SINGLE.CMP1 = one_count;}
#define DISABLE_DCCOUT    {TCA0.SINGLE.CMP1 = 0xFFFF;}
#define ENABLE_DCCINV     {TCA0.SINGLE.CMP2 = one_count;}
#define DISABLE_DCCINV    {TCA0.SINGLE.CMP2 = 0xFFFF;}


// Some empty #defines / inline code. These are needed for approaches with
// software
#define DCC_OUT_LOW     {;}
#define DCC_OUT_INV_LOW {;}
inline void dccFirtsHalfBitToggle(void)  {;}
inline void dccSecondHalfBitToggle(void) {;}

#define DEBUG1 digitalWrite(PIN_PA3, HIGH); digitalWrite(PIN_PA3, LOW);
#define DEBUG2 digitalWrite(PIN_PA4, HIGH); digitalWrite(PIN_PA4, LOW);
#define DEBUG3 digitalWrite(PIN_PA5, HIGH); digitalWrite(PIN_PA5, LOW);


//******************************************************************************
// setup_DCC_waveform_generator
//******************************************************************************
void DccPacketEngine::setupWaveformGenerator() {

  takeOverTCA0();


  // STEP1: stop the timer (and reset CTRLA), since it is using the same
  // registers as we may subsequently modify.
  TCA0.SINGLE.CTRLA = 0;
  // STEP 2: Direct the Port Multiplexer to the selected output port, and
  // configure the output pins for output.
  // The port and pins may be changed via the #defines above
  PORTMUX.TCAROUTEA = DCC_PORTMUX;
  pinMode(DCC_MONITOR_PIN, OUTPUT);
  pinMode(DCC_RAIL_PIN, OUTPUT);
  pinMode(DCC_RAIL_INV_PIN, OUTPUT);
  // STEP 3: Avoid that DxCore will configure TCA0
//  takeOverTCA0();
  // STEP 4: Set the TOP (PER = Period) Register to trigger the ISR
  // In addition, we set the CMP Registers for each Compare Unit for the
  // compare matches, thus the times the pins will toggle. The Normal and
  // Monitor start with a DCC 1 signal. The Inverted signal must skip
  // this first toggle moment, and will therefore be set on a higher time
  // (in fact, any time larger that the time for the first ISR will do).
  TCA0.SINGLE.CMP0 = one_count;
  TCA0.SINGLE.CMP1 = one_count;
  TCA0.SINGLE.CMP2 = 2 * one_count;
  //STEP 5: Set the timer mode to normal. Do not yet enable the compare units
  TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_FRQ_gc;

  TCA0.SINGLE.CTRLC =
      TCA_SINGLE_CMP0OV_bm
    | TCA_SINGLE_CMP1OV_bm
    | TCA_SINGLE_CMP2OV_bm;

  // STEP 6: Enable overflow interrupt
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
  // STEP 7: Disable event counting (optional, since this is the default)
  TCA0.SINGLE.EVCTRL &= ~(TCA_SINGLE_CNTEI_bm);
  // STEP 8: Set the prescaler and enble TCA0 to start
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc         // set Prescaler to 1
                    | TCA_SINGLE_ENABLE_bm;             // start in normal mode
  ENABLE_DCCMON;
  ENABLE_DCCOUT;
  ENABLE_DCCINV;

  TCA0.SINGLE.CTRLB =
      TCA_SINGLE_WGMODE_SINGLESLOPE_gc
    | TCA_SINGLE_CMP0EN_bm
    | TCA_SINGLE_CMP1EN_bm
    | TCA_SINGLE_CMP2EN_bm;

}



//******************************************************************************
// Continue with the processor independent code
//******************************************************************************
#include "DCCHardwareCommon.inc"
