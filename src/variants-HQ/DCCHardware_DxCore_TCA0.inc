//******************************************************************************************************
//
// file:      DCCHardware_DxCore_TCA0.inc
// author:    Aiko Pras
// history:   2026-03-01 V1.0.0 ap initial version
//
// purpose:   TCA0 specific initialisation code for DxCore and MEGACOREX
//
// This module depends on a global TcaTxEngine instance `tcaTx`.
// Functions configure compare units and ports for TCA0 in SINGLE-SLOPE PWM mode.
// Only pins mapped to compare units 0..2 are supported.
// Return value `true` indicates success, `false` failure (invalid unit or unsupported port).
//
// Can be migrate to TCA1, but that requires (amonst others) different multiplexer lohic
// See the Servo-TCA library how this can be done.
//
//******************************************************************************************************
#pragma once
#include <Arduino.h>


//======================================================================================================
// The following enables an easy switch between TCA0 and TCA1
// Only compile if we have a TCA0 timer, and this is not a MEGATINYCORE
#if defined(TCA0) && !defined(MEGATINYCORE_SERIES)

#define _TIMER        TCA0.SINGLE
#define takeOverTCA() takeOverTCA0()
#define resumeTCA()   resumeTCA0()
#define DCCHandler    TCA0_OVF_vect

//******************************************************************************************************
// The initialisation of the multplexer depends on the processor being used.
// DxCore and MEGACOREX processors require that all compare unit pins belong to the same port
//******************************************************************************************************
static inline boolean configurePortMultiplexer(uint8_t port) {
  boolean configured = false;
  // DxCore will preset the PORTMUX to a certain port during startup.
  // Therefore we have to clear the TCA0 bits before we can set it with new values.
  // Since registers are volatile variables, we have to load it to a temporary variable
  // For details, see DxCore EA48.md
  uint8_t tcaroutea = PORTMUX.TCAROUTEA;
  tcaroutea &= ~PORTMUX_TCA0_gm;                 // mask off the (TCA0) bits we will change.
  switch (port) {
    case PA:
      tcaroutea |= PORTMUX_TCA0_PORTA_gc;
      configured = true;
    break;
    #ifdef PORTB                                 // PORTB is not available on 28/32 pin processors
    case PB:
      tcaroutea |= PORTMUX_TCA0_PORTB_gc;
      configured = true;
    break;
    #endif
    case PC:
      tcaroutea |= PORTMUX_TCA0_PORTC_gc;
      configured = true;
    break;
    case PD:
      tcaroutea |= PORTMUX_TCA0_PORTD_gc;
      configured = true;
    break;
    #ifdef PORTE                                 // PORTE is not available on 28/32 pin processors
    case PE:
      tcaroutea |= PORTMUX_TCA0_PORTE_gc;
      configured = true;
    break;
    #endif
    case PF:
      tcaroutea |= PORTMUX_TCA0_PORTF_gc;
      configured = true;
    break;
    #ifdef PORTG                                 // PORTG is only available on 64 pin processors
    case PG:
      tcaroutea |= PORTMUX_TCA0_PORTG_gc;
      configured = true;
    break;
    #endif
  }
  if (configured) PORTMUX.TCAROUTEA = tcaroutea; // write the temp variable back to the register.
  return configured;
}

static inline bool configureCompareUnit(uint8_t unit) {
  tcaTx.compareUnitRegister = nullptr;
  tcaTx.compareUnitMask = 0;
  switch (unit) {
    case 0:
      tcaTx.compareUnitRegister = &_TIMER.CMP0;
      tcaTx.compareUnitMask     = TCA_SINGLE_CMP0EN_bm;
      break;
    case 1:
      tcaTx.compareUnitRegister = &_TIMER.CMP1;
      tcaTx.compareUnitMask     = TCA_SINGLE_CMP1EN_bm;
      break;
    case 2:
      tcaTx.compareUnitRegister = &_TIMER.CMP2;
      tcaTx.compareUnitMask     = TCA_SINGLE_CMP2EN_bm;
      break;
    default:
      return false;
  }
  return true;
}


static inline void setTcaRegisters() {
  takeOverTCA();          // Tell DxCore that we will use TCA0
  // STEP1: stop the timer (and reset CTRLA), since it is using the same
  // registers as we may subsequently modify.
  _TIMER.CTRLA = 0;
  // STEP 2: Configure CTRL A: Set the prescaler but do NOT yet enable the TCA peripheral
  _TIMER.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc;
  // STEP 3: Configure CTRL B: set the timer mode (single-slope PWM mode)
  _TIMER.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | tcaTx.compareUnitMask;
  // STEP 4: Configure EVCTRL: Disable event counting (which is the default)
  _TIMER.EVCTRL &= ~(TCA_SINGLE_CNTEI_bm);
  // STEP 5: Configure INTCTRL: Enable the overflow interrupt
  _TIMER.INTCTRL = TCA_SINGLE_OVF_bm;
  // STEP 6: Configure the PER (PERIOD) and the CMP registers for the first one pulse
  _TIMER.PER = ONE;
  *tcaTx.compareUnitRegister = ONE / 2;
  // STEP 7: Start TCA
  _TIMER.CTRLA |= TCA_SINGLE_ENABLE_bm;
}


//******************************************************************************************************
//
// Function: initTca
//
// Purpose:
//  1) Determine the port to which the selected pin belongs and configure the TCA0 port multiplexer.
//     All TCA0 compare unit pins must belong to the same port on DxCore / MegaCoreX processors.
//     digitalPinToPort() is used to map the Arduino pin number to the hardware port.
//     If the port is not supported by TCA0, the function returns false.
//  2) Determine which compare unit is attached to that pin.
//     For TCA0 this is straightforward: the first three pins of each port map to compare units:
//       Px0 → Compare Unit 0
//       Px1 → Compare Unit 1
//       Px2 → Compare Unit 2
//     digitalPinToBitPosition() is used to determine the bit position of the pin within the port.
//     If the bit position is outside 0..2, the function returns false.
//
// Notes:
//  - This function configures the global `tcaTx` struct (TcaTxEngine) to point to the proper compare unit
//    register and set its mask for enabling the correct compare channel.
//  - The pin is set to OUTPUT only after all hardware registers have been safely initialized.
//  - Returns `true` if initialization succeeds, `false` if an invalid port or compare unit is selected.
//
// Safety:
//  - tcaTx.compareUnitRegister is set to nullptr initially to prevent accidental dereference.
//  - Hardware registers are written only after successful compare unit and port configuration.
//
//******************************************************************************************************
static boolean initTca(byte pin) {
  uint8_t compareUnit = digitalPinToBitPosition(pin);
  uint8_t newPort = digitalPinToPort(pin);
  if (!configureCompareUnit(compareUnit)) return false;   // Return if there is no valid compare unit
  if (!configurePortMultiplexer(newPort)) return false;   // Return if the multiplexer doesn't support the port
  setTcaRegisters();
  // Create a pointer to PORTx.PINnCTRL. This is needed to enable inverting the DCC signal
  PORT_t* port = digitalPinToPortStruct(pin);             // digitalPinToPortStruct is a DxCore macro
  tcaTx.pinCtrlRegister = &port->PIN0CTRL + compareUnit;
  // Set the pin as output
  pinMode(pin, OUTPUT);
  return true;
}


//******************************************************************************************************
// MegaCoreX (ATmega4809, ATmega4808, ATmega3209, ATmega3208, ATmega1609, ATmega1608, ATmega809 and
// ATmega808) doesn't define takeOverTCA(). Therefore we copied the code from DxCore here,
// and removed several lines that are not needed by MegaCoreX.
//******************************************************************************************************
#if defined(MEGACOREX)
void takeOverTCA0() {
  TCA0.SPLIT.CTRLA = 0;          // Stop TCA0
  TCA0.SPLIT.CTRLESET = TCA_SPLIT_CMD_RESET_gc | 0x03; // Reset TCA0
}

void resumeTCA0() {}

#endif   // MEGACOREX
#endif   // TCA and not MegaTinyCore
