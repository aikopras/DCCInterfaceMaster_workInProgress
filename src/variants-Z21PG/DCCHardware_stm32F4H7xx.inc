//******************************************************************************
//
// File:            DCCHardware_dxcore_sw_tcb1.inc
// Purpose:         DCC Waveform Generator for STM32F4xx processors (bit-banging)
// Copyright 2026:  Rob van Hoeijen
// History:         2026-03-06 / Rob van Hoeijen - First version
//
//
// Historical credits:
// - Aiko Pras
//     * DCC Waveform Generator for DxCore processors (bit-banging)
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
// This file implements a STM32F4xx-specific hardware layer connecting the
// processor-independent DCC state machine (DCCHardwareCommon.inc) to the
// STM32F4 hardware. It provides:
//   - Configuration of a Universal timer for periodic half-bit interrupts
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
#include "HardwareTimer.h"

//******************************************************************************
// Part 1: Pins en output signals
//******************************************************************************
// Direct register access for pin changes
// Create a structure for each output signal. This structure provides
// direct access to the DxCore output registers, using a bitmask.
struct DccOut_t {
  volatile uint32_t *outset;   // BSRR register (set)
  volatile uint32_t *outclr;   // BSRR register (reset via <<16)
  volatile uint32_t *outtgl;   // ODR register (toggle)
  uint32_t bit;                // pin bitmask
  bool enable;
};

extern "C" void DCC_Timer_ISR(void);
// define timer object
HardwareTimer *DccTimer = nullptr;

// Instantiate three objects; one for each output signal
static DccOut_t dccMonitor;   // Create an instance for the dccMonitor
static DccOut_t dccRail;      // Create an instance for the dccRail
static DccOut_t dccRailInv;   // Create an instance for the dccRailInv
// static DccOut_t debug; // create instance for debug port
// Initialize a DCC output signal (pin) for direct register access
// Parameters:
//   - pinObj: DccOut_t object holding port registers and bit mask
//   - arduinoPin: Arduino pin number; set to 0xFF if unused
// For valid pins, this function:
//   - Sets the pin as OUTPUT
//   - Maps the Arduino pin to its PORTx registers
//   - Stores pointers to BSSR, ODR registers
//   - Stores the pin bit mask in the structure
//   - Disables toggling initially (enable = false)
// If the pin is 0xFF (not used), all pointers are set to nullptr and enable
// is false
inline void initDccPin(DccOut_t &pinObj, uint8_t arduinoPin)
{
  if (arduinoPin != 0xFF) {

    pinMode(arduinoPin, OUTPUT);

    PinName p = digitalPinToPinName(arduinoPin);
    GPIO_TypeDef *port = get_GPIO_Port(STM_PORT(p));

    pinObj.outset = &(port->BSRR);
    pinObj.outclr = &(port->BSRR);
    pinObj.outtgl = &(port->ODR);

    pinObj.bit = STM_LL_GPIO_PIN(p);
    pinObj.enable = false;

    *pinObj.outclr = pinObj.bit << 16;   // start LOW

  } else {

    pinObj.outset = nullptr;
    pinObj.outclr = nullptr;
    pinObj.outtgl = nullptr;
    pinObj.bit = 0;
    pinObj.enable = false;
  }
}


// #defines to set, clear and toggle the output signals
// Although only some will be used, for completeness all options are given.
#define DCCOUT_HGH          {*dccRail.outset = dccRail.bit;   }
#define DCCOUT_LOW          {*dccRail.outclr = dccRail.bit << 16;}
#define DCCOUT_TGL          {*dccRail.outtgl ^= dccRail.bit;}
#define DCCINV_HGH          {*dccRailInv.outset = dccRailInv.bit;}
#define DCCINV_LOW          {*dccRailInv.outclr = dccRailInv.bit << 16;}
#define DCCINV_TGL          {*dccRailInv.outtgl ^= dccRailInv.bit;}
#define DCCMON_HGH          {*dccMonitor.outset = dccMonitor.bit;}
#define DCCMON_LOW          {*dccMonitor.outclr = dccMonitor.bit << 16;}
#define DCCMON_TGL          {*dccMonitor.outtgl ^= dccMonitor.bit;}
// debug pins
#define DEBUG_HGH          {*debug.outset = debug.bit;}
#define DEBUG_LOW          {*debug.outclr = debug.bit << 16;}
#define DEBUG_TGL          {*debug.outtgl ^= debug.bit;}

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
// Timer #defines using TIM3
// We have to set the right ISR vector, and  clear the interrupt flag
#define ISR_START    extern \
"C" \
void \
DCC_Timer_ISR(void)
//******************************************************************************
// TODO: FOR TESTING
#define DEBUG1 digitalWrite(PE9, HIGH); digitalWrite(PE9, LOW);
#define DEBUG2 digitalWrite(PPF13, HIGH); digitalWrite(PF13, LOW);


#define ISR_PROLOGUE {; } // Empty, nothing to do
#define ISR_EPILOGUE {; } // Empty, nothing to do

// TICKS_PER_US       : Number of timer ticks per microsecond, based on F_CPU
// half_one_count     : Timer count for a half "1" bit (29 µs typical)
// one_count          : Timer count for a full "1" bit (58 µs typical)
// zero_count         : Timer count for a full "0" bit (100 µs typical)
#define TICKS_PER_US               1   // 1 usec ticks bsed opun processor TIM3 clock
#define half_one_count             (TICKS_PER_US * (29))     // RCN 217
#define one_count                  (TICKS_PER_US * (58))     // RCN 210
#define zero_count                 (TICKS_PER_US * (100))    // RCN 210

// Set the Compare Match Register
#define TMR_RAILCOM_STARTBIT        TIM3->ARR = half_one_count
#define TMR_ONE                     TIM3->ARR = one_count
#define TMR_ZERO                    TIM3->ARR = zero_count



inline void enableDccTimer() {

     /* >>> TIM3 CLOCK ENABLE <<< */

      __HAL_RCC_TIM3_CLK_ENABLE();

	   // define timer object
	   DccTimer = new HardwareTimer(TIM3);

    // 1 MHz timer  1 us resolutie
     //  Prescaler  1 MHz (exact zoals PSC = 83)
    uint32_t timerClk = DccTimer->getTimerClkFreq(); // 84 MHz
	  uint32_t factor = timerClk / 1000000;
    DccTimer->setPrescaleFactor(factor);

    // Start met zero_count  us
    DccTimer->setOverflow(one_count);
    // disable preload of ARR register if enabled this
    // cayses DCC stream to shift one half bit
    TIM3->CR1 &= ~TIM_CR1_ARPE;

    // Attach interrupt
    DccTimer->attachInterrupt(DCC_Timer_ISR);


    // set the priority of the TIM3 iRQ handler to highest
    // priority to avoud jitten in dcc stream
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);


  }

inline void startDccTimer() {TIM3->CR1   |= TIM_CR1_CEN; }
inline void disableDccTimer() {TIM3->CR1   &= ~TIM_CR1_CEN; }




//******************************************************************************
// Part 3: Continue with the processor independent code
//******************************************************************************
#include "DCCHardwareCommon.inc"
