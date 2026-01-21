//******************************************************************************************************
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
// Toggles are performed in the ISR using direct port access (software-controlled).
// Each output (normal rail, inverted rail, monitor) is represented by a small structure
// holding the port register pointer and pin bitmask. This ensures deterministic timing.
//
// Toggling strategy and H-bridge safety:
// - All rail transitions use toggles rather than set/clear to maintain consistent
//   behavior between software- and hardware-generated approaches.
// - When both normal and inverted rail outputs are enabled, their toggles are executed
//   back-to-back to minimize temporal offset and produce the cleanest zero-crossing
//   of the H-bridge output signal.
// - H-bridge safety requirement: both legs must never be high simultaneously to avoid
//   shoot-through. Therefore, the toggle order within each half-bit is critical.
//
//******************************************************************************************************
#include <Arduino.h>
#include "../DCCHardware.h"


// Direct register access for pin changes
// Create a structure for each output pin. This structure provides direct access
// to the AtMega output registers, using a bitmask.
typedef struct {
  volatile uint8_t *port;     // Pointer naar PORTx register
  uint8_t bit;                // Bitmask voor de pin
  uint8_t enable;             // Flag to decide if a pin toggle is allowed
} DccOut_t;

// Instantiate three objects; one for each pin.
static DccOut_t dccMonitor;   // Create an instance for the dccMonitor
static DccOut_t dccRail;      // Create an instance for the dccRail
static DccOut_t dccRailInv;   // Create an instance for the dccRailInv


// ATMega specific #defines to set, clear and toggle the output pins
// Although only some will be used, for completeness all options are given.
#define DCC_OUT_HGH          { *dccRail.port |=  dccRail.bit; }
#define DCC_OUT_LOW          { *dccRail.port &= ~dccRail.bit; }
#define DCC_OUT_TGL          { *dccRail.port ^=  dccRail.bit; }
#define DCC_OUT_INV_HGH      { *dccRailInv.port |=  dccRailInv.bit; }
#define DCC_OUT_INV_LOW      { *dccRailInv.port &= ~dccRailInv.bit; }
#define DCC_OUT_INV_TGL      { *dccRailInv.port ^=  dccRailInv.bit; }
#define DCC_OUT_MON_HGH      { *dccMonitor.port |=  dccMonitor.bit; }
#define DCC_OUT_MON_LOW      { *dccMonitor.port &= ~dccMonitor.bit; }
#define DCC_OUT_MON_TGL      { *dccMonitor.port ^=  dccMonitor.bit; }

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
// Timer macros for DCC waveform generation
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
//   - ISR_START    : Bind the ISR to the Timer1 interrupt vector
//   - ISR_PROLOGUE : Clear the interrupt flag at the start of ISR (not needed)
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
#define half_one_count             ((uint16_t)TICKS_PER_US * (29))   // RCN 217
#define one_count                  ((uint16_t)TICKS_PER_US * (58))   // RCN 210
#define zero_count                 ((uint16_t)TICKS_PER_US * (100))  // RCN 210

// We use TIMER1 in CTC mode mode and reload OCR1A each time
// Switching to other Timer2 should be doable
#define TMR_RAILCOM_STARTBIT()     { OCR1A = half_one_count; }
#define TMR_ONE()                  { OCR1A = one_count; }
#define TMR_ZERO()                 { OCR1A = zero_count; }

// We have to set the right ISR call and vector, and  clear the interrupt flag
#define ISR_START                  ISR(TIMER1_COMPA_vect)
#define ISR_PROLOGUE               {; }   // No need to clear the interrupt flag

// Enable / Disable the various channels
#define ENABLE_DCCMON              {dccMonitor.enable = true;}
#define DISABLE_DCCMON             {dccMonitor.enable = false;}
#define ENABLE_DCCOUT              {dccRail.enable = true;}
#define DISABLE_DCCOUT             {dccRail.enable = false;}
#define ENABLE_DCCINV              {dccRailInv.enable = true;}
#define DISABLE_DCCINV             {dccRailInv.enable = false;}

// TODO: FOR TESTING
#define PIN_TEST1   8
#define PIN_TEST2   9
#define PIN_TEST3   10

#define DEBUG1 digitalWrite(PIN_TEST1, HIGH); digitalWrite(PIN_TEST1, LOW);
#define DEBUG2 digitalWrite(PIN_TEST2, HIGH); digitalWrite(PIN_TEST2, LOW);
#define DEBUG3 digitalWrite(PIN_TEST2, HIGH); digitalWrite(PIN_TEST2, LOW);



//******************************************************************************
// Initialize a DCC output pin for direct register access (ATMega 328 / 2560)
// Parameters:
//   - pinObj: DccOut_t object holding port registers and bit mask
//   - arduinoPin: Arduino pin number; set to 0xFF if unused
//
// For valid pins, this function:
//   - Sets the pin as OUTPUT
//   - Maps the Arduino pin to its PORTx registers
//   - Stores pointers to PORTx registers
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
    pinObj.port = portOutputRegister(port);       // Port register
    pinObj.bit = digitalPinToBitMask(arduinoPin); // pin bitmask
    pinObj.enable = false;                        // Toggle pin not allowed
  }
  else {
    pinObj.port = nullptr;
    pinObj.bit = 0;
    pinObj.enable = false;
  }
}


//******************************************************************************
// Setup DCC waveform generator (ATMega 328 / 2560)
// This function configures the hardware and pins for DCC output:
//   - Stops Timer1 to safely modify registers
//   - Initializes rail, inverted rail, and monitor pins via initDccPin()
//   - Configures Timer1 in CTC mode, sets initial compare value
//   - Clears pending interrupts and enables capture/timeout interrupt
//   - Sets the three pins to their initial states (rail LOW, inverted HIGH,
//     monitor LOW)
//   - Enables all outputs for toggling
//   - Starts with a DCC "0" signal
//
// Timing definitions (one/zero/half-bit) are derived from F_CPU
//******************************************************************************
void DccPacketEngine::setupWaveformGenerator() {
  // We must stop timer1 first, since it is using the same registers as we may
  // subsequently modify. If we don't stop the timer first, the code may crash.
  TCCR1A = 0;   // Normal port operation, disconnect OC1A/OC1B
  TCCR1B = 0;   // Stop the timer by clearing clock select bits
  // Initialise the structure with registers and mask for the three pins
  // If a pin is not used (0xFF), the pointers to the registers will be cleared
  initDccPin(dccRail, dccRailPin);
  initDccPin(dccRailInv, dccRailInvPin);
  initDccPin(dccMonitor, dccMonitorPin);
  // Configure Timer1 in CTC mode.
  TCCR1A = 0;                          // Normal port operation, OC1A/OC1B disconnected
  TCCR1B = (1 << WGM12) | (1 << CS10); // CTC mode, prescaler 1
  OCR1A = zero_count;                  // Start with "0" pulse duration
  TCNT1 = 0;                           // Reset timer count
  TIMSK1 = (1 << OCIE1A);              // Enable Timer1 Compare Match A interrupt
  // Set the three pins to their start value
  DCC_OUT_HGH;
  DCC_OUT_INV_LOW;
  DCC_OUT_MON_HGH;
  // Enable the three output channels
  dccRail.enable = true;
  dccRailInv.enable = true;
  dccMonitor.enable = true;
  // Start with a DCC low-signal
  TMR_ZERO();
}


//******************************************************************************
// Continue with the processor independent code
//******************************************************************************
#include "DCCHardwareCommon.inc"
