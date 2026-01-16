//******************************************************************************************************
//
// File:            DCCHardware_dxcore.inc
// Purpose:         DCC Waveform Generator for DxCore processors (bit-banging)
// Copyright 2026:  Aiko Pras
// History:         2026-12-09 / Aiko Pras - First version
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
// - The general approach follows the original software and still uses bit-banging
// - Code has been split into separate drivers for different processors/hardware
//   (mechanism: one .inc driver file per processor)
// - Code has been cleaned up to improve readability; the external interface (.h)
//   has been adapted into a C++ class structure
// - Minor improvements to the state machine, such as timing around the RailCom start bit
// - This version currently serves as a foundation for future developments
//
// This code variant implements the DCC signal generation fully in software,
// where the three DCC-related pins are abstracted and toggled logically,
// allowing virtually any physical pin to be selected.
// The interrupt fires on every timer expiration and drives a state machine that represents the
// DCC protocol flow. A key concept is the distinction between the first and second halfbit of a
// DCC bit, which determines what actions are taken.
// The ISR therefore uses two switch–case constructs: one for the first halfbit
// and one for the second. During the first halfbit, the focus is mainly on configuring the next
// timer interval (e.g. for ‘0’, ‘1’, or timing the Railcom gap). During the second halfbit, the
// timer remains unchanged and the state machine is advanced to the next logical DCC state.
//
// NOTE ON TIMING AND PIN TOGGLE LATENCY
// ------------------------------------
// This driver generates the DCC waveform fully in software using a TCB timer and bit-banging,
// in order to allow maximum flexibility in the choice of output pins (normal, inverted and monitor
// outputs can be mapped to any pin).
// Because the DCC outputs are toggled via C++ inline functions using RAM-based structs,
// pointer dereferencing and enable checks, each pin toggle requires multiple CPU instructions.
// At 24 MHz this results in approximately 20 CPU cycles (~875 ns) between toggling the inverted
// and normal rail outputs.
//
// This delay is relatively large compared to the ideal DCC zero-crossing and deforms the DCC waveform.
//
// Possible optimizations to reduce this delay include:
//  - Using direct port access with hard-coded ports and bit masks
//  - Removing runtime enable checks
//  - Toggling both rail pins simultaneously when located on the same port
//  - Using hardware-based toggling (TCA / EVSYS / CCL)
//
// A separate TCA-based driver is provided which uses hardware pin toggling
// and produces an ideal, simultaneous zero-crossing, but at the cost of
// reduced flexibility in pin selection.
//
//******************************************************************************************************
#include <Arduino.h>
#include "../DCCHardware.h"


#define PREAMBLE_LENGTH 17        	// see RCN-211 (Ausgabe 23.07.2023)
#define PREAMBLE_LENGTH_SM 20       // see RCN-216 (Ausgabe 24.11.2025)
#define RAILCOM_CUTOUT_LENGTH 4	  	// see RCN-217 (Ausgabe 24.11.2025)


//******************************************************************************************************
// 1. Declaration and initialisation (constructor) of external object
//******************************************************************************************************
// dccPacketEngine is the object that controls packet transmission for different hardware
DccPacketEngine dccPacketEngine;


// Constructor definition
DccPacketEngine::DccPacketEngine(){
  dccRailPin = 6;                                 // Pin for DCC rail Signal, with RailCom support
  dccRailInvPin = 0xFF;	                          // Pin for DCC rail Signal, with RailCom support. Inverted
  dccMonitorPin = 0xFF;	                          // Pin for a continuous DCC Signal, ignoring the RailCom gap / Railpower
  // Packet related
  isWaiting = true;                               // notify to update() that we need the first packet
  dataSize = 0;
  for(int i=0; i<MaxDccSize; i++) {data[i] = 0;}
}


//******************************************************************************************************
// 2. Declaration and initialisation of internal stuff
//******************************************************************************************************
// Enumerated type describing the ISR state machine for DCC signal generation.
// Each state spans one or more DCC bits (2 × 58 µs for a '1', or 2 × 100 µs for a '0').
// A DCC bit consists of two halfbits; the flag 'firstHalfBit' indicates which half is active.
// This allows the ISR to control timing, evaluate state transitions, and update variables.
// The dccRailComStartBit is a special case: it is logically a '1' bit,
// but the signal is suppressed halfway through the first halfbit (after 29 µs).
// Therefore, the first halfbit of the RailCom start bit is split into two phases.
// The flag 'firstHalfBitRC' indicates which phase of this RailCom halfbit is active.
// The state 'dccPreambleStartBit' is used only immediately after the RailCom gap to
// reset variables and reinitialize the DCC bit stream.
typedef enum  {
  dccRailComStartBit,                 // 29us pulse to indicate the start of the gap
  dccRailComGap,                      // Together with the RailCom startbit, a gap of four 1s
  dccPreambleStartBit,                // First bit of the preamble, Always 1
  dccPreamble,                        // Sequence of 1 bits
  dccStartBitPacket,                  // Bit between the preamble and first dccData byte. Always 0
  dccStartBitData,                    // Bit between dccData bytes. Always 0
  dccData,                            // Eight data bits. Each bit is a 0 or 1
  dccEndBit                           // Last bit of the DCC packet. Always 1
} DCC_output_state_t;

// Define the structure for all variables used by the ISR
struct DCC_ISR_State {
  volatile DCC_output_state_t state = dccPreamble;         // Initial state of the ISR state machine
  volatile uint8_t bitsRemaining = PREAMBLE_LENGTH;        // Bits remaining in current phase (preamble or data byte)
  volatile uint8_t bytesRemainingInPacket = 0;             // How many bytes left to send
  volatile uint8_t currentPacket[6] = {0, 0, 0, 0, 0, 0};  // Current packet being sent
  volatile uint8_t currentPacketSize = 0;                  // Number of bytes in current packet
  volatile bool firstHalfBit = true;	                     // The ISR starts with the first DCC Halfbit
  volatile bool firstHalfBitRC = true;	                   // And the first part of the Railcom start bit
  volatile bool smEnabled = false;                         // SM active (long preamble, resets, SM flow allowed)
  volatile uint8_t smRepeatsRemaining = 0;                 // Number of retransmissions left for this SM packet
  volatile uint8_t smMaxRepeats = 10;	                     // Number of SM retransmissions after first send
  volatile bool railComGapFlag = false;	                   // In the gap, this flag is guaranteed to be true
  volatile bool railCom = true;                            // Should we generate a RailCom gap?
  volatile bool railPower = false;	                       // Controls if the railsignal is on/off
};

static DCC_ISR_State dccISR;                              // Create an instance of the struct for the ISR


//******************************************************************************************************
// Hardware specific functions
//******************************************************************************************************
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
#define DCC_TIMER_INTERRUPT        TCB1_INT_vect


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
// Internal support routines
//******************************************************************************
static inline void startServiceModeRepeats() {
  // Called once when a new SM packet is accepted.
  // Sets the repeat counter for this packet.
  dccISR.smRepeatsRemaining = dccISR.smMaxRepeats;
}

static inline void advanceServiceModeRepeat() {
  // Called when we resend the same SM packet again.
  // When this reaches 0, a new packet may be accepted.
  if (dccISR.smRepeatsRemaining > 0)
    dccISR.smRepeatsRemaining--;
}

static inline bool isServiceModeAckPacket(volatile uint8_t *packet) {
  // RCN-216: Service Mode packets have 0111xxxx as first byte
  return (packet[0] & 0xF0) == 0b01110000;
}


//******************************************************************************
//**********************************  ISR   ************************************
//******************************************************************************
// This is the Interrupt Service Routine (ISR) for Timer compare match.
ISR(DCC_TIMER_INTERRUPT) {
  uint8_t mask;                  // helper to determine which pin to toggle
  TCB1.INTFLAGS = TCB_CAPT_bm;   // Clear the interrupt flag
  //
  // The ISR is entered everytime the timer expires.
  // Thus every 100us (0-bit), 58us (1-bit or 29us (RailCom start bit)
  //
  // ===========================================================================
  // PART 1: FIRST HALFBIT: TOGGLE PINS AND CHANGE TIMERS (IF NEEDED)
  // ===========================================================================
  if (dccISR.firstHalfBit) {
    // The H-bridge must never have both sides active at the same time, as this
    // would result in a short circuit and destroy the H-bridge.
    // Therefore, the normal and inverted rail pins must never be high
    // simultaneously. At the start of the first half-bit, the normal rail pin
    // is low while the inverted rail pin is high. For this reason, during the
    // first half-bit, the inverted rail pin must be toggled first, followed
    // by the normal rail pin.
    dccOutInvTgl();                // Inverted DCC signal
    dccOutTgl();                   // Normal DCC signal
    dccMonTgl();                   // Monitor signal

    switch (dccISR.state) {

      case dccRailComStartBit:
        if (dccISR.firstHalfBitRC) {        // Is this the first part of the RailCom halfbit?
          TMR_RAILCOM_STARTBIT();           // Set the timer, to trigger after 29us
          dccRailInv.enable = false;        // The Inverse pin will not toggle during the RC gap
          dccMonitor.enable = false;        // The Monitor pin will skip the toggle after 29us
          dccISR.railComGapFlag = true;     // The RailCom gap just started
          // Next ISR (in 29us from now) we must handle the second part of the RailCom start bit.
          dccISR.firstHalfBitRC = false;
          // Since normal DCC halfbits are either 58us or 100us, we must stay in the first DCC halfbit.
          // However, at the end of the ISR there there will always be a change of DCC halfbits.
          // To avoid an extra if statement at the end of the ISR, we do "a trick" here, and
          // also a change from the first DCC halfbit to the second.
          // Changing twice means no change at all.
          dccISR.firstHalfBit = !dccISR.firstHalfBit;
        }
        else {                              // First halfbit, but second part of the RC startbit
          dccRail.enable = false;           // The normal pin will not toggle during the RC gap
          dccMonitor.enable = true;         // The Monitor pin will continue to toggle at ISRs
          dccISR.firstHalfBitRC = true;     // Next time back to the first part of the RC startbit
        }
      break;

      case dccRailComGap:
        // Nothing to do
      break;

      case dccPreambleStartBit:
        if (dccISR.railPower)               // Only if their there should be power on the rails
          dccRailInv.enable = true;         // We restart the inverted signal
        dccISR.railComGapFlag = false;      // Only now we are certain that the gap is over
      break;

      case dccPreamble:
        // Nothing to doe
      break;

      // About to send a data byte, but have to preceed the data with a '0' (start bit).
      // Send that '0', then move to dccData
      case dccStartBitPacket:
        TMR_ZERO();
        // Check if we have received a next packet to send?
        if (dccPacketEngine.isWaiting) {	// We didn't get the next packet until now!
          if (dccPacketEngine.isServiceModeEnabled()) {
            //load a default reset packet!
            dccISR.currentPacket[0] = 0;
            dccISR.currentPacket[1] = 0;
            dccISR.currentPacket[2] = 0;
            dccISR.currentPacketSize = 3;  //feed to the starting ISR.
          }
          else {
            //load a default idle packet!
            dccISR.currentPacket[0] = 0xFF;
            dccISR.currentPacket[1] = 0;
            dccISR.currentPacket[2] = 0xFF;
            dccISR.currentPacketSize = 3;  //feed to the starting ISR.
          }
        }
        else {
          // We received a new packet.  Can we use it, or are we still retransmitting SM packets?
          if (!dccPacketEngine.isServiceModeRepeating()) {
            // Yes, we can copy
            dccISR.currentPacket[0] = dccPacketEngine.data[0];
            dccISR.currentPacket[1] = dccPacketEngine.data[1];
            dccISR.currentPacket[2] = dccPacketEngine.data[2];
            dccISR.currentPacket[3] = dccPacketEngine.data[3];
            dccISR.currentPacket[4] = dccPacketEngine.data[4];
            dccISR.currentPacket[5] = dccPacketEngine.data[5];
            dccISR.currentPacketSize = dccPacketEngine.dataSize;
            // Before we tell the user that we can accept the next packet, we first check
            // if this is a SM packet that must be retransmitted. Retransmissions are
            // needed to make SM-ACK responses more reliable. We therefore check if this is
            // a SM packet.
            if (isServiceModeAckPacket(dccISR.currentPacket)) {
             	// CV read/write packet with ACK response!
              startServiceModeRepeats();
            }
          }
          else  {
            // No, we couldn't copy, since we still need to retransmit SM packets.
            advanceServiceModeRepeat();
          }
        }
        if (!dccPacketEngine.isServiceModeRepeating())
          dccPacketEngine.isWaiting = true;  // we can accept the next packet
      break;

      case dccStartBitData:
        TMR_ZERO();             // The bit between Data bytes should always be 0
      break;

      case dccData:
        mask = 1 << (dccISR.bitsRemaining - 1);
        // Determine if we have to send a 0 or a 1
        if (dccISR.currentPacket[dccISR.currentPacketSize - dccISR.bytesRemainingInPacket] & mask)
          TMR_ONE()
          else TMR_ZERO();
      break;

      case dccEndBit:
        TMR_ONE();             // The DCC end bit should always be 1
      break;

    } // END SWITCH CASE
  }  // END firstHalfBit

  // ===========================================================================
  // PART 2: SECOND HALFBIT: UPDATE THE STATE MACHINE
  // ===========================================================================
  else {
    // The H-bridge must never have both sides active at the same time, as this
    // would result in a short circuit and destroy the H-bridge.
    // Therefore, the normal and inverted rail pins must never be high
    // simultaneously. At the start of the second half-bit, the normal rail pin
    // is high while the inverted rail pin is low. For this reason, during the
    // first half-bit, the normal rail pin must be toggled first, followed
    // by the inverted rail pin.
    dccOutTgl();                   // Normal DCC signal
    dccOutInvTgl();                // Inverted DCC signal
    dccMonTgl();                   // Monitor signal

    switch (dccISR.state) {

      case dccRailComStartBit:
        TMR_ONE();
        dccISR.bitsRemaining--;
        dccISR.state = dccRailComGap;
      break;

      case dccRailComGap:
        dccISR.bitsRemaining--;
        if (dccISR.bitsRemaining == 0) {      // Last half of last RailCom Gap bit
          if (dccISR.railPower)               // Only if the rails may be powered
            dccRail.enable = true;            // The normal pin is allowed to toggle again
          dccISR.bitsRemaining = PREAMBLE_LENGTH - RAILCOM_CUTOUT_LENGTH;
          dccISR.state = dccPreambleStartBit;
        }
      break;

      case dccPreambleStartBit:
        dccISR.bitsRemaining--;
        dccISR.state = dccPreamble;
      break;

      case dccPreamble:
        dccISR.bitsRemaining--;
        if (dccISR.bitsRemaining == 0) dccISR.state = dccStartBitPacket;
      break;

      case dccStartBitPacket:
        dccISR.state = dccData;
        dccISR.bitsRemaining = 8;	                              	// Reset the counter for bit sending
        dccISR.bytesRemainingInPacket	= dccISR.currentPacketSize;	// Reset the counter to the packet_size
      break;

      case dccStartBitData:
        dccISR.state = dccData;
        dccISR.bitsRemaining = 8;	                 // Reset the counter for bit sending
      break;

      case dccData:
        dccISR.bitsRemaining--;                    // One bit less to go for this byte
        if(dccISR.bitsRemaining == 0) {            // No more bits for this byte?
          dccISR.bytesRemainingInPacket--;         // One byte less for this packet
          if(dccISR.bytesRemainingInPacket == 0)   // No more bytes to go for this packet?
            dccISR.state = dccEndBit;              // Ready for this packet
          else dccISR.state = dccStartBitData;     // This packet has still bytes left
        };
      break;

      case dccEndBit:
        if (!dccPacketEngine.isServiceModeEnabled()) {
          if (dccISR.railCom) {                    // We must generate a RailCom gap
            dccISR.state = dccRailComStartBit;
            dccISR.bitsRemaining = RAILCOM_CUTOUT_LENGTH;
          }
          else {
            if (dccISR.railPower)                   // Only if the rails may be powered
              dccRail.enable = true;                // The normal pin is allowed to toggle again
            dccISR.state = dccPreambleStartBit;
            dccISR.bitsRemaining = PREAMBLE_LENGTH; // Normal preamble (RCN 211)
            }
          }
        else {
          dccISR.state = dccPreamble;
          dccISR.bitsRemaining = PREAMBLE_LENGTH_SM; // Long preamble (RCN 216)
        }
      break;

    } // End switch
  } // End second half bit

  // Next ISR we will take the other halfbit
  dccISR.firstHalfBit = !dccISR.firstHalfBit;

}; // End ISR


//******************************************************************************
// Service Mode interface methods
//******************************************************************************
void DccPacketEngine::enterServiceMode(void) {
  // Enable Service Mode behaviour (long preamble, reset packets, SM flow).
  dccISR.smEnabled = true;
}

void DccPacketEngine::leaveServiceMode(void) {
  // Leave Service Mode immediately. Ongoing SM retransmissions are cancelled.
  dccISR.smEnabled = false;
  dccISR.smRepeatsRemaining = 0;
}

bool DccPacketEngine::isServiceModeEnabled(void) {
  // Returns true if the system is currently in Service Mode.
  return dccISR.smEnabled;
}

void DccPacketEngine::setServiceModeMaxRepeats(uint8_t value) {
  dccISR.smMaxRepeats = value;
  // TODO: ALLEEN VOOR TESTEN
  dccISR.smMaxRepeats = 15;
}

bool DccPacketEngine::isFirstServiceModePacket(void) {
  // True only for the first transmission of a new SM packet.
  return dccISR.smEnabled && (dccISR.smRepeatsRemaining == dccISR.smMaxRepeats);
}

bool DccPacketEngine::isServiceModeRepeating(void) {
  // True while the ISR is busy retransmitting the current SM packet.
  return dccISR.smEnabled && (dccISR.smRepeatsRemaining > 0);
}

void DccPacketEngine::stopServiceModeRepeats(void) {
  // Stop retransmission of the current SM packet (e.g. after ACK).
  dccISR.smRepeatsRemaining = 0;
}


//******************************************************************************
// RailCom interface methods
//******************************************************************************
void DccPacketEngine::setRailCom(bool active) { // Enable / disable generation of the RailCom gap
  dccISR.railCom = active;
}

bool DccPacketEngine::getRailCom(void) { // Is generation of the RailCom gap enabled?
  return dccISR.railCom;
}

// This routine allows higher-layer software to determine whether a RailCom gap
// is currently in progress, so that rail voltage and/or current measurements
// can be safely controlled.
// To avoid measurement errors, such measurements must not be performed during
// a RailCom gap. Therefore, a conservative signaling approach is used.
// The start of the gap is indicated immediately when the RailCom start bit begins,
// while the end of the gap is indicated slightly after the actual gap.
// This is implemented using the boolean flag 'railComGapFlag', which is set
// at the start of the RailCom start bit and cleared at the first bit of the
// subsequent preamble.
bool DccPacketEngine::railComGap(void) {
  return dccISR.railComGapFlag;
}


//******************************************************************************
// Start / Stop
//******************************************************************************
void DccPacketEngine::StopOutputSignal() {
  // This routine may be called when a short circuit is detected or when the user
  // presses the emergency stop button. In that case the normal and inverted DCC
  // outputs must stop toggling and be forced LOW, so no DCC power is present on
  // the rails. The monitor output is not disabled, allowing switches and other
  // accessories to remain operational.
  noInterrupts();
  dccOutLow();
  dccOutInvLow();
  dccRail.enable = false;
  dccRailInv.enable = false;
	dccISR.railPower = false;
  interrupts();
}

void DccPacketEngine::RunOutputSignal() {
  // Enabling power is delicate, because we must ensure that the normal and
  // inverted DCC signals always remain complementary and are never high at the
  // same time. For this reason, toggling is enabled at the packet end bit:
  // during the first half-bit the inverted signal is allowed to toggle again,
  // and during the second half-bit the normal signal is re-enabled.
	dccISR.railPower = true;
}
