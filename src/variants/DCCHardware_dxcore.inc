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
  serviceMode = 0;                                // This is not a Servoce Mode packet
  serviceModeRepeat = 10;	                        // Initial value for Service Mode Packets repeats
  railCom = false;	                              // Should we have a Railcom cut-out in the preamble?
  powerStatus = false;                          	// Set the railsignal on/off
}


//******************************************************************************************************
// 2. Declaration and initialisation of internal stuff
//******************************************************************************************************
// An enumerated type for keeping track of the state machine used in the ISR
// Given the structure of a DCC packet, the ISR can be in one of the following states:
// - dccPreamble: We send the preamble bits one by one, until all are done
// - dccStartBitPacket: Each data byte is preceded by a starbit ('0').
//   It may be a Packet-Startbit or a Datenbyte-Startbit
// - dccData: Sending the current data byte
// - dccStartBitData
// - dccEndBit: After the final data byte  is sent, send a '0'.
typedef enum  {
  dccRailComStartBit,
  dccRailComGap,
  dccPreamble,
  dccStartBitPacket,
  dccStartBitData,
  dccData,
  dccEndBit
} DCC_output_state_t;

// TODO currentPacket[6] = {0, 0, 0, 0, 0, 0};  meteen als idle packet initialiseren?
// TODO: Retransmission IDLEs in SM in 1 packet doen

// Define the structure for all variables used by the ISR
struct DCC_ISR_State {
  volatile DCC_output_state_t state = dccPreamble;         // Initial state of the ISR state machine
  volatile uint8_t bitsRemaining = PREAMBLE_LENGTH;        // Bits remaining in current phase (preamble or data byte)
  volatile uint8_t bytesRemainingInPacket = 0;             // How many bytes left to send
  volatile uint8_t currentPacket[6] = {0, 0, 0, 0, 0, 0};  // Current packet being sent
  volatile uint8_t currentPacketSize = 0;                  // Number of bytes in current packet
  volatile bool firstHalfBit = true;	                     // The ISR starts with the first DCC Halfbit
  volatile bool firstHalfBitRC = true;	                   // And the first part of the Railcom start bit
};

static DCC_ISR_State dccISR;                              // Create an instance of the struct for the ISR


//******************************************************************************************************
// DCC inline output functions
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

/* TCB clock = CLK_PER = F_CPU (typically 24 MHz) */
/* 1 tick = 1 / F_CPU seconds */
/* The timer counts are calculated from the selected clockspeed, and are therefore dynamic */
#define TICKS_PER_US               (F_CPU / 1000000UL)
#define half_one_count             (TICKS_PER_US * (29))
#define one_count                  (TICKS_PER_US * (58))
#define zero_count                 (TICKS_PER_US * (116))

/* We use TCB1 in periodic interrupt mode and reload CCMP each time */
#define TMR_RAILCOM_STARTBIT()     { TCB1.CCMP = half_one_count; }
#define TMR_ONE()                  { TCB1.CCMP = one_count; }
#define TMR_ZERO()                 { TCB1.CCMP = zero_count; }
#define DCC_TIMER_INTERRUPT        TCB1_INT_vect


#define DEBUG1 digitalWrite(PIN_PB3, HIGH); digitalWrite(PIN_PB3, LOW);
#define DEBUG2 digitalWrite(PIN_PB4, HIGH); digitalWrite(PIN_PB4, LOW);
#define DEBUG3 digitalWrite(PIN_PB5, HIGH); digitalWrite(PIN_PB5, LOW);


//******************************************************************************
//**********************************  ISR   ************************************
//******************************************************************************
// This is the Interrupt Service Routine (ISR) for Timer compare match.
ISR(DCC_TIMER_INTERRUPT) {
  TCB1.INTFLAGS = TCB_CAPT_bm;   // Clear the interrupt flag
  // The ISR is entered everytime the timer expires.
  // Thus every 100us (0-bit), 58us (1-bit or 29us (RailCom start bit)
  //
  // PART 1: TOGGLE THE PINS
  // ===========================================================================
  // This part is only needed for cases where we want to perform bit-banging
  uint8_t mask;                  // helper to determine the pin to toggle
  dccOutTgl();                   // Normal DCC signal
  dccOutInvTgl();                // Inverted DCC signal
  dccMonTgl();                   // Monitor signal

  // ===========================================================================
  // PART 2A: FIRST HALFBIT: CHANGE TIMERS (IF NEEDED)
  // ===========================================================================
  if (dccISR.firstHalfBit) {
    // We now handle the first halfbit, which means the pin is high.
    // During the first halfbit we may change the next timer value.
    // Such timer changes are not needed during the second half bit.
    // A special case is the RailCom startbit, which breaks the pattern that
    // halfbits are 59us or 100us

    switch (dccISR.state) {

      case dccRailComStartBit:
        if (dccISR.firstHalfBitRC) {        // Is this the first part of the RailCom halfbit?
          TMR_RAILCOM_STARTBIT();           // Set the timer, to trigger after 29us
          dccRailInv.enable = false;        // The Inverse pin will not toggle during the RC gap
          dccMonitor.enable = false;        // The Monitor pin will skip the toggle after 29us
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

      case dccPreamble:
        dccRailInv.enable = true;              // Only needed for the first preamble bit
      break;

      // About to send a data byte, but have to preceed the data with a '0' (start bit).
      // Send that '0', then move to dccData
      case dccStartBitPacket:
        TMR_ZERO();
        // TODO: probably move this to another state?
        // Check if we have received a next packet to send?
        if (dccPacketEngine.isWaiting) {	//ERROR! - We didn't get the next packet until now!
          if (dccPacketEngine.serviceMode > 0) {
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
          if ((dccPacketEngine.serviceMode == 0xFF) || (dccPacketEngine.serviceMode == 0)) {
            // Yes, we can copy
            dccISR.currentPacket[0] = dccPacketEngine.data[0];
            dccISR.currentPacket[1] = dccPacketEngine.data[1];
            dccISR.currentPacket[2] = dccPacketEngine.data[2];
            dccISR.currentPacket[3] = dccPacketEngine.data[3];
            dccISR.currentPacket[4] = dccPacketEngine.data[4];
            dccISR.currentPacket[5] = dccPacketEngine.data[5];
            dccISR.currentPacketSize = dccPacketEngine.dataSize;
            // TODO: I assume the SM retransmission better moves to the packet scheduler.
            // In addition, I do not believe this code is correct
            // In SM, there MUST be 9at least) 3 IDLE packets between SM packets.
            // Before we tell the user that we can accept the next packet, we first check
            // if this is a SM packet that must be retransmitted. Retransmissions are
            // needed to make SM-ACK responses more reliable. We therefore check if this is
            // a SM packet, and if yes, set the serviceMode to 0xFF minus the number
            // of required retransmissions.
            if ((dccISR.currentPacket[0] >> 4) == 0b0111) 		// CV read/write packet with ACK response!
              dccPacketEngine.serviceMode = 0xFF - dccPacketEngine.serviceModeRepeat;
          }
          // No, we couldn't copy, since we still need to retransmit SM packets.
          else dccPacketEngine.serviceMode++;	//count of internal Service Mode Packet repeat
        }
        if ((dccPacketEngine.serviceMode == 0) || (dccPacketEngine.serviceMode == 0xFF))
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
  // PART 2B: SECOND HALFBIT: UPDATE THE STATE MACHINE
  // ===========================================================================
  else {
    // In the second half bit we decide if we have to change the DCC state.
    // Here the DCC signal is low, and the timer value doesn't need to be changed.
    // An exception, however, is the RailCom startbit.

    switch (dccISR.state) {

      case dccRailComStartBit:
        TMR_ONE();
        dccISR.bitsRemaining = RAILCOM_CUTOUT_LENGTH - 1;
        dccISR.state = dccRailComGap;
      break;

      case dccRailComGap:
        dccISR.bitsRemaining--;
        if (dccISR.bitsRemaining == 0) {
          dccISR.bitsRemaining = PREAMBLE_LENGTH - RAILCOM_CUTOUT_LENGTH;
          dccISR.state = dccPreamble;
          dccRail.enable = true;              // Allow the normal pin to toggle
        }
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
        if (dccPacketEngine.serviceMode == 0) {       // Normal mode
          if (dccPacketEngine.railCom) {
            dccISR.state = dccRailComStartBit;
            dccISR.bitsRemaining = RAILCOM_CUTOUT_LENGTH;
          }
          else {
            dccISR.state = dccPreamble;
            dccISR.bitsRemaining = PREAMBLE_LENGTH;  // Normal preamble (RCN 211)
            }
          }
        else {
          dccISR.state = dccPreamble;
          dccISR.bitsRemaining = PREAMBLE_LENGTH_SM;  // Long preamble (RCN 216)
        }
      break;

    } // End switch
  } // End second half bit

  // Next ISR we will take the other halfbit
  dccISR.firstHalfBit = !dccISR.firstHalfBit;

}; // End ISR


//******************************************************************************
// RailCom helper routines
//******************************************************************************
bool DccPacketEngine::railComGap(void) {
  return ((dccISR.state == dccRailComStartBit) || (dccISR.state == dccRailComGap));
}

//******************************************************************************
// Start / Stop
//******************************************************************************
void DccPacketEngine::StopOutputSignal() {
	powerStatus = false;
  // Both DCC output pins must be made inactive / low
	digitalWrite(dccRailPin, LOW);
	if (dccRailInvPin != 0xFF) digitalWrite(dccRailInvPin, LOW);
}

void DccPacketEngine::RunOutputSignal() {
	powerStatus = true;
}


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
