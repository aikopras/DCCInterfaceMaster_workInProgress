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
//******************************************************************************************************
#include <Arduino.h>
#include "../DCCHardware.h"


#define PREAMBLE_LENGTH 17        	// see RCN-211 (Ausgabe 23.07.2023)
#define ADD_LONG_PREAMBLE_LENGTH 3	// see RCN-216 (Ausgabe 24.11.2025)
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
  powerStatus = false;                          	//set the railsignal on/off
}


//******************************************************************************************************
// 2. Declaration and initialisation of internal stuff
//******************************************************************************************************
// An enumerated type for keeping track of the state machine used in the ISR
// Given the structure of a DCC packet, the ISR can be in one of the following states:
// - dccPreambleStartBit: We start with the first bit of the preamble. We determine how
//   lang the preamble should be, and whether a RailCom Startbit is needed
// - dccPreambleContinue: We have send the first preamble (1) bit, and continue with
//   the next preamble bits, until all are done
// - dccSendStartBit: Each data byte is preceded by a starbit ('0').
//   It may be a Packet-Startbit or a Datenbyte-Startbit
// - dccSendDataByte: Sending the current data byte
// - dccPrepareNextDataByte
// - dccEndBit: After the final data byte  is sent, send a '0'.
typedef enum  {
  dccRailComStartBit,
  dccPreambleStartBit,
  dccPreambleContinue,
  dccSendStartBit,
  dccPrepareNextDataByte,
  dccSendDataByte,
  dccEndBit
} DCC_output_state_t;

// TODO currentPacket[6] = {0, 0, 0, 0, 0, 0};  meteen als idle packet initialiseren?

// Define the structure for all variables used by the ISR
struct DCC_ISR_State {
  volatile DCC_output_state_t state = dccPreambleStartBit; // Initial state of the ISR state machine
  volatile uint8_t bitsRemaining = PREAMBLE_LENGTH;        // Bits remaining in current phase (preamble or data byte)
  volatile uint8_t bytesRemainingInPacket = 0;             // How many bytes left to send
  volatile uint8_t currentPacket[6] = {0, 0, 0, 0, 0, 0};  // Current packet being sent
  volatile uint8_t currentPacketSize = 0;                  // Number of bytes in current packet
  volatile bool railComGapActive = false;                  // Are we within a Railcom gap?
  volatile bool railComStartBit = false;		               // Is the current pulse for a RailCom start bit?
  volatile bool firstHalfBit = false;	                     // Must be initialised as false
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
} DccOut_t;

static DccOut_t dccMonitor;   // Create an instance of the register struct for the dccMonitor
static DccOut_t dccRail;      // Create an instance of the register struct for the dccRail
static DccOut_t dccRailInv;   // Create an instance of the register struct for the dccRailInv

// As opposed to the original code for the Arduino UNO/MEGA, we perform an extra check
// if the register pointer is defined or NULL. Therefore the code works for 1, 2 or 3 pins
inline void dccMonLow()        { if (dccMonitor.outset)  *dccMonitor.outclr = dccMonitor.bit; }
inline void dccMonHigh()       { if (dccMonitor.outset)  *dccMonitor.outset = dccMonitor.bit; }
inline void dccOutLow()        { if (dccRail.outset)     *dccRail.outclr = dccRail.bit; }
inline void dccOutHigh()       { if (dccRail.outset)     *dccRail.outset = dccRail.bit; }
inline void dccOutInvLow()     { if (dccRailInv.outset)  *dccRailInv.outclr = dccRailInv.bit; }
inline void dccOutInvHigh()    { if (dccRailInv.outset)  *dccRailInv.outset = dccRailInv.bit; }

/* TCB clock = CLK_PER = F_CPU (typically 24 MHz) */
/* 1 tick = 1 / F_CPU seconds */
/* The timer counts are calculated from the selected clockspeed, and are therefore dynamic */
#define TICKS_PER_US               (F_CPU / 1000000UL)
#define half_one_count             (TICKS_PER_US * (29))
#define one_count                  (TICKS_PER_US * (58))
#define zero_count                 (TICKS_PER_US * (116))

/* We use TCB1 in periodic interrupt mode and reload CCMP each time */
#define TMR_SET_RAILCOM_STARTBIT() { TCB1.CCMP = half_one_count; }
#define TMR_SET_ONE()              { TCB1.CCMP = one_count; }
#define TMR_SET_ZERO()             { TCB1.CCMP = zero_count; }
#define DCC_TMR_SIGNAL             TCB1_INT_vect


// TODO: Wat hiermee doen?
#define last_timer  TCB1.CCMP

// digitalWrite(PIN_PB4, HIGH); digitalWrite(PIN_PB4, LOW);


//******************************************************************************
//**********************************  ISR   ************************************
//******************************************************************************
/// This is the Interrupt Service Routine (ISR) for Timer compare match.
ISR(DCC_TMR_SIGNAL) {
  TCB1.INTFLAGS = TCB_CAPT_bm;   // Clear the interrupt flag

  // The ISR is entered everytime the timer expires.
  // Thus every 100us (0-bit), 58us (1-bit or 29us (RailCom start bit)

  // ===========================================================================
  // PART 1: First handle the RailCom start bit
  // ===========================================================================
  // The RailCom startbit is a special case and will be tackled first.
  // The RailCom startbit should only apprear on the normal DCC railpin,
  // and not on the inverted or monitor pins. Once done, return immediately.
  if (dccISR.railComStartBit) {
    // During the first half bit, the "normal" rail output was high and should
    // become low. The timer was already 29us, so doesn't need to be reset
    // During the second half bit, the timer should be changed to a normal
    // 1 bit (for the preamble). The "normal" rail output was already low,
    // but the monitor output should also be made low.
    if (dccISR.firstHalfBit) {
      dccOutLow();
      dccISR.firstHalfBit = false;
    }
    else {
      TMR_SET_ONE();
      dccMonLow();                       // Needed, since we skip the ISR remainder
      dccISR.railComStartBit = false;    // Ready with the start bit
    }
    return;                              // Skip te remainder of the ISR
  }


  // ===========================================================================
  // PART 2: SET THE OUTPUT PINS TO THE REQUESTED VALUES
  // ===========================================================================
  // This part is only needed for approaches in which the timer does not toggle
  // the ouput pin in hardware, as part of a compare / match
  dccISR.firstHalfBit = !dccISR.firstHalfBit;  // Let's do the next DCC half bit

  // Set the output for the DCC Monitor Pin
  // This pin has true DCC output, without a RailCom gap and independent of power status
  if (!dccISR.firstHalfBit) dccMonLow();
    else dccMonHigh();

  // Set the output for the normal and inverse DCC Pins
  if (dccPacketEngine.powerStatus) {	       // Railpower ON?
    if (dccISR.railComGapActive) {	         // Are we within a Railcom CutOut?
      // Within the RailCom gap, both rail pins should be low, to ensure that any
      // connected H-Bridge / Booster doesn't put any signal on the line
      dccOutLow();      // RailPin LOW
      dccOutInvLow();   // RailInvPin LOW
    }
    else {
      // Normal operation = no RailCom Cutout
      if (dccISR.firstHalfBit == false) {     // is this the second halfbit?
        dccOutLow();
        dccOutInvHigh();
      }
      else {
        dccOutHigh();
        dccOutInvLow();
      }
    }
  }


  // ===========================================================================
  // PART 3A: SECOND HALFBIT: End of Preamble?
  // ===========================================================================

  if (dccISR.firstHalfBit == false) {
    // Step 3.1: In the second half bit, the DCC signal is low
    // Once we are here, the timer's value should be the same as we used
    // during the first half bit.
    // We check, however, if we are in the fourth one-bit of a preamble
    // If that is the case, the RailCom gap will be over the next time the ISR is called
    if ((dccISR.state == dccPreambleContinue) && dccISR.bitsRemaining == (PREAMBLE_LENGTH - RAILCOM_CUTOUT_LENGTH))
      dccISR.railComGapActive = false;
  }

  // ===========================================================================
  // PART 3B: FIRST HALFBIT: MAINTAIN THE STATE MACHINE AND SET THE TIMERS
  // ===========================================================================
  else {
    // We now have the first half bit, which means the pin is high.
    // We do 3 things:
    // 1) Determine the next timer value. In general this timer value need not
    // be only be changed for the first half bit, except in case of a RailCom startbit.
    // 2) Next to setting the timer, set some flags, if needed.
    // 3) Determine if a state change is needed. Such state change takes
    // effect in the next ISR.


    // The Idle and preamble state are taken together
    switch (dccISR.state) {

      case dccRailComStartBit:
      break;

      case dccPreambleStartBit:
        // This is the first pulse of the preamble
        // Check if this pulse should become a RailCom startbit.
        if ((dccPacketEngine.railCom) && (dccPacketEngine.serviceMode == 0)) { //in Service Mode no RailCom
            TMR_SET_RAILCOM_STARTBIT();             // set the timer, to trigger after 29us
            dccISR.railComGapActive = true;	        // start railcom cutout within the next circle
            dccISR.railComStartBit = true;		      // next Bit has only halve length
          }
          else TMR_SET_ONE();                       // set the timer to trigger for a normal preamble bit (59us)
        // Set the number of preamble bits. This number is not fixed, but
        // differs between normal and service mode.
        dccISR.bitsRemaining = PREAMBLE_LENGTH - 1;   // -1, since this is already the first
        if (dccPacketEngine.serviceMode > 0)
          dccISR.bitsRemaining += ADD_LONG_PREAMBLE_LENGTH;
        dccISR.state = dccPreambleContinue;
        digitalWrite(PIN_PB4, HIGH); digitalWrite(PIN_PB4, LOW);
      break;

      case dccPreambleContinue:
        // Decrement, and once all preamble bits have been transmitted,
        // we move to the dccSendStartBit state.
        dccISR.bitsRemaining--;
        if (dccISR.bitsRemaining == 0) dccISR.state = dccSendStartBit;
      break;

      // About to send a data byte, but have to preceed the data with a '0' (start bit).
      // Send that '0', then move to dccSendDataByte
      case dccSendStartBit:
        TMR_SET_ZERO();
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

        dccISR.state = dccSendDataByte;
        dccISR.bitsRemaining = 8;	                              	//reset the counter for bit sending
        dccISR.bytesRemainingInPacket	= dccISR.currentPacketSize;	//reset the counter to the packet_size
      break;

      /// About to send next data byte, but have to peceed the data with a '0'. Send that '0', then move to dccSendDataByte
      case dccPrepareNextDataByte:
        TMR_SET_ZERO();
        dccISR.state = dccSendDataByte;	//continue sending...
        dccISR.bitsRemaining = 8;	//reset the counter for bit sending
      break;

      /// Sending a data byte; current bit is tracked with dccISR.bitsRemaining, and current data byte  with dccISR.bytesRemainingInPacket
      case dccSendDataByte:
        if(((dccISR.currentPacket[dccISR.currentPacketSize-dccISR.bytesRemainingInPacket])>>(dccISR.bitsRemaining-1)) & 1) //is current bit a '1'?
        {
          TMR_SET_ONE();
        }
        else //or is it a '0'
        {
          TMR_SET_ZERO();
        }
        if(!--dccISR.bitsRemaining) //out of bits! time to either send a new data byte , or end the packet
        {
          dccISR.bytesRemainingInPacket--;
          if(dccISR.bytesRemainingInPacket == 0) //if no more data byte s, move to dccEndBit
          {
            dccISR.state = dccEndBit;
          }
          else //there are more data byte s…so, go back to dccSendStartBit
          {
            dccISR.state = dccPrepareNextDataByte;
          }
        }
      break;

      /// Done with the packet. Send out a final '1', then head back to dccPreambleStartBit
      case dccEndBit:
        TMR_SET_ONE();
        dccISR.state = dccPreambleStartBit;
        digitalWrite(PIN_PB3, HIGH); digitalWrite(PIN_PB3, LOW);
      break;

    } //END SWITCH CASE

  }  //END the pin is high.

}


//******************************************************************************
// RailCom helper routines
//******************************************************************************
bool DccPacketEngine::railComGap(void) {return dccISR.railComGapActive;}

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
  }
  else {
    pinObj.outset = nullptr;
    pinObj.outclr = nullptr;
    pinObj.outtgl = nullptr;
    pinObj.bit = 0;
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
  // Set the three pins low
  dccOutLow();
  dccOutInvLow();
  dccMonLow();
  // DCC_USES_TCB1 - DxCore
  TCB1.CTRLB = TCB_CNTMODE_INT_gc;      // periodic interrupt mode
  TCB1.CCMP  = one_count;               // start with "1" timing
  TCB1.INTFLAGS = TCB_CAPT_bm;          // clear pending interrupt flag
  TCB1.INTCTRL  = TCB_CAPT_bm;          // enable capture/timeout interrupt
  TCB1.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;   // CLK_PER, enable
  // Start with a DCC low-signal
  TMR_SET_ZERO();
}
