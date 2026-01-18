//******************************************************************************************************
//
// File:            DCCHardware_common.inc
// Purpose:         DCC Waveform Generator - Processor independent include file
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
// This file contains the processor-independent core of the DCC waveform generator.
// It implements the logical DCC protocol flow as an interrupt-driven state machine,
// but makes no assumptions about timers, registers, ports, interrupt vectors, or
// the mechanism used to toggle the rail outputs. All hardware-specific aspects
// (timer configuration, ISR binding, interrupt prologue, and physical pin control)
// are supplied by the including driver file.
//
// The state machine models the complete DCC bitstream, including preamble generation,
// start, data and end bits, Service Mode behaviour and RailCom cutouts. The actual rail
// signals may be generated either by software-controlled pin toggling (bit-banging)
// or by hardware peripherals such as timer compare units.
//
// A fundamental design choice is the explicit distinction between the first and second
// halfbit of every DCC bit. During the first halfbit, output transitions are triggered
// (or delegated to hardware) and the timing for the next halfbit is configured. During
// the second halfbit, the timing remains unchanged and only the logical DCC state
// machine is advanced. This separation allows accurate bit timing, safe control of
// complementary rail outputs, and a clean split between physical signal generation
// and protocol flow.
//
// Service Mode packet retransmissions are handled inside the ISR rather than in the
// scheduler. This ensures that repeats are tightly coupled to the actual transmission
// flow and allows higher software layers to detect acknowledgements reliably, without
// being affected by scheduler timing, buffering, or packet queue latency.
//
// A detailed description of the state machine, its states, and their transitions can
// be found in: extras/Statemachine.
//
//******************************************************************************************************
//******************************************************************************************************
// 1. Defines
//******************************************************************************************************
#define PREAMBLE_LENGTH 17        	// see RCN-211 (Ausgabe 23.07.2023)
#define PREAMBLE_LENGTH_SM 20       // see RCN-216 (Ausgabe 24.11.2025)
#define RAILCOM_CUTOUT_LENGTH 4	  	// see RCN-217 (Ausgabe 24.11.2025)


//******************************************************************************************************
// 2. Declaration and initialisation (constructor) of external object
//******************************************************************************************************
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

// dccPacketEngine is the object that controls packet transmission for different hardware
DccPacketEngine dccPacketEngine;


//******************************************************************************************************
// 3. Declaration and initialisation of internal data structures
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


//******************************************************************************
// 4. Service Mode interface methods
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
// 5. RailCom interface methods
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
// 6. Start / Stop interface methods
//******************************************************************************
void DccPacketEngine::StopOutputSignal() {
  // This routine may be called when a short circuit is detected or when the user
  // presses the emergency stop button. In that case the normal and inverted DCC
  // outputs must stop toggling and be forced LOW, so no DCC power is present on
  // the rails. The monitor output is not disabled, allowing switches and other
  // accessories to remain operational.
  noInterrupts();
  DCC_OUT_LOW;
  DCC_OUT_INV_LOW;
  DISABLE_DCCOUT;
  DISABLE_DCCINV;
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


//******************************************************************************
// 7. Inline ISR support routines
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
// This is the Interrupt Service Routine (ISR).
// The ISR is entered everytime the timer expires.
// Thus every 100us (0-bit), 58us (1-bit or 29us (RailCom start bit)

ISR_START {
  digitalWriteFast(PIN_PD3,HIGH);

  ISR_PROLOGUE;                  // Processor specific code may be here
  uint8_t mask;                  // helper to determine which bit we are
  //
  // ===========================================================================
  // PART 1: FIRST HALFBIT: TOGGLE PINS AND CHANGE TIMERS (IF NEEDED)
  // ===========================================================================
  if (dccISR.firstHalfBit) {
    // At the start of the first half-bit, the normal rail output is low while
    // the inverted rail output is high. When pin transitions are generated
    // in software, the required state change is performed explicitly here
    // by toggling the outputs.
    // In the hardware-assisted approach (toggle-on-compare-match),
    // dccFirtsHalfBitToggle() is implemented as an empty inline function,
    // as the required transition is handled automatically by the timer.
    dccFirtsHalfBitToggle();

    switch (dccISR.state) {

      case dccRailComStartBit:
        if (dccISR.firstHalfBitRC) {        // Is this the first part of the RailCom halfbit?
          TMR_RAILCOM_STARTBIT();           // Set the timer, to trigger after 29us
          DISABLE_DCCINV;                   // The Inverse pin will not toggle during the RC gap
          DISABLE_DCCMON;                   // The Monitor pin will skip the toggle after 29us
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
          DISABLE_DCCOUT;                   // The normal pin will not toggle during the RC gap
          ENABLE_DCCMON;                    // The Monitor pin will continue to toggle at ISRs
          dccISR.firstHalfBitRC = true;     // Next time back to the first part of the RC startbit
        }
      break;

      case dccRailComGap:
        // Nothing to do
      break;

      case dccPreambleStartBit:
        if (dccISR.railPower)               // Only if their there should be power on the rails
          ENABLE_DCCINV;                    // We restart the inverted signal
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
    // At the start of the first half-bit, the normal rail output is high while
    // the inverted rail output is low. When pin transitions are generated
    // in software, the required state change is performed explicitly here
    // by toggling the outputs.
    // In the hardware-assisted approach (toggle-on-compare-match),
    // dccFirtsHalfBitToggle() is implemented as an empty inline function,
    // as the required transition is handled automatically by the timer.
    dccSecondHalfBitToggle();

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
            ENABLE_DCCOUT;                    // The normal pin is allowed to toggle again
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
              ENABLE_DCCOUT;                        // The normal pin is allowed to toggle again
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

  digitalWriteFast(PIN_PD3,LOW);

}; // End ISR
