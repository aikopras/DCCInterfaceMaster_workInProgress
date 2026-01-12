/*
* DCC Waveform Generator
*
* modified by Philipp Gahtow
* Copyright digitalmoba@arcor.de, http://pgahtow.de
*
*/


#include "Arduino.h"

#if defined(__AVR__)
#include <avr/io.h>
#include <avr/interrupt.h>
#endif

#include "../DCCHardware.h"
#include "../DDCHardware_config.h"


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
// - dccIdle: there is nothing to put on the rails. In this case, the only legal thing
//   to do is to put a '1' on the rails.  The ISR should almost never be in this state.
// - dccSendPreamble: A packet has been made available, and so we should broadcast the
//   preamble with at least: 12 '1's in a row
// - dccSendLongpreamble: Additional '1's for Service Mode packets
// - dccSendStartBit: Each data byte is preceded by a starbit ('0').
//   It may be a Packet-Startbit or a Datenbyte-Startbit
// - dccSendDataByte: Sending the current data byte
// - dccPrepareNextDataByte
// - dccEndBit: After the final data byte  is sent, send a '0'.
typedef enum  {
  dccIdle,
  dccSendPreamble,
  dccSendStartBit,
  dccPrepareNextDataByte,
  dccSendDataByte,
  dccEndBit
} DCC_output_state_t;


// Define the structure for all variables used by the ISR
struct DCC_ISR_State {
  volatile DCC_output_state_t state = dccIdle;            // Current state of the ISR state machine
  volatile uint8_t bitsRemaining = PREAMBLE_LENGTH;       // Bits remaining in current phase (preamble or data byte)
  volatile uint8_t bytesRemainingInPacket = 0;            // How many bytes left to send
  volatile uint8_t currentPacket[6] = {0, 0, 0, 0, 0, 0}; // Current packet being sent
  volatile uint8_t currentPacketSize = 0;                 // Number of bytes in current packet
  volatile bool railComGapActive = false;                 // Are we within a Railcom gap?
  volatile bool railComStartBit = false;		              // Is the current pulse for a RailCom start bit?
  volatile bool firstHalfBit = false;	                    // Must be initialised as false
};

static DCC_ISR_State dccISR;                              // Create a global instance of the struct for the ISR


//******************************************************************************************************
#if defined(__AVR__)
//data for DCC direct register setting
uint8_t d1bit;		//_BV Bit for DCC
uint8_t d2bit;		//_BV Bit for DCC2
uint8_t d3bit;		//_BV Bit for S88
volatile uint8_t *d1reg, *d2reg, *d3reg;	//PORT Register
#endif

#if defined(ESP32)
static hw_timer_t * timer = NULL;
#endif


//******************************************************************************************************
// DCC inline output functions
//******************************************************************************************************
#if defined(__AVR__)
// --- Direct register access for AVR ---

inline void dccOutLow()        { *d1reg &= ~d1bit; }   // RailPin LOW
inline void dccOutHigh()       { *d1reg |=  d1bit; }   // RailPin HIGH
inline void dccOutInvLow()     { *d2reg &= ~d2bit; }   // RailInvPin LOW
inline void dccOutInvHigh()    { *d2reg |=  d2bit; }   // RailInvPin HIGH
inline void dccMonLow()        { *d3reg &= ~d3bit; }   // MonitorPin LOW
inline void dccMonHigh()       { *d3reg |=  d3bit; }   // MonitorPin HIGH

#define DCC_USES_TIMER1	//USE 16-bit TIMER1
//#define DCC_USES_TIMER2		//USE 8-bit TIMER2

#else
// --- DigitalWrite for other platforms (ESP8266/ESP32, etc.) ---
inline void dccOutLow()        { digitalWrite(dccPacketEngine.dccRailPin, LOW); }
inline void dccOutHigh()       { digitalWrite(dccPacketEngine.dccRailPin, HIGH); }
inline void dccOutInvLow()     { digitalWrite(dccPacketEngine.dccRailInvPin, LOW); }
inline void dccOutInvHigh()    { digitalWrite(dccPacketEngine.dccRailInvPin, HIGH); }
inline void dccMonLow()        { digitalWrite(dccPacketEngine.dccMonitorPin, LOW); }
inline void dccMonHigh()       { digitalWrite(dccPacketEngine.dccMonitorPin, HIGH); }

#endif


//******************************************************************************
//**********************************  ISR   ************************************
//******************************************************************************
/// This is the Interrupt Service Routine (ISR) for Timer compare match.
#if defined(__AVR__)
ISR(DCC_TMR_SIGNAL)
#elif defined (ESP8266) || defined (ESP32)
static void IRAM_ATTR onTimerISR()
#else
void DCC_ARM_TC_SIGNAL
#endif
{

#if defined(__SAM3X8E__)
  TC_GetStatus(DCC_ARM_TC_TIMER, DCC_ARM_TC_CHANNEL);
#endif

  // The ISR is entered everytime the timer expires.
  // Thus every 100us (0-bit), 58us (1-bit or 29us (RailCom start bit)

  // ===========================================================================
  // PART 1: SET THE OUTPUT PINS TO THE REQUESTED VALUES
  // ===========================================================================

  // Step 1.1: RailCom startbit
  // --------------------------
  // The RailCom startbit is a special case and will be tackled first
  // The RailCom startbit appears only on the normal DCC railpin, and not on the
  // inverted or monitor pins. No need to do anything else, so return immediately
  if (dccISR.railComStartBit) {
    // Once we are here, the output pin is high, and intended for a RailCom startbit.
    // The ISR was triggered, so at this moment the high output has lasted 29us.
    // The normal DCC railpin should therefore now be set to low.
    // In addition we (re)set the timer, to ensure that the next ISR will be again
    // after 29us. This ensures we keep the ISR synchronised to the normal 58us scheme
    // of the 1-bits for the preamble
    // Note: resetting the timer is not really needed, since the old timer value
    // was already at 29us
    dccOutLow();
    // TMR_SET_RAILCOM_STARTBIT();        // Not strictly needed,
    dccISR.railComStartBit = false;
    return;
  }

  // Step 1.2: Set the output pins to their required values
  // ------------------------------------------------------
  // This part is only needed for approaches in which the timer does not toggle
  // the ouput pin in hardware, as part of a compare / match
  dccISR.firstHalfBit = !dccISR.firstHalfBit;  // Let's do the next DCC half bit

  // Set the output for the DCC Monitor Pin
  // This pin has true DCC output, without a RailCom gap and independent of power status
  if (dccPacketEngine.dccMonitorPin != 0xFF) {
    if (!dccISR.firstHalfBit) dccMonLow();
    else dccMonHigh();
  }

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


//   digitalWrite(8,HIGH);digitalWrite(8,LOW);

  // ===========================================================================
  // PART 2A: SECOND HALFBIT: RAILCOM STARTBIT AND GAP
  // ===========================================================================

  if (dccISR.firstHalfBit == false) {
    // Step 2.1: In the second half bit, the DCC signal is low
    // Once we are here, the timer's value should be the same as we used
    // during the first half bit, except for the case where we have just
    // send a RailCom start bit. In such case, the next timer value must be 1
    if (last_timer == half_one_count) TMR_SET_ONE();

    // In addition, we check if we are in the fourth one-bit of a preamble
    // If that is the case, the RailCom gap will be over the next time the ISR is called
    if ((dccISR.state == dccSendPreamble) && dccISR.bitsRemaining == (PREAMBLE_LENGTH - RAILCOM_CUTOUT_LENGTH))
      dccISR.railComGapActive = false;
  }

  // ===========================================================================
  // PART 2B: FIRST HALFBIT: MAINTAIN THE STATE MACHINE AND SET THE TIMERS
  // ===========================================================================
  else {

    // We now have the first half bit, which means the pin is high.
    // We do 2 things:
    // 1) Determine the next timer value. In general this timer value need not
    // be changed for the second half bit, except in case of a RailCom startbit.
    // 2) Next to setting the timer, perform the other actions needed now.
    // 3) Determine if a state change is needed. Such state change takes
    // effect in the next ISR.

    // Note: the structure was changed to if/else because of an ESP32/ESP-IDF
    // compiler bug in switch/case #1330 - IRAM crash (Cache disabled but cached
    // memory region accessed). In future versions this structure may be changed
    // back to switch/case.

//        digitalWrite(8,HIGH);digitalWrite(8,LOW);


    // The Idle and preamble state are taken together
    if (dccISR.state == dccIdle || dccISR.state == dccSendPreamble) {
      if (dccISR.state == dccIdle) {
        // Check if this pulse should become a RailCom startbit.
        if ((dccPacketEngine.railCom) && (dccPacketEngine.serviceMode == 0)) { //in Service Mode no RailCom
          TMR_SET_RAILCOM_STARTBIT();             // set the timer, to trigger after 29us
          dccISR.railComGapActive = true;	        // start railcom cutout within the next circle
          dccISR.railComStartBit = true;		      // next Bit has only halve length
        }
        else TMR_SET_ONE();                       // set the timer to trigger for a normal preamble bit (59us)
        dccISR.state = dccSendPreamble;           // and fall through to dccSendPreamble
      }
      // Preamble: the number of preamble bits is not fixed, but differs between
      // normal and service mode. The idle phase, at the end of the state machine
      // sets this number. Here we just decrement, and once all preamble bits
      // have been transmitted, we move to the dccSendStartBit state.
      dccISR.bitsRemaining--;
      if (dccISR.bitsRemaining == 0) dccISR.state = dccSendStartBit;
    }

    // About to send a data byte, but have to preceed the data with a '0' (start bit).
    // Send that '0', then move to dccSendDataByte
    else if (dccISR.state == dccSendStartBit) {
        TMR_SET_ZERO();
        // Check if we have received a next packet to send?
        if (dccPacketEngine.isWaiting) {digitalWrite(9,HIGH);digitalWrite(9,LOW);}

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
        //break;
      }
      /// About to send next data byte, but have to peceed the data with a '0'. Send that '0', then move to dccSendDataByte
      else if (dccISR.state == dccPrepareNextDataByte) {
        TMR_SET_ZERO();
        dccISR.state = dccSendDataByte;	//continue sending...
        dccISR.bitsRemaining = 8;	//reset the counter for bit sending
        //break;
      }
      /// Sending a data byte; current bit is tracked with dccISR.bitsRemaining, and current data byte  with dccISR.bytesRemainingInPacket
      else if (dccISR.state == dccSendDataByte) {
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
        //break;
      }
      /// Done with the packet. Send out a final '1', then head back to dccIdle to check for a new packet.
      else if (dccISR.state == dccEndBit) {
        TMR_SET_ONE();
        dccISR.state = dccIdle;
        dccISR.bitsRemaining = PREAMBLE_LENGTH; //in preparation for a premable...
        if (dccPacketEngine.serviceMode > 0) { //long Preamble in Service Mode
          dccISR.bitsRemaining += ADD_LONG_PREAMBLE_LENGTH;	//additional '1's
        }
      }
    //} //END SWITCH CASE

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
/// Setup phase: configure and enable timer1 CTC interrupt, set OC1A and OC1B to toggle on CTC
void DccPacketEngine::setupWaveformGenerator() {

/*
 //Set the OC1A and OC1B pins (Timer1 output pins A and B) to output mode
 //On Arduino UNO, etc, OC1A is Port B/Pin 1 and OC1B Port B/Pin 2
 //On Arduino MEGA, etc, OC1A is or Port B/Pin 5 and OC1B Port B/Pin 6
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_AT90CAN128__) || defined(__AVR_AT90CAN64__) || defined(__AVR_AT90CAN32__)
  DDRB |= (1<<DDB5) | (1<<DDB6);
#else
  DDRB |= (1<<DDB1) | (1<<DDB2);
#endif

  // Configure timer1 in CTC mode, for waveform generation, set to toggle OC1A, OC1B, at /8 prescalar, interupt at CTC
  TCCR1A = (0<<COM1A1) | (1<<COM1A0) | (0<<COM1B1) | (1<<COM1B0) | (0<<WGM11) | (0<<WGM10);
  TCCR1B = (0<<ICNC1)  | (0<<ICES1)  | (0<<WGM13)  | (1<<WGM12)  | (0<<CS12)  | (1<<CS11) | (0<<CS10);

  //finally, force a toggle on OC1B so that pin OC1B will always complement pin OC1A
  TCCR1C |= (1<<FOC1B);

*/
	pinMode(dccRailPin, OUTPUT);		//Set output mode for DCC Pin

	/******************************************/
	#if defined(__AVR__)
	d1bit = digitalPinToBitMask(dccRailPin);
	d1reg = portOutputRegister(digitalPinToPort(dccRailPin));	//PORTB, PORTC, ....
	//DCC_OUTPUT1_OFF(); //LOW
  dccOutLow();

  pinMode(dccRailInvPin, OUTPUT);		//Set output mode for DCC Pin2
	d2bit = digitalPinToBitMask(dccRailInvPin);
	d2reg = portOutputRegister(digitalPinToPort(dccRailInvPin));
	//DCC_OUTPUT2_OFF(); //LOW
  dccOutInvLow();


	if (dccMonitorPin != 0xFF) {
		pinMode(dccMonitorPin, OUTPUT);		//Set output mode for DCC S88 Pin
		d3bit = digitalPinToBitMask(dccMonitorPin);
		d3reg = portOutputRegister(digitalPinToPort(dccMonitorPin));
		//*d3reg &= ~d3bit; //LOW
    dccMonLow();
	}

	DCC_INIT_COMPARATOR = 0;     // set entire TCCR1A register to 0
	DCC_TMR_CONTROL_REG = 0;     // same for TCCR1B

	// start by outputting a '1'
	TMR_SET_ONE(); //Whenever we set OCR1A, we must also set OCR1B, or else pin OC1B will get out of sync with OC1A!

	DCC_TMR_COUNT_REG = 0; //get the timer rolling (not really necessary? defaults to 0. Just in case.)

	DCC_TMR_CONTROL_SET();  //Timer Prescaler and mode set

	/******************************************/
	/******************************************/
	#else  //other

	digitalWrite(dccRailPin, DCC_OUTPUT1_OFF_legacy);

	pinMode(dccRailInvPin, OUTPUT);		//Set output mode for DCC Pin2
	digitalWrite(dccRailInvPin, DCC_OUTPUT2_OFF_legacy);

	if (dccMonitorPin != 0xFF) {
		pinMode(dccMonitorPin, OUTPUT);		//Set output mode for DCC S88 Pin
		digitalWrite(dccMonitorPin, LOW);
	}

	/******************************************/
	#if defined(ESP32)			//ESP32 Modul
	#if (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
	/* ESP Arduino core 3.x
	 */
	/* Use auto allocated timer of 4 */
	/* 1 tick takes 1us = 1MHz */
	timer = timerBegin(1000000); // ESP core 3.x, old ESP core 2.x was: DCC_ESP_TIMER_ID, DCC_ESP_TIMER_PRESCALE, DCC_ESP_TIMER_FLAG);
	#else
	/* ESP Arduino core 2.x
	 */
	/* Use 1st timer of 4 */
	/* 1 tick take 1/(80MHZ/80) = 1us so we set divider 80 and count up */
	timer = timerBegin(DCC_ESP_TIMER_ID, DCC_ESP_TIMER_PRESCALE, DCC_ESP_TIMER_FLAG);
	#endif

	/* Set alarm to call onTimer function every second 1 tick is 1us => 1 second is 1000000us */
	/* Repeat the alarm (third parameter) */
	// moved to section Enable the Interrupt (all MCUs): // timerAttachInterrupt(timer, &onTimerISR); // ESP core 3.x

	TMR_SET_ONE(); //start output "1"

	/******************************************/
	#elif defined (ESP8266)		//ESP8266
	noInterrupts();
	timer1_isr_init();
	timer1_enable(DCC_ESP_TIMER_DIV, DCC_ESP_TIMER_SET, DCC_ESP_TIMER_LOOP);

	TMR_SET_ONE(); //start output "1"
	interrupts();

	/******************************************/
	#else						//Arduino DUE
	// Tell the Power Management Controller to disable
	// the write protection of the (Timer/Counter) registers:
	pmc_set_writeprotect(false);

	// Enable clock for the timer
	pmc_enable_periph_clk((uint32_t)DCC_ARM_MATCH_INT);

	// Set up the Timer in waveform mode which creates a PWM
	// in UP mode with automatic trigger on RC Compare
	// and sets it up with the determined internal clock as clock input.
	TC_Configure(DCC_ARM_TC_TIMER, DCC_ARM_TC_CHANNEL, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2);

	// Set up timing...
	TMR_SET_ONE();

	// Enable the RC Compare Interrupt...
	DCC_ARM_TC_TIMER->TC_CHANNEL[DCC_ARM_TC_CHANNEL].TC_IER=TC_IER_CPCS;

	// ... and disable all others.
	DCC_ARM_TC_TIMER->TC_CHANNEL[DCC_ARM_TC_CHANNEL].TC_IDR=~TC_IER_CPCS;

	#endif

	/******************************************/
	#endif

	/******************************************/
	//Enable the Interrupt (all MCUs):

	#if defined(__AVR__)
	//enable match interrupt
	DCC_TMR_MATCH_INT();

	#elif defined(ESP8266) //ESP8266
	timer1_attachInterrupt(onTimerISR);

	#elif defined(ESP32)	//ESP32
	#if (ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0))
	// Code for version 3.x
	// nop
	#else
	/* ESP Arduino core 2.x
	 */
	timerAlarmEnable(timer);
	#endif

	timerAttachInterrupt(timer, &onTimerISR); // for ESP core 2.x and 3.x


	#else	//Arduino DUE
	NVIC_EnableIRQ(DCC_ARM_MATCH_INT);
	#endif
}
