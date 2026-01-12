#pragma once
#include <Arduino.h>

#define MaxDccSize 6                              // DCC messages can have a length upto this value

// The interface between the outside wrld and DCCHardware
class DccPacketEngine {
  public:

    // DCC output pins
    uint8_t dccRailPin;   	                      // Pin for DCC rail Signal, with RailCom support
    uint8_t dccRailInvPin;	                      // Pin for DCC rail Signal, with RailCom support. Inverted
    uint8_t dccMonitorPin;                        // Pin for a continuous DCC Signal, ignoring the RailCom gap / Railpower

    // DCC message specifics
    volatile uint8_t data[MaxDccSize];            // The contents of the last dcc message received
    volatile uint8_t dataSize;                    // 3 .. 6, including XOR
    volatile uint8_t isWaiting;                   // Flag that next DCC message can be offered to the ISR
    volatile uint8_t serviceMode;	                // Used in case the packet is a service mode packet
                                                  // 0 = Normal packet
                                                  // 255 = Service Mode packet
                                                  // < 255 = Service Mode Repeat counter
    volatile uint8_t serviceModeRepeat;	          // How often will Service Mode Packet be repeated?
                                                  // The schedular may WRITE this number, using CV value from EEPROM

    // RailCom specific

    bool railComGap(void);                        // Is the DCCHarware currently generating a Railcom gap?

    volatile bool railCom;	                      // Should we provide a cut out of four bit in the preamble
                                                  // The schedular may READ this flag
    // Init, start and stop
    void StopOutputSignal(void);                  // Level of DCC rail output pins become LOW (no impact on dccMonitorPin)
    void RunOutputSignal(void);                   // Level of DCC rail output pins become HIGH (no impact on dccMonitorPin)
    void setupWaveformGenerator();                // Setup and start the waveform generator (ISR)

    volatile bool powerStatus;	                  // Controls if the railsignal is on/off

    DccPacketEngine();                            // Constructor declaration

  private:

};

extern DccPacketEngine dccPacketEngine;           // The dccPacketEngine must be accessible externally
