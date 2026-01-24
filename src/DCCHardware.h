#pragma once
#include <Arduino.h>

 #define MaxDccSize 6                              // DCC messages can have a length upto this value

// The interface between the outside world and DCCHardware
class DccPacketEngine {
  public:

    // DCC output pins
    uint8_t dccRailPin;   	                      // Pin for DCC rail Signal, with RailCom support
    uint8_t dccRailInvPin;	                      // Pin for DCC rail Signal, with RailCom support. Inverted
    uint8_t dccMonitorPin;                        // Pin for a continuous DCC Signal, ignoring the RailCom gap / Railpower

    // DCC message specifics - new
    volatile bool canAcceptPacket;                // Flag that tells the user that a new packet may be sent
    void send(const uint8_t* data, uint8_t size); // Sends a DCC packet (3–6 bytes including XOR).

    // Service Mode (SM): Methods to enter / leave
    void enterServiceMode(void);                  // send Long preamble, resets and repeat SM packets
    void leaveServiceMode(void);                  // Back to normal mode
    bool isServiceModeEnabled(void);              // True while SM (long preamble / resets) is active

    // Service Mode packet transmission and repeats
    void setServiceModeMaxRepeats(uint8_t value); // Set maximum number of SM packet repeats
    bool isFirstServiceModePacket(void);          // True only for the first packet of a new SM repeat sequence
    bool isServiceModeRepeating(void);            // True while the ISR is repeating the current SM packet
    void stopServiceModeRepeats(void);            // Abort SM retransmissions (e.g. after ACK)

    // RailCom specific
    void setRailCom(bool active);                 // Enable / disable generation of the RailCom gap
    bool getRailCom(void);                        // Is generation of the RailCom gap enabled?
    bool railComGap(void);                        // Does the DCC Harware generate a Railcom gap at this moment?

    // Init, start and stop
    void StopOutputSignal(void);                  // Level of DCC rail output pins become LOW (no impact on dccMonitorPin)
    void RunOutputSignal(void);                   // Level of DCC rail output pins become HIGH (no impact on dccMonitorPin)
    void setupWaveformGenerator();                // Setup and start the waveform generator (ISR)

    DccPacketEngine();                            // Constructor declaration

  private:
    void resetStateMachine();                     // To (re)set the state machine to a defined state
};

extern DccPacketEngine dccPacketEngine;           // The dccPacketEngine must be accessible externally
