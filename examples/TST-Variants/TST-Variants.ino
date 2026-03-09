// Test file for dccHardware
//
// Relevant commands for the packet scheduler
// void setup(uint8_t pin, uint8_t pin2, uint8_t steps = DCC128, uint8_t format = ROCO, uint8_t power = ON);   //for any post-constructor initialization - with RailCom
// void enable_additional_DCC_output(uint8_t pin); //extra DCC signal for S88/LocoNet without Shutdown and Railcom
// void disable_additional_DCC_output(void);
#include <Arduino.h>
#include <DCCPacketScheduler_new.h>
#include <DCCHardware.h>
#include "pins.h"
#include "support.h"

extern DccPacketEngine dccPacketEngine;           // The dccPacketEngine must be accessible externally


void setup() {
  monitor.begin(115200);
  delay(500);
//  monitor.println("Type 0 to start test");
//  while (!readSerialCommand(Serial, inputValue) || inputValue != 0) {}
  monitor.println("Test started");

  pinMode(PIN_DCC, OUTPUT);
  pinMode(PIN_AUX, OUTPUT);
  pinMode(PIN_MONITOR, OUTPUT);
  
//  pinMode(PIN_TEST1, OUTPUT);
//  pinMode(PIN_TEST2, OUTPUT);
//  pinMode(PIN_TEST3, OUTPUT);
//  digitalWrite(PIN_TEST1, LOW);
//  digitalWrite(PIN_TEST2, LOW);
//  digitalWrite(PIN_TEST3, LOW);

  dps.setup(PIN_DCC, PIN_AUX , DCC128, SwitchFormat);  //with Railcom
  dps.enable_additional_DCC_output(PIN_MONITOR);
  dps.setpower(ON);

  accessory = true;
  dps.setrailcom(true);
  //  loco = true;

  //monitor.print("DCC Pin: "); monitor.println(dccPacketEngine.dccRailPin);
  //monitor.print("INV Pin: "); monitor.println(dccPacketEngine.dccRailAuxPin);
  //monitor.print("MON Pin: "); monitor.println(dccPacketEngine.dccMonitorPin);
  
}


void loop() {
  if (readSerialCommand(monitor, inputValue)) {
    monitor.print("Number received: ");
    monitor.println(inputValue);
    switch(inputValue) {
      case 1: dps.setpower(ON);                               // Signal on the rails
      return;
      case 2: dps.setpower(OFF);                              // No power on the rails
      return;
      case 3: dps.setrailcom(true);                           // With Railcom gap
      return;
      case 4: dps.setrailcom(false);                          // Without Railcom gap
      return;
      case 5: dps.enable_additional_DCC_output(PIN_MONITOR);  // Enable the monitor signal
      return;
      case 6: dps.disable_additional_DCC_output();            // Disable the monitor signal
      return;
      case 7: accessory = true;                              // Start sending Accessory commands
      return;
      case 8: accessory = false;                             // Stop sending Accessory commands
      return;
      case 9: loco = true;                                   // Start sending Loco commands
      return;
      case 10: loco = false;                                  // Stop sending Loco commands
      return;
      case 11: powerSwitching = true;                         // Start switching power on and off
      return;
      case 12: powerSwitching = false;                        // Stop switching power on and off
      return;
      case 13: testSM = true;                                 // Start Service Mode
      return;
      case 14: testSM = false;                                // Stop Service Mode
      return;
      case 15: dccPacketEngine.setRailComGapInAux(1);         // Aux pin with RC cutout
      return;
      case 16: dccPacketEngine.setRailComGapInAux(0);         // Aux pin without RC cutout
      return;
      case 17: dccPacketEngine.setAuxActiveLevel(1);          // Aux Active Level
      return;
      case 18: dccPacketEngine.setAuxActiveLevel(0);          // Aux Active Level
      return;
      case 19: dccPacketEngine.setDccSignalInverted(1);       // Invert DCC signal
      return;
      case 20: dccPacketEngine.setDccSignalInverted(0);       // Normal DCC signal
      return;      
      case 21: dccPacketEngine.setPreambleLength(17);         // Change Preamble
      return;
      case 22: dccPacketEngine.setPreambleLength(18);         // Change Preamble
      return;      
      case 23: dccPacketEngine.setPreambleLengthSM(20);       // Change Preamble SM
      return;
      case 24: dccPacketEngine.setPreambleLengthSM(22);       // Change Preamble SM
      return;      
      case 25: dccPacketEngine.enterServiceMode();            // Enter SM / driver direct
      return;
      case 26: dccPacketEngine.leaveServiceMode();            // Leave SM / driver direct
      return;      
      case 27: dccPacketEngine.RunOutputSignal();             //  
      return;
      case 28: dccPacketEngine.StopOutputSignal();            // 
      return;      
      case 29: testRCGap = true;  
      return;
      case 30: testRCGap = false; 
      return;      
      default:
      return;  
    }
  }

// digitalWrite(PIN_TEST1,HIGH);digitalWrite(PIN_TEST1,LOW);
// digitalWrite(PIN_TEST2,HIGH);digitalWrite(PIN_TEST2,LOW);
// digitalWrite(PIN_TEST3,HIGH);digitalWrite(PIN_TEST3,LOW);
  sendAccessory();
  sendLoco();
  powerOnOff();
  startSM();
  showRCGap();
  
  dps.update();
}
