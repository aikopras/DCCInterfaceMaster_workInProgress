// Test file for dccHardware
//
// Relevant commands for the packet scheduler
// void setup(uint8_t pin, uint8_t pin2, uint8_t steps = DCC128, uint8_t format = ROCO, uint8_t power = ON);   //for any post-constructor initialization - with RailCom
// void enable_additional_DCC_output(uint8_t pin); //extra DCC signal for S88/LocoNet without Shutdown and Railcom
// void disable_additional_DCC_output(void);

#if defined(__AVR_DA__) || defined(__AVR_DB__) || defined(__AVR_DD__) || defined(__AVR_EA__) || defined(__AVR_EB__)
#include <DCCPacketScheduler_new.h>
  #define monitor Serial1
  #define PIN_MONITOR PIN_PD0         // These pins can be used with TCA0
  #define PIN_DCC     PIN_PD1
  #define PIN_DCC_INV PIN_PD2
  #define PIN_TEST1   PIN_PD3
  #define PIN_TEST2   PIN_PD4
  #define PIN_TEST3   PIN_PD5
#else
  #include <DCCPacketScheduler_new.h>
  #define monitor Serial
  #define PIN_MONITOR 5
  #define PIN_DCC     6
  #define PIN_DCC_INV 7
  #define PIN_TEST1   8
  #define PIN_TEST2   9
  #define PIN_TEST3   10
#endif

#include "support.h"


void setup() {
  monitor.begin(115200);
  delay(500);
  monitor.println("Start Test");
  dps.setup(PIN_DCC, PIN_DCC_INV, DCC128, SwitchFormat);  //with Railcom
  dps.enable_additional_DCC_output(PIN_MONITOR);
  dps.setpower(ON);
  pinMode(PIN_TEST1, OUTPUT);
  pinMode(PIN_TEST2, OUTPUT);
  pinMode(PIN_TEST3, OUTPUT);
  digitalWrite(PIN_TEST1, LOW);
  digitalWrite(PIN_TEST2, LOW);
  digitalWrite(PIN_TEST3, LOW);
  // Temporary
  accessory = true;
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
      case 3: dps.setpower(ESTOP);                            // No Loco drive but rails have power
      return;
      case 4: dps.setpower(SERVICE);                          // Enter Service Mode
      return;
      case 5: dps.setrailcom(true);                           // With Railcom gap
      return;
      case 6: dps.setrailcom(false);                          // Without Railcom gap
      return;
      case 7: dps.enable_additional_DCC_output(PIN_MONITOR);  // Enable the monitor signal
      return;
      case 8: dps.disable_additional_DCC_output();            // Disable the monitor signal
      return;
      case 10: accessory = true;                              // Start sending Accessory commands
      return;
      case 11: accessory = false;                             // Stop sending Accessory commands
      return;
      case 12: loco = true;                                   // Start sending Loco commands
      return;
      case 13: loco = false;                                  // Stop sending Loco commands
      return;
      case 14: powerSwitching = true;                         // Start switching power on and off
      return;
      case 15: powerSwitching = false;                        // Stop switching power on and off
      return;
      case 16: testSM = true;                                 // Start Service Mode
      return;
      case 17: testSM = false;                                // Stop Service Mode
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
  dps.update();
}
