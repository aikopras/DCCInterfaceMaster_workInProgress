// Test file for dccHardware
//
// Relevant commands for the packet scheduler
// void setup(uint8_t pin, uint8_t pin2, uint8_t steps = DCC128, uint8_t format = ROCO, uint8_t power = ON);   //for any post-constructor initialization - with RailCom
// void enable_additional_DCC_output(uint8_t pin); //extra DCC signal for S88/LocoNet without Shutdown and Railcom
// void disable_additional_DCC_output(void);

#include <DCCPacketScheduler_new.h>
#define SwitchFormat 0   //ROCO (+4) or IB (+0)

#define monitor Serial1

#define PIN_MONITOR PIN_PB0
#define PIN_DCC     PIN_PB1
#define PIN_DCC_INV PIN_PB2
#define PIN_TEST1   PIN_PB3
#define PIN_TEST2   PIN_PB4
#define PIN_TEST3   PIN_PB5


DCCPacketScheduler dps;

bool      accessory;
bool      loco;
uint16_t  accAddress;
uint16_t  locoAddress;
long      accTimer;
long      locoTimer;


void setup() {
  monitor.begin(115200);
  delay(500);
  monitor.println("Start");
  dps.setup(PIN_DCC, PIN_DCC_INV, DCC128, SwitchFormat);  //with Railcom
  dps.enable_additional_DCC_output(PIN_MONITOR);
  dps.setpower(ON);
  pinMode(PIN_TEST1, OUTPUT);
  pinMode(PIN_TEST2, OUTPUT);
  pinMode(PIN_TEST3, OUTPUT);

  digitalWrite(PIN_TEST1, LOW);
  digitalWrite(PIN_TEST2, LOW);
  digitalWrite(PIN_TEST3, LOW);
//  dps.setrailcom(false);
//  dps.setpower(SERVICE);
  accessory = true;
}


void sendAccessory() {
  if (!accessory) return;
  if (millis() - accTimer > 100) {
    accAddress++;
    if (accAddress > 999) accAddress = 0;
    dps.setBasicAccessoryPos(accAddress, 1, true);
    accTimer = millis();
  }
};
  


void sendLoco() {
 if (!loco) return;
  if (millis() - locoTimer > 100) {
    locoAddress++;
    if (locoAddress > 999) locoAddress = 0;
    dps.setSpeed(locoAddress, 0);
    locoTimer = millis();
  }
};


void loop() {
  if (monitor.available() > 0) {
    int value = monitor.parseInt();
    monitor.print("Number received: ");
    monitor.println(value);
    switch(value) {
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
      case 14: dps.opsVerifyDirectCV(2, 2);                   // Service Mode: Verify if CV 2 = 2
      return;
      case 15: 
        dps.setpower(SERVICE);
        dps.opsVerifyDirectCV(2, 2);                          // Service Mode: Verify if CV 2 = 2
        dps.setpower(ON);
      return;
      default:
      return;  
    }
  }
  //digitalWrite(9,HIGH);digitalWrite(9,LOW);
  sendAccessory();
  sendLoco();
  dps.update();
}
