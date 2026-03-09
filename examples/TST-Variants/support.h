//#define SwitchFormat 0   //ROCO (+4) or IB (+0)
#define SwitchFormat 4   // ROCO

DCCPacketScheduler dps;

bool      accessory;
bool      loco;
bool      powerSwitching;
bool      testSM;
bool      testRCGap;
uint16_t  accAddress;
uint16_t  locoAddress;
long      accTimer;
long      locoTimer;
long      powerTimer;
long      smTimer;
long      inputValue;


bool readSerialCommand(Stream &ser, long &outValue) {
  // This allows non-blocking reading of the input
  // The "standard" Arduino int value = monitor.parseInt(); blocks the code for 1 second
  static long value = 0;
  static bool inNumber = false;
  while (ser.available()) {
    char c = ser.read();
    if (c >= '0' && c <= '9') {
      inNumber = true;
      value = value * 10 + (c - '0');
    }
    else if (c == '\n' || c == '\r') {
      if (inNumber) {
        outValue = value;
        value = 0;
        inNumber = false;
        return true;        // ✅ complete input received
      }
      // something else: ignore empty ENTER, 
    }
    else {
      // none numeric. Drop everything
      if (inNumber) {
        outValue = value;
        value = 0;
        inNumber = false;
        return true;
      }
    }
  }
  return false;             // ❌ incomplete input
}


void sendAccessory() {
  if (!accessory) return;
  if (millis() - accTimer > 1000) {
    accAddress++;
    if (accAddress > 999) accAddress = 0;
    dps.setBasicAccessoryPos(accAddress, 1, true);
    accTimer = millis();
  }
};
  
void sendLoco() {
 if (!loco) return;
  if (millis() - locoTimer > 1000) {
    locoAddress++;
    if (locoAddress > 999) locoAddress = 1;
    dps.setSpeed(locoAddress, 4);
    locoTimer = millis();
  }
};


uint8_t currentPower;
void powerOnOff(void) {
  if (!powerSwitching) return;
  if ((millis() - powerTimer) > 500) {
    powerTimer = millis();
    currentPower = currentPower + 1;
    if (currentPower >= 3) currentPower = 0;
    switch (currentPower) {
      case 0:
        dps.setpower(OFF);
      break;  
      case 1:
        dps.setpower(ON);
      break;  
      case 2:
        ;
      break;  
    }
  }
}

void startSM(void) {
  if (!testSM) return;
  if ((millis() - smTimer) > 700) {
    smTimer = millis();
//    digitalWrite(14, HIGH); 
    dps.opsVerifyDirectCV(4, 4);
//    digitalWrite(14, LOW);
  }
}

void showRCGap() {
  if (!testRCGap) return;
  if (dccPacketEngine.railComGap()) digitalWriteFast(PIN_TEST1, HIGH);
//  else digitalWriteFast(PIN_TEST1, LOW);
}
