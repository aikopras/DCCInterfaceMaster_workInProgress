#define SwitchFormat 0   //ROCO (+4) or IB (+0)

DCCPacketScheduler dps;

bool      accessory;
bool      loco;
bool      powerSwitching;
bool      testSM;
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

void powerOnOff(void) {
  if (!powerSwitching) return;
  if ((millis() - powerTimer) > 1000) {
    powerTimer = millis();
    digitalWrite(PIN_TEST3, HIGH); digitalWrite(PIN_TEST3, LOW);
    if (dps.getpower() == OFF) dps.setpower(ON);
      else dps.setpower(OFF);
  }
}

void startSM(void) {
  if (!testSM) return;
  if ((millis() - smTimer) > 3000) {
    smTimer = millis();
    digitalWrite(PIN_TEST3, HIGH); 
    dps.opsVerifyDirectCV(4, 4);
    digitalWrite(PIN_TEST3, LOW);
  }
}
