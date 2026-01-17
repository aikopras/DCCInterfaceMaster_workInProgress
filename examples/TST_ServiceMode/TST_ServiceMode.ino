#if defined(__AVR_DA__) || defined(__AVR_DB__) || defined(__AVR_DD__) || defined(__AVR_EA__) || defined(__AVR_EB__)
#include <DCCPacketScheduler_new.h>
  #define monitor Serial1
  #define PIN_MONITOR PIN_PB0
  #define PIN_DCC     PIN_PB1
  #define PIN_DCC_INV PIN_PB2
  #define PIN_TEST1   PIN_PB3
  #define PIN_TEST2   PIN_PB4
  #define PIN_TEST3   PIN_PB5
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

  
DCCPacketScheduler dps;
long lastTime;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Service mode test");

  pinMode(PIN_TEST1, OUTPUT);
  pinMode(PIN_TEST2, OUTPUT);
  digitalWrite(PIN_TEST1, LOW);
  digitalWrite(PIN_TEST2, LOW);
  
  dps.setup(PIN_DCC, PIN_DCC_INV, DCC128, 0);
  dps.enable_additional_DCC_output(PIN_MONITOR);

  dps.setpower(ON);
  delay(1000);   // even "normaal DCC" zichtbaar maken
}


void loop() {

  if ((millis() - lastTime) > 3000) {
    lastTime = millis();
    digitalWrite(PIN_TEST1, HIGH); 
    Serial.println("Start Service Mode verify CV4 == 4");
    dps.opsVerifyDirectCV(4, 4);
    digitalWrite(PIN_TEST1, LOW);
  }

  dps.update();
}
