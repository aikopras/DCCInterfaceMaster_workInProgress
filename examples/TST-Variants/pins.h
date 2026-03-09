#if defined(__AVR_DA__) || defined(__AVR_DB__) || defined(__AVR_DD__) || defined(__AVR_EA__) || defined(__AVR_EB__)
  #define monitor Serial1
  #define PIN_DCC     PIN_PD0         // These pins can be used with TCA0
  #define PIN_AUX     PIN_PA4         // One of the two possible output pins (PA4/PA5)
  #define PIN_MONITOR PIN_PD2
  #define PIN_TEST1   PIN_PF3
  #define PIN_TEST2   PIN_PD7
  #define PIN_TEST3   PIN_PD2
#elif defined(ESP32)
  #define monitor Serial
  #define PIN_DCC     18
  #define PIN_AUX     19
  #define PIN_MONITOR 21
//  #define PIN_MONITOR 0xFF
//  #define PIN_TEST1   12
//  #define PIN_TEST2   13
//  #define PIN_TEST3   14
#else
  #define monitor Serial
  #define PIN_MONITOR 14  // 5
  #define PIN_DCC     12  // 6
  #define PIN_AUX     13  // 7
  #define PIN_TEST1   8
  #define PIN_TEST2   9
  #define PIN_TEST3   10
#endif

// 8266: D5, D6, D7 als alias voor GPIO14, GPIO12, GPIO13.
