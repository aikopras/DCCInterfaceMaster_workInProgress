#include <Arduino.h>

#define Z21PG

// Create a define for all DxCore variants, to improve readability
#if defined(__AVR_DA__) || defined(__AVR_DB__) || defined(__AVR_DD__) || \
    defined(__AVR_EA__) || defined(__AVR_EB__)
  #define AVR_DXCORE
#endif

// Same for the ATMega Timer1 support
#if defined(__AVR_ATmega328P__)  || defined(__AVR_ATmega2560__) || \
    defined(__AVR_ATmega1280__) || defined(__AVR_ATmega32U4__) || \
    defined(__AVR_ATmega644P__)  || defined(__AVR_ATmega644PA__)

  #define ATMEGA_HAS_TIMER1
#endif


// ================= Z21PG =================
#if defined(Z21PG)

  // AVR DxCore
  #if defined(AVR_DXCORE)
    #include "variants-Z21PG/DCCHardware_dxcore_sw_tcb1.inc"

  // Traditional AVRs
  #elif defined(ATMEGA_HAS_TIMER1)
    #include "variants-Z21PG/DCCHardware_atmega_sw_timer1.inc"

  // ESP32 (all)
  #elif defined(ARDUINO_ARCH_ESP32)
    #include "variants-Z21PG/DCCHardware_ESP32.inc"
    // #include "variants-Z21PG/DCCHardware_ESP32S.inc"

  // ESP8266 / ESP8285
  #elif defined(ARDUINO_ARCH_ESP8266)
    #include "variants-Z21PG/DCCHardware_ESP8266.inc"

  // SAMD processors
  #elif defined(ARDUINO_ARCH_SAMD)
    #include "variants-Z21PG/DCCHardware_SAMD_sw_timer.inc"

  // Others
  #else
    #error No DCCHardware implementation for this processor / board (Z21PG mode)
  #endif



// ================= HQ =================
#else

  // AVR DxCore
  #if defined(AVR_DXCORE)
    #include "variants-HQ/DCCHardware_DxCore_TCA0.inc"

  // ESP32 (all)
  #elif defined(ARDUINO_ARCH_ESP32)
    #include "variants-HQ/DCCHardware-ESP32-RMT.inc"

  // Others
  #else
    #error No DCCHardware implementation for this processor / board (HQ mode)
  #endif

#endif
