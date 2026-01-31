#include <Arduino.h>

#define Z21PG
#include <Arduino.h>

#define Z21PG

// Create a define for all DxCore variants, to improve readability
#if defined(__AVR_DA__) || defined(__AVR_DB__) || defined(__AVR_DD__) || \
    defined(__AVR_EA__) || defined(__AVR_EB__)
  #define AVR_DXCORE
#endif


#if defined(Z21PG)

  // ---------------- AVR DxCore ----------------
  #if defined(AVR_DXCORE)
    #include "variants-Z21PG/DCCHardware_dxcore_sw_tcb1.inc"

  // ---------------- ESP32 (all) ----------------
  #elif defined(ARDUINO_ARCH_ESP32)

    // ---- ESP32-C series (RISC-V) ----
    #if defined(CONFIG_IDF_TARGET_ESP32C3) || \
        defined(CONFIG_IDF_TARGET_ESP32C5) || \
        defined(CONFIG_IDF_TARGET_ESP32C6)

      #include "variants-Z21PG/DCCHardware_ESP32C.inc"

    // ---- ESP32-S series (Xtensa: ESP32 / S2 / S3) ----
    #else
      #include "variants-Z21PG/DCCHardware_ESP32S.inc"
    #endif

  // ---------------- Other AVR ----------------
  #else
    #include "variants-Z21PG/DCCHardware_atmega_sw_timer1.inc"
  #endif


// ================= Non-Z21PG =================
#elif defined(AVR_DXCORE)
  #include "variants-HQ/DCCHardware_dxcore_hw_tca0.inc"
#endif
