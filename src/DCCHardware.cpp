#include <Arduino.h>

#define Z21PG

// Create a define for all DxCore variants, to improve readability
#if defined(__AVR_DA__) || defined(__AVR_DB__) || defined(__AVR_DD__) || \
    defined(__AVR_EA__) || defined(__AVR_EB__)
  #define AVR_DXCORE
#endif


#if defined(Z21PG)

  #if defined(AVR_DXCORE)
    // DxCore, all versions
    #include "variants-Z21PG/DCCHardware_dxcore_sw_tcb1.inc"            // TCB1 and software pin toggle
  #else
    // Other AVR (mega2560 etc.)
    #include "variants-Z21PG/DCCHardware_atmega_sw_timer1.inc"          // Timer1 and software pin toggle
    //#include "variants-Z21PG/DCCHardware_legacy.inc"                  // This is close to the original driver
  #endif

#elif defined(AVR_DXCORE)
  // DxCore, all versions (non-Z21PG)
  #include "variants-HQ/DCCHardware_dxcore_hw_tca0.inc"

#endif
