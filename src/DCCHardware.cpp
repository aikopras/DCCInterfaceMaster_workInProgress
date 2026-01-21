#include <Arduino.h>


#if defined(__AVR_DA__) || defined(__AVR_DB__) || defined(__AVR_DD__) || defined(__AVR_EA__) || defined(__AVR_EB__)
  // DxCore, all versions
  #include "variants/DCCHardware_dxcore_sw_tcb1.inc"            // TCB1 and software pin toggle
  // #include "variants/DCCHardware_dxcore_hw_tca0.inc"         // TCA0 and hardware pin toggle
#else
  //#include "variants/DCCHardware_legacy.inc"                  // This is the original driver
  #include "variants/DCCHardware_atmega_sw_timer1.inc"          // Functional equivalent to the original driver
#endif
