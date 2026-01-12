#include <Arduino.h>


#if defined(__AVR_DA__) || defined(__AVR_DB__) || defined(__AVR_DD__) || defined(__AVR_EA__) || defined(__AVR_EB__)
  // DxCore, all versions
  #include "variants/DCCHardware_dxcore.inc"
#else
  // This is the original driver
  #include "variants/DCCHardware_legacy.inc"
#endif
