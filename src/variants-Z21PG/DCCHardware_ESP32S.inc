//******************************************************************************
//
// File:            DCCHardware_ESP32S.inc
// Purpose:         DCC Waveform Generator for ESP32-S (Xtensa) processors
//                  (bit-banging), using direct register access for pin changes.
// Copyright 2026:  Aiko Pras
// History:         2026-01-31 / Aiko Pras - First version
//                  2026-02-23 / AP V2.0: HWTimer replaced by GPTimer (more portable)
//                                        Note: a generic ESP32 driver exists too.
//
//
// Historical credits:
// - Wolfgang Kufer (OpenDCC, http://opendcc.de)
//     * Original DCC waveform generation concepts and basic routines
//     * Provides timing calculations for '1' and '0' bits
//
// - Don Goodman-Wilson (Railstars CmdrArduino)
//     * Adapted OpenDCC waveform generation for Arduino
//     * Packet scheduling logic
//     * Initial integration with Arduino timers and interrupts
//
// - Philipp Gahtow (DCCInterfaceMaster, https://github.com/Digital-MoBa/DCCInterfaceMaster)
//     * Rewrote waveform generator for multiple MCU platforms (Arduino/ESP)
//     * Modified ISR and preamble handling
//     * Added support for service mode, RailCom cutouts and 3 DCC output pins
//
// Hardware layer overview:
// This file implements the ESP32-S series (Xtensa: ESP32 / S2 / S3) specific
// hardware abstraction layer for software-based DCC waveform generation
// using hw_timer_t.
//
// It connects the processor-independent DCC state machine
// (DCCHardwareCommon.inc) to the ESP32 hardware by providing:
//
//   - Configuration and control of an ESP32 hardware timer (GPTimer)
//     with microsecond resolution for accurate DCC bit timing
//   - Binding of the generic DCC ISR to an ESP32 timer interrupt callback
//   - Inline macros for fast, deterministic GPIO pin toggling using
//     direct register access
//   - Initialization, enabling, and disabling of the DCC rail outputs
//     and optional monitor output
//
// The DCC waveform is generated entirely in software (bit-banging).
// All output transitions are executed inside the timer ISR to guarantee
// precise timing independent of the main application code.
//
// Note 1: This implementation requires Arduino-ESP32 core version 3.x or higher
// (ESP-IDF Version 5).
//
//
// Comparison with the generic ESP32 driver
// ----------------------------------------
// A generic ESP32 hardware driver is also available that uses the official
// gpio_set_level() API for pin control. That implementation is fully
// portable across ESP32 families (Xtensa and RISC-V) and relies only on
// the supported ESP-IDF driver layer.
//
// This Xtensa-specific driver differs in that it uses direct register
// access (GPIO.out_w1ts / out_w1tc) for setting and clearing output pins.
// This approach results in:
//   - Slightly lower GPIO toggle latency
//   - Fully deterministic single-instruction register writes
//   - Independence from changes in the higher-level GPIO driver API
//
// In practice, for DCC bit timing (58 µs / 100 µs), the performance
// difference between direct register access and gpio_set_level() is
// negligible, since timing accuracy is primarily determined by the
// hardware timer and ISR latency rather than GPIO write speed.
//
// This file is therefore retained mainly as a low-level, Xtensa-optimized
// fallback implementation. It serves as a reference and backup solution
// in case future ESP-IDF or Arduino-ESP32 changes introduce unexpected
// problems in the generic driver.
//
//******************************************************************************
#include <Arduino.h>
#include "soc/gpio_struct.h"    // For GPIO.out
#include "driver/gptimer.h"
#include "../DCCHardware.h"


//******************************************************************************
// Part 1: Pins en output signals
//******************************************************************************
// Direct register access for pin changes
// Create a structure for each output pin. This structure provides
// direct access to the ESP32 GPIO registers, using a bitmask.
typedef struct {
  volatile uint32_t *outset;    // GPIO out set register
  volatile uint32_t *outclr;    // GPIO out clear register
  volatile uint32_t *outtgl;    // GPIO out toggle register
  uint32_t bit;                 // Bitmask for the pin
  volatile bool enable;         // No pin toggle during RailCom gap / Power-down
} DccOut_t;

// Instantiate three objects; one for each pin (output channel)
static DccOut_t dccMonitor;   // Create an instance for the dccMonitor
static DccOut_t dccRail;      // Create an instance for the dccRail
static DccOut_t dccRailInv;   // Create an instance for the dccRailInv

// Create a dummy pointer address, to avoid problems with null pointers
static volatile uint32_t gpio_dummy;

inline void initDccPin(DccOut_t &pinObj, uint8_t &arduinoPin) {
  if (GPIO_IS_VALID_OUTPUT_GPIO(arduinoPin)) {
    pinMode(arduinoPin, OUTPUT);
    uint8_t gpio = arduinoPin;            // Simply use the Arduino pin
    pinObj.enable = false;
    if (gpio < 32) {
      pinObj.bit    = (1UL << gpio);
      pinObj.outset = &GPIO.out_w1ts;
      pinObj.outclr = &GPIO.out_w1tc;
      pinObj.outtgl = &GPIO.out;           // toggle via XOR
      GPIO.out_w1tc = pinObj.bit;          // force LOW
    } else {
      pinObj.bit    = (1UL << (gpio - 32));
      pinObj.outset = &GPIO.out1_w1ts.val;
      pinObj.outclr = &GPIO.out1_w1tc.val;
      pinObj.outtgl = &GPIO.out1.val;      // toggle via XOR
      GPIO.out1_w1tc.val = pinObj.bit;     // force LOW
    }
  }
  else {
    pinObj.outset = &gpio_dummy;
    pinObj.outclr = &gpio_dummy;
    pinObj.outtgl = &gpio_dummy;
    pinObj.bit    = 0;
    pinObj.enable = false;
    arduinoPin = 0xFF;
  }
}

// #defines to set, clear and toggle the output signals
// Although only some will be used, for completeness all options are given.
#define DCCOUT_HGH          {*dccRail.outset = dccRail.bit;}
#define DCCOUT_LOW          {*dccRail.outclr = dccRail.bit;}
#define DCCOUT_TGL          {*dccRail.outtgl ^= dccRail.bit;}
#define DCCINV_HGH          {*dccRailInv.outset = dccRailInv.bit;}
#define DCCINV_LOW          {*dccRailInv.outclr = dccRailInv.bit;}
#define DCCINV_TGL          {*dccRailInv.outtgl ^= dccRailInv.bit;}
#define DCCMON_HGH          {*dccMonitor.outset = dccMonitor.bit;}
#define DCCMON_LOW          {*dccMonitor.outclr = dccMonitor.bit;}
#define DCCMON_TGL          {*dccMonitor.outtgl ^= dccMonitor.bit;}


// defines to enable / disable the output signals
#define ENABLE_DCCMON       {dccMonitor.enable = true;}
#define DISABLE_DCCMON      {dccMonitor.enable = false;}
#define ENABLE_DCCOUT       {dccRail.enable = true;}
#define DISABLE_DCCOUT      {dccRail.enable = false;}
#define ENABLE_DCCINV       {dccRailInv.enable = true;}
#define DISABLE_DCCINV      {dccRailInv.enable = false;}

#define DCCOUT_IS_ENABLED   dccRail.enable
#define DCCINV_IS_ENABLED   dccRailInv.enable
#define DCCMON_IS_ENABLED   dccMonitor.enable

//******************************************************************************
// Part 2: Timer and ISR
//******************************************************************************
// Timer #defines using gptimer_handle_t
static gptimer_handle_t dccTimer = NULL;

// Forward declaration for the timer ISR (to avoid compile errors)
static bool dccTimerIsr(gptimer_handle_t timer,
  const gptimer_alarm_event_data_t *edata, void *user_ctx);

// The ISR is a call-back
#define ISR_START static bool IRAM_ATTR dccTimerIsr(gptimer_handle_t timer, \
  const gptimer_alarm_event_data_t *edata, void *user_ctx)
#define ISR_PROLOGUE {(void)timer; (void)edata; (void)user_ctx;}
#define ISR_EPILOGUE {return false;}

#define half_one_count  29ULL           // RCN 217
#define one_count       58ULL           // RCN 210
#define zero_count      100ULL          // RCN 210

#define TMR_ONE              setDccTimerAlarm(one_count)
#define TMR_ZERO             setDccTimerAlarm(zero_count)
#define TMR_RAILCOM_STARTBIT setDccTimerAlarm(half_one_count)

static inline void setDccTimerAlarm(uint64_t newAlarmValue) {
  gptimer_alarm_config_t alarmConfig = {};                              // Allocate space
  alarmConfig.reload_count = 0;                                         // Restart CNT from 0
  alarmConfig.alarm_count = newAlarmValue;                              // Compare Match value
  alarmConfig.flags.auto_reload_on_alarm = true;                        // automatic reset upon match
  gptimer_set_alarm_action(dccTimer, &alarmConfig);
}

static inline void enableDccTimer() {
  // Configure ESP32 General Purpose Timer
  gptimer_config_t timerConfig = {};                                    // New empty timer config
  timerConfig.clk_src = GPTIMER_CLK_SRC_DEFAULT;                        // Default clock
  timerConfig.direction = GPTIMER_COUNT_UP;                             // Increment
  timerConfig.resolution_hz = 1000000;                                  // 1 MHz => 1 µs per tick
  timerConfig.intr_priority = 0;                                        // Default priority
  // Create a timer instance
  ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig, &dccTimer));
  // Init the Timer call back function
  gptimer_event_callbacks_t cbs = {};                                   // New empty call back
  cbs.on_alarm = dccTimerIsr;                                           // the call back name
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(dccTimer, &cbs, NULL));
  // Create and initialise the alarm configuration
  setDccTimerAlarm(one_count);                                          // start with "1"-bit
  // Enable the timer, but do not yet start the timer!!
  ESP_ERROR_CHECK(gptimer_enable(dccTimer));
}

static inline void startDccTimer() {
  // Start the timer, and thus the ISR state machine
  ESP_ERROR_CHECK(gptimer_start(dccTimer));
}

void disableDccTimer() {
  if (!dccTimer) return;
  ESP_ERROR_CHECK(gptimer_stop(dccTimer));                         // Stop
  ESP_ERROR_CHECK(gptimer_disable(dccTimer));                      // Disable
  ESP_ERROR_CHECK(gptimer_del_timer(dccTimer));                    // Delete
  dccTimer = NULL;
}


//******************************************************************************
// Part 3: Continue with the processor independent code
//******************************************************************************
#include "DCCHardwareCommon.inc"
