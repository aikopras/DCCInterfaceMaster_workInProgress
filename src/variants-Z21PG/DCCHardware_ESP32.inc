//******************************************************************************
//
// File:            DCCHardware_ESP32C.inc
// Purpose:         DCC Waveform Generator for ESP32 processors (bit-banging)
//                  ESP32-C (RISC-V) and ESP32-S (Xtensa)
// Copyright 2026:  Aiko Pras
// History:         2026-02-22 / Aiko Pras - First version
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
// This file implements the ESP32-S and ESP32-C series hardware abstraction layer
// for software-driven waveform generation using a hardware timer ISR.
//
// It connects the processor-independent DCC state machine
// (DCCHardwareCommon.inc) to the ESP32 hardware by providing:
//  - Configuration and control of an ESP32 hardware timer (gptimer)
//    with 1 µs timer resolution for accurate DCC bit timing
//  - Binding of the generic DCC ISR to an ESP32 timer interrupt callback
//  - Inline macros for fast, deterministic GPIO pin toggling using
//    gpio_set_level macros (see below)
//  - Initialization, enabling, and disabling of the DCC rail outputs
//    and optional monitor output
//
// The DCC waveform is generated entirely in software (bit-banging).
// All output transitions are executed inside the timer ISR to guarantee
// precise timing independent of the main application code.
//
//
// GPIO toggle method comparison (ESP-IDF v5+, ESP32 S/C variants)
// ---------------------------------------------------------------
// REG_WRITE():
//   - Fastest method (direct register access)
//   - Typical toggle time: ~20–40 ns
//   - Not portable across ESP32 families (register layout differs)
//   - Not future-proof (may break with new SoCs or IDF changes)
//
// gpio_ll_set_level():
//   - Low-level HAL function used internally by ESP-IDF
//   - Typical toggle time: ~40–80 ns
//   - Portable across ESP32 S/C variants
//   - Reasonably future-proof, but not part of the strict public API
//
// gpio_set_level():
//   - Official high-level driver API
//   - Typical toggle time: ~100–250 ns
//   - Fully portable across ESP32 families
//   - Fully future-proof and officially supported
//
// Note: This implementation requires Arduino-ESP32 core version 3.x or higher
// (ESP-IDF Version 5).
//
//******************************************************************************
#include <Arduino.h>
#include "driver/gptimer.h"
#include "../DCCHardware.h"


//******************************************************************************
// Part 1: Pins en output signals
//******************************************************************************
// Direct register access for pin changes
// Create a structure for each output pin. This structure provides
// direct access to the ESP32 GPIO register, using a bitmask.
typedef struct {
    gpio_num_t pin;        // GPIO number (Arduino pin)
    volatile bool enable;  // No pin toggle during RailCom gap / Power-down
} DccOut_t;

// Instantiate three objects; one for each pin (output channel)
static DccOut_t dccMonitor;   // Create an instance for the dccMonitor
static DccOut_t dccRail;      // Create an instance for the dccRail
static DccOut_t dccRailInv;   // Create an instance for the dccRailInv

// Create a dummy variable to avoid null pointer dereferencing
static volatile uint32_t gpio_dummy;

inline void initDccPin(DccOut_t &pinObj, uint8_t &arduinoPin) {
  if (GPIO_IS_VALID_OUTPUT_GPIO(arduinoPin)) {
    pinObj.pin = static_cast<gpio_num_t>(arduinoPin);
    pinObj.enable = false;
    gpio_reset_pin(pinObj.pin);
    gpio_set_direction(pinObj.pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pinObj.pin, 0);     // force LOW
  }
  else {
    pinObj.pin = GPIO_NUM_MAX;         // invalid pin
    pinObj.enable = false;
    arduinoPin = 0xFF;
  }
}

// #defines to set, clear and toggle the output signals
// Although only some will be used, for completeness all options are given.
#define DCCOUT_HGH   {gpio_set_level(dccRail.pin, 1);}
#define DCCOUT_LOW   {gpio_set_level(dccRail.pin, 0);}
#define DCCOUT_TGL   {gpio_set_level(dccRail.pin, !gpio_get_level(dccRail.pin));}
#define DCCINV_HGH   {gpio_set_level(dccRailInv.pin, 1);}
#define DCCINV_LOW   {gpio_set_level(dccRailInv.pin, 0);}
#define DCCINV_TGL   {gpio_set_level(dccRailInv.pin, !gpio_get_level(dccRailInv.pin));}
#define DCCMON_HGH   {gpio_set_level(dccMonitor.pin, 1);}
#define DCCMON_LOW   {gpio_set_level(dccMonitor.pin, 0);}
#define DCCMON_TGL   {gpio_set_level(dccMonitor.pin, !gpio_get_level(dccMonitor.pin));}

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

// Set the timer alarms for specific DCC timings (µs resolution)
#define TMR_ONE              setDccTimerAlarm(edata->alarm_value + one_count)
#define TMR_ZERO             setDccTimerAlarm(edata->alarm_value + zero_count)
#define TMR_RAILCOM_STARTBIT setDccTimerAlarm(edata->alarm_value + half_one_count)

static inline void setDccTimerAlarm(uint64_t newAlarmValue) {
  gptimer_alarm_config_t alarmConfig = {};                              // Allocate space
  alarmConfig.alarm_count = newAlarmValue;                              // One shot
  alarmConfig.flags.auto_reload_on_alarm = false;                       // automatic reset upon match
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
