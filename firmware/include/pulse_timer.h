#pragma once
#include "driver/timer.h"
#include "esp_timer.h"
#include "soc/gpio_struct.h"

// One independent alarm owns pulse release. IRAM code and peripheral registers
// remain accessible during flash writes; no allocation/logging in the ISR.
static_assert(PIN_PULSE >= 0 && PIN_PULSE < 32, "pulse GPIO must be in bank 0");
static bool pulseTimerReady = false;
static bool initPulseTimer(bool (*callback)(void *)) {
  timer_config_t config = {};
  config.divider = 80; // ESP32 APB clock / 80 = one microsecond
  config.counter_dir = TIMER_COUNT_UP;
  config.counter_en = TIMER_PAUSE;
  config.alarm_en = TIMER_ALARM_DIS;
  config.auto_reload = TIMER_AUTORELOAD_DIS;
  pulseTimerReady = timer_init(TIMER_GROUP_0, TIMER_0, &config) == ESP_OK &&
      timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, callback, nullptr,
                             ESP_INTR_FLAG_IRAM) == ESP_OK &&
      timer_enable_intr(TIMER_GROUP_0, TIMER_0) == ESP_OK;
  return pulseTimerReady;
}
static bool preparePulseTimer(uint32_t ms) {
  return pulseTimerReady &&
      timer_pause(TIMER_GROUP_0, TIMER_0) == ESP_OK &&
      timer_set_alarm(TIMER_GROUP_0, TIMER_0, TIMER_ALARM_DIS) == ESP_OK &&
      timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0) == ESP_OK &&
      timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, uint64_t(ms) * 1000) == ESP_OK &&
      timer_set_alarm(TIMER_GROUP_0, TIMER_0, TIMER_ALARM_EN) == ESP_OK;
}
static bool startPulseTimer() {
  return timer_start(TIMER_GROUP_0, TIMER_0) == ESP_OK;
}
static uint32_t IRAM_ATTR pulseMicros() { return (uint32_t)esp_timer_get_time(); }
static bool IRAM_ATTR pulsePadHigh() { return (GPIO.in & (1UL << PIN_PULSE)) != 0; }
static void IRAM_ATTR pulsePinLow() { GPIO.out_w1tc = 1UL << PIN_PULSE; }
