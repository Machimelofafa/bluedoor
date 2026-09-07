// Hardware-free replay of the production state machine, not a second model.
#include <cassert>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <iostream>
#include <string>
#include <type_traits>
#include "tunables.h"

class String {
 public:
  std::string value;
  String(const char *s = "") : value(s) {}
  String(std::string s) : value(s) {}
  template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
  String(T n) : value(std::to_string(n)) {}
  friend String operator+(const String &a, const String &b) {
    return a.value + b.value;
  }
};
static uint32_t clockMs;
static uint32_t millis() { return clockMs; }
static String tsNow() { return "replay"; }
static String fmtDur(uint32_t ms) { return String(ms); }
static void logEvent(const char *, const String &) {}
static size_t testStrlcpy(char *dst, const char *src, size_t size) {
  if (size) { std::strncpy(dst, src, size - 1); dst[size - 1] = 0; }
  return std::strlen(src);
}
static struct { void putUInt(const char *, uint32_t) {} } prefs;
#if PULSE_ENABLED
enum { LOW = 0, HIGH = 1, INPUT_PULLUP = 2, OUTPUT = 3 };
static int outputLevel, outputMode;
static unsigned gpioHighWrites;
static void pinMode(int, int mode) { outputMode = mode; }
static int digitalRead(int) { return outputMode == INPUT_PULLUP ? LOW : outputLevel; }
static void digitalWrite(int, int level) {
  outputLevel = level;
  if (level == HIGH) ++gpioHighWrites;
}
static void delayMicroseconds(unsigned) {}
#endif

#define strlcpy testStrlcpy
#include "machine.inc"

static void reset() {
  clockMs = 1;
  sysState = SysState::DisarmedBoot;
  stateSinceMs = lastEvidenceMs = lastSightMs = 0;
  shortRearm = false;
  enc = {}; agg = {}; ses = {};
  carHome = true;
  firedCount = relaxedCount = refusalCount = totalSightings = 0;
#if PULSE_ENABLED
  pulseCount = gpioHighWrites = 0;
  pulseActive = false;
  outputLevel = LOW;
  runMode = RunMode::DryRun;
#endif
}
static void at(uint32_t ms) { clockMs = ms; machineTick(); }
static void sight(uint32_t ms, int8_t rssi) {
  at(ms);
  onCarSighting(rssi, "test-beacon");
}
static const uint32_t base = AWAY_MIN_MS + 1;
static void armAway() {
  reset();
  setPresence(false, "test departure already confirmed");
  at(base);
  assert(sysState == SysState::Armed);
}
static void observedArrival() {
  // First eight individual packets from the 2026-09-07 failed HOME update.
  const int8_t rssi[] = {-99, -96, -85, -93, -89, -101, -88, -84};
  const uint32_t offsets[] = {0, 100, 4000, 4500, 4700, 4800, 4900, 5100};
  for (unsigned i = 0; i < 8; ++i) sight(base + offsets[i], rssi[i]);
  assert(firedCount == 1);
  assert(carHome); // Regression: the old firmware leaves this AWAY.
}
static void arrivalAndParkedWake() {
  armAway();
  observedArrival();
  // The real session peaked at -77, below the -75 transit threshold.
  sight(base + 6000, -77);
  sight(base + 60000, -93);
  sight(base + 84000, -89);
  at(base + 84000 + PRESENCE_QUIET_MS + 1);
  assert(carHome);
  at(base + 84000 + AWAY_MIN_MS);
  assert(sysState == SysState::Armed);
  assert(carHome); // Silence at home is not a departure.
  const auto wake = clockMs + 1;
  const int8_t ramp[] = {-99, -96, -85, -93, -89, -101, -88, -84};
  for (unsigned i = 0; i < 8; ++i) sight(wake + i * 1000, ramp[i]);
  assert(firedCount == 1); // Even a strict-looking parked wake cannot repeat.
  at(clockMs + PRESENCE_QUIET_MS + 1);
  assert(carHome);
}
static void sameSessionCannotUndoArrival() {
  armAway();
  observedArrival();
  // Synthetic adverse shape: a late strong peak must not undo acceptance.
  sight(base + 45000, -70);
  sight(base + 46000, -92);
  at(base + 46000 + PRESENCE_QUIET_MS + 1);
  assert(carHome);
}
static void departureThenNextArrival() {
  armAway();
  observedArrival();
  sight(base + 6000, -77);
  sight(base + 60000, -92);
  at(base + 60000 + AWAY_MIN_MS);
  const auto departure = clockMs + 1;
  // Representative samples with the observed departure's 224s/5s shape.
  for (unsigned ms = 0; ms < 224000; ms += 30000)
    sight(departure + ms, -92);
  sight(departure + 224000, -65);
  sight(departure + 229000, -92);
  at(departure + 229000 + PRESENCE_QUIET_MS + 1);
  assert(!carHome);
  assert(firedCount == 1); // No opening while departing from HOME.
  at(departure + 229000 + AWAY_MIN_MS);
  assert(sysState == SysState::Armed);
  const auto arrival = clockMs + 1;
  const int8_t ramp[] = {-99, -96, -85, -93, -89, -101, -88, -84};
  for (unsigned i = 0; i < 8; ++i) sight(arrival + i * 1000, ramp[i]);
  assert(firedCount == 2);
  assert(carHome);
}
static void unacceptedSessions() {
  reset();
  at(base);
  assert(carHome && firedCount == 0);
  sight(base + 1, -85);
  sight(base + 1000, -83);
  sight(base + 2000, -84);
  at(base + 2000 + PRESENCE_QUIET_MS + 1);
  assert(carHome && firedCount == 0);
  armAway();
  sight(base + 1, -94);
  sight(base + 2000, -90);
  at(base + 2000 + PRESENCE_QUIET_MS + 1);
  assert(!carHome && firedCount == 0);
}
int main() {
  arrivalAndParkedWake();
  sameSessionCannotUndoArrival();
  departureThenNextArrival();
  unacceptedSessions();
#if PULSE_ENABLED
  for (auto mode : {RunMode::Disabled, RunMode::DryRun, RunMode::Live}) {
    armAway();
    runMode = mode;
    observedArrival();
    const unsigned expected = mode == RunMode::Live ? 1 : 0;
    assert(pulseCount == expected && gpioHighWrites == expected);
    clockMs += PULSE_MS;
    pulseTick();
    assert(!pulseActive && outputLevel == LOW);
  }
#endif
  std::cout << "Presence regressions passed (PULSE_ENABLED=" << PULSE_ENABLED << ")\n";
}
