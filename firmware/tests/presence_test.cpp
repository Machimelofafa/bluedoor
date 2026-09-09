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
  enc = {}; agg = {}; ses = {}; signalBucket = {};
  carHome = true;
  presenceKnown = false;
  arrivalLatched = departureConfirmed = false;
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
#include "recorded_sightings.h"

template <size_t N>
static void replay(uint32_t start, const RecordedSight (&packets)[N]) {
  for (auto p : packets) sight(start + p.ms, p.rssi);
}
static void qualifiedApproach(uint32_t start) {
  // Synthetic timestamps exercise the 500ms near gate independently of the
  // recorded fixtures' one-second timestamp precision.
  const int8_t values[] = {-98, -96, -97, -95, -78, -76, -74, -73, -75, -74, -73};
  const uint32_t offsets[] = {0,100,200,300,2000,2100,2200,2300,2500,2700,2900};
  for (unsigned i = 0; i < 11; ++i) sight(start + offsets[i], values[i]);
}
static void observedArrival() {
  qualifiedApproach(base);
  assert(firedCount == 1);
  assert(arrivalLatched && !presenceKnown); // A command is not garage evidence.
}
static void finishPark(uint32_t start) {
  for (unsigned ms = 10000; ms <= 80000; ms += 10000) sight(start + ms, -92);
  at(start + 80000 + PRESENCE_QUIET_MS + 1);
  assert(presenceKnown && carHome && arrivalLatched);
}
static void arrivalAndParkedWake() {
  armAway(); observedArrival(); finishPark(base);
  at(base + 80000 + AWAY_MIN_MS);
  assert(sysState == SysState::Armed);
  const auto wake = clockMs + 1;
  qualifiedApproach(wake);
  assert(firedCount == 1); // Even a strong parked wake cannot repeat.
  at(clockMs + PRESENCE_QUIET_MS + 1);
  assert(carHome && arrivalLatched);
}
static void acceptedBelowTransitStillConfirmsPark() {
  armAway();
  const int8_t values[] = {-98,-97,-96,-95,-79,-78,-77,-76,-77,-77};
  const uint32_t offsets[] = {0,100,200,300,2000,2100,2200,2300,2600,2800};
  for (unsigned i=0;i<10;++i) sight(base + offsets[i], values[i]);
  assert(firedCount == 1 && !presenceKnown && arrivalLatched);
  assert(ses.peakRssi < PRESENCE_TRANSIT_DBM);
  finishPark(base);
}
static void sameSessionCannotUndoArrival() {
  armAway(); observedArrival();
  sight(base + 45000, -65);
  sight(base + 46000, -92);
  at(base + 46000 + PRESENCE_QUIET_MS + 1);
  assert(arrivalLatched && !presenceKnown && !departureConfirmed);
  // Silence does not silently release an unconfirmed command.
  at(base + 46000 + AWAY_MIN_MS);
  qualifiedApproach(clockMs + 1);
  assert(firedCount == 1);
}
static void departureThenNextArrival() {
  armAway(); observedArrival(); finishPark(base);
  const auto departure = clockMs + 1; // Still inside post-command lockout.
  replay(departure, strongDeparture);
  const auto last = departure + strongDeparture[sizeof(strongDeparture)/sizeof(*strongDeparture)-1].ms;
  at(last + PRESENCE_QUIET_MS + 1);
  assert(presenceKnown && !carHome && !arrivalLatched && departureConfirmed);
  assert(firedCount == 1);
  at(last + REARM_NOVERDICT_MS - 1);
  assert(sysState != SysState::Armed);
  at(last + REARM_NOVERDICT_MS);
  assert(sysState == SysState::Armed); // Confirmed departure does not need 10min.
  qualifiedApproach(clockMs + 1);
  assert(firedCount == 2 && arrivalLatched);
}
static void recordedUpperParkingThenReturn() {
  // Bootstrap autonomously from UNKNOWN through an observed departure.
  reset(); at(base);
  replay(base, strongDeparture);
  at(clockMs + REARM_NOVERDICT_MS);
  assert(presenceKnown && !carHome && sysState == SysState::Armed);
  replay(clockMs + 1, weakApproach);
  at(clockMs + ENCOUNTER_QUIET_MS + 1);
  at(clockMs + AWAY_MIN_MS);
  assert(!carHome && presenceKnown && !arrivalLatched && firedCount == 0);
  const auto departure = clockMs + 1;
  replay(departure, weakDeparture);
  at(clockMs + ENCOUNTER_QUIET_MS + 1);
  at(clockMs + REARM_NOVERDICT_MS);
  assert(!carHome && presenceKnown && firedCount == 0);
  const auto arrival = clockMs + 1;
  replay(arrival, strongArrival);
  assert(firedCount == 1);
  at(clockMs + PRESENCE_QUIET_MS + 1);
  assert(carHome && presenceKnown && arrivalLatched);
}
static void outsideWaitThenRamp() {
  armAway();
  replay(base, weakApproach);
  // Continue from upper parking without a quiet period or separate encounter.
  qualifiedApproach(clockMs + 1000);
  assert(firedCount == 1);
}
static void ambiguousOrWeakSessionsDoNotClearHome() {
  reset(); setPresence(true, "test garage"); at(base);
  replay(base + 1, weakDeparture);
  at(clockMs + PRESENCE_QUIET_MS + 1);
  assert(carHome && !departureConfirmed);
  const auto start = clockMs + 1;
  sight(start, -92); sight(start + 20000, -70); sight(start + 39000, -92);
  at(clockMs + PRESENCE_QUIET_MS + 1);
  assert(carHome && !departureConfirmed); // Near-equal head/tail is ambiguous.
}
static void bootUnknown() {
  reset(); at(base);
  qualifiedApproach(clockMs + 1);
  assert(!presenceKnown && firedCount == 0);
  // Only subsequently observed location evidence resolves boot uncertainty.
  at(clockMs + PRESENCE_QUIET_MS + 1);
  assert(!presenceKnown && firedCount == 0);
}
static void nearEvidenceMustSpanTime() {
  armAway();
  const int8_t values[] = {-98,-97,-96,-95,-77,-75,-74,-73};
  for (unsigned i=0;i<8;++i) sight(base + (i<4 ? i*100 : 2000+(i-4)*10), values[i]);
  assert(firedCount == 0); // A burst does not qualify.
  sight(base + 2040, -94); // Weak reception resets the hold.
  for (unsigned i=0;i<4;++i) sight(base + 2100+i*10, -74);
  assert(firedCount == 0);
  sight(base + 2590, -74);
  assert(firedCount == 0); // Only 490ms since the weak sample reset the hold.
  sight(base + 2600, -74);
  assert(firedCount == 1);
  armAway();
  for (unsigned i=0;i<8;++i) sight(base + (i<4 ? i*100 : 2000+(i-4)*10), values[i]);
  sight(base + 5000, -74); // Long missing interval cannot count as a hold.
  assert(firedCount == 0);
}
static void departureSignatureCannotOpen() {
  reset(); setPresence(true, "test garage"); at(base);
  replay(base + 1, strongDeparture);
  assert(firedCount == 0);
  at(clockMs + PRESENCE_QUIET_MS + 1);
  assert(!carHome && presenceKnown);
}
int main() {
  arrivalAndParkedWake();
  acceptedBelowTransitStillConfirmsPark();
  sameSessionCannotUndoArrival();
  departureThenNextArrival();
  recordedUpperParkingThenReturn();
  outsideWaitThenRamp();
  ambiguousOrWeakSessionsDoNotClearHome();
  bootUnknown();
  nearEvidenceMustSpanTime();
  departureSignatureCannotOpen();
#if PULSE_ENABLED
  for (auto mode : {RunMode::Disabled, RunMode::DryRun, RunMode::Live}) {
    armAway(); runMode = mode; observedArrival();
    const unsigned expected = mode == RunMode::Live ? 1 : 0;
    assert(pulseCount == expected && gpioHighWrites == expected);
    clockMs += PULSE_MS; pulseTick();
    assert(!pulseActive && outputLevel == LOW);
  }
#endif
  std::cout << "Presence and recorded-route regressions passed (PULSE_ENABLED=" << PULSE_ENABLED << ")\n";
}
