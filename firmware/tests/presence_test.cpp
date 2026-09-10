// Hardware-free replay of the production state machine, not a second model.
#include <cassert>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <iostream>
#include <string>
#include <type_traits>
#include "tunables.h"
#include "journey.h"
#define IRAM_ATTR
#define UCONNECT_AUX (DETECT_BLE && UCONNECT_HINT_ENABLED)

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
static bool pulseTimerReady = true, fakeTimerArmed = false, fakeTimerFails = false;
static uint32_t fakeTimerDurationMs = 0, fakeTimerDeadlineMs = 0;
static bool preparePulseTimer(uint32_t ms) { fakeTimerDurationMs = ms; return pulseTimerReady && !fakeTimerFails; }
static bool startPulseTimer() {
  fakeTimerArmed = true; fakeTimerDeadlineMs = clockMs + fakeTimerDurationMs; return true;
}
static uint32_t pulseMicros() { return clockMs * 1000u; }
static bool pulsePadHigh() { return outputLevel == HIGH; }
static void pulsePinLow() { outputLevel = LOW; }
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
  journey = {};
  radioLossPending = false;
  radioLossMs = 0;
  countsDirty = false;
#if UCONNECT_AUX
  uconnectScanActive = uconnectScanFault = uconnectEverChecked = uconnectCancelRequested = false;
  uconnectScanStartMs = uconnectSessionMs = uconnectLastCheckMs = 0;
  uconnectLastSeenMs = uconnectChecks = 0;
  uconnectLastRssi = 127;
#endif
  carHome = true;
  presenceKnown = false;
  arrivalLatched = departureConfirmed = false;
  firedCount = relaxedCount = refusalCount = totalSightings = 0;
#if PULSE_ENABLED
  pulseCount = gpioHighWrites = 0;
  pulseActive = false;
  pulseCompleted = false;
  fakeTimerArmed = fakeTimerFails = timerSelfCheckPending = false;
  pulseTimerReady = true;
  lastPulseDurationUs = timerSelfCheckUs = 0;
  outputLevel = LOW;
  outputMode = OUTPUT;
  runMode = RunMode::DryRun;
#endif
}
static void advanceClock(uint32_t ms) {
#if PULSE_ENABLED
  if (fakeTimerArmed && (int32_t)(ms - fakeTimerDeadlineMs) >= 0) {
    clockMs = fakeTimerDeadlineMs;
    fakeTimerArmed = false;
    pulseTimeout(nullptr);
  }
#endif
  clockMs = ms;
}
static void at(uint32_t ms) {
  advanceClock(ms);
#if PULSE_ENABLED
  pulseTick();
#endif
  machineTick();
}
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
#include "recorded_journeys.h"

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
  assert(ses.peakRssi < -75); // Moderate reception still supports a passage.
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

template <size_t N>
static void finishRecordedJourney(uint32_t start, const JourneyBucket (&buckets)[N], bool accepted = false) {
  // Exact aggregate replay into the production classifier, NOT reconstructed
  // packet input to the millisecond approach gate (tested separately above).
  journey = {};
  signalBucket = {}; // This aggregate fixture replaces the synthetic approach input.
  ses = {};
  ses.active = true;
  ses.arrivalAccepted = accepted;
  ses.startMs = start;
  for (auto b : buckets) {
    b.startMs += start; b.endMs += start;
    journey.add(b);
    ses.count += b.count;
    ses.lastMs = b.endMs;
    ses.ring[ses.rIdx] = b.median;
    ses.rIdx = (ses.rIdx + 1) % 8;
    if (ses.rN < 8) ++ses.rN;
  }
  lastEvidenceMs = ses.lastMs;
  at(ses.lastMs + PRESENCE_QUIET_MS + 1);
}

static void morningTripsRecoverAutonomously() {
  reset(); setPresence(true, "prior parked state"); arrivalLatched = true;
  finishRecordedJourney(base, morningDeparture);
  assert(journey.tailMs() > 15000); // The former hard cutoff missed this trip.
  assert(!carHome && presenceKnown && departureConfirmed && !arrivalLatched);
  assert(firedCount == 0); // Departure never opens.
  at(lastEvidenceMs + REARM_NOVERDICT_MS);
  const auto arrival = clockMs + 1;
  qualifiedApproach(arrival);
  assert(firedCount == 1);
  finishRecordedJourney(arrival, morningArrival, true);
  assert(carHome && arrivalLatched);
  finishRecordedJourney(clockMs + 1000, shortTripDeparture);
  assert(!carHome && !arrivalLatched && departureConfirmed && firedCount == 1);
  at(lastEvidenceMs + REARM_NOVERDICT_MS);
  const auto secondArrival = clockMs + 1;
  qualifiedApproach(secondArrival);
  assert(firedCount == 2);
  finishRecordedJourney(secondArrival, shortTripArrival, true);
  assert(carHome && arrivalLatched);
}

static void otherRetainedJourneysStayCorrect() {
  reset(); setPresence(true, "parked"); arrivalLatched = true;
  finishRecordedJourney(base, previousDeparture);
  assert(departureConfirmed && !carHome && !arrivalLatched);
  at(lastEvidenceMs + REARM_NOVERDICT_MS);
  qualifiedApproach(clockMs+1);
  finishRecordedJourney(clockMs+1, previousArrival, true);
  assert(carHome && arrivalLatched);
  finishRecordedJourney(clockMs+AWAY_MIN_MS, weakMorningActivity);
  assert(carHome && arrivalLatched && !departureConfirmed);
  finishRecordedJourney(clockMs+AWAY_MIN_MS, parkedDoorWake);
  assert(carHome && arrivalLatched && !departureConfirmed && firedCount == 1);
}

static void fadingTailAndParkedDensity() {
  JourneyEvidence trace;
  for (uint32_t ms = 0; ms < 120000; ms += 1000) trace.add({ms, ms+900, 5, -90});
  trace.add({120000,120900,8,-73});
  assert(trace.passage);
  const auto beforeTail = trace;
  // Longer than 15s and progressively sparse: a departure candidate awaits quiet.
  for (auto ms : {122000u,125000u,130000u,140000u,150000u})
    trace.add({ms,ms+100,1,-96});
  assert(trace.departureCandidate());
  // Identical initial activity, but regular parked reception must not release the latch.
  trace = beforeTail;
  for (uint32_t ms=122000;ms<=150000;ms+=1000) trace.add({ms,ms+500,2,-92});
  assert(!trace.departureCandidate());
  // Even one exceptionally strong late packet cannot replace the sustained passage.
  const auto pass = trace.passEndMs;
  trace.add({151000,151000,1,-50});
  assert(trace.passEndMs == pass);
}

static void delayedEvidenceKeepsReceptionTiming() {
  uint32_t expectedHead = 0, expectedTail = 0;
  for (uint32_t lag : {0u, 1000u, 5000u}) {
    reset(); setPresence(true, "garage"); arrivalLatched = true;
    for (auto p : strongDeparture) {
      advanceClock(base + p.ms + lag);
      onCarSighting(p.rssi, "test", base + p.ms);
    }
    at(base + 64000 + PRESENCE_QUIET_MS + lag + 1);
    assert(!carHome && departureConfirmed && !arrivalLatched && firedCount == 0);
    if (!lag) { expectedHead = journey.headMs(); expectedTail = journey.tailMs(); }
    assert(journey.headMs() == expectedHead && journey.tailMs() == expectedTail);
  }
  armAway();
  const int8_t values[] = {-98,-96,-97,-95,-78,-76,-74,-73,-75,-74,-73};
  const uint32_t times[] = {0,100,200,300,2000,2100,2200,2300,2500,2700,2900};
  for (unsigned i=0;i<11;++i) {
    advanceClock(base + 10000); // One delayed batch; received times remain distinct.
    onCarSighting(values[i], "test", base+times[i]);
  }
  assert(firedCount == 0 && enc.staleFlagged && enc.lastMed >= RSSI_TRIGGER_DBM);
}

static void radioLossCannotInventDeparture() {
  reset(); setPresence(true, "garage"); arrivalLatched = true;
  for (auto p : strongDeparture) {
    if (p.ms == 53000) noteRadioLoss();
    sight(base + p.ms, p.rssi);
  }
  at(clockMs + PRESENCE_QUIET_MS + 1);
  assert(carHome && arrivalLatched && !departureConfirmed);
  armAway(); noteRadioLoss(); qualifiedApproach(base+1);
  assert(firedCount == 0);
  at(clockMs + AWAY_MIN_MS);
  qualifiedApproach(clockMs+1);
  assert(firedCount == 1); // A later intact session recovers automatically.
}

static void readinessReflectsActualGates() {
  reset(); at(base);
  assert(std::string(readinessName()).find("unknown") != std::string::npos);
  setPresence(true, "parked");
  assert(std::string(readinessName()).find("in garage") != std::string::npos);
  setPresence(false, "departure");
#if PULSE_ENABLED
  runMode = RunMode::Live;
  assert(std::string(readinessName()) == "READY TO OPEN ON APPROACH");
  pulseTimerReady = false;
  assert(std::string(readinessName()).find("timer unavailable") != std::string::npos);
#endif
}

#if UCONNECT_AUX
static void uconnectOnlyCorroboratesParkedActivity() {
  reset(); at(base);
  for (uint32_t ms=0;ms<=16000;ms+=1000) sight(base+ms,-92);
  assert(canCheckUconnect(clockMs));
  beginUconnectCheck(clockMs);
  const auto evidence = lastEvidenceMs;
  onUconnectSighting(-76,clockMs+100);
  assert(ses.uconnectSeen && lastEvidenceMs == evidence && firedCount == 0);
  uconnectScanActive = false;
  at(clockMs + PRESENCE_QUIET_MS + 1);
  assert(carHome && presenceKnown && !departureConfirmed);
  // A missing head-unit signal, however long absent, never establishes departure.
  at(clockMs + AWAY_MIN_MS);
  assert(carHome && !departureConfirmed && firedCount == 0);
  // Never start Classic checks during an outside arrival, or after a near passage.
  armAway();
  for (uint32_t ms=0;ms<=16000;ms+=1000) sight(base+ms,-92);
  assert(!canCheckUconnect(clockMs));
  onUconnectSighting(-50,clockMs);
  assert(!ses.uconnectSeen && !carHome);
  qualifiedApproach(clockMs+1);
  assert(firedCount == 1 && !canCheckUconnect(clockMs));
}
#endif

#if PULSE_ENABLED
static void timerReleasesDespiteBlockedMainLoop() {
  reset();
  timerSelfCheckPending = true;
  timerSelfCheckStartUs = pulseMicros();
  assert(preparePulseTimer(250) && startPulseTimer());
  advanceClock(clockMs + 5000);
  assert(!timerSelfCheckPending && timerSelfCheckUs == 250000 && gpioHighWrites == 0 && pulseCount == 0);
  reset();
  assert(doPulse("manual/web"));
  const auto start = clockMs;
  assert(pulseActive && outputLevel == HIGH);
  assert(!doPulse("overlapping")); // Must not extend or duplicate a live pulse.
  advanceClock(start + 5000); // No pulseTick/machineTick/web handler runs.
  assert(!pulseActive && outputLevel == LOW && pulseCompleted);
  assert(pulseEndUs-pulseStartUs == PULSE_MS*1000u && pulseCount == 1);
  pulseTick();
  assert(lastPulseDurationUs == PULSE_MS*1000u);
  reset(); fakeTimerFails = true;
  assert(!doPulse("timer failure") && gpioHighWrites == 0 && pulseCount == 0);
  reset();
  clockMs = UINT32_MAX-100;
  assert(doPulse("rollover"));
  advanceClock(clockMs+PULSE_MS+1);
  pulseTick();
  assert(!pulseActive && outputLevel == LOW && lastPulseDurationUs == PULSE_MS*1000u);
}
#endif

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
  morningTripsRecoverAutonomously();
  otherRetainedJourneysStayCorrect();
  fadingTailAndParkedDensity();
  delayedEvidenceKeepsReceptionTiming();
  radioLossCannotInventDeparture();
  readinessReflectsActualGates();
#if UCONNECT_AUX
  uconnectOnlyCorroboratesParkedActivity();
#endif
#if PULSE_ENABLED
  timerReleasesDespiteBlockedMainLoop();
  for (auto mode : {RunMode::Disabled, RunMode::DryRun, RunMode::Live}) {
    armAway(); runMode = mode; observedArrival();
    const unsigned expected = mode == RunMode::Live ? 1 : 0;
    assert(pulseCount == expected && gpioHighWrites == expected);
    advanceClock(clockMs + PULSE_MS); pulseTick();
    assert(!pulseActive && outputLevel == LOW);
    if (mode == RunMode::Live) assert(lastPulseDurationUs == PULSE_MS * 1000u);
  }
#endif
  std::cout << "Presence, recorded journeys, radio timing and pulse regressions passed (PULSE_ENABLED="
            << PULSE_ENABLED << ", UCONNECT_HINT_ENABLED=" << UCONNECT_HINT_ENABLED << ")\n";
}
