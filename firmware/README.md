# Bluedoor firmware

One PlatformIO project, two source files, five build targets.

- [src/main.cpp](src/main.cpp): the scanner. Radio layer, arrival state
  machine, status web page, log, OTA, and (in production builds) the pulse
  output.
- [src/beacon.cpp](src/beacon.cpp): the in-car beacon. BLE advertising only.

| Target | Radio | Pulse | Use it for |
|---|---|---|---|
| `beacon` | advertises | – | **The board in the car.** Non-connectable advertising every 100 ms at +9 dBm, name in the packet, burned-in address. No WiFi, no OTA, no flash writes: it tolerates losing power at any moment. |
| `logger-ble` | BLE scan | never | **Listen-only scanner.** Full arrival logic, log and status page, no pin ever driven. Use it to find the mounting spot and to validate the logic before wiring anything. |
| `v1-ble` | BLE scan | yes | **Production scanner.** `logger-ble` plus run modes, token-gated controls, the GPIO 26 pulse and the post-pulse lockout. Boots in DRY-RUN. |
| `survey` | BLE + classic | never | Attended census of everything on both radios. Runs the WiFi + classic-inquiry mix that crashes the controller after a few hours, so never leave it running. |
| `logger` / `v1` | classic inquiry | never / yes | The original design, watching a car head unit's classic Bluetooth instead of a beacon. Kept for reference; see the design notes in the top-level README for why it does not work. |

Board: any ESP32-WROOM-32 dev board (tested on the Freenove FNK0090).

## Configure

```bash
cp include/config.example.h include/config.h
```

`config.h` is gitignored. Fill in:

| Field | Meaning |
|---|---|
| `WIFI_SSID`, `WIFI_PASS` | your LAN, for the status page and OTA |
| `BEACON_BLE_MAC` | the beacon's address, printed on its serial banner at boot |
| `OTA_PASSWORD` | required for over-the-air reflashing; blank or the placeholder keeps OTA off |
| `CONTROL_TOKEN` | required by the mode switch and manual pulse; controls stay refused while it is the placeholder |
| `DEVICE_HOSTNAME` | mDNS name, default `bluedoor` → `http://bluedoor.local` |
| `TZ_INFO` | POSIX timezone string for log timestamps |
| `SURVEY_IGNORE_MACS` | optional: fixed advertisers in your house to drop from the census |
| `CAR_BT_MAC` | only for the classic-radio targets |

Thresholds and timings live in [include/tunables.h](include/tunables.h),
which is committed and commented.

## Build, flash, watch

```bash
pio run -e beacon -t upload          # the car board, over USB
```
```bash
pio run -e logger-ble -t upload      # the garage board, first flash over USB
```
```bash
pio device monitor                   # serial log at 115200
```
```bash
pio run -e v1-ble -t upload --upload-port bluedoor.local   # later flashes, over the air
```

USB permission error on Linux: add yourself to `dialout` and log in again.

OTA at the edge of WiFi range can take a few attempts; a failed OTA is
harmless, the old firmware keeps running.

## Status page

`http://bluedoor.local`, auto-refreshing: run mode and state, counters,
last sighting, the active tunables, and the tail of the event log. Full log
at `/log` (rotates at 48 KB into `/log.old`; both survive reboots).

Controls, each requiring `CONTROL_TOKEN`:

- **Mark in log** with a free-text note: ground truth ("arrived by car",
  "opened the parked car's door") to compare against verdicts.
- **Mode**: `DISABLED` / `DRY-RUN` / `LIVE`, persisted. A reboot never
  escalates the mode. First boot is DRY-RUN.
- **Presence**: diagnostic override to IN GARAGE or OUTSIDE GARAGE. Boot
  location is UNKNOWN; normal operation resolves it from observed movement.
- **Manual pulse** (v1 builds): a single press for commissioning, refused in
  DISABLED. The optional hold field (50 to 20000 ms) is a bench aid: a 3 s hold
  lets you meter the pin, the optocoupler LED and the remote pads stage by
  stage. Manual pulses enter the same lockout as automatic ones.

## How the scanner decides

- **Disarmed at boot** until no sighting for `AWAY_MIN_MS` (10 min).
- **Inferred location**: UNKNOWN, GARAGE, or OUTSIDE GARAGE. OUTSIDE GARAGE
  includes upper-property parking; it does not assert that the car left the
  property. Weak sessions leave location unchanged. Boot is UNKNOWN and blocks
  automatic openings until a movement establishes location; silence does not
  prove the car left. This can miss the first arrival after reboot.
- **Approach gate**: at least eight packets over two seconds, a rise of at
  least 6 dB between the first and last four-packet medians, and both the
  newest sample and recent median at or above −80 dBm. Near reception must
  span 500 ms, resetting on a weak sample/median or a gap over 1500 ms. The
  gate is experimental: upper-parking reception in the captured false opening
  peaked at −88, while a genuine garage approach reached −68. Signal strength
  does not establish direction by itself.
- **Duplicate-command latch**: accepting an approach blocks repeats immediately
  in every run mode, but sets location UNKNOWN rather than claiming garage
  occupancy. After 60 seconds of silence ends the session, a post-peak tail
  of at least 15 seconds, longer than the head and ending below the approach
  threshold, infers GARAGE. An ambiguous accepted session keeps UNKNOWN and
  its latch. The accepted session can never classify itself as a departure.
- **Separate departure**: an unaccepted session needs a peak of at least −75,
  at least 15 seconds before the peak, a head at least 10 seconds longer than
  the tail, a tail at most 15 seconds, and an ending median at or below −85.
  After 60 seconds of silence it can infer OUTSIDE GARAGE and clear the latch.
  A sufficiently long post-peak tail instead infers GARAGE; other strong
  traces are ambiguous. These are route heuristics, not measured door state.
- **Opening eligibility**: only OUTSIDE GARAGE with a clear duplicate latch
  can open. Weak activity in upper parking neither issues a command nor
  consumes a subsequent approach down the ramp. GARAGE and UNKNOWN block
  even a strong approach signature, protecting against departures and wakes.
- **Relaxed rule**, logged only, never fires: two sightings above the trigger
  inside a minute, no ramp required. It exists so you can compare the two
  rules against your Mark lines and decide with data.
- **Wake-in-place refusal**: an encounter that starts already above
  `WAKE_STRONG_DBM` and stays within `WAKE_FLAT_DB` is the parked-car
  signature (someone opened a door and briefly powered the beacon). The whole
  encounter is latched non-fireable.
- **Lockout**: the normal quiet timer remains ten minutes. A separately
  confirmed departure permits a two-minute quiet re-arm, including after an
  automatic command. The duplicate latch cannot expire on silence alone.
  Unconfirmed short trips and a return within the same session can still be
  missed; they do not justify clearing the latch.
- An encounter with no verdict ends after `ENCOUNTER_QUIET_MS` of silence or
  `ENCOUNTER_MAX_MS` total. The scanner then re-arms after `REARM_NOVERDICT_MS`
  (2 min) of silence instead of the full `AWAY_MIN_MS`: a car that waited at
  the top of the ramp and left must not blind the scanner to its return. If
  the beacon is heard at `WAKE_STRONG_DBM` or stronger while disarmed, the car
  came in and parked, and the full absence applies again.

### Pulse safety (v1 builds)

- GPIO 26 is a non-strapping pin, driven LOW first thing in `setup()`. The
  10 kΩ hardware pulldown covers the boot-ROM window before that.
- Every pulse logs a **pad read-back** (the pin's actual level) and a **load
  check**: the pin floats briefly on its weak pull-up and the optocoupler LED
  path must drag it LOW. `NOTHING connected` in the log means a loose jumper
  or an open joint.
- `WEB_CONTROLS_NEED_TOKEN 0` in tunables drops the token requirement for
  bench work. The page shows a red banner while it is off. Put it back before
  the device goes anywhere near the door.
- Rolling-code remotes count presses the door cannot hear. Past a receiver's
  resync window (16 presses on BFT) it wants two consecutive presses to
  resync. Test within range, and never let an arrival press become a double
  press: on step-logic doors the second press is a *stop*.

## Reading the log

| Tag | Meaning |
|---|---|
| `VERDICT WOULD OPEN (strict)` | The firing rule fired. In v1 builds, followed by a `PULSE` line (LIVE) or a suppression line (DRY-RUN / DISABLED). `approach blocked` identifies the inferred location and duplicate latch; a separate `APPROACH` line records the rejected signal signature. |
| `PRESENCE` | Inferred location, command latch, and the reason for a change or refusal. A pulse does not confirm location or door movement. |
| `SESSION` | Completed session count, duration, first/peak/ending RSSI and time before/after the peak. Separate from the decision to avoid truncated evidence. |
| `SIGNAL` | All states: one-second buckets with uptime start, sample span, count, first/last/min/max RSSI and exact bucket median. Replaces armed per-packet flash logging. |
| `RELAXED` | The no-ramp comparison rule would have fired here. Never actuates. |
| `REFUSE` | Wake-in-place signature; the encounter is latched non-fireable. |
| `PULSE` | Actual, suppressed, refused or manual pulses, with the pad read-back and load check. |
| `MODE` | Run-mode changes and the mode restored at boot. |
| `ENC` | Encounter (sighting cluster while armed) started or ended, with RSSI stats and a `trend=` label: approaching / receding / steady. |
| `SIGHT` | Beacon sightings, aggregated per minute while disarmed, each with first→last RSSI and a trend label. A `went quiet` line timestamps when sightings stopped. A rising trail while disarmed (`disarmed ramp`) is logged peak by peak, with the trigger crossing, so an arrival the machine could not act on keeps its timing. |
| `STATE` | DISARMED (boot / seen / lockout) ↔ ARMED transitions with reasons. |
| `SURVEY` | Census (`BLE_SURVEY` on): every `BLE_SURVEY_MS`, one line per distinct advertiser heard, with sighting count, first/best/last RSSI, and the advertised name if any. A rising first→last across consecutive lines is an approach in progress. The `survey` target tags lines `BT` or `BLE`. |
| `HB` | Half-hourly heartbeat: uptime, free heap and largest free block, web requests served, WiFi drops, allocation failures. A gap means a crash or power loss. |
| `RADIO` | Classic-radio targets only: 5-minute census of inquiry results. |
| `PROBE` | Classic-radio targets only: a targeted page to the car for presence without RSSI. |
| `MARK` | Your annotations from the status page. |
| `ERR` | Init problems, refused control attempts (with source IP), and boot-time forensics: the reset reason, and after a watchdog reset the RTC-RAM breadcrumb and abort message from the previous run. |

Serial mirrors everything at 115200. WiFi loss never stops detection.

## Design notes

### Host regression checks

Run `python3 tests/run_presence_tests.py` from `firmware/` (Python 3 and a
C++17-capable `g++` required). The runner compiles the actual state-machine
section of `src/main.cpp` with stubbed clock, logging, NVS and GPIO, for both
logger and production configurations. Tests replay de-identified recorded
weak approaches/departures and strong arrivals/departures, alongside synthetic
parked wakeups, upper parking followed directly by a ramp approach, ambiguous
sessions, short re-arm after confirmed departure, boot UNKNOWN, near-signal
timing and the DISABLED/DRY-RUN/LIVE GPIO gates. Recorded fixtures have only
second-resolution timing; millisecond gate boundaries use synthetic inputs.
These checks do not validate radio reception or physical door movement.

### Runtime

- **BLE passive scanning, continuously**, restarted every
  `BLE_SCAN_RESTART_MS` as a liveness watchdog. The beacon's name rides in the
  advertising packet itself (non-connectable advertising cannot answer scan
  requests), so a passive scanner always sees it.
- **The classic radio is left compiled in but unused** on the BLE targets:
  the Arduino core's prebuilt Bluetooth libraries reject a BLE-only
  controller configuration, so the controller runs dual-mode with inquiry
  simply never started. Continuous classic inquiry alongside WiFi is what
  crashed the controller (an assert in the prebuilt stack under coexistence,
  every one to three hours); with inquiry retired the same board ran a 25 h
  soak clean with flat heap fragmentation.
- **The beacon reads its TX power back** from the controller and prints
  that, rather than the value requested. The link budget rests on that one
  number and a silently clamped radio would otherwise look identical to a
  working one.
- **Crash forensics** are built in because the controller crash was invisible
  from the application side: reset-reason decoding, an abort-message hook
  (`-Wl,--wrap=esp_system_abort`) that saves the message to RTC RAM, task
  breadcrumbs in RTC RAM, core-dump readout, and heap and temperature in the
  heartbeat. `WIFI_DUTY_TEST` in tunables alternates WiFi on and off inside
  one run so radio-quiet and WiFi-up windows can be compared directly.
- **Partition scheme `min_spiffs`**: two app slots so OTA stays possible, and
  a 128 KB LittleFS with traffic-dependent retention across the two rotation
  files.
- The scanner itself is non-connectable and non-discoverable. Outside the
  census, other devices' addresses are never logged.
- `soak/capture.sh` captures the serial port to a timestamped file across USB
  re-enumerations, for overnight soak tests.
