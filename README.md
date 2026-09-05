# Bluedoor

**Open the garage when the car arrives, without touching the garage.**

A two-ESP32 system that recognises the *car itself* (not a phone) driving up,
and presses the button of a genuine, already-enrolled garage remote through an
optocoupler. The door controller is never opened, rewired, or re-programmed:
it only ever sees a normal rolling-code press from its own remote. Total spend
under €45.

This README is the full project report: the goal, the design, every
experiment that shaped it (including the one that failed), the safety
reasoning, and the current status. Companion documents:

| Document | What it is |
|---|---|
| [COMPONENTS.md](COMPONENTS.md) | Bill of materials and the wiring map |
| [wiring-v1.html](wiring-v1.html) | Visual build guide: named parts, PC817 leg identification, resistor stripes, pad polarity measurement, build order. Open it in a browser. |
| [firmware/README.md](firmware/README.md) | Build targets, flashing, the log format, design notes |
| [firmware/](firmware/) | PlatformIO project: scanner (`main.cpp`) and in-car beacon (`beacon.cpp`) |

**Status (2026-09-05):** the electronics are proven end to end. A 250 ms pulse
from the ESP32 opens the door every time from beside it. The in-car beacon is
commissioned on the bench. What remains is the field week: beacon mounted in
the car, scanner in the garage in dry-run, arrival logic checked against real
arrivals before the pulse is enabled.

---

## 1. Goal and constraints

Recognise the car arriving home and open the existing garage door, subject to:

- **The door system stays 100 % untouched.** No wires into the controller, no
  new remotes enrolled, no firmware on the operator. Its remotes and the
  physical key keep working exactly as before.
- **Recognise the car, not the driver.** Several people drive it, some with
  phones paired to the car and some without. A phone-based trigger would work
  for one driver only.
- **Minimal spend.** Realistic budget €25 to €45, bought in stages so that a
  failed experiment costs as little as possible.

### What was ruled out, and why

| Option | Why not |
|---|---|
| Off-the-shelf smart openers (Tailwind iQ3, Meross MSG100, Remootio 3, BFT B-EBA gateway) | Every one recognises the *phone*, and most wire into the operator's control board. The two things the goal forbids. Prices €40 to €130. |
| Phone geofencing | Cheapest of all, but recognises the driver, not the car. |
| Camera + licence-plate recognition | Genuinely recognises the car, but needs a Pi or laptop plus camera (€50+), night lighting, and is fooled by a printed plate. |
| Cloning or replaying the remote's 433 MHz signal | BFT rolling code (KeeLoq-style). Replay is useless. |
| Emulating a new 433 MHz remote (ESP32 + CC1101, enrolled in learn mode) | Technically real, but enrolling a synthetic remote *writes to the door base*, which is no longer "untouched". It also needs a sub-GHz radio the ESP32 lacks. A €1 optocoupler pressing an already-paired genuine remote changes nothing in the receiver. |
| The car's own Bluetooth as the trigger | This was the original plan. It failed in the field. Section 3 explains. |

---

## 2. How it works

```
[Car arrives: BLE beacon on switched USB, +9 dBm, fixed MAC]
                 │
                 ▼
[Scanner ESP32 in the garage: passive BLE scan, MAC filter, RSSI trail]
                 │
                 ▼
[Arrival state machine: away ≥ 10 min, then a rising approach ramp]
                 │  strict verdict only, run-mode gate, post-pulse lockout
                 ▼
[GPIO 26 ── 330 Ω ── PC817 optocoupler ── across the remote's button pads]
                 │
                 ▼
[Sacrificial BFT MITTO 12V-UP remote, own 12 V battery]
                 │  a legitimate rolling-code press
                 ▼
             door opens
```

Two ESP32 boards:

- **The beacon** rides in the car, powered from the car's USB socket. It
  advertises its burned-in Bluetooth address and a name every 100 ms at
  +9 dBm, non-connectable. No WiFi, no flash writes, nothing that needs a
  clean shutdown, because the car cuts USB power about 10 to 15 s after
  ignition off.
- **The scanner** sits in the garage on a phone charger. It scans passively
  for that one address, feeds the RSSI trail into a state machine, serves a
  status page on the LAN, and drives the optocoupler.

The remote is a genuine BFT remote already enrolled on the door. The
optocoupler's output transistor sits across the pads of its button, so the
ESP32 "presses" it with full galvanic isolation. The remote keeps its own
battery. The door cannot tell the difference from a finger.

### The one thing a remote press cannot tell you

A BFT remote press is a *step* command, not an *open* command. The door's
logic was confirmed by hand on 2026-09-03: closed → opens, open → closes,
moving → stops, stopped mid-travel → reverses. A stray pulse on an open door
closes it, and one during travel stops it.

Two ways to handle this. **v1 (built)** has no door sensor and relies on
compensating controls: the system only fires on a confirmed arrival, when the
driver is present, watching, with a remote in hand. Plus boot-disarm,
approach evidence, cooldown, and a lockout after every pulse. The accepted
residual failure: if the door was left open when the car arrives, the pulse
closes or stops it. Annoying, not dangerous. The operator's own obstacle
detection remains active throughout. **v2 (optional)** adds a wired reed
contact as a hard interlock: never pulse unless the door is confirmed closed.
The firmware keeps that input stubbed so it is a plug-in upgrade.

---

## 3. The experiments

Everything below was measured, not assumed. Dates are when the measurement
was made.

### 3.1 Phase 0: is the car visible at all? (2026-08-22)

A phone running a Bluetooth scanner app, next to the car:

- The head unit ("Uconnect", ALPS Alpine module) is **classic Bluetooth
  (BR/EDR) only**. No BLE at all.
- Generally discoverable, RSSI −42 dBm at the car and −78 dBm at distance.
- It wakes and broadcasts as soon as a car door opens, before ignition.
- Still discoverable with ignition on and a paired phone streaming audio
  (−55 dBm from an unpaired scanner). An active A2DP link does not stop it
  answering inquiry.
- Classic Bluetooth has no address randomisation, so the address is stable.

Feasibility passed at phone level. That is a weaker claim than it sounds, and
the next phase was designed to test it properly before any more money was
spent.

### 3.2 Phase 0.5: the one-week logger. Failed. (2026-08-27)

The first ESP32 ran the *full* arrival pipeline and state machine for a week,
but could only log. Every sighting, every state transition, and every "would
open" verdict went to flash and a status page, with a button to mark real
arrivals as ground truth.

Three marked arrivals produced **zero** strict verdicts. Every encounter was
flat. The one relaxed-rule fire was on a *departure*: a false open.

The controlled test that settled it: a phone rode in the car as a positive
control on a drive out and back. In the five-minute window containing the
whole drive, the phone was heard 523 times and the car zero times.

| Car state | Heard by the ESP32 |
|---|---|
| Parked, radio on, someone inside | Yes, about 78 reports per 5 min at −75 dBm |
| Driving, ignition on | **No. Zero.** |
| Parked, ignition off, empty | No |

The car is visible only in the state that must *never* fire (someone sitting
in the parked car) and invisible in the one that must (arriving). An exact
inversion. Two supporting numbers made it worse:

- The head unit is a weak emitter. A phone on the driver's seat reads −38 to
  −41 dBm; the head unit in the same car reads −66 to −84. About 35 dB of
  headroom simply is not there.
- The wake-in-place refusal was dead code. Its threshold was −60 dBm and the
  car never exceeded −66, while noise at the detection floor (an 18 dB swing
  inside one encounter) looked convincingly like an approach ramp. In a live
  build that opens the door on someone sitting in the parked car.

**Decision:** keep the whole pipeline, replace the signal. A BLE beacon
carried in the car gives a strong, always-on, fixed-address signal the ramp
logic can actually use.

### 3.3 Mounting-spot RF survey (2026-08-27)

Walk test with a phone as the source, scanner inside near the closed door:

| Position | Best RSSI | Reports per 30 s |
|---|---|---|
| Top of the ramp | nothing (one −93 dBm blip) | 0 to 1 |
| Middle of the ramp | −77 to −81 dBm | 19 to 33 |
| In front of the door | −61 to −66 dBm | 63 to 186 |

A monotonic ramp of about 18 dB. Exactly the signature the state machine
wants, and proof the building is not the obstacle. The closed door itself
costs about 20 dB.

### 3.4 The reset mystery (2026-08-28 to 29)

The logger rebooted every one to three hours. The firmware grew a full
forensics kit to find out why: reset-reason decoding, RTC-RAM breadcrumbs
that survive a panic, core-dump readout, minimum-heap and largest-free-block
tracking, die temperature, an HTTP request counter, and a WiFi on/off duty
test to compare radio-quiet windows with WiFi-up windows inside one run.

The application was innocent every time: heap flat at 28 to 32 KB, temperature
flat, no requests served. The crashes were a **controller assert in the
prebuilt classic-Bluetooth stack** (`ASSERT_PARAM rwbt.c:393`) and interrupt
watchdog stalls inside its level-4 interrupt, under continuous inquiry with
WiFi coexistence. Desk conditions reproduced it, which ruled out the
environment.

Retiring continuous inquiry is the fix. The BLE build does exactly that, and
ran **25 h clean** in the same spot (versus four crashes in 9.5 h for classic),
with heap fragmentation flat. The classic radio is left compiled in but
unused, because the Arduino core's prebuilt BT libraries refuse a BLE-only
controller configuration.

### 3.5 In-car range test: the car body is a 20 dB wall (2026-08-29)

A phone advertising at +1 dBm rode in the car at the planned beacon position.
The scanner sat at a window at the bottom of the ramp, logging every
advertiser in 5 s buckets. One full drive-out, wait, drive-in cycle:

| Phase | Phone-in-car RSSI |
|---|---|
| In the garage, through the wall | −90 to −95, dense |
| Closest pass, exiting at the door | −79 to −83 |
| Away, a short distance from the house | silent for 66 s |
| Return descent, door wait, park | −84 to −88 |

The same phone in a pocket, walked past the same window, peaked at −64. The
**car body costs about 20 dB**, on top of the 20 dB the closed door was
already measured to cost. Consequences:

- An in-car transmitter at phone power never crosses the original −80 dBm
  trigger. The trigger for the beacon build moved to about **−90 dBm**, with
  the rising-ramp signature doing the real work. Identity comes from the
  fixed address, so a loose threshold cannot let another device false-trigger.
- At +9 dBm the beacon should read about −84 in the garage and −77 to −80 at
  the door. Opening during the descent is solid; street-range detection is
  borderline and is what the field week must answer.
- The timing budget is tight: descent about 3 s, then 4 to 10 s waiting at
  the door. Every second of street-range audibility is a second less waiting.

The same session re-ran the classic radio alongside BLE during a real
arrival. It heard nothing from the car at the best mounting spot it will ever
have. The Phase 0.5 verdict stood.

### 3.6 The beacon: a second ESP32 (2026-08-29 to 30)

AirTag, Tile and SmartTag were rejected because they rotate their addresses.
The beacon has to be something with a burned-in public address, and a second
ESP32 board costs €13. Verified on the bench:

- +9 dBm confirmed by **reading the TX power back from the controller**, not
  by trusting the value requested. The whole link budget rests on that number
  and a silently clamped radio looks identical to a working one otherwise.
- Name carried in the advertising packet itself, not a scan response. The
  advertising type is non-connectable and cannot answer scan requests, so the
  passive-scanning logger is guaranteed to see the name.
- **1.0 s from cold reset to the first advertisement heard** by an outside
  scanner. That number matters because the car's USB power comes and goes.

The car's USB is **switched**: about 10 to 15 s of grace after ignition off,
and a door opening re-powers it until about 10 s after the door closes. So a
parked car is a *silent* beacon, which turns out to be a feature. "Away for
10 minutes" arms the system whether the car is gone or garaged. The
wake-in-place refusal's one remaining job is the door-open blip: someone
rummaging in the parked car powers the beacon for 10 to 20 s of strong, flat
signal, which must be refused. Its threshold moved to −85 dBm for the beacon
build so it can actually fire.

### 3.7 The remote and the pulse circuit (2026-09-01 to 05)

The sacrificial remote is a 2010-era BFT MITTO 12V-UP: 433.92 MHz SAW
resonator, a single 12 V cylindrical cell, and four surface-mount tact
switches with large, easy pads. Measured before soldering: the enrolled
switch has five pads, two per contact group plus the metal shell. With the
battery in, one group sits at about +12 V relative to the other, which fixes
the optocoupler polarity: collector to the positive pad, emitter to the
negative. The optocoupler isolates the two circuits completely; the remote
runs above 3.3 V and is never powered from the ESP32.

The pulse circuit (PC817, 330 Ω series resistor, 10 kΩ pulldown) was soldered
dead-bug onto flying leads and bench-verified: about 6 mA through the LED,
0.08 V across the output in diode mode, which is saturated, "button pressed".

**First door opening from the ESP32: 2026-09-05.** A manual pulse from the
web page lit the remote's LED and opened the door. Getting there took a
stage-by-stage bench debug down to one open solder joint on the pad-4 lead,
after a splayed jumper contact had spent an hour faking a dead GPIO pin. Once
re-soldered, four plain 250 ms pulses beside the door opened it four times.

One lesson from that afternoon is baked into the firmware: pulses the door
cannot hear still advance the remote's rolling-code counter, and past 16
unheard presses the receiver wants two consecutive presses to resync. So
test pulses only within range, and never let an arrival pulse become a double
pulse, because BFT step logic makes the second press a *stop*. Resync is a
manual action only.

---

## 4. The arrival state machine

The rules, all tunable in [tunables.h](firmware/include/tunables.h):

- **Boot disarmed.** After any power-up or reset, nothing can fire until a
  confirmed away period (no sightings for 10 minutes) has been observed. The
  first sighting after boot is never an arrival.
- **Arrival = absence, then a ramp.** At least 10 minutes without a sighting,
  followed by an approach signature: three or more sightings with the RSSI
  rising at least 6 dB and crossing the trigger threshold (−90 dBm on the
  beacon build).
- **Wake-in-place refusal.** An encounter that starts already strong and stays
  flat (within 4 dB) is the parked-car signature. It latches the whole
  encounter as non-fireable and is logged as an explicit `REFUSE`.
- **A relaxed rule runs in parallel**, only for logging: two sightings above
  the threshold within a minute, no ramp required. Its purpose is to compare
  the two rules against ground truth, so the choice is made by data.
- **Post-pulse lockout.** After any pulse, automatic or manual, the door
  state is unknown. No further pulses until a full away period and a fresh
  arrival.
- **Arrival only.** Departures use the normal remote in the car, by design.
  Engine start inside the closed garage is a sustained signal with no
  preceding absence and does not fire.

Every rule was chosen against a concrete failure mode seen or anticipated in
the logs: reboots, someone opening the parked car, a neighbour's device, and
the drive-away that fooled the relaxed rule in week one.

---

## 5. Safety design

This opens a house. The firmware treats it like a lock.

- **Run modes** `DISABLED`, `DRY-RUN`, `LIVE`, persisted in flash. First boot
  is `DRY-RUN`, where verdicts log "pulse suppressed" and nothing moves. A
  reboot never escalates the mode.
- **Dry-run week first.** The armed system runs with the pulse disabled in
  software until its log is clean against real arrivals.
- **Boot safety on the pulse pin.** GPIO 26 is a non-strapping pin, driven
  low first thing in setup, with a 10 kΩ hardware pulldown covering the
  boot-ROM window. A boot, reset, or flash cycle can never press the button.
- **Every pulse self-checks.** It logs a pad read-back and a load check (the
  pin floats on its weak pull-up for a moment and the optocoupler LED must
  drag it low), so a loose jumper or an open joint is visible in the log.
- **Web controls are token-gated** and LAN-only. Controls stay refused while
  the token in `config.h` is still the placeholder, and the page shows a red
  banner if the token check is compiled out for bench work.
- **OTA updates need a real password** or stay disabled. A blank or
  placeholder password is logged as an error at boot.
- **Detection never depends on WiFi.** WiFi loss stops the status page, not
  the state machine.
- **The physical remotes and the key are untouched** and remain the fallback.

---

## 6. Hardware

Full list with quantities, prices and the wiring map in
[COMPONENTS.md](COMPONENTS.md). The visual guide in
[wiring-v1.html](wiring-v1.html) is meant to be opened at the bench.

| Part | Role | Approx. |
|---|---|---|
| ESP32-WROOM-32E dev board (Freenove FNK0090) × 2 | scanner + beacon | €13 each |
| PC817 optocoupler | isolated "finger" on the remote button | €1 |
| 330 Ω and 10 kΩ resistors | LED series, GPIO pulldown | pennies |
| BFT MITTO 12V-UP remote | already owned, sacrificed to the project | €0 |
| Replacement daily-carry remote (BFT MITTO COOL C2) | the one purchase that replaces something | €15 to €25 |
| Hookup wire, heat-shrink, jumpers, a box | assembly | €5 |

The board must be an original ESP32 with a classic Bluetooth radio for the
Phase 0 experiments. The C3, S3, C6 and H2 variants are BLE-only. In
hindsight, since the final design is BLE-only, a BLE-only board would work
for the beacon.

---

## 7. Firmware

One PlatformIO project, five build targets. See
[firmware/README.md](firmware/README.md) for the log format and design notes.

| Target | Purpose |
|---|---|
| `logger` | Phase 0.5 would-open logger on the classic radio. Log only, no GPIO. |
| `logger-ble` | Same logger, watching the BLE beacon. The current field build. |
| `survey` | BLE census plus classic inquiry, for attended drive-through tests. Runs the coexistence mix that crashes the controller; never left unattended. |
| `v1` | Production. Same pipeline plus run modes, token-gated web controls, the GPIO 26 pulse, and the post-pulse lockout. |
| `beacon` | The second board, carried in the car. |

Quick start:

```bash
cp firmware/include/config.example.h firmware/include/config.h   # then fill it in
```
```bash
cd firmware && pio run -e beacon -t upload      # beacon board; note the MAC it prints
```
```bash
cd firmware && pio run -e logger-ble -t upload  # scanner board
```

The status page is served at `http://bluedoor.local` on the LAN. After the
first USB flash, later builds go over the air with `--upload-port
bluedoor.local`, which is how the logger box becomes the production box
without leaving its mounting spot.

The census mode (`BLE_SURVEY` in tunables) logs every advertiser it hears with
an RSSI trail, which made all the range tests reflash-free: drive up with any
advertiser and read its trail out of the log.

---

## 8. Security and privacy

- A Bluetooth address is spoofable by someone who deliberately sniffs the
  car. This is **convenience-lock grade**. The absence-then-ramp requirement
  defeats naive replay and wake-in-place tricks; it does not defeat a
  determined attacker with a radio. Free hardening options if ever wanted:
  AND a second signal such as a phone geofence, or an authenticated
  connection to the beacon.
- The scanner is non-connectable and non-discoverable. Other devices'
  addresses are never logged outside the explicit census mode, only anonymous
  counts.
- **Nothing identifying is in this repository.** Addresses, WiFi
  credentials and tokens live in the gitignored `config.h`; the example file
  holds placeholders only. Site photos and the private planning notes are
  excluded, and the history was rewritten before publication. Do not publish
  your own copy of `config.h`, and strip GPS EXIF from any photo you add.

---

## 9. Lessons

- **Test at the level that will run in production, early.** Phone-level
  discoverability said yes; the ESP32 said no. The one-week logger cost
  nothing but a board that was needed anyway, and it stopped the project from
  buying parts for a design that could not work.
- **Log what the radio heard, not just what matched.** The census lines
  ("heard nothing" versus "was not listening") were what turned a silent log
  into a diagnosis.
- **Read hardware settings back.** The beacon's TX power is confirmed from the
  controller, not from the value asked for.
- **A stable prebuilt binary blob is still a binary blob.** The controller
  crash was invisible from the application side; reset-reason forensics and
  breadcrumbs in RTC RAM were what pointed at it.
- **Bench debugging is mostly connectors.** A splayed jumper contact cost an
  hour by faking a dead pin. Flux crust under the probe tips made good joints
  read open. Every pulse now logs its own electrical self-check.
- **Rolling codes count presses you cannot hear.** Test within range.

---

## 10. Status and next steps

Done:

- Car found and characterised; the car's own radio ruled out by measurement.
- Scanner stable (25 h soak) on the BLE build.
- Beacon commissioned: +9 dBm confirmed, 1.0 s cold start, fixed address.
- Remote mapped, pulse circuit built, **door opens on a 250 ms pulse**.
- Door step logic confirmed in every state.

Next:

1. Box the scanner and mount it in the garage.
2. Beacon in the car on switched USB; scanner on `logger-ble` in dry-run.
3. A week of real arrivals against the log. Street-range audibility at +9 dBm
   decides whether the door is fully open on arrival or still opening during
   the descent.
4. Enroll the replacement daily-carry remote, then move the sacrificial
   remote permanently into the box.
5. Flip to `LIVE` only when the dry-run log is clean.
6. v2, only if v1 annoys: wired reed contact as a hard "door closed"
   interlock (GPIO 27, already stubbed).

---

## Licence

MIT. See [LICENSE](LICENSE).
