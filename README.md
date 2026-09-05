# Bluedoor

**Your garage opens when your car arrives. The garage is never modified.**

Two ESP32 boards and a €1 optocoupler. One board rides in the car as a
Bluetooth beacon. The other sits in the garage, recognises the beacon
approaching, and presses the button of a spare garage remote that is already
enrolled on the door. The door controller sees a normal rolling-code press
from its own remote; nothing is wired into it, enrolled on it, or changed.

- Recognises the **car**, not a phone. Anyone who drives the car gets the
  door.
- Works with **any garage remote that has a push button**. No RF cloning, no
  rolling-code tricks.
- Under **€45** in parts, most of it the two ESP32 boards.
- Fails safe: boots disarmed, dry-run mode by default, hardware pulldown on
  the pulse pin, lockout after every press.

| Document | What it is |
|---|---|
| [COMPONENTS.md](COMPONENTS.md) | Bill of materials and the wiring map |
| [wiring-v1.html](wiring-v1.html) | Illustrated build guide for the remote side: parts, optocoupler pinout, resistor colours, soldering to the remote's pads. Open in a browser. |
| [firmware/README.md](firmware/README.md) | Build targets, configuration, the status page, the log format, tunables |

---

## What you need

**A garage door with an RF remote** that has a physical push button. The
guide uses a BFT MITTO, but any remote with a button and its own battery
works: the ESP32 only ever closes that button's contacts. You need a remote
you can sacrifice, so plan on buying a new daily-carry remote and dedicating
the old one to the project. Enroll the new one *before* you open the old one.

**A car with a USB socket that switches with the ignition.** The beacon lives
on it. Switched power is what makes a parked car silent, which the arrival
logic relies on. If your car's USB is always on, see "Adapting it" below.

**Parts** (full list in [COMPONENTS.md](COMPONENTS.md)):

| Part | Qty | Role |
|---|---|---|
| ESP32-WROOM-32 dev board | 2 | scanner in the garage, beacon in the car |
| PC817 optocoupler (or EL817 / LTV-817) | 1 | the isolated "finger" on the remote's button |
| 330 Ω resistor | 1 | optocoupler LED series resistor |
| 10 kΩ resistor | 1 | pulldown on the pulse pin (not optional) |
| Thin stranded wire, heat-shrink, a few Dupont jumpers, a small box | | assembly |
| 5 V USB charger and cable | 1 | power for the scanner |

**Tools:** soldering iron with a fine tip, a multimeter (mandatory: you
measure the remote's button polarity before soldering), wire strippers,
isopropyl alcohol.

**Software:** [PlatformIO](https://platformio.org/) (CLI or the VS Code
extension). The firmware targets the Arduino core for ESP32.

---

## How it works

```
 car ──▶ [beacon ESP32 on switched USB: BLE advertising, fixed MAC, +9 dBm]
                          │  radio
                          ▼
 garage  [scanner ESP32: passive BLE scan, filtered to that one MAC]
                          │  RSSI trail
                          ▼
         [arrival logic: away ≥ 10 min, then a rising approach ramp]
                          │  strict verdict, run-mode gate, lockout
                          ▼
         [GPIO 26 ── 330 Ω ── PC817 ── across the remote's button pads]
                          │
                          ▼
         [spare remote, own battery]  ──▶  a normal rolling-code press  ──▶  door
```

The scanner only fires on an **arrival**: the beacon must have been absent
for at least 10 minutes, then heard several times with a rising signal that
crosses a threshold. A parked car is silent (its USB is off), so "absent"
covers both "away" and "in the garage". Someone opening the parked car's door
powers the beacon for a few seconds at a strong, flat level; that signature is
recognised and explicitly refused. After any press the door state is unknown,
so nothing fires again until a full absence and a fresh arrival.

Departures are not automated. You press the remote in the car as before.

**One caveat you must understand before building this.** A remote press is
a *step* command on most garage controllers, not an *open* command: closed →
opens, open → closes, moving → stops. The scanner has no door sensor in this
version, so if the door was left open when the car arrives, the press closes
it. The design compensates by only firing when the driver is present and
watching, with a remote in hand to correct it. If that bothers you, the
firmware has a stubbed input for a wired door-closed contact (see "Adapting
it").

---

## Build

### 1. Flash the beacon

```bash
cd firmware
pio run -e beacon -t upload
pio device monitor
```

The serial banner prints the board's burned-in Bluetooth address and the
transmit power read back from the radio (it should say +9 dBm). Write the
address down: it goes into the scanner's config as `BEACON_BLE_MAC`. Plug the
beacon into the car's USB socket. It needs nothing else: no WiFi, no
configuration, and it tolerates losing power at any moment.

### 2. Configure and flash the scanner

```bash
cp firmware/include/config.example.h firmware/include/config.h
```

Fill in `config.h`: WiFi credentials, the beacon address, an OTA password and
a control token of your choosing. The file is gitignored so nothing personal
leaves your machine. Then:

```bash
cd firmware
pio run -e logger-ble -t upload
```

`logger-ble` is the **listen-only** build: it runs the full arrival logic but
can only log, and it never drives a pin. Open `http://bluedoor.local` on your
LAN to see the status page: state, counters, last sighting, and a tail of the
event log. Later reflashes can go over the air:

```bash
pio run -e logger-ble -t upload --upload-port bluedoor.local
```

### 3. Find the mounting spot

The scanner needs three things: USB power, radio reach to where the car
approaches, and RF reach from the remote to the door. Place it, then drive
out and back in. Every sighting of the beacon is logged with its signal
strength, and the `SURVEY` census lines list every advertiser heard with a
first/best/last RSSI trail, so you can read the whole approach out of the log
without knowing anything in advance.

What you are looking for: as the car approaches, the beacon's RSSI should rise
by 6 dB or more over at least three sightings and end above the trigger
threshold (`RSSI_TRIGGER_DBM`, default −90 dBm for the beacon). If it never
crosses the threshold, move the scanner closer to the approach path or loosen
the threshold. Expect a closed garage door and the car's body to each cost
around 20 dB.

Leave it running for a few days. Every real arrival should produce a
`VERDICT WOULD OPEN (strict)` line; nothing else should. Use the **Mark**
button on the status page to annotate real arrivals so you can compare.

### 4. Wire the remote

Follow [wiring-v1.html](wiring-v1.html). The short version:

1. Enroll your new daily-carry remote on the door first. The old remote is
   often the programming key, and it has to survive until this is done.
2. Open the old remote. Find the button that operates the door. With the
   multimeter, identify its two contact pads and which one is positive
   relative to the other with the battery in.
3. Solder two thin leads to those pads. Keep the remote's own battery.
4. Build the small assembly: GPIO 26 → 330 Ω → PC817 pin 1; PC817 pin 2 → GND;
   10 kΩ between GPIO 26 and GND; PC817 pin 4 (collector) → the positive pad;
   PC817 pin 3 (emitter) → the negative pad. The output side is directional:
   backwards, nothing burns but the button never presses.
5. Push the two jumpers onto the ESP32's GPIO 26 and GND header pins. Nothing
   is soldered to the ESP32 itself.

The 10 kΩ pulldown is not optional. Without it the pin floats during boot,
and a power cut could open your garage on its own when the power returns.

### 5. Commission

```bash
cd firmware
pio run -e v1-ble -t upload --upload-port bluedoor.local
```

`v1-ble` is the production build. It boots in **DRY-RUN**: arrival verdicts
are logged as "pulse suppressed" and the pin is never driven. Three modes,
switchable on the status page with your control token:

| Mode | Behaviour |
|---|---|
| `DISABLED` | never pulses, not even manually |
| `DRY-RUN` | full logic, pulse suppressed and logged (default on first boot) |
| `LIVE` | pulses on a strict arrival verdict |

1. Stand at the door with a working remote in hand and use the **manual
   pulse** button on the status page. The door should respond to a 250 ms
   press. Ignores it: lengthen `PULSE_MS`. Double-triggers: shorten it.
   Every pulse logs an electrical self-check of the pin and the optocoupler
   path, which is the first place to look if nothing happens.
2. Leave it in DRY-RUN for a week. Check every suppressed verdict against
   reality.
3. When the log is clean, switch to LIVE.

Test pulses only within range of the door. Presses the door cannot hear still
advance the remote's rolling-code counter, and after enough of them the
receiver wants two consecutive presses to resync.

---

## Tuning

Everything lives in [firmware/include/tunables.h](firmware/include/tunables.h)
and is documented there. The ones that matter:

| Tunable | Default | What it does |
|---|---|---|
| `AWAY_MIN_MS` | 10 min | absence required before the system arms |
| `RSSI_TRIGGER_DBM` | −90 (beacon) | the approach must cross this level |
| `APPROACH_MIN_SIGHTINGS` / `APPROACH_MIN_RISE_DB` | 3 / 6 dB | the "rising ramp" definition |
| `WAKE_STRONG_DBM` / `WAKE_FLAT_DB` | −85 / 4 dB | the "parked car woken in place" signature, refused |
| `PULSE_MS` | 250 ms | button press length |
| `BLE_SURVEY` | 1 | log every advertiser heard (turn off once commissioned) |

Reading the log: `SIGHT` lines are beacon sightings with a trend label,
`ENC` lines summarise an encounter while armed, `VERDICT` / `RELAXED` /
`REFUSE` are the three outcomes, `PULSE` and `MODE` are the actuator, `STATE`
is the arm/disarm machine, `HB` is a half-hourly heartbeat. Full table in
[firmware/README.md](firmware/README.md).

---

## Safety and security

- **Boot safety.** GPIO 26 is a non-strapping pin, driven low first thing at
  boot, with the hardware pulldown covering the boot-ROM window.
- **Disarmed after every reset** until a full absence has been observed.
- **Lockout after every press**, manual or automatic.
- **Web controls need a token** and are LAN-only. They stay refused while the
  token in `config.h` is the placeholder. OTA needs a real password or stays
  off.
- **Detection never depends on WiFi.** Losing WiFi loses the status page, not
  the arrival logic.
- **This is convenience-lock grade.** A Bluetooth address can be spoofed by
  someone who deliberately sniffs your car and then fakes an approach after a
  real absence. It defeats casual replay, not a determined attacker with a
  radio. Your physical remotes and key are untouched and remain the fallback.
  Free hardening options if you want them: require a second signal such as a
  phone geofence, or authenticate the beacon.
- **Privacy.** The scanner is non-connectable and non-discoverable. Outside
  the census mode, other devices' addresses are never logged, only counts.

---

## Adapting it

- **Another remote.** Any remote with a push button. Measure the button's
  pad polarity with the battery in; collector to the positive pad. If the
  remote runs from a coin cell below 3.3 V the PC817 still works; if it runs
  above 5 V (the MITTO uses a 12 V cell) that is exactly why the optocoupler
  is there. Never power the remote from the ESP32.
- **Always-on car USB.** The parked car will then keep advertising. The
  wake-in-place refusal still protects you from a parked car, but "absence"
  never happens at home, so the system cannot re-arm while the car is in the
  garage. Either power the beacon from a source that switches (a 12 V
  accessory socket through a buck converter), or add the door-closed contact
  below and rework the arming rule.
- **A door sensor.** `REED_ENABLED` in tunables, GPIO 27, an alarm-type
  magnetic contact to ground. The input is already plumbed as a hard
  interlock: never pulse unless the door reads closed.
- **A different board.** The firmware is written against the original ESP32
  (WROOM-32, dual-mode controller). The beacon is plain BLE advertising and
  should port to any ESP32 variant; the scanner uses the dual-mode controller
  API and has only been run on the WROOM-32.
- **Timing.** From "first heard" to "door fully open" you get a handful of
  seconds of warning. Whether the door is open when you reach it or still
  opening depends on your approach geometry and door speed. The scanner's
  position is the main lever.

---

## Design notes

Short answers to "why not X", in case you were about to try X.

- **Why not the car's own Bluetooth?** It was the first design. The car's
  head unit is classic Bluetooth, answers discovery only while parked with
  someone inside, never while driving, and is about 35 dB weaker than a
  phone. So it is visible exactly when it must not fire and invisible when it
  must. Measured over a week of logging with a phone as a positive control.
- **Why an ESP32 as the beacon and not an AirTag or Tile?** Those rotate
  their addresses. The ESP32 has a burned-in public address, transmits at
  +9 dBm, and starts advertising within a second of power-up, which matters
  on a switched USB socket.
- **Why BLE scanning rather than classic inquiry on the scanner?** Continuous
  classic inquiry alongside WiFi crashes the ESP32's prebuilt Bluetooth
  controller every one to three hours (a controller assert under coexistence).
  The BLE build ran a 25 h soak clean.
- **Why an optocoupler on a real remote instead of emulating the remote's
  radio?** Enrolling a synthetic transmitter writes to the door's receiver,
  which breaks the "untouched" rule, and it needs a sub-GHz radio the ESP32
  lacks. The optocoupler changes nothing on the door side.
- **Why the ramp requirement rather than a simple threshold?** A threshold
  alone fires on a parked car being opened, and on noise at the edge of range.
  Requiring absence, then a rise of several dB over several sightings, is
  what distinguishes an approach from everything else seen in the logs.

---

## Licence

MIT. See [LICENSE](LICENSE).
