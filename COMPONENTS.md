# Bluedoor — bill of materials and wiring map

Companion to [README.md](README.md). The illustrated version of the wiring,
with the optocoupler pinout, resistor colour codes and the soldering steps at
the remote, is [wiring-v1.html](wiring-v1.html).

Build v1 is deliberately minimal: two ESP32 boards, one optocoupler, two
resistors, and a spare remote. No door sensor (see the v2 kit at the end).

## Wiring map

```
                         ESP32 (scanner, in the garage)
                         ──────────────────────────────
  GPIO 26 ──[R1 330 Ω]──▶ PC817 pin 1 (LED anode)
                          PC817 pin 2 (LED cathode) ──▶ GND
  GPIO 26 ──[R2 10 kΩ]──▶ GND   (pulldown: a boot/reset can never
                                 press the button — NOT optional)

                          PC817 pin 4 (collector) ──▶ remote button pad (+)
                          PC817 pin 3 (emitter)  ──▶ remote button pad (−)
                          (measure pad polarity FIRST — output is directional)

  5V/USB  ◀── phone charger + USB cable

                         ESP32 (beacon, in the car)
                         ──────────────────────────
  USB     ◀── the car's switched USB socket. Nothing else connected.
```

GPIO 26 is a non-strapping pin, safe on boot. If you change it, avoid 0, 2,
5, 12 and 15.

The remote keeps its own battery. The optocoupler isolates the two circuits
completely, so the remote's voltage (12 V on the BFT MITTO used here) never
meets the ESP32.

## Shopping list

| # | Part | Qty | Notes | ~€ |
|---|---|---|---|---|
| 1 | ESP32-WROOM-32 dev board | 2 | one scanner, one beacon. Any WROOM-32 devkit; the Freenove FNK0090 was used. Not the C3/S3/C6/H2 variants for the scanner (different Bluetooth controller API). | 26 |
| 2 | PC817 optocoupler, DIP-4 | 1 + spare | EL817 / LTV-817 equivalents are fine | 1 |
| 3 | Resistor 330 Ω, 1/4 W | 1 + spare | opto LED series (≈6 mA from a 3.3 V GPIO); 220–470 Ω all fine | — |
| 4 | Resistor 10 kΩ, 1/4 W | 1 + spare | GPIO pulldown | — |
| 5 | Thin stranded hookup wire, AWG 26–28 | ~1 m | the two flying leads to the remote's button pads | 2 |
| 6 | Heat-shrink tubing, small sizes | a few | insulate every joint | 1 |
| 7 | Dupont jumper wires, female-to-female or male-to-female | a few | the assembly plugs onto the ESP32's header pins; nothing is soldered to the board | 2 |
| 8 | Small project box | 1 | houses the scanner ESP32 and the remote; one hole for the USB cable | 0–3 |
| 9 | A garage remote you can sacrifice | 1 | usually your old one, after enrolling a new daily-carry remote on the door | 0 |
| 10 | 5 V USB charger and cable | 1 | scanner power | 0 |

Total, excluding the boards and a replacement remote: **under €10**.
Resistors are cheapest as a small E12 assortment box if you have none.

No perfboard is needed. The optocoupler and two resistors are soldered
"dead-bug" onto the flying leads, heat-shrunk, and jumpered onto the ESP32's
header pins.

## Tools

| Item | Why |
|---|---|
| Soldering iron with a fine tip, 0.8 mm rosin-core solder | the remote's pads are small |
| Flux pen and desoldering braid | clean joints; braid is mistake insurance on the remote PCB |
| **Multimeter** | **mandatory before soldering**: button-pad polarity, continuity checks |
| Wire strippers, tweezers, tape, hot glue | assembly and strain relief |
| Isopropyl alcohol | flux residue off an old remote board |

## Order of work

1. Enroll the new daily-carry remote on the door. The old remote is usually
   the programming key and must survive intact until this is done.
2. Flash the beacon and the listen-only scanner; find the mounting spot with
   the log (README, "Find the mounting spot").
3. Range-check the old remote from that spot: it must still operate the door
   from where it will live.
4. Open the old remote, measure the button pads, solder the two leads.
5. Jumper everything together, flash the production build, commission in
   DRY-RUN with the manual pulse. Make it permanent once proven.

## v2 upgrade kit — optional

Only if v1's no-door-sensor behaviour annoys you (the door was left open on
arrival and the press closed it). The firmware input is already stubbed:
`REED_ENABLED` and `PIN_REED` (GPIO 27) in tunables.

| Part | Qty | Notes | ~€ |
|---|---|---|---|
| Wired magnetic door contact (alarm type, screw terminals) | 1 | closed when the magnet is adjacent → door closed = GPIO LOW | 3–5 |
| 2-core alarm/bell wire | 5–10 m | contact → ESP32 run | 3 |
| 100 nF ceramic capacitor | 1 | noise filter at the ESP32 end | — |
| Small toggle switch | 1 | optional physical arm/disable | 1 |
| Perfboard ~5×7 cm + female header strips | 1 + 2 | tidier permanent assembly | 2 |
