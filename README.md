# timecard-reader-firmware

PSS card-reader firmware. Publishes RFID tap events to MQTT
(`<site>/<stream>` topic, JSON payload), with on-device SD-card
logging as a forensic backup.

- **Current version:** `0.7.6` (see `FIRMWARE_VERSION` in `timecard_reader.ino`)
- **Topic published:** `carrwood/timecard` (default; overridable per-device via SD config)
- **Bridge consumer:** [ukstevem/timecard-bridge](https://github.com/ukstevem/timecard-bridge) — subscribes the same topic and writes to Supabase `timecard_events` via a durable SQLite outbox

## Hardware

- **M5Stack Core2** (ESP32, 320x240 IPS, SD slot, RTC, speaker)
- **WS1850S RFID** (MFRC522 over I2C @ address `0x28`, on Core2 Port A: SDA=21, SCL=22)
- **microSD card** holding `/timecard/config.ini` per-device config + `/timecard/YYYY-MM-DD.csv` daily tap logs
- **Audible tap feedback** — the Core2's **internal speaker** is the loudest source (bench-measured peak ~2.7 kHz, `SPK_LOUD_HZ`) and carries the cue. An **M5 Unit Buzzer (U085)** on **Port B (GPIO 26)** pulses in parallel at its ~4 kHz resonance (`BUZZER_PWM_FREQ_HZ`) — it's only ~72 dB and adds <1 dB, but localises the sound at the reader. Pass = two blips, fail = three blips + long blast (`beepOK`/`beepFail`). Buzzer needs no library and **cannot** go on a PaHUB (I²C-only, no PWM). For a genuinely louder alert, fit an active piezo siren (85–100 dB) or an amplified speaker unit.
- *(optional)* **PaHUB2** — I²C hub (PCA9548A @ `0x70`) if you ever need to hang more I²C units off Port A alongside the RFID reader. Downstream devices need a channel-select write before access.

## Required Arduino libraries

- `M5Unified`
- `PubSubClient`
- `MFRC522_I2C`
- `SD` (built-in)

Board: `esp32:esp32:m5stack_core2` (config in `sketch.json`). Note: ESP32
Arduino core 3.x renamed the board id from the older hyphenated
`m5stack-core2` — use the underscore form.

## Build / upload

Arduino IDE 2.x:

1. Open `timecard_reader.ino`
2. Copy `arduino_secrets.h.example` -> `arduino_secrets.h` and fill in WiFi/MQTT credentials (this file is gitignored — never commit credentials)
3. Select board: M5Stack-Core2
4. Upload

Headless (`arduino-cli`), e.g. the copy bundled with Arduino IDE 2.x
(`.../Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe`):

```sh
# one-time: the sketch needs these two libs in your sketchbook
arduino-cli lib install "PubSubClient" "MFRC522_I2C"

# compile + flash (COMx = the CH9102F USB port of the Core2)
arduino-cli compile --fqbn esp32:esp32:m5stack_core2 --upload -p COMx .
```

The **same binary flashes to every reader** — per-device identity comes
from the SD `config.ini` at runtime, not the build.

## Per-device runtime config (SD card)

Create `/timecard/config.ini` on the microSD card. Runtime values
override the build-time defaults from `arduino_secrets.h`:

```
wifi_ssid=PSS_Office
wifi_pass=your-wifi-password
mqtt_host=10.0.0.180
mqtt_port=1883
mqtt_user=timecard
mqtt_pass=letmein
site=carrwood
stream=timecard
actor=timecard
device_name=carr-tc-01
```

Only `site`, `stream`, `actor`, and `device_name` typically vary per
device. Allowed values:
- `site` ∈ {`carrwood`, `foxwood`}
- `stream` ∈ {`timecard`, `jobcard`}
- `actor` ∈ {`admin`, `test`, `harvester`, `timecard`}

If `device_name` is empty, a MAC-derived id (`core2-XXXXYYYYYYYY`) is
used.

## Fleet (known devices)

| device_name | site     | stream   | actor    |
|-------------|----------|----------|----------|
| `carr-tc-01`| carrwood | timecard | timecard |
| `carr-tc-02`| carrwood | timecard | timecard |

## SD-card logs

Every tap is appended to `/timecard/YYYY-MM-DD.csv` regardless of
MQTT state. 90-day rolling retention (`RETENTION_DAYS` in the sketch).
Forensic record — used for recovery if a tap was lost between reader
and Supabase.

To recover taps from a card: pull the SD, copy the relevant
`YYYY-MM-DD.csv`, replay rows via the `/hours` admin page (uses the
`record_manual_taps` RPC).

## Reader-side outbox (since v0.7.0)

Taps that `mqtt.publish()` does not accept (broker down, WiFi gone)
are appended to `/timecard/pending.jsonl` instead of being lost. On
MQTT reconnect — or every 30s while pending has content — the reader
drains the queue back through MQTT in arrival order. The bridge's
`UNIQUE(card_id, device_id, ts)` constraint (migration 023) makes
re-delivery idempotent, so a tap delivered twice during weird race
conditions still lands as a single row in Supabase.

Default backoff: drain attempted every 30s (`DRAIN_INTERVAL_MS`).
Whole pending file is loaded into RAM during the rewrite — fine for
backlogs up to ~1000 queued taps (~160 KB).

## MQTT payload shape

```json
{
  "event": "tap",
  "card_id": "A66A9500",
  "device_id": "carr-tc-01",
  "actor": "timecard",
  "ts": "2026-05-13T04:55:29Z",
  "firmware": "0.6.0"
}
```

Topic is `<site>/<stream>` — e.g. `carrwood/timecard`. QoS 0,
not-retained. Brokered by Mosquitto on `10.0.0.180:1883`.

## LWT / birth

On connect, the reader publishes `online` (retained) to
`<topic>/status`. The MQTT Last-Will-and-Testament is `offline`
(retained) on the same topic. Consumed by `mqtt-status-bridge`.

## Related repos / docs

- [ukstevem/timecard-bridge](https://github.com/ukstevem/timecard-bridge) — MQTT → Supabase ingestion bridge
- [ukstevem/mqtt-status-bridge](https://github.com/ukstevem/mqtt-status-bridge) — device presence/heartbeats
- [ukstevem/pss-employee-presence](https://github.com/ukstevem/pss-employee-presence) — Supabase schema (`timecard_events`, `employees`, RPCs) and the `/hours` admin UI for recovery

## Known issues

Tracked in [pss-employee-presence beads workspace](https://github.com/ukstevem/pss-employee-presence/blob/main/.beads/issues.jsonl), epic `pss-employee-presence-hc4`:

- `hc4.5` — Guard `publishAndLog` with `haveValidTime()` so taps before NTP sync don't get a 1999 timestamp
- `hc4.7` — Move hardcoded WiFi/MQTT defaults out of `timecard_reader.ino` into `arduino_secrets.h`
