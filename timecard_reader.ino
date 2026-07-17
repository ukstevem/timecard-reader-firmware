// ============================================================
// timecard-reader-firmware  v0.7.10
// https://github.com/ukstevem/timecard-reader-firmware
//
// Keep this banner in sync with FIRMWARE_VERSION below.
//
// REQUIRES: arduino_secrets.h next to this sketch (gitignored).
// Copy arduino_secrets.h.example -> arduino_secrets.h and fill in
// real WiFi / MQTT credentials before compiling.
// ============================================================

#include "arduino_secrets.h"

// ========= User settings (defaults; runtime SD config.ini wins) =========
const char* WIFI_SSID     = SECRET_WIFI_SSID;
const char* WIFI_PASSWORD = SECRET_WIFI_PASS;

const char* MQTT_HOST     = "10.0.0.180";
const uint16_t MQTT_PORT  = 1883;
const char* MQTT_USER     = SECRET_MQTT_USER;
const char* MQTT_PASS     = SECRET_MQTT_PASS;

const char* MQTT_TOPIC    = "carrwood/timecard";  // publishes "time,cardid"

// ======== JSON meta ========
const char* DEVICE_ACTOR      = "timecard";   // one of: admin, test, harvester, timecard
const char* FIRMWARE_VERSION  = "0.7.10";      // bump as you release

// ===== Runtime-configurable (defaults from existing constants) =====
String CFG_WIFI_SSID     = WIFI_SSID;
String CFG_WIFI_PASS     = WIFI_PASSWORD;

String CFG_MQTT_HOST     = MQTT_HOST;
uint16_t CFG_MQTT_PORT   = MQTT_PORT;
String CFG_MQTT_USER     = MQTT_USER;
String CFG_MQTT_PASS     = MQTT_PASS;

String CFG_SITE          = "carrwood";   // default
String CFG_STREAM        = "timecard";   // default
String CFG_ACTOR         = "timecard";   // default: one of admin|test|harvester|timecard
String CFG_DEVICE_NAME   = "";           // optional; if empty we’ll use MAC-based id

String CFG_TOPIC         = "";           // computed as "<site>/<stream>"

// =================================
#include <M5Unified.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <MFRC522_I2C.h>
#include <SD.h>
#include <time.h>
#include <ctype.h>
#include <cstring>  // for strrchr
#include <cstdio>   // for sscanf
#include <FS.h>

// forward decls if the bodies are below
bool parseKV(const String& line, String& k, String& v);
void loadConfigFromSD();

// ---------- UI Types & State ----------
struct UiStatus { bool wifiOK; bool mqttOK; };
struct HeaderCache { bool wifi; bool mqtt; int sdPct; int pending; bool init; };

HeaderCache hdrCache = { false, false, -1, -1, false };
UiStatus    ui       = { false, false };

// Taps sitting in the outbox waiting on proof of delivery. Tracked in RAM so
// the 1 Hz header redraw never has to read the SD card; kept in step by
// appendPending() / drainPending(), and seeded from the file at boot.
int pendingCount = 0;

// --- RFID / I2C config ---
#define RFID_I2C_ADDR 0x28
#define RFID_RST_PIN  0xFF

// SD config (TF card on M5 Core2 typically CS=4)
#define TF_CS 4
#define LOG_DIR "/timecard"
#define RETENTION_DAYS 90

// Reader-side outbox: taps that mqtt.publish() did not accept land here
// and are drained on MQTT reconnect (or periodically while pending).
// Daily CSV (LOG_DIR/YYYY-MM-DD.csv) is the forensic record and is
// unaffected by drain state.
#define PENDING_PATH (LOG_DIR "/pending.jsonl")
const uint32_t DRAIN_INTERVAL_MS = 30000;

// Time formatting (UTC) for payload
#define USE_ISO_TIME 1   // 1 => ISO 8601 UTC, 0 => epoch seconds

// Debounce window (ms) to avoid repeats if card stays near reader
const uint32_t RESCAN_BLOCK_MS = 5000;

// ========= Globals =========

char DEVICE_ID[24] = "core2-unknown";

MFRC522_I2C mfrc522(RFID_I2C_ADDR, RFID_RST_PIN, &Wire);
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

String lastUid = "";
uint32_t lastUidMillis = 0;

uint32_t lastReconnectAttempt = 0;
String lastPurgeDate = "";  // "YYYY-MM-DD" last time we purged
bool sdReady = false;

// Pending-queue state — used by drainPending() in loop()
bool prevMqttUp = false;
uint32_t lastDrainMillis = 0;

// An echo proves a queued tap landed, but the line isn't removed until a drain
// runs — so without this the header would sit amber at Q:1 for up to
// DRAIN_INTERVAL_MS (30s) after every ordinary tap. A counter that is amber
// half the time trains people to ignore it, which is how lost taps went
// unnoticed for a fortnight. So an echo schedules a drain shortly after.
// The short settle lets a burst of echoes land in ONE rewrite rather than one
// rewrite per tap.
const uint32_t ECHO_SETTLE_MS = 1500;
bool     drainScheduled   = false;
uint32_t drainScheduledAt = 0;

// Speaker volume for beep cues (M5Unified Speaker, 0..255)
const uint8_t SPEAKER_VOLUME = 255;

// ---------- External buzzer (M5 Unit Buzzer, U085) ----------
// PASSIVE buzzer -> needs a PWM square wave on a GPIO. It CANNOT be driven
// through the PaHUB2 (that mux only routes I2C SDA/SCL). Plug the Unit Buzzer
// into Core2 PORT B (black); its yellow signal wire lands on GPIO 26.
#define BUZZER_PIN            26     // Port B yellow. Use 13 if you wire it to Port C.
#define BUZZER_PWM_FREQ_HZ    4000   // this unit is loudest around its 4 kHz resonance
#define USE_INTERNAL_SPEAKER  1      // internal speaker is the LOUDEST source (bench-measured); it carries the cue
#define SPK_LOUD_HZ           2700   // Core2 internal speaker's loudest band (measured louder than 4 kHz)

// LEDC tone shim: pin-based on ESP32 Arduino core 3.x, channel-based on 2.x.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  inline void buzzerBegin(){ ledcAttach(BUZZER_PIN, BUZZER_PWM_FREQ_HZ, 10); ledcWriteTone(BUZZER_PIN, 0); }
  inline void buzzerTone(uint16_t f){ ledcWriteTone(BUZZER_PIN, f); }
  inline void buzzerOff(){ ledcWriteTone(BUZZER_PIN, 0); }
#else
  static const int BUZZER_LEDC_CH = 2;   // keep clear of LEDC channel 0
  inline void buzzerBegin(){ ledcSetup(BUZZER_LEDC_CH, BUZZER_PWM_FREQ_HZ, 10); ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CH); ledcWriteTone(BUZZER_LEDC_CH, 0); }
  inline void buzzerTone(uint16_t f){ ledcWriteTone(BUZZER_LEDC_CH, f); }
  inline void buzzerOff(){ ledcWriteTone(BUZZER_LEDC_CH, 0); }
#endif

// The pass/fail cues are played inline in beepOK()/beepFail() below, as plain
// buzzerTone()+delay() sequences. Deliberately NOT driven by an array-of-struct
// helper: a user-defined type in a function signature trips the Arduino .ino
// auto-prototype generator (it hoists the prototype above the type definition).

// Minimal-redraw cache for the clock
String prevTimeRendered = "";
String prevDateRendered = "";
int prevTimeW = 0, prevTimeH = 0;
int prevDateW = 0, prevDateH = 0;

// minute gating
int  lastClockMinute = -1;
bool drewPlaceholder  = false;

// LOUDNESS (bench-measured 2026-07-14): the Core2's amplified INTERNAL speaker is
// the loudest source, peaking near 2.7 kHz — louder than the external U085 buzzer,
// which is ~72 dB and adds <1 dB as a second source. So each cue "note" drives the
// internal speaker at SPK_LOUD_HZ AND pulses the external buzzer at its resonance in
// parallel (uses the fitted part, localises the sound). Pass/fail differ by rhythm.

// One cue note: internal speaker + external buzzer together, for ms.
inline void cueOn(uint16_t ms){
#if USE_INTERNAL_SPEAKER
  M5.Speaker.tone(SPK_LOUD_HZ, ms);   // non-blocking; auto-stops after ms
#endif
  buzzerTone(BUZZER_PWM_FREQ_HZ);
  delay(ms);
  buzzerOff();
}
inline void cueGap(uint16_t ms){
#if USE_INTERNAL_SPEAKER
  M5.Speaker.stop();
#endif
  buzzerOff();
  delay(ms);
}

// PASS: two short blips — "beep-beep".
inline void beepOK(){
  cueOn(90); cueGap(70); cueOn(90);
#if USE_INTERNAL_SPEAKER
  M5.Speaker.stop();
#endif
}
// FAIL: three urgent blips + a long blast — "bip-bip-bip-beeeep".
inline void beepFail(){
  for (int i = 0; i < 3; i++){ cueOn(70); cueGap(50); }
  cueOn(450);
#if USE_INTERNAL_SPEAKER
  M5.Speaker.stop();
#endif
}

// ---------- NTP / Time ----------
// Europe/London with DST (BST): last Sun in Mar (01:00) to last Sun in Oct (02:00)
static const char* TZ_LONDON = "GMT0BST,M3.5.0/1,M10.5.0/2";
static const char* NTP_1 = "pool.ntp.org";
static const char* NTP_2 = "time.google.com";

// ---------- UI CONFIG ----------
static const int HEADER_H = 22;     // slim admin bar
static const int PADDING  = 6;

// Colors (24-bit RGB888; 0xRRGGBB)
static const uint32_t COL_BG        = 0x000000; // black
static const uint32_t COL_HEADER_BG = 0x303030; // dark grey
static const uint32_t COL_OK        = 0x00FF00; // green
static const uint32_t COL_BAD       = 0xFF0000; // red
static const uint32_t COL_WARN      = 0xFFA000; // amber — queued taps waiting
static const uint32_t COL_TEXT      = 0xFFFFFF; // white
static const uint32_t COL_TIME      = 0xFFFFFF; // white
static const uint32_t COL_DATE      = 0xFFFFFF; // white

// ---------- Helpers ----------
static String toHex(const byte* data, byte len) {
  const char* HEX_CHARS = "0123456789ABCDEF";
  String out; out.reserve(len*2);
  for (byte i=0;i<len;++i){ out += HEX_CHARS[(data[i]>>4)&0x0F]; out += HEX_CHARS[data[i]&0x0F]; }
  return out;
}

static String iso8601_utc(time_t t){
  struct tm tm; gmtime_r(&t,&tm);
  char buf[32]; strftime(buf,sizeof(buf),"%Y-%m-%dT%H:%M:%SZ",&tm);
  return String(buf);
}
static String iso_date(time_t t){
  struct tm tm; gmtime_r(&t,&tm);
  char buf[16]; strftime(buf,sizeof(buf),"%Y-%m-%d",&tm);
  return String(buf);
}

bool i2cSeen(uint8_t addr){ Wire.beginTransmission(addr); return (Wire.endTransmission()==0); }
bool haveValidTime(){ return time(nullptr) > 1609459200; } // valid after 2021-01-01

// Julian day math to compare dates without extra libs
int daysFromCivil(int y, unsigned m, unsigned d){
  y -= m <= 2;
  const int era = (y >= 0 ? y : y-399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153*(m + (m>2?-3:9)) + 2)/5 + d-1;
  const unsigned doe = yoe * 365 + yoe/4 - yoe/100 + doy;
  return era * 146097 + (int)doe - 719468; // days since 1970-01-01
}
bool parseYmd(const char* name, int& y, int& m, int& d){
  // expect "YYYY-MM-DD.csv"
  if (strlen(name) < 14) return false;
  if (!(isdigit(name[0])&&isdigit(name[1])&&isdigit(name[2])&&isdigit(name[3])&&
        name[4]=='-'&&isdigit(name[5])&&isdigit(name[6])&&name[7]=='-'&&
        isdigit(name[8])&&isdigit(name[9]))) return false;
  y = (name[0]-'0')*1000 + (name[1]-'0')*100 + (name[2]-'0')*10 + (name[3]-'0');
  m = (name[5]-'0')*10 + (name[6]-'0');
  d = (name[8]-'0')*10 + (name[9]-'0');
  return true;
}

// --- Font helpers (LovyanGFX / M5Unified) ---
inline void useHeaderFont(){ M5.Display.setFont(&fonts::Font2); M5.Display.setTextSize(1); }              // small
inline void useClockFont() { M5.Display.setFont(&fonts::FreeMonoBold24pt7b); M5.Display.setTextSize(2); } // big, crisp, monospace
inline void useDateFont()  { M5.Display.setFont(&fonts::FreeSans12pt7b); M5.Display.setTextSize(1); }     // clean small
inline void useUiFont()    { M5.Display.setFont(&fonts::Font2); M5.Display.setTextSize(1); }              // general UI

// ---------- SD helpers ----------
int sdPercentFree(){
  if (!sdReady) return -1;
  uint64_t total = SD.totalBytes();
  uint64_t used  = SD.usedBytes();
  if (total == 0) return -1;
  uint64_t freeB = (total > used) ? (total - used) : 0;
  return (int)((freeB * 100ULL) / total);
}

// ---------- UI drawing ----------
void uiInitDisplay(){
  M5.Display.setRotation(1); // landscape
  M5.Display.fillScreen(COL_BG);
  M5.Display.setTextColor(COL_TEXT, COL_BG);
  M5.Display.setColorDepth(24);   // keep everything in RGB888
}

void drawDot(int x, int y, bool ok){
  M5.Display.fillCircle(x, y, 5, ok ? COL_OK : COL_BAD);
  M5.Display.drawCircle(x, y, 5, 0xFFFFFF); // white outline
}

void drawHeader(const UiStatus& s){
  // compute current SD % (or n/a)
  int pct = sdPercentFree();

  // Only redraw when something changed
  if (!hdrCache.init || hdrCache.wifi != s.wifiOK || hdrCache.mqtt != s.mqttOK
      || hdrCache.sdPct != pct || hdrCache.pending != pendingCount){
    hdrCache.wifi = s.wifiOK;
    hdrCache.mqtt = s.mqttOK;
    hdrCache.sdPct = pct;
    hdrCache.pending = pendingCount;
    hdrCache.init = true;

    M5.Display.fillRect(0, 0, M5.Display.width(), HEADER_H, COL_HEADER_BG);
    // small header font + WHITE on grey
    M5.Display.setTextDatum(TL_DATUM);
    useHeaderFont();
    M5.Display.setTextColor(0xFFFFFF, COL_HEADER_BG);

    int x = PADDING;
    int cy = HEADER_H/2;

    // WiFi dot + label
    M5.Display.fillCircle(x+6, cy, 5, s.wifiOK ? COL_OK : COL_BAD);
    M5.Display.drawCircle(x+6, cy, 5, 0xFFFFFF);
    M5.Display.setCursor(x+16, 4); M5.Display.print("WiFi");
    x += 60;

    // MQTT dot + label
    M5.Display.fillCircle(x+6, cy, 5, s.mqttOK ? COL_OK : COL_BAD);
    M5.Display.drawCircle(x+6, cy, 5, 0xFFFFFF);
    M5.Display.setCursor(x+16, 4); M5.Display.print("MQTT");
    x += 70;

    // SD %
    M5.Display.setCursor(x, 4);
    if (pct >= 0) M5.Display.printf("SD %d%% free", pct);
    else          M5.Display.print("SD n/a");

    // Outbox depth, right-aligned: taps read but not yet proven delivered.
    // Amber while anything is waiting, white at zero. Should sit at 0 on a
    // healthy link and climb only while the broker is unreachable.
    M5.Display.setTextDatum(TR_DATUM);
    M5.Display.setTextColor(pendingCount > 0 ? COL_WARN : 0xFFFFFF, COL_HEADER_BG);
    M5.Display.drawString(String("Q:") + String(pendingCount),
                          M5.Display.width() - PADDING, 4);
    M5.Display.setTextDatum(TL_DATUM);
    M5.Display.setTextColor(0xFFFFFF, COL_HEADER_BG);
  }
}

void clearMainArea(){
  M5.Display.fillRect(0, HEADER_H, M5.Display.width(), M5.Display.height()-HEADER_H, COL_BG);
}

void drawClock(){
  const int w  = M5.Display.width();
  const int cx = w / 2;

  // Y positions (tweaked for larger time text)
  const int yTime = HEADER_H + 64;
  const int yDate = yTime + 56;

  M5.Display.setTextDatum(MC_DATUM);

  // If time not valid yet, draw placeholder ONCE
  if (!haveValidTime()){
    if (!drewPlaceholder){
      // ---- TIME placeholder ----
      useClockFont();               // larger clock font (make sure useClockFont uses setTextSize(2))
      int fh = M5.Display.fontHeight();
      int tw = M5.Display.textWidth("--:--");

      if (prevTimeW > 0) {
        M5.Display.fillRect(cx - prevTimeW/2 - 2, yTime - prevTimeH/2 - 2,
                            prevTimeW + 4, prevTimeH + 4, COL_BG);
      }
      M5.Display.setTextColor(COL_TIME, COL_BG);
      M5.Display.drawString("--:--", cx, yTime);
      prevTimeRendered = "--:--"; prevTimeW = tw; prevTimeH = fh;

      // ---- DATE placeholder ----
      useDateFont();
      fh = M5.Display.fontHeight();
      tw = M5.Display.textWidth("Syncing time…");

      if (prevDateW > 0) {
        M5.Display.fillRect(cx - prevDateW/2 - 2, yDate - prevDateH/2 - 2,
                            prevDateW + 4, prevDateH + 4, COL_BG);
      }
      M5.Display.setTextColor(COL_DATE, COL_BG);
      M5.Display.drawString("Syncing time…", cx, yDate);
      prevDateRendered = "Syncing time…"; prevDateW = tw; prevDateH = fh;

      drewPlaceholder = true;
    }
    M5.Display.setTextDatum(TL_DATUM);
    return;
  }

  // Build current local time/date
  time_t now = time(nullptr);
  struct tm lt;
  localtime_r(&now, &lt); // BST-aware via TZ_LONDON

  // >>> Only draw when minute changed <<<
  if (lt.tm_min == lastClockMinute) {
    M5.Display.setTextDatum(TL_DATUM);
    return;  // skip drawing this loop
  }

  // Format strings (no seconds)
  char timeStr[16];
  strftime(timeStr, sizeof(timeStr), "%H:%M", &lt);

  char dateStr[32];
  strftime(dateStr, sizeof(dateStr), "%a, %d %b %Y", &lt);

  // ---- TIME ----
  useClockFont();
  int fhT = M5.Display.fontHeight();
  int twT = M5.Display.textWidth(timeStr);

  if (prevTimeW > 0) {
    M5.Display.fillRect(cx - prevTimeW/2 - 2, yTime - prevTimeH/2 - 2,
                        prevTimeW + 4, prevTimeH + 4, COL_BG);
  }
  M5.Display.setTextColor(COL_TIME, COL_BG);
  M5.Display.drawString(timeStr, cx, yTime);
  prevTimeRendered = timeStr; prevTimeW = twT; prevTimeH = fhT;

  // ---- DATE ----
  useDateFont();
  int fhD = M5.Display.fontHeight();
  int twD = M5.Display.textWidth(dateStr);

  if (prevDateW > 0) {
    M5.Display.fillRect(cx - prevDateW/2 - 2, yDate - prevDateH/2 - 2,
                        prevDateW + 4, prevDateH + 4, COL_BG);
  }
  M5.Display.setTextColor(COL_DATE, COL_BG);
  M5.Display.drawString(dateStr, cx, yDate);
  prevDateRendered = dateStr; prevDateW = twD; prevDateH = fhD;

  // Update gate + reset placeholder flag
  lastClockMinute = lt.tm_min;
  drewPlaceholder = false;

  M5.Display.setTextDatum(TL_DATUM);
}

void showScanBanner(bool success, const String& detail){
  const int boxW = M5.Display.width()  - (PADDING*2);
  const int boxH = 60;
  const int boxX = PADDING;
  const int boxY = M5.Display.height() - boxH - PADDING;

  uint32_t col = success ? COL_OK : COL_BAD;
  M5.Display.fillRoundRect(boxX, boxY, boxW, boxH, 8, col);
  M5.Display.drawRoundRect(boxX, boxY, boxW, boxH, 8, 0x000000);

  // Fixed small UI font so it won't affect the clock
  M5.Display.setTextDatum(ML_DATUM);
  useUiFont();
  M5.Display.setTextColor(0x000000, col);

  M5.Display.drawString(success ? "Scan Success" : "Scan Fail", boxX + 10, boxY + 18);
  if (detail.length()){
    M5.Display.drawString(detail, boxX + 10, boxY + 40);
  }

  M5.Display.setTextDatum(TL_DATUM);
}

void clearScanBanner(){
  int boxH = 60;
  int boxY = M5.Display.height() - boxH - PADDING;
  M5.Display.fillRect(0, boxY-2, M5.Display.width(), boxH+PADDING+4, COL_BG);
}

void uiUpdateAll(){
  drawHeader(ui);  // redraws only if changed
  // DO NOT clearMainArea();  // prevents flashing
  drawClock();     // repaints just slim bands behind text
}

// ---------- SD logging ----------
bool initSD(){
  if (SD.begin(TF_CS)) {
    SD.mkdir(LOG_DIR);
    sdReady = true;
    return true;
  }
  sdReady = false;
  return false;
}

bool appendLineToSD(const String& dateStr, const String& line){
  if (!sdReady) return false;
  String path = String(LOG_DIR) + "/" + dateStr + ".csv";
  File f = SD.open(path, FILE_APPEND);
  if (!f) {
    f = SD.open(path, FILE_WRITE);
    if (!f) return false;
    f.seek(f.size());
  }
  f.println(line);
  f.close();
  return true;
}

void purgeOldLogsIfNeeded(){
  if (!sdReady) return;
  if (!haveValidTime()) return; // wait for NTP
  String today = iso_date(time(nullptr));
  if (today == lastPurgeDate) return; // purge once per day

  File dir = SD.open(LOG_DIR);
  if (!dir) return;
  int yNow, mNow, dNow;
  sscanf(today.c_str(), "%d-%d-%d", &yNow, &mNow, &dNow);
  int todayDays = daysFromCivil(yNow, mNow, dNow);

  for (File f = dir.openNextFile(); f; f = dir.openNextFile()){
    if (f.isDirectory()) continue;
    const char* nm = f.name();
    const char* base = strrchr(nm, '/'); base = base ? base+1 : nm;

    int y,m,d;
    if (parseYmd(base, y,m,d)){
      int fileDays = daysFromCivil(y,m,d);
      if ((todayDays - fileDays) > RETENTION_DAYS){
        String fullPath = String(LOG_DIR) + "/" + String(base);
        SD.remove(fullPath);
      }
    }
  }
  dir.close();
  lastPurgeDate = today;
}

// ---------- Pending-queue (reader-side outbox) ----------
// Each line in PENDING_PATH is a complete JSON payload as published.
// We append on publish failure and drain on reconnect.

bool appendPending(const String& payload){
  if (!sdReady) return false;
  File f = SD.open(PENDING_PATH, FILE_APPEND);
  if (!f) {
    f = SD.open(PENDING_PATH, FILE_WRITE);
    if (!f) return false;
  }
  f.println(payload);
  f.close();
  pendingCount++;             // header shows outbox depth
  return true;
}

// Count queued taps on disk. Only called at boot — after that the count is
// maintained in RAM by appendPending()/drainPending().
int countPendingLines(){
  if (!sdReady) return 0;
  if (!SD.exists(PENDING_PATH)) return 0;
  File f = SD.open(PENDING_PATH, FILE_READ);
  if (!f) return 0;
  int n = 0;
  while (f.available()){
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length()) n++;
  }
  f.close();
  return n;
}

// ---------- Delivery confirmation (broker echo ACK) ----------
// PubSubClient is QoS 0 only: publish() returns true once the payload reaches
// the LOCAL TCP SOCKET, not when the broker has it. On a lossy link that lie
// cost us taps — drainPending() used to delete lines it had never delivered,
// and publishAndLog() skipped the outbox entirely on a "successful" publish.
//
// So prove delivery rather than assume it. The device subscribes to its own
// publish topic; the broker echoes back every message it accepts. That echo is
// end-to-end proof. Until it arrives the tap stays in the outbox and is
// re-published on each drain. A lost echo only costs a duplicate, and the
// bridge's UNIQUE(card_id, device_id, ts) makes duplicates free — so we bias
// all the way toward re-sending. Everything eventually gets through.
const uint8_t CONFIRMED_MAX = 64;
uint32_t confirmedHashes[CONFIRMED_MAX];
uint8_t  confirmedCount = 0;
uint8_t  confirmedNext  = 0;

uint32_t payloadHash(const uint8_t* data, unsigned int len){
  uint32_t h = 2166136261UL;                 // FNV-1a
  for (unsigned int i = 0; i < len; i++){ h ^= data[i]; h *= 16777619UL; }
  return h;
}
uint32_t payloadHashStr(const String& s){
  return payloadHash((const uint8_t*)s.c_str(), s.length());
}
bool isConfirmed(uint32_t h){
  for (uint8_t i = 0; i < confirmedCount; i++) if (confirmedHashes[i] == h) return true;
  return false;
}
void markConfirmed(uint32_t h){
  if (isConfirmed(h)) return;
  confirmedHashes[confirmedNext] = h;
  confirmedNext = (confirmedNext + 1) % CONFIRMED_MAX;
  if (confirmedCount < CONFIRMED_MAX) confirmedCount++;
}

// Echo from the broker on our own topic. The other reader's taps land here too
// and simply never match a line in our outbox, so they're harmless.
//
// Called from mqtt.loop() on the main task — a plain flag is safe here, no ISR.
void onMqttMessage(char* topic, uint8_t* payload, unsigned int length){
  markConfirmed(payloadHash(payload, length));

  // Something we're holding may now be provable: drain shortly, so Q returns
  // to 0 promptly instead of waiting out the 30s tick. Guarded on
  // pendingCount so the other reader's traffic doesn't schedule pointless
  // SD rewrites.
  if (pendingCount > 0 && !drainScheduled){
    drainScheduled   = true;
    drainScheduledAt = millis();
  }
}

// Read pending.jsonl. Drop lines the broker has echoed back (proven delivered);
// re-publish and KEEP everything else. Returns the count purged this pass.
// Notes:
//  - whole file is loaded into RAM during the rewrite — fine for normal
//    backlogs (a tap is ~160 bytes; 100 queued taps = ~16 KB)
//  - a line is only ever removed on proof of delivery, never on publish()
//  - daily CSV is unaffected (forensic record)
int drainPending(){
  if (!sdReady) return 0;
  if (!mqtt.connected()) return 0;
  if (!SD.exists(PENDING_PATH)) return 0;

  File r = SD.open(PENDING_PATH, FILE_READ);
  if (!r) return 0;

  String unsent;
  unsent.reserve(2048);
  int purged = 0, resent = 0, kept = 0;

  while (r.available()){
    String line = r.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;

    // Proven delivered — the broker echoed this exact payload back to us.
    if (isConfirmed(payloadHashStr(line))) { purged++; continue; }
    kept++;

    // Not proven. (Re)publish and KEEP the line regardless of what publish()
    // claims: its return value only means "handed to the socket". The line
    // survives until an echo proves the broker took it.
    if (mqtt.connected()){
      mqtt.publish(
        CFG_TOPIC.c_str(),
        (const uint8_t*)line.c_str(),
        (unsigned int)line.length(),
        /*retain=*/false
      );
      resent++;
    }
    unsent += line; unsent += '\n';
  }
  r.close();

  // Rewrite atomically-ish: remove then write. If power cuts between
  // the two, we lose ONLY the not-yet-rewritten lines — at worst the
  // count of (sent + kept) taps lost this pass. The daily CSV still
  // holds them as forensic evidence; bridge UNIQUE constraint protects
  // against duplicates if we later re-replay manually.
  SD.remove(PENDING_PATH);
  if (unsent.length()){
    File w = SD.open(PENDING_PATH, FILE_WRITE);
    if (w){
      w.print(unsent);
      w.close();
    }
  }

  pendingCount = kept;        // authoritative: what we just rewrote to disk

  if (purged || resent){
    Serial.printf("drainPending: purged=%d resent=%d queued=%d\n", purged, resent, kept);
  }
  return purged;
}

bool pendingHasContent(){
  if (!sdReady) return false;
  if (!SD.exists(PENDING_PATH)) return false;
  File f = SD.open(PENDING_PATH, FILE_READ);
  if (!f) return false;
  bool any = (f.size() > 0);
  f.close();
  return any;
}

/* ------------ SD config loader (INI-style) ------------ */
// key=value parser
bool parseKV(const String& line, String& k, String& v) {
  int eq = line.indexOf('=');
  if (eq <= 0) return false;
  k = line.substring(0, eq); v = line.substring(eq+1);
  k.trim(); v.trim();
  k.toLowerCase();
  return (k.length() && v.length());
}

// Reads /timecard/config.ini and fills CFG_* then computes CFG_TOPIC
void loadConfigFromSD() {
  if (!sdReady) return;  // SD not mounted -> keep defaults

  File f = SD.open(String(LOG_DIR) + "/config.ini", FILE_READ);
  if (!f) {
    // no per-device config file; keep defaults
    CFG_TOPIC = CFG_SITE + "/" + CFG_STREAM;
    return;
  }

  while (f.available()) {
    String line = f.readStringUntil('\n'); line.trim();
    if (!line.length() || line.startsWith("#") || line.startsWith(";")) continue;

    String k, v; 
    if (!parseKV(line, k, v)) continue;

    if (k=="wifi_ssid")        CFG_WIFI_SSID = v;
    else if (k=="wifi_pass")   CFG_WIFI_PASS = v;
    else if (k=="mqtt_host")   CFG_MQTT_HOST = v;
    else if (k=="mqtt_port")   CFG_MQTT_PORT = (uint16_t)v.toInt();
    else if (k=="mqtt_user")   CFG_MQTT_USER = v;
    else if (k=="mqtt_pass")   CFG_MQTT_PASS = v;
    else if (k=="site")        CFG_SITE = v;
    else if (k=="stream")      CFG_STREAM = v;
    else if (k=="actor")       CFG_ACTOR = v;
    else if (k=="device_name") CFG_DEVICE_NAME = v;
  }
  f.close();

  // normalise + validate
  CFG_SITE.toLowerCase();
  CFG_STREAM.toLowerCase();
  CFG_ACTOR.toLowerCase();

  if (!(CFG_STREAM=="timecard" || CFG_STREAM=="jobcard")) CFG_STREAM = "timecard";
  if (!(CFG_SITE=="carrwood" || CFG_SITE=="foxwood"))     CFG_SITE   = "carrwood";
  if (!(CFG_ACTOR=="admin" || CFG_ACTOR=="test" || CFG_ACTOR=="harvester" || CFG_ACTOR=="timecard"))
    CFG_ACTOR = "timecard";

  // final topic
  CFG_TOPIC = CFG_SITE + "/" + CFG_STREAM;
}


// ---------- Connectivity ----------
bool connectWiFi(uint32_t timeout_ms=15000){
  WiFi.mode(WIFI_STA);
  WiFi.begin(CFG_WIFI_SSID.c_str(), CFG_WIFI_PASS.c_str());
  uint32_t start=millis();
  while (WiFi.status()!=WL_CONNECTED && (millis()-start)<timeout_ms){
    delay(250);
  }
  return (WiFi.status()==WL_CONNECTED);
}

// ONE connection attempt. Returns true if connected.
//
// Deliberately does NOT retry or delay() internally. loop() owns the retry
// cadence. A blocking retry loop here starves the RFID poll: mqtt.connect()
// can itself block for setSocketTimeout() on a dead broker, so looping over it
// pinned the main loop for 4-6s out of every 5s whenever MQTT was down. Staff
// tapped, nothing read the card, and the SD outbox never saw the tap either --
// the outbox only helps for taps we actually manage to READ.
bool connectMQTTOnce(){
  if (WiFi.status()!=WL_CONNECTED) return false;

  mqtt.setServer(CFG_MQTT_HOST.c_str(), CFG_MQTT_PORT);
  mqtt.setKeepAlive(30);
  mqtt.setSocketTimeout(5);
  mqtt.setCallback(onMqttMessage);   // broker echo = delivery proof

  // Last Will & Testament (retained "offline")
  const String willTopic = CFG_TOPIC + "/status";
  String cid = String(DEVICE_ID);

  bool ok = mqtt.connect(
    cid.c_str(),
    CFG_MQTT_USER.c_str(),
    CFG_MQTT_PASS.c_str(),
    willTopic.c_str(),
    /*willQos=*/1,
    /*willRetain=*/true,
    "offline"
  );

  if (ok) {
    // Birth message: retained "online"
    mqtt.publish(willTopic.c_str(), "online", /*retain=*/true);
    // Subscribe to our own publish topic: the broker echoes back what it
    // accepts, which is how drainPending() proves a tap actually landed.
    mqtt.subscribe(CFG_TOPIC.c_str());
  }
  return ok;
}

// Blocking connect with retries. Only for setup(), where there is no RFID
// polling to starve yet. Do NOT call this from loop().
bool connectMQTT(uint32_t timeout_ms=10000){
  const uint32_t start = millis();
  while (!mqtt.connected() && (millis() - start) < timeout_ms) {
    if (!connectMQTTOnce()) delay(1000);
  }
  return mqtt.connected();
}



// ---------- Publish & Log (JSON) ----------
void publishAndLog(const String& isoTime, const String& uidHex){
  // NTP guard: refuse to publish before clock sync to avoid the 1999
  // timestamp bug (haveValidTime() returns true only after 2021-01-01).
  // Tap is dropped here rather than written to SD/pending with a junk
  // ts — staff should re-tap once the clock placeholder clears.
  if (!haveValidTime()) {
    showScanBanner(false, "Clock not set — retry");
    beepFail();
    return;
  }

  // JSON payload the bridge expects
  String payload;
  payload.reserve(160);
  payload  = "{\"event\":\"tap\"";
  payload += ",\"card_id\":\"";   payload += uidHex;           payload += "\"";
  payload += ",\"device_id\":\""; payload += DEVICE_ID;        payload += "\"";
  payload += ",\"actor\":\"";     payload += CFG_ACTOR;        payload += "\"";   // <- use CFG_ACTOR so SD config works
  payload += ",\"ts\":\"";        payload += isoTime;          payload += "\"";
  payload += ",\"firmware\":\"";  payload += FIRMWARE_VERSION; payload += "\"}";
  
  bool pubOK = false;
  if (mqtt.connected()){
    pubOK = mqtt.publish(
      CFG_TOPIC.c_str(),
      (const uint8_t*)payload.c_str(),
      (unsigned int)payload.length(),
      /*retain=*/false
    );

    if (!pubOK) {
      connectMQTTOnce();   // one attempt; don't stall the tap path with retries
      if (mqtt.connected()){
        pubOK = mqtt.publish(
          CFG_TOPIC.c_str(),
          (const uint8_t*)payload.c_str(),
          (unsigned int)payload.length(),
          /*retain=*/false
        );
      }
    }
  }

  // Log JSON to SD (one JSON object per line) — forensic record, always
  bool sdOK = appendLineToSD(isoTime.substring(0,10), payload);
  purgeOldLogsIfNeeded();

  // Reader-side outbox: queue EVERY tap, unconditionally.
  //
  // This used to be `if (!pubOK)`, which lost taps: pubOK comes from a QoS 0
  // publish() that returns true once the payload hits the local socket, so a
  // tap could report "sent", never reach the broker, and never be queued —
  // gone, with only the daily CSV as evidence. pubOK is not proof and must
  // never gate the outbox. drainPending() removes the line once the broker
  // echoes it back; duplicates are absorbed by the bridge's uniqueness index.
  bool queued = appendPending(payload);

  // UI feedback: green if MQTT delivered OR safely queued; red only if
  // we lost the tap entirely (no MQTT + no SD).
  bool delivered = pubOK || queued;

  // Concise human-friendly banner: status + local HH:MM
  char hhmm[6] = "--:--";
  if (haveValidTime()){
    time_t nowT = time(nullptr);
    struct tm lt;
    localtime_r(&nowT, &lt);
    strftime(hhmm, sizeof(hhmm), "%H:%M", &lt);
  }
  String friendly;
  if      (pubOK)  friendly = String("Sent  ") + hhmm;
  else if (queued) friendly = String("Saved offline  ") + hhmm;
  else             friendly = String("FAILED  ") + hhmm;

  showScanBanner(delivered, friendly);
  if (delivered) beepOK(); else beepFail();
}



// ---------- Setup / Loop ----------
unsigned long lastUITick = 0;
unsigned long bannerShownAt = 0;
const uint32_t BANNER_MS = 2000;

void setup(){
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Speaker.setVolume(SPEAKER_VOLUME);
  buzzerBegin();   // external M5 Unit Buzzer on Port B (GPIO 26)
  M5.Display.setColorDepth(24);
  M5.Display.setTextWrap(false);

  uint64_t mac = ESP.getEfuseMac();
  snprintf(DEVICE_ID, sizeof(DEVICE_ID), "core2-%04X%08X",
           (uint16_t)(mac>>32), (uint32_t)mac);

  uiInitDisplay();

  // Start with both lights red and a placeholder clock
  ui.wifiOK = false;
  ui.mqttOK = false;
  drawHeader(ui);
  drawClock();     // will render placeholder until time is valid

  Wire.begin();  // Core2 Port A: SDA 21, SCL 22

  // SD
  initSD();

  // Seed the outbox depth from disk so the header is honest from the first
  // frame — a backlog survives a reboot and should still be visible.
  pendingCount = countPendingLines();

  // Load per-device config from SD (overrides defaults)
  loadConfigFromSD();

  // Finalise topic (from config) if not already set
  if (CFG_TOPIC.length() == 0) CFG_TOPIC = String(MQTT_TOPIC); // safety
  if (CFG_DEVICE_NAME.length())
    snprintf(DEVICE_ID, sizeof(DEVICE_ID), "%s", CFG_DEVICE_NAME.c_str());

  // WiFi + NTP + MQTT
  ui.wifiOK = connectWiFi();
  drawHeader(ui);  // reflect WiFi immediately

  // Set timezone & SNTP for local display (BST aware)
  setenv("TZ", TZ_LONDON, 1);
  tzset();
  configTzTime(TZ_LONDON, NTP_1, NTP_2);

  ui.mqttOK = connectMQTT();  // or your connectMQTTOnce() if you switched
  drawHeader(ui);             // reflect MQTT immediately

  // RFID
  mfrc522.PCD_Init();

  // First full paint pass
  uiUpdateAll();
}


// Poll the RFID reader; handle a tap if a card is present. Returns fast when
// there is no card. Kept free of any network work so it can run every tick.
void pollCard(){
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) return;

  String uidHex = toHex(mfrc522.uid.uidByte, mfrc522.uid.size);

  // Debounce repeat scans
  uint32_t nowMs = millis();
  bool repeat = (uidHex == lastUid) && (nowMs - lastUidMillis < RESCAN_BLOCK_MS);
  if (!repeat){
    lastUid = uidHex; lastUidMillis = nowMs;

    time_t now = time(nullptr);
    String ts = USE_ISO_TIME ? iso8601_utc(now) : String((unsigned long)now);

    publishAndLog(ts, uidHex);
    bannerShownAt = millis();
  }

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}

void loop(){
  M5.update();

  // Keep MQTT client serviced first, so .connected() is up to date
  mqtt.loop();

  // Always reflect *current* states (no stale values)
  ui.wifiOK = (WiFi.status() == WL_CONNECTED);
  ui.mqttOK = mqtt.connected();

  // Read cards FIRST. A tap must never wait on the network: this used to sit at
  // the bottom of loop(), behind a blocking reconnect, so during an MQTT outage
  // the reader was only polled once every ~5s and staff taps went unread.
  pollCard();

  // Try reconnect occasionally, but don't overwrite the UI flag here.
  // ONE attempt per tick (connectMQTTOnce, not connectMQTT): a single failed
  // attempt can still block for the socket timeout, so the retry gap is timed
  // from when the attempt RETURNS -- that guarantees a real window of
  // responsive card polling between attempts rather than back-to-back stalls.
  if (ui.wifiOK && !ui.mqttOK) {
    if (millis() - lastReconnectAttempt > 5000) {
      connectMQTTOnce();
      lastReconnectAttempt = millis();
    }
  }

  // Drain pending-queue on MQTT reconnect (false -> true edge) or
  // periodically while pending has content. Bounded work per tick so
  // RFID polling stays responsive.
  bool mqttUpNow = ui.mqttOK;
  bool justCameUp = mqttUpNow && !prevMqttUp;
  bool overdue = mqttUpNow && (millis() - lastDrainMillis > DRAIN_INTERVAL_MS);
  // An echo landed for something we hold — purge it promptly so Q:n reflects
  // reality within a second or two rather than up to 30s later.
  bool echoDue = mqttUpNow && drainScheduled
                 && (millis() - drainScheduledAt > ECHO_SETTLE_MS);
  if (justCameUp || echoDue || (overdue && pendingHasContent())) {
    drainPending();
    lastDrainMillis = millis();
    drainScheduled  = false;
  }
  prevMqttUp = mqttUpNow;

  // UI tick (1 Hz)
  if (millis() - lastUITick >= 1000){
    lastUITick = millis();
    uiUpdateAll();
  }

  // Clear banner after a moment
  if (bannerShownAt && (millis() - bannerShownAt > BANNER_MS)){
    clearScanBanner();
    bannerShownAt = 0;
  }

  // Card polling now happens at the top of loop() via pollCard().
  delay(20);
}
