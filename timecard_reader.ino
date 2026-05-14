// ========= User settings =========
const char* WIFI_SSID     = "PSS_Office";
const char* WIFI_PASSWORD = "P550ffice$";

const char* MQTT_HOST     = "10.0.0.180";
const uint16_t MQTT_PORT  = 1883;
const char* MQTT_USER     = "timecard";
const char* MQTT_PASS     = "letmein";

const char* MQTT_TOPIC    = "carrwood/timecard";  // publishes "time,cardid"

// ======== JSON meta ========
const char* DEVICE_ACTOR      = "timecard";   // one of: admin, test, harvester, timecard
const char* FIRMWARE_VERSION  = "0.7.0";      // bump as you release

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
struct HeaderCache { bool wifi; bool mqtt; int sdPct; bool init; };

HeaderCache hdrCache = { false, false, -1, false };
UiStatus    ui       = { false, false };

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

// Minimal-redraw cache for the clock
String prevTimeRendered = "";
String prevDateRendered = "";
int prevTimeW = 0, prevTimeH = 0;
int prevDateW = 0, prevDateH = 0;

// minute gating
int  lastClockMinute = -1;
bool drewPlaceholder  = false;

inline void beepOK(){
  M5.Speaker.tone(1200, 70);
  delay(30);
  M5.Speaker.tone(1600, 90);
}
inline void beepFail(){
  M5.Speaker.tone(220, 140);
  delay(20);
  M5.Speaker.tone(160, 180);
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
  if (!hdrCache.init || hdrCache.wifi != s.wifiOK || hdrCache.mqtt != s.mqttOK || hdrCache.sdPct != pct){
    hdrCache.wifi = s.wifiOK;
    hdrCache.mqtt = s.mqttOK;
    hdrCache.sdPct = pct;
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
  return true;
}

// Read pending.jsonl, try to publish each line, rewrite file with only
// the ones that failed. Returns the count delivered this pass.
// Notes:
//  - whole file is loaded into RAM during the rewrite — fine for normal
//    backlogs (a tap is ~160 bytes; 100 queued taps = ~16 KB)
//  - if we lose MQTT mid-drain, the remaining lines stay queued
//  - daily CSV is unaffected (forensic record)
int drainPending(){
  if (!sdReady) return 0;
  if (!mqtt.connected()) return 0;
  if (!SD.exists(PENDING_PATH)) return 0;

  File r = SD.open(PENDING_PATH, FILE_READ);
  if (!r) return 0;

  String unsent;
  unsent.reserve(2048);
  int sent = 0, kept = 0;

  while (r.available()){
    String line = r.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;

    bool ok = mqtt.connected() && mqtt.publish(
      CFG_TOPIC.c_str(),
      (const uint8_t*)line.c_str(),
      (unsigned int)line.length(),
      /*retain=*/false
    );
    if (ok) sent++;
    else { unsent += line; unsent += '\n'; kept++; }
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

  if (sent || kept){
    Serial.printf("drainPending: sent=%d kept=%d\n", sent, kept);
  }
  return sent;
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

bool connectMQTT(uint32_t timeout_ms=10000){
  if (WiFi.status()!=WL_CONNECTED) return false;

  mqtt.setServer(CFG_MQTT_HOST.c_str(), CFG_MQTT_PORT);
  mqtt.setKeepAlive(30);
  mqtt.setSocketTimeout(5);

  // Last Will & Testament (retained "offline")
  const String willTopic = CFG_TOPIC + "/status";
  const char*  willMsg   = "offline";
  const uint8_t willQos  = 1;
  const bool    willRetain = true;

  const uint32_t start = millis();
  while (!mqtt.connected() && (millis() - start) < timeout_ms) {
    String cid = String(DEVICE_ID);
    bool ok = mqtt.connect(
      cid.c_str(),
      CFG_MQTT_USER.c_str(),
      CFG_MQTT_PASS.c_str(),
      willTopic.c_str(),
      willQos,
      willRetain,
      willMsg
    );
    if (!ok) delay(1000);
  }

  if (mqtt.connected()) {
    // Birth message: retained "online"
    const char* online = "online";
    mqtt.publish(willTopic.c_str(), online, /*retain=*/true);
  }
  return mqtt.connected();
}



// ---------- Publish & Log (JSON) ----------
void publishAndLog(const String& isoTime, const String& uidHex){
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
      connectMQTT(3000);
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

  // Reader-side outbox: if MQTT couldn't accept the publish, queue for
  // drainPending() to retry when MQTT comes back.
  bool queued = false;
  if (!pubOK) {
    queued = appendPending(payload);
  }

  // UI feedback: green if MQTT delivered OR safely queued; red only if
  // we lost the tap entirely (no MQTT + no SD).
  bool delivered = pubOK || queued;
  showScanBanner(delivered, payload);
  if (delivered) beepOK(); else beepFail();
}



// ---------- Setup / Loop ----------
unsigned long lastUITick = 0;
unsigned long bannerShownAt = 0;
const uint32_t BANNER_MS = 2000;

void setup(){
  auto cfg = M5.config();
  M5.begin(cfg);
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


void loop(){
  M5.update();

  // Keep MQTT client serviced first, so .connected() is up to date
  mqtt.loop();
  
  // Always reflect *current* states (no stale values)
  ui.wifiOK = (WiFi.status() == WL_CONNECTED);
  ui.mqttOK = mqtt.connected();
  
  // Try reconnect occasionally, but don't overwrite the UI flag here
  if (ui.wifiOK && !ui.mqttOK) {
    uint32_t now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      connectMQTT(4000);  // attempt; ui.mqttOK will flip to true on next loop if it succeeds
    }
  }

  // Drain pending-queue on MQTT reconnect (false -> true edge) or
  // periodically while pending has content. Bounded work per tick so
  // RFID polling stays responsive.
  bool mqttUpNow = ui.mqttOK;
  bool justCameUp = mqttUpNow && !prevMqttUp;
  bool overdue = mqttUpNow && (millis() - lastDrainMillis > DRAIN_INTERVAL_MS);
  if (justCameUp || (overdue && pendingHasContent())) {
    drainPending();
    lastDrainMillis = millis();
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

  // Card present?
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()){
    delay(20);
    return;
  }

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
  delay(20);
}
