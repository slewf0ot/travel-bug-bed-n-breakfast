/*  ESP32 DOIT DevKit V1 + OLED SSD1306 + Elechouse PN532 (all on Wire I2C)
    Night-only sleep:
      - We *only* deep sleep during quiet hours (22:00 → 07:00 by default).
      - PN532 is held in reset at night (RSTPDN LOW), no IRQ wake at night.
    Daytime:
      - No deep sleep (prevents 60s timer cycles and NACK spam).
      - PN532 is up; short, throttled polls.

    Wiring:
      I2C (Wire): SDA=21, SCL=22 (OLED 0x3C + PN532 share the bus)
      PN532 IRQ     -> GPIO33  (RTC-capable, but unused at night)
      PN532 RSTPDN  -> GPIO32  (we pull LOW at night to power down PN532)

    Serial command set (type 'help'):
      show
      list | add <UIDHEX> | remove <UIDHEX> | clear
      active_ms <ms> | welcome_ms <ms> | reject_ms <ms> | marquee_ms <ms> | code_delay_ms <ms>
      unlock <4digits>
      quiet on|off [start end]
      time | ntp
      sleepdiag | irq | sleepnow
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <sys/time.h>   // settimeofday


#include <PN532_I2C.h>
#include <PN532.h>              // Elechouse PN532
#include <Preferences.h>

#include <esp_sleep.h>
#include <driver/rtc_io.h>

#include <WiFi.h>
#include <time.h>

#include "esp_log.h"
#include "slewfoot_sign_128x64.h"  // provides slewfoot_sign_128x64_bits/width/height

// Forward decl for quietNow()
bool inQuietHours();

// ---------------- Pins / Bus ----------------
#define OLED_SDA   21
#define OLED_SCL   22
#define PN532_IRQ     33
#define PN532_RSTPDN  32

// ---------------- Objects ----------------
Adafruit_SSD1306 display(128, 64, &Wire, -1);
PN532_I2C  pn532_i2c(Wire);
PN532      nfc(pn532_i2c);

// ---------------- Global flags ----------------
static bool gNfcUp = false;      // true when PN532 out of reset and configured
static bool timeValid = false;   // set once we successfully get time (if quiet enabled)

// ---------------- Preferences ----------------
Preferences prefs;
static const char* PREF_NS = "tbbnb";

// ---------------- Settings (persisted) ----------------
uint32_t SET_ACTIVE_MS     = 120000; // OLED blanks after last scan
uint32_t SET_WELCOME_MS    = 3000;
uint32_t SET_REJECT_MS     = 3000;
uint16_t SET_MARQUEE_MS    = 35;
uint32_t SET_CODE_DELAY_MS = 1500;
uint32_t SET_SLEEP_MS       = 180000;  // sleep after 3 min idle (day)
uint32_t SET_DAY_FALLBACK_MS = 0;      // 0 = disabled; set >0 only if you want periodic wake

String   UNLOCK_CODE       = "3510";

// Quiet hours (night sleep)
bool     QUIET_EN          = true;
uint8_t  QUIET_START_H     = 22;
uint8_t  QUIET_END_H       = 7;

// Optional WiFi for NTP (only needed if QUIET_EN true)
const char* WIFI_SSID      = "";
const char* WIFI_PASSWORD  = "";
String SET_TZ              = "EST5EDT,M3.2.0/2,M11.1.0/2";

// ---- Last tag tracking ----
String   lastUidHex = "";
uint8_t  lastUidLen = 0;
bool     lastWasRecognized = false;   // whether the *last* read matched allowlist
String   lastGrantedHex = "";         // last UID that actually unlocked

// Survives deep sleep resets (lives in RTC slow memory)
RTC_DATA_ATTR time_t gPlannedWakeEpoch = 0;   // when we *intended* to wake
RTC_DATA_ATTR uint32_t gLastSyncEpoch = 0;    // last time we successfully synced (optional telemetry)

// ---------------- Allowlist ----------------
const uint8_t MAX_UIDS = 16;
String allowUids[MAX_UIDS];
uint8_t allowCount = 0;
#define ALLOW_WHEN_EMPTY true

// ---------------- State ----------------
enum Mode { MODE_SLEEP, MODE_SIGN, MODE_WELCOME, MODE_CODE, MODE_POOP };
Mode mode = MODE_SIGN;

bool displayAwake=false;
uint32_t lastScanMs=0, modeUntilMs=0;
uint32_t lastSignTick=0; uint8_t antsPhase=0;
uint8_t poopPhase=0; uint32_t lastPoopTick=0; const uint16_t POOP_FRAME_MS=160;
bool recognizedLast=false;
uint32_t codeShowAtMs = 0;  // when to switch WELCOME -> CODE

// ---- Admin/NDEF forward decls (place near top) ----
struct NdefResult { bool ok=false; bool isText=false; bool isUri=false; String text; };
struct AdminResult { bool ok=false; String msg; };

NdefResult readNdefUltralight();
AdminResult applyConfigText(String txt);
void applyKV(String k, String v, AdminResult &ar);
void showAdminFeedback(const AdminResult& ar);


// ---------------- Small log helper ----------------
static inline void logf(const char* fmt, ...) {
  char b[256];
  va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof(b), fmt, ap); va_end(ap);
  Serial.print(b);
}

void pn532HardReset() {
  pinMode(PN532_RSTPDN, OUTPUT);
  digitalWrite(PN532_RSTPDN, LOW);
  delay(50);   // hold low
  digitalWrite(PN532_RSTPDN, HIGH);
  delay(400);  // give it ample time to boot (datasheet-friendly)
}

bool i2cBusClear(int scl = OLED_SCL, int sda = OLED_SDA) {
pinMode(scl, INPUT_PULLUP);
pinMode(sda, INPUT_PULLUP);

if (digitalRead(sda) == LOW) {
  // Toggle SCL to free a wedged target
  pinMode(scl, OUTPUT);
  for (int i = 0; i < 16 && digitalRead(sda) == LOW; i++) {
    digitalWrite(scl, LOW);  delayMicroseconds(5);
    digitalWrite(scl, HIGH); delayMicroseconds(5);
  }
  // Send a STOP
  pinMode(sda, OUTPUT);
  digitalWrite(sda, LOW);  delayMicroseconds(5);
  digitalWrite(scl, HIGH); delayMicroseconds(5);
  digitalWrite(sda, HIGH); delayMicroseconds(5);
}
return digitalRead(sda) == HIGH;
}

bool pn532TryInit() {
  for (int i = 0; i < 5; i++) {
    pn532HardReset();         // proper reset+settle
    nfc.begin();
    uint32_t v = nfc.getFirmwareVersion();
    if (v) {
      logf("[PN532] IC=0x%02X FW=%u.%u\n", (v>>24)&0xFF, (v>>16)&0xFF, (v>>8)&0xFF);
      nfc.SAMConfig();
      nfc.setPassiveActivationRetries(0x20);
      return true;
    }
    delay(80);
  }
  return false;
}


void i2cScan() {
  Serial.println("I2C scan:");
  for (uint8_t a = 0x08; a < 0x78; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) Serial.printf("  0x%02X\n", a);
  }
}

void pnInfo() {
  uint32_t v = nfc.getFirmwareVersion();
  if (v) Serial.printf("PN532 OK IC=0x%02X FW=%u.%u\n",(v>>24)&0xFF,(v>>16)&0xFF,(v>>8)&0xFF);
  else   Serial.println("PN532 not responding");
}

static void setSystemTimeFromEpoch(time_t epoch, const char* tag){
  if (epoch <= 0) return;
  struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
  settimeofday(&tv, nullptr);
  setenv("TZ", SET_TZ.c_str(), 1); tzset();
  timeValid = true;
  tm tmp{}; 
  if (getLocalTime(&tmp)) {
    char b[40]; strftime(b,sizeof(b),"%F %T",&tmp);
    Serial.print("[TIME] set from "); Serial.print(tag); Serial.print(": "); Serial.println(b);
  } else {
    Serial.println("[TIME] set from fallback epoch (no local time yet)");
  }
}

// ---------------- Helpers: UID formatting ----------------
String uidHexNoSep(const uint8_t* uid, uint8_t len){
  static const char* H="0123456789ABCDEF";
  String s; s.reserve(len*2);
  for(uint8_t i=0;i<len;i++){ s+=H[(uid[i]>>4)&0xF]; s+=H[uid[i]&0xF]; }
  return s;
}
bool inAllowlist(const String& h){
  String s=h; s.replace(" ",""); s.toUpperCase();
  for(uint8_t i=0;i<allowCount;i++) if(allowUids[i]==s) return true;
  return false;
}
bool addAllow(const String& hex){
  if(allowCount>=MAX_UIDS) return false;
  String h=hex; h.replace(" ",""); h.toUpperCase();
  if(inAllowlist(h)) return true;
  allowUids[allowCount++]=h; return true;
}
bool removeAllow(const String& hex){
  String h=hex; h.replace(" ",""); h.toUpperCase();
  for(uint8_t i=0;i<allowCount;i++){
    if(allowUids[i]==h){
      for(uint8_t j=i+1;j<allowCount;j++) allowUids[j-1]=allowUids[j];
      allowCount--; return true;
    }
  }
  return false;
}
void clearAllow(){ allowCount=0; }

// ---------------- Persistence ----------------
void saveAll(){
  prefs.begin(PREF_NS,false);
  prefs.putUInt("active_ms",SET_ACTIVE_MS);
  prefs.putUInt("welcome_ms",SET_WELCOME_MS);
  prefs.putUInt("reject_ms", SET_REJECT_MS);
  prefs.putUInt("marquee_ms",SET_MARQUEE_MS);
  prefs.putUInt("code_delay",SET_CODE_DELAY_MS);
  prefs.putString("code", UNLOCK_CODE);
  prefs.putUInt("sleep_ms", SET_SLEEP_MS);
  prefs.putUInt("day_fallback_ms", SET_DAY_FALLBACK_MS);
  prefs.putString("tz", SET_TZ);

  prefs.putBool("quiet_en", QUIET_EN);
  prefs.putUChar("q_start", QUIET_START_H);
  prefs.putUChar("q_end",   QUIET_END_H);

  prefs.putUChar("count", allowCount);
  for(uint8_t i=0;i<allowCount;i++){
    String k = "uid"+String(i);
    prefs.putString(k.c_str(), allowUids[i]);
  }
  prefs.end();
}
void loadAll(){
  prefs.begin(PREF_NS,true);
  SET_ACTIVE_MS     = prefs.getUInt("active_ms",SET_ACTIVE_MS);
  SET_WELCOME_MS    = prefs.getUInt("welcome_ms",SET_WELCOME_MS);
  SET_REJECT_MS     = prefs.getUInt("reject_ms", SET_REJECT_MS);
  SET_MARQUEE_MS    = (uint16_t)prefs.getUInt("marquee_ms",SET_MARQUEE_MS);
  SET_CODE_DELAY_MS = prefs.getUInt("code_delay",SET_CODE_DELAY_MS);
  UNLOCK_CODE       = prefs.getString("code", UNLOCK_CODE);
  SET_SLEEP_MS      = prefs.getUInt("sleep_ms", SET_SLEEP_MS);
  SET_DAY_FALLBACK_MS = prefs.getUInt("day_fallback_ms", SET_DAY_FALLBACK_MS);
  SET_TZ            = prefs.getString("tz", "EST5EDT,M3,2.0/2,M11.1.0/2"); setenv("TZ", SET_TZ.c_str(), 1); tzset();
  QUIET_EN       = prefs.getBool("quiet_en", QUIET_EN);
  QUIET_START_H  = prefs.getUChar("q_start", QUIET_START_H);
  QUIET_END_H    = prefs.getUChar("q_end",   QUIET_END_H);

  allowCount = prefs.getUChar("count", 0); if(allowCount>MAX_UIDS) allowCount=MAX_UIDS;
  for(uint8_t i=0;i<allowCount;i++){
    String k="uid"+String(i);
    allowUids[i]=prefs.getString(k.c_str(),""); allowUids[i].toUpperCase();
  }
  prefs.end();
}

// ---------------- Display ----------------
static inline void displayWake(){ if(!displayAwake){ display.ssd1306_command(SSD1306_DISPLAYON); displayAwake=true; } }
static inline void displaySleep(){ if(displayAwake){ display.ssd1306_command(SSD1306_DISPLAYOFF); displayAwake=false; } }

void drawAntsBorder(uint8_t p){
  for(int x=(p%4); x<128; x+=4) display.drawFastHLine(x,0,2,SSD1306_WHITE);
  for(int x=((p+2)%4); x<128; x+=4) display.drawFastHLine(x,63,2,SSD1306_WHITE);
  for(int y=((p+1)%4); y<64; y+=4) display.drawFastVLine(0,y,2,SSD1306_WHITE);
  for(int y=((p+3)%4); y<64; y+=4) display.drawFastVLine(127,y,2,SSD1306_WHITE);
}
void drawSignFrame(){
  if(!displayAwake) return;
  uint32_t now=millis(); if(now-lastSignTick<SET_MARQUEE_MS) return; lastSignTick=now;
  display.clearDisplay();
  int w=(int)slewfoot_sign_128x64_width, h=(int)slewfoot_sign_128x64_height;
  int x=(128-w)/2, y=(64-h)/2;
  display.drawXBitmap(x,y,(const uint8_t*)slewfoot_sign_128x64_bits,w,h,SSD1306_WHITE);
  drawAntsBorder(antsPhase); antsPhase=(antsPhase+1)&3;
  display.display();
}
void drawWelcomeFrame(){
  if(!displayAwake) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(12,10); display.println("Welcome");
  display.setCursor(40,34); display.println("Home");
  drawAntsBorder(antsPhase); display.display();
}
void drawCodeFrame(){
  if(!displayAwake) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(10,10); display.println("Welcome Home");
  display.setTextSize(2);
  display.setCursor(12,34); display.print("CODE:");
  display.print(UNLOCK_CODE);
  drawAntsBorder(antsPhase); display.display();
}
void drawPoopFrame(){
  if(!displayAwake) return;
  uint32_t now=millis(); if(now-lastPoopTick<POOP_FRAME_MS) return; lastPoopTick=now; poopPhase=(poopPhase+1)%4;
  display.clearDisplay();
  for(int i=0;i<3;i++){ int y0=8+i*10+(poopPhase%2?1:-1); display.drawPixel(20,y0,SSD1306_WHITE); display.drawPixel(21,y0-1,SSD1306_WHITE); display.drawPixel(22,y0-2,SSD1306_WHITE); }
  for(int i=0;i<3;i++){ int y0=5+i*10+((poopPhase+1)%2?1:-1); display.drawPixel(64,y0,SSD1306_WHITE); display.drawPixel(65,y0-1,SSD1306_WHITE); display.drawPixel(66,y0-2,SSD1306_WHITE); }
  for(int i=0;i<3;i++){ int y0=10+i*10+(poopPhase%2?-1:1); display.drawPixel(104,y0,SSD1306_WHITE); display.drawPixel(105,y0-1,SSD1306_WHITE); display.drawPixel(106,y0-2,SSD1306_WHITE); }
  display.fillTriangle(40,52,88,52,64,28,SSD1306_WHITE);
  display.fillTriangle(32,60,96,60,64,36,SSD1306_WHITE);
  display.fillRoundRect(28,58,72,12,4,SSD1306_WHITE);
  display.fillRect(50,50,6,4,SSD1306_BLACK);
  display.fillRect(72,50,6,4,SSD1306_BLACK);
  display.fillRoundRect(56,55,16,3,1,SSD1306_BLACK);
  drawAntsBorder(antsPhase); display.display();
}

// ---------------- Raw I²C helpers to arm PN532 AutoPoll (unused at night) -------------
static const uint8_t PN532_I2C_ADDR = 0x24; // 7-bit
static inline uint8_t csum8(uint8_t x){ return (uint8_t)(~x + 1); }

bool pn532WaitReadyI2C(uint16_t timeout_ms){
  uint32_t t0 = millis();
  while ((millis() - t0) < timeout_ms){
    Wire.requestFrom(PN532_I2C_ADDR, (uint8_t)1);
    if (Wire.available() && Wire.read() == 0x01) return true;
    delay(5);
  }
  return false;
}
static inline bool pn532IsReadyI2C(){
  Wire.requestFrom(PN532_I2C_ADDR, (uint8_t)1);
  return (Wire.available() && Wire.read()==0x01);
}
int readHeader7(uint8_t out6[6], uint8_t &lead){
  uint8_t raw[7]={0}; int n=0;
  Wire.requestFrom(PN532_I2C_ADDR, (uint8_t)7);
  while (Wire.available() && n<7) raw[n++]=Wire.read();
  if(n<6) return 0;
  lead = (n==7)?raw[0]:0xFF;
  const uint8_t* p = (n==7 && raw[0]==0x01) ? &raw[1] : raw;
  for(int i=0;i<6;i++) out6[i]=p[i];
  return 6;
}
uint8_t pn532DrainOneFrame(){
  if (!pn532IsReadyI2C()) return 0;
  uint8_t hdr[6], lead=0;
  if (readHeader7(hdr, lead) != 6) return 0;
  if (hdr[0]==0x00 && hdr[1]==0x00 && hdr[2]==0xFF && hdr[3]==0x00 && hdr[4]==0xFF && hdr[5]==0x00) return 1; // ACK
  if (!(hdr[0]==0x00 && hdr[1]==0x00 && hdr[2]==0xFF)) return 0;
  uint8_t LEN = hdr[3], LCS = hdr[4]; if ((uint8_t)(LEN + LCS) != 0x00) return 0;
  uint8_t remain = (uint8_t)(LEN + 1);
  Wire.requestFrom(PN532_I2C_ADDR, remain); while (Wire.available()) (void)Wire.read();
  return 2;
}
void pn532DrainAllReady(uint16_t budget_ms = 300){
  uint32_t t0 = millis();
  while (millis()-t0 < budget_ms){ if (pn532DrainOneFrame()==0) break; delay(2); }
}
bool pn532WaitNotReadyStable(uint16_t quiet_ms = 150, uint16_t timeout_ms = 2000){
  uint32_t t0 = millis(), quietStart = 0;
  while (millis() - t0 < timeout_ms){
    if (!pn532IsReadyI2C()){
      if (quietStart == 0) quietStart = millis();
      if (millis() - quietStart >= quiet_ms) return true;
    } else { pn532DrainAllReady(120); quietStart = 0; }
    delay(3);
  }
  return false;
}
bool pn532SendCommandI2C(const uint8_t* data, uint8_t len){
  const uint8_t LEN = (uint8_t)(len + 1), LCS = csum8(LEN), TFI = 0xD4;
  uint8_t sum = TFI; for (uint8_t i=0;i<len;i++) sum += data[i];
  const uint8_t DCS = csum8(sum);

  Wire.beginTransmission(PN532_I2C_ADDR);
  Wire.write((uint8_t)0x00); Wire.write((uint8_t)0x00); Wire.write((uint8_t)0xFF);
  Wire.write(LEN); Wire.write(LCS); Wire.write(TFI); Wire.write(data, len); Wire.write(DCS); Wire.write((uint8_t)0x00);
  uint8_t err = Wire.endTransmission();
  if (err != 0){ logf("[PN532] I2C write err=%u\n", err); return false; }

  if (!pn532WaitReadyI2C(150)){ logf("[PN532] no ACK ready\n"); return false; }
  uint8_t hdr[6], lead=0; if (readHeader7(hdr, lead) != 6){ logf("[PN532] short ACK read\n"); return false; }
  bool ok = (hdr[0]==0x00 && hdr[1]==0x00 && hdr[2]==0xFF && hdr[3]==0x00 && hdr[4]==0xFF && hdr[5]==0x00);
  if (!ok){ logf("[PN532] bad ACK %02X %02X %02X %02X %02X %02X (lead=%02X)\n", hdr[0],hdr[1],hdr[2],hdr[3],hdr[4],hdr[5], lead); }
  return ok;
}
// 0x60 InAutoPoll: PollNr=0xFF, Period=0x01, TypeA(106)=0x00
bool pn532StartAutoPoll106A(){
  const uint8_t payload[] = { 0x60, 0xFF, 0x01, 0x00 };
  bool ok = pn532SendCommandI2C(payload, sizeof(payload));
  logf("[PN532] InAutoPoll start %s\n", ok?"OK":"FAIL");
  return ok;
}

// --- RTC hold helpers (keep lines quiet in deep sleep) ---
static inline void rtcHoldLevel(gpio_num_t pin, int level, bool pullup=true) {
  // Only works for RTC-capable GPIOs
  if (!rtc_gpio_is_valid_gpio(pin)) return;
  rtc_gpio_init(pin);
  rtc_gpio_set_direction(pin, RTC_GPIO_MODE_INPUT_ONLY); // input with pullup to hold level
  if (pullup) { rtc_gpio_pullup_en(pin); rtc_gpio_pulldown_dis(pin); }
  else        { rtc_gpio_pullup_dis(pin); rtc_gpio_pulldown_en(pin); }
  // Forcing level on input pins uses pull resistors; if you need output-hold, switch to OUTPUT_ONLY then gpio_hold.
  // Here we just want IRQ to stay HIGH via pullup so it doesn't chatter.
}

static inline void disableAllWakesExceptTimer() {
  // Make sure no leftover wake sources remain armed
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT0);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT1);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
  // (Timer will be enabled explicitly below)
}


void enterDeepSleep_DayIRQ(){
  // OLED off
  displaySleep();

  // Make sure PN532 is up to arm AutoPoll
  pn532PowerUp();
  // Start autonomous polling (will pull IRQ LOW on a tag)
  bool autoOK = pn532StartAutoPoll106A();
  if (autoOK){ pn532DrainAllReady(400); pn532WaitNotReadyStable(150,2000); }

  // Arm EXT0 on LOW (IRQ active-LOW)
  if (rtc_gpio_is_valid_gpio((gpio_num_t)PN532_IRQ)) {
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PN532_IRQ, 0);
  }

  // Optional safety timer (usually leave 0 to avoid cyclic wakes)
  if (SET_DAY_FALLBACK_MS > 0) {
    esp_sleep_enable_timer_wakeup((uint64_t)SET_DAY_FALLBACK_MS * 1000ULL);
  }

  // Radios off
  WiFi.disconnect(true,true);
  WiFi.mode(WIFI_OFF);

  delay(30);
  esp_deep_sleep_start();
}


// ---------------- PN532 power control ----------------
void pn532PowerDown(){  // hold in reset
  digitalWrite(PN532_RSTPDN, LOW);
  gNfcUp = false;
}
void pn532PowerUp(){
  digitalWrite(PN532_RSTPDN, HIGH);
  delay(10);
  gNfcUp = false; // will become true after init
}

// ---------------- Time helpers (quiet hours) ----------------
bool ensureTimeSync(uint32_t wifiMs=10000, uint32_t ntpMs=10000, bool force=false, bool verbose=true){
  // Try even if quiet is disabled if 'force' is true
  if (!force && !QUIET_EN) {
    if (verbose) Serial.println("[NTP] skipped (quiet disabled)");
    return false;
  }

  if (verbose) Serial.println("[NTP] start");

  // Bring WiFi up just for sync, then turn it back off
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(50);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t t0 = millis();
  while (WiFi.status()!=WL_CONNECTED && millis()-t0 < wifiMs) {
    delay(100);
  }
  if (WiFi.status()!=WL_CONNECTED) {
    if (verbose) Serial.println("[NTP] WiFi connect failed");
    WiFi.mode(WIFI_OFF);
    return false;
  }

  if (verbose) {
    Serial.print("[NTP] WiFi OK, IP="); Serial.println(WiFi.localIP());
    Serial.print("[NTP] RSSI="); Serial.println(WiFi.RSSI());
    Serial.println("[NTP] DNS test...");
  }

  IPAddress ip;
  if (!WiFi.hostByName("pool.ntp.org", ip)) {
    if (verbose) Serial.println("[NTP] DNS lookup failed");
    WiFi.disconnect(true, true); WiFi.mode(WIFI_OFF);
    return false;
  } else if (verbose) {
    Serial.print("[NTP] pool.ntp.org -> "); Serial.println(ip);
  }

  // Multiple servers; any one can succeed
  configTime(0,0,"time.google.com","time.nist.gov","pool.ntp.org");
  setenv("TZ", SET_TZ.c_str(), 1); tzset();

  tm tmp{}; uint32_t t1 = millis();
  timeValid = false;
  while (millis()-t1 < ntpMs){
    if (getLocalTime(&tmp)) {
      timeValid = true;
      gLastSyncEpoch = time(nullptr);
      if (verbose) {
        char b[40]; strftime(b,sizeof(b),"%F %T",&tmp);
        Serial.print("[NTP] synced "); Serial.println(b);
      }
      break;
    }
    delay(100);
  }

  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);

  if (!timeValid && verbose) Serial.println("[NTP] timed out");
  return timeValid;
}

bool inQuietHours(){
  if(!QUIET_EN) return false;
  tm now{}; if(!getLocalTime(&now)) return false;
  uint8_t h=now.tm_hour;
  if (QUIET_START_H == QUIET_END_H) return true;
  if (QUIET_START_H < QUIET_END_H)  return (h>=QUIET_START_H && h<QUIET_END_H);
  return (h>=QUIET_START_H || h<QUIET_END_H);
}
static inline bool quietNow(){
  return (QUIET_EN && timeValid && inQuietHours());
}
uint64_t microsUntilHour(uint8_t H){
  tm now{}; if(!getLocalTime(&now)) return 0;
  time_t tnow=mktime(&now);
  tm next=now; next.tm_hour=H; next.tm_min=0; next.tm_sec=0;
  time_t tnext=mktime(&next); if (tnext<=tnow) tnext+=24*3600;
  return (uint64_t)(tnext-tnow)*1000000ULL;
}


// Read a short NDEF from Mifare Ultralight / NTAG (pages 4..)
// Minimal TLV + TEXT/URI (SR) parser; enough for admin config tags.
NdefResult readNdefUltralight() {
  NdefResult r;
  // Read up to ~16 pages (64B) starting at page 4
  uint8_t buf[64]; memset(buf, 0, sizeof(buf));
  uint8_t pageBuf[4];
  uint8_t idx = 0;
  for (uint8_t p=4; p<4+16; ++p) {
    if (!nfc.mifareultralight_ReadPage(p, pageBuf)) return r;
    if (idx+4 <= sizeof(buf)) { memcpy(buf+idx, pageBuf, 4); idx+=4; }
  }

  // TLV scan
  uint16_t i=0;
  while (i+1 < idx) {
    uint8_t tlv = buf[i++];
    if (tlv==0x00) continue;            // NULL TLV
    if (tlv==0xFE) break;               // Terminator
    if (tlv!=0x03) {                    // not NDEF TLV
      // Skip length + value
      if (i>=idx) break;
      uint16_t len = buf[i++];
      if (len==0xFF) { if (i+2>idx) break; len = ((uint16_t)buf[i]<<8)|buf[i+1]; i+=2; }
      i += len;
      continue;
    }
    // NDEF TLV length
    if (i>=idx) break;
    uint16_t nlen = buf[i++];
    if (nlen==0xFF) { if (i+2>idx) break; nlen = ((uint16_t)buf[i]<<8)|buf[i+1]; i+=2; }
    if (i+nlen > idx) break;

    // NDEF record (assume single SR well-known)
    uint16_t j = i;
    if (j+2 > idx) break;
    uint8_t hdr = buf[j++];             // expect 0xD1 (MB|ME|SR|TNF=1)
    uint8_t tlen= buf[j++];             // type length
    uint8_t plen= buf[j++];             // payload length (SR)
    if (j+tlen+plen > i+nlen) break;
    const uint8_t* type = buf + j;      j += tlen;
    const uint8_t* pay  = buf + j;      // payload

    if (tlen==1 && type[0]=='T' && plen>=1) { // TEXT
      uint8_t status = pay[0];
      uint8_t langlen = status & 0x3F;
      if (1+langlen <= plen) {
        const char* txt = (const char*)(pay + 1 + langlen);
        uint8_t tlen2 = plen - 1 - langlen;
        r.ok=true; r.isText=true; r.text.reserve(tlen2);
        for (uint8_t k=0;k<tlen2;k++) r.text += (char)txt[k];
        return r;
      }
    } else if (tlen==1 && type[0]=='U' && plen>=1) { // URI (optional)
      r.ok=true; r.isUri=true;
      r.text = String((const char*)(pay+1), plen-1);
      return r;
    }
    break;
  }
  return r;
}

// Apply one key=value pair
void showAdminFeedback(const AdminResult& ar){
  displayWake();
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(4, 16);
  display.println(ar.ok ? "ADMIN OK" : "ADMIN ERR");
  display.setCursor(4, 32);
  display.println(ar.msg.length() ? ar.msg : (ar.ok ? "saved" : "parse"));
  display.display();
  delay(1200);
}

void applyKV(String k, String v, AdminResult &ar){
  k.toLowerCase(); v.trim();
  if (k=="active_ms")          { SET_ACTIVE_MS = (uint32_t)max( (long)10000, v.toInt()); ar.msg="active_ms"; }
  else if (k=="welcome_ms")    { SET_WELCOME_MS = (uint32_t)max( (long)200, v.toInt());  ar.msg="welcome_ms"; }
  else if (k=="reject_ms")     { SET_REJECT_MS  = (uint32_t)max( (long)200, v.toInt());  ar.msg="reject_ms"; }
  else if (k=="marquee_ms")    { long x=v.toInt(); if(x<10) x=10; if(x>65535) x=65535; SET_MARQUEE_MS=(uint16_t)x; ar.msg="marquee_ms"; }
  else if (k=="code_delay_ms") { long x=v.toInt(); if(x<0) x=0; SET_CODE_DELAY_MS=(uint32_t)x; ar.msg="code_delay_ms"; }
  else if (k=="code")          { if(v.length()==4){ UNLOCK_CODE=v; ar.msg="code"; } else { ar.ok=false; ar.msg="code len"; } }
  else if (k=="quiet")         { v.toLowerCase(); QUIET_EN = (v=="1"||v=="on"||v=="true"); ar.msg = (QUIET_EN?"quiet:on":"quiet:off"); }
  else if (k=="qstart")        { QUIET_START_H = (uint8_t)constrain(v.toInt(),0,23); ar.msg="qstart"; }
  else if (k=="qend")          { QUIET_END_H   = (uint8_t)constrain(v.toInt(),0,23); ar.msg="qend"; }
  else if (k=="add")           { if(addAllow(v)){ ar.msg="add ok"; } else { ar.ok=false; ar.msg="add fail"; } }
  else if (k=="remove")        { if(removeAllow(v)){ ar.msg="remove ok"; } else { ar.ok=false; ar.msg="remove nf"; } }
  else if (k=="clear")         { if(v=="1"||v=="true"){ clearAllow(); ar.msg="cleared"; } else { ar.ok=false; ar.msg="clear?"; } }
  else if (k=="sleep_ms")      { long x=v.toInt(); if(x<30000) x=30000; SET_SLEEP_MS=(uint32_t)x; ar.msg="sleep_ms"; }
  else if (k=="day_fallback_ms"){ long x=v.toInt(); if(x<0) x=0; SET_DAY_FALLBACK_MS=(uint32_t)x; ar.msg="day_fallback_ms"; }
    else if (k=="datetime") { // e.g. datetime=2025-11-11 21:03:00 or 2025-11-11T21:03:00
    if (setClockFromString(v)){ ar.msg="datetime"; }
    else { ar.ok=false; ar.msg="datetime parse"; }
  }
  else if (k=="epoch") {    // e.g. epoch=1731364980
    long long e = strtoll(v.c_str(), nullptr, 10);
    if (setClockFromEpoch((time_t)e)){ ar.msg="epoch"; }
    else { ar.ok=false; ar.msg="epoch bad"; }
  }
  else if (k=="tz") { 
    if (v.length()>2) { SET_TZ=v; setenv("TZ", SET_TZ.c_str(), 1); tzset(); ar.msg="tz"; } 
    else { ar.ok=false; ar.msg="tz invalid"; }
  }


  else { ar.ok=false; ar.msg = "unk key: "+k; }
}

AdminResult applyConfigText(String txt){
  AdminResult ar; ar.ok=true; ar.msg="saved";
  // split on newlines / semicolons / commas
  txt.replace("\r",""); txt.replace(";", "\n"); txt.replace(",", "\n");
  int start=0;
  while (start < (int)txt.length()){
    int nl = txt.indexOf('\n', start);
    String line = (nl>=0)? txt.substring(start,nl) : txt.substring(start);
    start = (nl>=0)? (nl+1) : txt.length();
    line.trim();
    if(!line.length()) continue;
    int eq = line.indexOf('=');
    if (eq<0){ ar.ok=false; ar.msg="no '='"; continue; }
    String k=line.substring(0,eq); String v=line.substring(eq+1);
    k.trim(); v.trim();
    applyKV(k,v,ar);
  }
  saveAll();
  return ar;
}

// ---------------- Sleep entries (night only) ----------------
void enterDeepSleep_NightTimer(){
  // Turn off OLED/PN532
  displaySleep();
  pn532PowerDown();

  // Radios OFF
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);

  // Don’t inherit daytime wakes
  disableAllWakesExceptTimer();

  // Keep IRQ quiet during deep sleep (pull HIGH)
  rtcHoldLevel((gpio_num_t)PN532_IRQ, 1 /*HIGH*/, true /*pullup*/);

  // Try to have time; if not, we’ll still pick a reasonable nap
  if (!timeValid) (void)ensureTimeSync(4000, 2500, /*force=*/true, /*verbose=*/true);

  uint64_t us = 0;
  time_t nowEpoch = time(nullptr);

  if (timeValid) {
    // When time is valid, schedule precise wake to QUIET_END_H
    uint64_t toEnd = microsUntilHour(QUIET_END_H);
    const uint64_t ONE_MIN  = 60ULL * 1000000ULL;
    if (toEnd < ONE_MIN) toEnd = ONE_MIN; // avoid thrash near boundary

    us = toEnd;
    gPlannedWakeEpoch = nowEpoch + (time_t)(us / 1000000ULL);   // <<< store planned wake time
  } else {
    // No time: nap ~30 minutes and try again next boot
    const uint64_t THIRTY_MIN = 30ULL * 60ULL * 1000000ULL;
    us = THIRTY_MIN;
    gPlannedWakeEpoch = 0; // unknown wake epoch in this case
  }

  logf("[SLEEP] Night: timer set (%llu us). plannedWake=%ld\n",
       (unsigned long long)us, (long)gPlannedWakeEpoch);

  esp_sleep_enable_timer_wakeup(us);

  delay(40); // let USB prints flush
  esp_deep_sleep_start();
}


// Small OLED toast line (1–2s)
static void oledToast(const char* line1, const char* line2 = nullptr, uint16_t ms=1200){
  displayWake();
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(6, 20); display.println(line1 ? line1 : "");
  if (line2){ display.setCursor(6, 34); display.println(line2); }
  display.display();
  delay(ms);
}

// Set RTC from UNIX epoch seconds
static bool setClockFromEpoch(time_t epoch){
  if (epoch <= 0) return false;
  struct timeval tv; tv.tv_sec = epoch; tv.tv_usec = 0;
  settimeofday(&tv, nullptr);
  setenv("TZ", SET_TZ.c_str(), 1); tzset();   // apply TZ to localtime conversion
  timeValid = true;
  return true;
}

// Parse "YYYY-MM-DD HH:MM:SS" or "YYYY-MM-DDTHH:MM:SS"
static bool setClockFromString(const String& s){
  int Y,M,D,h,m,sec;
  if (sscanf(s.c_str(), "%d-%d-%d %d:%d:%d", &Y,&M,&D,&h,&m,&sec) != 6 &&
      sscanf(s.c_str(), "%d-%d-%dT%d:%d:%d", &Y,&M,&D,&h,&m,&sec) != 6) {
    return false;
  }
  struct tm t = {};
  t.tm_year = Y - 1900; t.tm_mon = M - 1; t.tm_mday = D;
  t.tm_hour = h; t.tm_min = m; t.tm_sec = sec;
  // mktime expects local time per current TZ:
  setenv("TZ", SET_TZ.c_str(), 1); tzset();
  time_t epoch = mktime(&t);
  if (epoch <= 0) return false;
  return setClockFromEpoch(epoch);
}

// Pretty-print current local time to Serial (and option to return string)
static String nowString(){
  tm now{};
  if (getLocalTime(&now, 200)) {
    char b[40]; strftime(b, sizeof(b), "%F %T", &now);
    return String(b);
  }
  return String("unknown");
}


// ---------------- Serial commands ----------------
void printShow(){
  Serial.printf("active_ms=%u welcome_ms=%u reject_ms=%u marquee_ms=%u code_delay_ms=%u code=%s\n",
    SET_ACTIVE_MS, SET_WELCOME_MS, SET_REJECT_MS, SET_MARQUEE_MS, SET_CODE_DELAY_MS, UNLOCK_CODE.c_str());
  Serial.printf("quiet:%s %u->%u time:%s allowCount=%u\n",
    QUIET_EN?"on":"off", QUIET_START_H, QUIET_END_H, timeValid?"ok":"unknown", allowCount);
}
void handleSerial(){
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n'); line.trim();
  if (!line.length()) return;

  if (line=="help"){
    Serial.println(
      "Commands:\n"
      "  show\n"
      "  list | add <UIDHEX> | remove <UIDHEX> | clear\n"
      "  active_ms <ms> | welcome_ms <ms> | reject_ms <ms> | marquee_ms <ms> | code_delay_ms <ms>\n"
      "  sleep_ms <ms> | day_fallback_ms <ms>\n"
      "  unlock <4digits>\n"
      "  quiet on|off [start end]\n"
      "  time | date | ntp\n"
      "  settime YYYY-MM-DD HH:MM:SS | setepoch <unix> | settz <POSIX_TZ> | tz | checknow\n"
      "  last | lastgranted | addlast\n"
      "  i2cscan | pnreset | pninfo\n"
      "  sleepdiag | irq | sleepnow");
    return;
  }

  if (line=="show"){ printShow(); return; }
  if (line=="list"){ for(uint8_t i=0;i<allowCount;i++) Serial.println(allowUids[i]); return; }
  if (line.startsWith("add ")){ String h=line.substring(4); h.trim(); h.toUpperCase(); h.replace(" ",""); Serial.println(addAllow(h)?"ok":"fail/full"); saveAll(); return; }
  if (line.startsWith("remove ")){ String h=line.substring(7); h.trim(); h.toUpperCase(); h.replace(" ",""); Serial.println(removeAllow(h)?"ok":"notfound"); saveAll(); return; }
  if (line=="clear"){ clearAllow(); saveAll(); Serial.println("cleared"); return; }
  if (line=="checknow"){
  esp_sleep_wakeup_cause_t c = esp_sleep_get_wakeup_cause();
  bool synced = ensureTimeSync(10000, 10000, true, true);
  if (!synced && c == ESP_SLEEP_WAKEUP_TIMER && gPlannedWakeEpoch > 0) {
    setSystemTimeFromEpoch(gPlannedWakeEpoch, "planned-wake(manual)");
  }
  Serial.println(timeValid ? "time:ok" : "time:unknown");
  return;
  }

  if (line.startsWith("active_ms ")){ unsigned long v=line.substring(10).toInt(); if(v<10000UL)v=10000UL; SET_ACTIVE_MS=v; saveAll(); printShow(); return; }
  if (line.startsWith("welcome_ms ")){ unsigned long v=line.substring(11).toInt(); if(v<200UL) v=200UL; SET_WELCOME_MS=v; saveAll(); printShow(); return; }
  if (line.startsWith("reject_ms ")){ unsigned long v=line.substring(10).toInt(); if(v<200UL) v=200UL; SET_REJECT_MS=v; saveAll(); printShow(); return; }
  if (line.startsWith("marquee_ms ")){ long v=line.substring(11).toInt(); if(v<10)v=10; if(v>65535)v=65535; SET_MARQUEE_MS=(uint16_t)v; saveAll(); printShow(); return; }
  if (line.startsWith("code_delay_ms ")){ long v=line.substring(14).toInt(); if(v<0)v=0; SET_CODE_DELAY_MS=(uint32_t)v; saveAll(); printShow(); return; }

  if (line.startsWith("unlock ")){ String c=line.substring(7); c.trim(); if(c.length()==4){ UNLOCK_CODE=c; saveAll(); Serial.println("ok"); } else Serial.println("need 4 digits"); return; }

  if (line.startsWith("quiet ")){
    String rest=line.substring(6); rest.trim();
    if (rest=="on"){ QUIET_EN=true; saveAll(); Serial.println("quiet on"); return; }
    if (rest=="off"){ QUIET_EN=false; saveAll(); Serial.println("quiet off"); return; }
    int sp=rest.indexOf(' ');
    if (sp>0){
      String onoff=rest.substring(0,sp); onoff.trim();
      String times=rest.substring(sp+1); times.trim();
      int sp2=times.indexOf(' ');
      if (onoff=="on" && sp2>0){
        QUIET_EN=true; QUIET_START_H=(uint8_t)times.substring(0,sp2).toInt(); QUIET_END_H=(uint8_t)times.substring(sp2+1).toInt(); saveAll(); Serial.println("quiet updated"); return;
      }
    }
    Serial.println("usage: quiet on|off [start end]");
    return;
  }

  if (line=="time"){
    tm now{}; if(getLocalTime(&now)){ char b[40]; strftime(b,sizeof(b),"%F %T",&now); Serial.print("now "); Serial.println(b); }
    else Serial.println("time unknown");
    return;
  }

    // --- time / date reporting (alias) ---
  if (line=="time" || line=="date"){
    String s = nowString();
    Serial.print("now "); Serial.println(s);
    return;
  }

  // --- settime YYYY-MM-DD HH:MM:SS (or YYYY-MM-DDTHH:MM:SS) ---
  if (line.startsWith("settime ")){
    String ts = line.substring(8); ts.trim();
    bool ok = setClockFromString(ts);
    if (ok){
      String s = nowString();
      Serial.printf("time set: %s (tz=%s)\n", s.c_str(), SET_TZ.c_str());
      oledToast("TIME SET", s.c_str());
    } else {
      Serial.println("time parse fail (use: settime 2025-11-11 21:03:00)");
      oledToast("TIME PARSE FAIL");
    }
    return;
  }

  // --- setepoch <unix_seconds> ---
  if (line.startsWith("setepoch ")){
    long long v = strtoll(line.substring(9).c_str(), nullptr, 10);
    bool ok = setClockFromEpoch((time_t)v);
    if (ok){
      String s = nowString();
      Serial.printf("epoch set: %lld -> %s (tz=%s)\n", v, s.c_str(), SET_TZ.c_str());
      oledToast("EPOCH SET", s.c_str());
    } else {
      Serial.println("epoch set fail");
      oledToast("EPOCH SET FAIL");
    }
    return;
  }

  // --- settz <POSIX_TZ_string> (persists) ---
  if (line.startsWith("settz ")){
    String tz = line.substring(6); tz.trim();
    if (tz.length()==0){ Serial.println("usage: settz EST5EDT,M3.2.0/2,M11.1.0/2"); return; }
    SET_TZ = tz; saveAll();
    setenv("TZ", SET_TZ.c_str(), 1); tzset();
    // If we already had a valid clock, re-print in the new TZ
    String s = nowString();
    Serial.printf("tz set: %s\n", SET_TZ.c_str());
    oledToast("TZ SET", SET_TZ.c_str());
    if (s != "unknown") Serial.printf("now %s\n", s.c_str());
    return;
  }

  // --- tz (show current TZ string) ---
  if (line=="tz"){
    Serial.printf("tz=%s\n", SET_TZ.c_str());
    return;
  }



  if (line=="ntp"){ Serial.println(ensureTimeSync()?"synced":"fail"); return; }

  if (line=="sleepdiag"){
    Serial.printf("[DIAG] irq_rtccap=%d dig=%d quiet=%d time=%d NFCup=%d\n",
      (int)rtc_gpio_is_valid_gpio((gpio_num_t)PN532_IRQ),
      (int)digitalRead(PN532_IRQ),
      (int)(QUIET_EN && timeValid && inQuietHours()),
      (int)timeValid, (int)gNfcUp);
    return;
  }
  if (line=="irq"){
    Serial.printf("[IRQ] pin=%d level=%d\n", PN532_IRQ, (int)digitalRead(PN532_IRQ));
    return;
  }
  if (line=="sleepnow"){ enterDeepSleep_NightTimer(); return; }
  if (line=="i2cscan"){i2cScan(); return; }
  if (line=="pnreset"){pn532HardReset(); pnInfo(); return; }
  if (line=="pninfo"){pnInfo(); return; }
  // --- last-tag utilities ---
if (line == "last") {
  if (lastUidLen > 0) {
    Serial.printf("last=%s recognized=%s\n",
      lastUidHex.c_str(),
      lastWasRecognized ? "yes" : "no");
  } else {
    Serial.println("last=none");
  }
  return;
}

if (line == "lastgranted") {
  if (lastGrantedHex.length()) {
    Serial.printf("lastgranted=%s\n", lastGrantedHex.c_str());
  } else {
    Serial.println("lastgranted=none");
  }
  return;
}

if (line == "addlast") {
  if (lastUidLen > 0) {
    bool ok = addAllow(lastUidHex);
    if (ok) { saveAll(); Serial.printf("added %s\n", lastUidHex.c_str()); }
    else    { Serial.println("add fail/full or already present"); }
  } else {
    Serial.println("no last tag");
  }
  return;
}


  Serial.println("unknown; type 'help'");
}

// ---------------- Setup / Loop ----------------
void setup(){
  Serial.begin(115200);
  delay(200);
  esp_log_level_set("i2c", ESP_LOG_NONE);   // mute I2C driver spam early
  esp_log_level_set("i2c.master", ESP_LOG_NONE);

  logf("\nboot\n");

  // Radios OFF by default
  WiFi.disconnect(true,true);
  WiFi.mode(WIFI_OFF);

  // Clear the bus BEFORE Wire.begin
  i2cBusClear(OLED_SCL, OLED_SDA);

  Wire.begin(OLED_SDA, OLED_SCL, 100000);
  Wire.setClock(100000);
  Wire.setTimeOut(10);

  pinMode(PN532_RSTPDN, OUTPUT);
  digitalWrite(PN532_RSTPDN, HIGH);   // PN532 up during day
  pinMode(PN532_IRQ, INPUT_PULLUP);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    logf("[OLED] init failed\n");
  } else {
    display.clearDisplay(); display.display();
    display.ssd1306_command(SSD1306_DISPLAYOFF); displayAwake=false;
  }

  // Load persisted settings
  loadAll();

  // PN532 init
  bool ok = pn532TryInit();
  gNfcUp = ok;
  if (!ok) {
    logf("[PN532] not found (after retries)\n");
  }

  nfc.begin();
  uint32_t v = nfc.getFirmwareVersion();
  if(!v){
    logf("[PN532] not found\n");
    gNfcUp = false;
  } else {
    logf("[PN532] IC=0x%02X FW=%u.%u\n",(v>>24)&0xFF,(v>>16)&0xFF,(v>>8)&0xFF);
    nfc.SAMConfig();
    nfc.setPassiveActivationRetries(0x20);
    gNfcUp = true;
  }

  // If quiet enabled, try to have time available for night logic
  if (QUIET_EN) ensureTimeSync(6000,6000);

   // Determine wake cause *early*
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (cause == ESP_SLEEP_WAKEUP_TIMER) Serial.println("[WAKE] TIMER");
  else                                 Serial.println("[WAKE] COLD");

  // --- Boot-time time policy ---
  // Always try NTP at boot, regardless of quiet settings
  bool synced = ensureTimeSync(10000, 10000, /*force=*/true, /*verbose=*/true);

  if (!synced && cause == ESP_SLEEP_WAKEUP_TIMER && gPlannedWakeEpoch > 0) {
    // Couldn’t reach NTP after a night-timer wake — set time to the planned boundary
    setSystemTimeFromEpoch(gPlannedWakeEpoch, "planned-wake");
  }

  // Optional: if still not valid but we *ever* synced before, use last known + uptime as a coarse fallback
  if (!timeValid && gLastSyncEpoch > 0) {
    // This is rough (drift!), but better than "unknown"
    time_t approx = gLastSyncEpoch; // you could add millis()/1000 here, but that’s boot-relative only
    setSystemTimeFromEpoch(approx, "last-sync-epoch");
  }

  // Start UI
  displayWake(); mode=MODE_SIGN; lastScanMs=millis();
}

void loop(){
  handleSerial();

  // If we're in quiet hours and have valid time, go to sleep immediately
  if (quietNow()){
    enterDeepSleep_NightTimer();
  }

  // Short tag poll (only when PN532 is up)
  if (gNfcUp && digitalRead(PN532_RSTPDN) == HIGH) {
    static uint32_t lastPNTry = 0;
    if (millis() - lastPNTry >= 120) {          // ~8 polls/sec
      lastPNTry = millis();
      uint8_t uid[10]; uint8_t len = 0;

      // Short timeout so UI stays responsive
      if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &len, 25)) {
        lastScanMs = millis();
        displayWake();
          // Track "last tag scanned"
          lastUidLen  = len;
          lastUidHex  = (len ? uidHexNoSep(uid, len) : "");


        bool didAdmin = false;

        // Try to read NDEF (admin tags are TEXT with k=v lines)
        NdefResult nr = readNdefUltralight();
        if (nr.ok && nr.isText && nr.text.indexOf('=') >= 0) {
          AdminResult ar = applyConfigText(nr.text);
          showAdminFeedback(ar);
          didAdmin = true;
        }

        if (!didAdmin) {
          if (len >= 4) {
            String hex = uidHexNoSep(uid, len);
            bool rec = (allowCount > 0 ? inAllowlist(hex) : ALLOW_WHEN_EMPTY);

            recognizedLast    = rec;
            lastWasRecognized = rec;

            if (rec) {
              lastGrantedHex = hex;
              // ✅ Authorized tag → Welcome + code sequence
              mode = MODE_WELCOME;
              modeUntilMs = millis() + SET_WELCOME_MS;
              codeShowAtMs = millis() + SET_WELCOME_MS + SET_CODE_DELAY_MS;
            } else {
              // � Unknown tag → Poop animation
              mode = MODE_POOP;
              modeUntilMs = millis() + SET_REJECT_MS;
            }
          } else {
            // Invalid/short UID → also reject
            recognizedLast = false;
            mode = MODE_POOP;
            modeUntilMs = millis() + SET_REJECT_MS;
          }
        }
      }
    }
  }

  // After welcome, optionally show the code (recognized tags only)
  if (mode == MODE_WELCOME && recognizedLast) {
    if (millis() >= codeShowAtMs) {
      mode = MODE_CODE;
      uint32_t holdMs = (SET_WELCOME_MS > 1500U) ? SET_WELCOME_MS : 1500U;
      modeUntilMs = millis() + holdMs;
    }
  }

  // OLED idle blank
  if (mode!=MODE_SLEEP && (millis()-lastScanMs>SET_ACTIVE_MS)){
    mode=MODE_SLEEP; displaySleep();
  }

  // Mode timeouts back to SIGN
  if ((mode == MODE_CODE || mode == MODE_POOP || (mode == MODE_WELCOME && !recognizedLast))
      && millis() > modeUntilMs) {
    mode = MODE_SIGN;
  }
  // Daytime inactivity -> deep sleep with PN532 AutoPoll + EXT0 LOW wake
  if (!quietNow() && (millis() - lastScanMs > SET_SLEEP_MS)) {
    enterDeepSleep_DayIRQ();
  }

  if      (mode==MODE_SIGN)    { static uint32_t last=0; if(millis()-last>SET_MARQUEE_MS){ last=millis(); } drawSignFrame(); }
  else if (mode==MODE_WELCOME)  drawWelcomeFrame();
  else if (mode==MODE_CODE)     drawCodeFrame();
  else if (mode==MODE_POOP)     drawPoopFrame();
}
