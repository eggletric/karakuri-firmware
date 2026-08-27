#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <WiFi.h>
#include <LittleFS.h>
#include "switch_tinyusb.h"
#include "ble_link.h"
#include "procon2_dongle.h"
#include "sinput_usb.h"
#include "ds4_usb.h"
#include "procon_usb.h"
#include "version.h"

#ifndef FW_VERSION
#define FW_VERSION "dev"
#endif

// ==============================
// Config struct + globals
//   mode: "bt" (default) = connect to the app over BLE / "wifi" = TCP through a router
//   Only one of them starts (the radios share the CYW43, so they are mutually exclusive)
// ==============================
struct LinkConfig {
  String mode;       // "bt" | "wifi" | "dongle"
  String btName;     // BLE device name (empty auto-generates "Karakuri-XXXX")
  String ssid;
  String password;
  IPAddress localIP;
  String port;
  IPAddress gateway;
  IPAddress subnet;
  String usbMode;    // USB identity in dongle mode: "sinput" (default) | "ds4" | "switch" | "procon"
  String ds4MapC;    // token assigned to the C button in ds4 mode
  String ds4MapGL;   // same for GL in ds4 mode
  String ds4MapGR;   // same for GR in ds4 mode
  String swMapC;     // token assigned to the C button in switch mode
  String swMapGL;    // same for GL in switch mode
  String swMapGR;    // same for GR in switch mode
  bool macroOn;      // dongle macro recorder (record/replay controller input; mode=dongle only)
  bool valid;        // whether the Wi-Fi fields are complete (required only when mode=wifi)
};

LinkConfig gLinkConfig = {
  "bt", "", "", "", IPAddress(), "", IPAddress(), IPAddress(),
  "sinput", "touchpad", "none", "none", "none", "none", "none", false, false
};
bool wifiStarted  = false;
bool gWifiEnabled = false;   // whether we intend to use Wi-Fi (gates automatic reconnection)
bool fsMounted    = false;

// ==============================
// BLE globals
// ==============================
BleLinkService gBleLink;
bool   gBleStarted     = false;
String gBleActiveName  = "";     // the name put into the advertisement at startup
String gBleLineBuffer  = "";
bool   gBleWasConnected = false;

// The transport currently running ("" = not started / "bt" / "wifi" / "dongle").
// Switching transports requires a reboot because the radio stack cannot be rebuilt.
String gActiveTransport = "";

// ==============================
// Dongle globals (mode=dongle)
//   The Pico connects to a Pro Controller 2 as a BLE central and streams its input to USB HID.
//   It holds no link to the app (configuration is over USB serial only).
// ==============================
Procon2Link gProcon2;
// The SInput / DS4 USB HID instances are created only in dongle mode.
// Keeping them as permanent globals would leave an Adafruit_USBD_HID present in
// bt/wifi mode too, breaking USB enumeration and taking serial (CDC) down with it.
SInputUsb  *gSInput = nullptr;
DS4Usb     *gDS4 = nullptr;
ProconUsb  *gProconUsb = nullptr;
String   gActiveUsbMode      = "";   // the USB identity chosen at dongle startup (only a reboot changes it)
uint32_t gDongleNextScan     = 0;
bool     gDongleWasConnected = false;
uint8_t  gDongleDumpRemaining = 0;    // remaining output rounds for the DUMP command
uint32_t gDongleNextDumpAt = 0;
uint32_t gDongleLastRumbleAt = 0;
bool     gDongleRumblePending = false;
uint8_t  gDongleRumbleL = 0;
uint8_t  gDongleRumbleR = 0;

String normalizeMode(const String &m) {
  String v = m;
  v.trim();
  v.toLowerCase();
  if (v == "wifi") return "wifi";
  if (v == "dongle") return "dongle";   // the Pro Controller 2 -> USB conversion dongle
  return "bt";   // unset or unknown values fall back to bt
}

String normalizeUsbMode(const String &m) {
  String v = m;
  v.trim();
  v.toLowerCase();
  if (v == "ds4") return "ds4";
  if (v == "switch") return "switch";   // the HORI-compatible Switch pad identity
  if (v == "procon") return "procon";   // Pro Controller emulation (gyro + rumble on Switch)
  return "sinput";   // unset or unknown values fall back to sinput
}

// Assignment tokens for C/GL/GR in ds4 mode
bool isValidDs4Token(const String &v) {
  return v == "none" || v == "touchpad" || v == "ps" || v == "share" || v == "options" ||
         v == "l1" || v == "r1" || v == "l2" || v == "r2" || v == "l3" || v == "r3" ||
         v == "cross" || v == "circle" || v == "square" || v == "triangle";
}

String normalizeDs4Token(const String &m, const char *fallback) {
  String v = m;
  v.trim();
  v.toLowerCase();
  return isValidDs4Token(v) ? v : String(fallback);
}

// Assignment tokens for C/GL/GR in switch mode (every button NSGamepad has)
bool isValidSwitchToken(const String &v) {
  return v == "none" || v == "a" || v == "b" || v == "x" || v == "y" ||
         v == "l" || v == "r" || v == "zl" || v == "zr" ||
         v == "plus" || v == "minus" || v == "home" || v == "capture" ||
         v == "lstick" || v == "rstick";
}

String normalizeSwitchToken(const String &m, const char *fallback) {
  String v = m;
  v.trim();
  v.toLowerCase();
  return isValidSwitchToken(v) ? v : String(fallback);
}

String defaultBleName() {
  // Build a default name that distinguishes devices by the last 4 digits of the chip ID
  const char *id = rp2040.getChipID();
  size_t len = strlen(id);
  String suffix = (len >= 4) ? String(id + len - 4) : String(id);
  suffix.toUpperCase();
  return "Karakuri-" + suffix;
}

String effectiveBleName() {
  String n = gLinkConfig.btName;
  n.trim();
  return n.length() ? n : defaultBleName();
}

WiFiServer* gServer = nullptr;
uint16_t gTcpPort   = 5000;

WiFiClient gCurrentClient;
String     gWifiLineBuffer = "";

// For the Wi-Fi supervisor task (declared here because startWifiFromConfig touches it too)
uint32_t gWifiSuperviseNext = 0;
uint32_t gWifiRetryNext     = 0;

// Scratch buffers that avoid String reallocation (reserved in setup())
String gCmdLine  = "";
String gCmdUpper = "";
String gMacroCmd = "";

// ==============================
// FS helper: mount LittleFS
// ==============================
bool ensureFS() {
  if (fsMounted) return true;

  if (LittleFS.begin()) {
    Serial.println("[FS] LittleFS mounted");
    fsMounted = true;
    return true;
  }

  Serial.println("[FS] LittleFS mount failed, retrying once...");
  delay(50);

  if (LittleFS.begin()) {
    Serial.println("[FS] LittleFS mounted on second try");
    fsMounted = true;
    return true;
  }

  Serial.println("[FS] still failed, trying format...");

  if (!LittleFS.format()) {
    Serial.println("[FS] format failed");
    return false;
  }

  if (!LittleFS.begin()) {
    Serial.println("[FS] mount after format failed");
    return false;
  }

  Serial.println("[FS] formatted and mounted");
  fsMounted = true;
  return true;
}

// ==============================
// Port helper: only numeric values in 1..65535
// ==============================
bool parseTcpPort(const String &s, uint16_t &out) {
  if (s.length() == 0 || s.length() > 5) return false;
  for (unsigned int i = 0; i < s.length(); i++) {
    if (s[i] < '0' || s[i] > '9') return false;
  }
  long v = s.toInt();
  if (v < 1 || v > 65535) return false;
  out = (uint16_t)v;
  return true;
}

// ==============================
// Load config from /link.cfg
//   1: ssid
//   2: password
//   3: ip
//   4: port
//   5: gateway
//   6: subnet
//   7: mode ("bt" | "wifi" | "dongle")   NOTE: an older file without it means "bt" (the default)
//   8: btname                 NOTE: empty means the auto-generated name
//   9: usbmode ("sinput" | "ds4" | "switch" | "procon")   NOTE: the USB identity in dongle mode
//  10: ds4map (the c,gl,gr tokens, comma-separated)
//  11: macro ("on" | "off")              NOTE: the dongle macro recorder
//  12: switchmap (the c,gl,gr tokens for usbmode=switch, comma-separated)
// ==============================
// Returns whether the Wi-Fi fields are valid. mode/btname are applied as far as the
// file could be read (BT runs on its defaults even with no file).
bool loadLinkConfig() {
  // Reset every field to its default so an early return cannot leave stale values
  gLinkConfig.valid    = false;
  gLinkConfig.mode     = "bt";
  gLinkConfig.btName   = "";
  gLinkConfig.usbMode  = "sinput";
  gLinkConfig.ds4MapC  = "touchpad";
  gLinkConfig.ds4MapGL = "none";
  gLinkConfig.ds4MapGR = "none";
  gLinkConfig.swMapC   = "none";
  gLinkConfig.swMapGL  = "none";
  gLinkConfig.swMapGR  = "none";
  gLinkConfig.macroOn  = false;

  if (!ensureFS()) {
    Serial.println("[CFG] loadLinkConfig: FS not mounted");
    return false;
  }

  File f = LittleFS.open("/link.cfg", "r");
  if (!f) {
    Serial.println("[CFG] link.cfg not found -> using defaults (mode=bt)");
    return false;
  }

  // readStringUntil over a corrupt file lets String eat the whole heap; reject by size first
  if (f.size() > 4096) {
    Serial.println("[CFG] link.cfg too large -> using defaults (mode=bt)");
    f.close();
    return false;
  }

  String ssid = f.readStringUntil('\n'); ssid.trim();
  // Spaces around a password are valid characters, so do not trim; only drop a trailing CR
  String pass = f.readStringUntil('\n');
  if (pass.length() && pass[pass.length() - 1] == '\r') {
    pass.remove(pass.length() - 1);
  }
  String ip   = f.readStringUntil('\n'); ip.trim();
  String port = f.readStringUntil('\n'); port.trim();
  String gw   = f.readStringUntil('\n'); gw.trim();
  String sn   = f.readStringUntil('\n'); sn.trim();
  String mode = f.readStringUntil('\n');
  String btn  = f.readStringUntil('\n'); btn.trim();
  String usbm = f.readStringUntil('\n');
  String d4m  = f.readStringUntil('\n'); d4m.trim();
  String mac  = f.readStringUntil('\n'); mac.trim();
  String swm  = f.readStringUntil('\n'); swm.trim();
  f.close();

  gLinkConfig.mode    = normalizeMode(mode);
  gLinkConfig.btName  = btn;
  gLinkConfig.usbMode = normalizeUsbMode(usbm);

  // ds4map is "c,gl,gr", comma-separated
  {
    int c1 = d4m.indexOf(',');
    int c2 = (c1 >= 0) ? d4m.indexOf(',', c1 + 1) : -1;
    String tc = (c1 >= 0) ? d4m.substring(0, c1) : d4m;
    String tgl = (c1 >= 0 && c2 >= 0) ? d4m.substring(c1 + 1, c2) : "";
    String tgr = (c2 >= 0) ? d4m.substring(c2 + 1) : "";
    gLinkConfig.ds4MapC  = normalizeDs4Token(tc, "touchpad");
    gLinkConfig.ds4MapGL = normalizeDs4Token(tgl, "none");
    gLinkConfig.ds4MapGR = normalizeDs4Token(tgr, "none");
  }

  mac.toLowerCase();
  gLinkConfig.macroOn = (mac == "on" || mac == "1" || mac == "true");

  // switchmap is "c,gl,gr", comma-separated (same shape as ds4map)
  {
    int c1 = swm.indexOf(',');
    int c2 = (c1 >= 0) ? swm.indexOf(',', c1 + 1) : -1;
    String tc = (c1 >= 0) ? swm.substring(0, c1) : swm;
    String tgl = (c1 >= 0 && c2 >= 0) ? swm.substring(c1 + 1, c2) : "";
    String tgr = (c2 >= 0) ? swm.substring(c2 + 1) : "";
    gLinkConfig.swMapC  = normalizeSwitchToken(tc, "none");
    gLinkConfig.swMapGL = normalizeSwitchToken(tgl, "none");
    gLinkConfig.swMapGR = normalizeSwitchToken(tgr, "none");
  }

  // BT mode never uses the Wi-Fi settings. Printing them looks like an attempt to
  // connect to Wi-Fi, so print neither the validation log nor the values
  const bool wifiMode = (gLinkConfig.mode == "wifi");

  Serial.println("[CFG] link.cfg loaded OK");
  Serial.print("[CFG] MODE: "); Serial.println(gLinkConfig.mode);
  if (!wifiMode) {
    Serial.print("[CFG] BT NAME: "); Serial.println(effectiveBleName());
  }

  // ---- Validation of the Wi-Fi fields starts here ----
  gLinkConfig.ssid     = ssid;
  gLinkConfig.password = pass;
  gLinkConfig.port     = port.length() ? port : "5000";

  IPAddress ipAddr, gwAddr, snAddr;
  if (!ipAddr.fromString(ip) || !gwAddr.fromString(gw) || !snAddr.fromString(sn)) {
    if (wifiMode) Serial.println("[CFG] IP/GW/SN parse failed -> Wi-Fi config invalid");
    return false;
  }

  if (ssid.length() == 0) {
    if (wifiMode) Serial.println("[CFG] SSID empty -> Wi-Fi config invalid");
    return false;
  }

  uint16_t portNum;
  if (!parseTcpPort(gLinkConfig.port, portNum)) {
    if (wifiMode) {
      Serial.print("[CFG] PORT out of range (1-65535) -> Wi-Fi config invalid: ");
      Serial.println(gLinkConfig.port);
    }
    return false;
  }

  gLinkConfig.localIP  = ipAddr;
  gLinkConfig.gateway  = gwAddr;
  gLinkConfig.subnet   = snAddr;
  gLinkConfig.valid    = true;

  if (wifiMode) {
    Serial.print("[CFG] SSID: "); Serial.println(gLinkConfig.ssid);
    Serial.print("[CFG] IP  : "); Serial.println(gLinkConfig.localIP);
    Serial.print("[CFG] PORT: "); Serial.println(gLinkConfig.port);
    Serial.print("[CFG] GW  : "); Serial.println(gLinkConfig.gateway);
    Serial.print("[CFG] SN  : "); Serial.println(gLinkConfig.subnet);
  }

  return true;
}

// ==============================
// Save config to /link.cfg
// ==============================
bool saveLinkConfig(const LinkConfig &cfg) {
  if (!ensureFS()) {
    Serial.println("[CFG] saveLinkConfig: FS not mounted");
    return false;
  }

  // Writing /link.cfg in place loses the whole configuration if power is cut mid-write.
  // Write to tmp, then rename (LittleFS rename replaces the destination).
  const char *TMP = "/link.cfg.tmp";

  File f = LittleFS.open(TMP, "w");
  if (!f) {
    Serial.println("[CFG] open(/link.cfg.tmp, w) failed");
    return false;
  }

  bool ok = true;
  ok &= f.println(cfg.ssid) > 0;
  ok &= f.println(cfg.password) > 0;
  ok &= f.println(cfg.localIP.toString()) > 0;
  ok &= f.println(cfg.port) > 0;
  ok &= f.println(cfg.gateway.toString()) > 0;
  ok &= f.println(cfg.subnet.toString()) > 0;
  ok &= f.println(normalizeMode(cfg.mode)) > 0;
  ok &= f.println(cfg.btName) > 0;
  ok &= f.println(normalizeUsbMode(cfg.usbMode)) > 0;
  ok &= f.println(normalizeDs4Token(cfg.ds4MapC, "touchpad") + "," +
                  normalizeDs4Token(cfg.ds4MapGL, "none") + "," +
                  normalizeDs4Token(cfg.ds4MapGR, "none")) > 0;
  ok &= f.println(cfg.macroOn ? "on" : "off") > 0;
  ok &= f.println(normalizeSwitchToken(cfg.swMapC, "none") + "," +
                  normalizeSwitchToken(cfg.swMapGL, "none") + "," +
                  normalizeSwitchToken(cfg.swMapGR, "none")) > 0;
  if (ok && f.getWriteError()) ok = false;
  f.close();

  if (!ok) {
    Serial.println("[CFG] write to /link.cfg.tmp failed -> keeping previous config");
    LittleFS.remove(TMP);
    return false;
  }

  if (!LittleFS.rename(TMP, "/link.cfg")) {
    Serial.println("[CFG] rename(/link.cfg.tmp -> /link.cfg) failed");
    LittleFS.remove(TMP);
    return false;
  }

  Serial.println("[CFG] link.cfg saved");
  return true;
}

// ==============================
// Wi-Fi start/stop helpers
// ==============================
void stopWifiServer() {
  Serial.println("[WLAN] stopping radio and TCP server...");

  if (gServer) {
    gServer->stop();
    delete gServer;
    gServer = nullptr;
  }

  if (gCurrentClient) {
    gCurrentClient.stop();
  }
  gWifiLineBuffer = "";

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  gWifiEnabled = false;
  wifiStarted  = false;

  Serial.println("[WLAN] stopped.");
}

// Link-state helpers (defined further down in the supervisor section)
const char* wlanStatusName(int st);
void serviceTap();

// The definition lives after Gamepad (so HID keeps running while we wait for a connection)
bool wlanBeginAndWait(uint32_t timeoutMs);
void processSerialInput();
extern bool gWlanConnecting;

bool startTcpServer() {
  // Reuse an existing server: an lwIP listener survives a link drop, so it can keep
  // accepting once the link returns.
  if (gServer) return true;

  gServer = new WiFiServer(gTcpPort);
  if (!gServer) return false;

  gServer->begin();
  return true;
}

bool startWifiFromConfig() {
  if (!gLinkConfig.valid) {
    Serial.println("[WLAN] startWifiFromConfig: config invalid");
    return false;
  }

  // Fix the port before attempting to connect (so a reconnect listens on the same port)
  uint16_t portNum = 5000;
  if (!parseTcpPort(gLinkConfig.port, portNum)) {
    Serial.print("[WLAN] invalid port, aborting: ");
    Serial.println(gLinkConfig.port);
    wifiStarted = false;
    return false;
  }
  gTcpPort = portNum;

  gWifiEnabled = true;   // the supervisor takes care of reconnecting after a drop
  wifiStarted  = false;

  WiFi.mode(WIFI_STA);
  // CYW43's power saving (PM2) produces latency spikes on the order of 100ms by itself.
  // Passthrough puts constant responsiveness first, so disable it.
  WiFi.noLowPowerMode();

  Serial.print("[WLAN] connecting to router, SSID: \"");
  Serial.print(gLinkConfig.ssid);
  Serial.print("\" (ssid len=");
  Serial.print(gLinkConfig.ssid.length());
  Serial.print(", pass len=");
  Serial.print(gLinkConfig.password.length());
  Serial.println(")");

  if (wlanBeginAndWait(15000)) {
    Serial.print("[WLAN] router connected, IP = ");
    Serial.println(WiFi.localIP());
    if (startTcpServer()) {
      wifiStarted = true;
      Serial.print("[TCP] server listening on port ");
      Serial.print(gTcpPort);
      Serial.println(" (waiting for app)");
    } else {
      Serial.println("[TCP] server start failed");
    }
  } else {
    Serial.println("[WLAN] not connected yet -> supervisor will keep retrying");
  }

  uint32_t now = millis();
  gWifiSuperviseNext = now + 1000;
  gWifiRetryNext     = now + 10000;

  return true;
}

// ==============================
// BLE start helper
//   BLE.begin() happens exactly once (the core has no end()). Switching transports or
//   the BLE name is applied by the RESET handler rebooting.
// ==============================

// Build the advertisement and scan response ourselves. The core's BLEAdvertising
// leaves the service UUID out (BLEUUID's string parsing is broken), and including the
// UUID truncates the name to 8 characters under the 31-byte limit. So: flags +
// 128-bit UUID + as much of the name as fits, and the full name in the scan response.
// BTstack holds on to the pointer, so the buffers must be static.
static uint8_t gAdvData[31];
static uint8_t gScanRespData[31];

void bleSetDeterministicAdvertising(const String &name) {
  const size_t nameLen = name.length();

  uint8_t p = 0;
  gAdvData[p++] = 2;                    // Flags
  gAdvData[p++] = BLUETOOTH_DATA_TYPE_FLAGS;
  gAdvData[p++] = 0x06;                 // LE General Discoverable / BR/EDR not supported
  gAdvData[p++] = 17;                   // 128-bit service UUID (complete list)
  gAdvData[p++] = BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS;
  for (int i = 15; i >= 0; i--) {       // little-endian on air
    gAdvData[p++] = BleLinkService::SERVICE_UUID_BYTES[i];
  }
  size_t room = 31 - p - 2;             // what remains after the name AD structure (len + type)
  size_t n = (nameLen <= room) ? nameLen : room;
  if (n > 0) {
    gAdvData[p++] = (uint8_t)(n + 1);
    gAdvData[p++] = (nameLen <= room)
                        ? BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME
                        : BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME;
    memcpy(&gAdvData[p], name.c_str(), n);
    p += (uint8_t)n;
  }

  uint8_t q = 0;
  size_t rn = (nameLen <= 29) ? nameLen : 29;
  gScanRespData[q++] = (uint8_t)(rn + 1);
  gScanRespData[q++] = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME;
  memcpy(&gScanRespData[q], name.c_str(), rn);
  q += (uint8_t)rn;

  BluetoothLock lock;
  gap_advertisements_set_data(p, gAdvData);
  gap_scan_response_set_data(q, gScanRespData);
}

bool startBleFromConfig() {
  if (gBleStarted) return true;

  gBleActiveName = effectiveBleName();
  Serial.print("[BLE] starting, name: ");
  Serial.println(gBleActiveName);

  BLE.begin(gBleActiveName);
  BLE.server()->addService(&gBleLink);
  BLE.startAdvertising();
  // Overwrite the advertisement the core assembled with our own, service-UUID-carrying one
  bleSetDeterministicAdvertising(gBleActiveName);

  gBleStarted = true;
  Serial.println("[BLE] advertising (waiting for app)");
  return true;
}

// ==============================
// Dongle start helper (mode=dongle)
// ==============================
bool startDongleFromConfig() {
  Serial.println("[DONGLE] starting BLE central (Pro Controller 2 -> USB bridge)");
  BLE.begin();   // central role. No name and no advertising needed
  gDongleNextScan = millis() + 500;
  Serial.println("[DONGLE] scanning for Pro Controller 2 (press sync button to pair)");
  return true;
}

// ==============================
// Serial config protocol
// ==============================
bool cfgReceiving = false;
uint32_t cfgBeginAt = 0;   // when CFG BEGIN arrived (used to expire it if END never comes)
LinkConfig incomingCfg;
String serialLine = "";

// forward
// Responses are taken as a Print* so they can go to a TCP client or the BLE link alike.
// nullptr means "no destination" (disconnected, coming from serial, ...).
void handleCommand(const String& rawLine, Print *client = nullptr);
void handleConfigCommand(const String &line);

// Returns the destination currently connected to the app (nullptr if none)
Print* activeReplyTarget() {
  if (gActiveTransport == "bt") {
    return gBleLink.connected() ? (Print*)&gBleLink : nullptr;
  }
  if (gCurrentClient && gCurrentClient.connected()) {
    return &gCurrentClient;
  }
  return nullptr;
}

// ==============================
// Serial input task
// ==============================
void processSerialInput() {
  // Without an END, every later line keeps being eaten as a key=value setting and pad
  // input stops working. Bail out automatically after 10 seconds.
  if (cfgReceiving && (int32_t)(millis() - cfgBeginAt) > 10000) {
    cfgReceiving = false;
    Serial.println("[CFG] BEGIN timed out (no END) -> back to command mode");
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      String line = serialLine;
      // Do not trim pass=, so spaces inside the password itself survive
      if (!line.startsWith("pass=")) {
        line.trim();
      }
      if (line.length()) {
        Serial.print("[CFG] line: ");
        Serial.println(line);
        handleConfigCommand(line);
      }
      serialLine = "";
    } else if (c != '\r') {
      serialLine += c;
    }
    yield();
  }
}

void resetIncomingCfg() {
  // Start from the current configuration so a partial update keeps the other side
  // (changing only the BT name preserves the Wi-Fi SSID/IP)
  incomingCfg = gLinkConfig;
  incomingCfg.valid = false;
}

void handleConfigCommand(const String &line) {
  // A RESET during Wi-Fi connection setup corrupts the radio state, so make it wait
  if (gWlanConnecting && line == "RESET") {
    Serial.println("[CFG] busy (connecting to Wi-Fi), try RESET again later");
    return;
  }

  if (line == "VERSION") {
    Serial.print("[FW] VERSION ");
    Serial.println(FW_VERSION);
    return;
  }

  // Toggles the protocol-investigation logs ([GATT]/[P2RSP] and friends).
  // The work itself always runs; only the printing is switched
  if (line == "DEBUG") {
    Procon2Link::debugLog = !Procon2Link::debugLog;
    Serial.printf("[DONGLE] debug log %s\n", Procon2Link::debugLog ? "ON" : "OFF");
    return;
  }

  // Dumps the USB trace of the last two procon-identity sessions (recorded to
  // flash: while plugged into a console there is no PC to read serial from).
  // Timestamps in ms, RX = from the console, T81/T21 = our replies, GRQ = GET_REPORT
  if (line == "PTRACE") {
    bool any = false;
    for (const char *path : { "/ptrace.old", "/ptrace.log" }) {
      File f = LittleFS.open(path, "r");
      if (!f) continue;
      any = true;
      Serial.printf("[PTRACE] --- %s begin ---\n", path);
      while (f.available()) {
        Serial.write(f.read());
        yield();
      }
      f.close();
      Serial.printf("[PTRACE] --- %s end ---\n", path);
    }
    if (!any) Serial.println("[PTRACE] no trace recorded");
    return;
  }

  // Dongle macro recorder status (slot occupancy and the current state)
  if (line == "DMACRO") {
    if (gActiveTransport != "dongle") {
      Serial.println("[DMACRO] dongle mode only");
      return;
    }
    dmPrintStatus();
    return;
  }

  // Dumps the raw dongle report 10 times (for investigating byte offsets such as the IMU)
  if (line == "DUMP") {
    if (gActiveTransport != "dongle") {
      Serial.println("[DUMP] dongle mode only");
      return;
    }
    if (!gProcon2.connected()) {
      Serial.println("[DUMP] Pro Controller 2 not connected (press sync button)");
      return;
    }
    gDongleDumpRemaining = 10;
    Serial.println("[DONGLE] dumping next 10 reports...");
    return;
  }

  // Sends an arbitrary byte string to the Pro Controller 2 command char (for protocol work).
  // Example: P2CMD 0C910102000400002F000000
  if (line.startsWith("P2CMD ")) {
    if (gActiveTransport != "dongle" || !gProcon2.connected()) {
      Serial.println("[P2CMD] dongle mode + Pro Controller 2 connection required");
      return;
    }
    uint8_t buf[96];
    size_t n = 0;
    int hi = -1;
    for (unsigned i = 6; i < line.length() && n < sizeof(buf); i++) {
      char c = line[i];
      int v;
      if (c >= '0' && c <= '9') v = c - '0';
      else if (c >= 'A' && c <= 'F') v = c - 'A' + 10;
      else if (c >= 'a' && c <= 'f') v = c - 'a' + 10;
      else continue;   // treat whitespace and the like as separators
      if (hi < 0) { hi = v; }
      else { buf[n++] = (uint8_t)((hi << 4) | v); hi = -1; }
    }
    if (n == 0) {
      Serial.println("[P2CMD] no bytes");
      return;
    }
    bool ok = gProcon2.sendRaw(buf, n);
    Serial.printf("[P2CMD] %s (%u bytes)\n", ok ? "sent" : "FAILED", (unsigned)n);
    return;
  }

  // Sends an arbitrary byte string to the char named by the last 4 UUID digits (for protocol work).
  // Example: P2W 2B05 0C910102000400002F000000
  if (line.startsWith("P2W ")) {
    if (gActiveTransport != "dongle" || !gProcon2.connected()) {
      Serial.println("[P2W] dongle mode + Pro Controller 2 connection required");
      return;
    }
    int sp = line.indexOf(' ', 4);
    if (sp < 0) {
      Serial.println("[P2W] usage: P2W <uuid-suffix-4hex> <hex bytes>");
      return;
    }
    uint16_t suffix = (uint16_t)strtoul(line.substring(4, sp).c_str(), nullptr, 16);
    uint8_t buf[96];
    size_t n = 0;
    int hi = -1;
    for (unsigned i = sp + 1; i < line.length() && n < sizeof(buf); i++) {
      char c = line[i];
      int v;
      if (c >= '0' && c <= '9') v = c - '0';
      else if (c >= 'A' && c <= 'F') v = c - 'A' + 10;
      else if (c >= 'a' && c <= 'f') v = c - 'a' + 10;
      else continue;
      if (hi < 0) { hi = v; }
      else { buf[n++] = (uint8_t)((hi << 4) | v); hi = -1; }
    }
    if (n == 0) {
      Serial.println("[P2W] no bytes");
      return;
    }
    bool ok = gProcon2.sendRawTo(suffix, buf, n);
    Serial.printf("[P2W] %04X %s (%u bytes)\n", suffix, ok ? "sent" : "FAILED (char not found?)", (unsigned)n);
    return;
  }


  // ---- FS commands ----
  if (line == "FS FORMAT") {
    Serial.println("[FS] FORMAT requested...");
    cfgReceiving = false;   // discard a CFG that was still being received
    LittleFS.end();
    fsMounted = false;

    if (!LittleFS.format()) {
      Serial.println("[FS] format FAIL");
      return;
    }

    if (ensureFS()) {
      Serial.println("[FS] format success and mounted");
    } else {
      Serial.println("[FS] format success but mount FAIL");
    }
    return;
  }

  if (line == "FS INFO") {
    Serial.println("[FS] INFO requested");
    if (!ensureFS()) {
      Serial.println("[FS] ensureFS() failed -> cannot mount");
      return;
    }

    Serial.println("[FS] FS is mounted");
    if (LittleFS.exists("/link.cfg")) {
      Serial.println("[FS] link.cfg exists");
    } else {
      Serial.println("[FS] link.cfg missing");
    }
    return;
  }

  if (line == "FS TEST") {
    Serial.println("[FS] TEST begin");
    if (!ensureFS()) {
      Serial.println("[FS] ensureFS() failed -> mount FAIL");
      return;
    }

    File test = LittleFS.open("/test.txt","w");
    if (!test) {
      Serial.println("[FS] open(write) failed");
      return;
    }
    test.println("OK");
    test.close();

    File read = LittleFS.open("/test.txt","r");
    if (!read) {
      Serial.println("[FS] open(read) failed");
      return;
    }
    Serial.print("[FS] read result -> ");
    Serial.println(read.readString());
    read.close();

    Serial.println("[FS] TEST success");
    return;
  }

  // ---- RESET: reload link.cfg & apply ----
  // Switching transport (bt/wifi) and changing the BLE name need a reboot; the radio
  // stack cannot be rebuilt. Changing settings while staying on wifi reconnects as before.
  if (line == "RESET") {
    Serial.println("[CFG] RESET command received");
    cfgReceiving = false;   // discard a CFG that was still being received
    Serial.println("[CFG] Reloading link.cfg...");
    loadLinkConfig();

    const String newMode = gLinkConfig.mode;   // already normalized by loadLinkConfig
    Serial.print("[CFG] mode: ");
    Serial.print(gActiveTransport.length() ? gActiveTransport : "(none)");
    Serial.print(" -> ");
    Serial.println(newMode);

    bool needReboot = false;
    if (newMode != gActiveTransport) {
      needReboot = true;
    } else if (newMode == "bt" && effectiveBleName() != gBleActiveName) {
      needReboot = true;   // the advertised name can only be changed in BLE.begin()
    } else if (newMode == "dongle" && gLinkConfig.usbMode != gActiveUsbMode) {
      needReboot = true;   // the USB identity can only be chosen at startup
    }

    if (needReboot) {
      Serial.println("[CFG] transport/name changed -> rebooting to apply...");
      Serial.flush();
      delay(200);
      rp2040.reboot();
      return;   // unreachable
    }

    if (newMode != "wifi") {
      // bt / dongle: nothing to apply when the mode has not changed
      Serial.println("[CFG] mode unchanged, nothing to apply.");
      return;
    }

    // ---- Settings changed while staying on mode=wifi: reconnect as before ----
    if (!gLinkConfig.valid) {
      Serial.println("[CFG] Wi-Fi config invalid. Wi-Fi will be stopped.");
      stopWifiServer();
      Serial.println("[CFG] RESET handler done (Wi-Fi stopped).");
      return;
    }

    Serial.println("[CFG] Applying new Wi-Fi config (reconnect)...");
    stopWifiServer();
    if (startWifiFromConfig()) {
      Serial.println("[CFG] RESET handler done (connecting in background).");
    } else {
      Serial.println("[CFG] RESET handler done (Wi-Fi could not be started).");
    }
    return;
  }

  // ---- return current config ----
  // Do not print the settings for the side not in use: it reads as "that side is active".
  // The app also sends only the active mode's keys, and the hidden side survives
  // through the CFG BEGIN carry-over.
  if (line == "CFG GET") {
    Serial.println("[CFG] CURRENT");
    Serial.print("mode=");   Serial.println(gLinkConfig.mode);
    if (gLinkConfig.mode == "bt") {
      Serial.print("btname="); Serial.println(gLinkConfig.btName);
      // For reference: the name actually used for advertising (auto-generated when btname is empty)
      Serial.print("# effective btname="); Serial.println(effectiveBleName());
    }
    if (gLinkConfig.mode == "dongle") {
      Serial.print("usbmode="); Serial.println(gLinkConfig.usbMode);
      Serial.print("ds4map=");
      Serial.print(gLinkConfig.ds4MapC); Serial.print(",");
      Serial.print(gLinkConfig.ds4MapGL); Serial.print(",");
      Serial.println(gLinkConfig.ds4MapGR);
      Serial.print("switchmap=");
      Serial.print(gLinkConfig.swMapC); Serial.print(",");
      Serial.print(gLinkConfig.swMapGL); Serial.print(",");
      Serial.println(gLinkConfig.swMapGR);
      Serial.print("macro="); Serial.println(gLinkConfig.macroOn ? "on" : "off");
    }
    if (gLinkConfig.mode == "wifi") {
      if (gLinkConfig.valid) {
        Serial.print("ssid="); Serial.println(gLinkConfig.ssid);
        Serial.print("pass="); Serial.println(gLinkConfig.password);
        Serial.print("ip=");   Serial.println(gLinkConfig.localIP);
        Serial.print("port="); Serial.println(gLinkConfig.port);
        Serial.print("gw=");   Serial.println(gLinkConfig.gateway);
        Serial.print("sn=");   Serial.println(gLinkConfig.subnet);
        Serial.print("# ssid len="); Serial.print(gLinkConfig.ssid.length());
        Serial.print(", pass len="); Serial.println(gLinkConfig.password.length());
      } else {
        Serial.println("none");
      }
    }
    Serial.println("[CFG] CURRENT END");
    return;
  }

  // ---- CFG begin / end ----
  if (line == "CFG BEGIN") {
    cfgReceiving = true;
    cfgBeginAt = millis();
    resetIncomingCfg();
    Serial.println("[CFG] BEGIN");
    return;
  }

  if (line == "CFG END") {
    if (!cfgReceiving) {
      Serial.println("[CFG] END without BEGIN -> ignored");
      return;
    }
    cfgReceiving = false;

    incomingCfg.mode = normalizeMode(incomingCfg.mode);
    bool wifiFieldsValid = false;

    // The Wi-Fi fields are required only when mode=wifi.
    // With mode=bt they can be saved unset (the Wi-Fi settings stay preserved).
    if (incomingCfg.ssid.length() == 0) {
      if (incomingCfg.mode == "wifi") {
        Serial.println("[CFG] INVALID: ssid missing");
        return;
      }
    } else if (incomingCfg.port.length() == 0) {
      if (incomingCfg.mode == "wifi") {
        Serial.println("[CFG] INVALID: port missing");
        return;
      }
    } else {
      uint16_t tmpPort;
      if (!parseTcpPort(incomingCfg.port, tmpPort)) {
        Serial.print("[CFG] INVALID: port out of range (1-65535): ");
        Serial.println(incomingCfg.port);
        if (incomingCfg.mode == "wifi") return;
      } else if (incomingCfg.localIP == IPAddress() ||
                 incomingCfg.gateway == IPAddress() ||
                 incomingCfg.subnet == IPAddress()) {
        Serial.println("[CFG] INVALID: ip/gw/sn missing or invalid");
        if (incomingCfg.mode == "wifi") return;
      } else {
        wifiFieldsValid = true;
      }
    }

    if (incomingCfg.mode == "wifi" && !wifiFieldsValid) {
      Serial.println("[CFG] INVALID: Wi-Fi fields incomplete for mode=wifi");
      return;
    }

    {
      const String &pw = incomingCfg.password;
      if (pw.length() && (pw[0] == ' ' || pw[pw.length() - 1] == ' ')) {
        Serial.println("[CFG] WARNING: password has leading/trailing space");
      }
      const String &si = incomingCfg.ssid;
      if (si.length() && (si[0] == ' ' || si[si.length() - 1] == ' ')) {
        Serial.println("[CFG] WARNING: ssid has leading/trailing space");
      }
    }

    if (saveLinkConfig(incomingCfg)) {
      Serial.println("[CFG] SAVED. Send RESET or reboot to apply this config.");
      gLinkConfig = incomingCfg;
      gLinkConfig.valid = wifiFieldsValid;
    } else {
      Serial.println("[CFG] SAVE FAILED");
    }
    return;
  }

  if (!cfgReceiving) {
    // Outside CFG mode, interpret the line as a gamepad / macro command
    // (so the same commands available over Wi-Fi can also be typed on serial)
    handleCommand(line, nullptr);
    return;
  }

  // ---- parse key=value lines while in CFG mode ----
  if (line.startsWith("ssid=")) {
    incomingCfg.ssid = line.substring(5);
    incomingCfg.ssid.trim();
    Serial.print("[CFG] ssid="); Serial.println(incomingCfg.ssid);
  } else if (line.startsWith("pass=")) {
    // Do not trim here either (it would break a password with trailing spaces)
    incomingCfg.password = line.substring(5);
    Serial.println("[CFG] pass=******");
  } else if (line.startsWith("ip=")) {
    String v = line.substring(3); v.trim();
    IPAddress ip;
    if (ip.fromString(v)) {
      incomingCfg.localIP = ip;
      Serial.print("[CFG] ip="); Serial.println(incomingCfg.localIP);
    } else {
      Serial.print("[CFG] ip parse error: "); Serial.println(v);
    }
  } else if (line.startsWith("gw=")) {
    String v = line.substring(3); v.trim();
    IPAddress gw;
    if (gw.fromString(v)) {
      incomingCfg.gateway = gw;
      Serial.print("[CFG] gw="); Serial.println(incomingCfg.gateway);
    } else {
      Serial.print("[CFG] gw parse error: "); Serial.println(v);
    }
  } else if (line.startsWith("sn=")) {
    String v = line.substring(3); v.trim();
    IPAddress sn;
    if (sn.fromString(v)) {
      incomingCfg.subnet = sn;
      Serial.print("[CFG] sn="); Serial.println(incomingCfg.subnet);
    } else {
      Serial.print("[CFG] sn parse error: "); Serial.println(v);
    }
  } else if (line.startsWith("port=")) {
    incomingCfg.port = line.substring(5);
    incomingCfg.port.trim();
    Serial.print("[CFG] port="); Serial.println(incomingCfg.port);
  } else if (line.startsWith("mode=")) {
    incomingCfg.mode = normalizeMode(line.substring(5));
    Serial.print("[CFG] mode="); Serial.println(incomingCfg.mode);
  } else if (line.startsWith("btname=")) {
    incomingCfg.btName = line.substring(7);
    incomingCfg.btName.trim();
    Serial.print("[CFG] btname="); Serial.println(incomingCfg.btName);
  } else if (line.startsWith("usbmode=")) {
    incomingCfg.usbMode = normalizeUsbMode(line.substring(8));
    Serial.print("[CFG] usbmode="); Serial.println(incomingCfg.usbMode);
  } else if (line.startsWith("ds4map=")) {
    String v = line.substring(7);
    int c1 = v.indexOf(',');
    int c2 = (c1 >= 0) ? v.indexOf(',', c1 + 1) : -1;
    incomingCfg.ds4MapC  = normalizeDs4Token((c1 >= 0) ? v.substring(0, c1) : v, "touchpad");
    incomingCfg.ds4MapGL = normalizeDs4Token((c1 >= 0 && c2 >= 0) ? v.substring(c1 + 1, c2) : "", "none");
    incomingCfg.ds4MapGR = normalizeDs4Token((c2 >= 0) ? v.substring(c2 + 1) : "", "none");
    Serial.print("[CFG] ds4map=");
    Serial.print(incomingCfg.ds4MapC); Serial.print(",");
    Serial.print(incomingCfg.ds4MapGL); Serial.print(",");
    Serial.println(incomingCfg.ds4MapGR);
  } else if (line.startsWith("switchmap=")) {
    String v = line.substring(10);
    int c1 = v.indexOf(',');
    int c2 = (c1 >= 0) ? v.indexOf(',', c1 + 1) : -1;
    incomingCfg.swMapC  = normalizeSwitchToken((c1 >= 0) ? v.substring(0, c1) : v, "none");
    incomingCfg.swMapGL = normalizeSwitchToken((c1 >= 0 && c2 >= 0) ? v.substring(c1 + 1, c2) : "", "none");
    incomingCfg.swMapGR = normalizeSwitchToken((c2 >= 0) ? v.substring(c2 + 1) : "", "none");
    Serial.print("[CFG] switchmap=");
    Serial.print(incomingCfg.swMapC); Serial.print(",");
    Serial.print(incomingCfg.swMapGL); Serial.print(",");
    Serial.println(incomingCfg.swMapGR);
  } else if (line.startsWith("macro=")) {
    String v = line.substring(6);
    v.trim();
    v.toLowerCase();
    incomingCfg.macroOn = (v == "on" || v == "1" || v == "true");
    Serial.print("[CFG] macro="); Serial.println(incomingCfg.macroOn ? "on" : "off");
  } else {
    Serial.print("[CFG] unknown line in CFG mode: ");
    Serial.println(line);
  }
}

// ==============================
// Switch HID / Macro
// ==============================
Adafruit_USBD_HID G_usb_hid;
NSGamepad Gamepad(&G_usb_hid);

// 1800 steps, and at most 24 characters per line (25 bytes including the terminator)
const int    MACRO_MAX_STEPS    = 1800;
const size_t MACRO_LINE_MAXLEN  = 24;

// aligned(4): in dongle mode this buffer is reinterpreted as DongleMacroSample[]
// (see the dongle macro recorder section), whose uint32 member needs 4-byte alignment
char     macroLines[MACRO_MAX_STEPS][MACRO_LINE_MAXLEN + 1] __attribute__((aligned(4)));
int      macroLength      = 0;
int      macroDropped     = 0;   // number of steps that could not be accepted during load
bool     macroLoaded      = false;
bool     macroLoading     = false;
bool     macroRunning     = false;

uint32_t macroIntervalMs  = 100;
int      macroIndex       = 0;
uint32_t macroNextTick    = 0;

bool     internalMacroPlayback = false;
bool     macroFirstPass         = true;   // report only on the first pass so loop playback does not keep returning errors

// ---- Button name table (shared by BTN and TAP. Adding a button only touches this) ----
struct ButtonEntry {
  const char* name;
  uint8_t     code;
};

const ButtonEntry BUTTON_TABLE[] = {
  { "A",       NSButton_A },
  { "B",       NSButton_B },
  { "X",       NSButton_X },
  { "Y",       NSButton_Y },
  { "L",       NSButton_LeftTrigger },
  { "R",       NSButton_RightTrigger },
  { "ZL",      NSButton_LeftThrottle },
  { "ZR",      NSButton_RightThrottle },
  { "PLUS",    NSButton_Plus },
  { "MINUS",   NSButton_Minus },
  { "HOME",    NSButton_Home },
  { "CAPTURE", NSButton_Capture },
  { "LSTICK",  NSButton_LeftStick },
  { "RSTICK",  NSButton_RightStick },
};
const int BUTTON_TABLE_LEN = sizeof(BUTTON_TABLE) / sizeof(BUTTON_TABLE[0]);

int lookupButton(const String& name) {
  for (int i = 0; i < BUTTON_TABLE_LEN; i++) {
    if (name == BUTTON_TABLE[i].name) return (int)BUTTON_TABLE[i].code;
  }
  return -1;
}

// ---- D-Pad direction name table (shared by DPAD and DTAP) ----
struct DpadEntry {
  const char*   name;
  NSDirection_t dir;
};

const DpadEntry DPAD_TABLE[] = {
  { "UP",        NSGAMEPAD_DPAD_UP },
  { "DOWN",      NSGAMEPAD_DPAD_DOWN },
  { "LEFT",      NSGAMEPAD_DPAD_LEFT },
  { "RIGHT",     NSGAMEPAD_DPAD_RIGHT },
  { "UPLEFT",    NSGAMEPAD_DPAD_UP_LEFT },
  { "UPRIGHT",   NSGAMEPAD_DPAD_UP_RIGHT },
  { "DOWNLEFT",  NSGAMEPAD_DPAD_DOWN_LEFT },
  { "DOWNRIGHT", NSGAMEPAD_DPAD_DOWN_RIGHT },
};
const int DPAD_TABLE_LEN = sizeof(DPAD_TABLE) / sizeof(DPAD_TABLE[0]);

int lookupDpad(const String& name) {
  for (int i = 0; i < DPAD_TABLE_LEN; i++) {
    if (name == DPAD_TABLE[i].name) return (int)DPAD_TABLE[i].dir;
  }
  return -1;
}

// ---- TAP (press, then release after the given ms) ----
// serviceTap() performs the release asynchronously, so HID reports keep flowing while held.
int      gTapButton         = -1;   // -1 = no pending release
uint32_t gTapReleaseAt      = 0;
bool     gDtapActive        = false;   // whether a D-Pad tap release (CENTER) is pending
uint32_t gDtapReleaseAt     = 0;
uint32_t gCommandExtraWaitMs = 0;   // extra wait the previous command requires (a TAP press duration)

// ---- Stick helpers ----
void setLeftStick(uint8_t x, uint8_t y) {
  Gamepad.leftXAxis(x);
  Gamepad.leftYAxis(y);
}

void setRightStick(uint8_t x, uint8_t y) {
  Gamepad.rightXAxis(x);
  Gamepad.rightYAxis(y);
}

void centerSticks() {
  setLeftStick(128, 128);
  setRightStick(128, 128);
}

void dpadCenter() {
  Gamepad.dPad(NSGAMEPAD_DPAD_CENTERED);
}

void sendReport() {
  // Dongle mode with usbmode=sinput/ds4/procon never calls Gamepad.begin(). Letting
  // Gamepad.ready() through then makes TinyUSB index its internal array with an
  // unallocated instance number (0xFF). (usbmode=switch DOES begin it and relays
  // through this same path.)
  if (!G_usb_hid.isValid()) return;
  // NSGamepad::loop() skips a send within 1ms of the previous one, so several commands
  // in the same millisecond would hide intermediate states from the Switch
  // (BTN A DOWN + BTN A UP in one packet loses the press). Always send after handling.
  if (Gamepad.ready()) Gamepad.write();
}

void startTap(uint8_t button, uint32_t ms) {
  // Release a leftover TAP first (so rapid manual presses cannot leave a button held)
  if (gTapButton >= 0) {
    Gamepad.release((uint8_t)gTapButton);
    // Repeated presses of the same button need the released state sent once, or press
    // and release collapse into one report and the host never sees the new press
    sendReport();
  }

  Gamepad.press(button);
  gTapButton    = (int)button;
  gTapReleaseAt = millis() + ms;
  sendReport();
}

void startDtap(NSDirection_t dir, uint32_t ms) {
  Gamepad.dPad(dir);
  gDtapActive    = true;
  gDtapReleaseAt = millis() + ms;
  sendReport();
}

void serviceTap() {
  uint32_t now = millis();

  if (gTapButton >= 0 && (int32_t)(now - gTapReleaseAt) >= 0) {
    Gamepad.release((uint8_t)gTapButton);
    gTapButton = -1;
    sendReport();
  }

  if (gDtapActive && (int32_t)(now - gDtapReleaseAt) >= 0) {
    Gamepad.dPad(NSGAMEPAD_DPAD_CENTERED);
    gDtapActive = false;
    sendReport();
  }
}

void cancelTap() {
  gTapButton  = -1;
  gDtapActive = false;
}

void clearMacro() {
  cancelTap();
  macroRunning  = false;
  macroLoading  = false;
  macroLoaded   = false;
  macroLength   = 0;
  macroDropped  = 0;
  macroIndex    = 0;
  macroFirstPass = true;
  if (MACRO_MAX_STEPS > 0) {
    macroLines[0][0] = '\0';
  }
}

// Case-insensitive prefix match (kept String-free so no allocation is needed)
bool startsWithNoCase(const char* str, const char* prefix) {
  while (*prefix) {
    if (toupper((unsigned char)*str) != toupper((unsigned char)*prefix)) return false;
    str++;
    prefix++;
  }
  return true;
}

// ---- Macro tick ----
void tickMacro() {
  if (!macroRunning || !macroLoaded) return;
  if (!USBDevice.mounted()) return;

  uint32_t now = millis();
  if ((int32_t)(now - macroNextTick) < 0) return;

  // Take it from the fixed-length buffer; a String per tick would fragment the heap
  const char* raw = macroLines[macroIndex];
  uint32_t wait;

  if (startsWithNoCase(raw, "SLEEP ")) {
    long extra = strtol(raw + 6, nullptr, 10);
    if (extra < 0) extra = 0;
    if (extra > 600000L) extra = 600000L;

    Serial.print("[MACRO SLEEP] ");
    Serial.print(extra);
    Serial.println(" ms");

    wait = (uint32_t)extra;
  } else {
    gMacroCmd = raw;

    Serial.print("[MACRO PLAY] ");
    Serial.println(gMacroCmd);

    // Pass the response destination so unparsable steps can be reported back to the app
    gCommandExtraWaitMs = 0;
    internalMacroPlayback = true;
    handleCommand(gMacroCmd, activeReplyTarget());
    internalMacroPlayback = false;

    // TAP waits the press duration on top (serviceTap performs the release)
    wait = macroIntervalMs + gCommandExtraWaitMs;
    gCommandExtraWaitMs = 0;
  }

  macroIndex++;
  if (macroIndex >= macroLength) {
    macroIndex     = 0;
    macroFirstPass = false;
  }

  // Add to the previous scheduled time so execution time does not shift the period.
  // If we fall far behind (a USB disconnect, say), re-baseline instead of running the backlog.
  macroNextTick += wait;
  uint32_t after = millis();
  if ((int32_t)(after - macroNextTick) > (int32_t)wait) {
    macroNextTick = after + wait;
  }
}

// ---- Command handler ----
void handleCommand(const String& rawLine, Print *client) {
  // Put it into the reusable buffer (handleCommand is not re-entrant)
  gCmdLine = rawLine;
  gCmdLine.trim();
  if (!gCmdLine.length()) return;

  gCmdUpper = gCmdLine;
  gCmdUpper.toUpperCase();

  const String& line  = gCmdLine;
  const String& upper = gCmdUpper;

  // In dongle mode the pad state belongs to the Pro Controller 2 relay (with
  // usbmode=switch these commands would actuate the same NSGamepad and fight it)
  // and macroLines belongs to the macro recorder (MACRO LOAD would corrupt a
  // recording). Reject the whole command set.
  if (gActiveTransport == "dongle") {
    Serial.println("[CMD] gamepad/macro commands not available in dongle mode");
    return;
  }

  // MACRO STOP
  if (upper == "MACRO STOP") {
    clearMacro();          // this also clears any pending TAP release

    Gamepad.releaseAll();
    dpadCenter();
    centerSticks();
    sendReport();

    Serial.println("[MACRO] STOP & CLEAR");
    return;
  }

  // MACRO LOAD is accepted even while running. (Behind the "ignore while running" guard
  //  below it would drop LOAD/steps/END/START and the macro could never be replaced)
  if (upper.startsWith("MACRO LOAD ")) {
    int s = upper.lastIndexOf(' ');
    int iv = upper.substring(s + 1).toInt();
    if (iv < 10) iv = 10;
    macroIntervalMs = (uint32_t)iv;

    clearMacro();          // this also clears macroRunning
    macroLoading = true;

    // The previous macro may have ended with buttons held, so reset the input
    cancelTap();
    Gamepad.releaseAll();
    dpadCenter();
    centerSticks();
    sendReport();

    Serial.print("[MACRO] LOAD start, interval = ");
    Serial.print(macroIntervalMs);
    Serial.println(" ms");
    return;
  }

  // ignore external commands while macro running
  if (macroRunning && !internalMacroPlayback) {
    Serial.print("[MACRO] running, ignored: ");
    Serial.println(upper);
    return;
  }

  if (upper == "MACRO END") {
    macroLoading = false;
    macroLoaded  = (macroLength > 0);
    Serial.print("[MACRO] LOAD end. steps = ");
    Serial.print(macroLength);
    Serial.print(", dropped = ");
    Serial.println(macroDropped);

    // Return the result so the app can reconcile it against the number of steps it sent
    if (client) {
      client->print("MACRO LOADED ");
      client->print(macroLength);
      client->print(" ");
      client->println(macroDropped);
    }
    return;
  }

  if (upper == "MACRO START") {
    if (macroLoaded && macroLength > 0) {
      macroRunning   = true;
      macroIndex     = 0;
      macroFirstPass = true;
      macroNextTick  = millis();
      Serial.println("[MACRO] START");
    } else {
      Serial.println("[MACRO] START requested but no macro loaded");
    }
    return;
  }

  if (macroLoading) {
    if (macroLength >= MACRO_MAX_STEPS) {
      macroDropped++;
      if (macroDropped == 1) {
        Serial.print("[MACRO] buffer full (");
        Serial.print(MACRO_MAX_STEPS);
        Serial.println(" steps), dropping the rest");
      }
      return;
    }

    // A truncated step becomes an invalid line silently ignored at run time; discard and count it
    if (line.length() > MACRO_LINE_MAXLEN) {
      macroDropped++;
      Serial.print("[MACRO] step too long, dropped: ");
      Serial.println(line);
      return;
    }

    line.toCharArray(macroLines[macroLength], MACRO_LINE_MAXLEN + 1);
    macroLength++;

    // Printing every line would occupy serial for a long time on an 1800-line load, so thin it out
    if ((macroLength % 200) == 0) {
      Serial.print("[MACRO] loaded ");
      Serial.print(macroLength);
      Serial.println(" steps...");
    }
    return;
  }

  // ===== Normal commands from here =====

  // ALL UP (checked before "BTN <name> ...")
  if (upper == "BTN ALL UP") {
    cancelTap();
    Gamepad.releaseAll();
    dpadCenter();
    centerSticks();
    sendReport();
    return;
  }

  // BTN <NAME> DOWN / BTN <NAME> UP
  if (upper.startsWith("BTN ")) {
    int sp = upper.indexOf(' ', 4);
    if (sp > 4) {
      int code = lookupButton(upper.substring(4, sp));
      String act = upper.substring(sp + 1);
      if (code >= 0) {
        if (act == "DOWN") {
          Gamepad.press((uint8_t)code);
          sendReport();
          return;
        }
        if (act == "UP") {
          // Releasing the same button also clears its pending TAP release. Only the
          // button side (clearing the DTAP CENTER reservation would stick the D-Pad)
          if (gTapButton == code) gTapButton = -1;
          Gamepad.release((uint8_t)code);
          sendReport();
          return;
        }
      }
    }
    // Falling through here means an unknown command (handled by the else at the end)
  }

  // DTAP <DIR> <ms> : hold the D-Pad for <ms>, then return to neutral automatically.
  // Works like TAP (serviceTap sends CENTER asynchronously).
  if (upper.startsWith("DTAP ")) {
    int sp = upper.indexOf(' ', 5);
    if (sp > 5) {
      int dir = lookupDpad(upper.substring(5, sp));
      long ms = upper.substring(sp + 1).toInt();
      if (dir >= 0 && ms > 0) {
        if (ms > 60000L) ms = 60000L;
        startDtap((NSDirection_t)dir, (uint32_t)ms);
        gCommandExtraWaitMs = (uint32_t)ms;
        return;
      }
    }
    // A malformed DTAP is treated as an unknown command
  }

  // TAP <NAME> <ms> : press, then release after <ms>.
  // serviceTap() performs the release, so this does not block.
  // Inside a macro, tickMacro waits the press duration before advancing to the next step.
  if (upper.startsWith("TAP ")) {
    int sp = upper.indexOf(' ', 4);
    if (sp > 4) {
      int code = lookupButton(upper.substring(4, sp));
      long ms  = upper.substring(sp + 1).toInt();
      if (code >= 0 && ms > 0) {
        if (ms > 60000L) ms = 60000L;
        startTap((uint8_t)code, (uint32_t)ms);
        gCommandExtraWaitMs = (uint32_t)ms;
        return;
      }
    }
    // A malformed TAP is treated as an unknown command
  }

  // D-Pad (a manual direction discards a pending DTAP release, but only for a valid
  // direction; discarding on an unknown "DPAD FOO" would stick the D-Pad)
  if (upper == "DPAD CENTER")         { gDtapActive = false; dpadCenter(); }

  else if (upper == "DPAD UP")        { gDtapActive = false; Gamepad.dPad(NSGAMEPAD_DPAD_UP); }
  else if (upper == "DPAD DOWN")      { gDtapActive = false; Gamepad.dPad(NSGAMEPAD_DPAD_DOWN); }
  else if (upper == "DPAD LEFT")      { gDtapActive = false; Gamepad.dPad(NSGAMEPAD_DPAD_LEFT); }
  else if (upper == "DPAD RIGHT")     { gDtapActive = false; Gamepad.dPad(NSGAMEPAD_DPAD_RIGHT); }
  else if (upper == "DPAD UPLEFT")    { gDtapActive = false; Gamepad.dPad(NSGAMEPAD_DPAD_UP_LEFT); }
  else if (upper == "DPAD UPRIGHT")   { gDtapActive = false; Gamepad.dPad(NSGAMEPAD_DPAD_UP_RIGHT); }
  else if (upper == "DPAD DOWNLEFT")  { gDtapActive = false; Gamepad.dPad(NSGAMEPAD_DPAD_DOWN_LEFT); }
  else if (upper == "DPAD DOWNRIGHT") { gDtapActive = false; Gamepad.dPad(NSGAMEPAD_DPAD_DOWN_RIGHT); }

  // Stick presets
  else if (upper == "LSTICK CENTER") setLeftStick(128,128);
  else if (upper == "LSTICK UP")     setLeftStick(128,0);
  else if (upper == "LSTICK DOWN")   setLeftStick(128,255);
  else if (upper == "LSTICK LEFT")   setLeftStick(0,128);
  else if (upper == "LSTICK RIGHT")  setLeftStick(255,128);

  else if (upper == "RSTICK CENTER") setRightStick(128,128);
  else if (upper == "RSTICK UP")     setRightStick(128,0);
  else if (upper == "RSTICK DOWN")   setRightStick(128,255);
  else if (upper == "RSTICK LEFT")   setRightStick(0,128);
  else if (upper == "RSTICK RIGHT")  setRightStick(255,128);

  // Left stick numeric
  else if (upper.startsWith("LSTICK ")) {
    int s1 = line.indexOf(' ');
    int s2 = line.indexOf(' ', s1 + 1);
    if (s1 > 0 && s2 > s1) {
      uint8_t x = line.substring(s1 + 1, s2).toInt();
      uint8_t y = line.substring(s2 + 1).toInt();
      setLeftStick(x, y);
    }
  }

  // Right stick numeric
  else if (upper.startsWith("RSTICK ")) {
    int s1 = line.indexOf(' ');
    int s2 = line.indexOf(' ', s1 + 1);
    if (s1 > 0 && s2 > s1) {
      uint8_t x = line.substring(s1 + 1, s2).toInt();
      uint8_t y = line.substring(s2 + 1).toInt();
      setRightStick(x, y);
    }
  }

  // VERSION (Wi-Fi & Serial)
  else if (upper == "VERSION") {
    Serial.print("[FW] VERSION ");
    Serial.println(FW_VERSION);
    if (client) {
      client->println(FW_VERSION);
    }
  }

  // Do not silently drop a line we could not parse; return it to the sender.
  // But macros loop, so do not send it after the first pass (that would flood the TCP link)
  else {
    Serial.print("[CMD] unknown command: ");
    Serial.println(line);
    if (client && (!internalMacroPlayback || macroFirstPass)) {
      client->print("ERR unknown command: ");
      client->println(line);
    }
    return;
  }

  sendReport();
}

// ==============================
// Wi-Fi supervisor (non-blocking auto reconnect)
//   When the link drops, tear the server down and retry the connection every 10 seconds
// ==============================
uint8_t  gWifiDownTicks   = 0;   // consecutive seconds the link has not been UP
uint32_t gWifiJoinSince   = 0;   // when JOIN/NOIP (the connect handshake) started
uint32_t gWifiReportNext  = 0;   // when to print the next status report while disconnected

// NOTE: always read radio state through the official WiFi.status(). Calling
// cyw43_tcpip_link_status() directly without the lock has been observed on real
// hardware to leave the driver's connect (join) sequence unable to complete.
const char* wlanStatusName(int st) {
  switch (st) {
    case WL_CONNECTED:      return "CONNECTED";
    case WL_CONNECT_FAILED: return "CONNECT_FAILED";
    case WL_NO_SSID_AVAIL:  return "NO_SSID_AVAIL (SSID not found)";
    case WL_CONNECTION_LOST: return "CONNECTION_LOST";
    case WL_DISCONNECTED:   return "DISCONNECTED";
    case WL_IDLE_STATUS:    return "IDLE (connecting)";
    default:                return "UNKNOWN";
  }
}

// Starts Wi-Fi and waits for the link (up to timeoutMs). It blocks like the old
// implementation, but HID reports and TAP releases keep running, so input never stalls.
bool gWlanConnecting = false;

bool wlanBeginAndWait(uint32_t timeoutMs) {
  if (gWlanConnecting) return false;   // guards against re-entry from RESET and the like
  gWlanConnecting = true;

  // NOTE: arduino-pico's 3-argument config means (ip, dns, gateway).
  // Use the 4-argument form (ip, dns, gateway, subnet) to pass a subnet.
  WiFi.config(gLinkConfig.localIP, gLinkConfig.gateway,
              gLinkConfig.gateway, gLinkConfig.subnet);
  WiFi.begin(gLinkConfig.ssid.c_str(), gLinkConfig.password.c_str());

  uint32_t t0 = millis();
  int lastSt = -100;
  bool ok = false;

  while ((int32_t)(millis() - t0) < (int32_t)timeoutMs) {
    int st = WiFi.status();
    if (st != lastSt) {
      lastSt = st;
      Serial.print("[WLAN] status: ");
      Serial.println(wlanStatusName(st));
    }
    if (st == WL_CONNECTED) {
      ok = true;
      break;
    }
    if (st == WL_NO_SSID_AVAIL) {
      break;   // a failure that waiting will not fix
    }
    delay(100);
    yield();
    // Keep serial, HID and macros alive while waiting (RESET and other radio commands
    // are refused by the gWlanConnecting guard). processWifiClientTask() does nothing
    // while wifiStarted is false, so it is not called.
    processSerialInput();
    serviceTap();
    tickMacro();
    if (Gamepad.ready()) Gamepad.loop();
  }

  gWlanConnecting = false;
  return ok;
}

void processWifiSupervisorTask() {
  if (!gWifiEnabled || !gLinkConfig.valid) return;

  uint32_t now = millis();
  if ((int32_t)(now - gWifiSuperviseNext) < 0) return;
  gWifiSuperviseNext = now + 1000;

  int st = WiFi.status();

  if (st == WL_CONNECTED) {
    gWifiDownTicks = 0;
    gWifiJoinSince = 0;
    if (!wifiStarted) {
      Serial.print("[WLAN] router connected, IP = ");
      Serial.println(WiFi.localIP());
      if (startTcpServer()) {
        wifiStarted = true;
        Serial.print("[TCP] server listening on port ");
        Serial.print(gTcpPort);
        Serial.println(" (waiting for app)");
      } else {
        Serial.println("[TCP] server start failed");
      }
    }
    return;
  }

  // ---- Disconnected: report the current state every 5 seconds (never wait silently) ----
  if ((int32_t)(now - gWifiReportNext) >= 0) {
    gWifiReportNext = now + 5000;
    Serial.print("[WLAN] status: ");
    Serial.println(wlanStatusName(st));
  }

  // A connect in progress (IDLE) is neither failure nor success, and calling begin()
  // again would break the handshake. Wait, but retry after 30 seconds without progress.
  if (st == WL_IDLE_STATUS) {
    if (gWifiJoinSince == 0) {
      gWifiJoinSince = now;
    } else if ((int32_t)(now - gWifiJoinSince) > 30000) {
      Serial.println("[WLAN] stuck while connecting -> restarting connection");
      gWifiJoinSince = 0;
      WiFi.disconnect();
      gWifiRetryNext = now + 2000;   // wait a moment before calling begin() again
    }
    return;
  }
  gWifiJoinSince = 0;

  // ---- link down / failed ----
  if (wifiStarted) {
    // Do not call a momentary glitch a disconnect (confirm it over 5 consecutive seconds)
    gWifiDownTicks++;
    if (gWifiDownTicks < 5) return;

    Serial.print("[WLAN] router link lost: ");
    Serial.println(wlanStatusName(st));

    // Keep the server; only drop the client
    if (gCurrentClient) {
      gCurrentClient.stop();
    }
    gWifiLineBuffer = "";
    wifiStarted     = false;
    gWifiDownTicks  = 0;
    gWifiRetryNext  = now;
  }

  // Retry every 10 seconds
  if ((int32_t)(now - gWifiRetryNext) < 0) return;
  gWifiRetryNext = now + 10000;

  Serial.print("[WLAN] reconnecting to router... (last state: ");
  Serial.print(wlanStatusName(st));
  Serial.println(")");
  wlanBeginAndWait(15000);   // wait until the connection completes or definitively fails (HID keeps running)
}

// ==============================
// Wi-Fi task (non-blocking)
// ==============================
void processWifiClientTask() {
  if (!wifiStarted || !gServer) return;

  // Check whether the client disconnected since last time
  if (gCurrentClient && !gCurrentClient.connected()) {
    Serial.println("[TCP] app disconnected");
    gCurrentClient.stop();
    gWifiLineBuffer = "";
  }

  // Always accept a new connection, replacing any existing client: when the peer dies
  // without FIN (force-quit, sleep, out of range) connected() stays true forever.
  WiFiClient newClient = gServer->accept();
  if (newClient) {
    if (gCurrentClient && gCurrentClient.connected()) {
      Serial.println("[TCP] new app connection -> dropping previous one");
      gCurrentClient.stop();
    }
    gCurrentClient  = newClient;
    gWifiLineBuffer = "";
    Serial.println("[TCP] app connected");
  }

  // Handle I/O if a client is connected
  if (gCurrentClient && gCurrentClient.connected()) {
    while (gCurrentClient.available()) {
      char c = gCurrentClient.read();
      if (c == '\n') {
        String line = gWifiLineBuffer;
        line.trim();
        if (line.length()) {
          Serial.print("CMD: ");
          Serial.println(line);
          handleCommand(line, &gCurrentClient);
        }
        gWifiLineBuffer = "";
      } else if (c != '\r') {
        gWifiLineBuffer += c;
      }
      yield();
    }
  }
}

// ==============================
// BLE task (non-blocking)
//   The BT stack queues incoming data inside BleLinkService; here we only assemble
//   lines and hand them to handleCommand.
// ==============================
void processBleTask() {
  if (gActiveTransport != "bt" || !gBleStarted) return;

  bool nowConnected = gBleLink.connected();
  if (nowConnected != gBleWasConnected) {
    gBleWasConnected = nowConnected;
    gBleLineBuffer = "";
    Serial.println(nowConnected ? "[BLE] app connected" : "[BLE] app disconnected");
  }

  if (gBleLink.overflow()) {
    // A line with dropped bytes may be corrupt. The MACRO LOADED reconciliation detects it.
    Serial.println("[BLE] RX overflow: some data was dropped");
  }

  if (gBleLink.txOverflow()) {
    // Part of a response was lost. Notify the app so it sees a reason rather than just "no reply"
    Serial.println("[BLE] TX overflow: some response was dropped");
    gBleLink.println("[BLE] TX OVERFLOW");
    // The notification itself can overflow and re-set the flag, logging every iteration
    // until the queue drains, so drop it here.
    gBleLink.txOverflow();
  }

  int c;
  while ((c = gBleLink.read()) >= 0) {
    if (c == '\n') {
      String line = gBleLineBuffer;
      line.trim();
      if (line.length()) {
        Serial.print("CMD: ");
        Serial.println(line);
        handleCommand(line, gBleLink.connected() ? (Print*)&gBleLink : nullptr);
      }
      gBleLineBuffer = "";
    } else if (c != '\r') {
      gBleLineBuffer += (char)c;
    }
    yield();
  }

  gBleLink.pump();   // send queued responses as notifications
}

// ==============================
// Dongle task (mode=dongle)
//   Converts the full input report from the Pro Controller 2 into SInput (a USB identity
//   supported by SDL3) and relays rumble requests from the host back to the controller.
// ==============================

// Raw 12-bit stick value -> SInput s16. Center 2048, Y inverted
// (the Pro Controller 2 reports larger values upward, SDL treats down as positive)
// ---- Automatic stick center calibration ----
// A Pro Controller 2's stick center is off from 2048 by a few dozen counts per unit.
// Average the first few reports after connecting (sticks should be neutral right after
// the sync button) and use that as the center. Samples far from center are assumed to
// mean someone is touching it and are dropped; an axis with no valid samples keeps 2048.
const uint8_t DONGLE_CAL_SAMPLES = 16;      // 32Hz, so about 0.5 seconds
const int DONGLE_CAL_TOLERANCE = 400;
uint16_t gDongleCenter[4] = { 2048, 2048, 2048, 2048 };   // lx, ly, rx, ry
uint32_t gDongleCalSum[4];
uint16_t gDongleCalCount[4];
uint8_t gDongleCalReports = 0;
bool gDongleCalibrating = false;

void dongleStartCalibration() {
  for (int i = 0; i < 4; i++) {
    gDongleCenter[i] = 2048;
    gDongleCalSum[i] = 0;
    gDongleCalCount[i] = 0;
  }
  gDongleCalReports = 0;
  gDongleCalibrating = true;
}

void dongleCalibrate(const Procon2Report &r) {
  const uint16_t raw[4] = { r.lx, r.ly, r.rx, r.ry };
  for (int i = 0; i < 4; i++) {
    if (abs((long)raw[i] - 2048) <= DONGLE_CAL_TOLERANCE) {
      gDongleCalSum[i] += raw[i];
      gDongleCalCount[i]++;
    }
  }
  if (++gDongleCalReports < DONGLE_CAL_SAMPLES) return;

  for (int i = 0; i < 4; i++) {
    if (gDongleCalCount[i] > 0) {
      gDongleCenter[i] = (uint16_t)(gDongleCalSum[i] / gDongleCalCount[i]);
    }
  }
  gDongleCalibrating = false;
  Serial.printf("[DONGLE] stick centers: L=(%u,%u) R=(%u,%u)\n",
                gDongleCenter[0], gDongleCenter[1], gDongleCenter[2], gDongleCenter[3]);
}

int16_t dongleScaleStick(uint16_t raw12, bool invert, uint16_t center) {
  long d = (long)raw12 - center;
  if (invert) d = -d;
  long v = d * 16;
  if (v < -32768) v = -32768;
  if (v > 32767) v = 32767;
  return (int16_t)v;
}

// For DS4: 12-bit -> 8-bit (center 128). invert is for the Y axis (DS4 treats down as positive)
uint8_t dongleScaleStick8(uint16_t raw12, bool invert, uint16_t center) {
  long d = (long)raw12 - center;
  if (invert) d = -d;
  long v = 128 + d / 16;
  if (v < 0) v = 0;
  if (v > 255) v = 255;
  return (uint8_t)v;
}

void buildSInputFromProcon2(const Procon2Report &r, SInputState &s) {
  const uint32_t b = r.buttons;
  uint8_t b1 = 0, b2 = 0, b3 = 0, b4 = 0;

  // buttons_1: position-based face buttons (south/east/west/north) + D-Pad.
  // The host follows the face style in features (Nintendo) when interpreting the labels
  if (b & P2_BTN_B) b1 |= 0x01;       // south
  if (b & P2_BTN_A) b1 |= 0x02;       // east
  if (b & P2_BTN_Y) b1 |= 0x04;       // west
  if (b & P2_BTN_X) b1 |= 0x08;       // north
  if (b & P2_BTN_UP) b1 |= 0x10;
  if (b & P2_BTN_DOWN) b1 |= 0x20;
  if (b & P2_BTN_LEFT) b1 |= 0x40;
  if (b & P2_BTN_RIGHT) b1 |= 0x80;

  // buttons_2: stick clicks + shoulders + triggers + GL/GR (paddle 1)
  if (b & P2_BTN_LSTICK) b2 |= 0x01;
  if (b & P2_BTN_RSTICK) b2 |= 0x02;
  if (b & P2_BTN_L) b2 |= 0x04;
  if (b & P2_BTN_R) b2 |= 0x08;
  if (b & P2_BTN_ZL) b2 |= 0x10;
  if (b & P2_BTN_ZR) b2 |= 0x20;
  if (b & P2_BTN_GL) b2 |= 0x40;
  if (b & P2_BTN_GR) b2 |= 0x80;

  // buttons_3: system buttons
  if (b & P2_BTN_PLUS) b3 |= 0x01;    // start
  if (b & P2_BTN_MINUS) b3 |= 0x02;   // select
  if (b & P2_BTN_HOME) b3 |= 0x04;    // guide
  if (b & P2_BTN_CAPTURE) b3 |= 0x08; // share

  // buttons_4: the C button -> misc_4
  if (b & P2_BTN_C) b4 |= 0x02;

  s.buttons[0] = b1;
  s.buttons[1] = b2;
  s.buttons[2] = b3;
  s.buttons[3] = b4;

  s.lx = dongleScaleStick(r.lx, false, gDongleCenter[0]);
  s.ly = dongleScaleStick(r.ly, true, gDongleCenter[1]);
  s.rx = dongleScaleStick(r.rx, false, gDongleCenter[2]);
  s.ry = dongleScaleStick(r.ry, true, gDongleCenter[3]);

  s.timestampUs = micros();
  for (int i = 0; i < 3; i++) {
    s.accel[i] = r.accel[i];
    s.gyro[i] = r.gyro[i];
  }
}

// ---- DS4 conversion ----

// Applies a DS4 assignment token to the state (used to map C/GL/GR)
void ds4ApplyToken(const String &tok, DS4State &s) {
  if (tok == "touchpad") s.buttons3 |= 0x02;
  else if (tok == "ps") s.buttons3 |= 0x01;
  else if (tok == "share") s.buttons2 |= 0x10;
  else if (tok == "options") s.buttons2 |= 0x20;
  else if (tok == "l1") s.buttons2 |= 0x01;
  else if (tok == "r1") s.buttons2 |= 0x02;
  else if (tok == "l2") { s.buttons2 |= 0x04; s.l2 = 255; }
  else if (tok == "r2") { s.buttons2 |= 0x08; s.r2 = 255; }
  else if (tok == "l3") s.buttons2 |= 0x40;
  else if (tok == "r3") s.buttons2 |= 0x80;
  else if (tok == "cross") s.face |= 0x20;
  else if (tok == "circle") s.face |= 0x40;
  else if (tok == "square") s.face |= 0x10;
  else if (tok == "triangle") s.face |= 0x80;
  // "none" is ignored
}

uint8_t ds4HatFromButtons(uint32_t b) {
  const bool up = b & P2_BTN_UP, down = b & P2_BTN_DOWN;
  const bool left = b & P2_BTN_LEFT, right = b & P2_BTN_RIGHT;
  if (up && right) return 1;
  if (down && right) return 3;
  if (down && left) return 5;
  if (up && left) return 7;
  if (up) return 0;
  if (right) return 2;
  if (down) return 4;
  if (left) return 6;
  return 8;   // neutral
}

void buildDS4FromProcon2(const Procon2Report &r, DS4State &s) {
  s = {};
  const uint32_t b = r.buttons;

  s.hat = ds4HatFromButtons(b);
  if (b & P2_BTN_B) s.face |= 0x20;         // Cross (south)
  if (b & P2_BTN_A) s.face |= 0x40;         // Circle (east)
  if (b & P2_BTN_Y) s.face |= 0x10;         // Square (west)
  if (b & P2_BTN_X) s.face |= 0x80;         // Triangle (north)
  if (b & P2_BTN_L) s.buttons2 |= 0x01;     // L1
  if (b & P2_BTN_R) s.buttons2 |= 0x02;     // R1
  if (b & P2_BTN_ZL) { s.buttons2 |= 0x04; s.l2 = 255; }
  if (b & P2_BTN_ZR) { s.buttons2 |= 0x08; s.r2 = 255; }
  if (b & P2_BTN_MINUS) s.buttons2 |= 0x10; // Share
  if (b & P2_BTN_PLUS) s.buttons2 |= 0x20;  // Options
  if (b & P2_BTN_LSTICK) s.buttons2 |= 0x40;
  if (b & P2_BTN_RSTICK) s.buttons2 |= 0x80;
  if (b & P2_BTN_HOME) s.buttons3 |= 0x01;      // PS
  if (b & P2_BTN_CAPTURE) s.buttons3 |= 0x02;   // touchpad click

  // C / GL / GR follow their configured assignments
  if (b & P2_BTN_C) ds4ApplyToken(gLinkConfig.ds4MapC, s);
  if (b & P2_BTN_GL) ds4ApplyToken(gLinkConfig.ds4MapGL, s);
  if (b & P2_BTN_GR) ds4ApplyToken(gLinkConfig.ds4MapGR, s);

  // Sticks: 12-bit -> 8-bit. DS4 treats Y as positive downward, so invert
  s.lx = dongleScaleStick8(r.lx, false, gDongleCenter[0]);
  s.ly = dongleScaleStick8(r.ly, true, gDongleCenter[1]);
  s.rx = dongleScaleStick8(r.rx, false, gDongleCenter[2]);
  s.ry = dongleScaleStick8(r.ry, true, gDongleCenter[3]);

  // IMU: raw Pro Controller 2 values -> DS4 units (gyro 14.286 -> 16.384 LSB/dps, accel 4096 -> 8192 LSB/g)
  for (int i = 0; i < 3; i++) {
    long g = (long)r.gyro[i] * 11469 / 10000;
    if (g > 32767) g = 32767;
    if (g < -32768) g = -32768;
    s.gyro[i] = (int16_t)g;
    long a = (long)r.accel[i] * 2;
    if (a > 32767) a = 32767;
    if (a < -32768) a = -32768;
    s.accel[i] = (int16_t)a;
  }
}

// ---- Switch pad conversion (usbmode=switch: Pro Controller 2 -> Switch/Switch 2) ----
// The HORI-compatible identity has no IMU and no rumble. C/GL/GR do not exist on it,
// so switchmap assigns them to existing pad buttons (default none). The recorder's
// feedback vibration still works (it goes to the controller over BLE, not to the
// USB host).

// Assignment token -> NSButton bit ("none" and unknown -> 0)
uint16_t switchTokenBits(const String &tok) {
  if (tok == "a") return 1 << NSButton_A;
  if (tok == "b") return 1 << NSButton_B;
  if (tok == "x") return 1 << NSButton_X;
  if (tok == "y") return 1 << NSButton_Y;
  if (tok == "l") return 1 << NSButton_LeftTrigger;
  if (tok == "r") return 1 << NSButton_RightTrigger;
  if (tok == "zl") return 1 << NSButton_LeftThrottle;
  if (tok == "zr") return 1 << NSButton_RightThrottle;
  if (tok == "plus") return 1 << NSButton_Plus;
  if (tok == "minus") return 1 << NSButton_Minus;
  if (tok == "home") return 1 << NSButton_Home;
  if (tok == "capture") return 1 << NSButton_Capture;
  if (tok == "lstick") return 1 << NSButton_LeftStick;
  if (tok == "rstick") return 1 << NSButton_RightStick;
  return 0;
}

void dongleApplySwitchReport(const Procon2Report &r) {
  const uint32_t b = r.buttons;
  uint16_t btns = 0;
  if (b & P2_BTN_Y) btns |= 1 << NSButton_Y;
  if (b & P2_BTN_B) btns |= 1 << NSButton_B;
  if (b & P2_BTN_A) btns |= 1 << NSButton_A;
  if (b & P2_BTN_X) btns |= 1 << NSButton_X;
  if (b & P2_BTN_L) btns |= 1 << NSButton_LeftTrigger;
  if (b & P2_BTN_R) btns |= 1 << NSButton_RightTrigger;
  if (b & P2_BTN_ZL) btns |= 1 << NSButton_LeftThrottle;
  if (b & P2_BTN_ZR) btns |= 1 << NSButton_RightThrottle;
  if (b & P2_BTN_MINUS) btns |= 1 << NSButton_Minus;
  if (b & P2_BTN_PLUS) btns |= 1 << NSButton_Plus;
  if (b & P2_BTN_LSTICK) btns |= 1 << NSButton_LeftStick;
  if (b & P2_BTN_RSTICK) btns |= 1 << NSButton_RightStick;
  if (b & P2_BTN_HOME) btns |= 1 << NSButton_Home;
  if (b & P2_BTN_CAPTURE) btns |= 1 << NSButton_Capture;

  // C / GL / GR follow their configured assignments. With macro=on the C bit was
  // already stripped upstream (C is the recorder's control button), so only the
  // GL/GR assignments can fire there
  if (b & P2_BTN_C) btns |= switchTokenBits(gLinkConfig.swMapC);
  if (b & P2_BTN_GL) btns |= switchTokenBits(gLinkConfig.swMapGL);
  if (b & P2_BTN_GR) btns |= switchTokenBits(gLinkConfig.swMapGR);

  Gamepad.buttons(btns);

  Gamepad.dPad((b & P2_BTN_UP) != 0, (b & P2_BTN_DOWN) != 0,
               (b & P2_BTN_LEFT) != 0, (b & P2_BTN_RIGHT) != 0);

  // Same 12-bit -> 8-bit scaling as DS4 (the Switch pad also treats down as positive)
  Gamepad.leftXAxis(dongleScaleStick8(r.lx, false, gDongleCenter[0]));
  Gamepad.leftYAxis(dongleScaleStick8(r.ly, true, gDongleCenter[1]));
  Gamepad.rightXAxis(dongleScaleStick8(r.rx, false, gDongleCenter[2]));
  Gamepad.rightYAxis(dongleScaleStick8(r.ry, true, gDongleCenter[3]));

  sendReport();
}

// ---- Pro Controller conversion (usbmode=procon: gyro + rumble on Switch/Switch 2) ----
// Buttons and sticks map 1:1 (same console family, sticks stay raw 12-bit but are
// re-centered to 2048 with our calibration). The IMU is rotated back into the
// original Pro Controller's frame (see the comment in the function) and the gyro
// rescaled from the Pro Controller 2's 14.286 LSB/dps to the Pro Controller's 16.4
// (accel is 4096 LSB/g on both). C/GL/GR do not exist on a Pro Controller, so they
// follow switchmap (the token set is the same Switch button vocabulary).

// switchmap token -> (procon button byte index, bit)
void proconApplyToken(const String &tok, uint8_t *btns) {
  if (tok == "a") btns[0] |= 0x08;
  else if (tok == "b") btns[0] |= 0x04;
  else if (tok == "x") btns[0] |= 0x02;
  else if (tok == "y") btns[0] |= 0x01;
  else if (tok == "r") btns[0] |= 0x40;
  else if (tok == "zr") btns[0] |= 0x80;
  else if (tok == "minus") btns[1] |= 0x01;
  else if (tok == "plus") btns[1] |= 0x02;
  else if (tok == "rstick") btns[1] |= 0x04;
  else if (tok == "lstick") btns[1] |= 0x08;
  else if (tok == "home") btns[1] |= 0x10;
  else if (tok == "capture") btns[1] |= 0x20;
  else if (tok == "l") btns[2] |= 0x40;
  else if (tok == "zl") btns[2] |= 0x80;
  // "none" is ignored
}

uint16_t proconRecenter(uint16_t raw, uint16_t center) {
  long v = (long)raw - center + 2048;
  if (v < 0) v = 0;
  if (v > 4095) v = 4095;
  return (uint16_t)v;
}

void buildProconFromProcon2(const Procon2Report &r, ProconState &s) {
  const uint32_t b = r.buttons;
  memset(&s, 0, sizeof(s));

  if (b & P2_BTN_Y) s.buttons[0] |= 0x01;
  if (b & P2_BTN_X) s.buttons[0] |= 0x02;
  if (b & P2_BTN_B) s.buttons[0] |= 0x04;
  if (b & P2_BTN_A) s.buttons[0] |= 0x08;
  if (b & P2_BTN_R) s.buttons[0] |= 0x40;
  if (b & P2_BTN_ZR) s.buttons[0] |= 0x80;
  if (b & P2_BTN_MINUS) s.buttons[1] |= 0x01;
  if (b & P2_BTN_PLUS) s.buttons[1] |= 0x02;
  if (b & P2_BTN_RSTICK) s.buttons[1] |= 0x04;
  if (b & P2_BTN_LSTICK) s.buttons[1] |= 0x08;
  if (b & P2_BTN_HOME) s.buttons[1] |= 0x10;
  if (b & P2_BTN_CAPTURE) s.buttons[1] |= 0x20;
  if (b & P2_BTN_DOWN) s.buttons[2] |= 0x01;
  if (b & P2_BTN_UP) s.buttons[2] |= 0x02;
  if (b & P2_BTN_RIGHT) s.buttons[2] |= 0x04;
  if (b & P2_BTN_LEFT) s.buttons[2] |= 0x08;
  if (b & P2_BTN_L) s.buttons[2] |= 0x40;
  if (b & P2_BTN_ZL) s.buttons[2] |= 0x80;

  // C / GL / GR follow switchmap (macro=on strips C upstream, same as everywhere)
  if (b & P2_BTN_C) proconApplyToken(gLinkConfig.swMapC, s.buttons);
  if (b & P2_BTN_GL) proconApplyToken(gLinkConfig.swMapGL, s.buttons);
  if (b & P2_BTN_GR) proconApplyToken(gLinkConfig.swMapGR, s.buttons);

  s.lx = proconRecenter(r.lx, gDongleCenter[0]);
  s.ly = proconRecenter(r.ly, gDongleCenter[1]);
  s.rx = proconRecenter(r.rx, gDongleCenter[2]);
  s.ry = proconRecenter(r.ry, gDongleCenter[3]);

  // The Pro Controller 2's IMU frame is the original Pro Controller's rotated
  // -90 degrees about the vertical (yaw) axis: on real hardware pitch and roll
  // arrived swapped (lifting one grip moved the aim vertically, tilting
  // forward/back did nothing) while yaw was correct. Rotate the chip frame back
  // (x <- y2, y <- -x2, z unchanged), gyro and accelerometer alike. Scales:
  // accel 4096 LSB/g on both sides, gyro 14.286 -> 16.4 LSB/dps.
  const long a1[3] = { (long)r.accel[1], -(long)r.accel[0], (long)r.accel[2] };
  const long g1[3] = { (long)r.gyro[1],  -(long)r.gyro[0],  (long)r.gyro[2] };
  for (int i = 0; i < 3; i++) {
    long a = a1[i];
    if (a > 32767) a = 32767;   // negating -32768 overflows int16
    s.accel[i] = (int16_t)a;
    long g = g1[i] * 11480 / 10000;
    if (g > 32767) g = 32767;
    if (g < -32768) g = -32768;
    s.gyro[i] = (int16_t)g;
  }
}

// ---- Helpers that hide the differences between the USB identities ----

void dongleUsbTask() {
  if (gDS4) gDS4->task();
  else if (gSInput) gSInput->task();
  else if (gProconUsb) { gProconUsb->task(); proconTraceTask(); }
  // usbmode=switch needs nothing here: processHidAndMacroTask runs Gamepad.loop()
}

// Persists the procon-identity USB trace to /ptrace.log so a failure against a real
// console can be read back over serial afterwards (the PTRACE command; during the
// failing session the USB host is the console itself, so the trace has to survive
// until the next PC connection). Entries are staged in RAM and only written to flash
// once the bus has been quiet for a while — a flash write stalls the USB interrupt,
// which must never delay a handshake reply mid-conversation.
void proconTraceTask() {
  static String staging;
  static bool inited = false;
  static uint32_t lastEntryAt = 0;
  static uint32_t written = 0;

  if (!inited) {
    inited = true;
    staging.reserve(2600);
    if (fsMounted) {
      // Keep the previous session too: a plug into a PC that misses the serial
      // window starts a fresh procon session, and that must not destroy the
      // console trace this feature exists to capture
      LittleFS.remove("/ptrace.old");
      LittleFS.rename("/ptrace.log", "/ptrace.old");
    }
    // Boot marker: even a session where USB never enumerates leaves evidence
    // that a procon boot happened
    char boot[24];
    snprintf(boot, sizeof(boot), "%lu BOOT\n", (unsigned long)millis());
    staging += boot;
    lastEntryAt = millis();
  }

  ProconUsb::TraceEntry e;
  while (gProconUsb->takeTrace(e)) {
    if (written + staging.length() > 24576) return;   // cap: keep the FS safe
    // USB = mounted/suspended transition, HB = 1s heartbeat with the 0x30 send count
    static const char *DIR[6] = { "RX ", "T81", "T21", "GRQ", "USB", "HB " };
    char head[20];
    snprintf(head, sizeof(head), "%lu %s ", (unsigned long)e.ms,
             (e.dir < 6) ? DIR[e.dir] : "?? ");
    staging += head;
    for (uint8_t i = 0; i < e.len; i++) {
      char b[3];
      snprintf(b, sizeof(b), "%02X", e.data[i]);
      staging += b;
    }
    staging += '\n';
    lastEntryAt = millis();
  }

  if (!staging.length() || !fsMounted) return;
  // Write only in a quiet moment: a flash write stalls the loop long enough to
  // starve the 0x30 stream (measured: heartbeat cadence doubled), and the trace
  // showed the console keeps VBUS up even after its error, so there is no need
  // to race an imminent power cut
  bool quiet = (int32_t)(millis() - lastEntryAt) >= 300;
  if (!quiet && staging.length() < 4096) return;
  File f = LittleFS.open("/ptrace.log", "a");
  if (f) {
    f.print(staging);
    f.close();
    written += staging.length();
  }
  staging = "";
}

void dongleSendNeutral() {
  // Discard any in-flight rumble request too (carrying it into the next connection would buzz on connect)
  gDongleRumbleL = 0;
  gDongleRumbleR = 0;
  gDongleRumblePending = false;
  gDongleLastRumbleAt = 0;

  if (gDS4) {
    gDS4->setNeutral();
  } else if (gSInput) {
    SInputState neutral = {};
    gSInput->sendInput(neutral);
  } else if (gProconUsb) {
    gProconUsb->setNeutral();
  } else if (G_usb_hid.isValid()) {
    // usbmode=switch
    Gamepad.releaseAll();
    dpadCenter();
    centerSticks();
    sendReport();
  }
}

void dongleApplyReport(const Procon2Report &r) {
  if (gDS4) {
    DS4State s;
    buildDS4FromProcon2(r, s);
    gDS4->setState(s);   // DS4Usb::task() does the sending, every 4ms
  } else if (gSInput) {
    SInputState s = {};
    buildSInputFromProcon2(r, s);
    gSInput->sendInput(s);
  } else if (gProconUsb) {
    ProconState s;
    buildProconFromProcon2(r, s);
    gProconUsb->setState(s);   // ProconUsb::task() does the sending, every 8ms
  } else if (G_usb_hid.isValid()) {
    dongleApplySwitchReport(r);
  }
}

// Linear motor amplitude 0-255 (what DS4/SInput hosts send) -> the perceptual
// level 0-255 setRumble takes (the Switch amplitude-index scale). Inverse of the
// HD-rumble amplitude curve per dekuNukem's encode formulas; called only on
// rumble changes, so the float math is harmless.
uint8_t rumbleLinearToPerceptual(uint8_t linear) {
  if (linear == 0) return 0;
  float a = linear / 255.0f;
  float idx;
  if (a >= 0.23f)      idx = 32.0f * log2f(8.7f * a);
  else if (a >= 0.12f) idx = 16.0f * log2f(17.0f * a);
  else                 idx = 8.0f * log2f(34.0f * a);
  if (idx < 1.0f) idx = 1.0f;
  if (idx > 100.0f) idx = 100.0f;
  return (uint8_t)(idx * 2.55f + 0.5f);
}

bool dongleTakeRumble(uint8_t &l, uint8_t &r) {
  // DS4/SInput hand over linear motor values; the procon identity already
  // hands over the perceptual level
  if (gDS4 || gSInput) {
    bool got = gDS4 ? gDS4->takeRumble(l, r) : gSInput->takeRumble(l, r);
    if (got) {
      l = rumbleLinearToPerceptual(l);
      r = rumbleLinearToPerceptual(r);
    }
    return got;
  }
  if (gProconUsb) return gProconUsb->takeRumble(l, r);
  return false;
}

bool dongleRumbleActive() {
  if (gDS4) return gDS4->rumbleActive();
  if (gSInput) return gSInput->rumbleActive();
  if (gProconUsb) return gProconUsb->rumbleActive();
  return false;
}

// ==============================
// Dongle macro recorder (mode=dongle + macro=on)
//   Records the controller's raw input at a fixed 32Hz tick and replays it into the
//   SInput/DS4 conversion. The C button becomes a control button and is never
//   forwarded to the host (on DS4 the touchpad click is still reachable via CAPTURE).
//
//   C+slot button (A/B/X/Y/L/R/ZL/ZR)  -> loop-play that slot
//   C+GL or C+GR held for 1s           -> arm; releasing everything starts recording
//   while recording: C, or 90 seconds  -> stop (host output goes neutral)
//   then: slot button = save / C = discard; resumes relaying once all buttons are up
//   while playing: C stops (live input is otherwise not forwarded)
// ==============================
struct DongleMacroSample {
  uint32_t buttons;          // P2_BTN_* bitmask (raw)
  uint16_t lx, ly, rx, ry;   // raw 12-bit stick values
};
static_assert(sizeof(DongleMacroSample) == 12, "sample layout is the file format");

const uint16_t DM_MAX_SAMPLES = 2880;       // 90 seconds at the 32Hz tick
const uint32_t DM_TICK_US     = 31250;      // one tick, for both sampling and playback
const uint32_t DM_ARM_HOLD_MS = 1000;       // how long C+GL/GR must be held to arm
const uint32_t DM_START_DELAY_MS = 500;     // pause between releasing everything and recording
const uint8_t  DM_SLOT_COUNT  = 8;
const uint32_t DM_FILE_MAGIC  = 0x314D444B; // "KDM1" little-endian

// Recording/playback buffer: reuses macroLines (45KB). Safe because the string-macro
// engine cannot run in dongle mode (Gamepad is never begun) and the active transport
// only changes through a reboot, so the two users can never overlap.
DongleMacroSample * const gDmSamples = reinterpret_cast<DongleMacroSample *>(macroLines);
static_assert(DM_MAX_SAMPLES * sizeof(DongleMacroSample) <= sizeof(macroLines),
              "sample buffer must fit inside macroLines");

// GL/GR stay ordinary passthrough buttons and are NOT slots: C+GL/GR is the record
// trigger, so giving them slots would make the two gestures ambiguous
struct DmSlotEntry {
  uint32_t    mask;
  const char *name;
};
const DmSlotEntry DM_SLOTS[DM_SLOT_COUNT] = {
  { P2_BTN_A, "A" }, { P2_BTN_B, "B" }, { P2_BTN_X, "X" }, { P2_BTN_Y, "Y" },
  { P2_BTN_L, "L" }, { P2_BTN_R, "R" }, { P2_BTN_ZL, "ZL" }, { P2_BTN_ZR, "ZR" },
};
const uint32_t DM_SLOT_MASK_ALL = P2_BTN_A | P2_BTN_B | P2_BTN_X | P2_BTN_Y |
                                  P2_BTN_L | P2_BTN_R | P2_BTN_ZL | P2_BTN_ZR;

enum DmState : uint8_t {
  DM_RELAY,          // normal passthrough, watching for C combos
  DM_ARMED,          // long-press satisfied; recording starts once everything is released
  DM_RECORDING,      // sampling at 32Hz, passthrough continues
  DM_SLOT_WAIT,      // recording ended; host gets neutral until save (slot) / discard (C)
  DM_RELEASE_WAIT,   // neutral until all buttons are up (so the choice never leaks to the host)
  DM_PLAYING,        // replaying a slot in a loop; live input is ignored except C (stop)
};
DmState  gDmState       = DM_RELAY;
uint32_t gDmPrevButtons = 0;
uint32_t gDmArmMask     = 0;   // GL or GR while the long-press timer runs (0 = none)
uint32_t gDmArmDeadline = 0;
uint32_t gDmStartAt     = 0;   // when armed recording actually begins (0 = not scheduled)
uint16_t gDmCount       = 0;   // samples recorded so far
uint16_t gDmPlayLen     = 0;
uint16_t gDmPlayIndex   = 0;
uint32_t gDmNextTickUs  = 0;
Procon2Report gDmLastReport;   // latest report (the sampling tick reads from here)
bool     gDmHaveReport  = false;

// Firmware-generated vibration feedback. While a pattern is active the host rumble
// relay is skipped (the host's request stays pending in the USB class until we resume).
uint8_t  gDmVibPulses = 0;
bool     gDmVibOn     = false;
uint16_t gDmVibOnMs   = 0;
uint16_t gDmVibOffMs  = 0;
uint32_t gDmVibNextAt = 0;

bool dmVibBusy() { return gDmVibPulses > 0 || gDmVibOn; }

void dmVibrate(uint8_t pulses, uint16_t onMs, uint16_t offMs) {
  gDmVibPulses = pulses;
  gDmVibOnMs   = onMs;
  gDmVibOffMs  = offMs;
  gDmVibOn     = false;
  gDmVibNextAt = millis();   // fire on the next service call
}

void dmVibService(uint32_t now) {
  if (!dmVibBusy()) return;
  if ((int32_t)(now - gDmVibNextAt) < 0) return;
  // Pulse lengths are >= 80ms, which also satisfies setRumble's 30ms write spacing
  if (!gDmVibOn) {
    gProcon2.setRumble(200, 200);
    gDmVibOn = true;
    gDmVibNextAt = now + gDmVibOnMs;
  } else {
    gProcon2.setRumble(0, 0);
    gDmVibOn = false;
    gDmVibPulses--;
    gDmVibNextAt = now + gDmVibOffMs;
  }
}

const char* dmSlotPath(uint8_t idx) {
  static char path[12];
  snprintf(path, sizeof(path), "/dmacro%u", (unsigned)idx);
  return path;
}

struct DmFileHeader {
  uint32_t magic;
  uint16_t count;
  uint16_t reserved;
};

// Called after recording ends (host already neutral), so the flash-write freeze
// (~400ms for a full 90s take) cannot disturb live input.
bool dmSaveSlot(uint8_t idx) {
  if (!ensureFS()) return false;
  File f = LittleFS.open(dmSlotPath(idx), "w");
  if (!f) return false;

  DmFileHeader h = { DM_FILE_MAGIC, gDmCount, 0 };
  size_t bodyLen = (size_t)gDmCount * sizeof(DongleMacroSample);
  bool ok = f.write((const uint8_t *)&h, sizeof(h)) == sizeof(h) &&
            f.write((const uint8_t *)gDmSamples, bodyLen) == bodyLen;
  if (ok && f.getWriteError()) ok = false;
  f.close();

  if (!ok) LittleFS.remove(dmSlotPath(idx));   // no half-written slots
  return ok;
}

bool dmLoadSlot(uint8_t idx) {
  if (!ensureFS()) return false;
  File f = LittleFS.open(dmSlotPath(idx), "r");
  if (!f) return false;

  DmFileHeader h;
  size_t bodyLen = 0;
  bool ok = f.read((uint8_t *)&h, sizeof(h)) == (int)sizeof(h) &&
            h.magic == DM_FILE_MAGIC &&
            h.count >= 1 && h.count <= DM_MAX_SAMPLES;
  if (ok) {
    bodyLen = (size_t)h.count * sizeof(DongleMacroSample);
    ok = f.size() == sizeof(h) + bodyLen &&
         f.read((uint8_t *)gDmSamples, bodyLen) == (int)bodyLen;
  }
  f.close();

  if (ok) gDmPlayLen = h.count;
  return ok;
}

// Drops back to relaying, discarding any recording or playback in progress.
// Used on controller disconnect / USB unmount / macro=off via RESET.
void dmReset(const char *why) {
  if (gDmState == DM_RECORDING || gDmState == DM_SLOT_WAIT) {
    Serial.printf("[DMACRO] recording discarded (%s)\n", why);
  } else if (gDmState == DM_PLAYING) {
    Serial.printf("[DMACRO] playback stopped (%s)\n", why);
  }
  gDmState       = DM_RELAY;
  gDmPrevButtons = 0;
  gDmArmMask     = 0;
  gDmStartAt     = 0;
  gDmHaveReport  = false;
  gDmVibPulses   = 0;
  gDmVibOn       = false;
}

void dmStartRecording() {
  gDmCount      = 0;
  gDmState      = DM_RECORDING;
  gDmNextTickUs = micros();
  dmVibrate(1, 120, 80);
  Serial.println("[DMACRO] recording started");
}

void dmStopRecording(const char *why) {
  Serial.printf("[DMACRO] recording stopped (%s, %u samples)\n", why, gDmCount);
  dongleSendNeutral();
  if (gDmCount == 0) {
    // Nothing to save (C on the very first tick); skip slot selection
    gDmState = DM_RELEASE_WAIT;
    dmVibrate(1, 400, 80);
    return;
  }
  gDmState = DM_SLOT_WAIT;
  dmVibrate(1, 120, 80);
}

void dmStartPlayback(uint8_t slot) {
  if (!dmLoadSlot(slot)) {
    dmVibrate(3, 80, 80);
    Serial.printf("[DMACRO] slot %s is empty\n", DM_SLOTS[slot].name);
    return;
  }
  gDmPlayIndex  = 0;
  gDmState      = DM_PLAYING;
  gDmNextTickUs = micros();
  dmVibrate(2, 120, 100);
  Serial.printf("[DMACRO] playing slot %s (%u samples, loop)\n",
                DM_SLOTS[slot].name, gDmPlayLen);
}

// Consumes one fresh report. Returns whether it should still be forwarded to the host
// (the C bit is stripped by the caller either way).
bool dmHandleReport(const Procon2Report &r) {
  const uint32_t btns  = r.buttons;
  const uint32_t newly = btns & ~gDmPrevButtons;
  // C-first rule: only buttons pressed while C was ALREADY held count as commands.
  // Otherwise pressing C while holding a gameplay button would fire that button's slot.
  const bool cHeldBefore = (gDmPrevButtons & P2_BTN_C) != 0;
  bool forward = true;

  switch (gDmState) {
    case DM_RELAY:
      // Arm trigger (C+GL / C+GR): order-independent, unlike the slots below. The
      // back paddles are naturally gripped BEFORE C, so requiring C first made the
      // gesture fail on real hardware. The 1s hold starts once both are down, and
      // the candidate dies the moment either is released. Holding C itself for a
      // full second has no other meaning, so this cannot misfire during play.
      {
        uint32_t comboMask = 0;
        if (btns & P2_BTN_C) {
          if (btns & P2_BTN_GL) comboMask = P2_BTN_GL;
          else if (btns & P2_BTN_GR) comboMask = P2_BTN_GR;
        }
        if (comboMask != gDmArmMask) {
          gDmArmMask = comboMask;
          if (comboMask) gDmArmDeadline = millis() + DM_ARM_HOLD_MS;
        }
      }
      // Slot playback stays strictly C-first: only a slot button pressed while C
      // was ALREADY held counts, so pressing C while holding e.g. ZR in-game
      // cannot hijack into playback.
      if (cHeldBefore && (btns & P2_BTN_C)) {
        uint32_t slotNew = newly & DM_SLOT_MASK_ALL;
        if (slotNew) {
          for (uint8_t i = 0; i < DM_SLOT_COUNT; i++) {
            if (slotNew & DM_SLOTS[i].mask) { dmStartPlayback(i); break; }
          }
        }
      }
      // A playback start swallows this report, so the slot button never leaks out
      forward = (gDmState == DM_RELAY);
      break;

    case DM_ARMED:
      // Recording begins 0.5s AFTER everything is released (dmService fires it and
      // vibrates then), so the release itself never bleeds into the take. Pressing
      // anything during the wait resets it: the delay always runs from a full release
      if (btns != 0) gDmStartAt = 0;
      else if (gDmStartAt == 0) gDmStartAt = millis() + DM_START_DELAY_MS;
      break;

    case DM_RECORDING:
      if (newly & P2_BTN_C) {
        dmStopRecording("C");
        forward = false;   // the stopping C press is neither recorded nor forwarded
      }
      break;

    case DM_SLOT_WAIT: {
      forward = false;
      uint32_t slotNew = newly & DM_SLOT_MASK_ALL;
      if (slotNew) {
        for (uint8_t i = 0; i < DM_SLOT_COUNT; i++) {
          if (!(slotNew & DM_SLOTS[i].mask)) continue;
          if (dmSaveSlot(i)) {
            dmVibrate(2, 120, 100);
            gDmState = DM_RELEASE_WAIT;
            Serial.printf("[DMACRO] saved to slot %s (%u samples)\n",
                          DM_SLOTS[i].name, gDmCount);
          } else {
            // Stay here so another slot (or C = discard) can still be chosen
            dmVibrate(3, 80, 80);
            Serial.printf("[DMACRO] save to slot %s FAILED\n", DM_SLOTS[i].name);
          }
          break;
        }
      } else if (newly & P2_BTN_C) {
        dmVibrate(1, 400, 80);
        gDmState = DM_RELEASE_WAIT;
        Serial.println("[DMACRO] recording discarded");
      }
      break;
    }

    case DM_RELEASE_WAIT:
      forward = false;
      if (btns == 0) gDmState = DM_RELAY;
      break;

    case DM_PLAYING:
      forward = false;   // live input is deliberately not mixed into playback
      if (newly & P2_BTN_C) {
        dongleSendNeutral();
        gDmState = DM_RELEASE_WAIT;
        dmVibrate(1, 120, 80);
        Serial.println("[DMACRO] playback stopped");
      }
      break;
  }

  gDmPrevButtons = btns;
  return forward;
}

// Runs every dongle-task pass while connected (with or without a fresh report):
// vibration pattern, the arm timer, and the 32Hz sampling / playback ticks.
void dmService() {
  uint32_t now = millis();
  dmVibService(now);

  // While the host is unmounted nothing we send arrives, and a recording made
  // during that window would be of a dead session; drop back to relaying
  if (gDmState != DM_RELAY && !USBDevice.mounted()) {
    dmReset("USB disconnect");
    return;
  }

  if (gDmState == DM_RELAY && gDmArmMask && (int32_t)(now - gDmArmDeadline) >= 0) {
    gDmState   = DM_ARMED;
    gDmArmMask = 0;
    gDmStartAt = 0;
    dmVibrate(1, 120, 80);
    Serial.println("[DMACRO] armed: release all buttons to start recording");
  }

  // Armed and everything released: the 0.5s settle delay has passed -> record
  if (gDmState == DM_ARMED && gDmStartAt && (int32_t)(now - gDmStartAt) >= 0) {
    dmStartRecording();
  }

  if (gDmState == DM_RECORDING) {
    uint32_t nowUs = micros();
    if ((int32_t)(nowUs - gDmNextTickUs) >= 0) {
      if (gDmHaveReport && gDmCount < DM_MAX_SAMPLES) {
        DongleMacroSample &s = gDmSamples[gDmCount++];
        s.buttons = gDmLastReport.buttons;
        s.lx = gDmLastReport.lx;
        s.ly = gDmLastReport.ly;
        s.rx = gDmLastReport.rx;
        s.ry = gDmLastReport.ry;
      }
      gDmNextTickUs += DM_TICK_US;
      // Re-baseline instead of running a backlog if something stalled us
      if ((int32_t)(micros() - gDmNextTickUs) > (int32_t)DM_TICK_US) {
        gDmNextTickUs = micros() + DM_TICK_US;
      }
      if (gDmCount >= DM_MAX_SAMPLES) dmStopRecording("timeout");
    }
  } else if (gDmState == DM_PLAYING) {
    uint32_t nowUs = micros();
    if ((int32_t)(nowUs - gDmNextTickUs) >= 0) {
      const DongleMacroSample &s = gDmSamples[gDmPlayIndex];
      Procon2Report r = {};   // accel/gyro stay 0 = a controller at rest
      r.buttons = s.buttons;
      r.lx = s.lx;
      r.ly = s.ly;
      r.rx = s.rx;
      r.ry = s.ry;
      dongleApplyReport(r);
      gDmPlayIndex++;
      if (gDmPlayIndex >= gDmPlayLen) gDmPlayIndex = 0;
      gDmNextTickUs += DM_TICK_US;
      if ((int32_t)(micros() - gDmNextTickUs) > (int32_t)DM_TICK_US) {
        gDmNextTickUs = micros() + DM_TICK_US;
      }
    }
  }
}

// The DMACRO serial command (status for verification)
void dmPrintStatus() {
  static const char *STATE_NAMES[] = {
    "relay", "armed", "recording", "slot-wait", "release-wait", "playing"
  };
  Serial.printf("[DMACRO] macro=%s state=%s samples=%u\n",
                gLinkConfig.macroOn ? "on" : "off",
                STATE_NAMES[gDmState], (unsigned)gDmCount);
  bool fsOk = ensureFS();
  for (uint8_t i = 0; i < DM_SLOT_COUNT; i++) {
    bool saved = fsOk && LittleFS.exists(dmSlotPath(i));
    Serial.printf("[DMACRO] slot %-2s: %s\n", DM_SLOTS[i].name, saved ? "saved" : "empty");
  }
}

void processDongleTask() {
  if (gActiveTransport != "dongle") return;

  dongleUsbTask();   // FEATURES responses, the periodic DS4 stream, and so on

  // When the link is alive but notifications stop, connected() stays true and it never
  // recovers on its own. Drop the link and fall back to rescanning.
  if (gProcon2.stalled()) {
    Serial.println("[DONGLE] no report for 3s -> reconnecting");
    gProcon2.markDisconnected();
  }

  if (!gProcon2.connected()) {
    if (gDongleWasConnected) {
      // Just after a disconnect: send neutral so nothing stays held on the host
      gDongleWasConnected = false;
      gProcon2.markDisconnected();
      dongleSendNeutral();
      if (gDmState != DM_RELAY) dmReset("controller disconnect");
    }

    // BLE.scan() blocks for 2 seconds, so leave 1 second between scans for serial
    // configuration and the like to respond
    uint32_t now = millis();
    if ((int32_t)(now - gDongleNextScan) < 0) return;
    gDongleNextScan = now + 1000;

    if (gProcon2.scanAndConnect()) {
      gDongleWasConnected = true;
      dongleStartCalibration();   // re-measure the center on every connection
    }
    return;
  }

  gDongleWasConnected = true;
  uint32_t now = millis();

  // If macro=off arrived via RESET while the recorder was mid-flight, clean up
  if (!gLinkConfig.macroOn && gDmState != DM_RELAY) dmReset("macro disabled");

  // Rumble relay: send on change, and resend every 100ms while buzzing — the HD
  // pattern decays if left alone, and at 250ms a sustained rumble came through as
  // a train of short pulses. Neutral (both 0) is meant to decay, so it is not resent.
  // GATT writes block, so keep them at least 30ms apart.
  // Skipped while a recorder feedback pattern is playing (that owns the actuator;
  // the host's request stays pending in the USB class until we resume), during
  // recording (the recorder's vibration is the only feedback channel, so game rumble
  // would be indistinguishable from it), and during playback (the player is not
  // holding the controller as the game assumes; a buzzing idle controller is just
  // noise).
  if (!dmVibBusy() && gDmState != DM_RECORDING && gDmState != DM_PLAYING) {
    uint8_t l, r;
    if (dongleTakeRumble(l, r)) {
      gDongleRumbleL = l;
      gDongleRumbleR = r;
      gDongleRumblePending = true;
    }
    bool keepalive = dongleRumbleActive() &&
                     (gDongleRumbleL > 0 || gDongleRumbleR > 0) &&
                     (int32_t)(now - gDongleLastRumbleAt) >= 100;
    if ((gDongleRumblePending || keepalive) && (int32_t)(now - gDongleLastRumbleAt) >= 30) {
      gProcon2.setRumble(gDongleRumbleL, gDongleRumbleR);
      gDongleLastRumbleAt = now;
      gDongleRumblePending = false;
    }
  } else if (gDmState == DM_RECORDING || gDmState == DM_PLAYING) {
    // Discard (not defer) game rumble while recording/playing back, so a stale
    // request cannot buzz right after the recording or playback ends
    uint8_t l, r;
    dongleTakeRumble(l, r);
  }

  Procon2Report report;
  bool haveNew = gProcon2.fetchReport(report);
  if (haveNew) {
    if (gDongleCalibrating) dongleCalibrate(report);
    bool forward = true;
    if (gLinkConfig.macroOn) {
      gDmLastReport = report;
      gDmHaveReport = true;
      forward = dmHandleReport(report);
    }
    if (forward) {
      if (gLinkConfig.macroOn) {
        // C is the recorder's control button and never reaches the host
        Procon2Report masked = report;
        masked.buttons &= ~P2_BTN_C;
        // The IMU is relay-only and never recorded, so it is muted during recording:
        // otherwise the take would rely on gyro input that playback cannot reproduce
        if (gDmState == DM_RECORDING) {
          memset(masked.accel, 0, sizeof(masked.accel));
          memset(masked.gyro, 0, sizeof(masked.gyro));
        }
        dongleApplyReport(masked);
      } else {
        dongleApplyReport(report);
      }
    }
  }

  // The recorder's timers (vibration, arm long-press, 32Hz ticks) run with or
  // without a fresh report
  if (gLinkConfig.macroOn) dmService();

  if (!haveNew) return;

  // Drain the response-channel notification ring. Printing happens only under DEBUG and
  // stops at 15 lines per second, but the draining itself always runs.
  {
    static uint32_t rspWindowStart = 0;
    static uint8_t rspPrinted = 0;
    if ((int32_t)(now - rspWindowStart) >= 1000) {
      rspWindowStart = now;
      rspPrinted = 0;
    }
    uint8_t rsp[64];
    size_t rlen;
    uint16_t tag;
    while (gProcon2.takeCmdResponse(rsp, rlen, tag)) {
      if (!Procon2Link::debugLog) continue;   // keep draining, just discard
      if (rspPrinted >= 15) continue;
      rspPrinted++;
      Serial.printf("[P2RSP] from=%04X len=%u ", tag, (unsigned)rlen);
      for (size_t i = 0; i < rlen; i++) Serial.printf("%02X", rsp[i]);
      Serial.println();
    }
  }

  // DUMP command: dump raw reports to serial every 300ms
  if (gDongleDumpRemaining > 0 && (int32_t)(now - gDongleNextDumpAt) >= 0) {
    uint8_t raw[96];
    size_t len = gProcon2.copyRaw(raw);
    if (len > 0) {
      gDongleDumpRemaining--;
      gDongleNextDumpAt = now + 300;
      Serial.printf("[DUMP] len=%u ", (unsigned)len);
      for (size_t i = 0; i < len && i < 96; i++) Serial.printf("%02X", raw[i]);
      Serial.println();
    }
  }
}

// ==============================
// HID & Macro Task
// ==============================
void processHidAndMacroTask() {
  // Dongle mode with usbmode=sinput/ds4/procon never calls Gamepad.begin(), so this
  // must not be touched (TinyUSB would index its internal array with an unallocated
  // instance number). With usbmode=switch the Gamepad IS active: loop() flushes the
  // relayed state, and the macro/tap parts stay inert because serial gamepad
  // commands are rejected in dongle mode.
  if (!G_usb_hid.isValid()) return;

  if (Gamepad.ready()) {
    Gamepad.loop();
  }

  serviceTap();   // release a TAP if its deadline has arrived
  tickMacro();

  if (macroRunning && !USBDevice.mounted()) {
    clearMacro();
    Gamepad.releaseAll();
    dpadCenter();
    centerSticks();
    sendReport();
    Serial.println("[MACRO] stopped due to USB disconnect");
  }
}

// ==============================
// setup / loop
// ==============================
// ==============================
// Two-stage boot for the USB identity
//   Touching LittleFS (flash) before USB initialization has been observed on real hardware
//   to break startup so badly that the serial port never appears, so the USB identity
//   cannot come from the configuration file. Instead we put the identity for the next
//   boot into a watchdog scratch register (which survives a reboot) and reboot. At
//   startup only the register decides; the configuration is read once USB is stable,
//   and a mismatch rewrites the register and reboots. scratch[4..7] is pico-sdk's, so we use [2][3].
// ==============================
#include <hardware/watchdog.h>

const uint32_t USB_IDENT_MAGIC = 0x50494330;   // 'PIC0'
const uint32_t USB_IDENT_SINPUT = 1;
const uint32_t USB_IDENT_DS4 = 2;
const uint32_t USB_IDENT_PROCON = 3;

void rebootWithUsbIdentity(uint32_t ident) {
  watchdog_hw->scratch[2] = ident ? USB_IDENT_MAGIC : 0;
  watchdog_hw->scratch[3] = ident;
  Serial.flush();
  delay(200);
  rp2040.reboot();
}

void setup() {
  // Decide which USB identity to boot with from the scratch register (default = HORI pad).
  // Clear the flag once read (a power cycle or a normal reboot returns to the default)
  uint32_t identity = 0;
  if (watchdog_hw->scratch[2] == USB_IDENT_MAGIC) {
    identity = watchdog_hw->scratch[3];
  }
  watchdog_hw->scratch[2] = 0;
  watchdog_hw->scratch[3] = 0;

  // IMPORTANT: USB initialization (Serial=CDC / HID) must come first, in this exact
  // order, back to back (a safety condition confirmed on real hardware)
  Serial.begin(115200);
  // Set gActiveUsbMode even if allocation failed: empty would make the two-stage boot
  // check below see a mismatch every time and reboot in a loop. (Degraded operation
  // without USB HID; configuration over serial still works)
  if (identity == USB_IDENT_DS4) {
    gActiveUsbMode = "ds4";
    gDS4 = new DS4Usb();
    if (gDS4) gDS4->begin();
    else Serial.println("[BOOT] DS4Usb alloc failed -> USB gamepad disabled");
  } else if (identity == USB_IDENT_PROCON) {
    gActiveUsbMode = "procon";
    // A real Pro Controller is a single-interface HID device, but the console
    // accepts our CDC+HID composite fine (verified on a Switch 2): the 2162-0002
    // failure was the missing pairing-subcommand reply, not the descriptor. So
    // the serial config channel stays available in this identity too.
    gProconUsb = new ProconUsb();
    if (gProconUsb) gProconUsb->begin();
    else Serial.println("[BOOT] ProconUsb alloc failed -> USB gamepad disabled");
  } else if (identity == USB_IDENT_SINPUT) {
    gActiveUsbMode = "sinput";
    gSInput = new SInputUsb();
    if (gSInput) gSInput->begin();
    else Serial.println("[BOOT] SInputUsb alloc failed -> USB gamepad disabled");
  } else {
    // Identity 0 = the HORI-compatible Switch pad. Used by bt/wifi, and by dongle
    // mode with usbmode=switch (the Pro Controller 2 -> Switch/Switch 2 relay)
    gActiveUsbMode = "switch";
    Gamepad.begin();
  }
  delay(1000);        // wait for HID initialization
  Serial.println("Booting Pico W Switch Pad...");

  // Pre-reserve the buffers that tend to grow (sizes are rough)
  serialLine.reserve(256);
  gWifiLineBuffer.reserve(256);

  // Buffers reused during command handling. Reserving them up front keeps malloc/free
  // out of the run loop so the heap does not fragment during a long macro.
  gCmdLine.reserve(64);
  gCmdUpper.reserve(64);
  gMacroCmd.reserve(MACRO_LINE_MAXLEN + 1);
  gBleLineBuffer.reserve(256);

  // Read the configuration and start only the selected transport (after USB is stable).
  // BT can start on its default name even with no configuration file.
  loadLinkConfig();

  // If the USB identity the configuration asks for differs from the one now running,
  // put the requested identity into the scratch register and reboot (the two-stage boot).
  {
    uint32_t wanted = 0;
    if (gLinkConfig.mode == "dongle") {
      wanted = (gLinkConfig.usbMode == "ds4")    ? USB_IDENT_DS4
             : (gLinkConfig.usbMode == "procon") ? USB_IDENT_PROCON
             : (gLinkConfig.usbMode == "switch") ? 0   // identity 0 = the Switch pad
             : USB_IDENT_SINPUT;
    }
    uint32_t booted = (gActiveUsbMode == "ds4") ? USB_IDENT_DS4
                    : (gActiveUsbMode == "procon") ? USB_IDENT_PROCON
                    : (gActiveUsbMode == "sinput") ? USB_IDENT_SINPUT : 0;
    if (wanted != booted) {
      Serial.println("[BOOT] USB identity mismatch -> rebooting to switch...");
      rebootWithUsbIdentity(wanted);
      return;   // unreachable
    }
  }

  if (gLinkConfig.mode == "wifi") {
    gActiveTransport = "wifi";
    if (gLinkConfig.valid) {
      if (!startWifiFromConfig()) {
        Serial.println("[WLAN] could not start (check config) -> offline mode");
      }
    } else {
      Serial.println("[WLAN] Wi-Fi config invalid -> offline mode");
    }
  } else if (gLinkConfig.mode == "dongle") {
    gActiveTransport = "dongle";
    startDongleFromConfig();
  } else {
    gActiveTransport = "bt";
    startBleFromConfig();
  }
}

void loop() {
  processSerialInput();
  processWifiSupervisorTask();
  processWifiClientTask();
  processBleTask();
  processDongleTask();
  processHidAndMacroTask();

  yield();
  delay(0);
}

