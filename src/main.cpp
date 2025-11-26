#include <WiFi.h>
#include <WebServer.h>
#include <WiFiUdp.h>
#include <Preferences.h>

#define GPS_RX 20
#define GPS_TX 21
#define NMEA_BUF 256
#define UBX_MAX_PAYLOAD 256

const char* AP_SSID = "GPS_ESP32";
const char* AP_PASS = "12345678";

HardwareSerial GPSSerial(1);
WebServer server(80);
WiFiUDP udp;
Preferences prefs;

String cfg_sta_ssid = "";
String cfg_sta_pass = "";
String cfg_ip = "255.255.255.255";
uint16_t cfg_port = 10110;
bool cfg_broadcast = true;
uint8_t cfg_dynModel = 3;
uint16_t cfg_measRate = 1000;

IPAddress targetIP;
bool want_sta_connect = false;
unsigned long lastStaAttempt = 0;

char nmeaBuf[NMEA_BUF];
size_t nmeaLen = 0;
bool inSentence = false;

unsigned long lastUdpSend = 0;
const unsigned long UDP_INTERVAL = 1000;

// ================== UBX-apurit =========================
void ubxChecksum(const uint8_t* payload, uint16_t len, uint8_t &ckA, uint8_t &ckB) {
  ckA = 0; ckB = 0;
  for (uint16_t i=0;i<len;i++) { ckA = ckA + payload[i]; ckB = ckB + ckA; }
}

void sendUBX(uint8_t cls, uint8_t id, const uint8_t* data, uint16_t len) {
  if (len > UBX_MAX_PAYLOAD) return;
  
  uint8_t header[4] = { cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
  uint8_t ckA, ckB;
  
  uint8_t calcBuf[4 + UBX_MAX_PAYLOAD];
  memcpy(calcBuf, header, 4);
  if (len) memcpy(calcBuf+4, data, len);
  ubxChecksum(calcBuf, 4+len, ckA, ckB);

  GPSSerial.write(0xB5);
  GPSSerial.write(0x62);
  GPSSerial.write(header, 4);
  if (len) GPSSerial.write(data, len);
  GPSSerial.write(ckA);
  GPSSerial.write(ckB);
  GPSSerial.flush();
}

bool waitForAck(uint8_t cls, uint8_t id, uint32_t timeoutMs = 500) {
  uint32_t start = millis();
  enum { FIND_SYNC1, FIND_SYNC2, CLASS, MSGID, LEN1, LEN2, PAYLOAD, CK_A, CK_B } st = FIND_SYNC1;
  uint8_t ubxClass=0, ubxId=0;
  uint16_t len=0, rx=0;
  uint8_t pay[8];
  uint8_t ckA=0, ckB=0;
  while (millis() - start < timeoutMs) {
    if (!GPSSerial.available()) { delay(1); continue; }
    uint8_t b = (uint8_t)GPSSerial.read();
    switch (st) {
      case FIND_SYNC1: if (b==0xB5) st=FIND_SYNC2; break;
      case FIND_SYNC2: st=(b==0x62)?CLASS:FIND_SYNC1; break;
      case CLASS: ubxClass=b; st=MSGID; break;
      case MSGID: ubxId=b; st=LEN1; break;
      case LEN1: len=b; st=LEN2; break;
      case LEN2: len |= (uint16_t)b<<8; rx=0; st=(len?PAYLOAD:CK_A); break;
      case PAYLOAD:
        if (rx < sizeof(pay)) pay[rx] = b;
        rx++;
        if (rx>=len) st=CK_A;
        break;
      case CK_A: ckA=b; st=CK_B; break;
      case CK_B:
        ckB=b;
        if (ubxClass==0x05 && (ubxId==0x01 || ubxId==0x00) && len>=2) {
          if (pay[0]==cls && pay[1]==id) return (ubxId==0x01);
        }
        st=FIND_SYNC1; break;
    }
  }
  return false;
}

bool ubx_cfg_nav5_dynModel(uint8_t dynModel) {
  uint8_t p[36]; memset(p, 0, sizeof(p));
  p[0] = 0x01;
  p[2] = dynModel;
  sendUBX(0x06, 0x24, p, sizeof(p));
  return waitForAck(0x06, 0x24);
}

bool ubx_cfg_rate(uint16_t measRateMs, uint16_t navRate=1, uint16_t timeRef=1) {
  uint8_t p[6];
  p[0] = (uint8_t)(measRateMs & 0xFF);
  p[1] = (uint8_t)(measRateMs >> 8);
  p[2] = (uint8_t)(navRate & 0xFF);
  p[3] = (uint8_t)(navRate >> 8);
  p[4] = (uint8_t)(timeRef & 0xFF);
  p[5] = (uint8_t)(timeRef >> 8);
  sendUBX(0x06, 0x08, p, sizeof(p));
  return waitForAck(0x06, 0x08);
}

// ================== Wifi / Web ==========================
String htmlHeader() {
  return String(F(
    "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>GPS NMEA → UDP</title>"
    "<style>body{font-family:system-ui,Arial,sans-serif;max-width:760px;margin:16px auto;padding:0 10px}"
    "label{display:block;margin:8px 0}input,select{width:100%;padding:6px}"
    "small{color:#444}</style>"
    "</head><body><h2>ESP32-C3: u-blox NMEA → UDP</h2>"
  ));
}

String htmlFooter() { return String(F("</body></html>")); }

String wifiStatusText() {
  wl_status_t st = WiFi.status();
  switch (st) {
    case WL_CONNECTED:       return "CONNECTED";
    case WL_IDLE_STATUS:     return "IDLE";
    case WL_DISCONNECTED:    return "DISCONNECTED";
    case WL_CONNECT_FAILED:  return "CONNECT_FAILED";
    case WL_NO_SSID_AVAIL:   return "NO_SSID_AVAIL";
    default:                 return String((int)st);
  }
}

void handleRoot() {
  String page = htmlHeader();
  page += "<p>AP SSID: <b>" + String(AP_SSID) + "</b> (IP " + WiFi.softAPIP().toString() + ")</p>";
  page += "<p>STA: " + wifiStatusText() + " — SSID <b>" + (cfg_sta_ssid.length()?cfg_sta_ssid:"(none)") + "</b>";
  if (WiFi.status()==WL_CONNECTED) page += " — IP <b>" + WiFi.localIP().toString() + "</b>";
  page += "</p><hr/>";

  page += "<form method='POST' action='/save' style='display:grid;gap:10px'>";
  page += "<h3>Network & UDP</h3>";
  page += "<label>STA SSID:<br><input name='sta_ssid' value='" + cfg_sta_ssid + "'></label>";
  page += "<label>STA Password:<br><input name='sta_pass' type='password' value='" + cfg_sta_pass + "'></label>";
  page += "<label>Target IP (255.255.255.255 = broadcast):<br><input name='ip' value='" + cfg_ip + "'></label>";
  page += "<label>Target Port:<br><input name='port' type='number' value='" + String(cfg_port) + "'></label>";
  page += String("<label><input type='checkbox' name='broadcast' ") + (cfg_broadcast?"checked":"") + "> Use broadcast</label>";

  page += "<h3>u-blox receiver</h3>";
  page += "<label>Dynamic model:<br><select name='dyn'>"
          "<option value='0' " + String(cfg_dynModel==0?"selected":"") + ">Portable</option>"
          "<option value='1' " + String(cfg_dynModel==1?"selected":"") + ">Stationary</option>"
          "<option value='2' " + String(cfg_dynModel==2?"selected":"") + ">Pedestrian</option>"
          "<option value='3' " + String(cfg_dynModel==3?"selected":"") + ">Automotive</option>"
          "<option value='5' " + String(cfg_dynModel==5?"selected":"") + ">Sea (boats)</option>"
          "</select></label>";
  page += "<label>Navigation update rate:<br><select name='rate'>"
          "<option value='1000' " + String(cfg_measRate==1000?"selected":"") + ">1 Hz</option>"
          "<option value='500' "  + String(cfg_measRate==500 ?"selected":"") + ">2 Hz</option>"
          "<option value='200' "  + String(cfg_measRate==200 ?"selected":"") + ">5 Hz</option>"
          "<option value='100' "  + String(cfg_measRate==100 ?"selected":"") + ">10 Hz</option>"
          "</select></label>";

  page += "<button type='submit'>Save & Apply</button>";
  page += "</form>";

  page += "<p style='margin-top:10px'><a href='/scan'>Scan Wi-Fi</a> &nbsp;|&nbsp; "
          "<a href='/forget' onclick='return confirm(\"Forget STA credentials?\")'>Forget STA</a></p>";

  page += "<hr><p><small>NMEA virta välitetään GPS:ltä UDP:hen <b>kerran sekunnissa</b>. "
          "UBX-asetukset vaikuttavat vain u-bloxiin.</small></p>";
  page += htmlFooter();
  server.send(200, "text/html", page);
}

void handleSave() {
  if (server.hasArg("sta_ssid")) cfg_sta_ssid = server.arg("sta_ssid");
  if (server.hasArg("sta_pass")) cfg_sta_pass = server.arg("sta_pass");
  if (server.hasArg("ip"))       cfg_ip       = server.arg("ip");
  if (server.hasArg("port"))     cfg_port     = (uint16_t)server.arg("port").toInt();
  cfg_broadcast = server.hasArg("broadcast");

  if (server.hasArg("dyn"))         cfg_dynModel       = (uint8_t)server.arg("dyn").toInt();
  if (server.hasArg("rate"))        cfg_measRate       = (uint16_t)server.arg("rate").toInt();

  prefs.putString("sta_ssid", cfg_sta_ssid);
  prefs.putString("sta_pass", cfg_sta_pass);
  prefs.putString("ip",       cfg_ip);
  prefs.putUShort("port",     cfg_port);
  prefs.putBool("br",         cfg_broadcast);
  prefs.putUChar("dyn",       cfg_dynModel);
  prefs.putUShort("rate",     cfg_measRate);

  targetIP.fromString(cfg_ip);
  want_sta_connect = (cfg_sta_ssid.length() > 0);

  bool ok1 = ubx_cfg_nav5_dynModel(cfg_dynModel);
  bool ok2 = ubx_cfg_rate(cfg_measRate);

  String page = htmlHeader();
  page += "<p><b>Saved.</b> STA connect requested to <b>" + cfg_sta_ssid + "</b></p>";
  page += "<p>UBX apply: NAV5(dynModel) = " + String(ok1?"OK":"(no ACK)") +
          ", RATE = " + String(ok2?"OK":"(no ACK)") + "</p>";
  page += "<p><a href='/'>Back</a></p>";
  page += htmlFooter();
  server.send(200, "text/html", page);
}

void handleForget() {
  cfg_sta_ssid = "";
  cfg_sta_pass = "";
  prefs.putString("sta_ssid", "");
  prefs.putString("sta_pass", "");
  WiFi.disconnect(true, true);
  String page = htmlHeader();
  page += "<p><b>STA credentials cleared.</b></p><p><a href='/'>Back</a></p>";
  page += htmlFooter();
  server.send(200, "text/html", page);
}

void handleScan() {
  int n = WiFi.scanNetworks();
  String page = htmlHeader();
  page += "<h3>Scan results</h3><ul>";
  for (int i=0;i<n;i++) {
    page += "<li>" + WiFi.SSID(i) + " (" + String(WiFi.RSSI(i)) + " dBm)" + (WiFi.encryptionType(i)? " 🔒": "") + "</li>";
  }
  page += "</ul><p><a href='/'>Back</a></p>";
  page += htmlFooter();
  server.send(200, "text/html", page);
}

// ================== WiFi helperit ======================
void beginAP() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  delay(100);
  WiFi.mode(WIFI_AP_STA);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS, 6, 0, 4);
  Serial.printf("softAP(): %s, AP IP=%s\n", ok?"OK":"FAIL", WiFi.softAPIP().toString().c_str());
}

void tryConnectSTA() {
  if (!want_sta_connect) return;
  if (cfg_sta_ssid.length()==0) return;
  if (WiFi.status()==WL_CONNECTED) return;
  if (millis() - lastStaAttempt < 10000) return;
  lastStaAttempt = millis();
  Serial.printf("Connecting STA to SSID='%s'...\n", cfg_sta_ssid.c_str());
  WiFi.begin(cfg_sta_ssid.c_str(), cfg_sta_pass.c_str());
}

// ================== UDP lähetys - 1s väli ==============
void sendUDPLine(const char* line, size_t len) {
  unsigned long now = millis();

  if (now - lastUdpSend < UDP_INTERVAL) {
    return;
  }

  IPAddress destIP;
  if (cfg_broadcast) {
    destIP = IPAddress(255, 255, 255, 255);
  } else {
    if (targetIP == IPAddress(0, 0, 0, 0)) {
      targetIP.fromString(cfg_ip);
    }
    destIP = targetIP;
  }

  udp.beginPacket(destIP, cfg_port);
  udp.write((const uint8_t*)line, len);
  udp.write((const uint8_t*)"\r\n", 2);
  udp.endPacket();

  lastUdpSend = now;
}

// ================== NMEA käsittely =====================
void handleNMEALine(char* line, size_t len) {
  if (len == 0 || len >= NMEA_BUF) return;
  
  String s(line);
  Serial.println(s);
  sendUDPLine(line, len);
}

// ================== SETUP / LOOP =======================
void setup() {
  Serial.begin(115200);
  GPSSerial.begin(115200, SERIAL_8N1, GPS_RX, GPS_TX);

  prefs.begin("gpscfg", false);
  cfg_sta_ssid = prefs.getString("sta_ssid", "");
  cfg_sta_pass = prefs.getString("sta_pass", "");
  cfg_ip       = prefs.getString("ip", cfg_ip);
  cfg_port     = prefs.getUShort("port", cfg_port);
  cfg_broadcast= prefs.getBool("br", cfg_broadcast);
  cfg_dynModel = prefs.getUChar("dyn", cfg_dynModel);
  cfg_measRate = prefs.getUShort("rate", cfg_measRate);

  targetIP.fromString(cfg_ip);

  beginAP();
  want_sta_connect = (cfg_sta_ssid.length() > 0);
  if (want_sta_connect) WiFi.begin(cfg_sta_ssid.c_str(), cfg_sta_pass.c_str());

  udp.begin(cfg_port);

  server.on("/",       HTTP_GET,  handleRoot);
  server.on("/save",   HTTP_POST, handleSave);
  server.on("/scan",   HTTP_GET,  handleScan);
  server.on("/forget", HTTP_GET,  handleForget);
  server.begin();

  Serial.println("=== ESP32-C3 u-blox NMEA → UDP (AP+STA) - 1s interval ===");
  Serial.printf("AP SSID: %s  AP IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  Serial.printf("Target : %s:%u %s\n", cfg_ip.c_str(), cfg_port, cfg_broadcast?"(broadcast)":"(unicast)");

  bool ok1 = ubx_cfg_nav5_dynModel(cfg_dynModel);
  bool ok2 = ubx_cfg_rate(cfg_measRate);
  Serial.printf("Apply on boot: NAV5(dyn)=%s RATE=%s\n", ok1?"OK":"-", ok2?"OK":"-");
}

void loop() {
  server.handleClient();
  tryConnectSTA();

  while (GPSSerial.available()) {
    uint8_t b = (uint8_t)GPSSerial.read();

    if (b == '$') {
      inSentence = true;
      nmeaLen = 0;
      nmeaBuf[nmeaLen++] = '$';
      continue;
    }

    if (!inSentence) continue;

    if (b == '\n' || b == '\r') {
      if (nmeaLen > 1) {
        nmeaBuf[nmeaLen] = 0;
        handleNMEALine(nmeaBuf, nmeaLen);
      }
      inSentence = false;
      nmeaLen = 0;
      continue;
    }

    if (nmeaLen < NMEA_BUF-1) {
      nmeaBuf[nmeaLen++] = (char)b;
    } else {
      inSentence = false;
      nmeaLen = 0;
    }
  }
}
