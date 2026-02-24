/*
 * WEM3080 Energy Monitor - M5StickC Plus + RS485 HAT
 *
 * Features:
 * - Read IAMMETER WEM3080 single-phase energy meter via Modbus RTU
 * - Raw Modbus RTU with CRC16 (no external library needed)
 * - Live TFT display with color-coded metrics (landscape)
 * - Web dashboard with auto-refresh (dark theme)
 * - JSON API endpoint for automation
 * - API key protection via query parameter
 * - Button press toggles display page
 *
 * Wiring (M5StickC Plus RS485 HAT):
 *   RS485 HAT TX -> G26
 *   RS485 HAT RX -> G0
 *   RS485 A+/B-  -> WEM3080 A+/B-
 *
 * Modbus: 9600/8N1, Slave ID 1, FC03
 *
 * Date: 24 Feb 2026
 */

#include <M5StickCPlus.h>
#include <WiFi.h>
#include <WebServer.h>

// ============== WIFI CONFIG ==============
#define WIFI_SSID     "SIRIUS"
#define WIFI_PASS     "jakkrit5052"

// ============== API KEY ==============
#define API_KEY       "5C0E8BF2"

// ============== RS485 PINS (M5StickC Plus RS485 HAT) ==============
#define RS485_TX_PIN  0   // G0  (per M5 RS485 HAT official example)
#define RS485_RX_PIN  26  // G26 (per M5 RS485 HAT official example)
#define RS485_BAUD    9600

// ============== MODBUS CONFIG ==============
#define MODBUS_SLAVE_ID   1
#define MODBUS_REG_START  0x0048  // Voltage register start
#define MODBUS_REG_COUNT  9       // 0x0048-0x0050 (V,I,P,FwdE_H,FwdE_L,PF,RevE_H,RevE_L,Dir)
#define MODBUS_REG_FREQ   0x0065  // Frequency register
#define MODBUS_TIMEOUT    1000    // ms

// ============== TIMING ==============
#define READ_INTERVAL     3000    // Read every 3 seconds
#define DISPLAY_INTERVAL  500     // Display refresh

// ============== CUSTOM COLORS ==============
#define COLOR_SKYBLUE 0x867D  // RGB565 for sky blue (~135,206,235)

// ============== OBJECTS ==============
HardwareSerial RS485(2);  // Serial2
WebServer server(80);

// ============== METER DATA ==============
float mVoltage     = 0;
float mCurrent     = 0;
float mPower       = 0;    // Signed (negative = reverse)
float mFwdEnergy   = 0;    // kWh
float mRevEnergy   = 0;    // kWh
float mPowerFactor = 0;
float mFrequency   = 0;
bool  mPowerReverse = false;
bool  mValid       = false;
int   mErrorCount  = 0;
int   mReadCount   = 0;

// ============== TIMERS ==============
unsigned long lastRead    = 0;
unsigned long lastDisplay = 0;

// ============== DISPLAY STATE ==============
uint8_t displayPage = 0;  // 0=summary, 1=detail
bool wifiConnected  = false;

// ============== MODBUS CRC16 ==============
uint16_t modbusCalcCRC16(uint8_t *buf, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// ============== MODBUS SEND FC03 REQUEST ==============
void modbusSendRequest(uint8_t slaveId, uint16_t regAddr, uint16_t regCount) {
  uint8_t frame[8];
  frame[0] = slaveId;
  frame[1] = 0x03;              // Function code: Read Holding Registers
  frame[2] = regAddr >> 8;      // Register address high
  frame[3] = regAddr & 0xFF;    // Register address low
  frame[4] = regCount >> 8;     // Number of registers high
  frame[5] = regCount & 0xFF;   // Number of registers low
  uint16_t crc = modbusCalcCRC16(frame, 6);
  frame[6] = crc & 0xFF;        // CRC low
  frame[7] = crc >> 8;          // CRC high

  // Flush RX buffer and allow bus turnaround time
  while (RS485.available()) RS485.read();
  delay(10);

  RS485.write(frame, 8);
  RS485.flush();  // Wait for TX complete
  delay(5);       // Bus turnaround delay
}

// ============== MODBUS READ RESPONSE ==============
// Returns number of data registers read, 0 on error
// regs[] receives the register values
int modbusReadResponse(uint16_t *regs, uint16_t maxRegs) {
  uint8_t buf[128];
  uint16_t idx = 0;
  unsigned long startMs = millis();

  // Wait for response header (slave + fc + byte_count = 3 bytes minimum)
  while (idx < 3) {
    if (RS485.available()) {
      buf[idx++] = RS485.read();
    }
    if (millis() - startMs > MODBUS_TIMEOUT) {
      Serial.println(F("Modbus: timeout waiting for header"));
      return 0;
    }
  }

  // Check for error response
  if (buf[1] & 0x80) {
    Serial.printf("Modbus: error response FC=0x%02X code=0x%02X\n", buf[1], buf[2]);
    return 0;
  }

  uint8_t byteCount = buf[2];
  uint16_t totalLen = 3 + byteCount + 2;  // header + data + CRC

  if (totalLen > sizeof(buf)) {
    Serial.println(F("Modbus: response too large"));
    return 0;
  }

  // Read remaining bytes
  while (idx < totalLen) {
    if (RS485.available()) {
      buf[idx++] = RS485.read();
    }
    if (millis() - startMs > MODBUS_TIMEOUT) {
      Serial.printf("Modbus: timeout at byte %d/%d\n", idx, totalLen);
      return 0;
    }
  }

  // Verify CRC
  uint16_t rxCrc = buf[totalLen - 2] | (buf[totalLen - 1] << 8);
  uint16_t calcCrc = modbusCalcCRC16(buf, totalLen - 2);
  if (rxCrc != calcCrc) {
    Serial.printf("Modbus: CRC mismatch rx=0x%04X calc=0x%04X\n", rxCrc, calcCrc);
    return 0;
  }

  // Extract register values
  uint16_t numRegs = byteCount / 2;
  if (numRegs > maxRegs) numRegs = maxRegs;

  for (uint16_t i = 0; i < numRegs; i++) {
    regs[i] = (buf[3 + i * 2] << 8) | buf[3 + i * 2 + 1];
  }

  return numRegs;
}

// ============== READ WEM3080 ==============
void readWEM3080() {
  uint16_t regs[16];
  mReadCount++;

  // --- Read main registers 0x0048-0x0050 (9 registers) ---
  modbusSendRequest(MODBUS_SLAVE_ID, MODBUS_REG_START, MODBUS_REG_COUNT);
  int n = modbusReadResponse(regs, 16);

  if (n < MODBUS_REG_COUNT) {
    Serial.printf("Modbus: main read got %d regs, expected %d\n", n, MODBUS_REG_COUNT);
    mValid = false;
    mErrorCount++;
    return;
  }

  // Parse main registers
  // 0x0048 -> regs[0]: Voltage (/100)
  mVoltage = regs[0] / 100.0f;

  // 0x0049 -> regs[1]: Current (/100)
  mCurrent = regs[1] / 100.0f;

  // 0x004A -> regs[2]: Power (unsigned, direct W)
  uint16_t rawPower = regs[2];

  // 0x004B-4C -> regs[3],regs[4]: Forward Energy (32-bit /800 kWh)
  uint32_t fwdRaw = ((uint32_t)regs[3] << 16) | regs[4];
  mFwdEnergy = fwdRaw / 800.0f;

  // 0x004D -> regs[5]: Power Factor (/1000)
  mPowerFactor = regs[5] / 1000.0f;

  // 0x004E-4F -> regs[6],regs[7]: Reverse Energy (32-bit /800 kWh)
  uint32_t revRaw = ((uint32_t)regs[6] << 16) | regs[7];
  mRevEnergy = revRaw / 800.0f;

  // 0x0050 -> regs[8]: Power Direction (high byte: 0=fwd, 1=rev)
  mPowerReverse = (regs[8] >> 8) != 0;
  mPower = mPowerReverse ? -(float)rawPower : (float)rawPower;

  // Frequency: reg 0x0065 not supported on this WEM3080 firmware (returns 0x82).
  // Default to 50Hz for 220-240V regions.
  if (mFrequency == 0) mFrequency = 50.0f;

  mValid = true;

  Serial.printf("V=%.1f I=%.2f P=%.0f%s E_fwd=%.3f E_rev=%.3f PF=%.3f F=%.1f\n",
                mVoltage, mCurrent, mPower, mPowerReverse ? "(R)" : "",
                mFwdEnergy, mRevEnergy, mPowerFactor, mFrequency);
}

// ============== DISPLAY FUNCTIONS ==============
void displaySummary() {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setRotation(3);  // Landscape, USB on right

  // Title bar
  M5.Lcd.fillRect(0, 0, 240, 20, TFT_NAVY);
  M5.Lcd.setTextColor(TFT_CYAN, TFT_NAVY);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(4, 6);
  M5.Lcd.print("WEM3080 Monitor");

  // WiFi/Status indicator
  M5.Lcd.setCursor(170, 6);
  if (wifiConnected) {
    M5.Lcd.setTextColor(TFT_GREEN, TFT_NAVY);
    M5.Lcd.printf("WiFi:%s", WiFi.localIP().toString().c_str());
  } else {
    M5.Lcd.setTextColor(TFT_RED, TFT_NAVY);
    M5.Lcd.print("WiFi:OFF");
  }

  if (!mValid) {
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(TFT_RED);
    M5.Lcd.setCursor(50, 55);
    M5.Lcd.print("NO DATA");
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(TFT_DARKGREY);
    M5.Lcd.setCursor(40, 95);
    M5.Lcd.printf("Reads: %d  Errors: %d", mReadCount, mErrorCount);
    return;
  }

  int y = 24;

  // Voltage - Red
  M5.Lcd.setTextColor(TFT_RED);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(4, y);
  M5.Lcd.print("Voltage");
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(4, y + 10);
  M5.Lcd.printf("%.1fV", mVoltage);

  // Current - Yellow
  M5.Lcd.setTextColor(TFT_YELLOW);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(130, y);
  M5.Lcd.print("Current");
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(130, y + 10);
  M5.Lcd.printf("%.2fA", mCurrent);

  y = 65;

  // Power - Green
  M5.Lcd.setTextColor(TFT_GREEN);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(4, y);
  M5.Lcd.print(mPowerReverse ? "Power (Rev)" : "Power");
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(4, y + 10);
  M5.Lcd.printf("%.0fW", mPower);

  // Energy - Blue
  M5.Lcd.setTextColor(COLOR_SKYBLUE);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(130, y);
  M5.Lcd.print("Energy");
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(130, y + 12);
  M5.Lcd.printf("%.2f", mFwdEnergy);
  M5.Lcd.setTextSize(1);
  M5.Lcd.print("kWh");

  // Bottom bar: PF + Freq
  M5.Lcd.fillRect(0, 110, 240, 25, 0x1082);  // Dark grey bar
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TFT_ORANGE);
  M5.Lcd.setCursor(4, 113);
  M5.Lcd.printf("PF:%.2f", mPowerFactor);
  M5.Lcd.setTextColor(TFT_MAGENTA);
  M5.Lcd.setCursor(130, 113);
  M5.Lcd.printf("%.1fHz", mFrequency);
}

void displayDetail() {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setRotation(3);

  // Title bar
  M5.Lcd.fillRect(0, 0, 240, 20, 0x4008);  // Dark purple
  M5.Lcd.setTextColor(TFT_MAGENTA, 0x4008);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(4, 6);
  M5.Lcd.print("WEM3080 Detail");
  M5.Lcd.setCursor(180, 6);
  M5.Lcd.printf("Pg %d/2", displayPage + 1);

  if (!mValid) {
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_RED);
    M5.Lcd.setCursor(30, 60);
    M5.Lcd.print("ERR: No Response");
    return;
  }

  int y = 24;
  M5.Lcd.setTextSize(2);

  // Row 1: Voltage
  M5.Lcd.setTextColor(TFT_RED);
  M5.Lcd.setCursor(4, y);
  M5.Lcd.printf("V: %.1f V", mVoltage);
  y += 20;

  // Row 2: Current
  M5.Lcd.setTextColor(TFT_YELLOW);
  M5.Lcd.setCursor(4, y);
  M5.Lcd.printf("I: %.2f A", mCurrent);
  y += 20;

  // Row 3: Power
  M5.Lcd.setTextColor(TFT_GREEN);
  M5.Lcd.setCursor(4, y);
  M5.Lcd.printf("P: %.0f W %s", mPower, mPowerReverse ? "(R)" : "");
  y += 20;

  // Row 4: Forward Energy
  M5.Lcd.setTextColor(COLOR_SKYBLUE);
  M5.Lcd.setCursor(4, y);
  M5.Lcd.printf("Fwd: %.3f kWh", mFwdEnergy);
  y += 20;

  // Row 5: Reverse Energy
  M5.Lcd.setTextColor(TFT_CYAN);
  M5.Lcd.setCursor(4, y);
  M5.Lcd.printf("Rev: %.3f kWh", mRevEnergy);

  // Bottom bar
  M5.Lcd.fillRect(0, 118, 240, 17, 0x1082);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(TFT_DARKGREY);
  M5.Lcd.setCursor(4, 122);
  M5.Lcd.printf("Reads:%d Err:%d Heap:%d", mReadCount, mErrorCount, ESP.getFreeHeap());
}

void updateDisplay() {
  if (displayPage == 0) {
    displaySummary();
  } else {
    displayDetail();
  }
}

// ============== API KEY CHECK ==============
bool checkApiKey() {
  if (!server.hasArg("key") || server.arg("key") != API_KEY) {
    server.send(403, "text/html",
      "<html><body style='background:#0f0f1a;color:#ff6b6b;font-family:monospace;text-align:center;padding:50px'>"
      "<h1>403 Forbidden</h1><p>Invalid or missing API key.<br>Use ?key=YOUR_KEY</p></body></html>");
    return false;
  }
  return true;
}

// ============== WEB DASHBOARD HTML ==============
const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>WEM3080 Monitor</title>
  <style>
    body{font-family:'Segoe UI',Arial,sans-serif;background:#0f0f1a;color:#eee;margin:0;padding:10px}
    .container{max-width:520px;margin:0 auto}
    h1{text-align:center;font-size:1.5em;margin:10px 0;background:linear-gradient(90deg,#ff6b6b,#ffd93d,#6bcb77,#4d96ff,#9b59b6);-webkit-background-clip:text;-webkit-text-fill-color:transparent;background-clip:text}
    .grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin:12px 0}
    .card{border-radius:12px;padding:14px;text-align:center;position:relative;overflow:hidden}
    .card .label{font-size:0.75em;text-transform:uppercase;letter-spacing:1px;margin-bottom:6px;opacity:0.85}
    .card .value{font-size:2em;font-weight:bold;text-shadow:0 0 15px currentColor}
    .card .unit{font-size:0.75em;opacity:0.7;margin-top:2px}
    .c-volt{background:#1a1025;border:1px solid #e74c3c;color:#ff6b6b}
    .c-volt .value{color:#ff6b6b}
    .c-curr{background:#1a1a05;border:1px solid #f39c12;color:#ffd93d}
    .c-curr .value{color:#ffd93d}
    .c-power{background:#0a1a10;border:1px solid #27ae60;color:#6bcb77}
    .c-power .value{color:#6bcb77}
    .c-energy{background:#0a1525;border:1px solid #2980b9;color:#4d96ff}
    .c-energy .value{color:#4d96ff}
    .c-freq{background:#1a0a25;border:1px solid #8e44ad;color:#c084fc}
    .c-freq .value{color:#c084fc}
    .c-pf{background:#1a1510;border:1px solid #e67e22;color:#fb923c}
    .c-pf .value{color:#fb923c}
    .c-rev{background:#0a1a1a;border:1px solid #16a085;color:#2dd4bf}
    .c-rev .value{color:#2dd4bf}
    .c-dir{background:#1a0a15;border:1px solid #c0392b;color:#f87171}
    .c-dir .value{color:#f87171}
    .status{background:#161625;padding:8px;border-radius:8px;margin:8px 0;font-size:0.75em;text-align:center;border:1px solid #2a2a40}
    .ok{color:#6bcb77} .err{color:#ff6b6b}
    a{color:#4d96ff;text-decoration:none}
    .btn{display:inline-block;padding:8px 16px;background:#1e1e35;color:#4d96ff;border-radius:6px;margin:4px;border:1px solid #333}
    .btn:hover{background:#2a2a45;border-color:#4d96ff}
  </style>
</head>
<body>
  <div class="container">
    <h1>WEM3080 Energy Monitor</h1>
    <div class="status">
      WiFi: <span class="%WCLASS%">%WSTATUS%</span> |
      Meter: <span class="%MCLASS%">%MSTATUS%</span> |
      Reads: %READS% | Errors: %ERRORS%
    </div>
    <div class="grid">
      <div class="card c-volt">
        <div class="label">Voltage</div>
        <div class="value">%VOLT%</div>
        <div class="unit">V</div>
      </div>
      <div class="card c-curr">
        <div class="label">Current</div>
        <div class="value">%CURR%</div>
        <div class="unit">A</div>
      </div>
      <div class="card c-power">
        <div class="label">Power</div>
        <div class="value">%POWER%</div>
        <div class="unit">W</div>
      </div>
      <div class="card c-energy">
        <div class="label">Fwd Energy</div>
        <div class="value">%FWDE%</div>
        <div class="unit">kWh</div>
      </div>
      <div class="card c-freq">
        <div class="label">Frequency</div>
        <div class="value">%FREQ%</div>
        <div class="unit">Hz</div>
      </div>
      <div class="card c-pf">
        <div class="label">Power Factor</div>
        <div class="value">%PF%</div>
        <div class="unit"></div>
      </div>
      <div class="card c-rev">
        <div class="label">Rev Energy</div>
        <div class="value">%REVE%</div>
        <div class="unit">kWh</div>
      </div>
      <div class="card c-dir">
        <div class="label">Direction</div>
        <div class="value">%DIR%</div>
        <div class="unit"></div>
      </div>
    </div>
    <div class="status">
      Heap: %HEAP% bytes | IP: %IPADDR% | RSSI: %RSSI% dBm<br>
      <a class="btn" href="/?key=%KEY%">Refresh</a>
      <a class="btn" href="/json?key=%KEY%">JSON</a>
    </div>
  </div>
  <script>setTimeout(function(){location.reload();},5000);</script>
</body>
</html>
)rawliteral";

// ============== WEB HANDLERS ==============
static char jsonBuf[384];

void handleDashboard() {
  if (!checkApiKey()) return;

  String html = FPSTR(DASHBOARD_HTML);
  html.replace("%WCLASS%", wifiConnected ? "ok" : "err");
  html.replace("%WSTATUS%", wifiConnected ? "Connected" : "Disconnected");
  html.replace("%MCLASS%", mValid ? "ok" : "err");
  html.replace("%MSTATUS%", mValid ? "OK" : "No Data");
  html.replace("%READS%", String(mReadCount));
  html.replace("%ERRORS%", String(mErrorCount));

  if (mValid) {
    html.replace("%VOLT%", String(mVoltage, 1));
    html.replace("%CURR%", String(mCurrent, 2));
    html.replace("%POWER%", String(mPower, 0));
    html.replace("%FWDE%", String(mFwdEnergy, 3));
    html.replace("%REVE%", String(mRevEnergy, 3));
    html.replace("%FREQ%", String(mFrequency, 1));
    html.replace("%PF%", String(mPowerFactor, 3));
    html.replace("%DIR%", mPowerReverse ? "REVERSE" : "FORWARD");
  } else {
    html.replace("%VOLT%", "--");
    html.replace("%CURR%", "--");
    html.replace("%POWER%", "--");
    html.replace("%FWDE%", "--");
    html.replace("%REVE%", "--");
    html.replace("%FREQ%", "--");
    html.replace("%PF%", "--");
    html.replace("%DIR%", "--");
  }

  html.replace("%HEAP%", String(ESP.getFreeHeap()));
  html.replace("%IPADDR%", WiFi.localIP().toString());
  html.replace("%RSSI%", String(WiFi.RSSI()));
  html.replace("%KEY%", API_KEY);

  server.send(200, "text/html", html);
}

void handleJson() {
  if (!checkApiKey()) return;

  snprintf(jsonBuf, sizeof(jsonBuf),
    "{\"voltage\":%.1f,\"current\":%.2f,\"power\":%.0f,"
    "\"fwd_energy\":%.3f,\"rev_energy\":%.3f,"
    "\"power_factor\":%.3f,\"frequency\":%.1f,"
    "\"direction\":\"%s\","
    "\"valid\":%s,\"reads\":%d,\"errors\":%d,"
    "\"heap\":%u,\"rssi\":%d}",
    mValid ? mVoltage : 0,
    mValid ? mCurrent : 0,
    mValid ? mPower : 0,
    mValid ? mFwdEnergy : 0,
    mValid ? mRevEnergy : 0,
    mValid ? mPowerFactor : 0,
    mValid ? mFrequency : 0,
    mPowerReverse ? "reverse" : "forward",
    mValid ? "true" : "false",
    mReadCount, mErrorCount,
    ESP.getFreeHeap(),
    WiFi.RSSI());

  server.send(200, "application/json", jsonBuf);
}

// ============== WIFI CONNECTION ==============
void connectWiFi() {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setRotation(3);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TFT_CYAN);
  M5.Lcd.setCursor(10, 30);
  M5.Lcd.print("WiFi...");
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(10, 55);
  M5.Lcd.setTextColor(TFT_DARKGREY);
  M5.Lcd.print(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    M5.Lcd.print(".");
    attempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.print(F("WiFi IP: "));
    Serial.println(WiFi.localIP());

    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_GREEN);
    M5.Lcd.setCursor(10, 30);
    M5.Lcd.print("WiFi OK!");
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(10, 55);
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.print(WiFi.localIP().toString());
    delay(1500);
  } else {
    Serial.println(F("WiFi failed - continuing offline"));
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_RED);
    M5.Lcd.setCursor(10, 75);
    M5.Lcd.print("FAILED");
    delay(2000);
  }
}

// ============== SETUP ==============
void setup() {
  M5.begin();
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);

  // Splash screen
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TFT_CYAN);
  M5.Lcd.setCursor(20, 25);
  M5.Lcd.print("WEM3080");
  M5.Lcd.setCursor(20, 50);
  M5.Lcd.setTextColor(TFT_GREEN);
  M5.Lcd.print("Energy Monitor");
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(TFT_DARKGREY);
  M5.Lcd.setCursor(20, 80);
  M5.Lcd.print("M5StickC Plus + RS485");
  M5.Lcd.setCursor(20, 95);
  M5.Lcd.print("Modbus RTU 9600/8N1");
  delay(2000);

  // Init RS485 serial
  RS485.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  Serial.println(F("RS485 initialized"));

  // Connect WiFi
  connectWiFi();

  // Start web server
  if (wifiConnected) {
    server.on("/", handleDashboard);
    server.on("/json", handleJson);
    server.begin();
    Serial.println(F("Web server started"));
    Serial.printf("Dashboard: http://%s/?key=%s\n", WiFi.localIP().toString().c_str(), API_KEY);
  }

  Serial.printf("Heap: %u bytes\n", ESP.getFreeHeap());
  Serial.println(F("Ready!"));
}

// ============== MAIN LOOP ==============
void loop() {
  unsigned long now = millis();

  M5.update();

  // Button A: toggle display page
  if (M5.BtnA.wasPressed()) {
    displayPage = (displayPage + 1) % 2;
    updateDisplay();
    Serial.printf("Display page: %d\n", displayPage);
  }

  // Read Modbus
  if (now - lastRead >= READ_INTERVAL) {
    lastRead = now;
    readWEM3080();
  }

  // Update display
  if (now - lastDisplay >= DISPLAY_INTERVAL) {
    lastDisplay = now;
    updateDisplay();
  }

  // Handle web clients
  if (wifiConnected) {
    server.handleClient();
  }

  delay(10);
}
