#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include "arduino_secrets.h"

// Log-scale anchors for MindWave band power (raw ASIC counts).
#define EEG_BAND_MIN 10000.0f      // maps to 10%
#define EEG_BAND_MAX 1000000.0f    // maps to 100%
#define RF_DISPLAY_GAIN 1.5f       // L-RF / H-RF bar gain

// MQTT JSON payloads:
//   sensors/rf/rtlsdr    -> peak_power_dB
//   sensors/rf/hackrf    -> peak_power_dB
//   sensors/rf/eeg       -> mindwave attention, meditation, asic_bands
//   sensors/bio/gsr      -> nervous_level

// ========== Screen Config ==========
#define PANEL_WIDTH   96
#define PANEL_HEIGHT  48
#define CHAIN_LENGTH  1

#define NUM_PANELS 4
#define PANEL_W    24

// 0 = no divider lines, avoids blue vertical line confusion
#define SHOW_GRID 0

// ========== Pin Config ==========
#define R1_PIN  27
#define G1_PIN  26
#define B1_PIN  25
#define R2_PIN  33
#define G2_PIN  32
#define B2_PIN  14

#define A_PIN   19
#define B_PIN   18
#define C_PIN   5
#define D_PIN   17
#define E_PIN   16

#define CLK_PIN 22
#define LAT_PIN 21
#define OE_PIN  23

MatrixPanel_I2S_DMA *dma_display = nullptr;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

const char* MQTT_USER = MQTT_USERNAME;
const char* MQTT_PASSWORD = MQTT_PASS;

bool wifiReady = false;
bool mqttReady = false;
unsigned long lastWifiRetry = 0;
unsigned long lastMqttRetry = 0;
const unsigned long WIFI_RETRY_INTERVAL = 15000;
const unsigned long MQTT_RETRY_INTERVAL = 5000;

// ========== Live Data ==========
int lowMidRfLevel = 0;
int highRfLevel = 0;
int brainwaveLevel = 1;
int gsrLevel = 1;

int prevLowMidRfLevel = 0;
int prevHighRfLevel = 0;

int lowMidFluc = 0;
int highFluc = 0;

String lowMidFlucState = "LO-FL";
String highFlucState = "LO-FL";

String brainwaveState = "CALM";

float lastEegAlpha = 0;
float lastEegBeta = 0;
float lastEegTheta = 0;
float lastEegGamma = 0;
float lastEegAttention = 0;
float lastEegMeditation = 0;
bool hasLastEegBands = false;
String gsrState = "LOW";

unsigned long lastDisplayUpdate = 0;
unsigned long lastSerialPrint = 0;

const unsigned long DISPLAY_INTERVAL = 100;
const unsigned long SERIAL_PRINT_INTERVAL = 2000;

// ========== Function Declarations ==========
struct SensorPanelData {
  bool hasAttention;
  bool hasMeditation;
  bool hasPoorSignal;
  float attention;
  float meditation;
  float poorSignal;
  bool hasTheta;
  bool hasLowAlpha;
  bool hasHighAlpha;
  bool hasLowBeta;
  bool hasHighBeta;
  bool hasLowGamma;
  bool hasMidGamma;
  float theta;
  float lowAlpha;
  float highAlpha;
  float lowBeta;
  float highBeta;
  float lowGamma;
  float midGamma;
  bool hasBandsTheta;
  bool hasBandsAlpha;
  bool hasBandsBeta;
  bool hasBandsGamma;
  float bandsTheta;
  float bandsAlpha;
  float bandsBeta;
  float bandsGamma;
};

bool connectWiFi();
void connectMqtt();
void mqttCallback(char* topic, byte* payload, unsigned int length);
bool jsonGetFloat(const char* json, const char* key, float& outValue);
bool jsonGetInt(const char* json, const char* key, int& outValue);
void parseBandsBlock(const char* json, SensorPanelData& data);
float bandLogPercent(float rawPower);
int rfCombinedPercent();
void parseRtlPayload(const char* json);
void parseHackrfPayload(const char* json);
void parseEegPayload(const char* json);
void parseGsrPayload(const char* json);
int peakPowerDbToPercent(float peakDb);
int toLevel1_4(float value);
int classifyBrainwaveFromBands(float alpha, float beta, float theta, float gamma);
int classifyMindwaveScores(float attention, float meditation);
void applyBrainwaveFromBands(float alpha, float beta, float theta, float gamma);
void applyBrainwaveFromSensorData(const SensorPanelData& data);
void applyBrainwaveLevel(int level);
void applyGsrLevel(int level);
void updateFluctuationStates();
void printDataToSerial();

void drawDashboard();
void drawPanelBorders();
void drawRfPanel(int panelIndex, const char* label, int value, String flucState, int flucLevel);
void drawStatePanel(int panelIndex, const char* label, int level, String stateText);

void drawTinyText(int x, int y, const char* text, uint16_t color);
void drawTinyChar(int x, int y, char c, uint16_t color);
void drawTinyHeart(int x, int y, uint16_t color);

uint16_t getSpectrumColorFromPercent(int value);
uint16_t getSpectrumColorFromLevel(int level);

// ========== Setup ==========
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("HUB75 96x48 tiny vertical dashboard starting...");

  HUB75_I2S_CFG mxconfig;

  mxconfig.mx_width = PANEL_WIDTH;
  mxconfig.mx_height = PANEL_HEIGHT;
  mxconfig.chain_length = CHAIN_LENGTH;

  mxconfig.gpio.r1  = R1_PIN;
  mxconfig.gpio.g1  = G1_PIN;
  mxconfig.gpio.b1  = B1_PIN;
  mxconfig.gpio.r2  = R2_PIN;
  mxconfig.gpio.g2  = G2_PIN;
  mxconfig.gpio.b2  = B2_PIN;

  mxconfig.gpio.a   = A_PIN;
  mxconfig.gpio.b   = B_PIN;
  mxconfig.gpio.c   = C_PIN;
  mxconfig.gpio.d   = D_PIN;
  mxconfig.gpio.e   = E_PIN;

  mxconfig.gpio.clk = CLK_PIN;
  mxconfig.gpio.lat = LAT_PIN;
  mxconfig.gpio.oe  = OE_PIN;

  // If ghost lines still appear, try true.
  mxconfig.clkphase = false;

  mxconfig.i2sspeed = HUB75_I2S_CFG::HZ_8M;
  mxconfig.min_refresh_rate = 60;

  dma_display = new MatrixPanel_I2S_DMA(mxconfig);

  if (!dma_display->begin()) {
    Serial.println("Display init failed!");
    while (true) {
      delay(1000);
    }
  }

  dma_display->setBrightness8(100);
  dma_display->clearScreen();

  randomSeed(micros());

  connectWiFi();
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setBufferSize(1024);
  mqttClient.setKeepAlive(60);
  mqttClient.setCallback(mqttCallback);
  if (WiFi.status() == WL_CONNECTED) {
    connectMqtt();
  }

  Serial.println("Display initialized (MQTT mode).");
}

// ========== Main Loop ==========
void loop() {
  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    wifiReady = false;
    mqttReady = false;
    if (now - lastWifiRetry >= WIFI_RETRY_INTERVAL) {
      lastWifiRetry = now;
      connectWiFi();
    }
  } else {
    if (!wifiReady) {
      wifiReady = true;
      Serial.println("WiFi connected.");
      connectMqtt();
    }

    if (!mqttClient.connected()) {
      mqttReady = false;
      if (now - lastMqttRetry >= MQTT_RETRY_INTERVAL) {
        lastMqttRetry = now;
        connectMqtt();
      }
    } else if (!mqttReady) {
      mqttReady = true;
      Serial.println("MQTT subscribed.");
    }

    mqttClient.loop();
  }

  if (now - lastSerialPrint >= SERIAL_PRINT_INTERVAL) {
    lastSerialPrint = now;
    printDataToSerial();
  }

  if (now - lastDisplayUpdate >= DISPLAY_INTERVAL) {
    lastDisplayUpdate = now;
    drawDashboard();
  }
}

// ========== WiFi ==========
bool connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  Serial.print("Connecting to WiFi: ");
  Serial.println(SECRET_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(SECRET_SSID, SECRET_PASS);

  for (int i = 0; i < 30; i++) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("WiFi OK, IP: ");
      Serial.println(WiFi.localIP());
      wifiReady = true;
      return true;
    }
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connect failed.");
  wifiReady = false;
  return false;
}

// ========== MQTT ==========
void connectMqtt() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  Serial.print("Connecting MQTT ");
  Serial.print(MQTT_SERVER);
  Serial.print("... ");

  String clientId = "esp32-led-";
  clientId += String(random(0xffff), HEX);

  bool connected = false;
  if (strlen(MQTT_USER) > 0) {
    connected = mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD);
  } else {
    connected = mqttClient.connect(clientId.c_str());
  }

  if (!connected) {
    Serial.print("failed rc=");
    Serial.println(mqttClient.state());
    return;
  }

  mqttClient.subscribe(TOPIC_RF_RTL);
  mqttClient.subscribe(TOPIC_RF_HACKRF);
  mqttClient.subscribe(TOPIC_EEG);
  mqttClient.subscribe(TOPIC_GSR);

  Serial.println("OK");
  Serial.println("Subscribed:");
  Serial.println(TOPIC_RF_RTL);
  Serial.println(TOPIC_RF_HACKRF);
  Serial.println(TOPIC_EEG);
  Serial.println(TOPIC_GSR);
  mqttReady = true;
}

bool jsonGetFloat(const char* json, const char* key, float& outValue) {
  String search = String("\"") + key + "\":";
  String payload = String(json);
  int idx = payload.indexOf(search);
  if (idx < 0) {
    return false;
  }

  int start = idx + search.length();
  while (start < (int)payload.length() && payload.charAt(start) == ' ') {
    start++;
  }
  if (start < (int)payload.length() && payload.charAt(start) == '"') {
    return false;
  }

  int end = start;
  while (end < (int)payload.length()) {
    char c = payload.charAt(end);
    if ((c >= '0' && c <= '9') || c == '-' || c == '.' || c == 'e' || c == 'E' || c == '+') {
      end++;
    } else {
      break;
    }
  }

  if (end <= start) {
    return false;
  }

  outValue = payload.substring(start, end).toFloat();
  return true;
}

bool jsonGetInt(const char* json, const char* key, int& outValue) {
  float value = 0.0f;
  if (!jsonGetFloat(json, key, value)) {
    return false;
  }
  outValue = (int)(value + 0.5f);
  return true;
}

void parseBandsBlock(const char* json, SensorPanelData& data) {
  const char* bands = strstr(json, "\"bands\"");
  if (!bands) {
    return;
  }

  if (jsonGetFloat(bands, "theta", data.bandsTheta)) {
    data.hasBandsTheta = true;
  }
  if (jsonGetFloat(bands, "alpha", data.bandsAlpha)) {
    data.hasBandsAlpha = true;
  }
  if (jsonGetFloat(bands, "beta", data.bandsBeta)) {
    data.hasBandsBeta = true;
  }
  if (jsonGetFloat(bands, "gamma", data.bandsGamma)) {
    data.hasBandsGamma = true;
  }
}

float bandLogPercent(float rawPower) {
  if (rawPower <= 0.0f) {
    return 0.0f;
  }

  float logVal = log10f(rawPower);
  float logMin = log10f(EEG_BAND_MIN);  // 4.0 at 10,000
  float logMax = log10f(EEG_BAND_MAX);  // 6.0 at 1,000,000

  if (logVal <= logMin) {
    return constrain(10.0f * (logVal / logMin), 0.0f, 10.0f);
  }
  if (logVal >= logMax) {
    return 100.0f;
  }

  // 10,000 -> 10%, 1,000,000 -> 100%
  return 10.0f + ((logVal - logMin) / (logMax - logMin)) * 90.0f;
}

int rfCombinedPercent() {
  return constrain((lowMidRfLevel + highRfLevel) / 2, 0, 100);
}

void parseRtlPayload(const char* json) {
  float peakDb = 0.0f;
  if (!jsonGetFloat(json, "peak_power_dB", peakDb)) {
    return;
  }

  prevLowMidRfLevel = lowMidRfLevel;
  lowMidRfLevel = peakPowerDbToPercent(peakDb);
  updateFluctuationStates();

  Serial.print("MQTT rtlsdr peak_power_dB=");
  Serial.print(peakDb, 1);
  Serial.print(" -> L-RF ");
  Serial.print(lowMidRfLevel);
  Serial.println("%");
}

void parseHackrfPayload(const char* json) {
  float peakDb = 0.0f;
  if (!jsonGetFloat(json, "peak_power_dB", peakDb)) {
    return;
  }

  prevHighRfLevel = highRfLevel;
  highRfLevel = peakPowerDbToPercent(peakDb);
  updateFluctuationStates();

  Serial.print("MQTT hackrf peak_power_dB=");
  Serial.print(peakDb, 1);
  Serial.print(" -> H-RF ");
  Serial.print(highRfLevel);
  Serial.println("%");
}

void parseEegPayload(const char* json) {
  SensorPanelData data = {};

  if (jsonGetFloat(json, "attention", data.attention)) {
    data.hasAttention = true;
  }
  if (jsonGetFloat(json, "meditation", data.meditation)) {
    data.hasMeditation = true;
  }
  if (jsonGetFloat(json, "poor_signal", data.poorSignal)) {
    data.hasPoorSignal = true;
  }

  if (jsonGetFloat(json, "theta", data.theta)) {
    data.hasTheta = true;
  }
  if (jsonGetFloat(json, "low_alpha", data.lowAlpha)) {
    data.hasLowAlpha = true;
  }
  if (jsonGetFloat(json, "high_alpha", data.highAlpha)) {
    data.hasHighAlpha = true;
  }
  if (jsonGetFloat(json, "low_beta", data.lowBeta)) {
    data.hasLowBeta = true;
  }
  if (jsonGetFloat(json, "high_beta", data.highBeta)) {
    data.hasHighBeta = true;
  }
  if (jsonGetFloat(json, "low_gamma", data.lowGamma)) {
    data.hasLowGamma = true;
  }
  if (jsonGetFloat(json, "mid_gamma", data.midGamma)) {
    data.hasMidGamma = true;
  }

  parseBandsBlock(json, data);
  applyBrainwaveFromSensorData(data);
}

void parseGsrPayload(const char* json) {
  int level = 0;
  if (!jsonGetInt(json, "nervous_level", level)) {
    return;
  }

  applyGsrLevel(toLevel1_4((float)level));
  Serial.print("MQTT GSR nervous_level=");
  Serial.println(gsrLevel);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (length >= 1024) {
    return;
  }

  char message[1024];
  memcpy(message, payload, length);
  message[length] = '\0';

  if (strcmp(topic, TOPIC_RF_RTL) == 0) {
    parseRtlPayload(message);
  } else if (strcmp(topic, TOPIC_RF_HACKRF) == 0) {
    parseHackrfPayload(message);
  } else if (strcmp(topic, TOPIC_EEG) == 0) {
    parseEegPayload(message);
  } else if (strcmp(topic, TOPIC_GSR) == 0) {
    parseGsrPayload(message);
  }
}

int peakPowerDbToPercent(float peakDb) {
  float pct = 0.0f;

  if (peakDb >= 0.0f && peakDb <= 100.0f) {
    pct = peakDb;
  } else {
    pct = (float)map((int)(peakDb + 0.5f), -90, -30, 0, 100);
  }

  pct *= RF_DISPLAY_GAIN;
  return constrain((int)(pct + 0.5f), 0, 100);
}

int toLevel1_4(float value) {
  if (value >= 1.0f && value <= 4.0f) {
    return constrain((int)(value + 0.5f), 1, 4);
  }

  int level = map((int)(value + 0.5f), 0, 100, 1, 4);
  return constrain(level, 1, 4);
}

int classifyMindwaveScores(float attention, float meditation) {
  // Mindwave attention/meditation are 0-100 scores from the headset.
  if (meditation >= 55.0f && attention < 45.0f) {
    return 1;  // CALM - meditative, low attention demand
  }
  if (attention >= 75.0f && meditation < 45.0f) {
    return 4;  // GAMM - intense focused arousal
  }
  if (attention >= 60.0f && meditation < 40.0f) {
    return 3;  // ALRT - high attention, low calm
  }
  if (attention >= 40.0f) {
    return 2;  // FOC - engaged (e.g. attention=48, meditation=56)
  }
  return 1;
}

void applyBrainwaveFromSensorData(const SensorPanelData& data) {
  float rawAlpha = 0.0f;
  float rawBeta = 0.0f;
  float rawTheta = 0.0f;
  float rawGamma = 0.0f;

  if (data.hasBandsAlpha) {
    rawAlpha = data.bandsAlpha;
  } else {
    rawAlpha = (data.hasLowAlpha ? data.lowAlpha : 0.0f) + (data.hasHighAlpha ? data.highAlpha : 0.0f);
  }

  if (data.hasBandsBeta) {
    rawBeta = data.bandsBeta;
  } else {
    rawBeta = (data.hasLowBeta ? data.lowBeta : 0.0f) + (data.hasHighBeta ? data.highBeta : 0.0f);
  }

  if (data.hasBandsTheta) {
    rawTheta = data.bandsTheta;
  } else if (data.hasTheta) {
    rawTheta = data.theta;
  }

  if (data.hasBandsGamma) {
    rawGamma = data.bandsGamma;
  } else {
    rawGamma = (data.hasLowGamma ? data.lowGamma : 0.0f) + (data.hasMidGamma ? data.midGamma : 0.0f);
  }

  float bandTotal = rawAlpha + rawBeta + rawTheta + rawGamma;
  bool hasBandData = bandTotal > 0.0f;

  if (data.hasPoorSignal && data.poorSignal > 50.0f && !hasBandData) {
    applyBrainwaveLevel(1);
    Serial.println("EEG poor signal - holding CALM");
    return;
  }

  if (hasBandData) {
    float alpha = bandLogPercent(rawAlpha);
    float beta = bandLogPercent(rawBeta);
    float theta = bandLogPercent(rawTheta);
    float gamma = bandLogPercent(rawGamma);

    lastEegAlpha = alpha;
    lastEegBeta = beta;
    lastEegTheta = theta;
    lastEegGamma = gamma;
    lastEegAttention = data.hasAttention ? data.attention : 0.0f;
    lastEegMeditation = data.hasMeditation ? data.meditation : 0.0f;
    hasLastEegBands = true;

    applyBrainwaveFromBands(alpha, beta, theta, gamma);

    Serial.print("EEG log-scaled theta=");
    Serial.print(theta, 1);
    Serial.print(" alpha=");
    Serial.print(alpha, 1);
    Serial.print(" beta=");
    Serial.print(beta, 1);
    Serial.print(" gamma=");
    Serial.print(gamma, 1);
    Serial.print(" (raw theta=");
    Serial.print(rawTheta, 0);
    Serial.println(")");
    return;
  }

  if (data.hasAttention || data.hasMeditation) {
    float attention = data.hasAttention ? data.attention : 50.0f;
    float meditation = data.hasMeditation ? data.meditation : 50.0f;
    int level = classifyMindwaveScores(attention, meditation);
    applyBrainwaveLevel(level);

    lastEegAttention = attention;
    lastEegMeditation = meditation;
    hasLastEegBands = false;

    Serial.print("MQTT EEG attention=");
    Serial.print(attention, 0);
    Serial.print(" meditation=");
    Serial.print(meditation, 0);
    Serial.print(" -> L");
    Serial.print(level);
    Serial.print(" ");
    Serial.println(brainwaveState);
  }
}

int classifyBrainwaveFromBands(float alpha, float beta, float theta, float gamma) {
  float total = alpha + beta + theta + gamma;
  if (total < 0.001f) {
    return 1;
  }

  float thetaRatio = theta / total;
  float alphaRatio = alpha / total;
  float betaRatio = beta / total;
  float gammaRatio = gamma / total;

  // L4 GAMM: gamma + beta surge — intense / peak processing
  if (gammaRatio > 0.18f && betaRatio > 0.28f) {
    return 4;
  }

  // L3 ALRT: beta dominant, theta suppressed — alert / tense
  if (betaRatio > 0.32f && thetaRatio < 0.18f) {
    return 3;
  }

  // L1 CALM: theta + alpha dominant — relaxed / meditative
  if (thetaRatio > 0.22f || (thetaRatio + alphaRatio) > 0.50f) {
    return 1;
  }

  // L2 FOC: balanced alpha + beta — engaged focus
  if (betaRatio > 0.18f && alphaRatio > 0.15f) {
    return 2;
  }

  return 2;
}

void applyBrainwaveFromBands(float alpha, float beta, float theta, float gamma) {
  int level = classifyBrainwaveFromBands(alpha, beta, theta, gamma);
  applyBrainwaveLevel(level);

  Serial.print("EEG bands theta=");
  Serial.print(theta, 1);
  Serial.print(" alpha=");
  Serial.print(alpha, 1);
  Serial.print(" beta=");
  Serial.print(beta, 1);
  Serial.print(" gamma=");
  Serial.print(gamma, 1);
  Serial.print(" -> L");
  Serial.print(level);
  Serial.print(" ");
  Serial.println(brainwaveState);
}

void applyBrainwaveLevel(int level) {
  brainwaveLevel = constrain(level, 1, 4);

  if (brainwaveLevel == 1) {
    brainwaveState = "CALM";   // theta+alpha: relaxed / meditative
  } else if (brainwaveLevel == 2) {
    brainwaveState = "FOC";    // alpha+beta: focused
  } else if (brainwaveLevel == 3) {
    brainwaveState = "ALRT";   // beta high: alert / tense
  } else {
    brainwaveState = "GAMM";   // gamma+beta: intense peak (replaces HIGH)
  }
}

void applyGsrLevel(int level) {
  gsrLevel = constrain(level, 1, 4);

  if (gsrLevel == 1) {
    gsrState = "LOW";
  } else if (gsrLevel == 2) {
    gsrState = "MILD";
  } else if (gsrLevel == 3) {
    gsrState = "MID";
  } else {
    gsrState = "HIGH";
  }
}

void updateFluctuationStates() {
  lowMidFluc = abs(lowMidRfLevel - prevLowMidRfLevel);
  highFluc = abs(highRfLevel - prevHighRfLevel);

  lowMidFlucState = (lowMidFluc >= 25) ? "HI-FL" : "LO-FL";
  highFlucState = (highFluc >= 25) ? "HI-FL" : "LO-FL";
}

// ========== Draw Dashboard ==========
void drawDashboard() {
  dma_display->fillScreen(dma_display->color565(0, 0, 0));

  if (SHOW_GRID) {
    drawPanelBorders();
  }

  int lowFlucLevel = constrain(map(lowMidFluc, 0, 60, 1, 4), 1, 4);
  int highFlucLevel = constrain(map(highFluc, 0, 60, 1, 4), 1, 4);

  drawRfPanel(0, "L-RF", lowMidRfLevel, lowMidFlucState, lowFlucLevel);
  drawRfPanel(1, "H-RF", highRfLevel, highFlucState, highFlucLevel);
  drawStatePanel(2, "EEG", brainwaveLevel, brainwaveState);
  drawStatePanel(3, "GSR", gsrLevel, gsrState);
}

// ========== Panel Borders ==========
void drawPanelBorders() {
  uint16_t borderColor = dma_display->color565(5, 5, 8);

  for (int i = 1; i < NUM_PANELS; i++) {
    int x = i * PANEL_W;
    dma_display->drawLine(x, 0, x, PANEL_HEIGHT - 1, borderColor);
  }

  dma_display->drawRect(0, 0, PANEL_WIDTH, PANEL_HEIGHT, borderColor);
}

// ========== RF Panel ==========
void drawRfPanel(int panelIndex, const char* label, int value, String flucState, int flucLevel) {
  int x0 = panelIndex * PANEL_W;

  value = constrain(value, 0, 100);
  flucLevel = constrain(flucLevel, 1, 4);

  uint16_t textColor  = dma_display->color565(180, 180, 190);
  uint16_t labelColor = dma_display->color565(150, 100, 180);
  uint16_t rfColor    = getSpectrumColorFromPercent(value);
  uint16_t emptyColor = dma_display->color565(18, 18, 22);

  // Label: L-RF / H-RF
  drawTinyText(x0 + 3, 3, label, labelColor);

  // Value: 00-99
  char valueText[4];
  int shownValue = value;
  if (shownValue > 99) shownValue = 99;
  sprintf(valueText, "%02d", shownValue);

  drawTinyText(x0 + 8, 12, valueText, textColor);

  // Fluctuation text: HI-FL / LO-FL
  drawTinyText(x0 + 2, 23, flucState.c_str(), rfColor);

  // Hearts aligned to last line
  int heartY = 37;
  int startX = x0 + 2;
  int gap = 5;

  for (int i = 1; i <= 4; i++) {
    int hx = startX + (i - 1) * gap;

    if (i <= flucLevel) {
      drawTinyHeart(hx, heartY, rfColor);
    } else {
      drawTinyHeart(hx, heartY, emptyColor);
    }
  }
}

// ========== EEG / GSR Panel ==========
void drawStatePanel(int panelIndex, const char* label, int level, String stateText) {
  int x0 = panelIndex * PANEL_W;

  level = constrain(level, 1, 4);

  uint16_t textColor  = dma_display->color565(180, 180, 190);
  uint16_t labelColor = dma_display->color565(150, 100, 180);
  uint16_t levelColor = getSpectrumColorFromLevel(level);
  uint16_t emptyColor = dma_display->color565(18, 18, 22);

  drawTinyText(x0 + 6, 3, label, labelColor);

  char levelText[3];
  sprintf(levelText, "L%d", level);
  drawTinyText(x0 + 8, 12, levelText, textColor);

  drawTinyText(x0 + 4, 23, stateText.c_str(), levelColor);

  // Hearts aligned to last line
  int heartY = 37;
  int startX = x0 + 2;
  int gap = 5;

  for (int i = 1; i <= 4; i++) {
    int hx = startX + (i - 1) * gap;

    if (i <= level) {
      drawTinyHeart(hx, heartY, levelColor);
    } else {
      drawTinyHeart(hx, heartY, emptyColor);
    }
  }
}

// ========== Tiny Text Renderer ==========
// 3x5 font, 1 px spacing.
void drawTinyText(int x, int y, const char* text, uint16_t color) {
  int cursorX = x;

  for (int i = 0; text[i] != '\0'; i++) {
    drawTinyChar(cursorX, y, text[i], color);
    cursorX += 4;
  }
}

void drawTinyChar(int x, int y, char c, uint16_t color) {
  byte rows[5] = {0, 0, 0, 0, 0};

  switch (c) {
    case '0': rows[0]=0b111; rows[1]=0b101; rows[2]=0b101; rows[3]=0b101; rows[4]=0b111; break;
    case '1': rows[0]=0b010; rows[1]=0b110; rows[2]=0b010; rows[3]=0b010; rows[4]=0b111; break;
    case '2': rows[0]=0b111; rows[1]=0b001; rows[2]=0b111; rows[3]=0b100; rows[4]=0b111; break;
    case '3': rows[0]=0b111; rows[1]=0b001; rows[2]=0b111; rows[3]=0b001; rows[4]=0b111; break;
    case '4': rows[0]=0b101; rows[1]=0b101; rows[2]=0b111; rows[3]=0b001; rows[4]=0b001; break;
    case '5': rows[0]=0b111; rows[1]=0b100; rows[2]=0b111; rows[3]=0b001; rows[4]=0b111; break;
    case '6': rows[0]=0b111; rows[1]=0b100; rows[2]=0b111; rows[3]=0b101; rows[4]=0b111; break;
    case '7': rows[0]=0b111; rows[1]=0b001; rows[2]=0b010; rows[3]=0b010; rows[4]=0b010; break;
    case '8': rows[0]=0b111; rows[1]=0b101; rows[2]=0b111; rows[3]=0b101; rows[4]=0b111; break;
    case '9': rows[0]=0b111; rows[1]=0b101; rows[2]=0b111; rows[3]=0b001; rows[4]=0b111; break;

    case 'A': rows[0]=0b010; rows[1]=0b101; rows[2]=0b111; rows[3]=0b101; rows[4]=0b101; break;
    case 'C': rows[0]=0b111; rows[1]=0b100; rows[2]=0b100; rows[3]=0b100; rows[4]=0b111; break;
    case 'D': rows[0]=0b110; rows[1]=0b101; rows[2]=0b101; rows[3]=0b101; rows[4]=0b110; break;
    case 'E': rows[0]=0b111; rows[1]=0b100; rows[2]=0b111; rows[3]=0b100; rows[4]=0b111; break;
    case 'F': rows[0]=0b111; rows[1]=0b100; rows[2]=0b111; rows[3]=0b100; rows[4]=0b100; break;
    case 'G': rows[0]=0b111; rows[1]=0b100; rows[2]=0b101; rows[3]=0b101; rows[4]=0b111; break;
    case 'H': rows[0]=0b101; rows[1]=0b101; rows[2]=0b111; rows[3]=0b101; rows[4]=0b101; break;
    case 'I': rows[0]=0b111; rows[1]=0b010; rows[2]=0b010; rows[3]=0b010; rows[4]=0b111; break;
    case 'L': rows[0]=0b100; rows[1]=0b100; rows[2]=0b100; rows[3]=0b100; rows[4]=0b111; break;
    case 'M': rows[0]=0b101; rows[1]=0b111; rows[2]=0b111; rows[3]=0b101; rows[4]=0b101; break;
    case 'O': rows[0]=0b111; rows[1]=0b101; rows[2]=0b101; rows[3]=0b101; rows[4]=0b111; break;
    case 'R': rows[0]=0b110; rows[1]=0b101; rows[2]=0b110; rows[3]=0b101; rows[4]=0b101; break;
    case 'S': rows[0]=0b111; rows[1]=0b100; rows[2]=0b111; rows[3]=0b001; rows[4]=0b111; break;
    case 'T': rows[0]=0b111; rows[1]=0b010; rows[2]=0b010; rows[3]=0b010; rows[4]=0b010; break;
    case 'W': rows[0]=0b101; rows[1]=0b101; rows[2]=0b111; rows[3]=0b111; rows[4]=0b101; break;

    case '-': rows[0]=0b000; rows[1]=0b000; rows[2]=0b111; rows[3]=0b000; rows[4]=0b000; break;
    case ' ': rows[0]=0b000; rows[1]=0b000; rows[2]=0b000; rows[3]=0b000; rows[4]=0b000; break;
    default:  rows[0]=0b111; rows[1]=0b001; rows[2]=0b010; rows[3]=0b000; rows[4]=0b010; break;
  }

  for (int row = 0; row < 5; row++) {
    for (int col = 0; col < 3; col++) {
      if (rows[row] & (1 << (2 - col))) {
        dma_display->drawPixel(x + col, y + row, color);
      }
    }
  }
}

// ========== Tiny 5x5 Heart ==========
void drawTinyHeart(int x, int y, uint16_t color) {
  dma_display->drawPixel(x + 1, y + 0, color);
  dma_display->drawPixel(x + 3, y + 0, color);

  dma_display->drawPixel(x + 0, y + 1, color);
  dma_display->drawPixel(x + 1, y + 1, color);
  dma_display->drawPixel(x + 2, y + 1, color);
  dma_display->drawPixel(x + 3, y + 1, color);
  dma_display->drawPixel(x + 4, y + 1, color);

  dma_display->drawPixel(x + 0, y + 2, color);
  dma_display->drawPixel(x + 1, y + 2, color);
  dma_display->drawPixel(x + 2, y + 2, color);
  dma_display->drawPixel(x + 3, y + 2, color);
  dma_display->drawPixel(x + 4, y + 2, color);

  dma_display->drawPixel(x + 1, y + 3, color);
  dma_display->drawPixel(x + 2, y + 3, color);
  dma_display->drawPixel(x + 3, y + 3, color);

  dma_display->drawPixel(x + 2, y + 4, color);
}

// ========== Colors ==========
uint16_t getSpectrumColorFromPercent(int value) {
  value = constrain(value, 0, 100);

  int r, g, b;

  if (value < 33) {
    float t = value / 33.0;
    r = 90  + t * (210 - 90);
    g = 70  + t * (110 - 70);
    b = 180 + t * (180 - 180);
  } else if (value < 66) {
    float t = (value - 33) / 33.0;
    r = 210 + t * (180 - 210);
    g = 110 + t * (240 - 110);
    b = 180 + t * (100 - 180);
  } else {
    float t = (value - 66) / 34.0;
    r = 180 + t * (80 - 180);
    g = 240 + t * (230 - 240);
    b = 100 + t * (210 - 100);
  }

  return dma_display->color565(r, g, b);
}

uint16_t getSpectrumColorFromLevel(int level) {
  level = constrain(level, 1, 4);

  if (level == 1) {
    return dma_display->color565(90, 70, 180);
  } else if (level == 2) {
    return dma_display->color565(210, 110, 180);
  } else if (level == 3) {
    return dma_display->color565(180, 240, 100);
  } else {
    return dma_display->color565(80, 230, 210);
  }
}

// ========== Serial Debug ==========
void printDataToSerial() {
  Serial.print("Low-Mid RF: ");
  Serial.print(lowMidRfLevel);
  Serial.print("% | Low-Mid fluctuation: ");
  Serial.print(lowMidFluc);
  Serial.print(" ");
  Serial.print(lowMidFlucState);

  Serial.print(" | High RF: ");
  Serial.print(highRfLevel);
  Serial.print("% | High fluctuation: ");
  Serial.print(highFluc);
  Serial.print(" ");
  Serial.print(highFlucState);
  Serial.print(" | RF combined (analysis): ");
  Serial.print(rfCombinedPercent());
  Serial.print("%");

  Serial.print(" | Brainwave Level: ");
  Serial.print(brainwaveLevel);
  Serial.print(" ");
  Serial.print(brainwaveState);
  if (hasLastEegBands) {
    Serial.print(" (att=");
    Serial.print(lastEegAttention, 0);
    Serial.print(" med=");
    Serial.print(lastEegMeditation, 0);
    Serial.print(" theta=");
    Serial.print(lastEegTheta, 1);
    Serial.print(" alpha=");
    Serial.print(lastEegAlpha, 1);
    Serial.print(" beta=");
    Serial.print(lastEegBeta, 1);
    Serial.print(" gamma=");
    Serial.print(lastEegGamma, 1);
    Serial.print(")");
  }

  Serial.print(" | GSR Level: ");
  Serial.print(gsrLevel);
  Serial.print(" ");
  Serial.println(gsrState);
}