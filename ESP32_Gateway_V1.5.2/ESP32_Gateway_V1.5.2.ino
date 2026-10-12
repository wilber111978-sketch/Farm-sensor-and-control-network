// ============================================================
// ESP32 FARM GATEWAY FIRMWARE V1.5.2
// Seeed Wio-SX1262 edition
//
// V1.5.2 weather routing / command-response isolation (source: V1.5.1):
//   - Routes weather_* nodes to retained farm/weather/<node> topics.
//   - Publishes complete node command replies non-retained on
//     farm/node/command/response.
//   - Prevents partial soil command replies from replacing the last complete
//     retained soil telemetry packet.
//   - Preserves complete tank calibration replies on retained tank topics.
//
// V1.5.1 timing fix (source: V1.5.0):
//   - Adds a short guard delay after a successful uplink ACK before
//     sending any queued node command. This gives updated nodes time
//     to leave the ACK receive state and enter their normal command
//     receive window, reducing missed calibration/reboot commands.
//   - Preserves all V1.5.0 ACK, de-duplication, MQTT diagnostics,
//     existing command protocol, radio settings, and legacy-node support.
//
// V1.5.0 reliability protocol update (source: V1.4.4):
//   - Optional per-uplink LoRa delivery ACK using node + sequence/cycle.
//   - ACK is only sent when a node advertises "ack_req": true, so legacy
//     nodes remain fully backward-compatible.
//   - Duplicate retry packets are ACKed again but are not re-published as
//     fresh sensor readings.
//   - Per-reception diagnostics published to farm/gateway/rx/<node>.
//   - Gateway status now exposes ACK, duplicate, and RX diagnostic counters.
//   - Existing node-command/downlink protocol is preserved.
//
// Designed for:
//   - Existing ESP32 gateway
//   - Seeed Wio-SX1262 / SX1262 radio
//   - Tank Node V1.3/V1.4 remote calibration/downlink protocol
//   - Soil Node V2.0/V2.2
//
// Radio design notes:
//   * Wio-SX1262 uses a Semtech SX1262.
//   * Wio-SX1262 uses DC-DC regulation.
//   * Wio-SX1262 uses a TCXO powered from SX1262 DIO3.
//   * With 3.3 V module supply, TCXO is configured for 3.0 V.
//   * DIO2 controls the module's TX RF switch internally.
//   * The separate Wio RF_SW input must be HIGH during RX.
//     RadioLib controls this using setRfSwitchPins().
//
// IMPORTANT:
//   The GPIO assignments below are for the existing ESP32 gateway
//   wiring plan. When the physical Wio board arrives, verify its
//   header/B2B pin mapping before applying power.
// ============================================================


// ============================================================
// INCLUDES
// ============================================================
#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <RadioLib.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <esp_system.h>
#include "GatewaySecrets.h"


// ============================================================
// PIN DEFINITIONS
// ============================================================

// ESP32 VSPI pins
#define LORA_SCK        18
#define LORA_MISO       19
#define LORA_MOSI       23

// Wio-SX1262 control pins
#define LORA_NSS        5
#define LORA_DIO1       27
#define LORA_RESET      14
#define LORA_BUSY       26

// Wio-SX1262 has a separate external RX switch input.
// HIGH = receiver path enabled.
// GPIO25 was unused in the original gateway.
#define LORA_RF_SW      25

// Flow sensors. MQTT topic names are kept for FarmPi compatibility:
// GPIO34 is the Tank 1 feed (legacy "bore" topic).
// GPIO35 is the shared Tank 2/3 feed (legacy "distribution" topic).
#define BORE_FLOW_PIN   GPIO_NUM_34
#define DIST_FLOW_PIN   GPIO_NUM_35

// Pump / status
#define PUMP_RELAY_PIN  GPIO_NUM_32
#define LED_PIN         GPIO_NUM_2


// ============================================================
// CONFIGURATION
// ============================================================
#define FW_VERSION      "1.5.2"
#define FW_BUILD        "weather-route-command-response"
#define GATEWAY_ID      "farm_gateway"

// Wi-Fi and MQTT profiles. Fill the placeholders and change enabled
// to true before the gateway is boxed. Each network can use a
// different local FarmPi address.
struct WiFiProfile {
  const char* name;
  const char* ssid;
  const char* password;
  const char* mqttServer;
  bool enabled;
};

const WiFiProfile WIFI_PROFILES[] = {

  {
    "Direct FarmPi",
    FARM_DIRECT_SSID,
    FARM_DIRECT_PASSWORD,
    FARM_DIRECT_MQTT_HOST,
    FARM_DIRECT_ENABLED
  },

  {
    "Home",
    HOME_WIFI_SSID,
    HOME_WIFI_PASSWORD,
    HOME_MQTT_HOST,
    HOME_WIFI_ENABLED
  },

  {
    "Farm 4G",
    FARM_4G_SSID,
    FARM_4G_PASSWORD,
    FARM_4G_MQTT_HOST,
    FARM_4G_ENABLED
  },

  {
    "Farm StarNet",
    FARM_STARNET_SSID,
    FARM_STARNET_PASSWORD,
    FARM_STARNET_MQTT_HOST,
    FARM_STARNET_ENABLED
  }
};

#define WIFI_PROFILE_COUNT \
  (sizeof(WIFI_PROFILES) / sizeof(WIFI_PROFILES[0]))

// MQTT
const int   MQTT_PORT     = 1883;
const char* MQTT_USER     = "";
const char* MQTT_PASS     = "";
const char* MQTT_CLIENT_PREFIX = "farm_esp32";

// MQTT topics
#define TOPIC_GATEWAY       "farm/gateway/status"
#define TOPIC_BORE_FLOW     "farm/flow/bore"
#define TOPIC_DIST_FLOW     "farm/flow/distribution"
#define TOPIC_PUMP          "farm/bore/pump"
#define TOPIC_PUMP_CMD      "farm/bore/pump/set"
#define TOPIC_NODE_CMD      "farm/node/command"
#define TOPIC_NODE_CMD_STATUS "farm/node/command/status"
#define TOPIC_NODE_CMD_RESPONSE "farm/node/command/response"
#define TOPIC_TANK_PREFIX   "farm/tank/"
#define TOPIC_SOIL_PREFIX   "farm/soil/"
#define TOPIC_WEATHER_PREFIX "farm/weather/"
#define TOPIC_ALERTS        "farm/alerts"
#define TOPIC_GATEWAY_RX_PREFIX "farm/gateway/rx/"

// Delay between delivery ACK and queued command TX.
#define ACK_COMMAND_GUARD_MS 250UL

// Flow sensor calibration
#define BORE_PULSES_PER_LITRE  21.0f
#define DIST_PULSES_PER_LITRE  450.0f

#define FLOW_CALC_INTERVAL     2000UL
#define FLOW_REPORT_INTERVAL  10000UL
#define STATUS_INTERVAL       60000UL

// Network recovery. Connections are attempted in the background so
// MQTT trouble never stops LoRa reception or flow pulse counting.
#define WIFI_CONNECT_TIMEOUT_MS        15000UL
#define WIFI_RETRY_INTERVAL_MS          5000UL
#define WIFI_NETWORK_SETTLE_MS          2000UL
#define MQTT_RETRY_INITIAL_MS           5000UL
#define MQTT_RETRY_MAX_MS              60000UL
#define MQTT_SOCKET_TIMEOUT_SECONDS        2
#define MQTT_TCP_TIMEOUT_MS             1500UL

// WiFi.status() can remain WL_CONNECTED while the ESP32 TCP/IP path is
// unusable. Three consecutive broker TCP failures are enough to prove
// that the association alone is not a useful health check. Recycle the
// Wi-Fi driver at that point instead of remaining offline for ten minutes.
#define MQTT_FAILURES_BEFORE_WIFI_RECYCLE   3

// Mosquitto closes a silent 15-second keepalive client after about
// 24 seconds. A successful TCP/MQTT handshake followed by several of
// these short sessions indicates a wedged data path, even though every
// reconnect succeeds. Recycle Wi-Fi after the third short session.
#define MQTT_STABLE_SESSION_MS          120000UL
#define MQTT_SHORT_SESSIONS_BEFORE_WIFI_RECYCLE 3

// A continuously disconnected Wi-Fi radio is restarted after ten
// minutes because field testing showed that the ESP32 driver can
// otherwise remain stuck indefinitely. A Wi-Fi-connected but unusable
// MQTT path gets the same final recovery after ten minutes. FarmPi's
// broker is supervised by systemd, so a ten-minute broker outage is no
// longer a normal transient. LoRa reports are cached before either reset.
#define WIFI_WATCHDOG_REBOOT_MS        600000UL
#define MQTT_WATCHDOG_REBOOT_MS        600000UL

// If remote pump control is ever active, fail safe to OFF when control
// connectivity has been unavailable continuously for two minutes.
#define PUMP_NETWORK_FAILSAFE_MS       120000UL

// Retry a failed radio initialisation. If every node is silent for 90
// minutes, reset and completely reinitialise the SX1262. If another full
// 90-minute reporting window passes without a packet, restart the ESP32.
// Two consecutive LoRa watchdog reboots are allowed; after that the
// gateway keeps hard-resetting only the radio to avoid an endless reboot
// loop if all field nodes are intentionally powered off.
#define LORA_INIT_RETRY_MS             300000UL
#define LORA_SILENCE_HARD_RESET_MS    5400000UL
#define LORA_POST_RESET_GRACE_MS      5400000UL
#define MAX_CONSECUTIVE_LORA_REBOOTS          2

// NVS is written only immediately before a controlled watchdog reboot.
// This preserves the newest cached report for each node without causing
// routine flash wear during ordinary MQTT outages.
#define UPLINK_CACHE_NVS_NAMESPACE       "farmgw"
#define UPLINK_CACHE_NVS_KEY             "uplinks"
#define UPLINK_CACHE_MAGIC_KEY           "cachemagic"
#define UPLINK_CACHE_MAGIC             0x46574335UL

// RTC diagnostics survive ESP.restart() but reset after full power loss.
#define RTC_DIAGNOSTIC_MAGIC           0x46574735UL

// Either tank-feed meter above this rate proves water is moving.
// Controller LED inputs will be added only after their safe isolated
// electrical interface and GPIO assignments have been confirmed.
#define FLOW_ACTIVE_THRESHOLD_LPM  0.5f

// LoRa settings: MUST match all farm nodes.
#define LORA_FREQ       915.0
#define LORA_BW         125.0
#define LORA_SF         9
#define LORA_CR         5
#define LORA_SYNC       0x12
#define LORA_PWR        22
#define LORA_PREAMBLE   8

// Seeed Wio-SX1262 hardware settings.
#define WIO_TCXO_VOLTAGE  3.0f

// Fixed command queue size is plenty for the current 7 sensor nodes.
#define MAX_PENDING_COMMANDS  10
#define MAX_TRACKED_NODES     12
#define MAX_PENDING_UPLINKS   12
#define MAX_UPLINK_TOPIC_LEN  64
#define MAX_UPLINK_PAYLOAD_LEN 640


// ============================================================
// FORWARD DECLARATIONS
// ============================================================
struct PendingUplink;

void IRAM_ATTR boreFlowISR();
void IRAM_ATTR distFlowISR();
void IRAM_ATTR loraDio1ISR();

void beginWiFiConnection();
void serviceWiFi();
void serviceNetworkHealth();
void restartForNetworkWatchdog(const char* cause, uint8_t causeCode);
void logWiFiDetails();
const WiFiProfile& activeWiFiProfile();
bool selectNextEnabledWiFiProfile();
const char* resetReasonText(esp_reset_reason_t reason);
void setupMQTT();
void serviceMQTT();
void recycleWiFiAfterMqttFault(const char* reason);
const char* mqttStateText(int state);
void mqttCallback(char* topic, byte* payload, unsigned int length);

bool initLoRa();
bool armLoRaReceive();
void checkLoRa();
void serviceLoRaHealth();
bool hardResetLoRa();
void restartForLoRaWatchdog();
void handleLoRaPacket(String& incoming, float rssi, float snr);
bool sendUplinkAck(const char* nodeId, uint32_t sequence);
bool extractUplinkSequence(JsonDocument& doc, uint32_t& sequence);
void publishRxDiagnostic(
  const char* nodeId,
  uint32_t sequence,
  bool hasSequence,
  bool ackRequested,
  bool ackSent,
  bool duplicate,
  float rssi,
  float snr
);
void publishOrQueueUplink(
  const char* nodeId,
  const String& topic,
  const String& payload
);
bool queueLatestUplink(
  const char* nodeId,
  const String& topic,
  const String& payload
);
void flushPendingUplinks();
uint8_t pendingUplinkCount();
bool persistPendingUplinks();
void restorePendingUplinks();
void clearPersistedUplinks();
bool validatePendingUplink(const PendingUplink& slot);
bool queueNodeCommand(
  const String& node,
  const String& command,
  const String& payload
);
bool sendPendingCommandFor(const char* nodeId);
void publishCommandStatus(
  const char* nodeId,
  const char* command,
  const char* status
);

void calculateFlowRates();
void reportFlow();
void setPump(bool on);
void checkTankAlerts(JsonDocument& doc);
void publishStatus();


// ============================================================
// RADIO
// ============================================================

// Explicit SPI object use keeps the radio wiring obvious.
Module loraModule(
  LORA_NSS,
  LORA_DIO1,
  LORA_RESET,
  LORA_BUSY,
  SPI
);

SX1262 radio(&loraModule);

volatile bool loraIrqFlag = false;
volatile uint32_t loraDio1Interrupts = 0;
bool radioAvailable = false;

uint32_t packetsReceived = 0;
uint32_t packetsFailed = 0;
uint32_t radioReadErrors = 0;
uint32_t downlinksSent = 0;
uint32_t downlinksFailed = 0;
uint32_t radioInitRetries = 0;
uint32_t radioHardResets = 0;
uint32_t radioHardResetFailures = 0;
uint32_t radioReceiveArms = 0;
uint32_t radioReceiveArmFailures = 0;
uint32_t loraDio1PollRecoveries = 0;
uint32_t rfSwitchStateFaults = 0;

// V1.5 uplink delivery/diagnostic counters.
uint32_t uplinkAcksSent = 0;
uint32_t uplinkAcksFailed = 0;
uint32_t duplicateUplinks = 0;
uint32_t rxDiagnosticsPublished = 0;
uint32_t rxDiagnosticsDropped = 0;

unsigned long lastLoRaPacket = 0;
unsigned long lastRadioInitAttempt = 0;
unsigned long loraHardResetAt = 0;
bool loraRecoveryPending = false;


// ============================================================
// NODE TRACKING
// ============================================================
struct NodeState {
  String id;
  uint32_t packets;
  unsigned long lastSeen;
  float lastRSSI;
  float lastSNR;

  // V1.5 delivery tracking. A node opts in by sending ack_req=true.
  bool hasDeliverySequence;
  uint32_t lastDeliverySequence;
  uint32_t duplicatePackets;

  bool used;
};

NodeState nodes[MAX_TRACKED_NODES];

struct PendingCommand {
  String node;
  String command;
  String payload;
  bool active;
};

PendingCommand pendingCommands[MAX_PENDING_COMMANDS];

struct PendingUplink {
  bool active;
  char nodeId[24];
  char topic[MAX_UPLINK_TOPIC_LEN];
  char payload[MAX_UPLINK_PAYLOAD_LEN];
  unsigned long queuedAt;
};

PendingUplink pendingUplinks[MAX_PENDING_UPLINKS];

uint32_t uplinksCached = 0;
uint32_t uplinksFlushed = 0;
uint32_t uplinksDropped = 0;
uint32_t uplinksRestored = 0;
uint32_t cachePersistFailures = 0;
bool persistedUplinkCachePresent = false;


// ============================================================
// FLOW METERS
// ============================================================
volatile uint32_t borePulseCount = 0;
volatile uint32_t distPulseCount = 0;

// ESP32-safe critical section for ISR/shared counters.
portMUX_TYPE flowMux = portMUX_INITIALIZER_UNLOCKED;

float totalBoreLitres = 0.0f;
float totalDistLitres = 0.0f;
float currentBoreFlowLPM = 0.0f;
float currentDistFlowLPM = 0.0f;

unsigned long lastFlowCalc = 0;
unsigned long lastFlowReport = 0;


// ============================================================
// PUMP / GATEWAY DIAGNOSTICS
// ============================================================
bool pumpRunning = false;

uint32_t mqttReconnects = 0;
uint32_t mqttLoopFailures = 0;
uint32_t mqttShortSessions = 0;
uint32_t wifiReconnects = 0;
uint32_t mqttFailures = 0;
uint32_t wifiFailures = 0;
uint32_t wifiRecycles = 0;
uint32_t pumpFailsafeTrips = 0;

bool wifiConnecting = false;
bool wifiWasConnected = false;
bool wifiEverConnected = false;
bool wifiOutageActive = false;
bool mqttOutageActive = false;
bool mqttSessionActive = false;
bool bootWasNetworkWatchdog = false;
bool bootWasLoRaWatchdog = false;
uint8_t bootNetworkWatchdogCause = 0;
uint8_t consecutiveMqttFailures = 0;
uint8_t consecutiveShortMqttSessions = 0;
uint8_t currentWiFiProfile = 0;

unsigned long wifiAttemptStarted = 0;
unsigned long wifiConnectedAt = 0;
unsigned long lastWifiAttempt = 0;
unsigned long lastMqttAttempt = 0;
unsigned long wifiOutageStarted = 0;
unsigned long mqttOutageStarted = 0;
unsigned long mqttSessionStartedAt = 0;
unsigned long mqttRetryIntervalMs = MQTT_RETRY_INITIAL_MS;

uint32_t lastWifiOutageSeconds = 0;
uint32_t longestWifiOutageSeconds = 0;
uint32_t lastMqttOutageSeconds = 0;
uint32_t longestMqttOutageSeconds = 0;

esp_reset_reason_t bootResetReason = ESP_RST_UNKNOWN;

RTC_DATA_ATTR uint32_t rtcDiagnosticMagic = 0;
RTC_DATA_ATTR uint32_t rtcBootCount = 0;
RTC_DATA_ATTR uint32_t rtcNetworkWatchdogReboots = 0;
RTC_DATA_ATTR uint32_t rtcLoRaWatchdogReboots = 0;
RTC_DATA_ATTR uint8_t rtcConsecutiveLoRaWatchdogReboots = 0;
RTC_DATA_ATTR uint32_t rtcWatchdogPendingMarker = 0;
RTC_DATA_ATTR uint8_t rtcNetworkWatchdogCause = 0;

char mqttClientId[32] = {0};

unsigned long lastStatusPublish = 0;
unsigned long bootTime = 0;
unsigned long lastFlowAlert = 0;


// ============================================================
// MQTT CLIENT
// ============================================================
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);


// ============================================================
// SETUP
// ============================================================
void setup() {

  Serial.begin(115200);
  delay(200);

  bootResetReason = esp_reset_reason();

  if (rtcDiagnosticMagic != RTC_DIAGNOSTIC_MAGIC) {
    rtcDiagnosticMagic = RTC_DIAGNOSTIC_MAGIC;
    rtcBootCount = 0;
    rtcNetworkWatchdogReboots = 0;
    rtcLoRaWatchdogReboots = 0;
    rtcConsecutiveLoRaWatchdogReboots = 0;
    rtcWatchdogPendingMarker = 0;
    rtcNetworkWatchdogCause = 0;
  }

  bootWasNetworkWatchdog =
    rtcWatchdogPendingMarker == RTC_DIAGNOSTIC_MAGIC &&
    (
      rtcNetworkWatchdogCause == 1 ||
      rtcNetworkWatchdogCause == 2
    );

  bootWasLoRaWatchdog =
    rtcWatchdogPendingMarker == RTC_DIAGNOSTIC_MAGIC &&
    rtcNetworkWatchdogCause == 3;

  bootNetworkWatchdogCause =
    (bootWasNetworkWatchdog || bootWasLoRaWatchdog)
      ? rtcNetworkWatchdogCause
      : 0;

  rtcWatchdogPendingMarker = 0;
  rtcNetworkWatchdogCause = 0;
  rtcBootCount++;

  Serial.println();
  Serial.println("==================================");
  Serial.println("ESP32 FARM GATEWAY");
  Serial.print("FW: ");
  Serial.println(FW_VERSION);
  Serial.print("BUILD: ");
  Serial.println(FW_BUILD);
  Serial.print("RESET: ");
  Serial.println(resetReasonText(bootResetReason));
  Serial.print("BOOT COUNT: ");
  Serial.println(rtcBootCount);
  Serial.print("NETWORK WATCHDOG REBOOTS: ");
  Serial.println(rtcNetworkWatchdogReboots);
  Serial.print("LORA WATCHDOG REBOOTS: ");
  Serial.println(rtcLoRaWatchdogReboots);
  Serial.println("RADIO: Seeed Wio-SX1262");
  Serial.println("==================================");

  pinMode(LED_PIN, OUTPUT);
  pinMode(PUMP_RELAY_PIN, OUTPUT);
  digitalWrite(PUMP_RELAY_PIN, LOW);

  // GPIO34/35 on the classic ESP32 do NOT have internal pull-ups.
  // Use external pull-ups if the flow sensors have open-collector
  // outputs and the interface board does not already provide them.
  pinMode(BORE_FLOW_PIN, INPUT);
  pinMode(DIST_FLOW_PIN, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(BORE_FLOW_PIN),
    boreFlowISR,
    FALLING
  );

  attachInterrupt(
    digitalPinToInterrupt(DIST_FLOW_PIN),
    distFlowISR,
    FALLING
  );

  bootTime = millis();

  setupMQTT();
  restorePendingUplinks();

  // Start the radio before any network connection attempt. Sensor
  // traffic continues to be received even when Wi-Fi/MQTT is down.
  radioAvailable = initLoRa();
  lastRadioInitAttempt = millis();

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(false);

  beginWiFiConnection();

  Serial.println("[ESP32] Setup complete");
}


// ============================================================
// MAIN LOOP
// ============================================================
void loop() {

  // Process LoRa IRQ as early as possible. This matters because
  // V1.3 sensor nodes listen for a gateway reply for only ~3 seconds
  // immediately after their uplink.
  checkLoRa();

  serviceWiFi();
  serviceMQTT();
  serviceNetworkHealth();
  serviceLoRaHealth();

  // A TCP attempt is bounded to 1.5 seconds. Check the radio again
  // immediately in case an uplink arrived during that attempt.
  checkLoRa();

  unsigned long now = millis();

  if (now - lastFlowCalc >= FLOW_CALC_INTERVAL) {
    calculateFlowRates();
    lastFlowCalc = now;
  }

  if (now - lastFlowReport >= FLOW_REPORT_INTERVAL) {
    reportFlow();
    lastFlowReport = now;
  }

  if (now - lastStatusPublish >= STATUS_INTERVAL) {
    publishStatus();
    lastStatusPublish = now;
  }

  // Yield to the ESP32 system tasks and the Arduino core watchdog.
  delay(1);
}


// ============================================================
// INTERRUPTS
// ============================================================
void IRAM_ATTR boreFlowISR() {
  portENTER_CRITICAL_ISR(&flowMux);
  borePulseCount++;
  portEXIT_CRITICAL_ISR(&flowMux);
}

void IRAM_ATTR distFlowISR() {
  portENTER_CRITICAL_ISR(&flowMux);
  distPulseCount++;
  portEXIT_CRITICAL_ISR(&flowMux);
}

void IRAM_ATTR loraDio1ISR() {
  loraDio1Interrupts++;
  loraIrqFlag = true;
}


// ============================================================
// FLOW
// ============================================================
void calculateFlowRates() {

  uint32_t borePulses;
  uint32_t distPulses;

  portENTER_CRITICAL(&flowMux);
  borePulses = borePulseCount;
  distPulses = distPulseCount;
  borePulseCount = 0;
  distPulseCount = 0;
  portEXIT_CRITICAL(&flowMux);

  float boreLitres = borePulses / BORE_PULSES_PER_LITRE;
  float distLitres = distPulses / DIST_PULSES_PER_LITRE;

  totalBoreLitres += boreLitres;
  totalDistLitres += distLitres;

  float intervalMin = FLOW_CALC_INTERVAL / 60000.0f;

  currentBoreFlowLPM =
    (intervalMin > 0.0f) ? boreLitres / intervalMin : 0.0f;

  currentDistFlowLPM =
    (intervalMin > 0.0f) ? distLitres / intervalMin : 0.0f;
}


void reportFlow() {

  if (!mqtt.connected())
    return;

  StaticJsonDocument<256> boreDoc;

  boreDoc["bore_lpm"] =
    round(currentBoreFlowLPM * 10.0f) / 10.0f;

  boreDoc["bore_total"] =
    round(totalBoreLitres * 10.0f) / 10.0f;

  // Clear aliases describe the actual planned pipework while the
  // original keys remain unchanged for the existing FarmPi code.
  boreDoc["tank1_lpm"] =
    round(currentBoreFlowLPM * 10.0f) / 10.0f;

  boreDoc["tank1_total"] =
    round(totalBoreLitres * 10.0f) / 10.0f;

  boreDoc["pump"] =
    pumpRunning ? "on" : "off";

  boreDoc["pump_source"] =
    "gateway_relay";

  boreDoc["water_flowing"] =
    (
      currentBoreFlowLPM >= FLOW_ACTIVE_THRESHOLD_LPM ||
      currentDistFlowLPM >= FLOW_ACTIVE_THRESHOLD_LPM
    );

  String borePayload;
  serializeJson(boreDoc, borePayload);

  mqtt.publish(
    TOPIC_BORE_FLOW,
    borePayload.c_str(),
    true
  );

  StaticJsonDocument<224> distDoc;

  distDoc["dist_lpm"] =
    round(currentDistFlowLPM * 10.0f) / 10.0f;

  distDoc["dist_total"] =
    round(totalDistLitres * 10.0f) / 10.0f;

  distDoc["tank23_lpm"] =
    round(currentDistFlowLPM * 10.0f) / 10.0f;

  distDoc["tank23_total"] =
    round(totalDistLitres * 10.0f) / 10.0f;

  distDoc["water_flowing"] =
    (
      currentBoreFlowLPM >= FLOW_ACTIVE_THRESHOLD_LPM ||
      currentDistFlowLPM >= FLOW_ACTIVE_THRESHOLD_LPM
    );

  String distPayload;
  serializeJson(distDoc, distPayload);

  mqtt.publish(
    TOPIC_DIST_FLOW,
    distPayload.c_str(),
    true
  );

  Serial.print("[FLOW] Tank 1: ");
  Serial.print(borePayload);
  Serial.print(" | Tank 2/3: ");
  Serial.println(distPayload);

  if (
    pumpRunning &&
    currentBoreFlowLPM < FLOW_ACTIVE_THRESHOLD_LPM &&
    currentDistFlowLPM < FLOW_ACTIVE_THRESHOLD_LPM &&
    millis() - lastFlowAlert > 300000UL
  ) {

    lastFlowAlert = millis();

    Serial.println(
      "[ALERT] Pump running but bore flow low!"
    );

    mqtt.publish(
      TOPIC_ALERTS,
      "{\"alert\":\"bore_no_flow\"}"
    );
  }
}


// ============================================================
// PUMP
// ============================================================
void setPump(bool on) {

  pumpRunning = on;

  digitalWrite(
    PUMP_RELAY_PIN,
    on ? HIGH : LOW
  );

  StaticJsonDocument<64> doc;
  doc["pump"] = on ? "on" : "off";

  String payload;
  serializeJson(doc, payload);

  if (mqtt.connected()) {
    mqtt.publish(
      TOPIC_PUMP,
      payload.c_str(),
      true
    );
  }

  Serial.print("[PUMP] ");
  Serial.println(on ? "ON" : "OFF");
}


// ============================================================
// WIO-SX1262 RADIO INITIALISATION
// ============================================================
bool initLoRa() {

  Serial.println("[LoRa] Initialising Seeed Wio-SX1262...");

  Serial.print("[LoRa] SPI pins SCK=");
  Serial.print(LORA_SCK);
  Serial.print(" MISO=");
  Serial.print(LORA_MISO);
  Serial.print(" MOSI=");
  Serial.println(LORA_MOSI);

  Serial.print("[LoRa] NSS=");
  Serial.print(LORA_NSS);
  Serial.print(" DIO1=");
  Serial.print(LORA_DIO1);
  Serial.print(" RESET=");
  Serial.print(LORA_RESET);
  Serial.print(" BUSY=");
  Serial.print(LORA_BUSY);
  Serial.print(" RF_SW=");
  Serial.println(LORA_RF_SW);

  Serial.print("[LoRa] Params: ");
  Serial.print(LORA_FREQ, 1);
  Serial.print(" MHz, BW ");
  Serial.print(LORA_BW, 1);
  Serial.print(" kHz, SF");
  Serial.print(LORA_SF);
  Serial.print(", CR 4/");
  Serial.print(LORA_CR);
  Serial.print(", sync 0x");
  Serial.println(LORA_SYNC, HEX);

  SPI.begin(
    LORA_SCK,
    LORA_MISO,
    LORA_MOSI,
    LORA_NSS
  );

  // Register the external RX-enable line before initialising the radio.
  // This makes every RadioLib mode transition, including those performed
  // during recovery, use the Wio module's complete RF-switch truth table.
  radio.setRfSwitchPins(
    LORA_RF_SW,
    RADIOLIB_NC
  );

  // Keep the receive path disabled until startReceive() has configured
  // the SX1262. RadioLib will drive this HIGH when entering RX.
  digitalWrite(LORA_RF_SW, LOW);

  // Seeed documentation specifies:
  // - SX1262
  // - DC-DC supply
  // - DIO3-powered active TCXO
  //
  // RadioLib begin args after preamble:
  //   tcxoVoltage = 3.0 V
  //   useRegulatorLDO = false -> use DC-DC
  int state = radio.begin(
    LORA_FREQ,
    LORA_BW,
    LORA_SF,
    LORA_CR,
    LORA_SYNC,
    LORA_PWR,
    LORA_PREAMBLE,
    WIO_TCXO_VOLTAGE,
    false
  );

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("[LoRa] INIT FAILED: ");
    Serial.println(state);
    return false;
  }

  // The Wio module uses TWO RF switching controls:
  //
  // 1. SX1262 DIO2 is internally connected to the TX RF switch.
  //    This is also RadioLib's normal SX126x behaviour, but make
  //    it explicit here so the hardware intent is obvious.
  state = radio.setDio2AsRfSwitch(true);

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("[LoRa] DIO2 RF switch config failed: ");
    Serial.println(state);
    return false;
  }

  // 2. Wio RF_SW was registered above as the external RX enable.
  //    There is no external TX_EN pin because DIO2 handles TX.

  radio.setDio1Action(loraDio1ISR);

  if (!armLoRaReceive()) {
    return false;
  }

  Serial.println("[LoRa] Wio-SX1262 OK - continuous RX armed");

  return true;
}


// ============================================================
// ARM RECEIVER
// ============================================================
bool armLoRaReceive() {

  loraIrqFlag = false;

  int state = radio.startReceive();

  if (state != RADIOLIB_ERR_NONE) {
    radioReceiveArmFailures++;
    radioAvailable = false;
    Serial.print("[LoRa] startReceive failed: ");
    Serial.println(state);
    return false;
  }

  radioReceiveArms++;

  // A successful SPI command does not prove that the external antenna
  // path was selected. Verify the Wio RX-enable line after RadioLib has
  // entered RX and repair it defensively if an older library/core left
  // the GPIO low.
  delayMicroseconds(20);

  if (digitalRead(LORA_RF_SW) != HIGH) {
    rfSwitchStateFaults++;
    digitalWrite(LORA_RF_SW, HIGH);
    Serial.println("[LoRa] RF_SW was LOW after startReceive - forced HIGH");
  }

  return true;
}


// ============================================================
// LORA RECEIVE
// ============================================================
void checkLoRa() {

  if (!radioAvailable)
    return;

  if (!loraIrqFlag) {
    // DIO1 remains asserted until RadioLib clears the IRQ. Polling the
    // level as well as using the edge ISR prevents one missed interrupt
    // (for example during a long network operation) from hiding a packet
    // forever and leaving all receive/error counters at zero.
    if (digitalRead(LORA_DIO1) != HIGH)
      return;

    loraDio1PollRecoveries++;
    Serial.println("[LoRa] Recovered latched DIO1 event by polling");
  }

  loraIrqFlag = false;

  String incoming;

  int state = radio.readData(incoming);

  if (state != RADIOLIB_ERR_NONE) {

    radioReadErrors++;

    Serial.print("[LoRa] RX read error: ");
    Serial.println(state);

    armLoRaReceive();
    return;
  }

  float rssi = radio.getRSSI();
  float snr = radio.getSNR();

  Serial.print("[LoRa] RX ");
  Serial.print(rssi, 1);
  Serial.print(" dBm / ");
  Serial.print(snr, 1);
  Serial.print(" dB: ");
  Serial.println(incoming);

  handleLoRaPacket(
    incoming,
    rssi,
    snr
  );
}


void serviceLoRaHealth() {

  unsigned long now = millis();

  if (!radioAvailable) {

    if (
      now - lastRadioInitAttempt <
      LORA_INIT_RETRY_MS
    ) {
      return;
    }

    lastRadioInitAttempt = now;
    radioInitRetries++;

    Serial.println("[LoRa] Retrying radio initialisation");
    radioAvailable = initLoRa();

    if (radioAvailable && loraRecoveryPending) {
      // Give the successfully reinitialised receiver a complete node
      // reporting window before considering an ESP restart.
      loraHardResetAt = now;
    }

    return;
  }

  // Never disturb a packet whose DIO1 interrupt is already pending.
  if (loraIrqFlag)
    return;

  if (loraRecoveryPending) {

    if (
      now - loraHardResetAt <
      LORA_POST_RESET_GRACE_MS
    ) {
      return;
    }

    if (
      rtcConsecutiveLoRaWatchdogReboots <
      MAX_CONSECUTIVE_LORA_REBOOTS
    ) {
      restartForLoRaWatchdog();
      return;
    }

    // All nodes may genuinely be powered off. Continue recovering the
    // SX1262 every reporting window, but stop rebooting the whole gateway.
    Serial.println(
      "[LoRa] ESP restart limit reached - resetting radio only"
    );

    loraHardResetAt = now;
    hardResetLoRa();
    return;
  }

  unsigned long receiveReference =
    lastLoRaPacket == 0 ? bootTime : lastLoRaPacket;

  if (
    now - receiveReference <
    LORA_SILENCE_HARD_RESET_MS
  ) {
    return;
  }

  loraRecoveryPending = true;
  loraHardResetAt = now;

  hardResetLoRa();
}


bool hardResetLoRa() {

  radioHardResets++;

  Serial.println(
    "[LoRa] No packets for 90 minutes - hard reset and full reinitialisation"
  );

  loraIrqFlag = false;
  radioAvailable = false;

  int resetState = radio.reset(true);

  if (resetState != RADIOLIB_ERR_NONE) {
    Serial.print("[LoRa] Hardware reset verification failed: ");
    Serial.println(resetState);
  }

  delay(20);

  radioAvailable = initLoRa();
  lastRadioInitAttempt = millis();

  if (!radioAvailable) {
    radioHardResetFailures++;
    Serial.println("[LoRa] Full radio recovery failed");
    return false;
  }

  Serial.println("[LoRa] Full radio recovery completed");
  return true;
}


// ============================================================
// HANDLE NODE PACKET
// ============================================================
void handleLoRaPacket(
  String& incoming,
  float rssi,
  float snr
) {

  StaticJsonDocument<640> doc;

  DeserializationError err =
    deserializeJson(doc, incoming);

  if (err) {

    packetsFailed++;

    Serial.print("[LoRa] JSON parse error: ");
    Serial.println(err.c_str());

    armLoRaReceive();
    return;
  }

  const char* nodeId = doc["node"];

  if (
    !doc["node"].is<const char*>() ||
    nodeId == nullptr ||
    nodeId[0] == '\0'
  ) {

    packetsFailed++;

    Serial.println(
      "[LoRa] Missing or invalid node ID"
    );

    armLoRaReceive();
    return;
  }

  packetsReceived++;
  lastLoRaPacket = millis();

  // Any valid node packet proves that LoRa reception has recovered.
  loraRecoveryPending = false;
  loraHardResetAt = 0;
  rtcConsecutiveLoRaWatchdogReboots = 0;

  // Existing command acknowledgements from nodes use the "ack" field.
  // Delivery ACK requests deliberately use "ack_req" so both protocols
  // can coexist without ambiguity.
  bool isCommandAck =
    doc["ack"].is<const char*>();

  bool ackRequested =
    !isCommandAck &&
    (doc["ack_req"] | false);

  uint32_t deliverySequence = 0;
  bool hasDeliverySequence =
    ackRequested &&
    extractUplinkSequence(doc, deliverySequence);

  // Update compact node-health table and determine whether this packet is
  // a retry of the last sequence already accepted from this node.
  bool found = false;
  bool duplicate = false;
  int8_t nodeIndex = -1;

  for (uint8_t i = 0; i < MAX_TRACKED_NODES; i++) {

    if (nodes[i].used && nodes[i].id == nodeId) {

      nodeIndex = i;
      nodes[i].packets++;
      nodes[i].lastSeen = millis();
      nodes[i].lastRSSI = rssi;
      nodes[i].lastSNR = snr;

      if (
        hasDeliverySequence &&
        nodes[i].hasDeliverySequence &&
        nodes[i].lastDeliverySequence == deliverySequence
      ) {
        duplicate = true;
        nodes[i].duplicatePackets++;
        duplicateUplinks++;
      }

      found = true;
      break;
    }
  }

  if (!found) {

    for (uint8_t i = 0; i < MAX_TRACKED_NODES; i++) {

      if (!nodes[i].used) {

        nodes[i].used = true;
        nodes[i].id = nodeId;
        nodes[i].packets = 1;
        nodes[i].lastSeen = millis();
        nodes[i].lastRSSI = rssi;
        nodes[i].lastSNR = snr;
        nodes[i].hasDeliverySequence = false;
        nodes[i].lastDeliverySequence = 0;
        nodes[i].duplicatePackets = 0;

        nodeIndex = i;
        break;
      }
    }
  }

  // V1.5 reliability handshake:
  // ACK the radio delivery BEFORE MQTT work. Legacy nodes omit ack_req and
  // therefore continue to see exactly the V1.4 behaviour.
  bool ackSent = false;

  if (ackRequested) {

    if (!hasDeliverySequence) {
      Serial.print("[ACK] ");
      Serial.print(nodeId);
      Serial.println(" requested ACK but sent no seq/cycle");
    } else {
      ackSent = sendUplinkAck(nodeId, deliverySequence);
    }
  }

  // Existing queued commands are still delivered in the same wake window.
  // Updated nodes should remain in RX briefly after receiving the delivery
  // ACK so a command can follow it.
  bool commandSent = false;

  if (!isCommandAck) {

    // Updated V1.4/V2.2 nodes switch from the immediate ACK receive
    // state into their normal command receive window after accepting
    // the delivery ACK. Give that transition a small guard period
    // before transmitting any queued command.
    if (ackRequested && ackSent) {
      delay(ACK_COMMAND_GUARD_MS);
    }

    commandSent = sendPendingCommandFor(nodeId);
  }

  if (!commandSent) {
    armLoRaReceive();
  }

  // Commit the accepted sequence only after the ACK opportunity has been
  // handled. A repeated sequence is intentionally NOT committed again.
  if (
    hasDeliverySequence &&
    !duplicate &&
    nodeIndex >= 0
  ) {
    nodes[nodeIndex].hasDeliverySequence = true;
    nodes[nodeIndex].lastDeliverySequence = deliverySequence;
  }

  // Publish a lightweight diagnostic for every valid reception, including
  // retries. This provides a separate view of radio delivery quality.
  publishRxDiagnostic(
    nodeId,
    deliverySequence,
    hasDeliverySequence,
    ackRequested,
    ackSent,
    duplicate,
    rssi,
    snr
  );

  // Radio timing is safe now, so report existing command ACKs to MQTT.
  if (isCommandAck) {

    const char* ackCommand =
      doc["ack"] | "unknown";

    const char* ackStatus =
      doc["ack_status"] | "received";

    publishCommandStatus(
      nodeId,
      ackCommand,
      ackStatus
    );

    // Preserve the complete node response for calibration/diagnostic tools
    // without forcing them to read a retained sensor-state topic. Soil
    // command responses are often intentionally partial, so publishing them
    // on farm/soil/<node> would replace the last complete soil reading.
    if (mqtt.connected()) {
      doc["gateway_rssi"] =
        round(rssi * 10.0f) / 10.0f;

      doc["gateway_snr"] =
        round(snr * 10.0f) / 10.0f;

      String responsePayload;
      serializeJson(doc, responsePayload);

      mqtt.publish(
        TOPIC_NODE_CMD_RESPONSE,
        responsePayload.c_str(),
        false
      );
    }

    // Tank command acknowledgements deliberately contain a complete,
    // recalculated tank reading and may continue refreshing the retained tank
    // topic. Soil command acknowledgements are partial and must not replace
    // retained sensor telemetry.
    if (strncmp(nodeId, "soil_", 5) == 0) {
      return;
    }
  }

  // A retry carries the same sensor reading that FarmPi has already seen.
  // ACK it again so the node can stop retrying, but do not publish it as a
  // new reading or run tank alerts a second time.
  if (duplicate) {
    Serial.print("[ACK] Duplicate uplink suppressed: ");
    Serial.print(nodeId);
    Serial.print(" seq=");
    Serial.println(deliverySequence);
    return;
  }

  // Gateway metadata is appended after the time-critical
  // downlink opportunity has been handled.
  doc["gateway_rssi"] =
    round(rssi * 10.0f) / 10.0f;

  doc["gateway_snr"] =
    round(snr * 10.0f) / 10.0f;

  // Let FarmPi see whether this report used the delivery protocol.
  if (ackRequested) {
    doc["gateway_ack"] = ackSent;
  }

  String enriched;
  serializeJson(doc, enriched);

  String topic;

  if (strncmp(nodeId, "soil_", 5) == 0) {

    topic =
      String(TOPIC_SOIL_PREFIX) + nodeId;

  } else if (strncmp(nodeId, "tank_", 5) == 0) {

    topic =
      String(TOPIC_TANK_PREFIX) + nodeId;

  } else if (strncmp(nodeId, "weather_", 8) == 0) {

    topic =
      String(TOPIC_WEATHER_PREFIX) + nodeId;

  } else {

    topic =
      "farm/unknown/" + String(nodeId);
  }

  publishOrQueueUplink(
    nodeId,
    topic,
    enriched
  );

  if (
    !isCommandAck &&
    strncmp(nodeId, "tank_", 5) == 0
  ) {
    checkTankAlerts(doc);
  }
}


// ============================================================
// V1.5 UPLINK DELIVERY ACK / RX DIAGNOSTICS
// ============================================================
bool extractUplinkSequence(
  JsonDocument& doc,
  uint32_t& sequence
) {

  // Tanks already use "seq". Soil nodes currently use "cycle".
  // Future nodes may standardise on tx_seq; accept all three.
  if (doc["tx_seq"].is<uint32_t>()) {
    sequence = doc["tx_seq"].as<uint32_t>();
    return true;
  }

  if (doc["seq"].is<uint32_t>()) {
    sequence = doc["seq"].as<uint32_t>();
    return true;
  }

  if (doc["cycle"].is<uint32_t>()) {
    sequence = doc["cycle"].as<uint32_t>();
    return true;
  }

  return false;
}


bool sendUplinkAck(
  const char* nodeId,
  uint32_t sequence
) {

  StaticJsonDocument<128> ackDoc;

  ackDoc["ack"] = "uplink";
  ackDoc["node"] = nodeId;
  ackDoc["seq"] = sequence;

  String payload;
  serializeJson(ackDoc, payload);

  Serial.print("[ACK] -> ");
  Serial.print(nodeId);
  Serial.print(" seq=");
  Serial.println(sequence);

  int state =
    radio.transmit(payload);

  // Return to RX immediately so a retry, command ACK, or another node can
  // be heard with minimum dead time.
  armLoRaReceive();

  if (state == RADIOLIB_ERR_NONE) {
    uplinkAcksSent++;
    return true;
  }

  uplinkAcksFailed++;

  Serial.print("[ACK] TX FAIL: ");
  Serial.println(state);

  return false;
}


void publishRxDiagnostic(
  const char* nodeId,
  uint32_t sequence,
  bool hasSequence,
  bool ackRequested,
  bool ackSent,
  bool duplicate,
  float rssi,
  float snr
) {

  if (!mqtt.connected()) {
    rxDiagnosticsDropped++;
    return;
  }

  StaticJsonDocument<256> diag;

  diag["node"] = nodeId;

  if (hasSequence)
    diag["seq"] = sequence;

  diag["ack_requested"] = ackRequested;
  diag["ack_sent"] = ackSent;
  diag["duplicate"] = duplicate;
  diag["rssi"] = round(rssi * 10.0f) / 10.0f;
  diag["snr"] = round(snr * 10.0f) / 10.0f;
  diag["gateway_uptime_ms"] = millis();

  String payload;
  serializeJson(diag, payload);

  String topic =
    String(TOPIC_GATEWAY_RX_PREFIX) + nodeId;

  if (
    mqtt.publish(
      topic.c_str(),
      payload.c_str(),
      false
    )
  ) {
    rxDiagnosticsPublished++;
  } else {
    rxDiagnosticsDropped++;
  }
}


// ============================================================
// OFFLINE UPLINK CACHE
//
// Keep only the newest unsent report for each node. This bounds RAM
// use and ensures FarmPi receives current state after MQTT recovers.
// ============================================================
uint8_t pendingUplinkCount() {

  uint8_t count = 0;

  for (uint8_t i = 0; i < MAX_PENDING_UPLINKS; i++) {
    if (pendingUplinks[i].active)
      count++;
  }

  return count;
}


bool validatePendingUplink(const PendingUplink& slot) {

  if (!slot.active)
    return true;

  return
    slot.nodeId[0] != '\0' &&
    slot.topic[0] != '\0' &&
    slot.payload[0] != '\0' &&
    strnlen(slot.nodeId, sizeof(slot.nodeId)) < sizeof(slot.nodeId) &&
    strnlen(slot.topic, sizeof(slot.topic)) < sizeof(slot.topic) &&
    strnlen(slot.payload, sizeof(slot.payload)) < sizeof(slot.payload);
}


void clearPersistedUplinks() {

  Preferences preferences;

  if (!preferences.begin(UPLINK_CACHE_NVS_NAMESPACE, false))
    return;

  preferences.remove(UPLINK_CACHE_MAGIC_KEY);
  preferences.remove(UPLINK_CACHE_NVS_KEY);
  preferences.end();

  persistedUplinkCachePresent = false;
}


bool persistPendingUplinks() {

  uint8_t count = pendingUplinkCount();

  if (count == 0) {
    clearPersistedUplinks();
    return true;
  }

  Preferences preferences;

  if (!preferences.begin(UPLINK_CACHE_NVS_NAMESPACE, false)) {
    cachePersistFailures++;
    Serial.println("[UPLINK] NVS open failed");
    return false;
  }

  // Invalidate the old record before replacing its payload. The magic
  // is written last so a power interruption cannot validate half a save.
  preferences.remove(UPLINK_CACHE_MAGIC_KEY);

  size_t written = preferences.putBytes(
    UPLINK_CACHE_NVS_KEY,
    pendingUplinks,
    sizeof(pendingUplinks)
  );

  size_t magicWritten = preferences.putUInt(
    UPLINK_CACHE_MAGIC_KEY,
    UPLINK_CACHE_MAGIC
  );

  preferences.end();

  if (
    written != sizeof(pendingUplinks) ||
    magicWritten != sizeof(uint32_t)
  ) {
    cachePersistFailures++;
    Serial.println("[UPLINK] NVS save failed");
    return false;
  }

  persistedUplinkCachePresent = true;

  Serial.print("[UPLINK] Preserved for restart: ");
  Serial.println(count);

  return true;
}


void restorePendingUplinks() {

  Preferences preferences;

  if (!preferences.begin(UPLINK_CACHE_NVS_NAMESPACE, true))
    return;

  uint32_t magic = preferences.getUInt(
    UPLINK_CACHE_MAGIC_KEY,
    0
  );

  size_t storedLength = preferences.getBytesLength(
    UPLINK_CACHE_NVS_KEY
  );

  if (
    magic != UPLINK_CACHE_MAGIC ||
    storedLength != sizeof(pendingUplinks)
  ) {
    preferences.end();
    return;
  }

  size_t loaded = preferences.getBytes(
    UPLINK_CACHE_NVS_KEY,
    pendingUplinks,
    sizeof(pendingUplinks)
  );

  preferences.end();

  bool valid = loaded == sizeof(pendingUplinks);
  uint8_t restored = 0;

  for (uint8_t i = 0; i < MAX_PENDING_UPLINKS; i++) {
    if (!validatePendingUplink(pendingUplinks[i])) {
      valid = false;
      break;
    }

    if (pendingUplinks[i].active) {
      pendingUplinks[i].queuedAt = millis();
      restored++;
    }
  }

  if (!valid) {
    memset(pendingUplinks, 0, sizeof(pendingUplinks));
    clearPersistedUplinks();
    cachePersistFailures++;
    Serial.println("[UPLINK] Discarded invalid restart cache");
    return;
  }

  persistedUplinkCachePresent = restored > 0;
  uplinksRestored = restored;

  Serial.print("[UPLINK] Restored after restart: ");
  Serial.println(restored);
}


bool queueLatestUplink(
  const char* nodeId,
  const String& topic,
  const String& payload
) {

  if (
    strlen(nodeId) >= sizeof(pendingUplinks[0].nodeId) ||
    topic.length() >= sizeof(pendingUplinks[0].topic) ||
    payload.length() >= sizeof(pendingUplinks[0].payload)
  ) {
    uplinksDropped++;
    Serial.println("[UPLINK] Drop: packet exceeds cache size");
    return false;
  }

  int target = -1;
  bool replacing = false;

  for (uint8_t i = 0; i < MAX_PENDING_UPLINKS; i++) {
    if (
      pendingUplinks[i].active &&
      strcmp(pendingUplinks[i].nodeId, nodeId) == 0
    ) {
      target = i;
      replacing = true;
      break;
    }
  }

  if (target < 0) {
    for (uint8_t i = 0; i < MAX_PENDING_UPLINKS; i++) {
      if (!pendingUplinks[i].active) {
        target = i;
        break;
      }
    }
  }

  if (target < 0) {
    uplinksDropped++;
    Serial.println("[UPLINK] Drop: offline cache full");
    return false;
  }

  PendingUplink& slot = pendingUplinks[target];

  strlcpy(slot.nodeId, nodeId, sizeof(slot.nodeId));
  strlcpy(slot.topic, topic.c_str(), sizeof(slot.topic));
  strlcpy(slot.payload, payload.c_str(), sizeof(slot.payload));
  slot.queuedAt = millis();
  slot.active = true;

  uplinksCached++;

  Serial.print(
    replacing
      ? "[UPLINK] Updated cached report for "
      : "[UPLINK] Cached report for "
  );
  Serial.println(nodeId);

  return true;
}


void publishOrQueueUplink(
  const char* nodeId,
  const String& topic,
  const String& payload
) {

  if (
    mqtt.connected() &&
    mqtt.publish(
      topic.c_str(),
      payload.c_str(),
      true
    )
  ) {
    Serial.print("[MQTT] -> ");
    Serial.println(topic);
    return;
  }

  queueLatestUplink(
    nodeId,
    topic,
    payload
  );
}


void flushPendingUplinks() {

  if (!mqtt.connected())
    return;

  for (uint8_t i = 0; i < MAX_PENDING_UPLINKS; i++) {

    PendingUplink& slot = pendingUplinks[i];

    if (!slot.active)
      continue;

    if (!mqtt.publish(slot.topic, slot.payload, true)) {
      Serial.println("[UPLINK] Flush paused: MQTT publish failed");
      return;
    }

    Serial.print("[UPLINK] Flushed cached report for ");
    Serial.println(slot.nodeId);

    slot.active = false;
    uplinksFlushed++;
  }

  if (
    persistedUplinkCachePresent &&
    pendingUplinkCount() == 0
  ) {
    clearPersistedUplinks();
    Serial.println("[UPLINK] Cleared restart cache");
  }
}


// ============================================================
// NODE COMMAND QUEUE
//
// MQTT input example:
//   topic: farm/node/command
//   payload:
//     {"node":"soil_2","cmd":"ping"}
//
// The gateway stores the command. It is NOT transmitted immediately,
// because sleeping nodes cannot hear it.
//
// When soil_2 next sends its hourly uplink, the gateway immediately
// sends:
//     {"cmd":"ping"}
//
// while soil_2 is inside its three-second downlink window.
// ============================================================
bool queueNodeCommand(
  const String& node,
  const String& command,
  const String& payload
) {

  // Replace an existing pending command for this node.
  for (uint8_t i = 0; i < MAX_PENDING_COMMANDS; i++) {

    if (
      pendingCommands[i].active &&
      pendingCommands[i].node == node
    ) {

      pendingCommands[i].command = command;
      pendingCommands[i].payload = payload;

      Serial.print("[CMD] Replaced queued command for ");
      Serial.println(node);

      return true;
    }
  }

  // Otherwise use a free slot.
  for (uint8_t i = 0; i < MAX_PENDING_COMMANDS; i++) {

    if (!pendingCommands[i].active) {

      pendingCommands[i].active = true;
      pendingCommands[i].node = node;
      pendingCommands[i].command = command;
      pendingCommands[i].payload = payload;

      Serial.print("[CMD] Queued for ");
      Serial.print(node);
      Serial.print(": ");
      Serial.println(payload);

      return true;
    }
  }

  Serial.println("[CMD] Queue full");
  return false;
}


// ============================================================
// SEND PENDING NODE COMMAND
// ============================================================
bool sendPendingCommandFor(
  const char* nodeId
) {

  for (uint8_t i = 0; i < MAX_PENDING_COMMANDS; i++) {

    if (
      pendingCommands[i].active &&
      pendingCommands[i].node == nodeId
    ) {

      String payload =
        pendingCommands[i].payload;

      Serial.print("[DOWNLINK] -> ");
      Serial.print(nodeId);
      Serial.print(": ");
      Serial.println(payload);

      int state =
        radio.transmit(payload);

      // Re-arm RX immediately. The tank can acknowledge quickly and
      // MQTT/Serial work must not consume any of that receive window.
      armLoRaReceive();

      if (state == RADIOLIB_ERR_NONE) {

        downlinksSent++;

        Serial.println("[DOWNLINK] TX OK");

        publishCommandStatus(
          nodeId,
          pendingCommands[i].command.c_str(),
          "sent"
        );

        pendingCommands[i].active = false;
        pendingCommands[i].node = "";
        pendingCommands[i].command = "";
        pendingCommands[i].payload = "";

      } else {

        downlinksFailed++;

        Serial.print("[DOWNLINK] TX FAIL: ");
        Serial.println(state);

        publishCommandStatus(
          nodeId,
          pendingCommands[i].command.c_str(),
          "radio_tx_failed"
        );

        // Keep the command queued so it can be retried next time
        // that node wakes and reports.
      }

      return true;
    }
  }

  return false;
}


// ============================================================
// COMMAND DELIVERY STATUS
// ============================================================
void publishCommandStatus(
  const char* nodeId,
  const char* command,
  const char* status
) {

  if (!mqtt.connected())
    return;

  StaticJsonDocument<192> doc;

  doc["node"] = nodeId;
  doc["cmd"] = command;
  doc["status"] = status;

  String payload;
  serializeJson(doc, payload);

  mqtt.publish(
    TOPIC_NODE_CMD_STATUS,
    payload.c_str(),
    false
  );

  Serial.print("[CMD STATUS] ");
  Serial.println(payload);
}


// ============================================================
// TANK ALERTS
// Automatic pump control remains DISABLED.
// ============================================================
void checkTankAlerts(JsonDocument& doc) {

  float pct = doc["pct"] | -1.0f;

  if (pct < 0.0f)
    return;

  if (pct < 20.0f) {

    Serial.println(
      "[ALERT] Tank low - automatic bore start disabled"
    );

    if (mqtt.connected()) {
      mqtt.publish(
        TOPIC_ALERTS,
        "{\"alert\":\"tank_low\",\"action\":\"none\"}"
      );
    }
  }

  if (pct > 95.0f) {

    Serial.println(
      "[ALERT] Tank full - automatic bore stop disabled"
    );

    if (mqtt.connected()) {
      mqtt.publish(
        TOPIC_ALERTS,
        "{\"alert\":\"tank_full\",\"action\":\"none\"}"
      );
    }
  }
}


// ============================================================
// MQTT
// ============================================================
void setupMQTT() {

  mqtt.setCallback(
    mqttCallback
  );

  // V1.4.1 status contains watchdog and outage diagnostics in addition
  // to the original gateway counters.
  mqtt.setBufferSize(2048);

  mqtt.setSocketTimeout(
    MQTT_SOCKET_TIMEOUT_SECONDS
  );

  wifiClient.setConnectionTimeout(
    MQTT_TCP_TIMEOUT_MS
  );

  uint32_t suffix =
    (uint32_t)(ESP.getEfuseMac() & 0xFFFFFFULL);

  snprintf(
    mqttClientId,
    sizeof(mqttClientId),
    "%s_%06X",
    MQTT_CLIENT_PREFIX,
    suffix
  );

  Serial.print("[MQTT] Client ID: ");
  Serial.println(mqttClientId);
}


void recycleWiFiAfterMqttFault(const char* reason) {

  Serial.print("[MQTT] ");
  Serial.print(reason);
  Serial.println(" - recycling Wi-Fi driver");

  mqtt.disconnect();
  wifiClient.stop();

  // Only the network interface is reset. The radio remains in RX and
  // any LoRa reports that cannot be published stay in the uplink cache.
  WiFi.disconnect(true, false);
  WiFi.mode(WIFI_OFF);

  wifiWasConnected = false;
  wifiConnecting = false;
  wifiConnectedAt = 0;
  lastWifiAttempt = millis();
  lastMqttAttempt = 0;
  mqttSessionActive = false;
  mqttSessionStartedAt = 0;
  consecutiveMqttFailures = 0;
  consecutiveShortMqttSessions = 0;
  mqttRetryIntervalMs = MQTT_RETRY_INITIAL_MS;
  wifiRecycles++;
}


void serviceMQTT() {

  // serviceWiFi() must first accept and log the association. This
  // prevents a TCP attempt in the brief interval where WL_CONNECTED
  // is visible immediately after WiFi.begin() but LAN bridging and
  // ARP are not yet ready.
  if (
    !wifiWasConnected ||
    WiFi.status() != WL_CONNECTED
  ) {
    return;
  }

  unsigned long now = millis();

  if (
    now - wifiConnectedAt <
    WIFI_NETWORK_SETTLE_MS
  ) {
    return;
  }

  if (mqtt.connected()) {

    if (!mqttSessionActive) {
      mqttSessionActive = true;
      mqttSessionStartedAt = now;
    }

    bool loopOk = mqtt.loop();

    if (loopOk && mqtt.connected()) {

      if (
        consecutiveShortMqttSessions > 0 &&
        now - mqttSessionStartedAt >= MQTT_STABLE_SESSION_MS
      ) {
        consecutiveShortMqttSessions = 0;
        Serial.println("[MQTT] Connection stability restored");
      }

      flushPendingUplinks();
      return;
    }

    mqttLoopFailures++;
  }

  // Count an established session only once when it drops. This catches
  // the observed cycle where CONNECT succeeds but Mosquitto receives no
  // later traffic and closes the client at the keepalive timeout.
  if (mqttSessionActive) {

    unsigned long sessionMs = now - mqttSessionStartedAt;

    mqttSessionActive = false;
    mqttSessionStartedAt = 0;

    if (sessionMs < MQTT_STABLE_SESSION_MS) {
      mqttShortSessions++;

      if (consecutiveShortMqttSessions < 255) {
        consecutiveShortMqttSessions++;
      }

      Serial.print("[MQTT] Short session ended after ");
      Serial.print(sessionMs / 1000UL);
      Serial.print(" seconds (");
      Serial.print(consecutiveShortMqttSessions);
      Serial.print("/");
      Serial.print(MQTT_SHORT_SESSIONS_BEFORE_WIFI_RECYCLE);
      Serial.println(")");

      if (
        consecutiveShortMqttSessions >=
        MQTT_SHORT_SESSIONS_BEFORE_WIFI_RECYCLE
      ) {
        recycleWiFiAfterMqttFault("Repeated short MQTT sessions");
        return;
      }

    } else {
      consecutiveShortMqttSessions = 0;
    }
  }

  if (
    lastMqttAttempt != 0 &&
    now - lastMqttAttempt < mqttRetryIntervalMs
  ) {
    return;
  }

  lastMqttAttempt = now;

  const WiFiProfile& profile =
    activeWiFiProfile();

  mqtt.setServer(
    profile.mqttServer,
    MQTT_PORT
  );

  Serial.print("[MQTT] Connecting to ");
  Serial.print(profile.mqttServer);
  Serial.print(":");
  Serial.print(MQTT_PORT);
  Serial.print(" as ");
  Serial.print(mqttClientId);
  Serial.print("... ");

  bool connected =
    mqtt.connect(
      mqttClientId,
      MQTT_USER,
      MQTT_PASS,
      TOPIC_GATEWAY,
      0,
      true,
      "{\"status\":\"offline\"}"
    );

  if (connected) {

    Serial.println("OK");

    mqtt.subscribe(TOPIC_PUMP_CMD);
    mqtt.subscribe(TOPIC_NODE_CMD);

    mqttReconnects++;
    consecutiveMqttFailures = 0;
    mqttRetryIntervalMs = MQTT_RETRY_INITIAL_MS;
    mqttSessionActive = true;
    mqttSessionStartedAt = millis();

    flushPendingUplinks();

    publishStatus();
    lastStatusPublish = millis();

    return;
  }

  int state = mqtt.state();

  mqttFailures++;
  if (consecutiveMqttFailures < 255) {
    consecutiveMqttFailures++;
  }

  Serial.print("FAILED ");
  Serial.print(state);
  Serial.print(" (");
  Serial.print(mqttStateText(state));
  Serial.println(")");

  if (
    WiFi.status() == WL_CONNECTED &&
    consecutiveMqttFailures >= MQTT_FAILURES_BEFORE_WIFI_RECYCLE
  ) {
    recycleWiFiAfterMqttFault("Repeated MQTT connection failures");
    return;
  }

  mqtt.disconnect();
  wifiClient.stop();

  // MQTT is an application-layer connection. If Wi-Fi is still
  // associated, allow brief broker or LAN problems to recover without
  // disturbing it. Persistent TCP failures take the recovery path above.
  if (mqttRetryIntervalMs < MQTT_RETRY_MAX_MS) {
    mqttRetryIntervalMs *= 2;

    if (mqttRetryIntervalMs > MQTT_RETRY_MAX_MS) {
      mqttRetryIntervalMs = MQTT_RETRY_MAX_MS;
    }
  }

  Serial.print("[MQTT] Wi-Fi remains connected; retry in ");
  Serial.print(mqttRetryIntervalMs / 1000UL);
  Serial.println(" seconds");
}


const char* mqttStateText(int state) {

  switch (state) {
    case -4: return "connection timeout";
    case -3: return "connection lost";
    case -2: return "TCP connect failed";
    case -1: return "disconnected";
    case  1: return "bad protocol";
    case  2: return "bad client ID";
    case  3: return "broker unavailable";
    case  4: return "bad credentials";
    case  5: return "unauthorised";
    default: return "unknown";
  }
}


void mqttCallback(
  char* topic,
  byte* payload,
  unsigned int length
) {

  // Ignore the broker echo produced when a retained one-shot node
  // command is cleared.
  if (
    String(topic) == TOPIC_NODE_CMD &&
    length == 0
  ) {
    return;
  }

  String msg;

  msg.reserve(length + 1);

  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.print("[MQTT] CMD <- ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(msg);

  if (String(topic) == TOPIC_PUMP_CMD) {

    if (msg == "ON" || msg == "on") {
      setPump(true);
    }

    if (msg == "OFF" || msg == "off") {
      setPump(false);
    }

    return;
  }

  if (String(topic) == TOPIC_NODE_CMD) {

    // Commands are one-shot. Clearing any retained copy prevents a
    // calibration command from replaying after an MQTT reconnect.
    mqtt.publish(
      TOPIC_NODE_CMD,
      "",
      true
    );

    StaticJsonDocument<384> doc;

    DeserializationError err =
      deserializeJson(doc, msg);

    if (err) {

      Serial.print("[CMD] Invalid JSON: ");
      Serial.println(err.c_str());

      return;
    }

    if (doc.overflowed()) {

      Serial.println(
        "[CMD] JSON document overflow"
      );

      return;
    }

    const char* node = doc["node"];

    if (
      !doc["node"].is<const char*>() ||
      node == nullptr ||
      node[0] == '\0' ||
      strlen(node) > 31
    ) {

      Serial.println(
        "[CMD] Missing node"
      );

      return;
    }

    const char* cmd = doc["cmd"];

    if (
      !doc["cmd"].is<const char*>() ||
      cmd == nullptr ||
      cmd[0] == '\0'
    ) {

      Serial.println(
        "[CMD] Missing cmd"
      );

      return;
    }

    // Keep owned copies before removing the target from the JSON
    // document; pointers returned by ArduinoJson belong to the doc.
    String targetNode = node;
    String commandName = cmd;

    // The target node is implicit because the command is only sent
    // immediately after that node's own uplink. Remove "node" from
    // the radio payload to keep downlinks small and compatible with
    // the node parser.
    doc.remove("node");

    String radioPayload;
    serializeJson(doc, radioPayload);

    if (radioPayload.length() > 250) {

      Serial.println(
        "[CMD] Rejected: LoRa payload exceeds 250 bytes"
      );

      publishCommandStatus(
        targetNode.c_str(),
        commandName.c_str(),
        "too_large"
      );

      return;
    }

    bool queued =
      queueNodeCommand(
        targetNode,
        commandName,
        radioPayload
      );

    publishCommandStatus(
      targetNode.c_str(),
      commandName.c_str(),
      queued ? "queued" : "queue_full"
    );
  }
}


// ============================================================
// NETWORK / CONTROL FAILSAFES
// ============================================================
const char* resetReasonText(esp_reset_reason_t reason) {

  switch (reason) {
    case ESP_RST_POWERON:   return "power_on";
    case ESP_RST_EXT:       return "external";
    case ESP_RST_SW:        return "software";
    case ESP_RST_PANIC:     return "panic";
    case ESP_RST_INT_WDT:   return "interrupt_watchdog";
    case ESP_RST_TASK_WDT:  return "task_watchdog";
    case ESP_RST_WDT:       return "other_watchdog";
    case ESP_RST_DEEPSLEEP: return "deep_sleep";
    case ESP_RST_BROWNOUT:  return "brownout";
    case ESP_RST_SDIO:      return "sdio";
    default:                return "unknown";
  }
}


void restartForNetworkWatchdog(
  const char* cause,
  uint8_t causeCode
) {

  Serial.print("[FAILSAFE] ");
  Serial.print(cause);
  Serial.println(" unavailable for 10 minutes - restarting");

  // Outputs always fail safe before restarting, even if the relay is
  // not fitted yet.
  digitalWrite(PUMP_RELAY_PIN, LOW);
  pumpRunning = false;

  // Preserve the newest unsent report for each node. Failure to save
  // must not prevent recovery from a permanently stuck Wi-Fi driver.
  persistPendingUplinks();

  rtcDiagnosticMagic = RTC_DIAGNOSTIC_MAGIC;
  rtcNetworkWatchdogReboots++;
  rtcWatchdogPendingMarker = RTC_DIAGNOSTIC_MAGIC;
  rtcNetworkWatchdogCause = causeCode;

  Serial.flush();
  delay(250);
  ESP.restart();
}


void restartForLoRaWatchdog() {

  Serial.println(
    "[FAILSAFE] LoRa still silent after hard reset - restarting ESP32"
  );

  // A restart always returns control outputs to their safest state.
  digitalWrite(PUMP_RELAY_PIN, LOW);
  pumpRunning = false;

  persistPendingUplinks();

  rtcDiagnosticMagic = RTC_DIAGNOSTIC_MAGIC;
  rtcLoRaWatchdogReboots++;

  if (rtcConsecutiveLoRaWatchdogReboots < 255) {
    rtcConsecutiveLoRaWatchdogReboots++;
  }

  rtcWatchdogPendingMarker = RTC_DIAGNOSTIC_MAGIC;
  rtcNetworkWatchdogCause = 3;

  Serial.flush();
  delay(250);
  ESP.restart();
}


void serviceNetworkHealth() {

  unsigned long now = millis();
  bool wifiConnected = WiFi.status() == WL_CONNECTED;

  if (!wifiConnected) {

    // A Wi-Fi outage supersedes a broker-only outage.
    if (mqttOutageActive) {
      lastMqttOutageSeconds =
        (now - mqttOutageStarted) / 1000UL;

      if (lastMqttOutageSeconds > longestMqttOutageSeconds) {
        longestMqttOutageSeconds = lastMqttOutageSeconds;
      }

      mqttOutageActive = false;
    }

    if (!wifiOutageActive) {
      wifiOutageActive = true;
      wifiOutageStarted = now;
      Serial.println("[FAILSAFE] Wi-Fi outage timer started");
    }

    unsigned long outageMs = now - wifiOutageStarted;

    if (
      pumpRunning &&
      outageMs >= PUMP_NETWORK_FAILSAFE_MS
    ) {
      pumpFailsafeTrips++;
      setPump(false);
      Serial.println("[FAILSAFE] Pump forced OFF: network unavailable");
    }

    if (outageMs >= WIFI_WATCHDOG_REBOOT_MS) {
      restartForNetworkWatchdog("Wi-Fi", 1);
    }

    return;
  }

  if (wifiOutageActive) {
    lastWifiOutageSeconds =
      (now - wifiOutageStarted) / 1000UL;

    if (lastWifiOutageSeconds > longestWifiOutageSeconds) {
      longestWifiOutageSeconds = lastWifiOutageSeconds;
    }

    wifiOutageActive = false;

    Serial.print("[FAILSAFE] Wi-Fi recovered after ");
    Serial.print(lastWifiOutageSeconds);
    Serial.println(" seconds");
  }

  if (!mqtt.connected()) {

    if (!mqttOutageActive) {
      mqttOutageActive = true;
      mqttOutageStarted = now;
      Serial.println("[FAILSAFE] MQTT outage timer started");
    }

    if (
      pumpRunning &&
      now - mqttOutageStarted >= PUMP_NETWORK_FAILSAFE_MS
    ) {
      pumpFailsafeTrips++;
      setPump(false);
      Serial.println("[FAILSAFE] Pump forced OFF: MQTT unavailable");
    }

    if (
      now - mqttOutageStarted >=
      MQTT_WATCHDOG_REBOOT_MS
    ) {
      restartForNetworkWatchdog("MQTT", 2);
    }

    return;
  }

  if (mqttOutageActive) {
    lastMqttOutageSeconds =
      (now - mqttOutageStarted) / 1000UL;

    if (lastMqttOutageSeconds > longestMqttOutageSeconds) {
      longestMqttOutageSeconds = lastMqttOutageSeconds;
    }

    mqttOutageActive = false;

    Serial.print("[FAILSAFE] MQTT recovered after ");
    Serial.print(lastMqttOutageSeconds);
    Serial.println(" seconds");
  }
}


// ============================================================
// STATUS
// ============================================================
void publishStatus() {

  if (!mqtt.connected())
    return;

  unsigned long uptimeSeconds =
    (millis() - bootTime) / 1000UL;

  uint8_t activeNodes = 0;
  uint8_t queuedCommands = 0;

  for (uint8_t i = 0; i < MAX_TRACKED_NODES; i++) {
    if (nodes[i].used)
      activeNodes++;
  }

  for (uint8_t i = 0; i < MAX_PENDING_COMMANDS; i++) {
    if (pendingCommands[i].active)
      queuedCommands++;
  }

  StaticJsonDocument<2304> doc;

  const WiFiProfile& profile =
    activeWiFiProfile();

  doc["fw"] = FW_VERSION;
  doc["build"] = FW_BUILD;
  doc["gateway"] = GATEWAY_ID;
  doc["status"] = "online";

  doc["radio"] =
    !radioAvailable
      ? "offline"
      : lastLoRaPacket == 0
          ? "wio_sx1262_rx_unproven"
          : "wio_sx1262_ok";

  doc["ip"] =
    WiFi.localIP().toString();

  doc["wifi"] =
    WiFi.RSSI();

  doc["bssid"] =
    WiFi.BSSIDstr();

  doc["channel"] =
    WiFi.channel();

  doc["mqtt"] =
    mqtt.connected();

  doc["wifiProfile"] =
    profile.name;

  doc["ssid"] =
    profile.ssid;

  doc["mqttHost"] =
    profile.mqttServer;

  doc["heap"] =
    ESP.getFreeHeap();

  doc["uptime"] =
    uptimeSeconds;

  doc["rx"] =
    packetsReceived;

  doc["rxfail"] =
    packetsFailed;

  doc["radioerr"] =
    radioReadErrors;

  doc["downlink"] =
    downlinksSent;

  doc["downlinkFail"] =
    downlinksFailed;

  doc["uplinkAck"] =
    uplinkAcksSent;

  doc["uplinkAckFail"] =
    uplinkAcksFailed;

  doc["uplinkDuplicate"] =
    duplicateUplinks;

  doc["rxDiagPub"] =
    rxDiagnosticsPublished;

  doc["rxDiagDrop"] =
    rxDiagnosticsDropped;

  doc["nodes"] =
    activeNodes;

  doc["queued"] =
    queuedCommands;

  doc["uplinkQueued"] =
    pendingUplinkCount();

  doc["uplinkCached"] =
    uplinksCached;

  doc["uplinkFlushed"] =
    uplinksFlushed;

  doc["uplinkDropped"] =
    uplinksDropped;

  doc["uplinkRestored"] =
    uplinksRestored;

  doc["cacheSaveFail"] =
    cachePersistFailures;

  doc["mqttRecon"] =
    mqttReconnects;

  doc["mqttLoopFail"] =
    mqttLoopFailures;

  doc["mqttShort"] =
    mqttShortSessions;

  doc["mqttShortRun"] =
    consecutiveShortMqttSessions;

  doc["wifiRecon"] =
    wifiReconnects;

  doc["mqttFail"] =
    mqttFailures;

  doc["wifiFail"] =
    wifiFailures;

  doc["wifiReset"] =
    wifiRecycles;

  doc["reset"] =
    resetReasonText(bootResetReason);

  doc["boots"] =
    rtcBootCount;

  doc["netWdBoot"] =
    bootWasNetworkWatchdog;

  doc["netWdReboots"] =
    rtcNetworkWatchdogReboots;

  doc["netWdCause"] =
    bootNetworkWatchdogCause == 1
      ? "wifi"
      : bootNetworkWatchdogCause == 2
          ? "mqtt"
          : "none";

  doc["loraWdBoot"] =
    bootWasLoRaWatchdog;

  doc["loraWdReboots"] =
    rtcLoRaWatchdogReboots;

  doc["loraWdConsecutive"] =
    rtcConsecutiveLoRaWatchdogReboots;

  doc["loraRecovery"] =
    loraRecoveryPending;

  doc["loraSilent"] =
    lastLoRaPacket == 0
      ? (millis() - bootTime) / 1000UL
      : (millis() - lastLoRaPacket) / 1000UL;

  doc["wifiOutage"] =
    wifiOutageActive
      ? (millis() - wifiOutageStarted) / 1000UL
      : 0;

  doc["wifiOutLast"] =
    lastWifiOutageSeconds;

  doc["wifiOutMax"] =
    longestWifiOutageSeconds;

  doc["mqttOutage"] =
    mqttOutageActive
      ? (millis() - mqttOutageStarted) / 1000UL
      : 0;

  doc["mqttOutLast"] =
    lastMqttOutageSeconds;

  doc["mqttOutMax"] =
    longestMqttOutageSeconds;

  doc["pumpSafe"] =
    pumpFailsafeTrips;

  doc["radioInitRetry"] =
    radioInitRetries;

  doc["radioHardReset"] =
    radioHardResets;

  doc["radioHardFail"] =
    radioHardResetFailures;

  doc["rxArm"] =
    radioReceiveArms;

  doc["rxArmFail"] =
    radioReceiveArmFailures;

  doc["dio1Irq"] =
    loraDio1Interrupts;

  doc["dio1Poll"] =
    loraDio1PollRecoveries;

  doc["rfSwFault"] =
    rfSwitchStateFaults;

  doc["dio1Level"] =
    digitalRead(LORA_DIO1);

  doc["rfSwLevel"] =
    digitalRead(LORA_RF_SW);

  if (lastLoRaPacket == 0) {
    doc["lastLoRa"] = -1;
  } else {
    doc["lastLoRa"] =
      (millis() - lastLoRaPacket) / 1000UL;
  }

  String payload;
  serializeJson(doc, payload);

  mqtt.publish(
    TOPIC_GATEWAY,
    payload.c_str(),
    true
  );

  Serial.print("[STATUS] ");
  Serial.println(payload);
}


// ============================================================
// WIFI
// ============================================================
const WiFiProfile& activeWiFiProfile() {
  return WIFI_PROFILES[currentWiFiProfile];
}


bool selectNextEnabledWiFiProfile() {

  uint8_t startingProfile =
    currentWiFiProfile;

  for (
    uint8_t step = 1;
    step <= WIFI_PROFILE_COUNT;
    step++
  ) {

    uint8_t candidate =
      (startingProfile + step) % WIFI_PROFILE_COUNT;

    if (WIFI_PROFILES[candidate].enabled) {

      currentWiFiProfile = candidate;

      if (candidate != startingProfile) {

        Serial.print("[WiFi] Next profile: ");
        Serial.println(
          WIFI_PROFILES[candidate].name
        );

        return true;
      }

      return false;
    }
  }

  Serial.println(
    "[WiFi] ERROR: no enabled Wi-Fi profile"
  );

  return false;
}


void beginWiFiConnection() {

  if (
    WiFi.status() == WL_CONNECTED ||
    wifiConnecting
  ) {
    return;
  }

  if (!activeWiFiProfile().enabled) {

    selectNextEnabledWiFiProfile();

    if (!activeWiFiProfile().enabled)
      return;
  }

  const WiFiProfile& profile =
    activeWiFiProfile();

  // A full recovery cycle leaves the radio powered off. Always
  // restore station mode and the no-power-save policy before begin().
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(false);

  Serial.print("[WiFi] Starting profile ");
  Serial.print(profile.name);
  Serial.print(" - ");
  Serial.println(profile.ssid);

  WiFi.begin(
    profile.ssid,
    profile.password
  );

  wifiConnecting = true;
  wifiAttemptStarted = millis();
  lastWifiAttempt = wifiAttemptStarted;
}


void serviceWiFi() {

  wl_status_t status = WiFi.status();
  unsigned long now = millis();

  if (status == WL_CONNECTED) {

    wifiConnecting = false;

    if (!wifiWasConnected) {

      wifiWasConnected = true;

      if (wifiEverConnected) {
        wifiReconnects++;
      } else {
        wifiEverConnected = true;
      }

      consecutiveMqttFailures = 0;
      lastMqttAttempt = 0;
      mqttRetryIntervalMs = MQTT_RETRY_INITIAL_MS;
      wifiConnectedAt = now;

      digitalWrite(
        LED_PIN,
        HIGH
      );

      logWiFiDetails();
    }

    return;
  }

  if (wifiWasConnected) {

    Serial.print("[WiFi] Connection lost, status=");
    Serial.println((int)status);

    wifiWasConnected = false;
    wifiConnectedAt = 0;
    mqttSessionActive = false;
    mqttSessionStartedAt = 0;
    consecutiveShortMqttSessions = 0;
    mqtt.disconnect();
    wifiClient.stop();

    digitalWrite(
      LED_PIN,
      LOW
    );
  }

  if (
    wifiConnecting &&
    now - wifiAttemptStarted >= WIFI_CONNECT_TIMEOUT_MS
  ) {

    wifiFailures++;
    wifiConnecting = false;

    Serial.print("[WiFi] Connection timeout, status=");
    Serial.println((int)status);

    // Fully power-cycle the Wi-Fi driver. A soft disconnect was seen
    // to remain indefinitely in WL_IDLE_STATUS (status 0).
    WiFi.disconnect(
      true,
      false
    );

    WiFi.mode(WIFI_OFF);

    wifiRecycles++;

    selectNextEnabledWiFiProfile();

    lastWifiAttempt = now;
  }

  if (
    !wifiConnecting &&
    now - lastWifiAttempt >= WIFI_RETRY_INTERVAL_MS
  ) {
    beginWiFiConnection();
  }
}


void logWiFiDetails() {

  const WiFiProfile& profile =
    activeWiFiProfile();

  Serial.println("[WiFi] Connected");

  Serial.print("[WiFi] Profile: ");
  Serial.println(profile.name);

  Serial.print("[WiFi] SSID: ");
  Serial.println(profile.ssid);

  Serial.print("[WiFi] MQTT host: ");
  Serial.println(profile.mqttServer);

  Serial.print("[WiFi] IP: ");
  Serial.println(WiFi.localIP());

  Serial.print("[WiFi] Gateway: ");
  Serial.println(WiFi.gatewayIP());

  Serial.print("[WiFi] Subnet: ");
  Serial.println(WiFi.subnetMask());

  Serial.print("[WiFi] BSSID: ");
  Serial.println(WiFi.BSSIDstr());

  Serial.print("[WiFi] Channel: ");
  Serial.println(WiFi.channel());

  Serial.print("[WiFi] RSSI: ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}
