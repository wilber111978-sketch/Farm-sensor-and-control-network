// ============================================================
// TANK LEVEL NODE V1.4.0
// Farm Water Tank Monitor
//
// V1.4.0 reliability update (source: V1.3.1)
// - Requests a gateway ACK for every hourly uplink.
// - Retries the SAME seq with random 1-5 s backoff if ACK is missed.
// - Separates local RadioLib TX success from end-to-end delivery.
// - Reinitialises SX1262 after repeated missed-ACK cycles.
// - Power-cycles the DYP-A02 rail between failed measurement passes.
// - Still transmits a fault packet after bounded sensor recovery fails.
// - Adds compact ACK/sensor recovery diagnostics to telemetry.
// - Preserves EEPROM calibration, battery/charging logic, RTC STOP
//   sleep, DYP-A02 UART handling and remote calibration commands.
// - Requires Gateway V1.5.0+ for delivery acknowledgements.
//
// Hardware:
//   DX-SMART PJ26
//   STM32F103C8T6
//   DX-LR30 / SX1262 LoRa
//
// Sensor:
//   DYP-A02 Waterproof Ultrasonic
//   UART Automatic Version
//
// Tank 1
//
// Libraries:
//   RadioLib
//   ArduinoJson
//   STM32RTC
//   EEPROM (included with the STM32 Arduino core)
// ============================================================

#include <RadioLib.h>
#include <ArduinoJson.h>
#include <STM32RTC.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <stddef.h>
#include <string.h>


// ============================================================
// NODE CONFIG
// ============================================================

#define NODE_ID "tank_1"
#define FW_VERSION "1.4.0"


// ============================================================
// LORA PIN MAPPING
//
// Confirmed PJ26 / DX-LR30 mapping:
//
// NSS   -> PA4
// SCK   -> PA5
// MOSI  -> PA7
// MISO  -> PA6
// NRST  -> PA3
// BUSY  -> PA2
// DIO1  -> PC15
// RXEN  -> PA1
// TXEN  -> PA0
// ============================================================

#define LORA_NSS      PA4
#define LORA_DIO1     PC15
#define LORA_RESET    PA3
#define LORA_BUSY     PA2

#define LORA_RXEN     PA1
#define LORA_TXEN     PA0


// ============================================================
// LORA SPI
// ============================================================

SPISettings loraSPISettings(
  1000000,
  MSBFIRST,
  SPI_MODE0
);

Module loraModule(
  LORA_NSS,
  LORA_DIO1,
  LORA_RESET,
  LORA_BUSY,
  SPI,
  loraSPISettings
);

SX1262 radio(&loraModule);


// ============================================================
// LORA CONFIG
// MUST MATCH FARM GATEWAY
// ============================================================

#define LORA_FREQ     915.0
#define LORA_BW       125.0
#define LORA_SF       9
#define LORA_CR       5
#define LORA_SYNC     0x12
#define LORA_PWR      22
#define LORA_PREAMBLE 8

// Gateway sends a queued command immediately after this node's
// uplink. Keep the radio listening briefly for that downlink.
#define ACK_WAIT_MS            1800UL
#define DOWNLINK_WINDOW_MS     3000UL
#define DOWNLINK_POLL_MS       5UL

#define LORA_TX_ATTEMPTS       3
#define DELIVERY_ATTEMPTS      3
#define ACK_BACKOFF_MIN_MS     1000UL
#define ACK_BACKOFF_MAX_MS     5000UL
#define ACK_MISS_REINIT_CYCLES 3

#define SENSOR_PASS_ATTEMPTS   3
#define SENSOR_RECOVERY_OFF_MS 1000UL
#define MIN_VALID_READINGS     3


// ============================================================
// ULTRASONIC SENSOR
//
// DYP-A02 UART automatic:
//
// RED     -> switched 5V
// BLACK   -> LR7843 LOAD (low-side switched ground)
// YELLOW  -> PB6  (sensor RX / mode select)
// WHITE   -> PB7  (sensor TX / UART output)
//
// UART settings: 9600 baud, 8 data bits, no parity, 1 stop bit.
// Keep sensor RX HIGH for the processed/stable automatic output.
// Each valid frame is: 0xFF, distance high byte, distance low
// byte, checksum. Distance is supplied directly in millimetres.
// ============================================================

#define SONIC_MODE_PIN PB6
#define SONIC_UART_RX  PB7

// SoftwareSerial avoids sharing USART1 with the debug Serial port.
// Constructor pin order is MCU RX, MCU TX.
SoftwareSerial sonicSerial(
  SONIC_UART_RX,
  SONIC_MODE_PIN
);

#define STABILISE_MS  1200
#define NUM_READINGS  5


//
// PB4 drives the LR7843 module's active-HIGH PWM input.
#define SENSOR_PWR    PB4
#define USE_SENSOR_POWER_SWITCH true


// ============================================================
// BATTERY SENSE
//
// Battery + ---- 1k ----+---- PB0
//                       |
//                      2.2k
//                       |
//                      GND ---- PJ26 GND
//
// IMPORTANT: The battery/MPPT negative and PJ26 GND must be
// connected so PB0 has the same voltage reference as the ADC.
//
// Nominal divider ratio is 1.4545. The calibrated ratio below
// corrects the measured 3.905 V reading to 3.86 V on Tank 1.
// ============================================================

#define BATTERY_ADC PB0

#define DEFAULT_DIVIDER_RATIO 1.4380f

#define CHARGING_THRESHOLD_V 4.06f


// ============================================================
// BATTERY SOC TABLE
// ============================================================

struct VoltSoc {
  float volts;
  int pct;
};

const VoltSoc BATTERY_TABLE[] = {

  {4.20f, 100},
  {4.06f,  90},
  {3.98f,  80},
  {3.92f,  70},
  {3.87f,  60},
  {3.82f,  50},
  {3.79f,  40},
  {3.77f,  30},
  {3.74f,  20},
  {3.68f,  10},
  {3.45f,   5},
  {3.00f,   0}

};

#define BATTERY_TABLE_SIZE \
  (sizeof(BATTERY_TABLE) / sizeof(BATTERY_TABLE[0]))


// ============================================================
// TIMING
// ============================================================

#define SLEEP_SECONDS 3600


// ============================================================
// TANK CALIBRATION
//
// DEFAULT_FULL_DISTANCE_MM:
// distance sensor -> water when tank full
//
// DEFAULT_EMPTY_DISTANCE_MM:
// distance sensor -> tank bottom when empty
// ============================================================

#define DEFAULT_FULL_DISTANCE_MM   135
#define DEFAULT_EMPTY_DISTANCE_MM  2150
#define DEFAULT_TANK_CAPACITY_L    22500UL
#define DEFAULT_CAL_OFFSET_MM      0

// Persistent configuration guards. The magic/version distinguish
// valid V1.3 data from blank EEPROM or older unrelated contents.
#define CONFIG_MAGIC       0x544B3133UL
#define CONFIG_VERSION     1
#define CONFIG_EEPROM_ADDR 0


// ============================================================
// DATA STRUCTURE
// ============================================================

struct TankData {

  int distanceMM;

  float levelPct;

  int volumeL;

  float batteryVolts;

  int batteryPct;

  bool batteryCharging;

  bool sensorOk;

  bool valid;

  uint8_t validReadings;
};


struct TankConfig {

  uint32_t magic;

  uint32_t capacityL;

  float batteryDividerRatio;

  uint16_t version;

  uint16_t fullDistanceMM;

  uint16_t emptyDistanceMM;

  int16_t offsetMM;

  uint16_t crc;

  uint16_t reserved;
};


TankConfig tankConfig;


// ============================================================
// RTC / LOW POWER
// ============================================================

STM32RTC& rtc = STM32RTC::getInstance();

volatile bool rtcWoke = false;

uint32_t reportSequence = 0;

volatile bool radioIrqFlag = false;

bool lastDownlinkKnown = false;

float lastDownlinkRSSI = 0.0f;

float lastDownlinkSNR = 0.0f;

// V1.4 reliability diagnostics.
bool lastTxKnown = false;
bool lastTxOk = false;
bool lastAckKnown = false;
bool lastAckOk = false;
uint8_t lastDeliveryAttempts = 0;
uint32_t ackMissTotal = 0;
uint8_t consecutiveAckMissCycles = 0;
uint32_t radioReinitTotal = 0;

uint8_t sensorRetriesThisCycle = 0;
uint32_t sensorRestartTotal = 0;
uint32_t sensorFailedCycles = 0;

bool radioReady = false;


void radioDio1ISR() {

  radioIrqFlag = true;
}


// ============================================================
// FORWARD DECLARATIONS
// ============================================================

bool initLoRa();

bool transmitOneLoRaPacket(const String &payload);

void transmitLoRa(String payload);

void transmitLoRaReliable(
  const String &payload,
  const TankData &latestData
);

bool waitForDeliveryAck(
  uint32_t expectedSequence
);

void powerSensorOn();

void powerSensorOff();

TankData readTankLevel();

int readDYPA02();

void readBattery(TankData &d);

int voltageToPercent(float volts);

String buildPayload(TankData &d);

void stm32SleepSeconds(uint32_t seconds);

void rtcAlarmCallback(void* data);

uint16_t configCRC(const TankConfig &config);

bool configValuesValid(const TankConfig &config);

void loadTankConfig();

bool saveTankConfig(const TankConfig &config);

void printTankConfig();

void checkDownlink(const TankData &latestData);

void sendCommandAck(
  const char* command,
  const char* status,
  const TankData &latestData
);


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  delay(200);

  // STM32 ADC defaults can vary.
  // Force 12-bit so 4095 scaling below is correct.
  analogReadResolution(12);


  // ----------------------------------------------------------
  // ULTRASONIC
  // ----------------------------------------------------------

  // Keep both sensor signal pins high impedance while its
  // switched ground is off. This prevents phantom powering.
  pinMode(
    SONIC_MODE_PIN,
    INPUT_ANALOG
  );

  pinMode(
    SONIC_UART_RX,
    INPUT_ANALOG
  );


  // ----------------------------------------------------------
  // OPTIONAL SENSOR POWER
  // ----------------------------------------------------------

  pinMode(
    SENSOR_PWR,
    OUTPUT
  );

  digitalWrite(
    SENSOR_PWR,
    LOW
  );


  // ----------------------------------------------------------
  // BATTERY ADC
  // ----------------------------------------------------------

  pinMode(
    BATTERY_ADC,
    INPUT_ANALOG
  );


  // Load calibration before the first measurement. Blank or
  // invalid EEPROM automatically receives the compiled defaults.
  loadTankConfig();


  // ----------------------------------------------------------
  // LORA RF SWITCH
  // ----------------------------------------------------------

  pinMode(
    LORA_RXEN,
    OUTPUT
  );

  pinMode(
    LORA_TXEN,
    OUTPUT
  );

  // Idle / receive state.
  digitalWrite(
    LORA_RXEN,
    HIGH
  );

  digitalWrite(
    LORA_TXEN,
    LOW
  );


  // ----------------------------------------------------------
  // LORA
  // ----------------------------------------------------------

  radioReady =
    initLoRa();

  // DIO1 only sets a flag; all RadioLib work stays outside ISR.
  radio.setDio1Action(
    radioDio1ISR
  );


  Serial.println();

  Serial.println(
    "================================"
  );

  Serial.println(
    "Tank Node Started"
  );

  Serial.println(
    NODE_ID
  );

  Serial.print(
    "Firmware "
  );

  Serial.println(
    FW_VERSION
  );

  Serial.println(
    "================================"
  );

  printTankConfig();
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  reportSequence++;

  Serial.println();

  Serial.println(
    "[TANK NODE] Measurement Cycle"
  );


  TankData data =
    readTankLevel();


  String payload =
    buildPayload(data);


  Serial.println(
    payload
  );


  transmitLoRaReliable(
    payload,
    data
  );


  if(data.sensorOk) {

    Serial.println(
      "[OK] Tank Read Success"
    );

  } else {

    Serial.println(
      "[ERROR] Tank Read Failed - battery-only packet sent"
    );
  }


  Serial.print(
    "[SLEEP] "
  );

  Serial.print(
    SLEEP_SECONDS
  );

  Serial.println(
    " seconds"
  );


  Serial.flush();


  stm32SleepSeconds(
    SLEEP_SECONDS
  );
}


// ============================================================
// SENSOR POWER
// ============================================================

void powerSensorOn() {

  // Start from a high-impedance UART state so the unpowered
  // sensor cannot be fed through either signal wire.
  sonicSerial.end();

  pinMode(
    SONIC_MODE_PIN,
    INPUT_ANALOG
  );

  pinMode(
    SONIC_UART_RX,
    INPUT_ANALOG
  );

  if(USE_SENSOR_POWER_SWITCH) {

    digitalWrite(
      SENSOR_PWR,
      HIGH
    );

    delay(
      STABILISE_MS
    );

  } else {

    // Sensor currently powered directly from 3.3V.
    // Allow it to settle after MCU wake.
    delay(
      STABILISE_MS
    );
  }


  // Restart the UART after the sensor has stabilised so the first
  // measurement cannot begin in the middle of a startup frame.
  sonicSerial.end();

  sonicSerial.begin(
    9600
  );

  delay(
    300
  );


  // Discard startup and stale bytes before collecting readings.
  while(sonicSerial.available() > 0) {

    sonicSerial.read();
  }
}


void powerSensorOff() {

  // Release both UART lines before disconnecting the sensor's
  // ground. This avoids phantom powering through PB6 or PB7.
  sonicSerial.end();

  pinMode(
    SONIC_MODE_PIN,
    INPUT_ANALOG
  );

  pinMode(
    SONIC_UART_RX,
    INPUT_ANALOG
  );

  delayMicroseconds(100);

  if(USE_SENSOR_POWER_SWITCH) {

    digitalWrite(
      SENSOR_PWR,
      LOW
    );
  }
}


// ============================================================
// PERSISTENT TANK CONFIGURATION
// ============================================================

uint16_t configCRC(
  const TankConfig &config
) {

  const uint8_t* bytes =
    reinterpret_cast<const uint8_t*>(
      &config
    );

  uint16_t crc =
    0xFFFF;

  for(size_t i = 0; i < offsetof(TankConfig, crc); i++) {

    crc ^=
      (uint16_t)bytes[i] << 8;

    for(uint8_t bit = 0; bit < 8; bit++) {

      if(crc & 0x8000) {

        crc =
          (crc << 1) ^ 0x1021;

      } else {

        crc <<= 1;
      }
    }
  }

  return crc;
}


bool configValuesValid(
  const TankConfig &config
) {

  if(
    config.fullDistanceMM < 30 ||
    config.fullDistanceMM > 4450
  ) {

    return false;
  }

  if(
    config.emptyDistanceMM < 80 ||
    config.emptyDistanceMM > 4500
  ) {

    return false;
  }

  // Keep at least 50 mm between full and empty calibration.
  if(
    config.emptyDistanceMM <=
    config.fullDistanceMM + 50
  ) {

    return false;
  }

  if(
    config.capacityL < 100 ||
    config.capacityL > 1000000UL
  ) {

    return false;
  }

  if(
    config.offsetMM < -1000 ||
    config.offsetMM > 1000
  ) {

    return false;
  }

  if(!(
    config.batteryDividerRatio >= 1.0f &&
    config.batteryDividerRatio <= 2.5f
  )) {

    return false;
  }

  return true;
}


bool saveTankConfig(
  const TankConfig &config
) {

  TankConfig stored =
    config;

  stored.magic =
    CONFIG_MAGIC;

  stored.version =
    CONFIG_VERSION;

  stored.reserved =
    0;

  stored.crc =
    configCRC(stored);

  if(!configValuesValid(stored)) {

    return false;
  }

  bool changed =
    memcmp(
      &stored,
      &tankConfig,
      sizeof(TankConfig)
    ) != 0;

  if(changed) {

    EEPROM.put(
      CONFIG_EEPROM_ADDR,
      stored
    );
  }

  tankConfig =
    stored;

  return true;
}


void loadTankConfig() {

  TankConfig stored = {0};

  EEPROM.get(
    CONFIG_EEPROM_ADDR,
    stored
  );

  bool storedValid =
    stored.magic == CONFIG_MAGIC &&
    stored.version == CONFIG_VERSION &&
    stored.crc == configCRC(stored) &&
    configValuesValid(stored);

  if(storedValid) {

    tankConfig =
      stored;

    return;
  }

  TankConfig defaults = {0};

  defaults.fullDistanceMM =
    DEFAULT_FULL_DISTANCE_MM;

  defaults.emptyDistanceMM =
    DEFAULT_EMPTY_DISTANCE_MM;

  defaults.capacityL =
    DEFAULT_TANK_CAPACITY_L;

  defaults.batteryDividerRatio =
    DEFAULT_DIVIDER_RATIO;

  defaults.offsetMM =
    DEFAULT_CAL_OFFSET_MM;

  // This also assigns magic/version/CRC and stores defaults once.
  saveTankConfig(
    defaults
  );
}


void printTankConfig() {

  Serial.print(
    "[CONFIG] full="
  );

  Serial.print(
    tankConfig.fullDistanceMM
  );

  Serial.print(
    " mm, empty="
  );

  Serial.print(
    tankConfig.emptyDistanceMM
  );

  Serial.print(
    " mm, capacity="
  );

  Serial.print(
    tankConfig.capacityL
  );

  Serial.print(
    " L, offset="
  );

  Serial.print(
    tankConfig.offsetMM
  );

  Serial.print(
    " mm, battery ratio="
  );

  Serial.print(
    tankConfig.batteryDividerRatio,
    4
  );

  Serial.println(
    ""
  );
}


// ============================================================
// READ TANK
// ============================================================

TankData readTankLevel() {

  TankData result = {0};

  result.sensorOk = false;
  result.valid = true;

  sensorRetriesThisCycle = 0;

  int bestValues[NUM_READINGS] = {0};
  int bestCount = 0;


  for(
    uint8_t pass = 1;
    pass <= SENSOR_PASS_ATTEMPTS;
    pass++
  ) {

    if(pass > 1) {

      sensorRetriesThisCycle++;
      sensorRestartTotal++;

      Serial.print(
        "[SENSOR] DYP-A02 recovery power cycle "
      );

      Serial.print(
        pass - 1
      );

      Serial.print(
        "/"
      );

      Serial.println(
        SENSOR_PASS_ATTEMPTS - 1
      );

      delay(
        SENSOR_RECOVERY_OFF_MS
      );
    }


    powerSensorOn();


    int values[NUM_READINGS] = {0};
    int count = 0;


    for(int i = 0; i < NUM_READINGS; i++) {

      int dist =
        readDYPA02();


      if(dist > 0) {

        values[count++] =
          dist;


        Serial.print(
          "Reading "
        );

        Serial.print(
          i + 1
        );

        Serial.print(
          ": "
        );

        Serial.print(
          dist
        );

        Serial.println(
          " mm"
        );

      } else {

        Serial.print(
          "Reading "
        );

        Serial.print(
          i + 1
        );

        Serial.println(
          ": FAIL"
        );
      }


      delay(200);
    }


    powerSensorOff();


    if(count > bestCount) {

      bestCount =
        count;

      for(int i = 0; i < count; i++) {

        bestValues[i] =
          values[i];
      }
    }


    if(count >= MIN_VALID_READINGS) {

      bestCount =
        count;

      for(int i = 0; i < count; i++) {

        bestValues[i] =
          values[i];
      }


      if(pass > 1) {

        Serial.print(
          "[SENSOR] Recovered on pass "
        );

        Serial.println(
          pass
        );
      }

      break;
    }


    Serial.print(
      "[SENSOR] Pass "
    );

    Serial.print(
      pass
    );

    Serial.print(
      " unhealthy: valid reads="
    );

    Serial.println(
      count
    );
  }


  result.validReadings =
    bestCount;


  // Battery always gets reported, even if the sensor fails.
  readBattery(
    result
  );


  if(bestCount < MIN_VALID_READINGS) {

    sensorFailedCycles++;

    return result;
  }


  // Median sort.
  for(int i = 0; i < bestCount - 1; i++) {

    for(int j = i + 1; j < bestCount; j++) {

      if(bestValues[j] < bestValues[i]) {

        int temp =
          bestValues[i];

        bestValues[i] =
          bestValues[j];

        bestValues[j] =
          temp;
      }
    }
  }


  result.distanceMM =
    bestValues[bestCount / 2];


  if(
    result.distanceMM < 50 ||
    result.distanceMM > 10000
  ) {

    sensorFailedCycles++;

    return result;
  }


  int calibratedDistance =
    result.distanceMM +
    tankConfig.offsetMM;


  result.levelPct =
    (
      tankConfig.emptyDistanceMM -
      calibratedDistance
    )
    /
    (float)(
      tankConfig.emptyDistanceMM -
      tankConfig.fullDistanceMM
    )
    *
    100.0f;


  result.levelPct =
    constrain(
      result.levelPct,
      0.0f,
      100.0f
    );


  result.volumeL =
    (
      tankConfig.capacityL *
      result.levelPct
    )
    /
    100.0f;


  result.sensorOk =
    true;


  return result;
}

// ============================================================
// DYP-A02 AUTOMATIC UART READ
// Frame: 0xFF, DIST_H, DIST_L, CHECKSUM
// CHECKSUM = low 8 bits of (0xFF + DIST_H + DIST_L)
// ============================================================

int readDYPA02() {
  uint32_t deadline = millis() + 1500UL;

  while((int32_t)(millis() - deadline) < 0) {

    if(sonicSerial.available() <= 0) {
      delay(1);
      continue;
    }

    // Discard bytes until the frame header is found.
    if((uint8_t)sonicSerial.read() != 0xFF) {
      continue;
    }

    // At 9600 baud the remaining three bytes take only about
    // 3.2 ms. Allow 20 ms so a partial frame cannot block the node.
    uint32_t frameDeadline = millis() + 20UL;

    while(
      sonicSerial.available() < 3 &&
      (int32_t)(millis() - frameDeadline) < 0
    ) {
      delay(1);
    }

    if(sonicSerial.available() < 3) {
      continue;
    }

    uint8_t distanceHigh =
      (uint8_t)sonicSerial.read();

    uint8_t distanceLow =
      (uint8_t)sonicSerial.read();

    uint8_t receivedChecksum =
      (uint8_t)sonicSerial.read();

    uint8_t expectedChecksum =
      (uint8_t)(
        0xFF +
        distanceHigh +
        distanceLow
      );

    if(receivedChecksum != expectedChecksum) {
      continue;
    }

    int distanceMM =
      ((int)distanceHigh << 8) |
      distanceLow;

    // A02YYUW specified usable range is 30 to 4500 mm.
    if(distanceMM < 30 || distanceMM > 4500) {
      return -1;
    }

    return distanceMM;
  }

  return -1;
}


// ============================================================
// BATTERY SOC
// ============================================================

int voltageToPercent(float volts) {

  if(
    volts >=
    BATTERY_TABLE[0].volts
  ) {

    return
      BATTERY_TABLE[0].pct;
  }


  if(
    volts <=
    BATTERY_TABLE[
      BATTERY_TABLE_SIZE - 1
    ].volts
  ) {

    return
      BATTERY_TABLE[
        BATTERY_TABLE_SIZE - 1
      ].pct;
  }


  for(
    size_t i = 0;
    i < BATTERY_TABLE_SIZE - 1;
    i++
  ) {

    float vHigh =
      BATTERY_TABLE[i].volts;

    float vLow =
      BATTERY_TABLE[i + 1].volts;


    if(
      volts <= vHigh &&
      volts >= vLow
    ) {

      float span =
        vHigh - vLow;


      float frac =
        (
          span > 0.0f
        )
        ?
        (
          volts - vLow
        )
        /
        span
        :
        0.0f;


      int pctHigh =
        BATTERY_TABLE[i].pct;

      int pctLow =
        BATTERY_TABLE[i + 1].pct;


      return
        pctLow +
        (int)round(
          frac *
          (
            pctHigh -
            pctLow
          )
        );
    }
  }


  return 0;
}


// ============================================================
// READ BATTERY
// ============================================================

void readBattery(TankData &d) {

  analogReadResolution(12);
  pinMode(BATTERY_ADC, INPUT_ANALOG);

  delay(10);

  // Discard initial ADC conversions.
  for(int i = 0; i < 8; i++) {
    analogRead(BATTERY_ADC);
    delay(2);
  }

  uint32_t total = 0;

  for(int i = 0; i < 32; i++) {
    total += analogRead(BATTERY_ADC);
    delay(2);
  }

  float rawAverage =
    total / 32.0f;

  float vAdc =
    (rawAverage / 4095.0f) * 3.3f;

  d.batteryVolts =
    vAdc *
    tankConfig.batteryDividerRatio;

  if(d.batteryVolts >= CHARGING_THRESHOLD_V) {
    d.batteryCharging = true;
    d.batteryPct = -1;
  } else {
    d.batteryCharging = false;
    d.batteryPct =
      voltageToPercent(d.batteryVolts);
  }
}


// ============================================================
// BUILD PAYLOAD
// ============================================================

String buildPayload(TankData &d) {

  StaticJsonDocument<512> doc;


  doc["node"] =
    NODE_ID;


  doc["fw"] =
    FW_VERSION;


  doc["seq"] =
    reportSequence;

  doc["ack_req"] =
    true;


  doc["sensor_ok"] =
    d.sensorOk;


  doc["reads"] =
    d.validReadings;


  if(lastTxKnown) {

    doc["ltx"] =
      lastTxOk;
  }


  if(lastAckKnown) {

    doc["lack"] =
      lastAckOk;

    doc["txa"] =
      lastDeliveryAttempts;
  }


  doc["am"] =
    ackMissTotal;

  doc["sr"] =
    sensorRetriesThisCycle;

  doc["srt"] =
    sensorRestartTotal;


  if(!d.sensorOk) {

    doc["fault"] =
      "sonic";
  }


  // Active persisted calibration is included on every report so
  // a lost command acknowledgement is confirmed on the next hour.
  doc["full"] =
    tankConfig.fullDistanceMM;

  doc["empty"] =
    tankConfig.emptyDistanceMM;

  doc["cap"] =
    tankConfig.capacityL;

  doc["off"] =
    tankConfig.offsetMM;

  doc["br"] =
    serialized(
      String(
        tankConfig.batteryDividerRatio,
        4
      )
    );


  if(lastDownlinkKnown) {

    doc["dlr"] =
      serialized(
        String(
          lastDownlinkRSSI,
          1
        )
      );

    doc["dls"] =
      serialized(
        String(
          lastDownlinkSNR,
          1
        )
      );
  }


  // Battery always transmitted.
  doc["batv"] =
    serialized(
      String(
        d.batteryVolts,
        2
      )
    );


  if(d.batteryCharging) {

    doc["bat"] =
      "charging";

  } else {

    doc["bat"] =
      d.batteryPct;
  }


  if(d.sensorOk) {

    doc["dist"] =
      d.distanceMM;


    doc["pct"] =
      serialized(
        String(
          d.levelPct,
          1
        )
      );


    doc["vol"] =
      d.volumeL;
  }


  // SX1262 LoRa packets are limited to 255 bytes. Downlink link
  // metrics are optional and are removed first if diagnostics grow.
  if(measureJson(doc) > 250) {

    doc.remove("dlr");
    doc.remove("dls");
  }


  if(measureJson(doc) > 250) {

    doc.remove("srt");
    doc.remove("am");
  }


  if(measureJson(doc) > 250) {

    doc.remove("ltx");
  }


  String payload;


  serializeJson(
    doc,
    payload
  );


  return payload;
}


// ============================================================
// LORA INITIALISATION
// ============================================================

bool initLoRa() {

  Serial.print(
    "LoRa Init... "
  );


  int state =
    radio.begin(

      LORA_FREQ,

      LORA_BW,

      LORA_SF,

      LORA_CR,

      LORA_SYNC,

      LORA_PWR,

      LORA_PREAMBLE,

      0.0,

      false
    );


  if(
    state ==
    RADIOLIB_ERR_NONE
  ) {

    Serial.println(
      "OK"
    );

    return true;

  } else {

    Serial.print(
      "FAIL: "
    );

    Serial.println(
      state
    );

    return false;
  }
}


// ============================================================
// LORA TRANSMIT
// ============================================================

bool transmitOneLoRaPacket(
  const String &payload
) {

  if(!radioReady) {

    radioReady =
      initLoRa();

    if(radioReady) {

      radio.setDio1Action(
        radioDio1ISR
      );
    }
  }


  if(!radioReady) {

    return false;
  }


  for(
    uint8_t attempt = 1;
    attempt <= LORA_TX_ATTEMPTS;
    attempt++
  ) {

    digitalWrite(
      LORA_RXEN,
      LOW
    );

    digitalWrite(
      LORA_TXEN,
      HIGH
    );


    int state =
      radio.transmit(
        payload
      );


    digitalWrite(
      LORA_TXEN,
      LOW
    );

    digitalWrite(
      LORA_RXEN,
      HIGH
    );


    if(
      state ==
      RADIOLIB_ERR_NONE
    ) {

      return true;
    }


    Serial.print(
      "[LoRa] Local TX attempt "
    );

    Serial.print(
      attempt
    );

    Serial.print(
      " failed: "
    );

    Serial.println(
      state
    );


    radio.standby();

    delay(150);
  }


  radioReady =
    false;

  return false;
}


// Existing remote-calibration ACKs still use a normal single send.
void transmitLoRa(
  String payload
) {

  bool ok =
    transmitOneLoRaPacket(
      payload
    );


  if(ok) {

    Serial.println(
      "TX OK"
    );

  } else {

    Serial.println(
      "TX FAIL"
    );
  }
}


bool waitForDeliveryAck(
  uint32_t expectedSequence
) {

  if(!radioReady) {

    return false;
  }


  radio.standby();


  digitalWrite(
    LORA_TXEN,
    LOW
  );

  digitalWrite(
    LORA_RXEN,
    HIGH
  );


  radioIrqFlag =
    false;


  int state =
    radio.startReceive();


  if(
    state !=
    RADIOLIB_ERR_NONE
  ) {

    Serial.print(
      "[ACK] RX start failed: "
    );

    Serial.println(
      state
    );

    return false;
  }


  uint32_t started =
    millis();


  while(
    millis() - started <
    ACK_WAIT_MS
  ) {

    if(!radioIrqFlag) {

      delay(
        DOWNLINK_POLL_MS
      );

      continue;
    }


    radioIrqFlag =
      false;


    String received;


    state =
      radio.readData(
        received
      );


    if(
      state !=
      RADIOLIB_ERR_NONE
    ) {

      Serial.print(
        "[ACK] Read failed: "
      );

      Serial.println(
        state
      );

      radio.startReceive();

      continue;
    }


    lastDownlinkRSSI =
      radio.getRSSI();

    lastDownlinkSNR =
      radio.getSNR();

    lastDownlinkKnown =
      true;


    StaticJsonDocument<160> doc;


    DeserializationError error =
      deserializeJson(
        doc,
        received
      );


    if(!error) {

      const char* ack =
        doc["ack"] |
        "";

      const char* node =
        doc["node"] |
        "";

      uint32_t seq =
        doc["seq"] |
        0;


      if(
        strcmp(
          ack,
          "uplink"
        ) == 0 &&
        strcmp(
          node,
          NODE_ID
        ) == 0 &&
        seq ==
        expectedSequence
      ) {

        radio.standby();


        Serial.print(
          "[ACK] Delivery confirmed seq="
        );

        Serial.println(
          expectedSequence
        );


        return true;
      }
    }


    radio.startReceive();
  }


  radio.standby();


  return false;
}


void transmitLoRaReliable(
  const String &payload,
  const TankData &latestData
) {

  if(
    consecutiveAckMissCycles >=
    ACK_MISS_REINIT_CYCLES
  ) {

    Serial.println(
      "[ACK] Repeated misses: reinitialising LoRa"
    );


    radioReady =
      false;

    radioReinitTotal++;


    radioReady =
      initLoRa();


    if(radioReady) {

      radio.setDio1Action(
        radioDio1ISR
      );
    }
  }


  bool delivered =
    false;

  bool anyLocalTx =
    false;

  uint8_t attemptsUsed =
    0;


  for(
    uint8_t attempt = 1;
    attempt <= DELIVERY_ATTEMPTS;
    attempt++
  ) {

    attemptsUsed =
      attempt;


    Serial.print(
      "[TX] Delivery attempt "
    );

    Serial.print(
      attempt
    );

    Serial.print(
      "/"
    );

    Serial.println(
      DELIVERY_ATTEMPTS
    );


    bool localOk =
      transmitOneLoRaPacket(
        payload
      );


    lastTxKnown =
      true;

    lastTxOk =
      localOk;


    if(localOk) {

      anyLocalTx =
        true;


      if(
        waitForDeliveryAck(
          reportSequence
        )
      ) {

        delivered =
          true;

        break;
      }


      Serial.println(
        "[ACK] No matching gateway ACK"
      );
    }


    if(
      attempt <
      DELIVERY_ATTEMPTS
    ) {

      uint32_t backoff =
        random(
          ACK_BACKOFF_MIN_MS,
          ACK_BACKOFF_MAX_MS + 1
        );


      Serial.print(
        "[ACK] Random retry backoff "
      );

      Serial.print(
        backoff
      );

      Serial.println(
        " ms"
      );


      delay(
        backoff
      );
    }
  }


  lastAckKnown =
    true;

  lastAckOk =
    delivered;

  lastDeliveryAttempts =
    attemptsUsed;


  if(delivered) {

    consecutiveAckMissCycles =
      0;


    // Preserve remote calibration/downlink behaviour.
    checkDownlink(
      latestData
    );

  } else {

    ackMissTotal++;


    if(
      consecutiveAckMissCycles <
      255
    ) {

      consecutiveAckMissCycles++;
    }


    Serial.print(
      "[ACK] DELIVERY FAILED; consecutive cycles="
    );

    Serial.println(
      consecutiveAckMissCycles
    );


    if(!anyLocalTx) {

      radioReady =
        false;
    }
  }
}

// ============================================================
// COMMAND ACKNOWLEDGEMENT
//
// Includes normal telemetry so the gateway's retained tank topic
// never gets replaced by a command-only packet with no level.
// ============================================================

void sendCommandAck(
  const char* command,
  const char* status,
  const TankData &latestData
) {

  StaticJsonDocument<640> doc;

  doc["node"] =
    NODE_ID;

  doc["fw"] =
    FW_VERSION;

  doc["seq"] =
    reportSequence;

  doc["sensor_ok"] =
    latestData.sensorOk;

  doc["reads"] =
    latestData.validReadings;

  doc["batv"] =
    serialized(
      String(
        latestData.batteryVolts,
        2
      )
    );

  if(latestData.batteryCharging) {

    doc["bat"] =
      "charging";

  } else {

    doc["bat"] =
      latestData.batteryPct;
  }

  if(latestData.sensorOk) {

    int calibratedDistance =
      latestData.distanceMM +
      tankConfig.offsetMM;

    float currentLevelPct =
      (
        tankConfig.emptyDistanceMM -
        calibratedDistance
      ) /
      (float)(
        tankConfig.emptyDistanceMM -
        tankConfig.fullDistanceMM
      ) *
      100.0f;

    currentLevelPct =
      constrain(
        currentLevelPct,
        0.0f,
        100.0f
      );

    int currentVolumeL =
      (
        tankConfig.capacityL *
        currentLevelPct
      ) /
      100.0f;

    doc["dist"] =
      latestData.distanceMM;

    doc["pct"] =
      serialized(
        String(
          currentLevelPct,
          1
        )
      );

    doc["vol"] =
      currentVolumeL;
  }

  doc["full"] =
    tankConfig.fullDistanceMM;

  doc["empty"] =
    tankConfig.emptyDistanceMM;

  doc["cap"] =
    tankConfig.capacityL;

  doc["off"] =
    tankConfig.offsetMM;

  doc["br"] =
    serialized(
      String(
        tankConfig.batteryDividerRatio,
        4
      )
    );

  doc["ack"] =
    command;

  doc["ack_status"] =
    status;

  // Preserve the tank reading and acknowledgement if the packet
  // ever approaches the LoRa limit; sequence/read count are optional.
  if(measureJson(doc) > 250) {

    doc.remove("seq");
    doc.remove("reads");
  }

  String payload;

  serializeJson(
    doc,
    payload
  );

  Serial.print(
    "[DOWNLINK] ACK: "
  );

  Serial.println(
    payload
  );

  transmitLoRa(
    payload
  );
}


// ============================================================
// DOWNLINK / REMOTE CALIBRATION
//
// Supported gateway commands:
//   {"cmd":"ping"}
//   {"cmd":"get_config"}
//   {"cmd":"capture_full"}
//   {"cmd":"capture_empty"}
//   {"cmd":"set_cal","full":135,"empty":2150,
//    "capacity":22500,"offset":0,"bat_ratio":1.4380}
// ============================================================

void checkDownlink(
  const TankData &latestData
) {

  Serial.print(
    "[DOWNLINK] Listening "
  );

  Serial.print(
    DOWNLINK_WINDOW_MS / 1000UL
  );

  Serial.println(
    " seconds"
  );

  // Explicitly select the LR30 receive path.
  digitalWrite(
    LORA_TXEN,
    LOW
  );

  digitalWrite(
    LORA_RXEN,
    HIGH
  );

  radio.standby();

  // TX completion can leave DIO1 asserted; discard stale state.
  radioIrqFlag =
    false;

  int state =
    radio.startReceive();

  if(state != RADIOLIB_ERR_NONE) {

    Serial.print(
      "[DOWNLINK] RX start failed: "
    );

    Serial.println(
      state
    );

    return;
  }

  uint32_t started =
    millis();

  while(
    !radioIrqFlag &&
    millis() - started < DOWNLINK_WINDOW_MS
  ) {

    delay(
      DOWNLINK_POLL_MS
    );
  }

  if(!radioIrqFlag) {

    radio.standby();

    Serial.println(
      "[DOWNLINK] No command"
    );

    return;
  }

  radioIrqFlag =
    false;

  String received;

  state =
    radio.readData(
      received
    );

  if(state != RADIOLIB_ERR_NONE) {

    Serial.print(
      "[DOWNLINK] Read failed: "
    );

    Serial.println(
      state
    );

    radio.standby();

    return;
  }

  lastDownlinkRSSI =
    radio.getRSSI();

  lastDownlinkSNR =
    radio.getSNR();

  lastDownlinkKnown =
    true;

  radio.standby();

  Serial.print(
    "[DOWNLINK] Received: "
  );

  Serial.println(
    received
  );

  StaticJsonDocument<256> doc;

  DeserializationError error =
    deserializeJson(
      doc,
      received
    );

  if(error) {

    Serial.println(
      "[DOWNLINK] Ignored malformed JSON"
    );

    return;
  }

  const char* command =
    doc["cmd"] |
    "";

  if(command[0] == '\0') {

    Serial.println(
      "[DOWNLINK] Ignored command with no cmd"
    );

    return;
  }

  TankConfig candidate =
    tankConfig;

  const char* status =
    "unknown";

  bool shouldSave =
    false;

  bool fieldsValid =
    true;

  if(
    strcmp(command, "ping") == 0 ||
    strcmp(command, "get_config") == 0
  ) {

    status =
      "ok";

  } else if(strcmp(command, "capture_full") == 0) {

    if(latestData.sensorOk) {

      candidate.fullDistanceMM =
        latestData.distanceMM;

      shouldSave =
        true;

    } else {

      status =
        "no_sensor";
    }

  } else if(strcmp(command, "capture_empty") == 0) {

    if(latestData.sensorOk) {

      candidate.emptyDistanceMM =
        latestData.distanceMM;

      shouldSave =
        true;

    } else {

      status =
        "no_sensor";
    }

  } else if(strcmp(command, "set_cal") == 0) {

    bool supplied =
      false;

    if(doc.containsKey("full")) {

      long value =
        doc["full"].as<long>();

      supplied =
        true;

      if(value < 30 || value > 4450) {

        fieldsValid =
          false;

      } else {

        candidate.fullDistanceMM =
          (uint16_t)value;
      }
    }

    if(doc.containsKey("empty")) {

      long value =
        doc["empty"].as<long>();

      supplied =
        true;

      if(value < 80 || value > 4500) {

        fieldsValid =
          false;

      } else {

        candidate.emptyDistanceMM =
          (uint16_t)value;
      }
    }

    if(doc.containsKey("capacity")) {

      long value =
        doc["capacity"].as<long>();

      supplied =
        true;

      if(value < 100 || value > 1000000L) {

        fieldsValid =
          false;

      } else {

        candidate.capacityL =
          (uint32_t)value;
      }
    }

    if(doc.containsKey("offset")) {

      long value =
        doc["offset"].as<long>();

      supplied =
        true;

      if(value < -1000 || value > 1000) {

        fieldsValid =
          false;

      } else {

        candidate.offsetMM =
          (int16_t)value;
      }
    }

    if(doc.containsKey("bat_ratio")) {

      float value =
        doc["bat_ratio"].as<float>();

      supplied =
        true;

      if(!(value >= 1.0f && value <= 2.5f)) {

        fieldsValid =
          false;

      } else {

        candidate.batteryDividerRatio =
          value;
      }
    }

    shouldSave =
      supplied &&
      fieldsValid;

    if(!shouldSave) {

      status =
        "invalid";
    }
  }

  if(shouldSave) {

    if(
      configValuesValid(candidate) &&
      saveTankConfig(candidate)
    ) {

      status =
        "ok";

      printTankConfig();

    } else {

      status =
        "invalid";
    }
  }

  sendCommandAck(
    command,
    status,
    latestData
  );
}


// ============================================================
// LOW POWER RTC SLEEP
// ============================================================

void rtcAlarmCallback(void* data) {

  rtcWoke =
    true;
}


void stm32SleepSeconds(
  uint32_t seconds
) {

  rtcWoke =
    false;


  rtc.begin();


  rtc.attachInterrupt(
    rtcAlarmCallback
  );


  rtc.setAlarmEpoch(
    rtc.getEpoch() +
    seconds,

    STM32RTC::MATCH_DHHMMSS
  );


  HAL_PWR_EnterSTOPMode(
    PWR_LOWPOWERREGULATOR_ON,
    PWR_STOPENTRY_WFI
  );


  // Restore system clock after STM32 STOP mode.
  SystemClock_Config();


  rtc.detachInterrupt();


  // Keep the switched sensor off and its signal pins high
  // impedance. powerSensorOn() will restart UART after warm-up.
  pinMode(
    SENSOR_PWR,
    OUTPUT
  );

  digitalWrite(
    SENSOR_PWR,
    LOW
  );

  sonicSerial.end();

  pinMode(
    SONIC_MODE_PIN,
    INPUT_ANALOG
  );

  pinMode(
    SONIC_UART_RX,
    INPUT_ANALOG
  );


  // Restore LoRa RF switching pins.
  pinMode(
    LORA_RXEN,
    OUTPUT
  );

  pinMode(
    LORA_TXEN,
    OUTPUT
  );

  digitalWrite(
    LORA_RXEN,
    HIGH
  );

  digitalWrite(
    LORA_TXEN,
    LOW
  );
}
