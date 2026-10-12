// ============================================================
// SOIL NODE — Farm Truffle Orchard Monitor
// Hardware : PJ26 (STM32F103C8T6 + DX-LR20 LLCC68)
// Sensor   : RS485 7-in-1 Soil Sensor via MAX485/MAX3485
//
// Board Package:
// STM32duino
// Board: Generic STM32F1 Series
// Board part: BluePill F103CB (or C8 with 128k)
//
// REQUIRED Tools menu settings:
// - USB support (if available): None
// - U(S)ART support: Enabled (generic 'Serial')
// - Optimize: Smallest (-Os default) or Smallest with LTO
//   Debug/status output uses the core's built-in Serial1 object
//   (USART1, PA9/PA10) — already defined by this core, used as-is.
//   RS485 uses a custom "rs485Serial" object bound to USART3
//   (PB10/PB11) — see note near its declaration below for why
//   it's not called "Serial3".
//
// Libraries:
// RadioLib
// ArduinoJson
// ============================================================

#include <RadioLib.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

// ============================================================
// FIRMWARE VERSION — LR20 positive-side power switching build
//
// V2.2.1-LR20 pH diagnostics/calibration update (source: V2.2.0-LR20)
// - Manufacturer map confirmed: pH measurement register 0x0003, scale x0.1.
// - Reads/reports manufacturer PH offset register 0x0053 (signed, scale 1).
// - Adds ph_raw, ph_sensor and software-corrected ph telemetry.
// - Adds remote commands: read_ph_config, read_raw_registers,
//   set_ph_offset, reset_ph_offset, set_ph_software, reset_ph_software.
// - Manufacturer offset writes use Modbus function 0x06 plus read-back verify.
// - Optional node-side slope/offset correction is stored in STM32 EEPROM.
// - Existing LR20 radio, ACK/retry, sensor recovery and STOP sleep preserved.
//
// V2.2.0-LR20 reliability update (source: V2.1.0-LR20)
// - Requests a gateway delivery ACK for every uplink.
// - Retries the SAME cycle with random backoff when ACK is missed.
// - Keeps local RadioLib TX-error retry separate from gateway ACK.
// - Adds sensor power-cycle recovery for Halisense/MAX485 failures.
// - Adds delivery/recovery diagnostics to the next payload.
// - Reinitialises LoRa after repeated missed-ACK cycles.
// - Preserves LR20 LLCC68 pins, positive-side sensor switching,
//   STOP-mode sleep, RS485/UART mapping and existing measurements.
// - Compatible with Gateway V1.5.0; older gateways will not ACK,
//   so this firmware should be deployed only after Gateway V1.5.0.
// ============================================================
#define FW_VERSION "2.2.1-LR20"

// ============================================================
// LORA PINS
// Pin mapping confirmed against DX-SMART's own vendor firmware
// (UserConfig.h from their PJ26 LoRa example project), verified
// working on the tank node — which uses the same PJ26 base
// board, so the LoRa hat pin mapping should be identical
// regardless of which sensor is attached elsewhere on the
// board. The original values here (PB0/PA0/PB1) didn't match
// the vendor-confirmed set and are the likely cause of the
// persistent -2 (CHIP_NOT_FOUND) error.
// ============================================================
#define LORA_NSS      PA4
#define LORA_DIO1     PC15
#define LORA_RESET    PA3
#define LORA_BUSY     PA2

// PJ26 onboard RF switch, confirmed on the tank nodes.
#define LORA_RXEN     PA1
#define LORA_TXEN     PA0

// ============================================================
// DEBUG / STATUS SERIAL
// USART1, PA9(TX)/PA10(RX).
//
// IMPORTANT: Do NOT instantiate Serial1 here. The STM32duino
// Generic STM32F103C8 core already defines the global Serial1
// object. Defining another Uart Serial1(...) causes a linker
// "multiple definition of Serial1" error.
// ============================================================

// ============================================================
// RS485
// Uses USART3 on the STM32F103C8, mapped to PB10 (TX) / PB11
// (RX) — NOT USART1 (PA9/PA10). PA9/PA10 are reserved for
// Serial1 (debug/status) above; driving RS485 on those pins
// would collide with the programmer's USB-serial bridge and
// corrupts both the upload connection and the Serial Monitor
// output (confirmed on this hardware).
//
// Declared as "Uart" (the concrete class Serial1/Serial3 use in
// this core's Serial.h — "extern Uart Serial1;" etc.), not
// "HardwareSerial" — on this core, HardwareSerial is only the
// abstract ArduinoAPI base class and can't be instantiated
// directly; Uart is STM32's concrete implementation of it.
// ============================================================
#define RS485_RX    PB11
#define RS485_TX    PB10
#define RS485_DE    PB5
#define RS485_RE    PB5   // tie to same physical pin as RS485_DE
#define RS485_BAUD  4800

Uart rs485Serial(RS485_RX, RS485_TX);

// ============================================================
// SENSOR POWER MOSFET
// ============================================================
#define SENSOR_PWR  PB4

// ============================================================
// BATTERY ADC
// ============================================================
#define BAT_ADC     PB0

// ============================================================
// NODE CONFIG
// ============================================================
#define NODE_ID         "soil_1"

#define LORA_FREQ       915.0
#define LORA_BW         125.0
#define LORA_SF         9
#define LORA_CR         5
#define LORA_SYNC       0x12
#define LORA_PWR        22

#define SLEEP_SECONDS   3600

#define MODBUS_TIMEOUT       500
#define NUM_READINGS         3
#define MIN_VALID_READINGS   2
#define STABILISE_MS         500

// Gateway delivery acknowledgement / command window.
#define ACK_WAIT_MS              1800
#define DOWNLINK_WINDOW_MS       2500
#define DOWNLINK_POLL_MS         5

// Local RadioLib transmission errors are retried separately from
// end-to-end delivery ACK retries.
#define LORA_TX_ATTEMPTS         3
#define DELIVERY_ATTEMPTS        3
#define ACK_BACKOFF_MIN_MS       1000
#define ACK_BACKOFF_MAX_MS       5000

// If complete reporting cycles repeatedly miss the gateway ACK,
// force a full LoRa reinitialisation on the following cycle.
#define ACK_MISS_REINIT_CYCLES   3

// Sensor self-recovery. One normal read pass is followed by up to
// two complete sensor/MAX485 power cycles if the pass is unhealthy.
#define SENSOR_PASS_ATTEMPTS     3
#define SENSOR_RECOVERY_OFF_MS   1000

// ============================================================
// MODBUS
// ============================================================
#define SENSOR_ADDR     0x01
#define REG_START       0x0000
#define REG_COUNT       7

// Manufacturer TH-EC-PH-NPK map.
// 0x0003 is measured pH (raw x 0.1).
// 0x0053 is signed manufacturer PH offset; config tool declares scale 1.
#define PH_MEAS_REGISTER         0x0003
#define PH_MANUFACTURER_OFFSET   0x0053

// Local software pH calibration is deliberately separate from the
// sensor's own 0x0053 calibration. Defaults are neutral.
#define PH_CAL_MAGIC             0x50484331UL  // "PHC1"
#define PH_CAL_EEPROM_ADDR       0

// ============================================================
// RADIO
// Explicit slower SPI clock (1 MHz), same fix proven on the
// tank node — RadioLib's default clock was too fast relative to
// trace quality/MCU clock, corrupting the chip ID read and
// producing a false -2 (CHIP_NOT_FOUND) even with correct wiring.
// ============================================================
SPISettings loraSPISettings(1000000, MSBFIRST, SPI_MODE0);

Module loraModule(
  LORA_NSS,
  LORA_DIO1,
  LORA_RESET,
  LORA_BUSY,
  SPI,
  loraSPISettings
);

// DX-LR20 uses Semtech LLCC68, not the SX1262 fitted to DX-LR30.
LLCC68 radio(&loraModule);

// ============================================================
// DATA STRUCTURE / DIAGNOSTICS
// ============================================================
struct SoilData {
  float moisture;
  float temp;
  float ec;
  float ph;

  float nitrogen;
  float phosphorus;
  float potassium;

  float batteryVolts;
  int   batteryPct;

  uint8_t sampleCount;
  uint8_t sampleQuality;

  bool sensorOk;
  bool commOk;
  bool dataOk;
  bool valid;
};

// Runtime diagnostics are intentionally RAM-only. They are useful
// during a powered session without creating needless flash/EEPROM
// writes every hour.
uint32_t cycleCount = 0;

bool lastTxKnown = false;
bool lastTxOk = false;

// End-to-end delivery diagnostics. These describe the PREVIOUS
// completed uplink because the current ACK outcome is only known
// after the JSON packet has already been transmitted.
bool lastAckKnown = false;
bool lastAckOk = false;
uint8_t lastDeliveryAttempts = 0;
uint32_t ackMissTotal = 0;
uint8_t consecutiveAckMissCycles = 0;
uint32_t radioReinitTotal = 0;

// Sensor recovery diagnostics for the current/previous measurement.
uint8_t sensorRetriesThisCycle = 0;
uint32_t sensorRestartTotal = 0;
uint32_t sensorFailedCycles = 0;

bool lastRxKnown = false;
float lastRxRSSI = 0.0f;
float lastRxSNR = 0.0f;

// pH diagnostics and optional node-side software calibration.
struct PhSoftwareCalibration {
  uint32_t magic;
  float slope;
  float offset;
};

PhSoftwareCalibration phSoftwareCal = {
  PH_CAL_MAGIC,
  1.0f,
  0.0f
};

bool phManufacturerOffsetKnown = false;
int16_t phManufacturerOffsetRaw = 0;

bool phMeasurementKnown = false;
uint16_t phRaw = 0;
float phSensor = 0.0f;
float phCorrected = 0.0f;

bool radioReady = false;
bool rs485Started = false;

// DIO1 interrupt flag for RadioLib non-blocking receive.
volatile bool radioIrqFlag = false;

void radioDio1ISR() {
  radioIrqFlag = true;
}

// ============================================================
// pH SOFTWARE CALIBRATION STORAGE
// ============================================================
bool phSoftwareCalibrationSane(
  const PhSoftwareCalibration& c
) {
  if (c.magic != PH_CAL_MAGIC)
    return false;

  if (!isfinite(c.slope) || !isfinite(c.offset))
    return false;

  // Broad safety bounds: enough for real correction while rejecting
  // corrupt EEPROM values.
  if (c.slope < 0.25f || c.slope > 4.0f)
    return false;

  if (c.offset < -14.0f || c.offset > 14.0f)
    return false;

  return true;
}

void loadPhSoftwareCalibration() {

  PhSoftwareCalibration stored;
  EEPROM.get(
    PH_CAL_EEPROM_ADDR,
    stored
  );

  if (phSoftwareCalibrationSane(stored)) {
    phSoftwareCal = stored;
    Serial1.println("[PH] Loaded software calibration from EEPROM");
  } else {
    phSoftwareCal.magic = PH_CAL_MAGIC;
    phSoftwareCal.slope = 1.0f;
    phSoftwareCal.offset = 0.0f;
    Serial1.println("[PH] Using neutral software calibration");
  }
}

void savePhSoftwareCalibration() {

  phSoftwareCal.magic =
    PH_CAL_MAGIC;

  EEPROM.put(
    PH_CAL_EEPROM_ADDR,
    phSoftwareCal
  );
}

float applyPhSoftwareCalibration(
  float sensorPh
) {
  return
    sensorPh * phSoftwareCal.slope +
    phSoftwareCal.offset;
}

// ============================================================
// SETUP
// ============================================================
void setup() {

  Serial1.begin(115200);

  analogReadResolution(12);
  pinMode(BAT_ADC, INPUT_ANALOG);

  // Seed ACK retry jitter from ADC noise + startup timing.
  randomSeed(((uint32_t)analogRead(BAT_ADC) << 16) ^ micros());

  loadPhSoftwareCalibration();


  // Keep the switched sensor rail off and every RS485 logic pin
  // high-impedance. This prevents the STM32 UART idle-HIGH level
  // from phantom-powering an unpowered MAX485 through its DI pin.
  digitalWrite(SENSOR_PWR, LOW);
  pinMode(SENSOR_PWR, OUTPUT);
  pinMode(RS485_TX, INPUT_ANALOG);
  pinMode(RS485_RX, INPUT_ANALOG);
  pinMode(RS485_DE, INPUT_ANALOG);

  #if RS485_RE != RS485_DE
    pinMode(RS485_RE, INPUT_ANALOG);
  #endif

  // PJ26 onboard LoRa RF switch: receive/idle state.
  pinMode(LORA_RXEN, OUTPUT);
  pinMode(LORA_TXEN, OUTPUT);
  digitalWrite(LORA_RXEN, HIGH);
  digitalWrite(LORA_TXEN, LOW);

  radioReady = initLoRa();

  // DIO1 only sets a flag in the ISR. Radio processing remains in
  // normal code inside checkDownlink().
  radio.setDio1Action(radioDio1ISR);

  Serial1.println();
  Serial1.println("==================================");
  Serial1.println("SOIL NODE STARTED");
  Serial1.println("FW: " FW_VERSION);
  Serial1.println("NODE: " NODE_ID);
  Serial1.println("==================================");
}

// ============================================================
// LOOP
// ============================================================
void loop() {

  cycleCount++;

  Serial1.println("\n[SOIL NODE] Measurement Cycle");
  Serial1.print("[CYCLE] ");
  Serial1.println(cycleCount);

  SoilData data = readSoilSensor();

  String payload = buildPayload(data);

  Serial1.println(payload);

  transmitLoRaReliable(payload);

  if (data.sensorOk) {
    Serial1.println("[OK] Soil Read Success");
  } else if (!data.commOk) {
    Serial1.println("[ERROR] Soil sensor communication failed — battery-only packet sent");
  } else {
    Serial1.println("[ERROR] Soil sensor data invalid/insufficient — battery-only packet sent");
  }

  Serial1.print("[SLEEP] ");
  Serial1.print(SLEEP_SECONDS);
  Serial1.println(" seconds");
  Serial1.flush();

  // Put the LLCC68 and the external RF switch into their lowest
  // power state before the STM32 enters STOP mode.
  if (radioReady)
    radio.sleep();

  digitalWrite(LORA_RXEN, LOW);
  digitalWrite(LORA_TXEN, LOW);

  stm32SleepSeconds(SLEEP_SECONDS);
}

// ============================================================
// SENSOR POWER
// ============================================================
void powerSensorOn() {

  // PB4 drives the Pololu positive-side switch. VOUT supplies both
  // the 5 V soil probe and the 5 V MAX485 module.
  digitalWrite(SENSOR_PWR, HIGH);

  delay(STABILISE_MS);

  // Configure and start RS485 only after its switched supply is stable.
  pinMode(RS485_DE, OUTPUT);
  digitalWrite(RS485_DE, LOW);

  #if RS485_RE != RS485_DE
    pinMode(RS485_RE, OUTPUT);
    digitalWrite(RS485_RE, LOW);
  #endif

  rs485Serial.begin(RS485_BAUD);
  rs485Started = true;

  delay(50);

  while (rs485Serial.available())
    rs485Serial.read();
}

void powerSensorOff() {

  // Stop all MCU drive into the MAX485 before removing its 5 V.
  // Otherwise the UART/control pins can back-power the transceiver.
  if (rs485Started) {
    rs485ReceiveMode();
    delay(2);
    rs485Serial.end();
    rs485Started = false;
  }

  pinMode(RS485_TX, INPUT_ANALOG);
  pinMode(RS485_RX, INPUT_ANALOG);
  pinMode(RS485_DE, INPUT_ANALOG);

  #if RS485_RE != RS485_DE
    pinMode(RS485_RE, INPUT_ANALOG);
  #endif

  digitalWrite(SENSOR_PWR, LOW);

  delay(5);
}

// ============================================================
// SENSOR READ
// ============================================================

// Conservative physical sanity checks. These reject impossible
// moisture/temperature/pH values without inventing agronomic limits
// for EC or N/P/K, whose usable range depends heavily on the probe
// and soil. CRC/address/function checks are handled separately.
bool soilSampleSane(
  float moisture,
  float temp,
  float ph) {

  if (moisture < 0.0f || moisture > 100.0f)
    return false;

  if (temp < -40.0f || temp > 85.0f)
    return false;

  if (ph < 0.0f || ph > 14.0f)
    return false;

  return true;
}

// With three requested samples:
// - 3 valid samples -> median (excellent outlier rejection)
// - 2 valid samples -> average of the two
// - 1 valid sample  -> value retained for diagnostics, but the
//                     overall sensor result will not be accepted.
float filteredValue(float *values, uint8_t count) {

  if (count == 0)
    return 0.0f;

  if (count == 1)
    return values[0];

  if (count == 2)
    return (values[0] + values[1]) * 0.5f;

  // Median of first three samples.
  float a = values[0];
  float b = values[1];
  float c = values[2];

  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }

  return b;
}

SoilData readSoilSensor() {

  sensorRetriesThisCycle = 0;

  SoilData best = {
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    0.0f, 0,
    0, 0,
    false, false, false, true
  };

  uint8_t bestGoodReadings = 0;
  uint8_t bestCommReadings = 0;

  for (uint8_t pass = 1; pass <= SENSOR_PASS_ATTEMPTS; pass++) {

    float samples[7][NUM_READINGS] = {{0}};
    uint8_t goodReadings = 0;
    uint8_t commReadings = 0;

    if (pass > 1) {
      sensorRetriesThisCycle++;
      sensorRestartTotal++;

      Serial1.print("[SENSOR] Recovery power cycle ");
      Serial1.print(pass - 1);
      Serial1.print("/");
      Serial1.println(SENSOR_PASS_ATTEMPTS - 1);

      delay(SENSOR_RECOVERY_OFF_MS);
    }

    powerSensorOn();

    for (int i = 0; i < NUM_READINGS; i++) {

      uint16_t regs[REG_COUNT];

      if (modbusReadRegisters(
            SENSOR_ADDR,
            REG_START,
            REG_COUNT,
            regs)) {

        commReadings++;

        int16_t tempRaw = (int16_t)regs[1];

        float moisture = regs[0] / 10.0f;
        float temp     = tempRaw / 10.0f;
        float ec       = (float)regs[2];
        float ph       = regs[3] / 10.0f;
        float n        = (float)regs[4];
        float p        = (float)regs[5];
        float k        = (float)regs[6];

        if (soilSampleSane(moisture, temp, ph)) {

          samples[0][goodReadings] = moisture;
          samples[1][goodReadings] = temp;
          samples[2][goodReadings] = ec;
          samples[3][goodReadings] = ph;
          samples[4][goodReadings] = n;
          samples[5][goodReadings] = p;
          samples[6][goodReadings] = k;

          goodReadings++;
        }
      }

      if (i < NUM_READINGS - 1)
        delay(200);
    }

    // Read manufacturer PH offset while the sensor rail is still on.
    int16_t currentPhOffset = 0;

    if (readManufacturerPhOffset(currentPhOffset)) {
      phManufacturerOffsetRaw = currentPhOffset;
      phManufacturerOffsetKnown = true;
    }

    powerSensorOff();

    SoilData candidate = {
      0.0f, 0.0f, 0.0f, 0.0f,
      0.0f, 0.0f, 0.0f,
      0.0f, 0,
      goodReadings,
      (uint8_t)((goodReadings * 100U) / NUM_READINGS),
      (commReadings > 0) && (goodReadings >= MIN_VALID_READINGS),
      (commReadings > 0),
      (goodReadings >= MIN_VALID_READINGS),
      true
    };

    if (goodReadings > 0) {
      candidate.moisture   = filteredValue(samples[0], goodReadings);
      candidate.temp       = filteredValue(samples[1], goodReadings);
      candidate.ec         = filteredValue(samples[2], goodReadings);
      candidate.ph         = filteredValue(samples[3], goodReadings);
      candidate.nitrogen   = filteredValue(samples[4], goodReadings);
      candidate.phosphorus = filteredValue(samples[5], goodReadings);
      candidate.potassium  = filteredValue(samples[6], goodReadings);
    }

    // Keep the healthiest pass for diagnostics if all attempts fail.
    if (
      goodReadings > bestGoodReadings ||
      (
        goodReadings == bestGoodReadings &&
        commReadings > bestCommReadings
      )
    ) {
      best = candidate;
      bestGoodReadings = goodReadings;
      bestCommReadings = commReadings;
    }

    if (candidate.sensorOk) {
      best = candidate;

      if (pass > 1) {
        Serial1.print("[SENSOR] Recovered on pass ");
        Serial1.println(pass);
      }

      break;
    }

    Serial1.print("[SENSOR] Pass ");
    Serial1.print(pass);
    Serial1.print(" unhealthy: comm=");
    Serial1.print(commReadings);
    Serial1.print(" valid=");
    Serial1.println(goodReadings);
  }

  if (!best.sensorOk)
    sensorFailedCycles++;

  // Battery is independent of the switched sensor rail.
  best.batteryVolts = readBatteryVoltage();
  best.batteryPct = voltageToPercent(best.batteryVolts);

  if (best.sensorOk) {

    phSensor =
      best.ph;

    phRaw =
      (uint16_t)round(
        phSensor * 10.0f
      );

    phCorrected =
      applyPhSoftwareCalibration(
        phSensor
      );

    phMeasurementKnown =
      true;

    // Keep the existing dashboard "ph" field, but from V2.2.1 it is
    // explicitly the node-side corrected value. ph_sensor and ph_raw
    // are also transmitted so the original measurement is never hidden.
    best.ph =
      phCorrected;
  } else {

    phMeasurementKnown =
      false;
  }

  return best;
}

// ============================================================
// RS485 DIRECTION CONTROL
// Drives DE and RE together. DE is active-HIGH (enables the
// driver/TX side), RE is active-LOW (enables the receiver/RX
// side) on virtually all MAX485/MAX3485 parts, so one GPIO
// toggling HIGH/LOW correctly puts the transceiver into TX or
// RX mode whether DE/RE are tied on the board or wired together
// here.
// ============================================================
void rs485TransmitMode() {
  digitalWrite(RS485_DE, HIGH);
  #if RS485_RE != RS485_DE
    digitalWrite(RS485_RE, HIGH);
  #endif
}

void rs485ReceiveMode() {
  digitalWrite(RS485_DE, LOW);
  #if RS485_RE != RS485_DE
    digitalWrite(RS485_RE, LOW);
  #endif
}

// ============================================================
// MODBUS FC03
// ============================================================
bool modbusReadRegisters(
  uint8_t addr,
  uint16_t reg,
  uint8_t count,
  uint16_t* out) {

  uint8_t request[8];

  request[0] = addr;
  request[1] = 0x03;

  request[2] = reg >> 8;
  request[3] = reg & 0xFF;

  request[4] = 0x00;
  request[5] = count;

  uint16_t crc = modbusCRC(request, 6);

  request[6] = crc & 0xFF;
  request[7] = crc >> 8;

  while (rs485Serial.available())
    rs485Serial.read();

  rs485TransmitMode();

  delayMicroseconds(100);

  rs485Serial.write(request, 8);

  rs485Serial.flush();

  delayMicroseconds(100);

  rs485ReceiveMode();

  int expectedLen = 5 + (count * 2);

  uint8_t response[32];

  int received = 0;

  unsigned long start = millis();

  while (
    millis() - start < MODBUS_TIMEOUT &&
    received < expectedLen) {

    if (rs485Serial.available()) {

      response[received++] = rs485Serial.read();

    }
  }

  if (received < expectedLen)
    return false;

  if (response[0] != addr)
    return false;

  if (response[1] != 0x03)
    return false;

  if (response[2] != count * 2)
    return false;

  uint16_t calcCRC =
    modbusCRC(response, received - 2);

  uint16_t rxCRC =
    response[received - 2] |
    (response[received - 1] << 8);

  if (calcCRC != rxCRC)
    return false;

  for (int i = 0; i < count; i++) {

    out[i] =
      (response[3 + i * 2] << 8) |
      response[4 + i * 2];
  }

  return true;
}

// ============================================================
// MODBUS WRITE SINGLE REGISTER (function 0x06)
// ============================================================
bool modbusWriteSingleRegister(
  uint8_t addr,
  uint16_t reg,
  uint16_t value
) {

  uint8_t request[8];

  request[0] = addr;
  request[1] = 0x06;
  request[2] = reg >> 8;
  request[3] = reg & 0xFF;
  request[4] = value >> 8;
  request[5] = value & 0xFF;

  uint16_t crc =
    modbusCRC(request, 6);

  request[6] = crc & 0xFF;
  request[7] = crc >> 8;

  while (rs485Serial.available())
    rs485Serial.read();

  rs485TransmitMode();
  delayMicroseconds(100);

  rs485Serial.write(
    request,
    sizeof(request)
  );

  rs485Serial.flush();
  delayMicroseconds(100);

  rs485ReceiveMode();

  uint8_t response[8];
  uint8_t received = 0;
  unsigned long started = millis();

  while (
    millis() - started < MODBUS_TIMEOUT &&
    received < sizeof(response)
  ) {

    if (rs485Serial.available())
      response[received++] = rs485Serial.read();
  }

  if (received != sizeof(response))
    return false;

  for (uint8_t i = 0; i < 6; i++) {
    if (response[i] != request[i])
      return false;
  }

  uint16_t calcCRC =
    modbusCRC(response, 6);

  uint16_t rxCRC =
    response[6] |
    (response[7] << 8);

  return calcCRC == rxCRC;
}

bool readManufacturerPhOffset(
  int16_t& value
) {

  uint16_t raw = 0;

  if (!modbusReadRegisters(
        SENSOR_ADDR,
        PH_MANUFACTURER_OFFSET,
        1,
        &raw
      )) {
    return false;
  }

  value = (int16_t)raw;
  return true;
}

bool readRawMeasurementRegisters(
  uint16_t* regs
) {
  return modbusReadRegisters(
    SENSOR_ADDR,
    REG_START,
    REG_COUNT,
    regs
  );
}

// ============================================================
// CRC16 MODBUS
// ============================================================
uint16_t modbusCRC(uint8_t* buf, int len) {

  uint16_t crc = 0xFFFF;

  for (int i = 0; i < len; i++) {

    crc ^= buf[i];

    for (int j = 0; j < 8; j++) {

      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }

  return crc;
}

// ============================================================
// LOW-POWER SLEEP (STM32 STOP mode, RTC-driven wake)
//
// IMPORTANT: this does NOT use the IWDG as a wake source. On
// STM32F1 the IWDG keeps counting in STOP mode, and once
// started it cannot be paused — its expiry forces a full chip
// RESET, not a clean wake-up (setup() would rerun from scratch
// every cycle, re-initialising LoRa constantly and breaking the
// "sleep 3600s between readings" behaviour). Using it as a
// sleep timer is a common mistake; the correct STOP-mode wake
// source is the RTC wakeup timer, which resumes execution right
// after HAL_PWR_EnterSTOPMode() instead of resetting the chip.
//
// The RTC wakeup counter on STM32F1's RTC is a 32-bit counter
// clocked (typically) from LSI/LSE, so periods up to SLEEP_SECONDS
// = 3600 are done in ONE stop cycle rather than chunked loops.
//
// NOTE: waking from STOP mode drops the system clock to HSI, so
// SystemClock_Config() must be called again immediately after
// waking, before anything (Serial3/RS485, ADC, etc.) is used, or
// peripherals will run at the wrong clock and produce garbage.
// ============================================================
#include <STM32RTC.h>

STM32RTC& rtc = STM32RTC::getInstance();
volatile bool rtcWoke = false;

void rtcAlarmCallback(void* data) {
  rtcWoke = true;
}

void stm32SleepSeconds(uint32_t seconds) {

  rtcWoke = false;

  rtc.begin();  // uses LSI by default on STM32duino F1 configs

  // Set wakeup alarm to fire once, `seconds` from now.
  rtc.attachInterrupt(rtcAlarmCallback);
  rtc.setAlarmEpoch(rtc.getEpoch() + seconds, STM32RTC::MATCH_DHHMMSS);

  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

  // Clocks drop to HSI in STOP mode — restore full config before
  // touching any peripheral (RS485/Serial3, ADC, LoRa SPI, etc.)
  SystemClock_Config();

  rtc.detachInterrupt();

  // Restore only always-powered peripherals after STOP mode. Keep
  // the sensor rail off and RS485 pins high-impedance until the next
  // call to powerSensorOn().
  pinMode(BAT_ADC, INPUT_ANALOG);
  digitalWrite(SENSOR_PWR, LOW);
  pinMode(SENSOR_PWR, OUTPUT);
  pinMode(RS485_TX, INPUT_ANALOG);
  pinMode(RS485_RX, INPUT_ANALOG);
  pinMode(RS485_DE, INPUT_ANALOG);

  #if RS485_RE != RS485_DE
    pinMode(RS485_RE, INPUT_ANALOG);
  #endif

  pinMode(LORA_RXEN, OUTPUT);
  pinMode(LORA_TXEN, OUTPUT);
  digitalWrite(LORA_RXEN, HIGH);
  digitalWrite(LORA_TXEN, LOW);

  if (radioReady) {
    int state = radio.standby();

    if (state != RADIOLIB_ERR_NONE)
      radioReady = false;
  }
}

// ============================================================
// BATTERY
// Voltage divider: 1k (battery side) + 2.2k (GND side), tap at
// PB0. Battery/MPPT negative must share PJ26 GND.
// Divider ratio = (1000+2200)/2200 = 1.4545.
// ============================================================
#define BATTERY_DIVIDER_RATIO   1.4545f

struct BatteryPoint {
  float volts;
  int pct;
};

// Approximate Li-ion/LiPo resting-voltage state of charge.
// Intended for remote health monitoring, not precision coulomb counting.
const BatteryPoint BATTERY_TABLE[] = {
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

const uint8_t BATTERY_TABLE_COUNT =
  sizeof(BATTERY_TABLE) / sizeof(BATTERY_TABLE[0]);

int voltageToPercent(float volts) {

  if (volts >= BATTERY_TABLE[0].volts)
    return 100;

  if (volts <= BATTERY_TABLE[BATTERY_TABLE_COUNT - 1].volts)
    return 0;

  for (uint8_t i = 0; i < BATTERY_TABLE_COUNT - 1; i++) {

    float vHigh = BATTERY_TABLE[i].volts;
    float vLow  = BATTERY_TABLE[i + 1].volts;

    if (volts <= vHigh && volts >= vLow) {

      float span = vHigh - vLow;
      float frac = (span > 0.0f) ? (volts - vLow) / span : 0.0f;

      int pctHigh = BATTERY_TABLE[i].pct;
      int pctLow  = BATTERY_TABLE[i + 1].pct;

      return pctLow + (int)round(frac * (pctHigh - pctLow));
    }
  }

  return 0;
}

float readBatteryVoltage() {

  analogReadResolution(12);
  pinMode(BAT_ADC, INPUT_ANALOG);

  // Seed ACK retry jitter from ADC noise + startup timing.
  randomSeed(((uint32_t)analogRead(BAT_ADC) << 16) ^ micros());

  delay(10);

  // Discard initial conversions after wake/reconfiguration.
  for (uint8_t i = 0; i < 8; i++) {
    analogRead(BAT_ADC);
    delay(2);
  }

  // Average multiple conversions to reduce ADC noise.
  uint32_t total = 0;
  const uint8_t adcSamples = 32;

  for (uint8_t i = 0; i < adcSamples; i++) {
    total += analogRead(BAT_ADC);
    delay(2);
  }

  float raw = total / (float)adcSamples;

  float volts =
      (raw / 4095.0f)
    * 3.3f
    * BATTERY_DIVIDER_RATIO;

  return volts;
}

// ============================================================
// JSON PAYLOAD
// ============================================================
String buildPayload(const SoilData& d) {

  StaticJsonDocument<512> doc;

  doc["fw"] = FW_VERSION;
  doc["node"] = NODE_ID;
  doc["cycle"] = cycleCount;

  // Gateway V1.5.0 uses this opt-in flag so legacy nodes remain
  // backward-compatible while updated nodes receive delivery ACKs.
  doc["ack_req"] = true;

  doc["samples"] = d.sampleCount;
  doc["quality"] = d.sampleQuality;

  doc["sensor_ok"] = d.sensorOk;
  doc["comm_ok"] = d.commOk;
  doc["data_ok"] = d.dataOk;

  if (lastTxKnown)
    doc["last_tx"] = lastTxOk;

  if (lastAckKnown) {
    doc["last_ack"] = lastAckOk;
    doc["last_tx_attempts"] = lastDeliveryAttempts;
  }

  doc["ack_miss_total"] = ackMissTotal;
  doc["ack_miss_cycles"] = consecutiveAckMissCycles;
  doc["radio_reinit_total"] = radioReinitTotal;

  doc["sensor_retries"] = sensorRetriesThisCycle;
  doc["sensor_restarts_total"] = sensorRestartTotal;
  doc["sensor_failed_cycles"] = sensorFailedCycles;

  if (!d.sensorOk) {
    if (!d.commOk)
      doc["fault"] = "sensor_comm";
    else
      doc["fault"] = "sensor_data";
  }

  if (lastRxKnown) {
    doc["last_rx_rssi"] = serialized(String(lastRxRSSI, 1));
    doc["last_rx_snr"] = serialized(String(lastRxSNR, 1));
  }

  // Battery is always reported, even when the soil sensor fails.
  doc["batv"] = serialized(String(d.batteryVolts, 2));
  doc["bat"] = d.batteryPct;

  if (d.sensorOk) {

    // Dashboard-compatible field name.
    doc["moisture"] = serialized(String(d.moisture, 1));
    doc["temp"] = serialized(String(d.temp, 1));
    doc["ec"] = (int)round(d.ec);
    doc["ph"] = serialized(String(d.ph, 2));

    if (phMeasurementKnown) {
      doc["ph_raw"] = phRaw;
      doc["ph_sensor"] = serialized(String(phSensor, 1));
    }

    if (phManufacturerOffsetKnown)
      doc["ph_mfg_off"] = phManufacturerOffsetRaw;
    doc["n"] = (int)round(d.nitrogen);
    doc["p"] = (int)round(d.phosphorus);
    doc["k"] = (int)round(d.potassium);
  }

  String out;
  serializeJson(doc, out);
  return out;
}

// ============================================================
// LORA
// ============================================================
bool initLoRa() {

  for (uint8_t attempt = 1; attempt <= LORA_TX_ATTEMPTS; attempt++) {

    Serial1.print("[LoRa] Initialising (attempt ");
    Serial1.print(attempt);
    Serial1.print(")... ");

    int state = radio.begin(
      LORA_FREQ,
      LORA_BW,
      LORA_SF,
      LORA_CR,
      LORA_SYNC,
      LORA_PWR,
      8,      // preamble length
      0.0,    // XTAL module: no TCXO voltage
      false   // use the module's DC-DC regulator
    );

    if (state == RADIOLIB_ERR_NONE) {
      Serial1.println("OK");
      return true;
    }

    Serial1.print("FAILED: ");
    Serial1.println(state);
    delay(150);
  }

  return false;
}

bool transmitOneLoRaPacket(const String& payload) {

  if (!radioReady)
    radioReady = initLoRa();

  if (!radioReady)
    return false;

  for (uint8_t attempt = 1; attempt <= LORA_TX_ATTEMPTS; attempt++) {

    digitalWrite(LORA_RXEN, LOW);
    digitalWrite(LORA_TXEN, HIGH);

    int state = radio.transmit(payload);

    digitalWrite(LORA_TXEN, LOW);
    digitalWrite(LORA_RXEN, HIGH);

    if (state == RADIOLIB_ERR_NONE)
      return true;

    Serial1.print("[LoRa] Local TX attempt ");
    Serial1.print(attempt);
    Serial1.print(" failed: ");
    Serial1.println(state);

    radio.standby();
    delay(150);
  }

  radioReady = false;
  return false;
}


// ============================================================
// GATEWAY FRAME PROCESSING
// Returns true only for the matching V1.5 delivery ACK.
// Existing soil "ping" downlinks are still recognised.
// ============================================================
// ============================================================
// REMOTE pH COMMAND RESPONSES
// ============================================================
void sendJsonCommandResponse(
  JsonDocument& response
) {

  String payload;
  serializeJson(
    response,
    payload
  );

  digitalWrite(LORA_RXEN, LOW);
  digitalWrite(LORA_TXEN, HIGH);


  int state =
    radio.transmit(payload);

  digitalWrite(LORA_TXEN, LOW);
  digitalWrite(LORA_RXEN, HIGH);
  radio.standby();

  Serial1.print("[PH CMD] Response TX: ");
  Serial1.println(
    state == RADIOLIB_ERR_NONE ?
    "OK" :
    "FAIL"
  );


}

void sendPhConfigResponse(
  const char* command,
  const char* status
) {

  StaticJsonDocument<256> response;

  response["node"] = NODE_ID;
  response["ack"] = command;
  response["ack_status"] = status;

  response["ph_sw_slope"] =
    serialized(
      String(
        phSoftwareCal.slope,
        4
      )
    );

  response["ph_sw_offset"] =
    serialized(
      String(
        phSoftwareCal.offset,
        3
      )
    );

  if (phManufacturerOffsetKnown)
    response["ph_mfg_off"] =
      phManufacturerOffsetRaw;

  if (phMeasurementKnown) {
    response["ph_raw"] = phRaw;
    response["ph_sensor"] =
      serialized(String(phSensor, 1));
    response["ph"] =
      serialized(String(phCorrected, 2));
  }

  sendJsonCommandResponse(response);
}

bool executePhCommand(
  JsonDocument& doc
) {

  const char* cmd =
    doc["cmd"] | "";

  if (
    strcmp(cmd, "read_ph_config") != 0 &&
    strcmp(cmd, "read_raw_registers") != 0 &&
    strcmp(cmd, "set_ph_offset") != 0 &&
    strcmp(cmd, "reset_ph_offset") != 0 &&
    strcmp(cmd, "set_ph_software") != 0 &&
    strcmp(cmd, "reset_ph_software") != 0
  ) {
    return false;
  }

  // Software-only commands do not need to power the RS485 probe.
  if (strcmp(cmd, "set_ph_software") == 0) {

    if (
      !doc["slope"].is<float>() &&
      !doc["slope"].is<double>()
    ) {
      sendPhConfigResponse(
        cmd,
        "missing_slope"
      );
      return true;
    }

    float slope =
      doc["slope"].as<float>();

    float offset =
      doc["offset"] | 0.0f;

    PhSoftwareCalibration candidate = {
      PH_CAL_MAGIC,
      slope,
      offset
    };

    if (!phSoftwareCalibrationSane(candidate)) {
      sendPhConfigResponse(
        cmd,
        "rejected_range"
      );
      return true;
    }

    phSoftwareCal =
      candidate;

    savePhSoftwareCalibration();

    sendPhConfigResponse(
      cmd,
      "ok"
    );

    return true;
  }

  if (strcmp(cmd, "reset_ph_software") == 0) {

    phSoftwareCal.magic =
      PH_CAL_MAGIC;

    phSoftwareCal.slope =
      1.0f;

    phSoftwareCal.offset =
      0.0f;

    savePhSoftwareCalibration();

    sendPhConfigResponse(
      cmd,
      "ok"
    );

    return true;
  }

  powerSensorOn();

  if (strcmp(cmd, "read_ph_config") == 0) {

    int16_t sensorOffset = 0;

    if (readManufacturerPhOffset(sensorOffset)) {
      phManufacturerOffsetRaw =
        sensorOffset;

      phManufacturerOffsetKnown =
        true;

      powerSensorOff();

      sendPhConfigResponse(
        cmd,
        "ok"
      );
    } else {
      powerSensorOff();

      sendPhConfigResponse(
        cmd,
        "sensor_read_failed"
      );
    }

    return true;
  }

  if (strcmp(cmd, "read_raw_registers") == 0) {

    uint16_t regs[REG_COUNT] = {0};
    bool ok =
      readRawMeasurementRegisters(
        regs
      );

    int16_t sensorOffset = 0;
    bool offsetOk =
      readManufacturerPhOffset(
        sensorOffset
      );

    if (offsetOk) {
      phManufacturerOffsetRaw =
        sensorOffset;
      phManufacturerOffsetKnown =
        true;
    }

    powerSensorOff();

    StaticJsonDocument<320> response;

    response["node"] = NODE_ID;
    response["ack"] = cmd;
    response["ack_status"] =
      ok ? "ok" : "sensor_read_failed";

    if (ok) {
      JsonArray raw =
        response.createNestedArray("raw");

      for (uint8_t i = 0; i < REG_COUNT; i++)
        raw.add(regs[i]);

      response["ph_raw"] =
        regs[3];

      response["ph_sensor"] =
        serialized(
          String(
            regs[3] / 10.0f,
            1
          )
        );
    }

    if (offsetOk)
      response["ph_mfg_off"] =
        sensorOffset;

    sendJsonCommandResponse(
      response
    );

    return true;
  }

  if (
    strcmp(cmd, "set_ph_offset") == 0 ||
    strcmp(cmd, "reset_ph_offset") == 0
  ) {

    int32_t requested = 0;

    if (strcmp(cmd, "set_ph_offset") == 0) {

      if (!doc["value"].is<int>()) {
        powerSensorOff();

        sendPhConfigResponse(
          cmd,
          "missing_value"
        );

        return true;
      }

      requested =
        doc["value"].as<int32_t>();
    }

    // Manufacturer tool describes this as a signed register with scale 1.
    // Use a deliberately conservative remote range. Wider values can be
    // enabled later if Halisense provides a different documented limit.
    if (
      requested < -20 ||
      requested > 20
    ) {
      powerSensorOff();

      sendPhConfigResponse(
        cmd,
        "rejected_range"
      );

      return true;
    }

    bool writeOk =
      modbusWriteSingleRegister(
        SENSOR_ADDR,
        PH_MANUFACTURER_OFFSET,
        (uint16_t)(int16_t)requested
      );

    int16_t verify = 0;
    bool readBackOk =
      readManufacturerPhOffset(
        verify
      );

    powerSensorOff();

    if (
      writeOk &&
      readBackOk &&
      verify == (int16_t)requested
    ) {
      phManufacturerOffsetRaw =
        verify;

      phManufacturerOffsetKnown =
        true;

      sendPhConfigResponse(
        cmd,
        "ok"
      );
    } else {
      sendPhConfigResponse(
        cmd,
        "verify_failed"
      );
    }

    return true;
  }

  powerSensorOff();
  return true;
}


bool processGatewayFrame(
  const String& received,
  uint32_t expectedCycle) {

  lastRxRSSI = radio.getRSSI();
  lastRxSNR = radio.getSNR();
  lastRxKnown = true;

  Serial1.print("[DOWNLINK] Received: ");
  Serial1.println(received);

  StaticJsonDocument<192> doc;
  DeserializationError err =
    deserializeJson(doc, received);

  if (err) {
    Serial1.println("[DOWNLINK] Ignored: malformed JSON");
    return false;
  }

  const char* ack = doc["ack"] | "";

  if (strcmp(ack, "uplink") == 0) {

    const char* node = doc["node"] | "";
    uint32_t seq = doc["seq"] | 0;

    if (
      strcmp(node, NODE_ID) == 0 &&
      seq == expectedCycle
    ) {
      Serial1.print("[ACK] Delivery confirmed for cycle ");
      Serial1.println(expectedCycle);
      return true;
    }

    Serial1.println("[ACK] Ignored: ACK did not match this node/cycle");
    return false;
  }

  const char* cmd = doc["cmd"] | "";
  const char* target = doc["target"] | "";

  if (
    target[0] != '\0' &&
    strcmp(target, NODE_ID) != 0
  ) {
    Serial1.println("[DOWNLINK] Ignored: command targets another node");
    return false;
  }

  if (executePhCommand(doc))
    return false;

  if (strcmp(cmd, "ping") == 0) {
    Serial1.println("[DOWNLINK] Ping received");
  } else if (cmd[0] != '\0') {
    Serial1.println("[DOWNLINK] Command received but not implemented in V2.2");
  }

  return false;
}


// ============================================================
// WAIT FOR ACK
// The node stays in interrupt-driven receive long enough to receive
// the gateway's immediate ACK. Any command arriving in that period
// is also processed.
// ============================================================
bool waitForDeliveryAck(
  uint32_t expectedCycle,
  uint32_t timeoutMs) {

  radio.standby();

  digitalWrite(LORA_TXEN, LOW);
  digitalWrite(LORA_RXEN, HIGH);

  radioIrqFlag = false;

  int state = radio.startReceive();

  if (state != RADIOLIB_ERR_NONE) {
    Serial1.print("[ACK] Failed to start RX: ");
    Serial1.println(state);
    return false;
  }

  unsigned long start = millis();

  while (millis() - start < timeoutMs) {

    if (!radioIrqFlag) {
      delay(DOWNLINK_POLL_MS);
      continue;
    }

    radioIrqFlag = false;

    String received;
    state = radio.readData(received);

    // Re-arm receive immediately to minimise the gap before a possible
    // gateway command or another ACK frame.
    radio.startReceive();

    if (state != RADIOLIB_ERR_NONE) {
      Serial1.print("[ACK] Read error: ");
      Serial1.println(state);
      continue;
    }

    if (processGatewayFrame(received, expectedCycle)) {
      radio.standby();
      return true;
    }
  }

  radio.standby();
  return false;
}


// ============================================================
// POST-ACK COMMAND WINDOW
// Preserves the existing gateway -> node downlink opportunity.
// ============================================================
void listenForGatewayCommands(
  uint32_t expectedCycle,
  uint32_t windowMs) {

  if (!radioReady)
    return;

  radio.standby();

  digitalWrite(LORA_TXEN, LOW);
  digitalWrite(LORA_RXEN, HIGH);

  radioIrqFlag = false;

  int state = radio.startReceive();

  if (state != RADIOLIB_ERR_NONE) {
    Serial1.print("[DOWNLINK] Failed to start RX: ");
    Serial1.println(state);
    return;
  }

  unsigned long start = millis();

  while (millis() - start < windowMs) {

    if (!radioIrqFlag) {
      delay(DOWNLINK_POLL_MS);
      continue;
    }

    radioIrqFlag = false;

    String received;
    state = radio.readData(received);

    radio.startReceive();

    if (state != RADIOLIB_ERR_NONE) {
      Serial1.print("[DOWNLINK] Read error: ");
      Serial1.println(state);
      continue;
    }

    // Duplicate delivery ACKs are harmless; ping/other commands are
    // processed by the same parser.
    processGatewayFrame(received, expectedCycle);
  }

  radio.standby();
}


// ============================================================
// RELIABLE UPLINK
// - Same cycle number is reused for all retries.
// - Local RadioLib errors are retried first.
// - A successful local TX is not considered delivered until the
//   gateway ACKs NODE_ID + cycle.
// - Random backoff greatly reduces repeat collisions.
// ============================================================
void transmitLoRaReliable(String& payload) {

  // If several complete cycles have missed ACKs, rebuild the radio
  // state before attempting this cycle. This is a radio recovery,
  // not a sensor/PJ26 power cycle.
  if (consecutiveAckMissCycles >= ACK_MISS_REINIT_CYCLES) {

    Serial1.println("[ACK] Repeated misses: reinitialising LoRa");

    radioReady = false;
    radioReinitTotal++;

    if (!initLoRa()) {
      Serial1.println("[LoRa] Reinitialisation failed");
    } else {
      radioReady = true;
      radio.setDio1Action(radioDio1ISR);
    }
  }

  bool delivered = false;
  bool anyLocalTx = false;
  uint8_t deliveryAttemptsUsed = 0;

  for (
    uint8_t attempt = 1;
    attempt <= DELIVERY_ATTEMPTS;
    attempt++
  ) {

    deliveryAttemptsUsed = attempt;

    Serial1.print("[TX] Delivery attempt ");
    Serial1.print(attempt);
    Serial1.print("/");
    Serial1.println(DELIVERY_ATTEMPTS);

    bool localTxOk =
      transmitOneLoRaPacket(payload);

    lastTxKnown = true;
    lastTxOk = localTxOk;

    if (!localTxOk) {

      Serial1.println("[TX] Local radio TX failed");

    } else {

      anyLocalTx = true;

      Serial1.println("[TX] Local TX OK; waiting for gateway ACK");

      if (waitForDeliveryAck(cycleCount, ACK_WAIT_MS)) {
        delivered = true;
        break;
      }

      Serial1.println("[ACK] No matching gateway ACK");
    }

    if (attempt < DELIVERY_ATTEMPTS) {

      uint32_t backoff =
        (uint32_t)random(
          ACK_BACKOFF_MIN_MS,
          ACK_BACKOFF_MAX_MS + 1
        );

      Serial1.print("[ACK] Random retry backoff ");
      Serial1.print(backoff);
      Serial1.println(" ms");

      delay(backoff);
    }
  }

  lastAckKnown = true;
  lastAckOk = delivered;
  lastDeliveryAttempts = deliveryAttemptsUsed;

  if (delivered) {

    consecutiveAckMissCycles = 0;

    Serial1.println("[ACK] Uplink delivery confirmed");

    // Gateway V1.5 sends a queued command immediately after the ACK,
    // so retain the normal downlink window.
    listenForGatewayCommands(
      cycleCount,
      DOWNLINK_WINDOW_MS
    );

  } else {

    ackMissTotal++;

    if (consecutiveAckMissCycles < 255)
      consecutiveAckMissCycles++;

    Serial1.print("[ACK] DELIVERY FAILED; consecutive cycles=");
    Serial1.println(consecutiveAckMissCycles);

    // If no local transmission worked at all, force a radio rebuild
    // next cycle regardless of the ACK-miss threshold.
    if (!anyLocalTx)
      radioReady = false;
  }
}

