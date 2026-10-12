SOIL pH CALIBRATION / DIAGNOSTIC KEY
Farm Truffle Orchard Monitor
Firmware:
  Soil 1 LR20 : V2.2.1-LR20
  LR30 build  : V2.2.1-LR30

PURPOSE
-------
These builds are intended to be flashed before the farm visit so the pH
channel can be diagnosed and calibrated later from FarmPi without another
USB reflash.

MANUFACTURER INFORMATION USED
-----------------------------
The supplied Halisense/CWT TH-EC-PH-NPK material defines the main
measurement block as:

  0x0000  moisture       raw x 0.1
  0x0001  temperature    raw x 0.1 C
  0x0002  conductivity   raw x 1
  0x0003  pH             raw x 0.1
  0x0004  nitrogen       raw x 1
  0x0005  phosphorus     raw x 1
  0x0006  potassium      raw x 1

The manufacturer's configuration tool defines:

  register 0x0053 (decimal 83; PLC-style display may be 40084)
  name       : PH offset
  type       : signed
  scale      : 1
  writable   : yes

IMPORTANT:
The manufacturer's PH offset is therefore treated here as a SIGNED RAW
INTEGER. The firmware does NOT assume that +1 means +0.1 pH. The supplied
configuration-tool metadata says scale 1. Confirm real behaviour with
Halisense/reference testing before writing offsets to all probes.

NORMAL TELEMETRY
----------------
ph
  Final pH after optional node-side software correction.

ph_sensor
  pH reported by the Halisense measurement register before node-side
  software correction.

ph_raw
  Raw pH register value corresponding to 0x0003.
  Expected relationship from the manufacturer:
      ph_sensor = ph_raw / 10.0

ph_mfg_off
  Signed raw value currently read from manufacturer register 0x0053.
  This is the calibration stored in the Halisense probe itself.

Example:
  "ph_raw": 30
  "ph_sensor": 3.0
  "ph": 3.00
  "ph_mfg_off": 0

If ph_raw remains 30 in genuinely different pH conditions, DO NOT solve
that by adding an offset. First establish why the pH channel is not
responding.

REMOTE COMMAND FORMAT
---------------------
Commands are targeted to one node. The gateway should queue the complete
JSON packet for delivery when that sleeping node next reports.

1. READ CURRENT pH CONFIGURATION

  {"cmd":"read_ph_config","target":"soil_1"}

Response includes:
  ph_mfg_off
  ph_sw_slope
  ph_sw_offset
  ph_raw / ph_sensor / ph when a measurement is available


2. READ ALL SEVEN RAW MEASUREMENT REGISTERS

  {"cmd":"read_raw_registers","target":"soil_1"}

Typical response:
  {
    "node":"soil_1",
    "ack":"read_raw_registers",
    "ack_status":"ok",
    "raw":[766,164,185,30,8,13,30],
    "ph_raw":30,
    "ph_sensor":3.0,
    "ph_mfg_off":0
  }

Array order:
  raw[0] moisture
  raw[1] temperature
  raw[2] EC
  raw[3] pH
  raw[4] N
  raw[5] P
  raw[6] K


3. SET MANUFACTURER PH OFFSET (Halisense register 0x0053)

  {"cmd":"set_ph_offset","target":"soil_1","value":1}

'value' is a SIGNED RAW INTEGER, not a pH decimal.

The node:
  - powers the probe/MAX485
  - writes 0x0053 using Modbus function 0x06
  - reads 0x0053 back
  - only reports "ok" if read-back exactly matches

Remote safety range currently allowed:
  -20 through +20

Do not mass-apply a value until the behaviour of one probe has been
confirmed against a real reference test.


4. RESET MANUFACTURER PH OFFSET TO ZERO

  {"cmd":"reset_ph_offset","target":"soil_1"}

This writes signed raw zero to 0x0053 and verifies it by reading it back.


5. SET NODE-SIDE SOFTWARE CALIBRATION

  {"cmd":"set_ph_software","target":"soil_1","slope":1.0,"offset":0.0}

Calculation:
  ph = ph_sensor * slope + offset

This calibration is stored in STM32 EEPROM and survives restart/power loss.

Allowed software ranges:
  slope  : 0.25 to 4.0
  offset : -14.0 to +14.0

This layer does NOT alter the Halisense probe. It exists so a future
two-point calibration can correct both slope and offset even though the
manufacturer exposes only a PH offset register in the supplied material.


6. RESET NODE-SIDE SOFTWARE CALIBRATION

  {"cmd":"reset_ph_software","target":"soil_1"}

Returns to:
  slope  = 1.0
  offset = 0.0


RECOMMENDED FARM TEST
---------------------
1. Flash the new firmware.
2. Do NOT change either calibration layer initially.
3. Record:
     ph_raw
     ph_sensor
     ph_mfg_off
4. Confirm actual soil pH independently.
5. Test at least one probe in two or preferably three distinctly different
   known/reference pH conditions.
6. Watch ph_raw:
     - If it changes sensibly, the pH channel is responding and calibration
       may be meaningful.
     - If it remains fixed around 30 / pH 3.0, do not hide the fault with
       an offset.
7. If correction is required:
     - prefer confirmed manufacturer offset behaviour for a simple constant
       error, OR
     - use node-side slope + offset after two-point calibration.
8. Change one probe first, verify, then apply equivalent logic to others.

TWO-POINT SOFTWARE CALIBRATION
------------------------------
For sensor values X1, X2 and trusted reference values Y1, Y2:

  slope  = (Y2 - Y1) / (X2 - X1)
  offset = Y1 - slope * X1

Example only:
  sensor 4.0 -> reference 4.5
  sensor 7.0 -> reference 7.8

  slope  = (7.8 - 4.5) / (7.0 - 4.0) = 1.1
  offset = 4.5 - 1.1 * 4.0 = 0.1

Command:
  {"cmd":"set_ph_software","target":"soil_1","slope":1.1,"offset":0.1}

Do not calculate calibration from one point unless you have already proven
the sensor response slope is sound.

NODE IDs
--------
Current supplied builds:
  LR20 file is fixed to NODE_ID "soil_1".
  LR30 file is fixed to NODE_ID "soil_2".

For field deployment, keep a separate sketch folder for every physical
node. Soil 1 must remain the LR20 build. Soil 2/3/4 use LR30 hardware but
should each have their own fixed NODE_ID to avoid accidental duplicate IDs.
