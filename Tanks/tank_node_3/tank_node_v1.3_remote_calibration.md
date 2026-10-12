# Tank node v1.3 — remote calibration

Firmware file: `tank_node_v1.3.ino`

## What v1.3 adds

- The full and empty sensor distances, tank capacity, distance offset, and battery-divider ratio are saved in non-volatile memory.
- The settings survive normal sleep cycles and loss of battery power.
- The tank listens for a gateway command for three seconds after every hourly report.
- Every normal report includes the active settings as `full`, `empty`, `cap`, `off`, and `br`.
- A received command produces an immediate acknowledgement containing `ack` and `ack_status` plus the current tank reading.
- Invalid or unsafe values are rejected without changing the saved configuration.

The gateway firmware does not need changing. It already listens on `farm/node/command`, queues a command while the node sleeps, and transmits it immediately after that node's next report.

## First test on Tank 1

Flash v1.3 and check the serial monitor. The startup output should show:

```text
Firmware: 1.3.0
[CONFIG] full=135 mm, empty=2150 mm, capacity=22500 L, offset=0 mm, battery ratio=1.4380
```

The normal JSON report should also contain:

```json
"fw":"1.3.0","full":135,"empty":2150,"cap":22500,"off":0,"br":1.4380
```

## Sending commands from the FarmPi

SSH into the FarmPi and use the following commands. A command remains queued at the gateway until `tank_1` next reports. To test immediately, publish the command and then reset Tank 1.

Only one command can be queued for a node. Wait for its acknowledgement before sending the next one, because a newer command replaces an older queued command for the same node.

### Confirm communication and read the settings

```bash
mosquitto_pub -h localhost -t farm/node/command -m '{"node":"tank_1","cmd":"ping"}'
```

```bash
mosquitto_pub -h localhost -t farm/node/command -m '{"node":"tank_1","cmd":"get_config"}'
```

### Capture the full distance

Do this when the tank is at the chosen full level:

```bash
mosquitto_pub -h localhost -t farm/node/command -m '{"node":"tank_1","cmd":"capture_full"}'
```

The node uses the valid ultrasonic measurement from that same reporting cycle and saves it as the full distance.

### Capture the empty distance

Do this when the tank is at the chosen empty level:

```bash
mosquitto_pub -h localhost -t farm/node/command -m '{"node":"tank_1","cmd":"capture_empty"}'
```

### Enter measured values directly

Any one or more fields can be supplied. This example sets all fields:

```bash
mosquitto_pub -h localhost -t farm/node/command -m '{"node":"tank_1","cmd":"set_cal","full":135,"empty":2150,"capacity":22500,"offset":0,"bat_ratio":1.4380}'
```

Field meanings:

- `full`: sensor-to-water distance in millimetres when full
- `empty`: sensor-to-water distance in millimetres when empty
- `capacity`: usable tank capacity in litres
- `offset`: correction added to every measured distance, normally zero
- `bat_ratio`: battery-divider calibration ratio

For example, to change only the empty distance:

```bash
mosquitto_pub -h localhost -t farm/node/command -m '{"node":"tank_1","cmd":"set_cal","empty":2180}'
```

## Confirming success

A successful command produces fields similar to:

```json
"ack":"set_cal","ack_status":"ok"
```

Other possible results are:

- `invalid`: a value was outside its safe range, or empty was not at least 50 mm farther than full
- `no_sensor`: a capture command was received during a failed ultrasonic reading
- `unknown`: the command name was not recognised

The acknowledgement includes the recalculated `pct` and `vol`, so the FarmPi retained tank reading remains complete after calibration.

## Tank 2 and Tank 3

Change only `NODE_ID` before flashing each physical node:

```cpp
#define NODE_ID "tank_2"
```

or:

```cpp
#define NODE_ID "tank_3"
```

Then target that node name in the MQTT command. Each node stores its own calibration locally.

## Note about reflashing

The configuration survives power loss. Depending on the Arduino upload method and its erase setting, a later firmware flash may erase the emulated EEPROM. If that happens, the node safely returns to the compiled defaults and reports those values in its next packet.
