# Farm sensor and control network

A solar-powered LoRa sensor network for monitoring a remote farm. ESP32 tank
and soil nodes send measurements to an ESP32 gateway, which forwards them over
MQTT to the local FarmPi dashboard.

## Current components

- `ESP32_Gateway_V1.5.2/` — LoRa-to-MQTT gateway with ACK handling, command
  response routing, and support for future `weather_*` nodes.
- `Soil/` — four soil node sketches and pH calibration notes.
- `Tanks/` — three ultrasonic tank level node sketches and remote calibration
  notes.
- `FarmPi/` — dashboard, node watchdog, service definitions, dependency list,
  and deployment notes.

## Network overview

```text
Tank, soil, and future weather nodes
                |
            LoRa 915 MHz
                |
          ESP32 gateway
                |
               MQTT
                |
     FarmPi dashboard/watchdog
```

The dashboard combines physical-node data with an online Open-Meteo forecast
for the configured farm location. A future physical `weather_1` station will
publish through the gateway to `farm/weather/weather_1`.

## Configuration and secrets

Real credentials are deliberately excluded from Git. Before building or
deploying:

1. Copy `ESP32_Gateway_V1.5.2/GatewaySecrets.h.example` to
   `GatewaySecrets.h` and enter the gateway network settings.
2. Copy `FarmPi/.env.example` to `FarmPi/.env` and enter the FarmPi private
   settings.
3. Never commit either real configuration file.

See `FarmPi/README.md` for FarmPi installation and weather configuration.

## Photos and 3D models

- Upload build and installation photos under `assets/photos/`.
- Upload printable enclosure and mounting models under `hardware/stl/`.
- Each component has its own folder and a short naming guide.
- Remove GPS metadata from photos before publishing them.

## Design goals

- Long-range, low-power operation
- Solar-powered unattended nodes
- Reliable ACK, retry, watchdog, and recovery behaviour
- Modular node types and MQTT routing
- Maintainable local source suitable for repeatable deployment
