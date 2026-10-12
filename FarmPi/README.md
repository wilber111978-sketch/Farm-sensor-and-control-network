# FarmPi dashboard

This folder is the source-controlled copy of the software deployed at
`/opt/farm-dashboard` on FarmPi.

## Setup

1. Copy `.env.example` to `.env` and fill in private credentials and the farm's
   exact WGS84 coordinates. Do not commit `.env` or publish the precise farm
   location.
2. Create a Python virtual environment and install `requirements.txt`.
3. Copy `farm-dashboard.service` and `farm-node-watchdog.service` to
   `/etc/systemd/system/`, reload systemd, and enable the services.

## Weather data

- `farm/weather/weather_1` is reserved for the physical LoRa weather station.
- `farm/weather/forecast` is retained online forecast data fetched from
  Open-Meteo every 30 minutes by default.
- The dashboard shows a fresh station temperature first, then falls back to
  the online current forecast. The detail panel always shows the forecast.
- The public example deliberately leaves the precise location blank. Store it
  only in FarmPi's private `.env` file.
- FarmPi does not attempt IP-based automatic positioning because rural mobile,
  satellite, and routed internet connections can report the wrong location.

## Source-control exclusions

The real `.env`, camera snapshots, virtual environments, Python caches, and
deployment backups are intentionally excluded from Git.

