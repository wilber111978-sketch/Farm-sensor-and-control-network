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


## Camera live view (go2rtc)

The Solar camera streams H.265. Re-encoding that to MJPEG on a Raspberry Pi 3
cannot keep up (measured 0.91x on the substream, 0.15x on 4K), which caused
growing delay, freezes and corrupted frames.

`install-go2rtc.sh` installs [go2rtc](https://github.com/AlexxIT/go2rtc), which
relays the camera's own video to the browser without decoding it (WebRTC, or
MSE for H.265). The tablet decodes the video in hardware, like the camera app.

```bash
git clone --depth 1 -b camera-go2rtc https://github.com/wilber111978-sketch/farm-sensor-and-control-network ~/farm-src
sudo bash ~/farm-src/FarmPi/install-go2rtc.sh
```

- The installer never replaces the deployed `app.py` (which can be newer than
  Git). It merges only the camera changes via `tools/apply_camera_update.py`
  and leaves the file untouched if they don't match exactly.
- Re-run the script after changing camera settings in `.env`.
- go2rtc is locked down to streaming only (`allow_paths: [/api/ws]`, no
  exec/ffmpeg modules). It listens on ports 1984 and 8555.
- If go2rtc is not running, the page falls back to the old ffmpeg view
  (substream only, low frame rate). The "Basic view" button forces it.
- H.264 on the camera's substream gives the widest browser support and the
  lowest delay (WebRTC). With H.265, Chrome on Android plays it over MSE.
- PTZ buttons move while held and stop on release.
