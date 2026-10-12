from flask import Flask, Response, jsonify, render_template_string, request, send_file
import hashlib
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import Request, urlopen

import paho.mqtt.client as mqtt


app = Flask(__name__)


BASE_DIR = Path(__file__).resolve().parent


def load_local_env(path=BASE_DIR / ".env"):
    """Load simple KEY=VALUE pairs without adding another Python dependency."""
    try:
        for raw_line in path.read_text(encoding="utf-8").splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            key = key.strip()
            value = value.strip().strip('"').strip("'")
            if key:
                os.environ.setdefault(key, value)
    except FileNotFoundError:
        pass


load_local_env()

# Solar/orchard camera. Credentials remain server-side in /opt/farm-dashboard/.env.
SOLAR_CAMERA_HOST = os.getenv("SOLAR_CAMERA_HOST", "192.168.1.100").strip()
SOLAR_CAMERA_USER = os.getenv("SOLAR_CAMERA_USER", "admin").strip()
SOLAR_CAMERA_PASSWORD = os.getenv("SOLAR_CAMERA_PASSWORD", "").strip()
SOLAR_CAMERA_RTSP_PORT = int(os.getenv("SOLAR_CAMERA_RTSP_PORT", "554"))
SOLAR_CAMERA_HTTP_PORT = int(os.getenv("SOLAR_CAMERA_HTTP_PORT", "80"))
SOLAR_CAMERA_PROFILE = os.getenv("SOLAR_CAMERA_PROFILE", "MainStream").strip() or "MainStream"
SOLAR_CAMERA_PTZ_SPEED = max(0.05, min(1.0, float(os.getenv("SOLAR_CAMERA_PTZ_SPEED", "0.55"))))
CAMERA_DIR = BASE_DIR / "static" / "cameras"
SOLAR_CAMERA_LAST_FRAME = CAMERA_DIR / "solar_last.jpg"
CAMERA_DIR.mkdir(parents=True, exist_ok=True)


# =============================================================================
# HUNTER HYDRAWISE SETTINGS - EDIT THESE BEFORE UPLOADING TO THE FARMPI
# =============================================================================
# This is shown only as the account name in the FarmPi information panel.
# Hunter's personal REST API authenticates with the API key, not the username.
HYDRAWISE_USERNAME = os.getenv("HYDRAWISE_USERNAME", "").strip()

# In Hydrawise: Menu -> Account Details -> Account Settings -> Generate API Key
HYDRAWISE_API_KEY = os.getenv("HYDRAWISE_API_KEY", "").strip()

# Leave blank when the account has one controller. If the account has more than
# one, enter the required controller ID after the first successful connection.
HYDRAWISE_CONTROLLER_ID = os.getenv("HYDRAWISE_CONTROLLER_ID", "").strip()

# Normally no changes are required below this line.
HYDRAWISE_API_BASE = "https://api.hydrawise.com/api/v1/"
HYDRAWISE_APP_URL = "https://app.hydrawise.com"
# =============================================================================

# Online forecast. Coordinates stay in .env and are never committed.
WEATHER_LATITUDE = os.getenv("WEATHER_LATITUDE", "").strip()
WEATHER_LONGITUDE = os.getenv("WEATHER_LONGITUDE", "").strip()
WEATHER_LOCATION_LABEL = os.getenv("WEATHER_LOCATION_LABEL", "Farm").strip() or "Farm"
WEATHER_REFRESH_SECONDS = max(900, int(os.getenv("WEATHER_REFRESH_SECONDS", "1800")))
WEATHER_FORECAST_TOPIC = "farm/weather/forecast"
WEATHER_API_URL = "https://api.open-meteo.com/v1/forecast"

farm_data = {}
last_seen = {}
data_lock = threading.Lock()

hydrawise_lock = threading.Lock()
hydrawise_wake = threading.Event()
hydrawise_state = {
    "configured": False,
    "api_ok": False,
    "error": "API key not configured",
    "last_update": None,
    "next_poll": None,
    "schedule": None,
    "customer": None,
    "last_action": None,
}

weather_lock = threading.Lock()
weather_state = {
    "configured": False,
    "api_ok": False,
    "provider": "Open-Meteo",
    "location": WEATHER_LOCATION_LABEL,
    "error": "Weather coordinates not configured",
    "fetched_at": None,
    "current": None,
    "daily": [],
}

mqtt_client = None


HTML = r"""
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1">
  <title>FarmPi</title>
  <style>
    * { box-sizing: border-box; }
    html, body {
      width: 100%; height: 100%; margin: 0; overflow: hidden;
      background: #111; font-family: Arial, sans-serif;
    }
    body { display: flex; align-items: center; justify-content: center; }
    #farm {
      position: relative;
      width: min(100vw, 150vh);
      height: min(100vh, 66.6667vw);
      flex: 0 0 auto;
      background: #111 url('/static/images/farm-background.png') center/100% 100% no-repeat;
    }

    /* Compact round markers. Positions are percentages of the farm image. */
    .marker {
      position: absolute; transform: translate(-50%, -50%);
      width: 54px; height: 54px; padding: 3px;
      border: 3px solid #858585; border-radius: 50%;
      background: rgba(15, 18, 17, .86); color: #fff;
      display: flex; flex-direction: column; align-items: center; justify-content: center;
      text-align: center; text-decoration: none; cursor: pointer; user-select: none;
      box-shadow: 0 2px 8px rgba(0,0,0,.55);
      transition: transform .15s ease, border-color .2s ease;
    }
    .marker:hover { transform: translate(-50%, -50%) scale(1.09); z-index: 20; }
    .marker .icon { font-size: 15px; line-height: 15px; }
    .marker .name { margin-top: 2px; font-size: 8px; line-height: 8px; font-weight: 700; }
    .marker .reading { margin-top: 2px; max-width: 48px; font-size: 8px; line-height: 8px; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
    .marker.online { border-color: #55dd78; }
    .marker.warning { border-color: #f2c94c; }
    .marker.offline { border-color: #ef5350; }
    .marker.pending { border-color: #858585; opacity: .88; }

    .camera-marker { overflow: hidden; padding: 0; background: #131715; }
    .camera-marker .camera-thumb { position: absolute; inset: 0; width: 100%; height: 100%; object-fit: cover; }
    .camera-marker .camera-fallback { position: relative; z-index: 1; font-size: 18px; }
    .camera-marker .camera-label { position: absolute; z-index: 2; left: 2px; right: 2px; bottom: 2px; padding: 2px 1px; border-radius: 8px; background: rgba(0,0,0,.62); color: #fff; font-size: 7px; line-height: 8px; font-weight: 700; text-shadow: 0 1px 2px #000; }
    .camera-marker .camera-dot { position: absolute; z-index: 3; top: 3px; right: 3px; width: 9px; height: 9px; border: 1px solid rgba(255,255,255,.8); border-radius: 50%; background: #858585; box-shadow: 0 1px 3px rgba(0,0,0,.6); }
    .camera-marker.online .camera-dot { background: #55dd78; }
    .camera-marker.offline .camera-dot { background: #ef5350; }
    .camera-marker.pending .camera-dot { background: #858585; }

    #detailPanel {
      position: fixed; z-index: 1000; top: 0; right: -470px;
      width: 450px; max-width: 94vw; height: 100vh; padding: 24px;
      overflow-y: auto; color: #fff; background: rgba(18, 21, 20, .97);
      box-shadow: -6px 0 20px rgba(0,0,0,.5); transition: right .25s ease;
    }
    #detailPanel.open { right: 0; }
    #closePanel {
      position: absolute; top: 10px; right: 14px; border: 0;
      color: #aaa; background: transparent; font-size: 30px; cursor: pointer;
    }
    #detailTitle { margin: 8px 35px 6px 0; font-size: 24px; }
    #detailStatus { margin-bottom: 18px; font-size: 13px; }
    .detail-row { display: flex; justify-content: space-between; gap: 18px; padding: 10px 0; border-bottom: 1px solid #333; font-size: 14px; }
    .detail-label { color: #aaa; }
    .detail-value { text-align: right; font-weight: 700; overflow-wrap: anywhere; }
    .online-text { color: #55dd78; }
    .warning-text { color: #f2c94c; }
    .offline-text { color: #ef5350; }
    .pending-text { color: #aaa; }
    .detail-section { margin: 20px 0 8px; color: #7ee89a; font-size: 15px; font-weight: 700; }
    .zone-card { margin: 10px 0; padding: 12px; border: 1px solid #3b443f; border-radius: 9px; background: #171b19; }
    .zone-title { font-size: 15px; font-weight: 700; cursor: pointer; }
    .zone-card[open] .zone-title { margin-bottom: 8px; }
    .zone-meta { margin: 4px 0; color: #ccc; font-size: 13px; line-height: 1.35; }
    .irrigation-summary { margin: 8px 0 16px; padding: 14px; border: 1px solid #3b5f49; border-radius: 9px; background: #15231b; }
    .irrigation-summary-label { color: #7ee89a; font-size: 11px; font-weight: 700; letter-spacing: .08em; }
    .irrigation-summary-main { margin-top: 6px; font-size: 18px; font-weight: 700; }
    .irrigation-summary-meta { margin-top: 5px; color: #ccc; font-size: 13px; }
    .diagnostics { margin-top: 20px; padding-top: 10px; border-top: 1px solid #333; color: #aaa; }
    .diagnostics > summary { cursor: pointer; font-size: 13px; font-weight: 700; }
    .detail-actions { display: flex; flex-wrap: wrap; gap: 7px; margin-top: 10px; }
    .detail-action, .app-link {
      border: 1px solid #607067; border-radius: 6px; padding: 7px 10px;
      color: #fff; background: #29322d; font-size: 12px; text-decoration: none; cursor: pointer;
    }
    .detail-action.danger { border-color: #9d4946; background: #572b29; }
    .detail-action:disabled { opacity: .55; cursor: wait; }
    .api-error { margin: 10px 0; padding: 10px; border-radius: 6px; color: #ffc6c4; background: #522725; font-size: 13px; }

    /* Hydrawise orchard watering overlays. */
    .irrigation-zone {
      position: absolute;
      z-index: 2;
      pointer-events: none;
      opacity: 0;
      border-radius: 18px;
      background: rgba(45, 155, 255, .18);
      box-shadow: 0 0 14px rgba(45, 155, 255, 0), inset 0 0 18px rgba(70, 180, 255, 0);
      transition: opacity .4s ease;
    }
    .irrigation-zone.watering {
      opacity: 1;
      animation: wateringPulse 1.8s ease-in-out infinite;
    }
    @keyframes wateringPulse {
      0%, 100% {
        background: rgba(45, 155, 255, .12);
        box-shadow: 0 0 8px rgba(45, 155, 255, .25), inset 0 0 12px rgba(80, 190, 255, .15);
      }
      50% {
        background: rgba(45, 155, 255, .32);
        box-shadow: 0 0 24px rgba(45, 155, 255, .70), inset 0 0 25px rgba(80, 190, 255, .40);
      }
    }

    @media (max-width: 700px) {
      .marker { width: 46px; height: 46px; border-width: 2px; }
      .marker .icon { font-size: 14px; line-height: 14px; }
      .marker .name, .marker .reading { font-size: 8px; line-height: 8px; }
    }
  </style>
</head>
<body>
<div id="farm">
  <!-- Initial orchard approximations: Hydrawise relays 1-7 map left to right. -->
  <div id="irrigation_zone_1" class="irrigation-zone" style="left:25%;top:9%;width:9%;height:31%;"></div>
  <div id="irrigation_zone_2" class="irrigation-zone" style="left:34%;top:9%;width:9%;height:31%;"></div>
  <div id="irrigation_zone_3" class="irrigation-zone" style="left:43%;top:9%;width:9%;height:31%;"></div>
  <div id="irrigation_zone_4" class="irrigation-zone" style="left:52%;top:9%;width:9%;height:31%;"></div>
  <div id="irrigation_zone_5" class="irrigation-zone" style="left:61%;top:9%;width:9%;height:31%;"></div>
  <div id="irrigation_zone_6" class="irrigation-zone" style="left:70%;top:9%;width:9%;height:31%;"></div>
  <div id="irrigation_zone_7" class="irrigation-zone" style="left:79%;top:9%;width:9%;height:31%;"></div>

  <!-- Tanks: field tank, left of house, and slightly-left-adjusted right-of-house tank. -->
  <button id="tank_1" class="marker pending" data-title="Tank 1" style="left:22%;top:35%"><span class="icon">💧</span><span class="name">TANK 1</span><span class="reading">--</span></button>
  <button id="tank_2" class="marker pending" data-title="Tank 2" style="left:57%;top:75%"><span class="icon">💧</span><span class="name">TANK 2</span><span class="reading">--</span></button>
  <button id="tank_3" class="marker pending" data-title="Tank 3" style="left:81%;top:77%"><span class="icon">💧</span><span class="name">TANK 3</span><span class="reading">--</span></button>

  <!-- Soil 1-4 sample the orchard evenly from left to right. -->
  <button id="soil_1" class="marker pending" data-title="Soil 1" style="left:34%;top:24%"><span class="icon">🌱</span><span class="name">SOIL 1</span><span class="reading">--</span></button>
  <button id="soil_2" class="marker pending" data-title="Soil 2" style="left:51%;top:24%"><span class="icon">🌱</span><span class="name">SOIL 2</span><span class="reading">--</span></button>
  <button id="soil_3" class="marker pending" data-title="Soil 3" style="left:68%;top:24%"><span class="icon">🌱</span><span class="name">SOIL 3</span><span class="reading">--</span></button>
  <button id="soil_4" class="marker pending" data-title="Soil 4" style="left:85%;top:24%"><span class="icon">🌱</span><span class="name">SOIL 4</span><span class="reading">--</span></button>

  <!-- Gateway is physically over the bore pump; its panel includes bore/pump/flow data. -->
  <button id="gateway" class="marker pending" data-title="Gateway &amp; Bore Pump" style="left:50%;top:36%"><span class="icon">📡</span><span class="name">GATEWAY</span><span class="reading">--</span></button>

  <!-- Hunter Hydrawise status and controls open in the FarmPi information panel. -->
  <button id="irrigation" class="marker pending" data-title="Hunter Hydrawise" aria-label="Open Hunter Hydrawise irrigation information" style="left:34%;top:37%"><span class="icon">🚿</span><span class="name">IRRIGATION</span><span class="reading">API SETUP</span></button>

  <!-- Planned integrations. Grey means not yet connected, not failed. -->
  <button id="solar" class="marker pending" data-title="Solar System" style="left:16%;top:22%"><span class="icon">☀️</span><span class="name">SOLAR</span><span class="reading">PLANNED</span></button>
  <button id="weather" class="marker pending" data-title="Farm Weather" style="left:91%;top:18%"><span class="icon">🌤️</span><span class="name">WEATHER</span><span class="reading">--</span></button>

  <!-- Cameras: gate, solar array, house, and shed bottom corner above its doors. -->
  <button id="camera_gate" class="marker pending" data-title="Gate Camera" style="left:43%;top:54%"><span class="icon">📷</span><span class="name">GATE CAM</span><span class="reading">PLANNED</span></button>
  <button id="camera_solar" class="marker camera-marker pending" data-title="Solar / Orchard Camera" aria-label="Open Solar / Orchard Camera" style="left:11%;top:15%">
    <img class="camera-thumb" alt="Solar camera last frame" hidden>
    <span class="camera-fallback icon">📷</span>
    <span class="camera-label">SOLAR CAM</span>
    <span class="camera-dot" aria-hidden="true"></span>
  </button>
  <button id="camera_house" class="marker pending" data-title="House Camera" style="left:72%;top:69%"><span class="icon">📷</span><span class="name">HOUSE CAM</span><span class="reading">PLANNED</span></button>
  <button id="camera_shed" class="marker pending" data-title="Shed Camera" style="left:26%;top:72%"><span class="icon">📷</span><span class="name">SHED CAM</span><span class="reading">PLANNED</span></button>
</div>

<aside id="detailPanel" aria-live="polite">
  <button id="closePanel" aria-label="Close">&times;</button>
  <h2 id="detailTitle">Device</h2>
  <div id="detailStatus"></div>
  <div id="detailContent"></div>
</aside>

<script>
  let farmData = {};
  let hydrawiseData = {};
  let openId = null;

  // Tank and soil nodes normally report once per hour.
  const NODE_WARN = 5400;
  const NODE_OFFLINE = 7200;
  const GATEWAY_WARN = 120;
  const GATEWAY_OFFLINE = 300;
  const WEATHER_WARN = 5400;
  const WEATHER_OFFLINE = 7200;

  const staticDetails = {
    solar: [['Location', 'Solar array'], ['Connection', 'Planned']],
    camera_gate: [['Location', 'Main gate'], ['Connection', 'Planned']],
    camera_solar: [['Location', 'Solar array overlooking orchard'], ['Connection', 'Camera page']],
    camera_house: [['Location', 'House'], ['Connection', 'Planned']],
    camera_shed: [['Location', 'Bottom shed corner above doors'], ['Connection', 'Planned']]
  };

  const display = (value, suffix = '') => value === undefined || value === null || value === '' ? '--' : `${value}${suffix}`;
  const titleCase = value => String(value ?? '--').replaceAll('_', ' ').replace(/\b\w/g, c => c.toUpperCase());

  function humanAge(seconds) {
    if (seconds === undefined || seconds === null) return 'Never received';
    if (seconds < 60) return `${Math.round(seconds)} sec ago`;
    if (seconds < 3600) return `${Math.round(seconds / 60)} min ago`;
    if (seconds < 86400) return `${(seconds / 3600).toFixed(1)} hr ago`;
    return `${(seconds / 86400).toFixed(1)} days ago`;
  }

  function humanDuration(seconds) {
    const value = Number(seconds);
    if (!Number.isFinite(value)) return '--';
    if (value < 60) return `${Math.round(value)} sec`;
    if (value < 3600) return `${Math.round(value / 60)} min`;
    return `${(value / 3600).toFixed(1)} hr`;
  }

  function localTime(epoch) {
    const value = Number(epoch);
    return Number.isFinite(value) && value > 0 ? new Date(value * 1000).toLocaleString() : '--';
  }

  function irrigationCountdown(seconds) {
    const value = Math.max(0, Number(seconds));
    if (!Number.isFinite(value)) return '--';
    const days = Math.floor(value / 86400);
    const hours = Math.floor((value % 86400) / 3600);
    const minutes = Math.floor((value % 3600) / 60);
    if (days) return `${days}d ${hours}h`;
    if (hours) return `${hours}h ${minutes}m`;
    return `${Math.max(1, minutes)}m`;
  }

  function irrigationNextRun(zone, schedule) {
    const scheduledSeconds = Number(zone?.time);
    if (!Number.isFinite(scheduledSeconds) || scheduledSeconds <= 1) return null;
    const responseTime = Number(schedule?.time);
    const epoch = Number.isFinite(responseTime) && responseTime > 0
      ? responseTime + scheduledSeconds
      : Date.now() / 1000 + scheduledSeconds;
    const seconds = Math.max(0, epoch - Date.now() / 1000);
    const date = new Date(epoch * 1000);
    const label = date.toLocaleString([], {weekday: 'short', hour: 'numeric', minute: '2-digit'});
    return {seconds, label};
  }

  function setHealth(el, age, type = 'node') {
    el.classList.remove('online', 'warning', 'offline', 'pending');
    const warn = type === 'gateway' ? GATEWAY_WARN : NODE_WARN;
    const offline = type === 'gateway' ? GATEWAY_OFFLINE : NODE_OFFLINE;
    let health = 'online';
    if (age === undefined || age === null) health = 'pending';
    else if (age >= offline) health = 'offline';
    else if (age >= warn) health = 'warning';
    el.classList.add(health);
    return health;
  }

  function updateTank(id) {
    const el = document.getElementById(id);
    const item = farmData[`farm/tank/${id}`];
    if (!item) { setHealth(el, null); el.querySelector('.reading').textContent = '--'; return; }
    const d = item.data || {};
    setHealth(el, item.age);
    const level = d.pct ?? d.level ?? d.level_percent ?? d.percent;
    el.querySelector('.reading').textContent = level !== undefined ? `${level}%` : `BAT ${display(d.bat, '%')}`;
  }

  function updateSoil(id) {
    const el = document.getElementById(id);
    const item = farmData[`farm/soil/${id}`];
    if (!item) { setHealth(el, null); el.querySelector('.reading').textContent = '--'; return; }
    const d = item.data || {};
    setHealth(el, item.age);
    const moisture = d.moist ?? d.moisture ?? d.soil_moisture ?? d.humidity;
    el.querySelector('.reading').textContent = moisture !== undefined ? `${moisture}%` : `BAT ${display(d.bat, '%')}`;
  }

  function weatherDescription(code) {
    const value = Number(code);
    if (value === 0) return 'Clear';
    if ([1, 2].includes(value)) return 'Partly cloudy';
    if (value === 3) return 'Overcast';
    if ([45, 48].includes(value)) return 'Fog';
    if ([51, 53, 55, 56, 57].includes(value)) return 'Drizzle';
    if ([61, 63, 65, 66, 67, 80, 81, 82].includes(value)) return 'Rain';
    if ([71, 73, 75, 77, 85, 86].includes(value)) return 'Snow';
    if ([95, 96, 99].includes(value)) return 'Thunderstorm';
    return 'Unknown';
  }

  function weatherIcon(code, isDay = 1) {
    const value = Number(code);
    if (value === 0) return Number(isDay) === 0 ? '🌙' : '☀️';
    if ([1, 2].includes(value)) return Number(isDay) === 0 ? '☁️' : '🌤️';
    if ([3, 45, 48].includes(value)) return '☁️';
    if ([51, 53, 55, 56, 57, 61, 63, 65, 66, 67, 80, 81, 82].includes(value)) return '🌧️';
    if ([71, 73, 75, 77, 85, 86].includes(value)) return '🌨️';
    if ([95, 96, 99].includes(value)) return '⛈️';
    return '🌤️';
  }

  function updateWeather() {
    const el = document.getElementById('weather');
    const station = farmData['farm/weather/weather_1'];
    const forecast = farmData['farm/weather/forecast'];
    const stationFresh = station && station.age < WEATHER_OFFLINE;
    const current = forecast?.data?.current || {};
    const forecastAge = forecast?.data?.fetched_at
      ? Math.max(0, Date.now() / 1000 - Number(forecast.data.fetched_at))
      : forecast?.age;

    el.classList.remove('online', 'warning', 'offline', 'pending');
    let health = 'pending';
    if (stationFresh) {
      health = station.age >= WEATHER_WARN ? 'warning' : 'online';
    } else if (forecast?.data?.configured === false) {
      health = 'pending';
    } else if (forecast?.data?.api_ok && Number.isFinite(forecastAge)) {
      health = forecastAge >= WEATHER_OFFLINE ? 'warning' : 'online';
    } else if (station || forecast) {
      health = 'offline';
    }
    el.classList.add(health);

    const stationData = station?.data || {};
    const temperature = stationFresh
      ? (stationData.temp ?? stationData.temperature)
      : (current.temp ?? current.temperature_2m);
    const code = stationFresh
      ? (stationData.weather_code ?? stationData.code)
      : current.weather_code;
    const isDay = current.is_day ?? 1;
    el.querySelector('.icon').textContent = weatherIcon(code, isDay);
    el.querySelector('.reading').textContent = temperature === undefined ? '--' : `${temperature}°C`;
  }

  function updateGateway() {
    const el = document.getElementById('gateway');
    const gateway = farmData['farm/gateway/status'];
    const bore = farmData['farm/flow/bore'];
    if (!gateway && !bore) { setHealth(el, null, 'gateway'); el.querySelector('.reading').textContent = '--'; return; }
    const age = Math.min(gateway?.age ?? Infinity, bore?.age ?? Infinity);
    setHealth(el, Number.isFinite(age) ? age : null, 'gateway');
    const pump = bore?.data?.pump;
    const flow = bore?.data?.bore_lpm;
    el.querySelector('.reading').textContent = pump !== undefined ? `PUMP ${String(pump).toUpperCase()}` : (flow !== undefined ? `${flow} L/M` : `${display(gateway?.data?.nodes)} NODES`);
  }

  function updateIrrigationZones() {
    for (let relay = 1; relay <= 7; relay++) {
      document.getElementById(`irrigation_zone_${relay}`)?.classList.remove('watering');
    }

    (hydrawiseData.schedule?.relays || []).forEach(zone => {
      const relay = Number(zone.relay);
      if (Number(zone.time) === 1 && relay >= 1 && relay <= 7) {
        document.getElementById(`irrigation_zone_${relay}`)?.classList.add('watering');
      }
    });
  }

  function updateIrrigation() {
    updateIrrigationZones();
    const el = document.getElementById('irrigation');
    el.classList.remove('online', 'warning', 'offline', 'pending');
    if (!hydrawiseData.configured) {
      el.classList.add('pending');
      el.querySelector('.reading').textContent = 'API SETUP';
      return;
    }
    if (!hydrawiseData.api_ok) {
      el.classList.add('offline');
      el.querySelector('.reading').textContent = 'API ERROR';
      return;
    }
    el.classList.add('online');
    const running = (hydrawiseData.schedule?.relays || []).find(zone => Number(zone.time) === 1);
    el.querySelector('.reading').textContent = running ? `${running.name} ${humanDuration(running.run)}` : 'IDLE';
  }

  function row(label, value) {
    const outer = document.createElement('div'); outer.className = 'detail-row';
    const left = document.createElement('div'); left.className = 'detail-label'; left.textContent = label;
    const right = document.createElement('div'); right.className = 'detail-value'; right.textContent = value;
    outer.append(left, right); return outer;
  }

  function addRows(rows) {
    const content = document.getElementById('detailContent'); content.replaceChildren();
    rows.forEach(([label, value]) => content.appendChild(row(label, String(value))));
  }

  function sectionTitle(text) {
    const heading = document.createElement('div');
    heading.className = 'detail-section'; heading.textContent = text;
    return heading;
  }

  function appendRows(content, rows) {
    rows.forEach(([label, value]) => content.appendChild(row(label, String(value))));
  }

  function actionButton(label, action, relayId = null, custom = null, danger = false) {
    const button = document.createElement('button');
    button.className = `detail-action${danger ? ' danger' : ''}`;
    button.textContent = label;
    button.addEventListener('click', () => hydrawiseAction(button, action, relayId, custom));
    return button;
  }

  async function hydrawiseAction(button, action, relayId = null, custom = null) {
    const descriptions = {
      run: `Start this zone for ${humanDuration(custom)}?`,
      stop: 'Stop this zone now?',
      stopall: 'Stop all irrigation zones now?',
      suspendall: 'Suspend all irrigation for 24 hours?'
    };
    if (!confirm(descriptions[action] || 'Send this command to Hydrawise?')) return;
    button.disabled = true;
    try {
      const response = await fetch('/api/hydrawise/action', {
        method: 'POST', headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({action, relay_id: relayId, custom})
      });
      const result = await response.json();
      if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
      alert(result.message || 'Hydrawise command accepted');
      await refreshHydrawise();
    } catch (error) {
      alert(`Hydrawise command failed: ${error.message}`);
    } finally {
      button.disabled = false;
    }
  }

  function renderHydrawisePanel() {
    const status = document.getElementById('detailStatus');
    const content = document.getElementById('detailContent');
    const diagnosticsWasOpen = content.querySelector('.diagnostics')?.open ?? false;
    const openZones = new Set(
      Array.from(content.querySelectorAll('.zone-card[open]')).map(card => card.dataset.relay)
    );
    content.replaceChildren();
    const schedule = hydrawiseData.schedule || {};
    const customer = hydrawiseData.customer || {};
    const relays = Array.isArray(schedule.relays) ? schedule.relays : [];
    const sensors = Array.isArray(schedule.sensors) ? schedule.sensors : [];
    const running = relays.find(zone => Number(zone.time) === 1);
    const upcoming = relays
      .map(zone => ({zone, next: irrigationNextRun(zone, schedule)}))
      .filter(item => item.next)
      .sort((a, b) => a.next.seconds - b.next.seconds)[0];

    if (!hydrawiseData.configured) {
      status.className = 'pending-text'; status.textContent = 'API KEY NOT CONFIGURED';
      const error = document.createElement('div'); error.className = 'api-error';
      error.textContent = 'Set HYDRAWISE_USERNAME and HYDRAWISE_API_KEY in .env, then restart the dashboard.';
      content.appendChild(error);
    } else if (!hydrawiseData.api_ok) {
      status.className = 'offline-text'; status.textContent = 'API UNAVAILABLE';
      const error = document.createElement('div'); error.className = 'api-error';
      error.textContent = hydrawiseData.error || 'No response from Hydrawise'; content.appendChild(error);
    } else {
      status.className = 'online-text'; status.textContent = running ? 'WATERING' : 'CONNECTED — IDLE';
    }

    if (hydrawiseData.api_ok) {
      const summary = document.createElement('div'); summary.className = 'irrigation-summary';
      const label = document.createElement('div'); label.className = 'irrigation-summary-label';
      const main = document.createElement('div'); main.className = 'irrigation-summary-main';
      const meta = document.createElement('div'); meta.className = 'irrigation-summary-meta';
      if (running) {
        label.textContent = 'WATERING NOW';
        main.textContent = running.name || `Zone ${running.relay}`;
        meta.textContent = `${humanDuration(running.run)} remaining`;
      } else if (upcoming) {
        label.textContent = 'NEXT HYDRAWISE-ADJUSTED RUN';
        main.textContent = `${upcoming.zone.name || `Zone ${upcoming.zone.relay}`} · ${upcoming.next.label}`;
        meta.textContent = `Starts in ${irrigationCountdown(upcoming.next.seconds)} · ${humanDuration(upcoming.zone.run)}`;
      } else {
        label.textContent = 'NEXT RUN';
        main.textContent = 'No run currently scheduled';
        meta.textContent = 'Hydrawise may have suspended watering for weather or sensor conditions.';
      }
      summary.append(label, main, meta); content.appendChild(summary);
    }

    const allActions = document.createElement('div'); allActions.className = 'detail-actions';
    const openApp = document.createElement('a'); openApp.className = 'app-link';
    openApp.href = hydrawiseData.app_url || 'https://app.hydrawise.com'; openApp.target = '_blank';
    openApp.rel = 'noopener noreferrer'; openApp.textContent = 'History & Water Use';
    allActions.append(openApp);
    if (hydrawiseData.api_ok) {
      allActions.append(actionButton('Stop All', 'stopall', null, null, true));
      allActions.append(actionButton('Suspend All 24h', 'suspendall', null, 86400, true));
    }
    content.appendChild(allActions);

    content.appendChild(sectionTitle('Zones'));
    if (!relays.length) {
      const empty = document.createElement('div'); empty.className = 'zone-meta';
      empty.textContent = hydrawiseData.api_ok ? 'No zones returned by this controller.' : 'Waiting for Hydrawise data.';
      content.appendChild(empty);
    }
    relays.forEach(zone => {
      const isRunning = Number(zone.time) === 1;
      const next = irrigationNextRun(zone, schedule);
      const relayKey = String(zone.relay);
      const card = document.createElement('details'); card.className = 'zone-card'; card.dataset.relay = relayKey;
      if (isRunning || openZones.has(relayKey)) card.open = true;
      const title = document.createElement('summary'); title.className = 'zone-title';
      title.textContent = isRunning
        ? `${zone.name || `Zone ${zone.relay}`} — WATERING · ${humanDuration(zone.run)} left`
        : next
          ? `${zone.name || `Zone ${zone.relay}`} — ${next.label} · in ${irrigationCountdown(next.seconds)}`
          : `${zone.name || `Zone ${zone.relay}`} — Not scheduled`;
      card.appendChild(title);
      const details = [isRunning ? 'Hydrawise is running this zone now.' : `Calculated duration: ${humanDuration(zone.run)}`];
      details.forEach(value => { const meta = document.createElement('div'); meta.className = 'zone-meta'; meta.textContent = value; card.appendChild(meta); });
      if (hydrawiseData.api_ok) {
        const actions = document.createElement('div'); actions.className = 'detail-actions';
        actions.append(actionButton('Run 5 min', 'run', zone.relay_id, 300));
        actions.append(actionButton('Run 10 min', 'run', zone.relay_id, 600));
        actions.append(actionButton('Stop', 'stop', zone.relay_id, null, true));
        card.appendChild(actions);
      }
      content.appendChild(card);
    });

    const diagnostics = document.createElement('details'); diagnostics.className = 'diagnostics';
    diagnostics.open = diagnosticsWasOpen;
    const diagnosticTitle = document.createElement('summary'); diagnosticTitle.textContent = 'Diagnostics';
    diagnostics.appendChild(diagnosticTitle);
    appendRows(diagnostics, [
      ['Connection', hydrawiseData.api_ok ? 'OK' : 'Not connected'],
      ['Controller', display(customer.current_controller)],
      ['Last update', localTime(hydrawiseData.last_update)],
      ['Next API poll', localTime(hydrawiseData.next_poll)],
      ['Zones', relays.length],
      ['Sensors', sensors.length]
    ]);
    sensors.forEach((sensor, index) => appendRows(diagnostics, [
      [`Sensor ${index + 1}`, Number(sensor.type) === 1 ? 'Rain sensor' : Number(sensor.type) === 3 ? 'Flow meter' : `Type ${display(sensor.type)}`],
      ['Linked zones', Array.isArray(sensor.relays) ? sensor.relays.length : 0]
    ]));
    if (schedule.message) appendRows(diagnostics, [['Hydrawise message', schedule.message]]);
    if (hydrawiseData.last_action) appendRows(diagnostics, [['Last command', hydrawiseData.last_action]]);
    content.appendChild(diagnostics);
  }

  function deviceRows(id) {
    if (staticDetails[id]) return { age: null, health: 'pending', rows: staticDetails[id] };
    if (id === 'weather') {
      const marker = document.getElementById(id);
      const station = farmData['farm/weather/weather_1'];
      const forecast = farmData['farm/weather/forecast'];
      const sd = station?.data || {};
      const fd = forecast?.data || {};
      const current = fd.current || {};
      const stationFresh = station && station.age < WEATHER_OFFLINE;
      const rows = [
        ['Displayed source', stationFresh ? 'Weather station' : fd.api_ok ? 'Online forecast' : 'Unavailable'],
        ['Location', display(fd.location)],
        ['Temperature', display(stationFresh ? (sd.temp ?? sd.temperature) : current.temp, ' °C')],
        ['Feels like', display(current.apparent_temp, ' °C')],
        ['Humidity', display(stationFresh ? (sd.humidity ?? sd.rh) : current.humidity, '%')],
        ['Conditions', weatherDescription(stationFresh ? (sd.weather_code ?? sd.code) : current.weather_code)],
        ['Wind', display(stationFresh ? sd.wind_speed : current.wind_speed, ' km/h')],
        ['Wind direction', display(stationFresh ? sd.wind_dir : current.wind_dir, '°')],
        ['Current rain', display(stationFresh ? (sd.rain_mm ?? sd.rain) : current.precipitation, ' mm')],
        ['Station report', humanAge(station?.age)],
        ['Forecast fetched', localTime(fd.fetched_at)],
        ['Forecast provider', display(fd.provider)]
      ];
      (fd.daily || []).slice(0, 7).forEach(day => rows.push([
        day.date,
        `${weatherDescription(day.weather_code)} · ${display(day.temp_min, '°')}–${display(day.temp_max, '°C')} · rain ${display(day.rain_chance, '%')}`
      ]));
      const health = marker.classList.contains('offline') ? 'offline' : marker.classList.contains('warning') ? 'warning' : marker.classList.contains('online') ? 'online' : 'pending';
      return {age: stationFresh ? station.age : forecast?.age, health, rows};
    }
    if (id === 'gateway') {
      const g = farmData['farm/gateway/status'];
      const b = farmData['farm/flow/bore'];
      const dist = farmData['farm/flow/distribution'];
      const gd = g?.data || {}, bd = b?.data || {}, dd = dist?.data || {};
      const age = Math.min(g?.age ?? Infinity, b?.age ?? Infinity);
      return { age: Number.isFinite(age) ? age : null, health: document.getElementById(id).classList.contains('offline') ? 'offline' : document.getElementById(id).classList.contains('warning') ? 'warning' : g || b ? 'online' : 'pending', rows: [
        ['Gateway', display(gd.status)], ['Radio', display(gd.radio)], ['Wi-Fi', display(gd.wifi, ' dBm')],
        ['LoRa nodes', display(gd.nodes)], ['LoRa packets RX', display(gd.rx)], ['Last LoRa', display(gd.lastLoRa, ' sec')],
        ['Bore pump', titleCase(bd.pump)], ['Bore flow', display(bd.bore_lpm, ' L/min')], ['Bore total', display(bd.bore_total, ' L')],
        ['Distribution flow', display(dd.dist_lpm, ' L/min')], ['Distribution total', display(dd.dist_total, ' L')],
        ['Last gateway/bore update', humanAge(Number.isFinite(age) ? age : null)]
      ]};
    }
    const isTank = id.startsWith('tank_');
    const item = farmData[isTank ? `farm/tank/${id}` : `farm/soil/${id}`];
    const d = item?.data || {};
    const measurement = isTank ? (d.pct ?? d.level ?? d.level_percent ?? d.percent) : (d.moist ?? d.moisture ?? d.soil_moisture ?? d.humidity);
    const rows = [[isTank ? 'Tank level' : 'Soil moisture', display(measurement, '%')]];
    if (!isTank) {
      rows.push(
        ['Temperature', display(d.temp, ' °C')],
        ['Electrical conductivity', display(d.ec, ' µS/cm')],
        ['pH', display(d.ph)],
        ['Nitrogen', display(d.n, ' mg/kg')],
        ['Phosphorus', display(d.p, ' mg/kg')],
        ['Potassium', display(d.k, ' mg/kg')],
        ['Samples', display(d.samples)],
        ['Sample quality', display(d.quality, '%')],
        ['Communication', d.comm_ok === undefined ? '--' : d.comm_ok ? 'OK' : 'Fault'],
        ['Data validity', d.data_ok === undefined ? '--' : d.data_ok ? 'OK' : 'Fault'],
        ['Firmware', display(d.fw)],
        ['Cycle', display(d.cycle)]
      );
    }
    rows.push(
      ['Battery', display(d.bat, '%')],
      ['Battery voltage', display(d.batv, ' V')],
      ['RSSI', display(d.gateway_rssi, ' dBm')],
      ['SNR', display(d.gateway_snr, ' dB')],
      ['Sensor', d.sensor_ok === undefined ? '--' : d.sensor_ok ? 'OK' : 'Fault / not ready'],
      ['Last report', humanAge(item?.age)]
    );
    const marker = document.getElementById(id);
    const health = marker.classList.contains('offline') ? 'offline' : marker.classList.contains('warning') ? 'warning' : item ? 'online' : 'pending';
    return { age: item?.age, health, rows };
  }

  function openDevice(id) {
    if (id === 'camera_solar') {
      window.location.href = '/camera/solar';
      return;
    }
    openId = id;
    const el = document.getElementById(id);
    document.getElementById('detailTitle').textContent = el.dataset.title;
    if (id === 'irrigation') {
      renderHydrawisePanel();
      document.getElementById('detailPanel').classList.add('open');
      return;
    }
    const detail = deviceRows(id);
    const status = document.getElementById('detailStatus');
    const labels = {online: 'ONLINE', warning: 'LATE', offline: 'OFFLINE', pending: 'NOT CONNECTED'};
    status.className = `${detail.health}-text`;
    status.textContent = labels[detail.health];
    addRows(detail.rows);
    document.getElementById('detailPanel').classList.add('open');
  }

  async function refresh() {
    try {
      const response = await fetch('/api/status', {cache: 'no-store'});
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      farmData = await response.json();
      ['tank_1','tank_2','tank_3'].forEach(updateTank);
      ['soil_1','soil_2','soil_3','soil_4'].forEach(updateSoil);
      updateGateway();
      updateWeather();
      if (openId) openDevice(openId);
    } catch (error) {
      console.error('FarmPi refresh failed:', error);
    }
  }

  async function refreshSolarCamera() {
    const marker = document.getElementById('camera_solar');
    if (!marker) return;
    try {
      const response = await fetch('/api/camera/solar/status', {cache: 'no-store'});
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const data = await response.json();
      marker.classList.remove('online', 'offline', 'pending', 'warning');
      marker.classList.add(data.online ? 'online' : 'offline');
      marker.title = data.online ? 'Solar / Orchard Camera — ONLINE' : 'Solar / Orchard Camera — OFFLINE';
      const thumb = marker.querySelector('.camera-thumb');
      const fallback = marker.querySelector('.camera-fallback');
      if (data.has_thumbnail) {
        const stamp = data.thumbnail_mtime || 0;
        if (thumb.dataset.stamp !== String(stamp)) {
          thumb.src = `/api/camera/solar/thumbnail?v=${stamp}`;
          thumb.dataset.stamp = String(stamp);
        }
        thumb.hidden = false; fallback.hidden = true;
      } else {
        thumb.hidden = true; fallback.hidden = false;
      }
    } catch (error) {
      marker.classList.remove('online', 'pending', 'warning');
      marker.classList.add('offline');
      console.error('Solar camera status failed:', error);
    }
  }

  async function refreshHydrawise() {
    try {
      const response = await fetch('/api/hydrawise/status', {cache: 'no-store'});
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      hydrawiseData = await response.json();
      updateIrrigation();
    } catch (error) {
      console.error('Hydrawise refresh failed:', error);
      hydrawiseData = {configured: true, api_ok: false, error: error.message};
      updateIrrigation();
    }
  }

  document.querySelectorAll('.marker').forEach(el => el.addEventListener('click', () => openDevice(el.id)));
  document.getElementById('closePanel').addEventListener('click', () => { openId = null; document.getElementById('detailPanel').classList.remove('open'); });
  document.addEventListener('keydown', event => { if (event.key === 'Escape') document.getElementById('closePanel').click(); });
  refresh();
  refreshHydrawise();
  refreshSolarCamera();
  setInterval(refresh, 2000);
  setInterval(refreshHydrawise, 10000);
  setInterval(refreshSolarCamera, 30000);
</script>
</body>
</html>
"""


CAMERA_HTML = r"""
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, viewport-fit=cover">
  <title>Solar / Orchard Camera - FarmPi</title>
  <style>
    * { box-sizing: border-box; }
    html, body { margin: 0; min-height: 100%; background: #0d0f0e; color: #fff; font-family: Arial, sans-serif; }
    body { min-height: 100vh; }
    .topbar { position: sticky; top: 0; z-index: 10; display: flex; align-items: center; gap: 12px; padding: 10px 14px; background: rgba(15,18,17,.96); border-bottom: 1px solid #2a302d; }
    .back { color: #fff; text-decoration: none; border: 1px solid #46514b; border-radius: 8px; padding: 8px 11px; background: #252c28; }
    .title { flex: 1; min-width: 0; }
    .title strong { display: block; font-size: 17px; }
    .title span { display: block; margin-top: 2px; color: #aab4ae; font-size: 12px; }
    .status { display: inline-flex; align-items: center; gap: 6px; font-size: 12px; font-weight: 700; }
    .status-dot { width: 9px; height: 9px; border-radius: 50%; background: #858585; }
    .status.online .status-dot { background: #55dd78; }
    .status.offline .status-dot { background: #ef5350; }
    main { width: min(1180px, 100%); margin: 0 auto; padding: 14px; }
    .viewer { position: relative; background: #000; border: 1px solid #303733; border-radius: 12px; overflow: hidden; min-height: 280px; display: grid; place-items: center; }
    #feed { display: block; width: 100%; max-height: calc(100vh - 260px); object-fit: contain; background: #000; }
    .overlay { position: absolute; top: 10px; left: 10px; padding: 5px 8px; border-radius: 7px; background: rgba(0,0,0,.62); font-size: 11px; font-weight: 700; }
    .toolbar { display: flex; flex-wrap: wrap; gap: 8px; margin: 12px 0; }
    button, .button { border: 1px solid #4a5750; border-radius: 8px; padding: 9px 12px; color: #fff; background: #27302b; font: inherit; font-size: 13px; cursor: pointer; text-decoration: none; }
    button:hover, .button:hover { background: #344039; }
    button.active { border-color: #55dd78; background: #20472c; }
    button:disabled { opacity: .45; cursor: wait; }
    .controls { display: grid; grid-template-columns: minmax(260px, 420px) 1fr; gap: 14px; align-items: start; }
    .panel { padding: 14px; border: 1px solid #303733; border-radius: 12px; background: #151917; }
    .panel h3 { margin: 0 0 12px; font-size: 14px; color: #7ee89a; }
    .ptz { display: grid; grid-template-columns: repeat(3, 64px); grid-template-rows: repeat(3, 52px); justify-content: center; gap: 7px; touch-action: none; }
    .ptz button { font-size: 20px; padding: 0; }
    .zoom { display: flex; justify-content: center; gap: 8px; margin-top: 10px; }
    .zoom button { min-width: 96px; }
    .presets { display: flex; flex-wrap: wrap; gap: 7px; }
    .hint { margin-top: 10px; color: #9ba69f; font-size: 12px; line-height: 1.4; }
    #error { display: none; margin-top: 10px; padding: 10px; border-radius: 8px; background: #522725; color: #ffd2d0; font-size: 12px; }
    @media (max-width: 720px) {
      main { padding: 8px; }
      .controls { grid-template-columns: 1fr; }
      .viewer { min-height: 220px; border-radius: 8px; }
      #feed { max-height: 56vh; }
      .topbar { padding: 8px; }
      .title strong { font-size: 15px; }
      .title span { font-size: 11px; }
    }
  </style>
</head>
<body>
  <header class="topbar">
    <a class="back" href="/">← Farm</a>
    <div class="title"><strong>Solar / Orchard Camera</strong><span>Solar array overlooking the orchard</span></div>
    <div id="status" class="status"><span class="status-dot"></span><span id="statusText">CHECKING</span></div>
  </header>
  <main>
    <section id="viewer" class="viewer">
      <img id="feed" alt="Solar / Orchard Camera live feed">
      <div id="profileLabel" class="overlay">SUBSTREAM · LIVE</div>
    </section>
    <div class="toolbar">
      <button id="subBtn" class="active">Live Substream</button>
      <button id="mainBtn">HQ (4K source)</button>
      <button id="snapshotBtn">Save Thumbnail</button>
      <button id="fullscreenBtn">Fullscreen</button>
    </div>
    <div class="controls">
      <section class="panel">
        <h3>PTZ</h3>
        <div class="ptz">
          <span></span><button data-move="up" aria-label="Tilt up">▲</button><span></span>
          <button data-move="left" aria-label="Pan left">◀</button><button id="homeBtn" aria-label="Home">⌂</button><button data-move="right" aria-label="Pan right">▶</button>
          <span></span><button data-move="down" aria-label="Tilt down">▼</button><span></span>
        </div>
        <div class="zoom"><button data-move="zoom_in">Zoom +</button><button data-move="zoom_out">Zoom −</button></div>
        <div class="hint">Hold a direction or zoom button to move. Releasing sends STOP immediately.</div>
      </section>
      <section class="panel">
        <h3>Presets</h3>
        <div id="presets" class="presets"><span class="hint">Loading presets…</span></div>
        <div id="error"></div>
      </section>
    </div>
  </main>
<script>
  let profile = 'sub';
  const feed = document.getElementById('feed');
  const errorBox = document.getElementById('error');
  function showError(message) { errorBox.textContent = message; errorBox.style.display = message ? 'block' : 'none'; }
  function setProfile(next) {
    profile = next;
    document.getElementById('subBtn').classList.toggle('active', next === 'sub');
    document.getElementById('mainBtn').classList.toggle('active', next === 'main');
    document.getElementById('profileLabel').textContent = next === 'sub' ? 'SUBSTREAM · LIVE' : '4K SOURCE · LIVE';
    feed.src = `/api/camera/solar/live?profile=${next}&t=${Date.now()}`;
  }
  async function status() {
    try {
      const r = await fetch('/api/camera/solar/status', {cache:'no-store'}); const d = await r.json();
      const el = document.getElementById('status'); el.className = `status ${d.online ? 'online' : 'offline'}`;
      document.getElementById('statusText').textContent = d.online ? 'ONLINE' : 'OFFLINE';
      if (!d.ffmpeg) showError('ffmpeg is not installed on FarmPi. Run the included install-camera-support.sh script.');
      else if (!d.configured) showError('Solar camera password is not configured in /opt/farm-dashboard/.env.');
    } catch (_) { document.getElementById('statusText').textContent = 'ERROR'; }
  }
  async function ptz(action, extra={}) {
    try {
      const r = await fetch('/api/camera/solar/ptz', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({action, ...extra})});
      const d = await r.json(); if (!r.ok) throw new Error(d.error || `HTTP ${r.status}`); showError(''); return d;
    } catch (e) { showError(`PTZ: ${e.message}`); }
  }
  document.querySelectorAll('[data-move]').forEach(btn => {
    btn.style.touchAction = 'manipulation';
    btn.style.userSelect = 'none';
    btn.style.webkitUserSelect = 'none';
    btn.style.webkitTouchCallout = 'none';

    btn.addEventListener('contextmenu', e => e.preventDefault());

    btn.addEventListener('pointerdown', async e => {
      e.preventDefault();

      if (btn.dataset.busy === '1') return;
      btn.dataset.busy = '1';

      await ptz(btn.dataset.move);

      setTimeout(async () => {
        await ptz('stop');
        btn.dataset.busy = '0';
      }, 450);
    });
  });
  document.getElementById('homeBtn').onclick = () => ptz('home');
  document.getElementById('subBtn').onclick = () => setProfile('sub');
  document.getElementById('mainBtn').onclick = () => setProfile('main');
  document.getElementById('fullscreenBtn').onclick = () => document.getElementById('viewer').requestFullscreen?.();
  document.getElementById('snapshotBtn').onclick = async () => { const r = await fetch('/api/camera/solar/snapshot', {method:'POST'}); const d = await r.json(); if (!r.ok) showError(d.error || 'Snapshot failed'); };
  async function loadPresets() {
    const box = document.getElementById('presets');
    try {
      const r = await fetch('/api/camera/solar/presets', {cache:'no-store'}); const d = await r.json();
      box.innerHTML = '';
      if (d.presets?.length) d.presets.forEach(p => { const b=document.createElement('button'); b.textContent=p.name || `Preset ${p.token}`; b.onclick=()=>ptz('preset',{token:p.token}); box.appendChild(b); });
      else box.innerHTML = '<span class="hint">No saved presets reported by the camera.</span>';
    } catch (e) { box.innerHTML = '<span class="hint">Preset list unavailable.</span>'; }
  }
  function saveLastFrame() { navigator.sendBeacon('/api/camera/solar/snapshot'); }
  window.addEventListener('pagehide', saveLastFrame);
  feed.onerror = () => showError('Live video could not start. Check camera credentials and ffmpeg on FarmPi.');
  feed.onload = () => showError('');
  setProfile('sub'); status(); loadPresets(); setInterval(status, 15000);
</script>
</body>
</html>
"""


@app.route("/")
def index():
    return render_template_string(HTML)


def solar_camera_password_token():
    if not SOLAR_CAMERA_PASSWORD:
        return ""
    return hashlib.md5(SOLAR_CAMERA_PASSWORD.encode("utf-8")).hexdigest().upper()


def solar_rtsp_uri(profile="sub"):
    stream = "stream0" if profile == "main" else "stream1"
    token = solar_camera_password_token()
    if not token:
        return None
    from urllib.parse import quote
    return (
        f"rtsp://{SOLAR_CAMERA_HOST}:{SOLAR_CAMERA_RTSP_PORT}/{stream}"
        f"?username={quote(SOLAR_CAMERA_USER, safe='')}&password={quote(token, safe='')}"
    )


def camera_online(timeout=0.8):
    try:
        with socket.create_connection((SOLAR_CAMERA_HOST, SOLAR_CAMERA_HTTP_PORT), timeout=timeout):
            return True
    except OSError:
        return False


def onvif_ptz_request(body_xml, timeout=3):
    envelope = f'''<?xml version="1.0" encoding="UTF-8"?>
<s:Envelope xmlns:s="http://www.w3.org/2003/05/soap-envelope"
 xmlns:tptz="http://www.onvif.org/ver20/ptz/wsdl"
 xmlns:tt="http://www.onvif.org/ver10/schema">
<s:Body>{body_xml}</s:Body></s:Envelope>'''
    url = f"http://{SOLAR_CAMERA_HOST}:{SOLAR_CAMERA_HTTP_PORT}/onvif/ptz"
    req = Request(url, data=envelope.encode("utf-8"), headers={"Content-Type": "application/soap+xml; charset=utf-8"}, method="POST")
    try:
        with urlopen(req, timeout=timeout) as response:
            return response.read()
    except HTTPError as error:
        raise RuntimeError(f"Camera returned HTTP {error.code}") from None
    except URLError as error:
        raise RuntimeError(f"Could not reach camera PTZ service: {error.reason}") from None


def capture_solar_thumbnail(profile="sub"):
    uri = solar_rtsp_uri(profile)
    if not uri:
        raise RuntimeError("Solar camera password is not configured in .env")
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        raise RuntimeError("ffmpeg is not installed on FarmPi")
    temp_path = SOLAR_CAMERA_LAST_FRAME.with_suffix(".tmp.jpg")
    command = [
        ffmpeg, "-hide_banner", "-loglevel", "error", "-rtsp_transport", "tcp",
        "-i", uri, "-frames:v", "1", "-q:v", "3", "-y", str(temp_path)
    ]
    result = subprocess.run(command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, timeout=12)
    if result.returncode != 0 or not temp_path.exists():
        message = result.stderr.decode("utf-8", errors="ignore").strip()[-300:]
        raise RuntimeError(message or "Camera snapshot failed")
    temp_path.replace(SOLAR_CAMERA_LAST_FRAME)


@app.route("/camera/solar")
def solar_camera_page():
    return render_template_string(CAMERA_HTML)


@app.route("/api/camera/solar/status")
def solar_camera_status():
    has_thumb = SOLAR_CAMERA_LAST_FRAME.exists()
    return jsonify({
        "online": camera_online(),
        "ffmpeg": bool(shutil.which("ffmpeg")),
        "configured": bool(SOLAR_CAMERA_PASSWORD),
        "has_thumbnail": has_thumb,
        "thumbnail_mtime": int(SOLAR_CAMERA_LAST_FRAME.stat().st_mtime) if has_thumb else None,
    })


@app.route("/api/camera/solar/thumbnail")
def solar_camera_thumbnail():
    if not SOLAR_CAMERA_LAST_FRAME.exists():
        return ("", 404)
    return send_file(SOLAR_CAMERA_LAST_FRAME, mimetype="image/jpeg", max_age=0)


@app.route("/api/camera/solar/snapshot", methods=["POST"])
def solar_camera_snapshot():
    def worker():
        try:
            capture_solar_thumbnail("sub")
        except Exception as error:
            print(f"[CAMERA] Solar thumbnail capture failed: {error}", flush=True)
    threading.Thread(target=worker, daemon=True).start()
    return jsonify({"ok": True, "message": "Thumbnail capture queued"}), 202


@app.route("/api/camera/solar/live")
def solar_camera_live():
    profile = "main" if request.args.get("profile") == "main" else "sub"
    uri = solar_rtsp_uri(profile)
    if not uri:
        return jsonify({"error": "Solar camera password is not configured in .env"}), 503
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        return jsonify({"error": "ffmpeg is not installed on FarmPi"}), 503

    if profile == "main":
        video_filter = "fps=4,scale=1600:-2"
        quality = "5"
    else:
        video_filter = "fps=10,scale=960:-2"
        quality = "6"
    command = [
        ffmpeg, "-hide_banner", "-loglevel", "error", "-rtsp_transport", "tcp",
        "-i", uri, "-an", "-vf", video_filter, "-q:v", quality,
        "-f", "mpjpeg", "-boundary_tag", "ffmpeg", "pipe:1"
    ]

    def generate():
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
        try:
            while True:
                chunk = process.stdout.read(32768)
                if not chunk:
                    break
                yield chunk
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()

    return Response(generate(), content_type="multipart/x-mixed-replace; boundary=ffmpeg", headers={"Cache-Control": "no-store"})


@app.route("/api/camera/solar/ptz", methods=["POST"])
def solar_camera_ptz():
    payload = request.get_json(silent=True) or {}
    action = str(payload.get("action", "")).lower()
    speed = SOLAR_CAMERA_PTZ_SPEED
    if action in {"up", "down", "left", "right", "zoom_in", "zoom_out"}:
        x = y = z = 0.0
        if action == "left": x = -speed
        elif action == "right": x = speed
        elif action == "up": y = speed
        elif action == "down": y = -speed
        elif action == "zoom_in": z = speed
        elif action == "zoom_out": z = -speed
        body = (
            f"<tptz:ContinuousMove><tptz:ProfileToken>{SOLAR_CAMERA_PROFILE}</tptz:ProfileToken>"
            f"<tptz:Velocity><tt:PanTilt x=\"{x}\" y=\"{y}\"/><tt:Zoom x=\"{z}\"/></tptz:Velocity>"
            f"</tptz:ContinuousMove>"
        )
    elif action == "stop":
        body = (
            f"<tptz:Stop><tptz:ProfileToken>{SOLAR_CAMERA_PROFILE}</tptz:ProfileToken>"
            "<tptz:PanTilt>true</tptz:PanTilt><tptz:Zoom>true</tptz:Zoom></tptz:Stop>"
        )
    elif action == "home":
        body = f"<tptz:GotoHomePosition><tptz:ProfileToken>{SOLAR_CAMERA_PROFILE}</tptz:ProfileToken></tptz:GotoHomePosition>"
    elif action == "preset":
        token = str(payload.get("token", "")).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;").replace('"', "&quot;")
        if not token:
            return jsonify({"error": "Preset token is required"}), 400
        body = (
            f"<tptz:GotoPreset><tptz:ProfileToken>{SOLAR_CAMERA_PROFILE}</tptz:ProfileToken>"
            f"<tptz:PresetToken>{token}</tptz:PresetToken></tptz:GotoPreset>"
        )
    else:
        return jsonify({"error": "Unsupported PTZ action"}), 400
    try:
        onvif_ptz_request(body)
        return jsonify({"ok": True})
    except Exception as error:
        return jsonify({"error": str(error)}), 502


@app.route("/api/camera/solar/presets")
def solar_camera_presets():
    body = f"<tptz:GetPresets><tptz:ProfileToken>{SOLAR_CAMERA_PROFILE}</tptz:ProfileToken></tptz:GetPresets>"
    try:
        xml_data = onvif_ptz_request(body)
        root = ET.fromstring(xml_data)
        presets = []
        for element in root.iter():
            if element.tag.endswith("Presets"):
                token = element.attrib.get("token") or element.attrib.get("Token")
                name = None
                for child in element:
                    if child.tag.endswith("Name"):
                        name = child.text
                        break
                if token:
                    presets.append({"token": token, "name": name or f"Preset {token}"})
        return jsonify({"presets": presets})
    except Exception as error:
        return jsonify({"error": str(error), "presets": []}), 502


@app.route("/api/status")
def status():
    now = time.time()
    with data_lock:
        output = {
            topic: {"data": data, "age": round(now - last_seen[topic], 1)}
            for topic, data in farm_data.items()
        }
    return jsonify(output)


def weather_coordinates():
    try:
        latitude = float(WEATHER_LATITUDE)
        longitude = float(WEATHER_LONGITUDE)
    except (TypeError, ValueError):
        return None
    if not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
        return None
    return latitude, longitude


def publish_weather_state():
    with weather_lock:
        payload = json.dumps(weather_state, separators=(",", ":"))
    client = mqtt_client
    if client is not None and client.is_connected():
        client.publish(WEATHER_FORECAST_TOPIC, payload, qos=1, retain=True)


def refresh_weather():
    coordinates = weather_coordinates()
    if coordinates is None:
        with weather_lock:
            weather_state.update({
                "configured": False,
                "api_ok": False,
                "error": "Set WEATHER_LATITUDE and WEATHER_LONGITUDE in .env",
                "fetched_at": None,
                "current": None,
                "daily": [],
            })
        publish_weather_state()
        return

    latitude, longitude = coordinates
    parameters = {
        "latitude": latitude,
        "longitude": longitude,
        "timezone": "auto",
        "forecast_days": 7,
        "current": ",".join([
            "temperature_2m", "relative_humidity_2m", "apparent_temperature",
            "precipitation", "weather_code", "wind_speed_10m",
            "wind_direction_10m", "is_day",
        ]),
        "daily": ",".join([
            "weather_code", "temperature_2m_max", "temperature_2m_min",
            "precipitation_sum", "precipitation_probability_max",
            "wind_speed_10m_max",
        ]),
    }
    request_url = f"{WEATHER_API_URL}?{urlencode(parameters)}"
    api_request = Request(
        request_url,
        headers={"Accept": "application/json", "User-Agent": "FarmPi-Weather/1.0"},
    )

    try:
        with urlopen(api_request, timeout=15) as response:
            result = json.loads(response.read().decode("utf-8"))

        current = result.get("current") or {}
        daily = result.get("daily") or {}
        dates = daily.get("time") or []
        days = []
        for index, date in enumerate(dates[:7]):
            def daily_value(name):
                values = daily.get(name) or []
                return values[index] if index < len(values) else None

            days.append({
                "date": date,
                "weather_code": daily_value("weather_code"),
                "temp_max": daily_value("temperature_2m_max"),
                "temp_min": daily_value("temperature_2m_min"),
                "rain_mm": daily_value("precipitation_sum"),
                "rain_chance": daily_value("precipitation_probability_max"),
                "wind_max": daily_value("wind_speed_10m_max"),
            })

        normalized_current = {
            "time": current.get("time"),
            "temp": current.get("temperature_2m"),
            "humidity": current.get("relative_humidity_2m"),
            "apparent_temp": current.get("apparent_temperature"),
            "precipitation": current.get("precipitation"),
            "weather_code": current.get("weather_code"),
            "wind_speed": current.get("wind_speed_10m"),
            "wind_dir": current.get("wind_direction_10m"),
            "is_day": current.get("is_day"),
        }

        with weather_lock:
            weather_state.update({
                "configured": True,
                "api_ok": True,
                "provider": "Open-Meteo",
                "location": WEATHER_LOCATION_LABEL,
                "error": None,
                "fetched_at": time.time(),
                "timezone": result.get("timezone"),
                "current": normalized_current,
                "daily": days,
            })
    except (HTTPError, URLError, TimeoutError, json.JSONDecodeError, ValueError) as error:
        with weather_lock:
            weather_state.update({
                "configured": True,
                "api_ok": False,
                "error": str(error),
            })
        print(f"[WEATHER] Forecast refresh failed: {error}", flush=True)

    publish_weather_state()


def weather_worker():
    while True:
        refresh_weather()
        time.sleep(WEATHER_REFRESH_SECONDS if weather_coordinates() else 300)


def hydrawise_is_configured():
    key = HYDRAWISE_API_KEY.strip()
    return bool(key and "PASTE_YOUR_" not in key and "ENTER_YOUR_" not in key)


def hydrawise_api_request(endpoint, parameters=None, include_controller=True):
    parameters = dict(parameters or {})
    parameters["api_key"] = HYDRAWISE_API_KEY.strip()
    controller_id = HYDRAWISE_CONTROLLER_ID.strip()
    if include_controller and controller_id:
        parameters["controller_id"] = controller_id
    url = f"{HYDRAWISE_API_BASE}{endpoint}?{urlencode(parameters)}"
    api_request = Request(url, headers={"Accept": "application/json", "User-Agent": "FarmPi-Hydrawise/1.0"})
    try:
        with urlopen(api_request, timeout=12) as response:
            return json.loads(response.read().decode("utf-8"))
    except HTTPError as error:
        raise RuntimeError(f"Hydrawise returned HTTP {error.code}") from None
    except URLError as error:
        raise RuntimeError(f"Could not reach Hydrawise: {error.reason}") from None
    except json.JSONDecodeError:
        raise RuntimeError("Hydrawise returned an invalid response") from None


def refresh_hydrawise():
    if not hydrawise_is_configured():
        with hydrawise_lock:
            hydrawise_state.update({
                "configured": False,
                "api_ok": False,
                "error": "API key not configured",
                "last_update": None,
                "next_poll": None,
            })
        return 10

    try:
        with hydrawise_lock:
            customer_due = not hydrawise_state.get("customer") or time.time() - hydrawise_state.get("_customer_updated", 0) >= 21600

        customer = None
        if customer_due:
            customer = hydrawise_api_request("customerdetails.php", include_controller=False)
            with hydrawise_lock:
                hydrawise_state["customer"] = customer
                hydrawise_state["_customer_updated"] = time.time()
        schedule = hydrawise_api_request("statusschedule.php")
        now = time.time()
        try:
            next_poll_seconds = max(5, int(schedule.get("nextpoll", 60)))
        except (TypeError, ValueError):
            next_poll_seconds = 60

        with hydrawise_lock:
            hydrawise_state.update({
                "configured": True,
                "api_ok": True,
                "error": None,
                "last_update": now,
                "next_poll": now + next_poll_seconds,
                "schedule": schedule,
            })
            if customer is not None:
                hydrawise_state["customer"] = customer
                hydrawise_state["_customer_updated"] = now
        return next_poll_seconds
    except Exception as error:
        now = time.time()
        with hydrawise_lock:
            hydrawise_state.update({
                "configured": True,
                "api_ok": False,
                "error": str(error),
                "next_poll": now + 60,
            })
        return 60


def hydrawise_worker():
    while True:
        delay = refresh_hydrawise()
        hydrawise_wake.wait(timeout=delay)
        hydrawise_wake.clear()


@app.route("/api/hydrawise/status")
def hydrawise_status():
    with hydrawise_lock:
        output = {key: value for key, value in hydrawise_state.items() if not key.startswith("_")}
    output["username"] = HYDRAWISE_USERNAME.strip()
    output["app_url"] = HYDRAWISE_APP_URL
    return jsonify(output)


@app.route("/api/hydrawise/action", methods=["POST"])
def hydrawise_action():
    if not hydrawise_is_configured():
        return jsonify({"error": "Hydrawise API key is not configured"}), 503

    payload = request.get_json(silent=True) or {}
    action = str(payload.get("action", "")).lower()
    allowed = {"run", "stop", "stopall", "runall", "suspend", "suspendall"}
    if action not in allowed:
        return jsonify({"error": "Unsupported Hydrawise action"}), 400

    parameters = {"action": action}
    if action in {"run", "stop", "suspend"}:
        try:
            parameters["relay_id"] = int(payload["relay_id"])
        except (KeyError, TypeError, ValueError):
            return jsonify({"error": "A valid zone ID is required"}), 400

    if action in {"run", "runall"}:
        try:
            duration = int(payload.get("custom", 300))
        except (TypeError, ValueError):
            return jsonify({"error": "Invalid watering duration"}), 400
        if not 1 <= duration <= 7200:
            return jsonify({"error": "Watering duration must be between 1 second and 2 hours"}), 400
        parameters.update({"period_id": 999, "custom": duration})

    if action in {"suspend", "suspendall"}:
        try:
            duration = int(payload.get("custom", 86400))
        except (TypeError, ValueError):
            return jsonify({"error": "Invalid suspension duration"}), 400
        if not 300 <= duration <= 604800:
            return jsonify({"error": "Suspension must be between 5 minutes and 7 days"}), 400
        parameters.update({"period_id": 999, "custom": int(time.time()) + duration})

    try:
        result = hydrawise_api_request("setzone.php", parameters)
        if result.get("message_type") == "error":
            return jsonify({"error": result.get("message", "Hydrawise rejected the command")}), 502
        message = result.get("message", "Hydrawise command accepted")
        with hydrawise_lock:
            hydrawise_state["last_action"] = message
        hydrawise_wake.set()
        return jsonify({"ok": True, "message": message})
    except Exception as error:
        return jsonify({"error": str(error)}), 502


def on_connect(client, userdata, flags, reason_code, properties):
    global mqtt_client
    mqtt_client = client
    print("[MQTT] Connected:", reason_code, flush=True)
    client.subscribe("farm/#")
    publish_weather_state()


def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        with data_lock:
            farm_data[msg.topic] = payload
            last_seen[msg.topic] = time.time()
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        print(f"[MQTT] Ignored invalid payload on {msg.topic}: {error}", flush=True)


def mqtt_worker():
    while True:
        try:
            client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
            client.on_connect = on_connect
            client.on_message = on_message
            client.connect("localhost", 1883, 60)
            client.loop_forever()
        except Exception as error:
            print(f"[MQTT] Connection error: {error}; retrying in 5 seconds", flush=True)
            time.sleep(5)


threading.Thread(target=mqtt_worker, daemon=True).start()
threading.Thread(target=hydrawise_worker, daemon=True).start()
threading.Thread(target=weather_worker, daemon=True).start()


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False)
