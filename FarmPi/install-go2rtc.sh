#!/bin/bash
# Install/upgrade go2rtc on FarmPi and deploy the updated camera page.
# Run from a checkout of this repo:   sudo bash FarmPi/install-go2rtc.sh
# Safe to re-run. Never replaces app.py: merges the camera changes into it (with backup).
set -euo pipefail

GO2RTC_VERSION="v1.9.15"
APP_DIR="/opt/farm-dashboard"
GO2RTC_DIR="/opt/go2rtc"
SERVICE_USER="farmpi"
SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "$(id -u)" -ne 0 ]; then
  echo "Please run with sudo:  sudo bash $0"; exit 1
fi
[ -f "$APP_DIR/.env" ] || { echo "Missing $APP_DIR/.env"; exit 1; }

case "$(uname -m)" in
  aarch64|arm64) ARCH="arm64" ;;
  armv7l|armv8l) ARCH="arm" ;;
  armv6l)        ARCH="armv6" ;;
  x86_64)        ARCH="amd64" ;;
  *) echo "Unsupported CPU: $(uname -m)"; exit 1 ;;
esac

echo "==> Downloading go2rtc $GO2RTC_VERSION ($ARCH)"
TMP_BIN="$(mktemp)"
curl -fL --retry 3 -o "$TMP_BIN" \
  "https://github.com/AlexxIT/go2rtc/releases/download/$GO2RTC_VERSION/go2rtc_linux_$ARCH"
install -m 0755 "$TMP_BIN" /usr/local/bin/go2rtc
rm -f "$TMP_BIN"
/usr/local/bin/go2rtc -version || true

# The deployed app.py can be newer than Git (tank history, fault states...),
# so it is never replaced: only the camera changes are merged into it.
STAMP="$(date +%Y%m%d-%H%M%S)"
echo "==> Merging camera update into deployed app.py (backup: app.py.backup-$STAMP)"
cp -a "$APP_DIR/app.py" "$APP_DIR/app.py.backup-$STAMP"
install -d -o "$SERVICE_USER" -g "$SERVICE_USER" "$APP_DIR/tools" "$APP_DIR/static/go2rtc"
for f in make_go2rtc_config.py apply_camera_update.py camera_update_hunks.json; do
  install -o "$SERVICE_USER" -g "$SERVICE_USER" -m 0644 "$SRC_DIR/tools/$f" "$APP_DIR/tools/"
done
MERGED=1
python3 "$APP_DIR/tools/apply_camera_update.py" "$APP_DIR/app.py" || MERGED=0

echo "==> Downloading go2rtc browser player ($GO2RTC_VERSION)"
for f in video-rtc.js video-stream.js; do
  curl -fsSL --retry 3 -o "$APP_DIR/static/go2rtc/$f" \
    "https://raw.githubusercontent.com/AlexxIT/go2rtc/$GO2RTC_VERSION/www/$f"
done
chown -R "$SERVICE_USER:$SERVICE_USER" "$APP_DIR/static/go2rtc"

echo "==> Writing $GO2RTC_DIR/go2rtc.yaml from .env (private, mode 600)"
install -d -o "$SERVICE_USER" -g "$SERVICE_USER" -m 0750 "$GO2RTC_DIR"
python3 "$APP_DIR/tools/make_go2rtc_config.py" "$APP_DIR/.env" > "$GO2RTC_DIR/go2rtc.yaml.new"
mv "$GO2RTC_DIR/go2rtc.yaml.new" "$GO2RTC_DIR/go2rtc.yaml"
chown "$SERVICE_USER:$SERVICE_USER" "$GO2RTC_DIR/go2rtc.yaml"
chmod 600 "$GO2RTC_DIR/go2rtc.yaml"

echo "==> Installing services"
install -m 0644 "$SRC_DIR/go2rtc.service" /etc/systemd/system/go2rtc.service
systemctl daemon-reload
systemctl enable go2rtc >/dev/null
systemctl restart go2rtc
systemctl restart farm-dashboard

sleep 3
echo
echo "go2rtc:         $(systemctl is-active go2rtc)"
echo "farm-dashboard: $(systemctl is-active farm-dashboard)"
if command -v vcgencmd >/dev/null; then echo "power:          $(vcgencmd get_throttled) (0x0 = healthy)"; fi
echo
if [ "$MERGED" = 1 ]; then
  echo "Done. Open the Solar camera page in FarmPi. The label should read 'LIVE · DIRECT'."
else
  echo "go2rtc is installed, but the camera page could NOT be merged into your app.py."
  echo "Your app.py was left exactly as it was. Send the message above to get it merged."
fi
echo "Undo: sudo systemctl disable --now go2rtc && sudo cp $APP_DIR/app.py.backup-$STAMP $APP_DIR/app.py && sudo systemctl restart farm-dashboard"
