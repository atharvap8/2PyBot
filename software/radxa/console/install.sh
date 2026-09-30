#!/bin/bash
# 2PyBot Console installer: dependencies, MediaMTX, hotspot, and boot services.
# Run: SSID=2PyBot PSK="$HOTSPOT_PASSWORD" sudo -E bash install.sh
set -e
[ "$EUID" -eq 0 ] || { echo "run with sudo"; exit 1; }

APP_USER="${SUDO_USER:-radxa}"
APP_DIR="$(cd "$(dirname "$0")" && pwd)"
SSID="${SSID:-2PyBot}"
PSK="${PSK:?Set PSK to the hotspot password before running the installer}"
BAND="${BAND:-a}"
CHAN="${CHAN:-36}"
COUNTRY="${COUNTRY:-IN}"
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyUSB0}"
WIFI_DEV="${WIFI_DEV:-$(nmcli -t -f DEVICE,TYPE device | awk -F: '$2=="wifi"{print $1;exit}')}"

echo "== 1/7 packages"
apt-get update -qq
apt-get install -y -qq python3-pip python3-venv v4l-utils ffmpeg network-manager \
                       avahi-daemon wget iw >/dev/null

echo "== 2/7 python dependencies"
PIPFLAGS=""
if pip3 install --help 2>/dev/null | grep -q -- --break-system-packages; then
  PIPFLAGS="--break-system-packages"
fi
pip3 install $PIPFLAGS -q --upgrade pip setuptools wheel || true
pip3 install $PIPFLAGS -q -r "$APP_DIR/requirements.txt"

echo "== 3/7 MediaMTX"
if [ ! -x /usr/local/bin/mediamtx ]; then
  ARCH=$(uname -m)
  case "$ARCH" in
    aarch64|arm64) MTX=linux_arm64v8 ;;
    armv7l)        MTX=linux_armv7 ;;
    x86_64)        MTX=linux_amd64 ;;
    *) echo "unknown architecture $ARCH; install mediamtx manually"; MTX="" ;;
  esac
  if [ -n "$MTX" ]; then
    VER=$(wget -qO- https://api.github.com/repos/bluenviron/mediamtx/releases/latest \
          | jq -r .tag_name 2>/dev/null || echo "")
    [ -z "$VER" ] && VER="v1.9.3"
    echo "downloading mediamtx $VER ($MTX)"
    wget -q "https://github.com/bluenviron/mediamtx/releases/download/${VER}/mediamtx_${VER}_${MTX}.tar.gz" \
         -O /tmp/mtx.tgz && tar -xzf /tmp/mtx.tgz -C /tmp mediamtx \
      && install -m755 /tmp/mediamtx /usr/local/bin/mediamtx \
      || echo "mediamtx download failed; fetch it manually"
  fi
fi
mkdir -p /etc/2pybot
cp "$APP_DIR/mediamtx.yml" /etc/2pybot/mediamtx.yml

echo "== 4/7 permissions"
usermod -aG dialout,video "$APP_USER" || true
mkdir -p /var/lib/2pybot && chown "$APP_USER:$APP_USER" /var/lib/2pybot
cat > /etc/udev/rules.d/99-2pybot.rules <<'UDEV'
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", SYMLINK+="2pybot"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", SYMLINK+="2pybot"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", SYMLINK+="2pybot"
UDEV
udevadm control --reload 2>/dev/null || true

echo "== 5/7 hotspot on ${WIFI_DEV:-<none>} (${BAND} ch${CHAN})"
if [ -z "$WIFI_DEV" ]; then
  echo "no Wi-Fi device; console remains available on the LAN"
else
  iw reg set "$COUNTRY" 2>/dev/null || true
  AP_OK=$(iw list 2>/dev/null | grep -A12 "Supported interface modes" | grep -c "\* AP" || true)
  HAS5=$(iw list 2>/dev/null | grep -c "5180 MHz" || true)
  [ "$AP_OK" = "0" ] && echo "this Wi-Fi device may not support AP mode"
  if [ "$BAND" = "a" ] && [ "$HAS5" = "0" ]; then
    echo "no 5 GHz channels reported; using 2.4 GHz"
    BAND=bg; CHAN=6
  fi
  nmcli connection delete 2pybot-ap >/dev/null 2>&1 || true
  nmcli connection add type wifi ifname "$WIFI_DEV" con-name 2pybot-ap \
        autoconnect yes ssid "$SSID" >/dev/null
  nmcli connection modify 2pybot-ap \
        802-11-wireless.mode ap 802-11-wireless.band "$BAND" \
        802-11-wireless.channel "$CHAN" \
        802-11-wireless.powersave 2 \
        ipv4.method shared ipv6.method ignore \
        wifi-sec.key-mgmt wpa-psk wifi-sec.proto rsn wifi-sec.pairwise ccmp \
        wifi-sec.group ccmp wifi-sec.psk "$PSK" \
        connection.autoconnect-priority 100 >/dev/null
  nmcli connection up 2pybot-ap >/dev/null 2>&1 \
    && echo "hotspot up: SSID=$SSID band=$BAND channel=$CHAN gateway=10.42.0.1" \
    || echo "hotspot startup failed; check: nmcli con up 2pybot-ap"
fi

echo "== 6/7 services"
sed -e "s|__USER__|$APP_USER|" -e "s|__DIR__|$APP_DIR|" \
    "$APP_DIR/systemd/2pybot-console.service" > /etc/systemd/system/2pybot-console.service
sed -i "s|BOT_SERIAL=/dev/ttyUSB0|BOT_SERIAL=$SERIAL_PORT|" /etc/systemd/system/2pybot-console.service
cp "$APP_DIR/systemd/2pybot-mediamtx.service" /etc/systemd/system/
systemctl daemon-reload
systemctl enable 2pybot-mediamtx 2pybot-console >/dev/null
systemctl restart 2pybot-mediamtx || true
systemctl restart 2pybot-console

echo "== 7/7 done"
echo "Hotspot SSID: $SSID (band $BAND channel $CHAN)"
echo "Console: http://10.42.0.1:8080 or http://$(hostname).local:8080"
echo "WebRTC: http://10.42.0.1:8889/cam/whep"
echo "API: http://10.42.0.1:8080/api/status"
echo "Logs: journalctl -u 2pybot-console -f"
echo "Reboot once to apply dialout/video group membership."
