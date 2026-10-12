#!/bin/bash
# install_hotspot_fallback.sh — install or refresh the boot-time hotspot fallback on mjolnir (RUNBOOK "Network", mode B).
#
#   sudo rig/install_hotspot_fallback.sh           copy script + unit, enable at boot, run it once now (no-op while HP2 is up),
#                                                   and write the rtl8852ce power-save fix the hotspot needs (reboot to apply)
#   sudo rig/install_hotspot_fallback.sh remove    stop, disable, delete (keeps the modprobe fix)
#
# Afterwards:  systemctl status hotspot-fallback      journalctl -u hotspot-fallback -b
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
BIN=/usr/local/bin/hotspot_fallback.sh
UNIT=/etc/systemd/system/hotspot-fallback.service
SVC=hotspot-fallback
MODPROBE=/etc/modprobe.d/rtl8852ce-hotspot.conf

[ "$(id -u)" -eq 0 ] || { echo "run with sudo: sudo $0 ${1:-}"; exit 1; }

case "${1:-install}" in
install)
  nmcli -g connection.id con show quest-hotspot >/dev/null 2>&1 || { echo "[!!] NetworkManager profile quest-hotspot missing — see CUSTOM_SETUP.md §1 (Headset Wi-Fi)"; exit 1; }
  install -m 0755 -o root -g root "$HERE/hotspot_fallback.sh" "$BIN"
  install -m 0644 -o root -g root "$HERE/hotspot-fallback.service" "$UNIT"
  if [ ! -f "$MODPROBE" ]; then
    echo "options rtl8852ce rtw_ips_mode=0 rtw_lps_mode=0" > "$MODPROBE"
    echo "[ok] wrote $MODPROBE (rtl8852ce power-save off — AP mode wedges without it; takes effect at the next reboot)"
  fi
  systemctl daemon-reload
  systemctl enable "$SVC" >/dev/null
  systemctl restart "$SVC"      # oneshot: returns when the script is done (immediately while HP2 is connected)
  journalctl -u "$SVC" -b --no-pager -n 3 -o cat || true
  if [ "$(systemctl show -p Result --value "$SVC")" = success ]; then
    echo "[ok] $SVC enabled; at every boot it starts mjolnir-xr when HP2 is not joined within ~45-75 s"
  else
    echo "[!!] $SVC did not finish cleanly — journalctl -u $SVC -b"; exit 1
  fi
  ;;
remove)
  systemctl disable --now "$SVC" 2>/dev/null || true
  rm -f "$UNIT" "$BIN"
  systemctl daemon-reload
  echo "[ok] $SVC removed ($MODPROBE kept)"
  ;;
*)
  echo "usage: sudo $0 [install|remove]"; exit 1;;
esac
