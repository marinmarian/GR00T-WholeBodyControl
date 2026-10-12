#!/bin/bash
# hotspot_fallback.sh — bring up the mjolnir-xr hotspot at boot when the HP2 router is not around (RUNBOOK "Network").
#
# Runs once from the systemd unit hotspot-fallback.service (sudo rig/install_hotspot_fallback.sh). Logic:
#   1. wait up to $WAIT s (default 45) for mjolnir's wifi (wlP1p1s0) to be connected — normally NetworkManager has
#      joined HP2 (autoconnect, priority 100) within ~10 s of boot;
#   2. if HP2 is reached over the cable instead (mgbe1_0, "Wired connection 2"), do nothing either;
#   3. if HP2's SSID is visible in a fresh scan, give NM another $GRACE s (default 30) — it may be mid-association;
#   4. otherwise:  nmcli con up quest-hotspot   (mode B: PICO + laptop join mjolnir-xr, mjolnir = 10.42.0.1).
# Never touches an already connected wifi, so it cannot kick mjolnir off HP2 in the lab. One radio: once the hotspot
# is up, HP2 is only re-joined by hand (~/sonic-teleop.sh net hp2) or at the next boot.
#
# Env: WAIT, GRACE (seconds), WIFI_IF (default wlP1p1s0), HP2_SSID (default HP2), HP2_GW (default 192.168.123.1),
#      HOTSPOT_CON (default quest-hotspot). Dry test of the fallback path without touching the real hotspot:
#      WAIT=1 GRACE=1 WIFI_IF=wlBOGUS HP2_GW=192.0.2.1 HOTSPOT_CON=bogus rig/hotspot_fallback.sh   (expect: FAILED, exit 1)
set -u
WAIT=${WAIT:-45}; GRACE=${GRACE:-30}
WIFI_IF=${WIFI_IF:-wlP1p1s0}; HP2_SSID=${HP2_SSID:-HP2}; HP2_GW=${HP2_GW:-192.168.123.1}; HOTSPOT_CON=${HOTSPOT_CON:-quest-hotspot}

log() { echo "hotspot-fallback: $*"; }
wifi_connected() { nmcli -t -f DEVICE,STATE dev 2>/dev/null | grep -q "^$WIFI_IF:connected"; }
hp2_via_cable() {   # mode A over mgbe1_0: a non-.222 source towards the router that answers
  local src; src=$(ip -4 route get "$HP2_GW" 2>/dev/null | sed -n 's/.* src \([0-9.]*\).*/\1/p')
  [ -n "$src" ] && [ "$src" != 192.168.123.222 ] && ping -c1 -W1 "$HP2_GW" >/dev/null 2>&1
}
settle() {   # $1 = seconds to wait for a wifi/cable connection; returns 0 as soon as one is there
  local i; for ((i = 0; i < $1; i++)); do
    if wifi_connected; then log "$WIFI_IF connected ($(nmcli -t -f NAME,DEVICE con show --active 2>/dev/null | grep ":$WIFI_IF$" | cut -d: -f1)) after ${i}s — nothing to do"; return 0; fi
    if hp2_via_cable; then log "HP2 reached over the cable after ${i}s — nothing to do"; return 0; fi
    sleep 1
  done; return 1
}

settle "$WAIT" && exit 0
if nmcli -t -f SSID dev wifi list ifname "$WIFI_IF" --rescan yes 2>/dev/null | grep -qx "$HP2_SSID"; then
  log "$HP2_SSID is visible but not joined after ${WAIT}s — giving NetworkManager ${GRACE}s more"
  settle "$GRACE" && exit 0
  log "$HP2_SSID still not joined; falling back to the hotspot"
else
  log "$HP2_SSID not visible after ${WAIT}s; falling back to the hotspot"
fi
if nmcli con up "$HOTSPOT_CON" >/dev/null 2>&1; then
  log "mode B: hotspot up ($HOTSPOT_CON on $WIFI_IF, $(ip -4 -o addr show "$WIFI_IF" 2>/dev/null | awk '{print $4}' | head -1)) — PICO + laptop join mjolnir-xr, ssh marin@10.42.0.1"
else
  log "nmcli con up $HOTSPOT_CON FAILED — see RUNBOOK 'Hotspot gone' (sudo systemctl restart wpa_supplicant NetworkManager; sudo nmcli con up $HOTSPOT_CON)"
  exit 1
fi
