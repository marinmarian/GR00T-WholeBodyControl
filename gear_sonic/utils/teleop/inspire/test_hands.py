#!/usr/bin/env python3
"""
Open the Inspire RH56DFTP hands (all fingers to the fully-open position).

Standalone connectivity check for the vendored Modbus TCP driver.
Angle convention: 1000 = open, 0 = closed.

Usage (from repo root, .venv_teleop active):
  python gear_sonic/utils/teleop/inspire/test_hands.py                  # both hands
  python gear_sonic/utils/teleop/inspire/test_hands.py --hand right
  python gear_sonic/utils/teleop/inspire/test_hands.py --cycle          # open->close->open
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from inspire_hand_modbus import (
    InspireHandModbusTCP,
    DEFAULT_LEFT_IP,
    DEFAULT_RIGHT_IP,
    Inspire_Num_Motors,
)

OPEN = 1000
CLOSE = 0


def exercise_hand(ip, label, speed, cycle):
    hand = InspireHandModbusTCP(ip, label=label)
    if not hand.connect():
        print(f"{label}: could not connect at {ip}:{hand.port} "
              f"(is the hand powered and on the network?)")
        return False
    print(f"{label}: connected at {ip}:{hand.port}")
    try:
        hand.write_speed([speed] * Inspire_Num_Motors)
        if cycle:
            print(f"{label}: closing...")
            hand.write_angles([CLOSE] * Inspire_Num_Motors)
            time.sleep(1.5)
            print(f"{label}: angles at closed: {hand.read_angles()}")
        print(f"{label}: opening...")
        hand.write_angles([OPEN] * Inspire_Num_Motors)
        time.sleep(1.5)
        print(f"{label}: final angles: {hand.read_angles()}")
        return True
    finally:
        hand.close()


def main():
    p = argparse.ArgumentParser(description="Test the Inspire hands over Modbus TCP.")
    p.add_argument("--hand", choices=["left", "right", "both"], default="both")
    p.add_argument("--left-ip", default=os.environ.get("INSPIRE_LEFT_IP", DEFAULT_LEFT_IP))
    p.add_argument("--right-ip", default=os.environ.get("INSPIRE_RIGHT_IP", DEFAULT_RIGHT_IP))
    p.add_argument("--speed", type=int, default=500, help="finger speed 0-1000")
    p.add_argument("--cycle", action="store_true", help="close then open (default: open only)")
    args = p.parse_args()

    targets = []
    if args.hand in ("left", "both"):
        targets.append((args.left_ip, "LeftHand"))
    if args.hand in ("right", "both"):
        targets.append((args.right_ip, "RightHand"))

    ok = sum(exercise_hand(ip, label, args.speed, args.cycle) for ip, label in targets)
    print(f"done. {ok}/{len(targets)} hand(s) responded.")
    if ok == 0:
        sys.exit(1)


if __name__ == "__main__":
    main()
