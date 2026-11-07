#!/usr/bin/env python3
"""Scripted session for the README: the firmware in Renode, driven over SocketCAN
by motion_host.MotionHost exactly as an operator's host would, plus the E-stop
button (SW1) pressed through the Renode monitor.

  ENABLE -> PARKED, read the force gain, ENGAGE -> ENGAGED
  launch, cornering, lane change, road bump, excessive (3 g) cue
  braking, with the E-stop pressed 2 s in: stop, descent, release, RESET -> PARKED

Usage: demo_session.py --log session.csv [--channel vcan0] [--renode-port 4444]
"""
from __future__ import annotations

import argparse
import pathlib
import time

import can

from motion_host import MotionHost, load_scenario
from renode_ctl import RenodeMonitor

SCENARIOS = ["launch", "cornering", "lane_change", "bump", "excessive"]


def ctl_wait(host: MotionHost, seconds: float) -> None:
    """Wait for `seconds` of controller time."""
    t_end = host.ctl_clock.now() + seconds * 1e3
    while host.ctl_clock.now() < t_end:
        time.sleep(0.01)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--log", type=pathlib.Path, required=True)
    p.add_argument("--channel", default="vcan0")
    p.add_argument("--renode-port", type=int, default=4444)
    a = p.parse_args()

    mon = RenodeMonitor(port=a.renode_port)
    bus = can.Bus(interface="socketcan", channel=a.channel)
    host = MotionHost(bus, a.log)
    try:
        host.wait_alive()
        print("force_gain =", host.get_param("force_gain"))
        print("ENABLE", host.command("enable")[0])
        host.wait_state("PARKED")
        host.start_stream()
        ctl_wait(host, 0.5)
        print("ENGAGE", host.command("engage")[0])
        host.wait_state("ENGAGED")
        ctl_wait(host, 1.0)
        for name in SCENARIOS:
            print("play", name)
            host.mark(name)
            host.play(load_scenario(name))
        print("play braking, E-stop after 2 s")
        host.mark("braking")
        host.play(load_scenario("braking"), wait=False)
        ctl_wait(host, 2.0)
        host.mark("E-stop")
        mon.cmd("sysbus.gpioPortG.sw1 Press")
        host.wait_state("ESTOP")
        print("ESTOP; RESET while held:", host.command("reset")[0])
        ctl_wait(host, 1.0)
        mon.cmd("sysbus.gpioPortG.sw1 Release")
        while not host.ps.flags.get("descended"):
            time.sleep(0.05)
        ctl_wait(host, 0.5)
        print("descended; RESET", host.command("reset")[0])
        host.wait_state("PARKED", timeout=20)
        ctl_wait(host, 1.0)
        print("faults:", host.ps.faults)
    finally:
        host.close()
        bus.shutdown()
        mon.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
