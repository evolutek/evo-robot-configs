"""Homologation: out-and-back along the stack lane at y=1300.

Simple straight-line trajectory exercise used to validate the full
mobility chain (Trajman + Pilot + carte-asserv + Lidar/avoidance) on a
controlled, empty table. Mirrors the spirit of an FFROB homologation:
prove the robot can leave its starting zone, traverse a defined corridor,
and come back inside the start zone.

Trajectory (coordinates are in the canonical Evo frame, mm):

       y
       ^
  1300 |  S------>------>------>------>------|
       |  |200             800        1200   |
       +--+---------------------------------- > x

  S = start zone, robot expected at (200, 1300, theta=0)
  1) S          -> (800,  1300)   leave start zone, arrive at the stack
  2) (800,1300) -> (1200, 1300)   push/cross the stack lane
  3) (1200,1300)-> (200,  1300)   come back into the start zone

Hololutek is holonomic, so the heading is irrelevant for translation —
no `head_to()` is needed between waypoints. The robot translates without
re-orienting itself.

Side handling: Trajman's internal transform mirrors coordinates based on
the side picked at GUI setup (yellow = identity, blue = mirror). The
waypoints below are written in the YELLOW reference and will be mirrored
automatically when running on BLUE — do NOT pre-mirror here.

Pre-conditions (operator):
  - Robot physically placed at (200, 1300) with theta=0.
  - Pose set via the GUI before launching the script (the script does NOT
    call `set_position()` — that would mask a bad physical placement).
  - Trajman enabled and motors un-freed (standard match-start flow).

Avoidance is left enabled (default `avoid=True`) so the lidar can stop
the robot if anything wanders onto the corridor during the test.
"""

from __future__ import annotations

from evo_lib.types.vect import Vect2D
from evo_robot.ai.script import ScriptContext, script
from evo_robot.trajman.trajman import TrajmanPeripheral

WAYPOINTS: tuple[tuple[float, float], ...] = (
    (800.0, 1300.0),   # leave start zone, reach the stack
    (1200.0, 1300.0),  # cross the stack lane
    (200.0, 1300.0),   # return into the start zone
)


@script(args=[])
def main(ctx: ScriptContext) -> None:
    log = ctx.logger()
    trajman = ctx.peripheral("trajman", TrajmanPeripheral)

    log.info("homologation: start (assumes pose ~ (200, 1300, 0))")

    for x, y in WAYPOINTS:
        log.info(f"homologation: go_to ({x:.0f}, {y:.0f})")
        status = trajman.go_to(Vect2D(x, y), avoid=True).wait()
        log.info(f"homologation: ({x:.0f}, {y:.0f}) -> {status}")

    log.info("homologation: done")
