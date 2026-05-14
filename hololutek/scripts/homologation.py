"""Homologation: out-and-back along the stack lane at y=1300, presenting
a face at each waypoint via a head_to before each go_to.

Pre-conditions:
  - Robot physically placed at START_POSE (200, 1300, theta=0).
  - Trajman enabled. The script handles set_position + unfree itself.

Avoidance left enabled (default avoid=True).
"""

from __future__ import annotations

from math import pi

from evo_lib.types.pose import Pose2D
from evo_lib.types.vect import Vect2D
from evo_robot.ai.script import ScriptContext, script
from evo_robot.trajman.trajman import TrajmanPeripheral

START_POSE = Pose2D(x=200.0, y=1300.0, heading=0.0)

WAYPOINTS: tuple[Pose2D, ...] = (
    Pose2D(x=800.0, y=1300.0, heading=0.0),
    Pose2D(x=1200.0, y=1300.0, heading=2 * pi / 3),
    Pose2D(x=200.0, y=1300.0, heading=-2 * pi / 3),
)


@script(args=[])
def main(ctx: ScriptContext) -> None:
    log = ctx.logger()
    trajman = ctx.peripheral("trajman", TrajmanPeripheral)

    log.info(
        f"homologation: set_position ({START_POSE.x:.0f}, "
        f"{START_POSE.y:.0f}, theta={START_POSE.heading:.3f} rad)"
    )
    trajman.set_position(START_POSE)

    log.info("homologation: unfree motors")
    trajman.unfree().wait()

    for pose in WAYPOINTS:
        log.info(f"homologation: head_to ({pose.heading:.3f} rad)")
        trajman.head_to(pose.heading, avoid=True).wait()
        log.info(f"homologation: go_to ({pose.x:.0f}, {pose.y:.0f})")
        status = trajman.go_to(Vect2D(pose.x, pose.y), avoid=True).wait()
        log.info(f"homologation: ({pose.x:.0f}, {pose.y:.0f}) -> {status}")

    log.info("homologation: done")
