"""Release on one face: lower arms, open tips.

Two steps:
  1. all arms of the face → down (bring the gripper to the drop level)
  2. all tips of the face → opened   (let go)

Order is critical and not symmetric with `grab`: arms must reach the drop
level *before* the tips open, otherwise the piece falls during the descent
and lands outside the intended drop zone.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

from evo_lib.argtypes import ArgTypes
from evo_robot.ai.script import ScriptContext, script

_spec = importlib.util.spec_from_file_location(
    "_evolutek_holo_move_helper_release", Path(__file__).parent / "move.py"
)
assert _spec is not None and _spec.loader is not None
_move_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_move_module)
move = _move_module.main


@script(
    args=[
        ("face", ArgTypes.I64()),
    ]
)
def main(ctx: ScriptContext, face: int) -> None:
    log = ctx.logger()
    face = int(face)

    log.info(f"release face={face}: arms down")
    move(ctx, face, "down")

    log.info(f"release face={face}: tips opened")
    move(ctx, face, "opened")
