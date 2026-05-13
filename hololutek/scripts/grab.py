"""Grab on one face: close all tips, stow the barriers.

Two steps:
  1. all tips of the face → closed   (clamp on whatever the gripper has reached)
  2. all barriers of the face → stored   (retract the guides so they don't snag)

This is intentionally narrower than legacy `robot_actions.py:grab(face)` —
the color-read + per-finger rejection step has moved to `store`, so `grab`
stays mechanically simple and synchronous.

Order matters: tips must close *before* the barriers retract, otherwise an
object can escape between the still-open tips and the receding barrier.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

from evo_lib.argtypes import ArgTypes
from evo_robot.ai.script import ScriptContext, script

_spec = importlib.util.spec_from_file_location(
    "_evolutek_holo_move_helper_grab", Path(__file__).parent / "move.py"
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

    log.info(f"grab face={face}: tips closed")
    move(ctx, face, "closed")

    log.info(f"grab face={face}: barriers stored")
    move(ctx, face, "stored")
