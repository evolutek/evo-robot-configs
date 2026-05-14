"""Prepare a face for grabbing: lower arms, open barriers.

Two steps on a single face:
  1. all arms → down (gripper reaches the work zone)
  2. all barriers on that face → `barrier_pos`

`barrier_pos` is taken as an argument so the caller can pick how wide to
open (`near` / `mid` / `far`). Only fingers 1 and 4 of each face have a
barrier servo — the move dispatcher silently skips the others.

Inspired by legacy `robot_actions.py:prepare_grab(face)`, which fired
shoulders DOWN + tips OPENED + flippers A. The tips/flippers parts move
to `release` / `init` respectively in this split — `prepare` only handles
the pre-grab geometry (arms down, gate open).
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

from evo_lib.argtypes import ArgTypes
from evo_robot.ai.script import ScriptContext, script

_spec = importlib.util.spec_from_file_location(
    "_evolutek_holo_move_helper_prepare", Path(__file__).parent / "move.py"
)
assert _spec is not None and _spec.loader is not None
_move_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_move_module)
move = _move_module.main


@script(
    args=[
        ("face", ArgTypes.I64()),
        ("barrier_pos", ArgTypes.String()),
    ]
)
def main(ctx: ScriptContext, face: int, barrier_pos: str) -> None:
    log = ctx.logger()
    face = int(face)

    log.info(f"prepare face={face}: arms down")
    move(ctx, face, "down")

    log.info(f"prepare face={face}: barriers -> {barrier_pos}")
    move(ctx, face, barrier_pos)
