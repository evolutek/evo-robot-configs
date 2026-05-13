"""Initial actuator setup — fold tips, raise arms, set flippers to A on every face.

Mirrors the spirit of legacy `robot_actions.py:initial()`, extended with a
tip-folding pre-step so the gripper starts in its most retracted profile
before the arms swing up.

Order is intentional:
  1. tips → folded   (smallest footprint, safe before any other motion)
  2. arms → up       (clears the chassis)
  3. flippers → a    (neutral side, before any color-based rejection)

Each step uses the polymorphic `move(face, pos)` dispatcher from move.py,
which fans out over the 4 fingers of the face in a single SYNC_WRITE for
the AX12 shoulders (see move.py for the rationale).
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

from evo_robot.ai.script import ScriptContext, script

# Reuse the polymorphic `move(id, pos)` dispatcher from the sibling move.py.
# Loaded by file path because the omnissiah ScriptsManager does not register
# loaded scripts in sys.modules, so `from script.move import main` fails.
_spec = importlib.util.spec_from_file_location(
    "_evolutek_holo_move_helper_init", Path(__file__).parent / "move.py"
)
assert _spec is not None and _spec.loader is not None
_move_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_move_module)
move = _move_module.main


FACES = (1, 2, 3)


@script(args=[])
def main(ctx: ScriptContext) -> None:
    log = ctx.logger()

    log.info("init: fold tips on all faces")
    for face in FACES:
        move(ctx, face, "folded")

    log.info("init: raise arms on all faces")
    for face in FACES:
        move(ctx, face, "up")

    log.info("init: set flippers to A on all faces")
    for face in FACES:
        move(ctx, face, "a")
