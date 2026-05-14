from __future__ import annotations

from evo_lib.task import Task
from evo_robot.ai.script import ScriptContext, script


FACES = (1, 2, 3)


@script(args=[])
def main(ctx: ScriptContext) -> None:
    log = ctx.logger()

    log.info("init: fold tips on all faces")
    Task.wait_all(*(ctx.run_script("move", id=f, pos="folded") for f in FACES))

    log.info("init: arms up, flippers A, barriers stored on all faces")
    Task.wait_all(*(
        ctx.run_script("move", id=f, pos=pos)
        for f in FACES
        for pos in ("up", "a", "stored")
    ))
