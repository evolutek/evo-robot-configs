from __future__ import annotations

from evo_lib.drivers.smart_servo.ax12 import AX12, AX12Bus
from evo_lib.task import Task
from evo_robot.ai.script import ScriptContext, script


FACES = (1, 2, 3)
FINGERS = (1, 2, 3, 4)
ARM_SPEED = 175


@script(args=[])
def main(ctx: ScriptContext) -> None:
    log = ctx.logger()

    log.info("init: fold tips on all faces")
    Task.wait_all(*(ctx.run_script("move", id=f, pos="folded") for f in FACES))

    log.info(f"init: arms up (speed {ARM_SPEED}), flippers A, barriers stored on all faces")
    _arms_up(ctx)
    Task.wait_all(*(
        ctx.run_script("move", id=f, pos=pos)
        for f in FACES
        for pos in ("a", "stored")
    ))


def _arms_up(ctx: ScriptContext) -> None:
    actions_cfg = ctx.robot().get_configs_manager().get_configs_by_name("actions")[0]
    arm_positions = (
        actions_cfg.raw.get_object("values").get_object("positions").get_object("arm")
    )
    arm_keys = set(arm_positions.keys())

    bus = ctx.peripheral("bus_ax12", AX12Bus)
    raws: dict[int, int] = {}
    speeds: dict[int, int] = {}
    waits: list[tuple[AX12, int]] = []

    for face in FACES:
        for finger in FINGERS:
            arm = face * 10 + finger
            if str(arm) not in arm_keys:
                continue
            raw = int(arm_positions.get_object(str(arm))["up"])
            ax12 = ctx.peripheral(f"ax12_{arm}", AX12)
            raws[ax12.id] = raw
            speeds[ax12.id] = ARM_SPEED
            waits.append((ax12, raw))

    if not raws:
        return

    bus.sync_write_speeds(speeds).wait()
    bus.sync_write_goal_positions(raws).wait()
    Task.wait_all(*(ax12.wait_until_position(raw) for ax12, raw in waits))
