from evo_lib.argtypes import ArgTypes
from evo_lib.interfaces.servo import Servo

from evo_robot.ai.script import ScriptContext, script


@script(args=[("side", ArgTypes.String(choices=["left", "right"]))])
def main(ctx: ScriptContext, side: str):
    servo = ctx.peripheral("servo", Servo)

    if side == "front":
        compacting_arms = [CompactingArmId.FRONT_LEFT, CompactingArmId.FRONT_RIGHT]
        reversing_arms = [ReversingArmId.FRONT_LEFT, ReversingArmId.FRONT_RIGHT]
        color_sensors = [1, 2, 3, 4]
    else:
        raise RuntimeError("Invalid side: %s" % side)

    ctx.get_action("move_elevator").run(id="FRONT", position="LOWEST")

    ctx.get_action("move_lifting_arm").run(LiftingArmId.FRONT_1, LiftingArmPosition.PRE_GRAB)
    ctx.get_action("move_lifting_arm").run(LiftingArmId.FRONT_2, LiftingArmPosition.PRE_GRAB)
    ctx.get_action("move_lifting_arm").run(LiftingArmId.FRONT_3, LiftingArmPosition.PRE_GRAB)
    ctx.get_action("move_lifting_arm").run(LiftingArmId.FRONT_4, LiftingArmPosition.PRE_GRAB)

    ctx.get_action("move_compacting_arm").run(CompactingArmId.FRONT_RIGHT, CompactingArmPosition.OPENED)
    ctx.get_action("move_compacting_arm").run(CompactingArmId.FRONT_LEFT, CompactingArmPosition.OPENED)
    sleep(0.2)

    ctx.get_action("forward").run(130, avoid=True)

    ctx.get_action("move_compacting_arm").run(CompactingArmId.FRONT_RIGHT, CompactingArmPosition.PRE_TASSED)
    ctx.get_action("move_compacting_arm").run(CompactingArmId.FRONT_LEFT, CompactingArmPosition.PRE_TASSED)
    sleep(0.15)
    ctx.get_action("move_compacting_arm").run(CompactingArmId.FRONT_RIGHT, CompactingArmPosition.TASSED)
    ctx.get_action("move_compacting_arm").run(CompactingArmId.FRONT_LEFT, CompactingArmPosition.TASSED)
    sleep(0.5)

    ctx.get_action("actuators").run.pumps_grab(ids=[2, 3, 4, 5])

    ctx.get_action("move_lifting_arm").run(LiftingArmId.FRONT_1, LiftingArmPosition.GRAB)
    ctx.get_action("move_lifting_arm").run(LiftingArmId.FRONT_2, LiftingArmPosition.GRAB)
    ctx.get_action("move_lifting_arm").run(LiftingArmId.FRONT_3, LiftingArmPosition.GRAB)
    ctx.get_action("move_lifting_arm").run(LiftingArmId.FRONT_4, LiftingArmPosition.GRAB)
    sleep(0.15)

    ctx.get_action("move_compacting_arm").run(CompactingArmId.FRONT_RIGHT, CompactingArmPosition.OPENED)
    ctx.get_action("move_compacting_arm").run(CompactingArmId.FRONT_LEFT, CompactingArmPosition.OPENED)
    sleep(0.06)

    ctx.get_action("move_lifting_arm").run(LiftingArmId.FRONT_1, LiftingArmPosition.OPENED)
    ctx.get_action("move_lifting_arm").run(LiftingArmId.FRONT_2, LiftingArmPosition.OPENED)
    ctx.get_action("move_lifting_arm").run(LiftingArmId.FRONT_3, LiftingArmPosition.OPENED)
    ctx.get_action("move_lifting_arm").run(LiftingArmId.FRONT_4, LiftingArmPosition.OPENED)
    sleep(0.2)

    ctx.get_action("move_compacting_arm").run(CompactingArmId.FRONT_RIGHT, CompactingArmPosition.CLOSED)
    ctx.get_action("move_compacting_arm").run(CompactingArmId.FRONT_LEFT, CompactingArmPosition.CLOSED)
    sleep(0.3)
