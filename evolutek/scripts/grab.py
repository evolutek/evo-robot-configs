from evo_lib.argtypes import ArgTypes
from evo_lib.task import Task

from evo_robot.ai.script import ScriptContext, script
from evo_robot.trajman.trajman import TrajmanPeripheral


@script(args=[("side", ArgTypes.String(choices=["front", "back"]))])
def main(ctx: ScriptContext, side: str):
    # Peripherals
    trajman = ctx.peripheral("trajman", TrajmanPeripheral)

    # Actions
    move_grab_elevator = ctx.action("move_grab_elevator")
    move_lifting_arm = ctx.action("move_lifting_arm")
    move_compacting_arm = ctx.action("move_compacting_arm")

    Task.wait_all(
        move_grab_elevator.run(id="front", pos="lowest"),
        move_lifting_arm.run(side=side, arm=1, pos="pre_grab"),
        move_lifting_arm.run(side=side, arm=2, pos="pre_grab"),
        move_lifting_arm.run(side=side, arm=3, pos="pre_grab"),
        move_lifting_arm.run(side=side, arm=4, pos="pre_grab"),
        move_compacting_arm.run(side=side, arm="right", pos="opened"),
        move_compacting_arm.run(side=side, arm="left", pos="opened"),
    )

    trajman.forward(130).wait()

    Task.wait_all(
        move_compacting_arm.run(side=side, arm="right", pos="pre_tassed"),
        move_compacting_arm.run(side=side, arm="left", pos="pre_tassed"),
    )
    Task.wait_all(
        move_compacting_arm.run(side=side, arm="right", pos="tassed"),
        move_compacting_arm.run(side=side, arm="left", pos="tassed"),
    )

    Task.wait_all(
        ctx.action("actuators").run.pumps_grab(ids=[2, 3, 4, 5]),
        move_lifting_arm.run(side=side, arm=1, pos="grab"),
        move_lifting_arm.run(side=side, arm=2, pos="grab"),
        move_lifting_arm.run(side=side, arm=3, pos="grab"),
        move_lifting_arm.run(side=side, arm=4, pos="grab"),
    )

    Task.wait_all(
        move_compacting_arm.run(side=side, arm="right", pos="opened"),
        move_compacting_arm.run(side=side, arm="left", pos="opened"),
    )

    Task.wait_all(
        move_lifting_arm.run(side=side, arm=1, pos="opened"),
        move_lifting_arm.run(side=side, arm=2, pos="opened"),
        move_lifting_arm.run(side=side, arm=3, pos="opened"),
        move_lifting_arm.run(side=side, arm=4, pos="opened"),
    )

    Task.wait_all(
        move_compacting_arm.run(side=side, arm="right", pos="closed"),
        move_compacting_arm.run(side=side, arm="left", pos="closed"),
    )
