from evo_robot.ai.script import ScriptContext, script
from evo_robot.trajman.trajman import TrajmanPeripheral

from evo_lib.types.vect import Vect2D
from evo_lib.types.pose import Pose2D


@script()
def main(ctx: ScriptContext):
    trajman = ctx.peripheral("trajman", TrajmanPeripheral)

    # trajman.free().wait()
    # trajman.set_position(Pose2D(300, 1100, 0)).wait()
    # trajman.unfree().wait()

    trajman.go_to(Vect2D(900, 1100)).wait()
    #grab(ctx, "front")
