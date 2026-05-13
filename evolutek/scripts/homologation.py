from evo_robot.ai.script import ScriptContext, script
from evo_robot.trajman.trajman import TrajmanPeripheral

from evo_lib.types.vect import Vect2D

from .grab import main as grab


@script()
def main(ctx: ScriptContext):
    trajman = ctx.peripheral("trajman", TrajmanPeripheral)

    trajman.go_to(Vect2D(300, 1100)).wait()
    grab(ctx, "front")
