from evo_lib.argtypes import ArgTypes

from evo_robot.ai.script import ScriptContext, script
from evo_robot.trajman.trajman import TrajmanPeripheral


@script(args=[("side", ArgTypes.String(choices=["left", "right"]))])
def main(ctx: ScriptContext, side: str):
    trajman = ctx.peripheral("trajman", TrajmanPeripheral)

    # On checke toutes les couleurs
    # Si elles sont toutes bonnes on lâche tout