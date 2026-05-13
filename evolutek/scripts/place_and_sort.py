from evo_lib.argtypes import ArgTypes
from evo_lib.task import Task
from evo_lib.interfaces.color_sensor import ColorSensor

from evo_robot.ai.script import ScriptContext, script
from evo_robot.trajman.trajman import TrajmanPeripheral


def need_to_reverse(ctx: ScriptContext, side: str, arm: int) -> bool:
    color_sensor = ctx.peripheral(f"color_sensor.{side}.{arm}", ColorSensor)

    color = color_sensor.read_color().wait()
    k = color.r / color.b

    return (k > 1.5) != ctx.is_primary_team()


@script(args=[("side", ArgTypes.String(choices=["front", "back"]))])
def main(ctx: ScriptContext, side: str):
    if side != "front":
        raise RuntimeError(f"Invalid side: {side}")

    # Peripherals
    trajman = ctx.peripheral("trajman", TrajmanPeripheral)

    # Actions
    move_lifting_arm = ctx.action("move_lifting_arm")
    move_elevator = ctx.action("move_elevator")
    move_reversing_arm = ctx.action("move_reversing_arm")
    move_reversing_head = ctx.action("move_reversing_head")
    move_reversing_arm_high = ctx.action("move_reversing_arm_high")
    pump_grab = ctx.action("pump_grab")
    pump_drop = ctx.action("pump_drop")

    lifting_arms = [1, 2, 3, 4]

    # --- Read color sensors ---
    colors = [need_to_reverse(ctx, side, arm) for arm in lifting_arms]

    # --- Prepare to reverse ---
    Task.wait_all(*[
        move_lifting_arm.run(side=side, arm=arm, pos="drop")
        for arm in lifting_arms
    ])

    move_elevator.run(id="front", pos="reverse_up").wait()

    move_reversing_arm_high.run(pos="top").wait()

    # --- Start reversing colors loop ---
    for _ in range(2):
        if all(colors):
            break

        left_crate_arm = None
        right_crate_arm = None

        # Logic to determine which crates to swap (matching your original priority)
        if not colors[0]: right_crate_arm = 1
        if not colors[3]: left_crate_arm = 4
        if not colors[1] and right_crate_arm is None: right_crate_arm = 2
        if not colors[2] and left_crate_arm is None: left_crate_arm = 3
        if not colors[1] and left_crate_arm is None: left_crate_arm = 2
        if not colors[2] and right_crate_arm is None: right_crate_arm = 3

        # Move reversing arms and heads
        sync_prepare_reversing = []
        if right_crate_arm is not None:
            sync_prepare_reversing.append(move_reversing_arm.run(side="right", pos=f"crate{right_crate_arm}"))
            sync_prepare_reversing.append(move_reversing_head.run(side="right", pos="reversed"))
        if left_crate_arm is not None:
            sync_prepare_reversing.append(move_reversing_arm.run(side="left", pos=f"crate{left_crate_arm}"))
            sync_prepare_reversing.append(move_reversing_head.run(side="left", pos="reversed"))

        Task.wait_all(*sync_prepare_reversing)

        # Enable reversing arm pumps
        reversing_pump_arms = []
        if right_crate_arm is not None: reversing_pump_arms.append("reversing_right")
        if left_crate_arm is not None: reversing_pump_arms.append("reversing_left")
        Task.wait_all(*[
            pump_grab.run(side=side, arm=arm)
            for arm in reversing_pump_arms
        ])

        # Stick lifting arms and move elevator down
        sync_grab = []
        if right_crate_arm is not None:
            sync_grab.append(move_lifting_arm.run(side=side, arm=right_crate_arm, pos="grab"))
        if left_crate_arm is not None:
            sync_grab.append(move_lifting_arm.run(side=side, arm=left_crate_arm, pos="grab"))
        Task.wait_all(*sync_grab)

        move_elevator.run(id="front", pos="reverse_down").wait()

        # Swap Pump states (Drop from lifting arms)
        drop_arms = []
        if right_crate_arm is not None: drop_arms.append(f"crate_{right_crate_arm}")
        if left_crate_arm is not None: drop_arms.append(f"crate_{left_crate_arm}")
        Task.wait_all(*[
            pump_drop.run(side=side, arm=arm)
            for arm in drop_arms
        ])

        # Detach and move elevator up
        sync_detach = []
        if right_crate_arm is not None:
            sync_detach.append(move_lifting_arm.run(side=side, arm=right_crate_arm, pos="drop"))
        if left_crate_arm is not None:
            sync_detach.append(move_lifting_arm.run(side=side, arm=left_crate_arm, pos="drop"))
        Task.wait_all(*sync_detach)

        move_elevator.run(id="front", pos="reverse_up").wait()

        # Reverse heads to Normal
        sync_reverse = []
        if right_crate_arm is not None:
            sync_reverse.append(move_reversing_head.run(side="right", pos="normal"))
            colors[right_crate_arm] = True
        if left_crate_arm is not None:
            sync_reverse.append(move_reversing_head.run(side="left", pos="normal"))
            colors[left_crate_arm] = True
        Task.wait_all(*sync_reverse)

        # Reset reversing arms to neutral positions
        sync_reset = []
        if right_crate_arm is not None:
            sync_reset.append(move_reversing_arm.run(side="right", pos="crate2"))
        if left_crate_arm is not None:
            sync_reset.append(move_reversing_arm.run(side="left", pos="crate3"))
        Task.wait_all(*sync_reset)

        # Release from reversing arms
        Task.wait_all(*[
            pump_drop.run(side=side, arm=arm)
            for arm in reversing_pump_arms
        ])

        # Move base
        trajman.forward(-160).wait()

    # --- Final Cleanup ---
    Task.wait_all(
        move_reversing_head.run(side="right", pos="normal"),
        move_reversing_head.run(side="left", pos="normal")
    )

    Task.wait_all(
        move_reversing_arm.run(side="right", pos="closed"),
        move_reversing_arm.run(side="left", pos="closed")
    )

    move_reversing_arm_high.run(pos="closed").wait()

    move_elevator.run(id="front", pos="lowest").wait()

    Task.wait_all(*[
        pump_drop.run(side=side, arm=f"crate_{arm}")
        for arm in lifting_arms
    ])

    Task.wait_all(*[
        move_lifting_arm.run(side=side, arm=arm, pos="opened")
        for arm in lifting_arms
    ])

    trajman.forward(-160).wait()
