"""High-level polymorphic actuator dispatch — `move(id, pos)`.

Standalone replacement for the legacy `move(id, pos)` dispatcher from
`services/python/evolutek/lib/robot/robot_actuators.py:400`, without the
hardcoded `arm_positions` / `tip_positions` / `flipper_positions` /
`barrier_positions` sets.

The legal name -> kind mapping is derived once from the actions.json5
config tree (`values.positions.{arm|tip|flipper|barrier}`), so the script
and the action engine share a single source of truth. If two kinds ever
declare the same position name, the index build raises at first call.

Dispatch by id length:
  - 1 digit  (face)  : fan out over fingers 1..4 of that face. Arms that
                       don't physically support the inferred kind (e.g.
                       barrier only on fingers 1 and 4) are silently
                       skipped, matching legacy `move_barriers`.
  - 2 digits (arm)   : single arm. Kind inferred from `pos` via the
                       reverse index.
  - 3 digits (servo) : specific non-shoulder servo on arm. Arm = id // 10,
                       kind picked from the trailing digit (1=flipper,
                       2=tip, 3=barrier per legacy `robot_actuators.py:437`).
                       `pos` is cross-checked against the kind.
"""

from __future__ import annotations

from typing import Any

from evo_lib.argtypes import ArgTypes
from evo_lib.task import Task
from evo_robot.ai.script import ScriptContext, script

ACTION_BY_KIND: dict[str, str] = {
    "arm": "move_arm",
    "tip": "move_tip_from_arm",
    "flipper": "move_flipper_from_arm",
    "barrier": "move_barrier_from_arm",
}

SERVO_DIGIT_TO_KIND: dict[int, str] = {
    1: "flipper",
    2: "tip",
    3: "barrier",
}


_position_to_kind: dict[str, str] | None = None
_arms_per_kind: dict[str, set[int]] | None = None
_positions_obj = None  # ConfigObject for `values.positions`, used for raw lookups


def _build_indexes(ctx: ScriptContext) -> None:
    """Derive name -> kind and kind -> arm-set from the loaded actions config.

    Reads the same ConfigObject the action engine consumes, so the script
    cannot disagree with the engine on the legal name space. Detects
    cross-kind name collisions and raises before any move is attempted.
    """
    global _position_to_kind, _arms_per_kind, _positions_obj

    actions_cfg = ctx.robot().get_configs_manager().get_configs_by_name("actions")[0]
    positions_obj = actions_cfg.raw.get_object("values").get_object("positions")
    _positions_obj = positions_obj

    name_to_kind: dict[str, str] = {}
    arms_per_kind: dict[str, set[int]] = {}

    for kind in ACTION_BY_KIND:
        kind_obj = positions_obj.get_object(kind)
        arms_per_kind[kind] = {int(arm_key) for arm_key in kind_obj.keys()}
        for arm_key in kind_obj.keys():
            for pos_name in kind_obj.get_object(arm_key).keys():
                prev = name_to_kind.get(pos_name)
                if prev is not None and prev != kind:
                    raise ValueError(
                        f"position name {pos_name!r} is ambiguous: "
                        f"appears in both {prev!r} and {kind!r}. "
                        f"Pick a unique name per kind in actions.json5."
                    )
                name_to_kind[pos_name] = kind

    _position_to_kind = name_to_kind
    _arms_per_kind = arms_per_kind


def _invoke(ctx: ScriptContext, kind: str, arm: int, pos: str) -> Any:
    """Fire the atomic move action; returns the running task (caller waits)."""
    action_name = ACTION_BY_KIND[kind]
    ctx.logger().info(f"-> {action_name}(arm={arm}, position={pos!r})")
    return ctx.action(action_name).run(arm=arm, position=pos)


def _kind_from_pos(pos: str) -> str:
    assert _position_to_kind is not None
    kind = _position_to_kind.get(pos)
    if kind is None:
        raise ValueError(
            f"move: unknown position {pos!r} (known: {sorted(_position_to_kind)})"
        )
    return kind


@script(
    args=[
        ("id", ArgTypes.I64()),
        ("pos", ArgTypes.String()),
    ]
)
def main(ctx: ScriptContext, id: int, pos: str) -> None:
    if _position_to_kind is None:
        _build_indexes(ctx)
    assert _arms_per_kind is not None

    s = str(int(id))

    if len(s) == 1:
        kind = _kind_from_pos(pos)
        face = int(s)
        legal_arms = _arms_per_kind[kind]
        # TODO: SYNC_WRITE for kind=="arm" once AX12Bus exposes it.
        tasks = []
        for finger in (1, 2, 3, 4):
            arm = face * 10 + finger
            if arm not in legal_arms:
                ctx.logger().debug(
                    f"move face={face} pos={pos!r}: skip arm {arm} (no {kind})"
                )
                continue
            tasks.append(_invoke(ctx, kind, arm, pos))
        Task.wait_all(*tasks)
        return

    if len(s) == 2:
        kind = _kind_from_pos(pos)
        arm = int(s)
        if arm not in _arms_per_kind[kind]:
            raise ValueError(f"move: arm {arm} has no {kind}")
        _invoke(ctx, kind, arm, pos).wait()
        return

    if len(s) == 3:
        n = int(s)
        arm = n // 10
        servo_digit = n % 10
        kind = SERVO_DIGIT_TO_KIND.get(servo_digit)
        if kind is None:
            raise ValueError(
                f"move: invalid servo digit {servo_digit} in id={id} "
                f"(expected 1=flipper, 2=tip, 3=barrier)"
            )
        if arm not in _arms_per_kind[kind]:
            raise ValueError(f"move: arm {arm} has no {kind}")
        # Legacy didn't cross-check; we do so a 3-digit call with an
        # incompatible pos fails fast instead of silently routing wrong.
        if _kind_from_pos(pos) != kind:
            raise ValueError(
                f"move: position {pos!r} does not belong to kind {kind!r} "
                f"(servo digit {servo_digit} on arm {arm})"
            )
        _invoke(ctx, kind, arm, pos).wait()
        return

    raise ValueError(f"move: invalid id {id!r} (expected 1, 2, or 3 digits)")
