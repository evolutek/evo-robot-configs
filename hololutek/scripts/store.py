"""Color-sort and pack after a grab: read each finger, reject wrong-team pieces.

Sequence:
  1. Verify tips are closed (read-back, warning-only — does not abort).
  2. Even arms (2, 4) → up; odd arms (1, 3) → half. Asymmetric heights
     give the color sensors a clean line of sight on the grabbed piece.
  3. Read each color sensor of the face, classify against the team color.
  4. For each finger whose piece is *not* the team color, flip the flipper
     to position B — the piece is dropped on the reject side.
  5. Raise the remaining odd arms (1, 3) to up so the face is uniform.

The verification in step 1 is a soft check: the grab dispatcher already
runs `move(face, "closed")`, but on a noisy bus a servo can stall (e.g.
AX12 angle-limit error). A read-back catches this without forcing the
caller to wrap every move in a try/except.

`team_color` is parsed loosely: "YELLOW"/"yellow"/"y" all map to
`NamedColor.Yellow`. The default `Palette` uses pure-color references —
swap for a calibrated `TCS34725_DEFAULT_PALETTE` once available.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

from evo_lib.argtypes import ArgTypes
from evo_lib.drivers.color_sensor.tcs34725 import TCS34725
from evo_lib.drivers.servo.pwm_servo import PWMServo
from evo_lib.types.color import PURE_COLORS, NamedColor, Palette
from evo_robot.ai.script import ScriptContext, script

_spec = importlib.util.spec_from_file_location(
    "_evolutek_holo_move_helper_store", Path(__file__).parent / "move.py"
)
assert _spec is not None and _spec.loader is not None
_move_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_move_module)
move = _move_module.main


TIP_CLOSED_TOLERANCE_DEG = 10.0


def _parse_team_color(s: str) -> NamedColor:
    """Map an input string to a NamedColor; raise if not yellow or blue."""
    k = s.strip().lower()
    if k in ("yellow", "y", "jaune", "j"):
        return NamedColor.Yellow
    if k in ("blue", "b", "bleu"):
        return NamedColor.Blue
    raise ValueError(f"store: team_color must be yellow or blue, got {s!r}")


@script(
    args=[
        ("face", ArgTypes.I64()),
        ("team_color", ArgTypes.String()),
    ]
)
def main(ctx: ScriptContext, face: int, team_color: str) -> None:
    log = ctx.logger()
    face = int(face)
    team = _parse_team_color(team_color)

    # 1) Soft verification that grab() already closed the tips. Reading every
    # tip's current angle keeps us robust to a stalled AX12 / dropped PWM.
    actions_cfg = ctx.robot().get_configs_manager().get_configs_by_name("actions")[0]
    tip_positions = (
        actions_cfg.raw.get_object("values").get_object("positions").get_object("tip")
    )
    for finger in (1, 2, 3, 4):
        arm = face * 10 + finger
        expected = float(tip_positions.get_object(str(arm))["closed"])
        servo = ctx.peripheral(f"servo_{arm}2", PWMServo)
        (measured,) = servo.get_angle().wait()
        if abs(measured - expected) > TIP_CLOSED_TOLERANCE_DEG:
            log.warning(
                f"store face={face}: tip {arm} not closed "
                f"(expected {expected:.1f}°, measured {measured:.1f}°)"
            )

    # 2) Asymmetric heights so the color sensors see the grabbed piece cleanly.
    log.info(f"store face={face}: even arms up, odd arms half")
    move(ctx, face * 10 + 2, "up")
    move(ctx, face * 10 + 4, "up")
    move(ctx, face * 10 + 1, "half")
    move(ctx, face * 10 + 3, "half")

    # 3) Color reading. We classify against the pure-color palette; tighten
    # `max_distance` once a calibrated reference set is available.
    palette = Palette(refs=PURE_COLORS)
    measured_colors: list[NamedColor] = []
    for finger in (1, 2, 3, 4):
        sensor_id = face * 10 + finger
        sensor = ctx.peripheral(f"color_sensor_{sensor_id}", TCS34725)
        try:
            (rgbc,) = sensor.read_color().wait()
            named = palette.classify(rgbc)
            log.info(
                f"store face={face}: finger {sensor_id} "
                f"r={rgbc.r} g={rgbc.g} b={rgbc.b} -> {named.name}"
            )
        except Exception as exc:
            log.warning(f"store face={face}: color_read({sensor_id}) failed: {exc}")
            named = NamedColor.Unknown
        measured_colors.append(named)

    # 4) Reject mismatches. Unknown stays — we don't blindly flip when the
    # reading failed (preserves the legacy "None -> no flip" behavior).
    for finger in (1, 2, 3, 4):
        c = measured_colors[finger - 1]
        if c is NamedColor.Unknown:
            continue
        if c is not team:
            arm = face * 10 + finger
            log.info(f"store face={face}: reject finger {arm} ({c.name} != {team.name})")
            move(ctx, arm, "b")

    # 5) Finish raising odd arms so the whole face is at "up".
    log.info(f"store face={face}: odd arms up")
    move(ctx, face * 10 + 1, "up")
    move(ctx, face * 10 + 3, "up")
