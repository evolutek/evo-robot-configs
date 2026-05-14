"""Hololutek 2026 — verbatim port of the legacy strategy, with inlined helpers.

Sources (services repo, branch holo-2026 @ commit 629b44f6 'config belgique'):
  - python/evolutek/services/ai.py            (orchestrator FSM + Selecting/Making loop)
  - python/evolutek/lib/ai/goals.py            (Goals manager: selection, secondary)
  - python/evolutek/lib/robot/robot_actions.py (prepare_grab, grab, drop, cursor, barrier)
  - python/evolutek/lib/robot/robot_actuators.py (move dispatcher, move_* primitives)
  - python/conf.d/strategies.json              (goals + 2 strategies + 2 starting positions)

Design choice: monolithic. The full Selecting -> Making -> Selecting loop, the
Goals manager, the strategies.json payload, AND the polymorphic legacy helpers
(`move`, `barrier`, `cursor`, `cursor_stow`, `prepare_grab`, `grab`, `drop`,
`sleep`) all live inline in this file. Reason: this is a *replay* of the
holo-2026 legacy strategy, not a starting point for new work. Keeping it
monolithic preserves the boundary between "what was the 2026 strategy" and
"what is reusable infrastructure".

The legacy FSM (Setup/Waiting/Ending/Error) is delegated to omnissiah's
RobotFSM. This script encapsulates only the inner Selecting -> Making loop
plus the per-strategy starting-pose and (optional) recalibration.

Hardware mapping
----------------
Two categories of legacy calls:

  1) Pure dispatchers (no state of their own): `move`, `barrier`, `cursor`,
     `cursor_stow`, `prepare_grab`, `grab`, `drop`, `sleep`. These are
     implemented as Python helpers in this file. Each helper calls into the
     four atomic actions declared by `actions.json5`:
       - move_arm(arm, position)
       - move_tip_from_arm(arm, position)
       - move_flipper_from_arm(arm, position)
       - move_barrier_from_arm(arm, position)

  2) Movement & sensors (`goto_avoid`, `goto_with_path`, `goth`, `set_pose`,
     `recalibration`, color sensors): not yet ported to actions.json5. The
     `_invoke_action` helper looks each up via `ctx.action(name)`; if missing
     it logs a warning and treats as Done so the loop keeps flowing. Useful
     for dry-run validation when downstream actions are still stubs.

Two-color handling
------------------
Unlike belgium-2025, holo-2026 does NOT mirror the goal map at runtime.
Instead, the two strategies "match jaune" and "match bleu" share the goal
sequence and only differ in starting_position and exit_home_X. Behaviour
preserved: no Y mirroring. Inside helpers, the legacy `self.side` flag is
mirrored by the module-level `_SIDE` variable set from the strategy name.

Known legacy quirks (preserved)
-------------------------------
- `return_home` is defined TWICE in strategies.json. The second entry
  (250, 300, theta=-5*pi/6) wins in a Python dict and is the one referenced
  by both strategies. The first (400, 400, theta=-pi/6) is dead config.
- `drop_home.theta` is the literal "-pi/pi/2" which evaluates to -0.5 rad in
  Python. Likely a typo for "-pi/2", but mirrored verbatim.
- `_legacy_barrier` on the yellow side uses a remap formula that produces
  out-of-range arm ids (e.g. arm 7 for barrier 31 yellow). Reproduced as-is
  with a warning — see `_legacy_barrier` docstring.
- `_legacy_grab` swaps colors: side 0 (yellow) keeps BLUE, side 1 (blue)
  keeps YELLOW. Preserved verbatim from legacy `robot_actions.py:71-72`.
"""

from __future__ import annotations

import time as _time
from dataclasses import dataclass, field
from enum import Enum
from math import pi
from threading import Event, Timer
from time import time
from typing import Any

from evo_lib.argtypes import ArgTypes
from evo_robot.ai.script import ScriptContext, script

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

MATCH_DURATION_S = 100.0

# Position vocabularies (verbatim from robot_actuators.py:404-407).
# Determine which actuator a legacy `move(id, pos)` targets.
ARM_POSITIONS = {"up", "half", "down", "cursor"}
TIP_POSITIONS = {"flipped", "open", "closed", "close", "openned", "opened"}
BARRIER_POSITIONS = {"stored", "near", "mid", "far", "stow", "store"}
FLIPPER_POSITIONS = {"a", "b"}

# Module-level `side` flag, set by main() from the chosen strategy. The legacy
# code threaded `self.side` through every method; here we keep it global so the
# helpers stay simple. Convention from legacy `cursor()`/`barrier()`:
#   - side = 0 (False) -> yellow
#   - side = 1 (True)  -> blue
_SIDE = 0


class RobotStatus(str, Enum):
    """Mirror of evolutek.lib.status.RobotStatus, kept verbatim for traceability."""

    Done = "Done"
    Reached = "Reached"
    NotReached = "NotReached"
    Aborted = "Aborted"
    Timeout = "Timeout"
    Unreachable = "Unreachable"
    Ok = "Ok"
    Failed = "Failed"


class AvoidStrategy(Enum):
    """Mirror of evolutek.lib.ai.goals.AvoidStrategy."""

    Wait = "wait"
    Timeout = "timeout"
    Skip = "skip"


# ---------------------------------------------------------------------------
# Inlined data — verbatim from python/conf.d/strategies.json (holo-2026)
# ---------------------------------------------------------------------------

STARTING_POSITIONS: dict[str, dict[str, Any]] = {
    "jaune": {"x": 155, "y": 495, "theta": 0.0},
    "bleu": {"x": 155, "y": 495, "theta": pi},
}


# Each entry corresponds to one entry in the legacy goals[] array.
# `theta` strings ("-pi/6", "5*pi/6", ...) have been pre-evaluated to floats.
# Note the duplicated `return_home` and the "-pi/pi/2" quirk — see module
# docstring "Known legacy quirks".
GOALS_DATA: list[dict[str, Any]] = [
    {
        "name": "drop_banner",
        "x": 1875, "y": 1225, "theta": -pi / 2,
        "actions": [
            {"fct": "drop", "args": {"group": "elevator_AB"}},
            {"fct": "grab", "args": {"group": "outer_CA"}},
            {"fct": "grab", "args": {"group": "elevator_CA"}},
        ],
    },
    {"name": "grab_first", "x": 1100, "y": 1100, "theta": -pi / 6, "actions": []},
    {
        "name": "drop_first",
        "x": 1700, "y": 1225, "theta": 2 * pi / 3,
        "actions": [
            {"fct": "drop", "args": {"group": "elevator_CA"}},
            {"fct": "drop", "args": {"group": "outer_CA"}},
        ],
    },
    # First `return_home` (400, 400) — legacy duplicate, dead config.
    # Kept for transparency, overwritten by the second entry below.
    {"name": "return_home", "x": 400, "y": 400, "theta": -pi / 6, "actions": []},

    {"name": "exit_home_yellow", "x": 800, "y": 450, "theta": 0.0, "actions": []},
    {"name": "exit_home_blue", "x": 800, "y": 450, "theta": pi, "actions": []},

    {
        "name": "present_upper_zone",
        "x": 950, "y": 1100, "theta": pi / 6,
        "actions": [
            {"fct": "prepare_grab", "args": {"face": "2"}},
            {"fct": "barrier", "args": {"id": "24", "pos": "far"}},
        ],
    },
    {
        "name": "grab_upper_zone",
        "x": 1000, "y": 1200, "theta": pi / 6,
        "actions": [{"fct": "grab", "args": {"face": "2"}}],
    },
    {
        "name": "align_lower_zone",
        "x": 1550, "y": 1200, "theta": 5 * pi / 6,
        "actions": [
            {"fct": "prepare_grab", "args": {"face": "3"}},
            {"fct": "barrier", "args": {"id": "31", "pos": "far"}},
        ],
    },
    {
        "name": "grab_lower_zone",
        "x": 1630, "y": 1000, "theta": 5 * pi / 6,
        "actions": [{"fct": "grab", "args": {"face": "3"}}],
    },
    {
        "name": "align_cursor",
        "x": 1750, "y": 1350, "theta": -pi / 2,
        "actions": [{"fct": "cursor", "args": {}}],
    },
    {
        "name": "go_cursor",
        "x": 1750, "y": 720, "theta": -pi / 2,
        "actions": [{"fct": "cursor_stow", "args": {}}],
    },
    {
        "name": "drop_lower",
        "x": 1600, "y": 700, "theta": 5 * pi / 6,
        "actions": [
            {"fct": "move", "args": {"id": "3", "pos": "down"}},
            {"fct": "sleep", "args": {"time": "1"}},
            {"fct": "move", "args": {"id": "3", "pos": "open"}},
        ],
    },
    {
        "name": "exit_lower",
        "x": 1500, "y": 700, "theta": 5 * pi / 6,
        "actions": [{"fct": "move", "args": {"id": "3", "pos": "up"}}],
    },
    {
        "name": "present_left_lower",
        "x": 1500, "y": 450, "theta": pi / 3,
        "actions": [
            {"fct": "prepare_grab", "args": {"face": "3"}},
            {"fct": "barrier", "args": {"id": "34", "pos": "far"}},
        ],
    },
    {
        "name": "grab_left_lower",
        "x": 1600, "y": 400, "theta": pi / 3,
        "actions": [{"fct": "grab", "args": {"face": "3"}}],
    },
    {
        "name": "drop_left",
        "x": 1200, "y": 400, "theta": pi / 3,
        "actions": [
            {"fct": "move", "args": {"id": "3", "pos": "down"}},
            {"fct": "sleep", "args": {"time": "1"}},
            {"fct": "move", "args": {"id": "3", "pos": "open"}},
        ],
    },
    {
        "name": "exit_left",
        "x": 1200, "y": 500, "theta": pi / 3,
        "actions": [{"fct": "move", "args": {"id": "3", "pos": "up"}}],
    },
    {
        "name": "present_left_upper",
        "x": 900, "y": 500, "theta": pi / 3,
        "actions": [
            {"fct": "prepare_grab", "args": {"face": "3"}},
            {"fct": "barrier", "args": {"id": "31", "pos": "far"}},
        ],
    },
    {
        "name": "grab_left_upper",
        "x": 700, "y": 440, "theta": pi / 3,
        "actions": [{"fct": "grab", "args": {"face": "3"}}],
    },
    {
        "name": "drop_middle",
        "x": 1100, "y": 600, "theta": pi / 2,
        "actions": [
            {"fct": "move", "args": {"id": "2", "pos": "down"}},
            {"fct": "sleep", "args": {"time": "1"}},
            {"fct": "move", "args": {"id": "2", "pos": "open"}},
        ],
    },
    {
        "name": "exit_middle",
        "x": 950, "y": 450, "theta": pi / 2,
        "actions": [{"fct": "move", "args": {"id": "2", "pos": "up"}}],
    },
    # Second `return_home` overrides the first one above.
    {"name": "return_home", "x": 250, "y": 300, "theta": -5 * pi / 6, "actions": []},
    {
        "name": "drop_home",
        # Legacy literal "-pi/pi/2" -> eval = -0.5 rad. Almost certainly a typo
        # for -pi/2, but preserving legacy behavior. Change with care.
        "x": 250, "y": 300, "theta": -pi / pi / 2,
        "actions": [
            {"fct": "move", "args": {"id": "3", "pos": "down"}},
            {"fct": "sleep", "args": {"time": "1"}},
            {"fct": "move", "args": {"id": "3", "pos": "open"}},
            {"fct": "sleep", "args": {"time": "1"}},
            {"fct": "move", "args": {"id": "3", "pos": "up"}},
        ],
    },
]


# Both strategies share the same long match sequence; only exit_home_X differs.
_COMMON_GOALS_TAIL = [
    "present_upper_zone",
    "grab_upper_zone",
    "align_lower_zone",
    "grab_lower_zone",
    "align_cursor",
    "go_cursor",
    "drop_lower",
    "exit_lower",
    "present_left_lower",
    "grab_left_lower",
    "drop_left",
    "exit_left",
    "present_left_upper",
    "grab_left_upper",
    "drop_middle",
    "exit_middle",
    "return_home",
    "drop_home",
]

STRATEGIES_DATA: dict[str, dict[str, Any]] = {
    "match jaune": {
        "starting_position": "jaune",
        "side": 0,  # legacy: not self.side -> yellow
        "use_pathfinding": False,
        "goals": ["exit_home_yellow", *_COMMON_GOALS_TAIL],
    },
    "match bleu": {
        "starting_position": "bleu",
        "side": 1,  # legacy: self.side -> blue
        "use_pathfinding": False,
        "goals": ["exit_home_blue", *_COMMON_GOALS_TAIL],
    },
}


# ---------------------------------------------------------------------------
# Domain types — minimal port of evolutek/lib/ai/goals.py
# ---------------------------------------------------------------------------


@dataclass
class Action:
    """A single legacy AI action: hardware call with optional avoid behavior."""

    fct: str
    args: dict[str, Any] = field(default_factory=dict)
    avoid_strategy: AvoidStrategy = AvoidStrategy.Wait
    timeout: float | None = None


@dataclass
class Goal:
    """A goal: position to reach + optional theta + ordered list of actions."""

    name: str
    x: float
    y: float
    theta: float | None
    actions: list[Action]
    secondary_goal: str | None = None
    timeout: float | None = None


@dataclass
class Strategy:
    """A strategy: starting pose + side flag + ordered list of goal names."""

    name: str
    starting_position: dict[str, Any]
    side: int
    goals: list[str]
    use_pathfinding: bool


class Goals:
    """Manager mirroring evolutek/lib/ai/goals.py:Goals (holo-2026 trim)."""

    def __init__(self) -> None:
        self.starting_positions = STARTING_POSITIONS
        self.goals: dict[str, Goal] = {}
        for raw in GOALS_DATA:
            # Last write wins — preserves the legacy `return_home` override.
            self.goals[raw["name"]] = Goal(
                name=raw["name"],
                x=raw["x"],
                y=raw["y"],
                theta=raw["theta"],
                actions=[
                    Action(
                        fct=a["fct"],
                        args=dict(a.get("args", {})),
                        avoid_strategy=AvoidStrategy(a.get("avoid_strategy", "wait")),
                        timeout=a.get("timeout"),
                    )
                    for a in raw.get("actions", [])
                ],
                secondary_goal=raw.get("secondary_goal"),
                timeout=raw.get("timeout"),
            )

        self.strategies: dict[str, Strategy] = {}
        for sname, sraw in STRATEGIES_DATA.items():
            self.strategies[sname] = Strategy(
                name=sname,
                starting_position=self.starting_positions[sraw["starting_position"]],
                side=sraw["side"],
                goals=list(sraw["goals"]),
                use_pathfinding=sraw["use_pathfinding"],
            )

        self.current_strategy: Strategy | None = None
        self.current = 0

    def reset(self, name: str) -> None:
        if name not in self.strategies:
            raise ValueError(f"Unknown strategy: {name}")
        self.current_strategy = self.strategies[name]
        self.current = 0

    def get_goal(self) -> Goal | None:
        assert self.current_strategy is not None
        if self.current >= len(self.current_strategy.goals):
            return None
        return self.goals[self.current_strategy.goals[self.current]]

    def get_secondary_goal(self, name: str) -> Goal:
        return self.goals[name]

    def finish_goal(self) -> None:
        self.current += 1


# ---------------------------------------------------------------------------
# Low-level action invocation
# ---------------------------------------------------------------------------


def _invoke_atomic_action(
    ctx: ScriptContext,
    fct: str,
    args: dict[str, Any],
    timeout: float | None = None,
) -> RobotStatus:
    """Invoke a named omnissiah action (the ones declared in actions.json5).

    Falls back to Done with a warning if the action is missing, so the
    Selecting/Making loop keeps flowing even on partially-implemented robots.
    """

    log = ctx.logger()
    try:
        action = ctx.action(fct)
    except Exception as exc:
        log.warning(f"action '{fct}' not available — treating as Done ({exc})")
        return RobotStatus.Done

    log.info(f"-> {fct}({args})")
    try:
        task = action.run(args)
    except Exception as exc:
        log.error(f"action '{fct}' failed to start: {exc}")
        return RobotStatus.Aborted

    try:
        task.wait(timeout=timeout)
    except Exception as exc:
        cls = type(exc).__name__
        if cls == "TaskTimeoutError":
            log.warning(f"action '{fct}' timed out after {timeout}s")
            return RobotStatus.Timeout
        if cls == "TaskCancelledError":
            log.info(f"action '{fct}' aborted")
            return RobotStatus.Aborted
        log.error(f"action '{fct}' raised {cls}: {exc}")
        return RobotStatus.NotReached
    return RobotStatus.Done


# ---------------------------------------------------------------------------
# Legacy helpers — verbatim ports of robot_actuators.py / robot_actions.py
# ---------------------------------------------------------------------------


def _atomic_action_for_pos(pos: str) -> str | None:
    """Return the actions.json5 name that matches a legacy `move()` pos category."""
    if pos in ARM_POSITIONS:
        return "move_arm"
    if pos in TIP_POSITIONS:
        return "move_tip_from_arm"
    if pos in BARRIER_POSITIONS:
        return "move_barrier_from_arm"
    if pos in FLIPPER_POSITIONS:
        return "move_flipper_from_arm"
    return None


def _legacy_move(ctx: ScriptContext, id: str | int, pos: str) -> RobotStatus:
    """Reproduce legacy `move(id, pos)` dispatcher (robot_actuators.py:400).

    `id` length determines target:
      - 1 digit ('3'): face — applies pos to all 4 arms of the face
      - 2 digits ('31'): arm — single arm
      - 3 digits ('312'): servo — arm = first two digits, servo type from last
        digit (1=tip, 2=flipper, 3=barrier)
    """
    log = ctx.logger()
    action_name = _atomic_action_for_pos(pos)
    if action_name is None:
        log.error(f"_legacy_move: unknown pos '{pos}'")
        return RobotStatus.Failed

    n = int(id)
    s = str(n)

    if len(s) == 1:
        # face: apply to all 4 arms (face N -> arms N1..N4)
        results = [
            _invoke_atomic_action(ctx, action_name, {"arm": n * 10 + k, "position": pos})
            for k in range(1, 5)
        ]
        if all(r in (RobotStatus.Done, RobotStatus.Reached) for r in results):
            return RobotStatus.Done
        return RobotStatus.NotReached

    if len(s) == 2:
        return _invoke_atomic_action(ctx, action_name, {"arm": n, "position": pos})

    if len(s) == 3:
        arm = n // 10
        servo_digit = n % 10
        # Note: digit 1=tip, 2=flipper, 3=barrier (verbatim from legacy
        # robot_actuators.py:437). The action_name from pos must agree with the
        # servo digit, otherwise the move targets the wrong servo. The legacy
        # code didn't enforce that either.
        servo_action = {1: "move_tip_from_arm", 2: "move_flipper_from_arm", 3: "move_barrier_from_arm"}.get(servo_digit)
        if servo_action is None:
            log.error(f"_legacy_move: unknown servo digit '{servo_digit}'")
            return RobotStatus.Failed
        return _invoke_atomic_action(ctx, servo_action, {"arm": arm, "position": pos})

    log.error(f"_legacy_move: bad id '{id}'")
    return RobotStatus.Failed


def _legacy_barrier(ctx: ScriptContext, id: str | int, pos: str) -> RobotStatus:
    """Reproduce legacy `barrier(id, pos)` (robot_actions.py:44-57).

    The legacy code remaps `id` on the yellow side (`not self.side`):
        new_id = id // 10
        if id % 10 == 1: new_id += 4
        else:            new_id += 1

    This produces single-digit "face-like" ids that don't correspond to real
    arms (e.g. id=31, yellow -> new_id=7). The dispatch into `move(new_id, pos)`
    then takes the 1-digit "face" branch with a non-existent face. Almost
    certainly a bug in the legacy code, reproduced as-is.
    """
    n = int(id)
    new_id: int
    if not _SIDE:  # yellow
        new_id = n // 10
        if n % 10 == 1:
            new_id += 4
        else:
            new_id += 1
        ctx.logger().warning(
            f"_legacy_barrier yellow remap: id={n} -> new_id={new_id} "
            f"(legacy bug preserved; new_id is likely out of range)"
        )
    else:  # blue
        new_id = n
    return _legacy_move(ctx, new_id, pos)


def _legacy_cursor(ctx: ScriptContext) -> RobotStatus:
    """Reproduce legacy `cursor()` (robot_actions.py:18-27): cursor arm + open tip."""
    arm_id = 14 if not _SIDE else 11
    s1 = _legacy_move(ctx, arm_id, "cursor")
    if s1 not in (RobotStatus.Done, RobotStatus.Reached):
        return s1
    return _legacy_move(ctx, arm_id, "open")


def _legacy_cursor_stow(ctx: ScriptContext) -> RobotStatus:
    """Reproduce legacy `cursor_stow()` (robot_actions.py:29-39): retract arm."""
    arm_id = 14 if not _SIDE else 11
    return _legacy_move(ctx, arm_id, "up")


def _legacy_prepare_grab(ctx: ScriptContext, face: str | int) -> RobotStatus:
    """Reproduce legacy `prepare_grab(face)` (robot_actions.py:60-67).

    Three moves on the whole face:
      1. shoulders DOWN
      2. tips OPENED
      3. flippers A
    """
    face = int(face)
    statuses = [
        _legacy_move(ctx, face, "down"),
        _legacy_move(ctx, face, "opened"),
        _legacy_move(ctx, face, "a"),
    ]
    if all(s in (RobotStatus.Done, RobotStatus.Reached) for s in statuses):
        return RobotStatus.Done
    return RobotStatus.NotReached


def _legacy_grab(ctx: ScriptContext, face: str | int) -> RobotStatus:
    """Reproduce legacy `grab(face)` (robot_actions.py:70-108).

    Color-sensor read is best-effort: each `color_enable`/`color_read` call goes
    through `_invoke_atomic_action`, so missing actions degrade to Done and an
    empty color list, in which case the per-finger flipper-reject step is a
    no-op.
    """
    log = ctx.logger()
    face = int(face)

    # Legacy swap: side 0 (yellow) wants BLUE pieces, side 1 (blue) wants YELLOW.
    # Verbatim from robot_actions.py:71-74.
    goal_color = "BLUE" if not _SIDE else "YELLOW"

    statuses: list[RobotStatus] = []
    statuses.append(_legacy_move(ctx, face, "closed"))
    _time.sleep(0.5)

    # Enable color sensors on all 4 fingers of the face.
    for i in range(1, 5):
        sensor_id = i + face * 10
        _invoke_atomic_action(ctx, "color_enable", {"id": sensor_id, "enabled": 1})

    _time.sleep(0.1)

    # Read colors. Missing color_read action => empty / unknown colors.
    colors: list[str | None] = []
    for i in range(1, 5):
        sensor_id = i + face * 10
        try:
            color_action = ctx.action("color_read")
            task = color_action.run({"id": sensor_id})
            result = task.wait(timeout=1.0)
            color = result[0] if result else None
        except Exception as exc:
            log.warning(f"color_read({sensor_id}) unavailable: {exc}")
            color = None
        log.info(f"finger {sensor_id} reads color={color}")
        colors.append(color)

    for i in range(1, 5):
        _invoke_atomic_action(ctx, "color_enable", {"id": i + face * 10, "enabled": 0})

    # Raise arms individually (legacy pattern: 1=up, 2=half, 3=up, 4=half).
    statuses.append(_legacy_move(ctx, face * 10 + 1, "up"))
    statuses.append(_legacy_move(ctx, face * 10 + 2, "half"))
    statuses.append(_legacy_move(ctx, face * 10 + 3, "up"))
    statuses.append(_legacy_move(ctx, face * 10 + 4, "half"))
    _time.sleep(0.5)

    # Per-finger color filter: if read color != goal_color, flip the finger to
    # position "b" (reject). When colors are unavailable, the comparison is
    # `None != goal_color` which is True — so EVERY finger would flip. To avoid
    # rejecting blindly we skip the filter when colors come back as None.
    for i in range(1, 5):
        c = colors[i - 1]
        if c is not None and c != goal_color:
            statuses.append(_legacy_move(ctx, face * 10 + i, "b"))
    _time.sleep(0.5)

    statuses.append(_legacy_move(ctx, face, "up"))

    if all(s in (RobotStatus.Done, RobotStatus.Reached) for s in statuses):
        return RobotStatus.Done
    return RobotStatus.NotReached


def _legacy_drop(ctx: ScriptContext, **kwargs: Any) -> RobotStatus:
    """Reproduce legacy `drop()` — magnet drop, currently a stub on Hololutek.

    Legacy `drop(group=...)` triggers magnet groups (elevator_AB, outer_CA,
    elevator_CA). Those EVs are not yet wired into actions.json5. Stub returns
    Done after a warning so the loop continues; goal sequencing can still be
    validated.
    """
    ctx.logger().warning(f"_legacy_drop: stub (no magnet action available) args={kwargs}")
    return RobotStatus.Done


def _legacy_sleep(ctx: ScriptContext, time: str | float) -> RobotStatus:
    """Reproduce legacy `sleep(time)` — `time.sleep(float(t))`.

    Note: the legacy strategies.json passes `time` as a string ("1"). We coerce.
    """
    seconds = float(time)
    ctx.logger().info(f"sleep({seconds:.2f}s)")
    _time.sleep(seconds)
    return RobotStatus.Done


# ---------------------------------------------------------------------------
# Dispatcher: legacy fct name -> helper or atomic action
# ---------------------------------------------------------------------------


def _invoke_legacy_action(
    ctx: ScriptContext,
    fct: str,
    args: dict[str, Any],
    timeout: float | None = None,
) -> RobotStatus:
    """Dispatch a legacy fct name to a local helper, or fall through to actions.json5.

    Order of resolution:
      1. Legacy helper (move, barrier, cursor, cursor_stow, prepare_grab,
         grab, drop, sleep) — handles polymorphism that doesn't fit the
         atomic action model.
      2. Atomic action declared in actions.json5 — via _invoke_atomic_action.
      3. Missing — _invoke_atomic_action logs a warning and returns Done.
    """
    if fct == "move":
        return _legacy_move(ctx, args["id"], args["pos"])
    if fct == "barrier":
        return _legacy_barrier(ctx, args["id"], args["pos"])
    if fct == "cursor":
        return _legacy_cursor(ctx)
    if fct == "cursor_stow":
        return _legacy_cursor_stow(ctx)
    if fct == "prepare_grab":
        return _legacy_prepare_grab(ctx, args["face"])
    if fct == "grab":
        if "face" in args:
            return _legacy_grab(ctx, args["face"])
        # `grab(group=...)` form used by `drop_banner` for outer_CA/elevator_CA
        # magnet groups — semantically a magnet capture, currently a stub.
        ctx.logger().warning(f"grab(group=...) is a stub on Hololutek: args={args}")
        return RobotStatus.Done
    if fct == "drop":
        return _legacy_drop(ctx, **args)
    if fct == "sleep":
        return _legacy_sleep(ctx, args["time"])

    return _invoke_atomic_action(ctx, fct, args, timeout=timeout)


def _run_goto(ctx: ScriptContext, goal: Goal, use_pathfinding: bool) -> RobotStatus:
    """Replicate the legacy goto policy: pathfinding or direct."""
    if use_pathfinding:
        return _invoke_atomic_action(ctx, "goto_with_path", {"x": goal.x, "y": goal.y}, timeout=goal.timeout)
    return _invoke_atomic_action(ctx, "goto_avoid", {"x": goal.x, "y": goal.y}, timeout=goal.timeout)


def _run_goth(ctx: ScriptContext, theta: float) -> RobotStatus:
    return _invoke_atomic_action(ctx, "goth", {"theta": theta})


# ---------------------------------------------------------------------------
# Main entry point — exposed as a script:* node in the graph editor
# ---------------------------------------------------------------------------


@script(args=[
    ("strategy", ArgTypes.String(choices=list(STRATEGIES_DATA.keys()))),
    ("recalibrate", ArgTypes.Bool()),
])
def main(ctx: ScriptContext, strategy: str, recalibrate: bool) -> None:
    """Hololutek 2026 strategy entry point.

    Args:
        strategy: "match jaune" or "match bleu" (legacy strategy names).
        recalibrate: whether to run wall recalibration after setting start pose.
    """
    global _SIDE

    log = ctx.logger()
    log.info(f"=== Hololutek 2026 strategy: {strategy} (recal={recalibrate}) ===")

    goals = Goals()
    goals.reset(strategy)
    assert goals.current_strategy is not None
    _SIDE = goals.current_strategy.side  # picked up by all _legacy_* helpers

    score = 0  # holo-2026 strategies.json has no per-goal scores; kept for log parity
    match_starting_time = time()
    match_end = Event()
    end_timer = Timer(MATCH_DURATION_S, match_end.set)
    end_timer.daemon = True
    end_timer.start()

    try:
        # ---- SETUP-equivalent: starting pose + (optional) recalibration ----
        sp = goals.current_strategy.starting_position
        log.info(f"setting starting pose ({sp['x']}, {sp['y']}, {sp['theta']:.3f}), side={_SIDE}")
        _invoke_atomic_action(ctx, "set_pose", {"x": sp["x"], "y": sp["y"], "theta": sp["theta"]})

        if recalibrate:
            log.info("running wall recalibration")
            recal_status = _invoke_atomic_action(ctx, "recalibration", {})
            if recal_status == RobotStatus.Aborted:
                log.error("recalibration aborted, stopping strategy")
                return

        # ---- Selecting -> Making loop ----
        # The legacy FSM ping-pongs between two states until match_end fires.
        # We collapse it into one loop with explicit `continue` per transition.
        current_goal: Goal | None = None
        while not match_end.is_set():
            # === SELECTING ===
            if current_goal is None:
                current_goal = goals.get_goal()
                if current_goal is None:
                    log.info("no more goals — strategy complete")
                    break
                log.info(f"--- goal #{goals.current}: {current_goal.name} ---")

            # === MAKING ===
            goal_starting_time = time()

            # 1. Goto destination
            goto_status = _run_goto(ctx, current_goal, goals.current_strategy.use_pathfinding)

            if goto_status == RobotStatus.Aborted:
                log.info("goto aborted, re-selecting")
                current_goal = None
                continue

            if goto_status == RobotStatus.Timeout:
                if current_goal.secondary_goal is not None:
                    log.warning(
                        f"goto timeout, switching to secondary goal '{current_goal.secondary_goal}'"
                    )
                    current_goal = goals.get_secondary_goal(current_goal.secondary_goal)
                    continue
                log.error("goto timeout with no secondary, finishing this goal")

            if goto_status not in (RobotStatus.Done, RobotStatus.Reached):
                log.error(f"goto returned {goto_status}, abandoning goal")
                goals.finish_goal()
                current_goal = None
                continue

            # 2. Orient (theta) if specified
            if current_goal.theta is not None:
                goth_status = _run_goth(ctx, current_goal.theta)
                if goth_status == RobotStatus.Aborted:
                    current_goal = None
                    continue
                if goth_status not in (RobotStatus.Done, RobotStatus.Reached):
                    log.error(f"goth returned {goth_status}, abandoning goal")
                    goals.finish_goal()
                    current_goal = None
                    continue

            # 3. Goal actions in order
            aborted = False
            for action in current_goal.actions:
                if match_end.is_set():
                    log.warning("match ended mid-action sequence")
                    aborted = True
                    break

                act_status = _invoke_legacy_action(ctx, action.fct, action.args, timeout=action.timeout)

                if act_status == RobotStatus.Aborted:
                    aborted = True
                    break

                if act_status == RobotStatus.Timeout and action.avoid_strategy == AvoidStrategy.Timeout:
                    log.info(f"action {action.fct} timed out (Timeout strategy) — continuing")
                    continue

                if act_status == RobotStatus.NotReached and action.avoid_strategy == AvoidStrategy.Skip:
                    log.info(f"action {action.fct} not reached (Skip strategy) — continuing")
                    continue

                if act_status not in (RobotStatus.Done, RobotStatus.Reached):
                    log.error(f"action {action.fct} returned {act_status}, abandoning goal")
                    aborted = True
                    break

            if aborted:
                current_goal = None
                continue

            # 4. Goal complete: advance
            goals.finish_goal()
            log.info(
                f"goal {current_goal.name} done in {time() - goal_starting_time:.2f}s"
            )
            current_goal = None

        log.info(
            f"=== match finished. score={score}, "
            f"elapsed={time() - match_starting_time:.2f}s ==="
        )
    finally:
        end_timer.cancel()
