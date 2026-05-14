# evo_robot_configs

JSON5 configuration for Evolutek competition robots (Coupe de France de Robotique).

## Ecosystem

| Repo | Role |
|------|------|
| **evo_robot_configs** (this repo) | JSON5 configuration per robot |
| **evo_hl_library** | Shared library: drivers, config loader, logging, graph engine |
| **evo_hl_omnissiah** | Robot brain: peripheral manager, AI, Trajman |
| **evo_tools** | Standalone tools: CLI, config verifier, debug scripts |

## Why JSON5

Config files use [JSON5](https://json5.org/) instead of plain JSON. Hex numbers make I2C addresses readable (`0x40` instead of `64`), inline comments document the config in place, and trailing commas allow safe reordering during calibration.

## Layout

One folder per robot at the root of the repo. The folder name is what omnissiah expects via `--robot <name>`.

```
evo_robot_configs/
└── <robot>/
    ├── peripherals.json5   # Hardware tree: drivers, buses, sensors, actuators
    ├── actions.json5       # Named hardware actions (move_arm, move_tip, ...)
    ├── strategies.json5    # Match strategies selectable from the GUI at setup
    └── graphes.json5       # AI graphs (will be split into graphs/ folder later)
```

### What each file is for

| File | Consumed by | Contents |
|------|-------------|----------|
| `peripherals.json5` | `peripheral_manager` | Flat declaration of every driver instance. Each entry has a `driver` name and an `args` object. Entries can reference each other (e.g. a `pwm_servo` references a PCA channel via `constellation_face1.pwm.ch14`). |
| `actions.json5` | `ai_manager` (action engine) | Two sections: `values` is a named position bank (lookup tables for arm angles, servo presets, etc) and `actions` is a registry of parametrized hardware actions. Each action declares typed inputs and a list of `commands` targeting peripherals. Templated names like `ax12_{arm}` allow one action definition to drive any of the 12 arms. |
| `strategies.json5` | `ai_manager` | List of match strategies. Each strategy has a `runner` (today only `graph`), a target `graph` name and a `title` shown in the GUI. The operator picks one of these from the setup screen before launching the match. |
| `graphes.json5` | `graph_ai` | AI graphs. Each top-level key is a graph name; nodes inside reference action types (`action:move_arm`), built-in flow nodes (`entry`, `if_else`, `wait`) or peripheral commands. A graph that is referenced by a strategy becomes a launchable match plan. |

## Static vs dynamic

These files are static, loaded at boot, never modified at runtime. Runtime game state (gripper inventory, opponent position, scored points) lives in code, inside omnissiah.

## Config loading

Omnissiah builds two `RobotConfigOverlay` layers via `RobotConfigManager`:

1. `common` overlay rooted at the repo root (used for files shared across robots, none today).
2. `<robot_name>` overlay rooted at `<repo>/<robot>/`.

The loader resolves a config by name across both overlays in order, robot-specific first. There is no path-based merging or year-based nesting. If a file is not found in any overlay, the brain crashes at boot.
