# evo_robot_configs

JSON5 configuration for Evolutek competition robots (Coupe de France de Robotique).

## Ecosystem

| Repo | Role |
|------|------|
| **evo_robot_configs** (this repo) | JSON5 configuration per robot and year |
| **evo_hl_library** | Shared library — drivers, config loader, logging, LocalBus |
| **evo_hl_omnissiah** | Robot brain: Orchestrator, IA, Trajman, Action, Client |
| **evo_tools** | Standalone tools — CLI, config verifier, debug scripts |

## Why JSON5

Config files use [JSON5](https://json5.org/) instead of plain JSON. Hex numbers make I2C addresses readable (`0x40` instead of `64`), inline comments document the config in place, and trailing commas allow safe reordering during calibration.

## Layered configuration

Configuration is split into layers, each with a different rate of change. A chassis definition that never changes doesn't belong in the same file as servo angles tweaked every session.

| Layer | File | Changes when | Consumed by |
|-------|------|-------------|-------------|
| Platform | `platforms/<robot>.json5` | Hardware redesign (rare) | Orchestrator (bus/driver init) |
| Table | `<year>/table.json5` | New competition year | WorldModel (initial state) |
| Assemblies | `<year>/robots/<robot>/assemblies.json5` | At assembly time | Action (hardware mapping) |
| Dimensions | `<year>/robots/<robot>/dimensions.json5` | At assembly time | Trajman (collision footprint) |
| Actions | `<year>/robots/<robot>/actions.json5` | During development | Action (positions + sequences) |
| Strategy | `<year>/robots/<robot>/strategy.json5` | During development | IA (goals + starting positions) |

```
evo_robot_configs/
├── platforms/
│   └── <robot>.json5           # Fixed hardware: boards, buses, sensors
└── <year>/
    ├── table.json5              # Table layout, zones, scoring rules
    └── robots/
        └── <robot>/
            ├── assemblies.json5 # Actuator/sensor → hardware ID mapping
            ├── dimensions.json5 # Footprint profiles per level × state
            ├── actions.json5    # Named positions + action sequences
            └── strategy.json5   # Goals, starting positions
```

## Static vs Dynamic

These config files are **static** — loaded at boot, never modified at runtime.

The dynamic state of the game (which crates have been picked up, adversary position, etc.) lives in the **WorldModel** inside Omnissiah, initialized from `table.json5` and updated by sensors, actions, and the balise maître via MQTT.

## Config loading

Configs are loaded and merged in order by **evo_hl_library** (`ConfigLoader`). This repo only contains the data.
