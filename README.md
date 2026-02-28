# evo_robot_configs

JSON5 configuration for Evolutek competition robots (Coupe de France de Robotique).

## Ecosystem

| Repo | Role |
|------|------|
| **evo_robot_configs** | JSON5 configuration per robot and year |
| **evo_utils** | Shared utilities (config loader, logging, etc.) |
| **evo_hl_library** | Reusable hardware drivers |
| **evo_hl_omnissiah** | Robot brain: module orchestration, strategy/AI |

## Why JSON5

Config files use [JSON5](https://json5.org/) instead of plain JSON. Hex numbers make I2C addresses readable (`0x40` instead of `64`), inline comments document the config in place, and trailing commas allow safe reordering during calibration.

## Layered configuration

Configuration is split into layers, each with a different rate of change. A chassis definition that never changes doesn't belong in the same file as servo angles tweaked every session.

| Layer | File | Changes when |
|-------|------|-------------|
| Platform | `platforms/<robot>.json5` | Hardware redesign (rare) |
| Table | `<year>/table.json5` | New competition year |
| Robot | `<year>/robots/<robot>/robot.json5` | At assembly time |
| Actions | `<year>/robots/<robot>/actions.json5` | During development (constant) |
| Strategy | `<year>/robots/<robot>/strategy.json5` | During development |

```
evo_robot_configs/
├── platforms/
│   └── <robot>.json5
└── <year>/
    ├── table.json5
    └── robots/
        └── <robot>/
            ├── robot.json5
            ├── actions.json5
            └── strategy.json5
```

Configs are loaded and merged in order by **evo_utils**. This repo only contains the data.
