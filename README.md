# evo_robot_configs

YAML configuration for Evolutek competition robots (Coupe de France de Robotique).

## Ecosystem

| Repo | Role |
|------|------|
| **evo_robot_configs** (this repo) | YAML configuration per robot and year |
| **evo_hl_library** | Reusable hardware drivers and utilities |
| **evo_hl_omnissiah** | Robot brain: module orchestration, strategy/AI |

## Philosophy

Configuration is organized in **layers**, each with a distinct rate of change.
Separating by rate of change means each file has a single reason to be modified, reducing merge conflicts and keeping diffs readable.

| Layer | File | Changes when | Modified by |
|-------|------|-------------|-------------|
| Platform | `platforms/<robot>.yaml` | Hardware redesign (rare) | Hardware lead |
| Table | `<year>/table.yaml` | New competition year | Rules/regulations |
| Robot | `<year>/robots/<robot>/robot.yaml` | At assembly time | Hardware + Software |
| Actions | `<year>/robots/<robot>/actions.yaml` | During development (constant) | Software |
| Strategy | `<year>/robots/<robot>/strategy.yaml` | During development | Strategy/AI |

## Directory structure

```
evo_robot_configs/
├── platforms/
│   └── <robot>.yaml                # Chassis definition (stable)
└── <year>/
    ├── table.yaml                  # Playing field (shared between robots)
    └── robots/
        └── <robot>/
            ├── robot.yaml          # Hardware mounted this year
            ├── actions.yaml        # Named positions, angles
            └── strategy.yaml       # Goals, starting positions, scoring
```

## File formats

### Platform — `platforms/<robot>.yaml`

Defines the physical chassis: what remains if you remove everything mounted on it each year.

```yaml
name: hololutek
type: holonomic

dimensions:
  radius: 150       # mm
  height: 350       # mm

faces: 3

wheels:
  count: 3
  type: omni

# Module types this platform supports (structural, not concrete)
modules:
  - propulsion
  - localization
  - actuators
```

**Rule of thumb**: if the value stays the same when switching to a new competition year on the same chassis, it belongs here. The platform declares *what types of modules* the robot has, not their concrete configuration.

### Table — `<year>/table.yaml`

Playing field dimensions and zones for a given year, shared across all robots.

```yaml
dimensions:
  width: 3000       # mm
  height: 2000      # mm

zones:
  - name: start_blue
    x: 0
    y: 0
    width: 400
    height: 400
```

### Robot — `<year>/robots/<robot>/robot.yaml`

Hardware mounted on the chassis this year: module instances, communication buses, and mechanical sub-assemblies.

#### Modules

Each module type declared in the platform is configured here with concrete values.

```yaml
modules:
  propulsion:
    can_board_type: 0x04
    can_board_id: 1

  localization:
    lidar:
      device: /dev/ttyUSB1
      model: rplidar_a2
    recal_sensors:
      adc_address: 0x48
      channels: [0, 1, 2, 3]

  actuators:
    boards:
      - address_pca: 0x40
        address_mcp: 0x20
        address_tca: 0x70
        frequency: 50
```

The number of actuator boards can vary from year to year. Adding a second board is just another entry in the list.

#### Buses

Communication interfaces used by drivers.

```yaml
buses:
  ax12:
    device: /dev/ttyUSB0
    baudrate: 1000000
  i2c:
    bus: 1
```

#### Assemblies

Mechanical sub-assemblies are **named groups of components** with a `type` field that determines which driver/assembly class to instantiate. This abstraction works for any robot architecture: holonomic faces, differential arms, or any other grouping.

```yaml
assemblies:
  face_0:
    type: gripper
    fingers: 4
    ax12_offset: 0
    pca_offset: 0
    color_offset: 0
  face_1:
    type: gripper
    fingers: 4
    ax12_offset: 8
    pca_offset: 4
    color_offset: 4
  face_2:
    type: gripper
    fingers: 4
    ax12_offset: 16
    pca_offset: 8
    color_offset: 8
```

The `type` field is the key: it tells the code which class to instantiate. Each type defines the parameters it expects.

**Holonomic robot (identical faces)**: each face is a `gripper` assembly with auto-mapping offsets.

**Holonomic robot (mixed faces)**: face types can differ.

```yaml
assemblies:
  face_0:
    type: gripper
    fingers: 4
    ax12_offset: 0
    pca_offset: 0
    color_offset: 0
  face_1:
    type: gripper
    fingers: 4
    ax12_offset: 8
    pca_offset: 4
    color_offset: 4
  face_2:
    type: suction_arm
    ax12_ids: [17, 18]
    pump_gpio: 8
```

**Differential robot**: no faces, just functional groups.

```yaml
assemblies:
  left_arm:
    type: arm
    ax12_ids: [1, 2]
    servo_channels: [0, 1]
  right_arm:
    type: arm
    ax12_ids: [3, 4]
    servo_channels: [2, 3]
  elevator:
    type: elevator
    stepper_id: 0
```

#### Auto-mapping

Assembly types that contain repeated identical sub-units (like a gripper with N fingers) can use **offset-based auto-mapping** to compute component IDs from the structure.

For a `gripper` assembly with finger index `D`:

| Component | Formula |
|-----------|---------|
| `ax12_base` | `ax12_offset + (D * 2) + 1` |
| `ax12_tip` | `ax12_offset + (D * 2) + 2` |
| `servo` | `pca_offset + D` |
| `color` | `color_offset + D` |

**Example** — `face_1`, finger 2:

| Component | Computation | Result |
|-----------|-------------|--------|
| `ax12_base` | `8 + (2 * 2) + 1` | ID 13 |
| `ax12_tip` | `8 + (2 * 2) + 2` | ID 14 |
| `servo` | `4 + 2` | PCA channel 6 |
| `color` | `4 + 2` | TCA channel 6 |

Auto-mapping is an internal detail of the assembly type, not a global mechanism. Assembly types that don't have repeated sub-units simply list their IDs explicitly.

### Actions — `<year>/robots/<robot>/actions.yaml`

Named positions for each assembly type.
All instances of a given type share the same positions by default, with optional per-instance overrides.

```yaml
finger:
  positions:
    open:
      ax12_base: 512
      ax12_tip: 200
      servo: 90
    closed:
      ax12_base: 300
      ax12_tip: 600
      servo: 10
    detect:
      ax12_base: 400
      ax12_tip: 400
      servo: 45

  overrides:
    face_0:
      finger_2:
        closed:
          ax12_base: 310
```

#### Override resolution

Positions are resolved by merging the default with any applicable override.
Only the specified keys are overridden, everything else keeps the default value.

```
default:   {ax12_base: 300, ax12_tip: 600, servo: 10}
override:  {ax12_base: 310}
─────────────────────────────────────────────────────
result:    {ax12_base: 310, ax12_tip: 600, servo: 10}
```

### Strategy — `<year>/robots/<robot>/strategy.yaml`

Game strategy: starting positions, goals, actions, and scoring.

```yaml
starting_positions:
  - name: default
    x: 200
    y: 1000
    theta: 0

goals:
  - name: grab_cakes
    position:
      x: 800
      y: 600
    theta: 1.5708  # pi/2
    score: 15
    actions:
      - grab_soft
      - wait: 100
      - closed
```

## Config loading

Configs are loaded and merged in order: platform, table, robot, actions, strategy.
The loading code lives in **evo_hl_omnissiah**, not in this repository.
This repo only contains the data.
