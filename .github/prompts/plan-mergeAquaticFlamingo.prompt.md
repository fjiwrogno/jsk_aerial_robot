# Merge Plan: `dev/aquatic_flamingo` → `dev/add_depth_control`

## What the branch contains

Two commits:
- **mocap driver swap** — switch `mocap_optitrack` pkg to `mocap_nokov`, comment out optitrack config block in `mocap.launch`
- **speed mode functionality** — add `SpeedMode` enum (LOW/MEDIUM/HIGH) with per-mode pitch limits and joystick toggle

## Files changed

| File | Change |
|---|---|
| `aerial_robot_base/launch/external_module/mocap.launch` | swap pkg name + comment out optitrack config |
| `robots/flamingo/include/flamingo/control/flamingo_controller.h` | add `SpeedMode` enum, `PITCH_LIMIT_*` constants |
| `robots/flamingo/src/control/flamingo_controller.cpp` | tilt compensation in `depthControlLoop()` + speed-mode toggle & pitch refactor in `joyCallback()` |

## Steps

### 1. Ensure clean working tree (on `dev/add_depth_control`)
```bash
git status
# stage or stash any uncommitted work
```

### 2. Perform the merge
```bash
git merge cc_repo/dev/aquatic_flamingo
```

### 3. If conflict in `flamingo_controller.cpp`

**In `depthControlLoop()`** — keep aquatic_flamingo's tilt compensation (pure additive on top of `dev/add_depth_control`):
```cpp
double tilt_factor = 1.0 / std::max(0.7, cos(current_pitch) * cos(current_roll));
total_thrust = base_hover_thrust * tilt_factor;
```

**In `joyCallback()`** — keep the speed-mode block from aquatic_flamingo; verify the pitch/roll stick refactor does not drop any depth-control-specific button handling from `dev/add_depth_control`.

**In `mocap.launch`** — accept the aquatic_flamingo version entirely (simple driver swap, no conflict expected).

### 4. If conflict in `flamingo_controller.h`
Add the `SpeedMode` enum and three `PITCH_LIMIT_*` constants alongside whatever new members `dev/add_depth_control` already has.

### 5. Finalise merge
```bash
git add aerial_robot_base/launch/external_module/mocap.launch
git add robots/flamingo/include/flamingo/control/flamingo_controller.h
git add robots/flamingo/src/control/flamingo_controller.cpp
git commit
```

### 6. Build & verify
```bash
catkin build flamingo
```

## Verification checklist

- [ ] `catkin build flamingo` — no compile errors
- [ ] `depthControlLoop()` applies `tilt_factor` to hover thrust correctly
- [ ] Joystick speed-mode toggle cycles LOW → MEDIUM → HIGH → LOW
- [ ] Depth-control-specific joy buttons from `dev/add_depth_control` are still intact
- [ ] `mocap_node` launches with `mocap_nokov` pkg

## Out of scope for this merge

The devNeed.md dual-motor goal (hardcode aquatic `motor_info_` in `attitude_control.cpp`, zero allocation matrix → set `min_duty_`) was **never in `dev/aquatic_flamingo`**. It is a separate implementation task to be done after this merge.
