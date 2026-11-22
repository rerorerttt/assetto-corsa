# AC DriftLab 0.2.0

CSP Lua application for experimental control of Assetto Corsa cars through the native CSP Custom AI shared-memory interface.

## Installation

Copy the entire `AC_DriftLab` folder to:

`assettocorsa/apps/lua/AC_DriftLab/`

The entry script must be named exactly `AC_DriftLab.lua`. Enable the app in Content Manager → Settings → Assetto Corsa → Apps (Lua Apps), then open it from the in-game app sidebar.

## Required CSP / track configuration

CSP Custom AI is experimental and is restricted to tracks that explicitly permit it.

1. In `assettocorsa/extension/config/new_behaviour.ini`, enable the `[CUSTOM_AI]` section with `ENABLED=1`. The overall New Behavior module must also be enabled.
2. In the selected track's `surfaces.ini`, enable extended physics and add:

```ini
[_EXTRA_PERMISSIONS]
ALLOW_CUSTOM_AI_MANIPULATION=1
```

3. Start an offline session on that track. If CSP I/O access is blocked for the app, grant the app the required I/O permission.

## Control model

The app uses CSP's native `ac.writeMemoryMappedFile()` and `ac.readMemoryMappedFile()` APIs. It creates `AcTools.CSP.NewBehaviour.CustomAI.CarControls<N>.v0`; CSP should then create the matching readback section `AcTools.CSP.NewBehaviour.CustomAI.Car<N>.v0`. The readback section is treated as confirmation that CSP accepted the controller.

Only steering, throttle and brake are varied. No gear-up or gear-down commands are sent. Autoclutch, autoblip and automatic shifting flags are enabled so the car's automatic gearbox can change gears itself. No AI fastline or spline position is read for navigation.

The first stage is deliberately an experiment, not a finished drift AI. It sends bounded, ramp-limited steering/throttle/brake patterns and records the vehicle response. The initial speed and yaw-rate safety limits are 75 km/h and 3.0 rad/s.

The app can start manually, or automatically when the selected car becomes AI-controlled (for example, after activating AC's AI Autopilot). The manual start path also allows testing Custom AI directly on car index 0.

## Logs

Logs are written under the CSP ScriptConfig folder in `AC_DriftLab/<experiment-id>/`. Telemetry is split into sequential CSV chunks to avoid repeatedly rewriting a growing file. Event logs are written in their own CSV chunks. The telemetry includes absolute timestamp, experiment-relative time, commanded and actual controls, speed, gear, RPM, world/local velocity, angular velocity, acceleration, heading vector and estimated slip angle.

## Validation

The app structure, native memory-mapped API and Custom AI data offsets are based on the CSP Lua SDK and Custom AI documentation. The readback offsets used for speed, gear, RPM, steering, velocity and local angular velocity follow the documented `cai_car_data` layout. This repository build cannot be runtime-tested against the user's local CSP 0.3.2-preview2 installation; the first in-game run must confirm that CSP creates the readback map and that the selected track grants Custom AI permission.
