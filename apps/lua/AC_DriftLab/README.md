# AC DriftLab 0.1.1

CSP Lua app for controlled AI input experiments in Assetto Corsa.

## Correct installation layout

The app entry script name must match the folder name:

`assettocorsa/apps/lua/AC_DriftLab/AC_DriftLab.lua`

The folder also contains `manifest.ini` and `icon.png`. Do not rename the Lua entry file.

## Behavior

- The app is registered as a normal CSP Lua app through its manifest and has a taskbar icon.
- `LAZY = NONE` keeps `script.update(dt)` running even while the app window is closed.
- Car index 0 is selected by default. When the player activates AC's AI autopilot and CSP reports `isAIControlled = true`, the experiment starts automatically.
- Other AI cars can be selected in the app by their 0-based car index.
- The app controls steering, throttle and brake only. It never sends gear commands.
- No AI fastline/spline is read or required.
- Inputs are bounded and ramp-limited. At 70 km/h or yaw rate above 2.5 rad/s, a one-second recovery phase is applied.
- Stopping releases steering back to the original AI input using `steer = math.huge` and restores combined pedal input.

## Logs

Requires CSP Lua I/O access. Files are written to the app ScriptConfig folder under `AC_DriftLab`:
- `<experiment>_telemetry.csv`
- `<experiment>_events.csv`

## Validation status

The app structure and API names have been checked against the CSP Lua SDK and app manifest documentation. Actual runtime behavior still needs testing in CSP 0.3.2-preview2. In particular, verify that the built-in AI autopilot toggles `ac.getCar(0).isAIControlled` in the target session.
