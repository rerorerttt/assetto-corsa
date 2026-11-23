# RoutePilot 0.3

CSP Lua app for Assetto Corsa. Records a player-driven route and makes AI cars follow it using custom steering, throttle and brake control.

## Install

Copy the `RoutePilot` folder into:

`assettocorsa/apps/lua/RoutePilot/`

Enable RoutePilot in Content Manager's Lua Apps list. Keep `manifest.ini`, `RoutePilot.lua` and `logger.lua` together.

## Current capabilities

- Record, save and load a route for the current track/layout.
- Place active AI cars at intervals along a loop route on startup.
- Follow the recorded route with a pure-pursuit steering controller and a basic curvature-based speed planner.
- Draw the route radar inline in the main app window.
- Log periodic AI telemetry to `routepilot_trace.txt`.
- Override steering, throttle and brake only; gearbox and transmission remain under vehicle control.

## Current limitations

This is a stable route-following baseline, not yet a complete racing AI. It does not currently implement overtaking, coordinated yielding, head-on/crossing negotiation, obstacle detection or stuck/off-route recovery. The current following logic slows behind a car in front rather than planning and executing a pass. High-speed racing behaviour is under development and has not been validated in-game.

The startup spawn direction was corrected in commit `7461f57d58bc65b4fe535a38ad437d2ff59fb529` after the committed telemetry showed the AI forward vector opposite to the route tangent. Verify the first post-spawn telemetry before treating the correction as confirmed.

Read `DEVELOPMENT.md` before modifying the controller. It records the current source state, SDK findings, telemetry analysis, validation status and next implementation steps.
