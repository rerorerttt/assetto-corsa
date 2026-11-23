# RoutePilot 0.3 — Racing AI

CSP Lua app for Assetto Corsa. Records a player-driven route and controls AI steering, throttle and brake to follow it.

## Current source

- Records, saves and loads a route for the current track/layout.
- Places active AI cars at intervals along a loop route on startup.
- Checks the spawned car's heading against the recorded route before allowing throttle.
- Uses lookahead-angle steering and a braking-distance speed planner. The steering function is not yet a fully geometry-based pure-pursuit controller.
- Logs requested steering, actual steering-wheel angle, whether the CSP control override was consumed, heading alignment, lookahead distance/angle, route error, speed, throttle, brake and vehicle dynamics.
- Lane-changing overtakes and track-obstacle sensing remain disabled while route following is revalidated.

## Latest diagnosis

The 2026-11-23 trace showed the cars steering away from the recorded line. The prior cross-product sign change was incorrect for the observed CSP steering response and has been reverted. The heading guard also remains capable of permanently stopping a misaligned car; no automatic recovery exists yet. The full SDK/source audit and remaining issues are documented in DEVELOPMENT.md.

The steering-sign correction and telemetry instrumentation are source-level changes only. No LuaJIT compiler or Assetto Corsa runtime was available for validation; test the current branch in-game before treating the controller as fixed.
