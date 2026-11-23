# RoutePilot 0.3 — Racing AI

CSP Lua app for Assetto Corsa. Records a player-driven route and controls AI steering, throttle and brake to follow it.

## Current source

- Records, saves and loads a route for the current track/layout.
- Places active AI cars at intervals along a loop route on startup. The spawn direction uses the negative route tangent, as indicated by previous in-game heading telemetry.
- Checks the spawned car's heading against the recorded route before allowing throttle.
- Uses lookahead-angle steering and a braking-distance speed planner. The steering function is not yet a fully geometry-based pure-pursuit controller.
- Enables follower-only overtakes: the faster following car moves left, temporarily raises its own target speed (capped by the configured maximum), passes and returns to the recorded route.
- The slower leading car is not commanded to move aside, slow down, brake, or change its target speed for an overtake. It continues its normal route controller.
- A single global passing reservation serializes overtakes. The follower checks nearby traffic in the left corridor and aborts the manoeuvre if that corridor becomes occupied.
- Track-obstacle sensing and collision prediction remain disabled.
- Logs requested steering, actual steering-wheel angle, whether the CSP control override was consumed, heading alignment, lookahead distance/angle, route error, speed, throttle, brake, vehicle dynamics and pass state/offset.

## Validation status

The passing code has been enabled in source but has not been tested in the Assetto Corsa runtime. The left-corridor check is not a track-width or track-boundary guarantee. Confirm pass events and verify that the leader's speed, throttle, brake and lateral position remain unaffected by the follower's overtake. The heading guard can still stop a misaligned car; automatic recovery is not implemented. The full source/SDK audit, pass design and limitations are documented in DEVELOPMENT.md.

No LuaJIT compiler or Assetto Corsa runtime was available for validation. Treat this as an implementation ready for an in-game test, not as a verified working race controller.
