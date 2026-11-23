# RoutePilot 0.4

CSP Lua app for Assetto Corsa. Records a custom route from the player car and uses it to control AI cars on that route, with centralized traffic coordination and recovery.

## Install

Copy the `RoutePilot` folder into:

`assettocorsa/apps/lua/RoutePilot/`

Enable RoutePilot in Content Manager's Lua Apps list and open it in a session. Keep `manifest.ini`, `RoutePilot.lua`, and `logger.lua` together.

## First use

1. Enable route recording and drive the desired route.
2. Stop recording after completing the desired lap/route.
3. Keep loop mode enabled for a circular route.
4. Save the route for the current track/layout.
5. Enable AI cars in the session and enable RoutePilot.

Routes are stored separately by track/layout. The app attempts to load the saved route on startup. The route file currently uses plain XYZ rows.

## Current controller

- Uses a player-recorded world-space route rather than relying on the track's stock AI fast lane.
- Uses Pure Pursuit-style steering and a curvature-based speed planner.
- Controls steering, throttle and brake through CSP; gearbox/transmission remain under vehicle control.
- Builds one active-car snapshot and coordinates same-direction pass/yield pairs and opposite-direction meeting/yield pairs.
- Applies per-car pace variation to create race pressure and passing opportunities.
- Uses car-to-car following and predicted collision checks, plus stuck/airborne/off-route recovery.
- Writes periodic telemetry to `routepilot_trace.txt`.

## Current development status

The `routepilot-traffic-coordinator` branch is under active development. Commit `62fc5f5` introduced an arcade tuning pass: higher default speed, more responsive steering/throttle, revised curvature/braking assumptions, 2.65 m traffic offsets, a shorter lane-rejection window and alternating AI pace multipliers. These values have not yet been validated in-game. Collect a fresh trace after testing; the previous trace is baseline evidence from before this tuning pass.

Static track-obstacle scanning is planned but not implemented yet. The checked-in CSP SDK documents `physics.raycastTrack()` for ray tests against physics surfaces; see `DEVELOPMENT.md` for the proposed cached obstacle-map design and its limitations.

## Development handoff

Before continuing work, read `DEVELOPMENT.md`. It records the active branch, reviewed commits, known traffic bugs, verified CSP APIs, design constraints, and validation status. Update it whenever the implementation, API findings, file format, or test results change.

## Important limitations

- Traffic coordination is an evolving system and must be tested on the actual track, especially where the route crosses itself or contains opposing traffic.
- The obstacle cache is not implemented. `physics.raycastTrack()` can detect collision-enabled physics surfaces; it cannot guarantee detection of visible objects without physics collision geometry.
- No code change should be described as in-game verified until a fresh session and telemetry confirm it.
