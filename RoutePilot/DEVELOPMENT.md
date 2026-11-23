# RoutePilot — persistent development handoff

Last reviewed: 2026-11-23
Active branch: `routepilot-traffic-coordinator`
Reviewed implementation before this documentation sync: `593db56fba3565b8e286fc95254ecd18f1b267e6`
Main implementation: `RoutePilot/RoutePilot.lua`
Latest supplied telemetry: `RoutePilot/routepilot_trace.txt`
CSP SDK snapshot: `lua-sdk/`, especially `lua-sdk/ac_apps/README.md`.

## Product goal

RoutePilot is an arcade-style race controller for player-recorded routes, including oval and crossover layouts. It should produce high-speed, decisive, coordinated traffic rather than cautious road-following. Cars should know their paired pass/yield or crossing right-of-way decision before entering a conflict. Preserve the existing CSP app manifest, UI, route recording/loading, recovery and logger. Gearbox/transmission remain under vehicle control.

## Findings from the latest supplied trace

The latest trace is about 55 seconds and contains 275 AI samples (55 per car, five AI cars). The five-car mean of per-car average speeds is about 15.59 m/s (56 km/h); the maximum sample is 33.01 m/s (119 km/h). Mean target speed is about 28.65 m/s (103 km/h), so actual speed is far below the controller's requested target. There are 26 samples with brake > 0.05. Role samples: none=150, pass=50, yield=50, yield_player=9, meet_right=16. These role counts prove pair assignments occur, not that an overtake is completed. The trace contains emergency brake commands up to 0.75 while cars have pass/yield roles; the old collision predictor was overriding the coordinator.

The supplied `line_nevadadrift.txt` has 277 plain XYZ rows and no obstacle metadata. This confirms that no collision map had previously been generated or stored with that route.

## Current implementation changes

The current source contains the following first major arcade/traffic revision:

- Default speed setting is 240 km/h; UI range is 100–320 km/h. AI pace multipliers vary deterministically by car index (approximately 0.925–1.21), creating a meaningful speed spread and a reason to pass.
- Curvature planning uses a 24 m/s² lateral-acceleration assumption and a 26 m/s minimum corner target before pace scaling. This is intentionally arcade-oriented and must be validated against the actual recorded route.
- Drift throttle no longer collapses to 5% at high slip; at maximum slip severity it retains 72% of the calculated throttle.
- Same-direction passes can be initiated by a faster assigned pace even before a large measured closing speed develops. The pass/yield offset is 3.25 m, lateral offset response is faster, and third-car lane rejection is limited to a 9 m longitudinal window.
- Mutual reserved pairs are exempt from normal following and predictive collision braking while separated by more than 2.5 m. The emergency/predicted collision lateral envelope is tighter than the previous 2.3 m test, to reduce false braking at nearby route crossings.
- Perpendicular/crossing AI conflicts now receive deterministic `cross_go` / `cross_yield` roles. The higher-pace car (car index breaks ties) owns the crossing; the yielding car moves right and receives a reduced target speed. The reservation is retained while the pair is close.
- Coordinator transitions are logged as `TRAFFIC_PLAN`; releases of pass roles are logged as `PASS_RELEASE`. These records make assignment changes visible in the next trace.

These are code changes, not proof of successful racing. A completed pass must be established from position/progress over time, not by counting `PASS_ROLE=pass` samples.

## Static collision-map scanner

The scanner is now implemented in `RoutePilot.lua` using the SDK-documented `physics.raycastTrack(pos, dir, length, point, normal, includeNoise, backfaceCulling)` API.

- On route load, a missing or checksum-mismatched cache starts an incremental scan. It samples each route point in eight horizontal directions at three heights (0.55, 1.10 and 1.65 m), with 9 m rays and a 48-ray-per-frame budget.
- The nearest hit per route-point/direction is stored with its hit point, normal, height and distance.
- The versioned cache is appended to the route file as comment rows: `#RP_OBSTACLE_V1 version checksum count` and `#RP_OBS ...`. The existing XYZ parser ignores these records, so older route files remain readable.
- Cache reuse requires matching version and route-coordinate checksum. The UI exposes a rescan action and displays whether a map is ready or scanning.
- Cached static hit points are consulted when checking an AI passing corridor.

Important: the scanner runs inside Assetto Corsa. It cannot raycast the game world from GitHub or this development environment. The repository copy of `RoutePilot/line_nevadadrift.txt` therefore remains XYZ-only until the app runs the scan locally and the resulting runtime route file is uploaded/committed. Do not add a fake zero-obstacle header to the repository copy. After the user runs the app, inspect the log for `OBSTACLE_SCAN_FINISH` and `OBSTACLE_SCAN_SAVED`, then retrieve the generated local `line_nevadadrift.txt` to include the actual cache.

SDK limitation: `physics.raycastTrack()` tests physics surfaces. It does not guarantee detection of visible render-only KN5 meshes or objects without collision geometry. Do not claim every visible wall or pole is detected.

## Remaining work

1. Validate Lua syntax and CSP runtime behaviour. No Lua interpreter or Assetto Corsa runtime is available in this development environment.
2. Test the new high-speed profile with one AI car on the route. Check that it accelerates beyond the old 10–24 m/s range and does not oscillate or leave the line in tight corners.
3. Test a pack with at least 5 AI cars. Verify actual completed passes, not just role assignment; record pass start, target, pass side, clear/release, and route progress.
4. Test same-direction passes with a third car in the passing lane, perpendicular crossing conflicts, head-on meetings, and an AI passing the player.
5. Inspect the generated obstacle cache from Nevada. Confirm ray count, cache save/load, obstacle locations and that passing corridors are rejected only when physically blocked. If the map is empty, determine whether the route has no nearby collision surfaces or the ray geometry/heights need adjustment.
6. Review false emergency braking. The tightened collision envelope is a compromise, not a substitute for testing swept future vehicle envelopes. Never disable all third-party collision checks.
7. Continue adding plan rejection reasons, maneuver IDs, predicted meeting times and pass completion telemetry.

## SDK APIs confirmed

- `physics.raycastTrack(...)`: raycasts physics surfaces and optionally writes hit point/normal; shorter rays are faster. The SDK documents it as available even when `physics.allowed()` is false.
- `physics.setAICarPosition(carIndex, pos, dir)`: AI repositioning.
- `ac.overrideCarControls(carIndex)`: control override used for steering, throttle and brake. The app deliberately does not override gears.
- `ac.getCarColliders()` returns car colliders; it is not a documented enumerator for static track KN5 meshes.
- Stock AI spline/track-coordinate functions do not replace custom-route geometry for an oval crossover.

## Development rules

Before editing, fetch the current branch head and re-read the current Lua source, manifest, logger, route file and latest trace. Work on `routepilot-traffic-coordinator` unless explicitly told otherwise. Preserve the UI, route recording/loading, recovery and logger. Verify APIs against the checked-in SDK docs. Update this handoff whenever implementation, file format, known issues or validation status changes. Never call a change in-game tested until the user has run it and supplied a fresh trace.
