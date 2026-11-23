# RoutePilot 0.3 — Racing AI

CSP Lua app for Assetto Corsa. Records a player-driven route and controls AI steering, throttle and brake to follow it and perform coordinated left-side overtakes.

## Current features

- Record, save and load a route for the current track/layout.
- Place active AI cars at intervals along a loop route on startup.
- Follow the recorded route with pure-pursuit steering and a longer-range curvature/braking speed planner.
- Racing defaults: 150–230 km/h cruise range, adjustable in the app up to 300 km/h.
- Lane-changing overtakes are temporarily disabled while spawn heading and route tracking are revalidated.
- Inline route radar and telemetry/event logging, including pass start, abort, completion and release.

## Important limitations

The latest uploaded trace exposed a reversed spawn heading and unstable route tracking. The spawn direction has been corrected in source and overtaking is temporarily disabled to isolate the route follower. This corrective build still requires an in-game test; do not treat it as validated.

Read `DEVELOPMENT.md` for the implementation details, SDK findings, known risks and exact next validation steps. Do not treat historical descriptions later in that file as current features unless they are confirmed by the current source.
