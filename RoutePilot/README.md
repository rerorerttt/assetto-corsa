# RoutePilot 0.3 — Racing AI

CSP Lua app for Assetto Corsa. Records a player-driven route and controls AI steering, throttle and brake to follow it and perform coordinated left-side overtakes.

## Current features

- Record, save and load a route for the current track/layout.
- Place active AI cars at intervals along a loop route on startup.
- Follow the recorded route with pure-pursuit steering and a longer-range curvature/braking speed planner.
- Racing defaults: 150–230 km/h cruise range, adjustable in the app up to 300 km/h.
- Left-only overtaking: check the left traffic corridor, reserve the passing lane, move 3.8 m left, temporarily raise target speed, pass the leader without asking it to slow down, then return to the route.
- One active pass reservation at a time to avoid simultaneous conflicting lane changes.
- Inline route radar and telemetry/event logging, including pass start, abort, completion and release.

## Important limitations

The new racing controller is an initial implementation and has not yet been validated in Assetto Corsa. The left-lane check detects nearby cars, including oncoming traffic, but is not yet a full track-boundary or collision prediction system. Track-edge protection currently reacts to the car's `wheelsOutside` state. Speed, lateral-acceleration and pass-offset settings require telemetry-based tuning on the target track.

Read `DEVELOPMENT.md` for the implementation details, SDK findings, known risks and exact next validation steps. Do not treat historical descriptions later in that file as current features unless they are confirmed by the current source.
