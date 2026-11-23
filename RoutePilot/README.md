# RoutePilot 0.3 — Racing AI

CSP Lua app for Assetto Corsa. Records a player-driven route and controls AI steering, throttle and brake to follow it.

## Current source

- Records, saves and loads a route for the current track/layout.
- Places active AI cars at intervals along a loop route on startup.
- Follows the recorded route with pure-pursuit steering and a distance-based corner/braking planner.
- Default cruise range is 180–260 km/h; corner speeds are limited by measured route curvature and braking distance.
- Lane-changing overtakes are disabled while the basic route follower is revalidated.
- Logs spawn heading alignment, route error, steering, speed, throttle, brake and vehicle dynamics.

## Validation status

The latest uploaded trace showed that the spawn vector pointed opposite to the route: the actual `car.look` faced against the recorded line, causing full-lock steering and wrong-way acceleration from the first second. The current source now passes the forward route tangent, improves steering response and extends corner preview to the braking horizon. This is a source-level correction and still requires an in-game run with the exact current build; it is not yet verified as stable.

Read `DEVELOPMENT.md` for the trace evidence, SDK findings, implementation details and exact validation criteria.
