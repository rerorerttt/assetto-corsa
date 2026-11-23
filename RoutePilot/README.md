# RoutePilot 0.3 — Racing AI

CSP Lua app for Assetto Corsa. Records a player-driven route and controls AI steering, throttle and brake to follow it.

## Current source

- Records, saves and loads a route for the current track/layout.
- Places active AI cars at intervals along a loop route on startup.
- Checks the spawned car's actual heading against the recorded route before allowing throttle.
- Follows the recorded route with pure-pursuit steering and a braking-distance speed planner that measures distance to the next route vertex.
- Default cruise range is 180–260 km/h; corner speeds are limited by measured route curvature and braking distance.
- Lane-changing overtakes and track-obstacle sensing are currently disabled while the route follower is revalidated.
- Logs spawn heading alignment, route error, steering, speed, throttle, brake and vehicle dynamics.

## Latest diagnosis and validation

The latest uploaded trace proved that the previous positive spawn direction made all five cars face against the recorded route (heading_dot from -0.902 to -1.000). This caused full-lock steering under full throttle immediately after spawn. The latest source reverses the direction vector, adds a wrong-heading brake guard, preserves segment progress for corner-speed planning and uses heading agreement when selecting a route segment at crossovers.

The same trace shows four of five cars travelling far outside the route and below the world surface before the log ends. The app did not delete them; the current source has no off-route recovery. The trace contains no coordinated pass events, and pass execution is disabled in the current source. The correction is not yet tested in-game. Use the exact current commit and inspect the new trace before enabling overtaking again.

Read DEVELOPMENT.md for detailed telemetry evidence, source changes and validation criteria.
