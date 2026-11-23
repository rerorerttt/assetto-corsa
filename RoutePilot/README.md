# RoutePilot 0.3 — Racing AI

CSP Lua app for Assetto Corsa. Records a player-driven route and controls AI steering, throttle and brake to follow it.

## Current source

- Records, saves and loads a route for the current track/layout.
- Places active AI cars at intervals along a loop route on startup.
- Checks the spawned car's actual heading against the recorded route before allowing throttle.
- Follows the route with pure-pursuit steering using the corrected X/Z angle convention and advances the lookahead target across exhausted route segments.
- Uses a braking-distance speed planner to calculate corner speeds from upcoming route curvature.
- Lane-changing overtakes and track-obstacle sensing remain disabled while route following is revalidated.
- Logs spawn heading alignment, route error, steering, speed, throttle, brake and vehicle dynamics.

## Latest diagnosis and validation

The latest in-game trace confirmed that spawn heading is now correct, but exposed two independent route-following defects: the signed-angle cross product commanded steering in the opposite direction, and the lookahead routine could stop at a just-passed segment endpoint, reducing its target distance to nearly zero. Both source defects were corrected in commit cfaaa28608be53d43866fe811ba7665486731b32.

This is not yet runtime-verified. The same trace showed that the heading guard can leave cars permanently stopped because it brakes and zeros steering without a reorientation/recovery state. See DEVELOPMENT.md for telemetry evidence, remaining issues and exact test criteria.
