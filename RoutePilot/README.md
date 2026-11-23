# RoutePilot 0.5

CSP Lua app for Assetto Corsa. Records a custom route from the player car and controls AI cars on that route, with arcade pace, coordinated overtaking, crossing traffic coordination and recovery.

## Install

Copy the `RoutePilot` folder into:

`assettocorsa/apps/lua/RoutePilot/`

Enable RoutePilot in Content Manager's Lua Apps list. Keep `manifest.ini`, `RoutePilot.lua` and `logger.lua` together.

## First use

Record a route by driving it, stop recording, keep loop mode enabled for a circuit, and save the route for the current track/layout. RoutePilot loads the matching route on startup. The app controls steering, throttle and brake; the car retains control of its gearbox/transmission.

## Current controller

- Restored to the last version explicitly confirmed by the user as launching and working.
- The original inline radar drawing and global `windowMain(dt)` callback are restored.
- Overtake initiation is restored to the last user-confirmed runtime baseline; no experimental pace-trigger change is active.
- The previously committed arcade pace/overtake tuning is rolled back for now; it must be reapplied later as isolated, testable changes.
- Static obstacle detection and live track raycast avoidance remain disabled. AI-to-AI traffic coordination and recovery logic are from the restored baseline.
- Stuck, airborne and off-route recovery; periodic telemetry in `routepilot_trace.txt`.

## Collision map file behaviour

Old `#RP_OBSTACLE_V1` and `#RP_OBS` rows in route files are ignored. The scan button and map-hit overlays are removed. Track obstacle sensing will remain disabled until a validated route-local detector is implemented. Vehicle-to-vehicle traffic coordination is still active.

## Validation status

The branch has been rolled back to the last user-confirmed working runtime source and manifest. The current files match the confirmed baseline source and manifest blobs. This restores the previous UI code path but has not yet been re-tested in the user's game; do not claim runtime success until the user confirms it.

Read `DEVELOPMENT.md` before changing the coordinator or obstacle cache. It contains the current implementation, SDK references, telemetry findings and required test cases.


## Startup compatibility

The main-window callback is the user-confirmed global `windowMain(dt)` declaration. The single manifest window uses the previously confirmed configuration without an added `FLAGS = MAIN` entry. The route radar is drawn inline using the original `ui.invisibleButton("line_radar", ...)` canvas. Do not replace this with a child window or change callback registration without first validating the exact CSP build in-game.
