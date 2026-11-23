# RoutePilot 0.5

CSP Lua app for Assetto Corsa. Records a custom route from the player car and controls AI cars on that route, with arcade pace, coordinated overtaking, crossing traffic coordination and recovery.

## Install

Copy the `RoutePilot` folder into:

`assettocorsa/apps/lua/RoutePilot/`

Enable RoutePilot in Content Manager's Lua Apps list. Keep `manifest.ini`, `RoutePilot.lua` and `logger.lua` together.

## First use

Record a route by driving it, stop recording, keep loop mode enabled for a circuit, and save the route for the current track/layout. RoutePilot loads the matching route on startup. The app controls steering, throttle and brake; the car retains control of its gearbox/transmission.

## Current controller

- Arcade speed profile: 240 km/h internal cruise target, with deterministic per-car pace variation.
- More permissive corner-speed planning, stronger throttle response and faster steering response for a less cautious arcade feel.
- Earlier same-direction pass requests: a small real closing speed or a 2.5% pace advantage can trigger an overtake while there is still setup distance.
- Centralized pass/yield reservations, head-on right-side meetings, and deterministic right-of-way for crossing AI paths.
- Reserved AI pairs are exempted from normal mutual following and most predictive braking while carrying out their agreed manoeuvre.
- Static obstacle detection and live track raycast avoidance are currently disabled because the generated map and side-ray readings were not reliable enough for control. AI-to-AI traffic coordination remains active.
- Stuck, airborne and off-route recovery; periodic telemetry in `routepilot_trace.txt`.

## Collision map file behaviour

Old `#RP_OBSTACLE_V1` and `#RP_OBS` rows in route files are ignored. The scan button and map-hit overlays are removed. Track obstacle sensing will remain disabled until a validated route-local detector is implemented. Vehicle-to-vehicle traffic coordination is still active.

## Validation status

The user confirmed the rollback revision launches. The arcade pace/overtake and embedded radar changes are committed but have not yet been tested in-game. A fresh user-run test and trace are required before claiming that the cars now race at the intended pace or complete passes.

Read `DEVELOPMENT.md` before changing the coordinator or obstacle cache. It contains the current implementation, SDK references, telemetry findings and required test cases.


## Startup compatibility

The main-window callback remains the user-confirmed working global `windowMain(dt)` declaration. The manifest explicitly marks its single window as `MAIN`; the radar is rendered in an ImGui `ui.childWindow()` region inside that same window, not in a second top-level window. Do not change the callback registration form without a verified in-game reproduction on the user's CSP build.
