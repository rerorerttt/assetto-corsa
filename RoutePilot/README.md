# RoutePilot 0.2.1

CSP Lua app for Assetto Corsa. Records a route from the player's car and uses it to control AI cars without relying on the track's stock AI fast lane.

## Install

Copy the `RoutePilot` folder into:

`assettocorsa/apps/lua/RoutePilot/`

The folder must contain:
- `manifest.ini`
- `RoutePilot.lua`

Enable RoutePilot in Content Manager's Lua Apps list and open it in a session.

## First use

1. Drive the desired route with the player car.
2. Press **Начать запись маршрута** before driving.
3. Drive one full lap and press **Остановить запись**.
4. Keep **Замкнутый маршрут (круг)** enabled for a lap route.
5. Press **Сохранить маршрут этой трассы**.
6. Add AI cars to the session and press **Запустить автопилот**.

Routes are stored separately by full track ID using CSP `ac.storage` and are autoloaded when the app starts on a track with a saved route.

## Current scope

- Records player-car world X/Z coordinates and speed.
- Stores and loads routes per track/layout, with autoload on app start.
- Uses a shared Catmull–Rom route geometry module with an arc-length lookup table, continuity-constrained route projection, signed cross-track feedback, true sampled-curve curvature, and a curvature/braking speed planner.
- Traffic gaps, lane occupancy, and pass clearance are measured along cumulative route progress, not by projecting onto the car's current heading. The controller logs cross-track error, heading error, signed local route curvature, route progress, lane target, and steering saturation.
- Controls only steering, throttle and brake. No gear, clutch, reverse or stock AI spline commands are issued.
- Applies to active AI-controlled cars with index 1 and above.

## Changelog

- 0.2.1: fixed route autoload — Lua apps have no `script.onSessionStart` callback, so autoload now happens on the first `script.update` call.
- 0.2.0: car-following collision avoidance, minimum-distance slider, slowing-car counter.

## Traffic behavior (speed-based coordinated passing)

- All AI-controlled cars, including the player car while Assetto Corsa's native Ctrl+8 AI takeover is active, use the same RoutePilot route follower. The app detects the takeover through the car's existing `isAIControlled` state; it adds no key binding or separate control.
- There is no alternating slow/fast class and no hardcoded 40 km/h cruise assignment. Every controlled car uses the configured `speedMax` as its cruise ceiling, then the curvature planner calculates its current safe target speed.
- Any car may request a pass when its current safe target speed exceeds the leader's speed by at least 1.5 m/s and the estimated pass distance is within 180 m. The passing lane must fit the complete vehicle footprint and be clear ahead, behind, and alongside.
- A local route-progress reservation allows only one active pass within a 110 m traffic cluster. Cars behind the active passer follow at a calculated safe speed and retry after the reservation is released. Separate clusters farther apart can pass independently.
- During a pass, the car also follows traffic in the passing lane. It does not return until its rear has cleared the target and the original lane is clear. If the route is too narrow for two lanes, it stays single-file and follows the car ahead using a braking-distance speed limit.
- Failed pass requests do not authorize forcing a lane change. The car stays in its lane and reduces speed according to the gap and speed of the vehicle ahead.

## Known limitations

- Passing is enabled only when the recorded corridor has at least 2.7 m on each side of its midpoint. A symmetric 7 m profile gives lane centers at +1.75 m and -1.75 m; a 3 m profile correctly disables passing.
- The arc-length route geometry and path follower are source-implemented but have not yet been validated in Assetto Corsa/CSP. Normalized steering response still needs in-game calibration from trace telemetry.
- Route-progress conflict checks currently assume traffic follows the same recorded route direction. Opposing-direction traffic on a shared oval still requires explicit direction-aware conflict handling.
- If a locked route projection is lost, the driver currently fails safe by braking rather than attempting an unverified global branch reacquisition.
- Pit handling and automatic recovery from every spin/off-track situation are outside the current traffic implementation; the existing stuck-car rescue remains in place.


### Manual driver telemetry test

When a saved or completed route with at least two points is available, RoutePilot also records the manually driven player car (index 0) to `routepilot_trace.txt` once per second. Its row uses the same fields as AI telemetry and is marked `CAR=0` and `MODE=PLAYER`. Steering is the normalized live controller input; throttle and brake are the actual car input values. The player is observed passively every update frame to keep route progress continuous. No AI controls are applied to the player car. `TARGET_SPEED=-1.00` means not applicable to manual driving.

### Predictive arc steering and traction control

The AI path follower now uses a pure-pursuit curvature command aimed at a Catmull–Rom spline sample selected by arc distance ahead, with a small sampled-curvature feed-forward term. It no longer uses the next recorded point as its steering target. Preview distance scales with speed (10–28 m), so steering is initiated before the bend.

Throttle is limited by positive wheel slip reported by CSP wheel telemetry and by lateral acceleration in corners. Wheelspin causes an immediate torque reduction with slower throttle recovery. The trace includes WHEEL_SLIP, TC_SCALE, and STEER_CURV for diagnosis. This is a feedback limiter, not a guarantee of zero slip; in-game testing across the target cars is still required.

The curvature speed planner no longer forces an 11 m/s minimum in tight bends; its calculated lateral-acceleration speed cap is allowed to slow the AI further when the route curvature requires it.


## Best-known-good baseline

The user has tested the 2026-11-24 RoutePilot build in Assetto Corsa and confirmed accurate lane keeping and clean coordinated overtaking. Preserve this revision as the known-good baseline. Future behavior changes should be developed on separate experimental branches created from the baseline, not directly on top of it.


## Experimental boundary-safety build

The separate branch `routepilot/predictive-boundary-guard-2026-11-24` contains an experimental correction for AI spawn direction and predictive road-boundary avoidance. It uses the recorded route's forward direction when spawning, heading-aware route selection for the spawn reference, a first-acquisition distance/direction gate, and a lookahead corridor check that transforms saved bounds into spline coordinates and accounts for the car's rotated bounding-box footprint. New trace fields are `BOUNDARY`, `SAFE`, and `BODY`.

This build is not yet validated in Assetto Corsa/CSP. Keep using the confirmed baseline unless deliberately testing this experimental branch.
