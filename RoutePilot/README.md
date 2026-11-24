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

## Traffic behavior (alternating left-lane / right-lane passing)

- All AI cars are assigned to the left lane initially.
- AI order alternates deterministically: car index 1 = 40 km/h, index 2 = fast class (uses the existing app speed limit and curve planner), index 3 = 40 km/h, and so on.
- Only fast-class cars initiate overtakes. They detect a slower car in the left lane, check that the right lane has enough recorded width and is clear ahead/behind, move to the right lane, and remain there until their rear has cleared the target by approximately two car lengths.
- After that clearance, the car checks that the left lane is clear and returns to it. If it is not clear, the car stays in the right lane and retries; it does not cut back across another car.
- Passing requires a corridor wide enough for two lanes (at least 2.7 m on each side of the corridor midpoint). On narrower routes, cars remain single-file.
- Lane centers are derived from the saved route bounds. With a symmetric 7 m corridor, the nominal centers are +1.75 m (left) and -1.75 m (right) from the recorded route.

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

The separate branch `routepilot/predictive-boundary-guard-2026-11-24` contains an experimental correction for AI spawn direction and predictive road-boundary avoidance. It uses the recorded route's forward direction when spawning, heading-aware route selection for the spawn reference, a first-acquisition distance/direction gate, and a lookahead corridor check that accounts for the car's bounding-box footprint. New trace fields are `BOUNDARY`, `SAFE`, and `BODY`.

This build is not yet validated in Assetto Corsa/CSP. Keep using the confirmed baseline unless deliberately testing this experimental branch.
