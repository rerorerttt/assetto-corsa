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
- Applies to every active AI-controlled car, including car 0 while Assetto Corsa's native Ctrl+8 autopilot is active. Manual player control remains untouched.

## Changelog

- 0.2.1: fixed route autoload — Lua apps have no `script.onSessionStart` callback, so autoload now happens on the first `script.update` call.
- 0.2.0: car-following collision avoidance, minimum-distance slider, slowing-car counter.

## Traffic behavior (multi-offset passing)

- Every AI car uses the same configured cruise-speed ceiling and curvature/braking planner. There is no index-based 40 km/h class.
- Any car may pass a slower vehicle when its pace advantage and estimated pass distance make the manoeuvre feasible.
- The passing target is selected dynamically from the complete safe lateral corridor, not from a fixed right-lane center. Candidate offsets are checked in both directions and rejected if the vehicle footprint, another car ahead/behind/alongside, or another car's active passing reservation conflicts.
- Passing reservations are local to the selected offset: a nearby car using the same passing corridor blocks a conflicting pass, while a physically separate free corridor remains available. Same-corridor reservations cover at least 80 m or 3 seconds of travel.
- The overtaking car stays on its selected offset until the target's rear is cleared, then returns to its original offset only after that offset is clear.
- When the current lane is blocked and no passing offset is safe, target speed is limited using the nearest leaders across the full safe corridor. The car waits for a free corridor instead of trying to force through occupied space.
- Urgent passes increase lateral path transition speed to 5 m/s and steering response to 10/s, amplify the path-following steering request by 2.6, and permit the normalized steering command to use the full -1…+1 range. Steering still follows the selected safe path; it is not held at a constant full-lock value.
- Stuck-car rescue is deferred while another vehicle is detected ahead anywhere in the safe corridor within 100 m. A car waiting in traffic is not teleported merely because its speed remains low.

## Known limitations

- Passing is enabled only when the recorded corridor has at least 2.7 m on each side of its midpoint. A symmetric 7 m profile gives lane centers at +1.75 m and -1.75 m; a 3 m profile correctly disables passing.
- The arc-length route geometry and path follower are source-implemented but have not yet been validated in Assetto Corsa/CSP. Normalized steering response still needs in-game calibration from trace telemetry.
- Route-progress conflict checks currently assume traffic follows the same recorded route direction. Opposing-direction traffic on a shared oval still requires explicit direction-aware conflict handling.
- If a locked route projection is lost, the driver currently fails safe by braking rather than attempting an unverified global branch reacquisition.
- Pit handling and automatic recovery from every spin/off-track situation are outside the current traffic implementation; the existing stuck-car rescue remains in place.


### Player driving and native autopilot

When the player drives manually, RoutePilot records car 0 passively as `MODE=PLAYER` and never overrides its controls. When Ctrl+8 activates Assetto Corsa's native AI state for car 0, RoutePilot adopts that car and runs the same route follower, speed planner, lane occupancy checks, passing reservation, overtake and braking logic used by the other AI cars. The trace marks this state as `MODE=AI_AUTOPILOT`; no additional RoutePilot button is added.

### Predictive arc steering and traction control

The AI path follower now uses a pure-pursuit curvature command aimed at a Catmull–Rom spline sample selected by arc distance ahead, with a small sampled-curvature feed-forward term. It no longer uses the next recorded point as its steering target. Preview distance scales with speed (10–28 m), so steering is initiated before the bend.

Throttle is limited by positive wheel slip reported by CSP wheel telemetry and by lateral acceleration in corners. Wheelspin causes an immediate torque reduction with slower throttle recovery. The trace includes WHEEL_SLIP, TC_SCALE, and STEER_CURV for diagnosis. This is a feedback limiter, not a guarantee of zero slip; in-game testing across the target cars is still required.

The curvature speed planner no longer forces an 11 m/s minimum in tight bends; its calculated lateral-acceleration speed cap is allowed to slow the AI further when the route curvature requires it.


## Best-known-good baseline

The user has tested the 2026-11-24 RoutePilot build in Assetto Corsa and confirmed accurate lane keeping and clean coordinated overtaking. Preserve this revision as the known-good baseline. Future behavior changes should be developed on separate experimental branches created from the baseline, not directly on top of it.


## Experimental boundary-safety build

The separate branch `routepilot/predictive-boundary-guard-2026-11-24` contains an experimental correction for AI spawn direction and predictive road-boundary avoidance. It uses the recorded route's forward direction when spawning, heading-aware route selection for the spawn reference, a first-acquisition distance/direction gate, and a lookahead corridor check that transforms saved bounds into spline coordinates and accounts for the car's rotated bounding-box footprint. New trace fields are `BOUNDARY`, `SAFE`, and `BODY`.

This build is not yet validated in Assetto Corsa/CSP. Keep using the confirmed baseline unless deliberately testing this experimental branch.
