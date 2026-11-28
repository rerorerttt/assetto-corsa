# RoutePilot 0.10.0 — directed CSP lane-network controller

CSP Lua app for Assetto Corsa. When a CSP Traffic Tool network is available, AI cars follow its authored directed lanes and use only graph connections for junction routing and lane changes. Manual RoutePilot routes remain available as a fallback.

## Install

Copy the `RoutePilot` folder into:

`assettocorsa/apps/lua/RoutePilot/`

The folder must contain:
- `manifest.ini`
- `RoutePilot.lua`
- `driver.lua`
- `traffic_network.lua`
- `csp_converter.lua`
- the other Lua modules shipped in this folder

Enable RoutePilot in Content Manager's Lua Apps list and open it in a session.

## Automatic CSP Traffic Tool conversion

RoutePilot reads the current track's original CSP `traffic.json` without modifying it. Every valid authored lane is retained with its original point order, coordinates, lane ID and loop flag. The converter does not merge lanes into a centreline and does not import or generate road boundaries. Malformed or duplicate lane records are counted and reported rather than silently substituted.

The runtime graph contains directed junction transitions extracted from CSP intersection polygons and filtered by `disallowedTrajectories`. It also contains separate lane-change links only where two source lanes remain side-by-side over a continuous distance and their tangents confirm the same travel direction. Opposing-direction lanes are not valid lane-change targets. AI cars are attached to a nearby lane in the direction of travel, spawned across available source lanes, follow that lane's original geometry, and can switch only through its recorded graph links. At junctions they can continue only through an outgoing directed transition. The map and optional world overlay display all imported lanes with arrows showing their authored direction; boundaries are omitted.

The graph-based controller is an experimental replacement for the legacy single-route controller in CSP mode. It uses local lane projection, forward curvature preview, same-lane following and occupancy checks before lane changes. Manual recorded routes continue to use the legacy controller when no usable CSP network is loaded.

## First use

1. If the track has a CSP Traffic Tool `traffic.json`, RoutePilot imports it automatically when no saved RoutePilot line exists.
2. Wait for the status to show the converted point count, then add AI cars and start the autopilot.
3. Manual recording remains available as an override: press **Начать запись маршрута**, drive the route, stop recording, and save it for this track/layout.

Routes are stored separately by full track ID using CSP `ac.storage` and are autoloaded when the app starts on a track with a saved route.

## Current scope

- Records player-car world X/Z coordinates and speed.
- Stores and loads routes per track/layout, with autoload on app start.
- Uses a shared Catmull–Rom route geometry module with an arc-length lookup table, continuity-constrained route projection, signed cross-track feedback, true sampled-curve curvature, and a curvature/braking speed planner.
- Traffic gaps, lane occupancy, and pass clearance are measured along cumulative route progress, not by projecting onto the car's current heading. The controller logs cross-track error, heading error, signed local route curvature, route progress, lane target, and steering saturation.
- Controls only steering, throttle and brake. No gear, clutch, reverse or stock AI spline commands are issued.
- Applies to every active AI-controlled car, including car 0 while Assetto Corsa's native Ctrl+8 autopilot is active. Manual player control remains untouched.

## Changelog

- 0.10.0: experimental full directed CSP lane graph, source-lane visualization, distributed lane spawning, graph-only junction routing and same-direction adjacent-lane changes. No generated road boundaries or merged centreline.

- 0.2.1: fixed route autoload — Lua apps have no `script.onSessionStart` callback, so autoload now happens on the first `script.update` call.
- 0.2.0: car-following collision avoidance, minimum-distance slider, slowing-car counter.

## Traffic behavior (multi-offset passing)

- No configured 170 km/h cruise ceiling: straight-line speed is not capped by RoutePilot. The planner limits speed from upcoming route curvature and the car's measured braking response; vehicle physics determines attainable top speed.
- Any car may pass a slower vehicle when its pace advantage and estimated pass distance make the manoeuvre feasible.
- Passing targets use discrete logical lane slots derived from the measured corridor width (3/7/11/15 m → 1/2/3/4 slots). The complete transition, side-by-side phase and return are checked before a slot is reserved; arbitrary 0.30 m passing offsets are not used.
- Passing reservations are local to the selected offset and limited to nearby time/space conflicts, so a physically separate free corridor remains available. Cars entering the same narrow corridor are serialized by active reservations.
- The overtaking car stays on its selected offset until the target's rear is cleared, then returns to its original offset only after that offset is clear.
- When the current lane is blocked and no passing offset is safe, following speed is limited by the leader in the car's current corridor only. A slow vehicle in a separate lane cannot globally throttle traffic using another free corridor.
- Emergency passing does not increase steering gain or issue full-lock pulses. Lateral transition distance grows with speed and the estimated lateral-acceleration allowance; a pass is rejected when there is insufficient distance to complete the transition before reaching the target.
- Stuck detection is not reset merely because another car exists ahead. After ten seconds without movement, the app attempts a respawn only at a point whose actual spawn position is clear of other vehicles.

## Experimental 0.9.1 changes (2026-11-25)

- Removed the fixed 170 km/h cruise setting. RoutePilot uses an unbounded cruise target; speed is limited by upcoming route curvature and braking distance.
- Braking deceleration and usable lateral grip are estimated from measured vehicle response, then filtered and used by curve planning and car-following. Acceleration is also learned to estimate whether a pass can be completed.
- The forward planning horizon expands with speed and estimated stopping distance, including a reaction-time margin.
- Lane occupancy checks physical vehicle-width overlap, active passing reservations, the swept lateral transition, and opposing-direction traffic using local heading geometry.
- Following speed is evaluated in the current corridor rather than taking the minimum speed across every lateral offset.
- A stationary queue no longer disables stuck recovery. Rescue is attempted after ten seconds and validates the actual spawn position.
- This source revision is experimental and has not yet been validated in Assetto Corsa/CSP. Test each blocked lane, a bumper-distance stationary obstacle, and multiple AI cars passing one at a time.

## Known limitations

- The directed CSP network controller has not yet been validated in Assetto Corsa/CSP. Its junction extraction, lane-change links, spawn placement, steering calibration and traffic clearances require in-game testing on Shuto and other multi-carriageway layouts. Source changes and repository commits are not proof of correct runtime behavior.\n- The graph controller currently uses a separate lightweight following/steering loop; it does not yet reuse the legacy controller's full reservation coordinator, traction estimator or rescue behavior.\n- Passing still requires enough physical width for the complete vehicle footprints plus a safety margin; the controller searches the available safe corridor rather than requiring two pre-defined 2.7 m lanes.
- The arc-length route geometry and path follower are source-implemented but have not yet been validated in Assetto Corsa/CSP. Normalized steering response still needs in-game calibration from trace telemetry.
- Near-field occupancy and leader detection classify each car against its own route tangent. Opposing-direction vehicles use local geometry and closing-speed braking; long-range reservations still need in-game validation on shared ovals and hairpins.
- If a locked route projection is lost, the driver currently fails safe by braking rather than attempting an unverified global branch reacquisition.
- Pit handling and automatic recovery from every spin/off-track situation remain outside the current traffic implementation.


### Player driving and native autopilot

When the player drives manually, RoutePilot records car 0 passively as `MODE=PLAYER` and never overrides its controls. When Ctrl+8 activates Assetto Corsa's native AI state for car 0, RoutePilot adopts that car and runs the same route follower, speed planner, lane occupancy checks, passing reservation, overtake and braking logic used by the other AI cars. The trace marks this state as `MODE=AI_AUTOPILOT`; no additional RoutePilot button is added.

### Predictive arc steering and traction control

The AI path follower uses pure-pursuit curvature aimed at a Catmull–Rom spline sample selected by arc distance ahead. The planner curvature is not added a second time to the pure-pursuit command. Preview distance scales with speed and now extends to 85 m. The racing-line plan is refreshed according to speed, and the preview uses the car's full progress through the existing plan rather than a fixed 30 m offset.

Throttle is limited by positive wheel slip reported by CSP wheel telemetry and by lateral acceleration in corners. Wheelspin causes an immediate torque reduction with slower throttle recovery. The trace includes WHEEL_SLIP, TC_SCALE, and STEER_CURV for diagnosis. This is a feedback limiter, not a guarantee of zero slip; in-game testing across the target cars is still required.

The curvature speed planner no longer forces an 11 m/s minimum in tight bends; its calculated lateral-acceleration speed cap is allowed to slow the AI further when the route curvature requires it.


## Best-known-good baseline

The user has tested the 2026-11-24 RoutePilot build in Assetto Corsa and confirmed accurate lane keeping and clean coordinated overtaking. Preserve this revision as the known-good baseline. Future behavior changes should be developed on separate experimental branches created from the baseline, not directly on top of it.


## Experimental boundary-safety build

The separate branch `routepilot/predictive-boundary-guard-2026-11-24` contains an experimental correction for AI spawn direction and predictive road-boundary avoidance. It uses the recorded route's forward direction when spawning, heading-aware route selection for the spawn reference, a first-acquisition distance/direction gate, and a lookahead corridor check that transforms saved bounds into spline coordinates and accounts for the car's rotated bounding-box footprint. New trace fields are `BOUNDARY`, `SAFE`, and `BODY`.

This build is not yet validated in Assetto Corsa/CSP. Keep using the confirmed baseline unless deliberately testing this experimental branch.

## Experimental racing-corridor optimizer (2026-11-25)

The branch `routepilot/racing-corridor-optimizer-2026-11-25` adds `racing_line.lua`. It builds a short-horizon path from the recorded route and its saved left/right bounds, then optimizes the actual offset path for lower curvature while constraining the vehicle footprint inside the corridor. The speed planner reads curvature from this optimized path instead of blindly treating the recorded reference line as the driven line.

- On open road, the optimizer may use the complete safe road width to open a corner and then naturally converge toward the preferred lane on straights.
- When traffic is close to the route, racing-line optimization is suspended and the car holds its lane. During an overtake or return, the path is locked to the selected/reserved offset.
- The steering input is no longer artificially capped at 75%; the initial lateral-acceleration target is raised to 8 m/s².
- This branch is experimental and has **not** been validated in Assetto Corsa/CSP. In-game tests are required for corner entry/exit speed, road-boundary clearance, side-by-side traffic, opposing traffic, and multi-car pass reservations. Do not treat source-level changes as proof of collision-free driving.

