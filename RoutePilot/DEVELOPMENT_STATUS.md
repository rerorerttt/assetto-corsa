# RoutePilot development status

Updated: 2026-11-24

## Active traffic model

This branch is based directly on `main` and replaces the previous generic/random overtake behavior with the requested minimal two-lane rule set.

- Every AI starts in the left lane, whose center is derived from the saved left/right route bounds.
- AI cars alternate by AI car index: odd indices (1, 3, 5, ...) are capped at 40 km/h; even indices (2, 4, 6, ...) use the app's existing `speedMax` and curvature/braking planner.
- Only fast-class cars overtake.
- A fast car detects a slower vehicle ahead in its current left lane, checks that the right lane exists and is clear in a longitudinal window both ahead and behind, then moves to the right lane.
- It stays in the right lane until its front has advanced by the combined vehicle lengths (approximately two car bodies) beyond the target. It then checks the left lane; if occupied, it remains right and retries on later updates. It returns left only when that lane is clear.
- Passing is disabled unless each half of the saved corridor is at least 2.7 m wide. With symmetric 7 m bounds, lane centers are +1.75 m and -1.75 m from the recorded route. On narrower single-lane corridors, all cars stay centered on the recorded route instead of being shifted into a fictitious left lane.
- The existing stuck-car rescue and `teleportHome()` mechanism from `main` were not modified.

## Files changed

- `RoutePilot/driver.lua`: deterministic slow/fast assignment, left-lane target, right-lane pass, two-body clearance before return, and lane occupancy checks.
- `RoutePilot/README.md`: documents the behavior.

## Validation status

Static source inspection only. This branch has not been executed in Assetto Corsa/CSP. Required in-game checks:

1. Confirm all AI cars adopt the controller and initially settle into the left lane.
2. Confirm speed pattern by AI index: 1=40 km/h, 2=fast, 3=40 km/h, 4=fast.
3. Confirm fast cars pass only using the right lane and do not enter it when occupied.
4. Confirm a fast car remains right until it has cleared the target by approximately two vehicle lengths, then returns left only when clear.
5. Confirm single-lane/narrow bounds disable passing.
6. Confirm the existing stuck recovery still teleports a car after its original timeout.

Do not treat static checks as runtime validation. Preserve existing working spawn, AI adoption, steering, and rescue behavior unless a reproducible defect requires a targeted change.


## 2026-11-24 trace-driven corrections

The uploaded `routepilot_trace.txt` exposed three concrete defects in the prior build:

- `laneOccupied()` used a 2.2 m center-distance threshold. On the 7 m symmetric corridor, the player car on the recorded centerline (0 m) was therefore incorrectly treated as occupying the right lane centered at -1.75 m. This caused repeated `OVERTAKE_DENIED right_lane_occupied`. The lane-center threshold is narrowed to 1.25 m so a car between lane centers is not assigned to either lane by mistake.
- `walkLine()` returned only a future point, while `linePointAtLateral()` applied lane offset using the tangent of the segment under the car. In bends this used the wrong normal and distorted the offset path. `walkLine()` now also returns its sampled segment and interpolation parameter; lane offset is applied using the tangent at the future sample.
- Steering preview was configured as 14 m + 0.8 seconds of speed, capped at 50 m. This is excessive for the 3 m-spaced Nevada route points and makes fast cars aim across the inside of corners. Preview is reduced to 7 m + 0.10 seconds, capped at 18 m; steering response/filter are made more responsive.
- The trace also showed a once-per-second `UPDATE_ERROR`: telemetry formatted the removed `p.pace` field with `%.2f`, aborting the outer update callback. Telemetry now safely prints 0 when that legacy field is absent.

These are source-level corrections. The updated Lua has not yet been run in Assetto Corsa; verify corner tracking, right-lane entry, and the telemetry error count in a new trace before calling the behavior validated.


Additional controller refinements in the same trace-driven pass:

- Lane changes now ramp the commanded lateral offset at 1.5 m/s instead of stepping instantly from the left-lane center to the right-lane center and back.
- The fast-car curve planner's lateral-acceleration target is reduced from 9.0 to 6.0 m/s² to leave more grip margin and reduce corner overspeed. This is a conservative control target, not a claim about every car's physical grip.
- Steering preview and lane-offset geometry changes remain as described above. The slow/fast alternation, 40 km/h slow cap, two-body pass clearance, and existing teleport recovery are retained.

## 2026-11-24 latest user trace: controller and lane-change diagnosis

The newest 74-second trace on `skidpad05_skidpad` confirms that the previous steering/lane-change corrections did not solve the reported behavior. Do not treat PR #6 as validated.

Confirmed from the trace and current source:

- The route bounds loaded from disk are `[-1.5, +1.5]` m for all sampled telemetry: a 3 m corridor. `laneCenters()` deliberately returns `nil` unless both halves of the corridor are at least 2.7 m wide. Therefore `updateOvertake()` exits before it can initiate a pass. The trace contains no `OVERTAKE_BEGIN`, `OVERTAKE_END`, or `OVERTAKE_DENIED` events, and every telemetry sample has `OT_SIDE=0`. This is not a failed lane-change ramp: the two-lane state machine is never entered because the loaded route is single-lane. A real 7 m two-lane bounds profile is required; do not fabricate a 7 m corridor on a physically narrower road.
- Steering telemetry proves loss of tracking: commands repeatedly saturate at `STEER=+/-0.750` while signed lateral offset grows to 9–14 m. Some cars track the same route with sub-meter error, so the issue is not simply a globally too-small steering gain. The current controller maps target-point heading error directly to normalized steering (`angle * STEER_GAIN`) rather than calculating path curvature/vehicle steering demand. The target is sampled from a piecewise-linear polyline and lane offset uses one segment tangent, so target direction can change abruptly at vertices. The current trace does not provide enough information to claim one universal gain adjustment will fix this.
- The local nearest-segment search can fall back to a global nearest-segment scan after the car is more than 15 m from the route. On a closed or self-near route this can snap the controller to a geometrically nearby but topologically wrong branch. This must be guarded by route-progress continuity and heading agreement before a global re-acquisition is accepted.
- Overtake detection and clearance still use world-space projections onto the car's instantaneous heading (`carAheadFull()` and `passedDistance`). On bends this is not equivalent to progress along the recorded route and can misclassify front/behind or the two-body clearance. Lane occupancy should also be evaluated in route coordinates, not only by the current heading vector.

Required next implementation work:

1. Replace point-angle steering with a path-following controller that uses a smooth route tangent/curvature, signed cross-track error, heading error, and lateral-velocity damping. The controller must aim along a continuous path rather than chase each raw recorded vertex; add telemetry for heading error, target lateral offset, curvature, and steering saturation.
2. Make route-segment tracking continuity-aware. Keep route progress monotonic for normal forward motion and only re-acquire globally when both distance and heading/topology checks support it.
3. Convert front/behind, lane occupancy, and two-body pass clearance to signed route-progress distances. Preserve the fixed sequence: left lane -> right lane -> complete two-body clearance -> verify left lane clear ahead and behind -> left lane.
4. Keep the physical corridor constraint explicit. The current trace's 3 m bounds cannot support two lanes; test the passing state machine only after loading/recording the intended 7 m bounds profile.
5. Re-run static checks and update this file after each code change. No Assetto Corsa/CSP runtime test has been performed by the assistant; only a new user trace can validate actual handling.

## 2026-11-24 implementation pass: path tracking and route-space traffic

Implemented in source on this branch (not yet runtime-validated):

- Replaced the raw point-angle steering command with a smooth Catmull–Rom path sample and a Stanley-style feedback controller. Steering now combines future path tangent/heading error, signed cross-track error to the active lane center, a bounded preview correction, and reduced lateral-slip damping. The future lane offset uses the tangent of the smoothed sample, not the raw segment chord.
- Added an estimated preview-path curvature and telemetry for CROSS, HEAD, CURVE, route progress, lane target, and steering saturation. Curvature is currently diagnostic; no unverified wheelbase or steering-lock property is assumed from the CSP API.
- Added cached cumulative route distance and per-car route projection. Front/behind gaps, lane occupancy windows, and pass clearance are now calculated along the recorded route rather than by projecting world-space positions onto the car's instantaneous heading.
- The passing car remains in the right lane until route-progress clearance reaches the sum of both vehicle lengths. Returning left still requires a separate occupancy check over both ahead and behind route distances.
- Added per-car route-segment locking. A car may use global nearest-segment reacquisition when it first joins the route; after that, its projection stays within the local progress window to avoid jumping to a geometrically close branch at a crossing. Route metrics rebuild when the number of recorded nodes changes.
- Added an explicit OVERTAKE_DISABLED corridor_too_narrow log for fast cars when the loaded corridor cannot support two lanes. The 3 m profile in the supplied trace remains single-lane; the user must load/record a genuine wide-enough profile (the intended symmetric profile is 7 m).
- First route acquisition is now a one-time global search biased toward the segment direction that best matches the car's heading (absolute alignment, so reverse-facing spawn orientation is not assumed). After acquisition, each car stores its own segment and uses only the local route window. Lane-change destination reservations are visible to other AI cars immediately, before the vehicle has physically crossed into the target lane, preventing multiple fast cars from claiming the same gap in one update cycle. Clearance windows expand with vehicle speed.

Important remaining limits and validation:

- The controller is a first implementation of smooth path tracking, not a vehicle-model controller with calibrated wheelbase/steering ratio. Its normalized steering response still needs in-game calibration against the new CROSS, HEAD, CURVE, and SAT telemetry. Do not increase gains blindly.
- Route-progress traffic logic assumes vehicles follow the same recorded route direction. Opposing-direction traffic on a shared physical oval still needs a direction-aware conflict rule and must be tested explicitly; do not claim that oncoming negotiation is solved.
- Static source inspection has not established runtime correctness. No Assetto Corsa/CSP session has been run by the assistant. Check Lua load/errors, line tracking on straights and bends, first-frame route acquisition, traffic gaps, both lane transitions, and recovery before treating this build as usable.
- Spawn, AI adoption, speed alternation, and teleportHome() were retained. No merge to main was performed.

Source check after this implementation pass: the edited Lua files have balanced block keywords and delimiters, and targeted source assertions confirm that the driver uses route-space leader/pass calculations, lane reservations, smoothed path sampling, first-acquisition route locking, and the existing rescue call. This is only a structural/source check; no Lua compiler was available and no Assetto Corsa/CSP runtime test has been performed.


## 2026-11-24 first-stage route geometry replacement

Implemented on the PR branch; not yet validated in Assetto Corsa/CSP:

- Added `RoutePilot/route_geometry.lua` as the single route-geometry source for the driver. It samples the recorded Catmull–Rom route, accumulates distance along the sampled curve (arc-length lookup table), and exposes route sampling and projection in that same coordinate frame.
- Route projection now returns signed lateral offset relative to the projected curve tangent, route progress in meters, segment metadata, and projection distance. The controller uses that projection for its cross-track error and samples its lookahead point by arc distance; it no longer mixes the raw chord-based `findNearestSeg/lateralOf/walkLine` frame with a Catmull–Rom tangent.
- Curvature is calculated from signed change in neighboring sampled tangents divided by the distance along the sampled route. The speed planner samples this curvature at fixed route-distance intervals ahead instead of using the previous vertex-angle estimate.
- Once a car acquires the route, projection is restricted to a bounded forward/backward arc-distance window around its previous progress. Closed-loop wrap is handled in signed progress space. If a locked projection cannot be found, the driver fails safe (zero throttle/full brake) rather than silently snapping to a distant branch.
- Locked projections search only the local arc-length window using binary search into the sample table; full-route scans are reserved for initial acquisition. Sampling uses 8 subdivisions per recorded segment to limit cache size.
- Loading a route invalidates the cached geometry so a route with the same point count cannot accidentally keep the old spline.

Scope deliberately not claimed as complete: this stage does not calibrate normalized steering against each car's steering ratio/wheelbase, does not add lane-boundary recovery, and does not solve direction-aware opposing traffic. Bounds are still read from the saved per-node corridor and are not expanded or fabricated.

Validation performed: source-level inspection of the changed modules and call sites. No Lua interpreter/compiler and no Assetto Corsa/CSP runtime were available, so syntax/runtime behavior and actual vehicle tracking remain unverified. The next in-game trace must confirm monotonic route progress, stable segment identity through bends and the start/finish seam, consistent signed lateral offset, and finite/non-spiking curvature before this stage is accepted.
