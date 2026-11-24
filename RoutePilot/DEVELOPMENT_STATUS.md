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
