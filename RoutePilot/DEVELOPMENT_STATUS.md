# RoutePilot development status

Updated: 2026-11-26

## Active traffic model — 0.9.3

Active branch: `routepilot/two-lane-traffic-rework-2026-11-26`.

The previous trace was reviewed against the active Lua source. It showed that the advertised slow/fast traffic classes were not actually present in `driver.lua`: every AI used `cruiseSpeed = math.huge`. It also showed that the racing-line planner selected the route midpoint as its base preference, so cars were not physically distributed into logical lanes. The old yield handshake then made passing conditional on the target moving away, while a failed fixed-lane plan forced target speed to zero.

The current implementation replaces that chain with a direct adjacent-lane policy:

- Odd AI indices (1, 3, 5, ...) are slow-class cars capped at 40 km/h.
- Even AI indices (2, 4, 6, ...) are fast-class cars with no fixed cruise cap; only fast-class cars initiate overtakes.
- Outside overtakes, every AI continues to use the full-width racing-line optimizer; no permanent lane assignment or lane-centre bias is applied. For an overtake, the target is assigned to the logical slot nearest its current racing line and the faster car takes an adjacent slot.
- Lane slots are derived from measured bounds: widths 3/7/11/15 m yield 1/2/3/4 slots. On a single-lane corridor there is no pass.
- A fast car detecting a slower AI leader reserves a paired lane change: the target moves to its nearest logical lane slot and the overtaker moves to an adjacent slot in the same manoeuvre. Both begin immediately; there is no yield-ready handshake or timeout.
- Both lane transitions, the pass phase, and the return are checked against road bounds and all active cars. The target's predicted lane change is included in the collision calculation rather than treating it as stationary or ignoring it.
- The exact fixed-offset racing-line plan is prevalidated before a pass starts. If a later far-horizon replan fails, it retries a shorter reserved horizon and keeps the last validated near-field plan while it remains usable. It returns to the base lane only when that lane is clear; it never merges into an occupied source lane. If neither lane has a validated continuation, braking remains the final collision-avoidance response.
- The target holds its temporary lane until the overtaker has cleared it by two vehicle lengths and returned to its original racing line; the target's temporary reservation is then released.
- If the adjacent lane is occupied or the predicted swept path is unsafe, the car follows the leader using the current-lane braking model. The controller does not force an unsafe pass.

## Trace findings that motivated the redesign

The supplied `routepilot_trace.txt` showed:
- `ROUTEPILOT 0.9.1 INITIALIZED` even though the manifest declared 0.9.2; the internal version string was stale.
- `CAR_INIT ... cruise=PHYSICS_LIMITED` for every AI, while the actual `newPersonality()` set every car's `cruiseSpeed` to `math.huge`; the active code had no slow/fast speed differential.
- Repeated `YIELD_REQUEST` / `YIELD_ABORT reason=timeout` events and traffic states `WAIT_YIELD` / `YIELDING`, proving that the old pass policy waited on a target-side manoeuvre.
- 38 `RACE_LINE_BLOCKED ... cause=NO_PATH` records after car 4 entered `PASSING`, followed by `RESCUE_ATTEMPT id=4 success=true`. The update loop converted a failed fixed-offset pass plan into a zero-speed target, causing the observed standstill.
- The most common pass denials were `leader_has_no_safe_yield_space`, `predicted_vehicle_conflict`, and `yielded_corridor_no_longer_clear`.

## Files changed in 0.9.3

- `RoutePilot/driver.lua`: deterministic slow/fast classes; base-lane assignment; immediate adjacent-lane passing without leader-yield state; target-inclusive predicted collision checks; exact pass-plan prevalidation; pass abort and base-lane replanning on runtime plan failure.
- `RoutePilot/racing_line.lua`: lane-preference cost in the optimizer.
- `RoutePilot/RoutePilot.lua`: traffic diagnostics now record speed class, base lane, pass target, plan failure, stuck time, and control status; obsolete yield-queue scheduling removed; app version string updated.
- `RoutePilot/manifest.ini`: version 0.9.3.
- `RoutePilot/README.md`: current traffic model and validation requirements.

## Validation status

Source review and GitHub consistency checks only. The new Lua has not been run in Assetto Corsa/CSP, so actual overtakes, collision-free lane changes, corner transitions, and stop-free operation are not yet runtime-verified. The next test must use this exact 0.9.3 branch and capture a fresh trace with **Трафик лог** enabled. Verify first that cars 1/3/5 settle at 40 km/h in the left lane, cars 2/4 use the adjacent lane to pass, and no `RACE_LINE_BLOCKED mode=PASS` or `OVERTAKE_ABORT reason=pass_plan_no_path` loop occurs.

Historical notes below describe earlier revisions and are superseded wherever they conflict with the active 0.9.3 model.

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

The latest 153-second trace on `skidpad05_skidpad` confirms that the previous steering/lane-change corrections did not solve the reported behavior. Do not treat PR #6 as validated.

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


## 2026-11-24 manual player telemetry test build

Added passive telemetry for the human-driven car (car index 0), using the same one-second log cadence and the same telemetry formatter as AI cars:

- The player is observed every app update frame to keep route projection/progress continuous; the most recent snapshot is written by the existing once-per-second logger.
- The player line is written as `CAR=0 ... MODE=PLAYER` in the existing `routepilot_trace.txt`, alongside AI rows if AI cars are present.
- Position, speed, velocity-derived slip/yaw/lateral acceleration, route segment, route progress, distance from the drawn route, signed lateral offset, heading error, sampled route curvature, and saved bounds are measured from the player car and the shared route geometry.
- `STEER` is the normalized live controller input from the documented `ac.getControllerSteerValue()`; `GAS` and `BRAKE` are the actual `ac.StateCar.gas` and `ac.StateCar.brake` values. These are observational values, not commands.
- `TARGET_SPEED=-1.00` marks “not applicable” for the manually driven car; `PACE=0`, `OT_SIDE=0`, and `LANE=0` are neutral placeholders because the player has no AI target or overtake lane. `SAT` is set only when normalized player steering input is effectively at its full limit.
- The player observer is separate from `state.p` and is never passed to `Driver.update()`. It does not call `ac.overrideCarControls()`, so it cannot steer, throttle, brake, or otherwise take control of the player's car.
- Player telemetry is emitted only when a route with at least two points exists and route recording is not actively in progress. It works whether the route was loaded from disk or has just been drawn and stopped.

Validation: verified the input/property names against the repository's bundled CSP Lua SDK (`ac.getControllerSteerValue()`, `ac.StateCar.gas`, `ac.StateCar.brake`, `ac.StateCar.position/velocity/look/speedMs`). Source-level checks confirm player observation is called per update, the row is written through the existing logger, and no player control override is made. No Assetto Corsa/CSP runtime is available here; the next in-game check must confirm one `CAR=0 MODE=PLAYER` row per second with changing inputs and route metrics while the car is driven manually.


## 2026-11-24 predictive arc steering and wheel-slip traction control

Implemented on the feature branch; pending in-game validation:

- Replaced the previous blended heading/cross-track point controller with a pure-pursuit curvature controller. The target is sampled from the Catmull–Rom route by arc distance ahead of the car; the controller calculates signed curvature from the target point in the vehicle frame. It does not steer toward the next raw recorded node or treat each node pair as the driving target.
- Increased the speed-dependent preview from 7 + 0.10 × speed (clamped to 7–18 m) to 12 + 0.35 × speed (clamped to 10–28 m). The sampled spline curvature at 55% of the preview distance is added as a 20% feed-forward term, so steering begins before the heading error becomes large. Normalized steering gain is now 4.5; steering limit and rate limit remain in place.
- Added a closed-loop traction limiter using the documented CSP ac.StateCar.wheels[0..3].slipRatio and .speedDifference values. Positive longitudinal wheel slip cuts throttle immediately; throttle is restored gradually at 1.25 scale units/second. The configured intervention starts at 0.10 slip and reaches its maximum 88% cut at 0.28 slip.
- Added a cornering grip reserve: requested throttle is reduced as measured lateral acceleration approaches the planner's configured lateral-acceleration limit. The existing low-speed launch cap remains as a secondary safeguard.
- The same wheel-slip measurement is included in passive player telemetry for comparison. AI trace rows now append WHEEL_SLIP, TC_SCALE, and STEER_CURV; player rows show measured wheel slip and a neutral traction scale because the app does not control the player.
- Verified the wheel and steering-related field names against the repository's bundled CSP Lua SDK. Static source assertions confirm the pure-pursuit arc target, curvature feed-forward, wheel-slip read, throttle cut/recovery, telemetry fields, and absence of player control override.

Limitations: this is a feedback controller, not a physical guarantee that no tyre can ever spin. Grip, tyre model, drivetrain layout, surface, and physics update timing vary by car. No Assetto Corsa/CSP runtime test or Lua compiler test has been run in this environment. The next in-game trace must be used to verify steering sign, bend entry/exit, lateral error, wheel-slip response, and whether throttle oscillates. Do not treat this implementation as validated until that test is reviewed.

- Corrected the curvature speed cap: removed the old 11 m/s minimum-apex-speed floor, which could override the calculated safe speed in tight bends. The planner now preserves the curvature-derived speed cap instead of forcing the AI to enter a bend too fast.


## 2026-11-24 BEST-KNOWN-GOOD BASELINE

The user tested this build in Assetto Corsa and explicitly confirmed it as the best-performing RoutePilot version so far: cars hold their assigned lanes accurately, faster cars change lanes and overtake cleanly. Treat this exact revision as the known-good baseline. Do not replace its lane assignment, lane-following, overtaking coordination, or steering/throttle behavior during future experiments without first creating a separate experimental branch from this baseline. Preserve this branch and compare future changes against it. User's in-game confirmation is the validation evidence for lane discipline and overtaking; it does not imply every possible car/track combination has been tested.


## 2026-11-24 experimental: spawn direction and predictive boundary guard

Created experimental branch `routepilot/predictive-boundary-guard-2026-11-24` from the user-confirmed best-known-good commit `15b65f3b9be588f210e7b4490eb8c63ba9ad8864`. The baseline branch remains untouched.

The fresh `nevadadrift` trace exposed an initial route-tracking failure: immediately after `SPAWN_FINISH`, AI rows reported 70–81 m line error and heading errors close to 180 degrees. Source inspection found that spawn passed the negated recorded-route direction to `physics.setAICarPosition()`. The experimental branch now passes the recorded forward vector. Spawn reference projection also scores candidate route segments using the player's heading, reducing wrong-branch selection at crossings, and logs the selected reference and each spawned car's position/direction.

The driver now rejects first-time route acquisition when the nearest route sample is more than 8 m away or points substantially against the car's heading. Rejected acquisition fails safe instead of commanding the AI to drive toward a distant, topologically unrelated section.

A predictive corridor guard was added to steering. It samples saved bounds across the full lookahead horizon, transforms those bounds from the raw node-chord frame into the smoothed spline's lateral frame, includes the vehicle's rotated AABB footprint (width and length) plus a safety margin, and constrains the lane target to the common safe lateral interval. It predicts lateral drift from current velocity and adds corrective steering before the projected footprint reaches a boundary. If predicted boundary error exceeds 0.45 m, the speed target is reduced. Trace rows now include `BOUNDARY`, `SAFE`, and `BODY` diagnostics.

Validation performed so far: source-level inspection and targeted static assertions only. This branch has not been run in Assetto Corsa/CSP. In-game verification is required for spawn heading, first route acquisition, corner cutting, full-body boundary clearance, and lane-change/overtake preservation. Do not merge this branch or alter the best-known-good baseline until the user tests it.


## 2026-11-24 follow-up: deterministic spawn projection and boundary footprint

The fresh trace on `routepilot/alternating-left-lane-passing-2026-11-24` records `LOAD_ROUTE points=240`, `SPAWN_FINISH moved=5 failed=0`, then the first AI samples show 70.49–81.25 m route distance errors and heading errors close to 180 degrees. The player sample at the same start has about 20 m route distance error. This confirms that the AI's first route association is not trustworthy and that steering-gain changes are not an appropriate fix. The checked-in `line_nevadadrift.txt` currently contains 230 points, not the 240 reported by the runtime trace; therefore the exact route data used in that session is not present in this branch and the geometric failure cannot be reproduced exactly from the committed route file.

The SDK declaration for `physics.setAICarPosition(carIndex, pos, dir)` documents `dir` only as a vector and does not specify its sign convention. The experimental positive-vector change has therefore been reverted to the previously user-verified negative-vector call. No claim is made that the sign itself caused the route jump.

Spawn now retains the exact raw route segment index and interpolation parameter used to place each AI. The driver maps that pair into the Catmull–Rom geometry's arc-length coordinate with `Geometry.distanceAt()` and seeds the car's route lock before its first control update. This avoids relying on a global nearest-point search at a crossing or near-parallel section. The initial seeded projection is rejected if it is more than 8 m from the expected local spline; it fails safe instead of acquiring a distant branch. Spawn trace rows include the raw segment and interpolation parameter, and the driver logs the resulting seeded progress.

Geometry sample/projection metadata now interpolates the raw segment parameter when both spline samples belong to the same recorded segment. This is required for boundary points to be transformed from the raw chord frame into the smoothed spline frame without snapping the interpolation parameter to one of two sample endpoints.

The predictive boundary guard now calculates the vehicle footprint envelope against each future route tangent across the preview horizon, rather than holding the current heading angle constant through the whole bend. If the intersection of safe lateral intervals is infeasible, it records the corridor deficit, chooses the minimax midpoint instead of an unrelated single-sample center, and requests stronger deceleration for a substantial deficit. Lane-offset clamping no longer teleports the commanded offset instantly to the safe interval; it preserves the existing rate-limited transition while boundary steering corrects the trajectory.

Validation performed: inspected the fresh trace, confirmed the SDK declaration, checked that spawn hints are carried from placement through driver initialization, and ran source-level assertions for the new route seeding, metadata interpolation, footprint calculation, deficit telemetry, and braking path. No Lua interpreter/compiler or Assetto Corsa/CSP runtime is available here. This branch remains experimental and has not been tested in-game. The user-confirmed best-known-good branch and PR #6 were not modified.

Additional source-level check for the route-distance seed mapping: on the checked-in 230-point Nevada route, the Catmull–Rom geometry contains 1,841 samples; mapping raw segment/t values to spline arc distance differs from the corresponding raw-polyline distance by at most 1.42 m across sampled test positions. This is within the 8 m seed rejection threshold for that checked-in route. It does not validate the separate 240-point route reported by the runtime trace.


## 2026-11-24 follow-up: remove false boundary-induced stops

The user's latest in-game trace confirms that spawn route seeding now works and the predictive wall guard prevented visible track departures. It also shows repeated full braking and stationary AI cars despite route projection remaining valid and `DEFICIT=0.00`.

Trace inspection found the direct cause in the driver update: `boundaryEmergency` was set whenever the predicted lateral correction magnitude exceeded 0.45 m, even when the common safe corridor was feasible. The update loop then reduced target speed by 4 m/s (or 8 m/s for a deficit), recalculated every frame. This made a lateral steering correction behave like a persistent deceleration command; as speed fell below the decrement, target speed became zero. Example: at 13.78 s CAR 4 had `LINE_ERROR=2.05`, `BOUNDARY=1.29`, `DEFICIT=0.00`, `SPEED=2.93`, and `TARGET_SPEED=0.00`; at 14.78 s it remained stationary with valid route tracking and no boundary deficit. Similar patterns occur for other cars.

Removed the boundary-emergency speed reduction. The predictive corridor still clamps the lane target to the safe interval and adds corrective steering based on predicted lateral drift; the body-footprint guard and its deficit telemetry remain. Boundary correction is now handled by steering rather than automatically braking. This change does not remove curvature-based speed planning, leader-following safety, or the projection-loss fail-safe.

Validation: inspected 2,814 AI telemetry rows in the supplied trace; 99 contained `BRAKE=1.000`, and the representative stop sequence matched the boundary-emergency deceleration exactly. The code change was checked against the exact source block and committed on this experimental branch. No Lua compiler or Assetto Corsa/CSP runtime is available here, so the new behavior still requires an in-game test. The best-known-good baseline and PR #6 remain untouched.


## 2026-11-25 predictive close-obstacle passing and player detection

Reviewed the latest routepilot_trace.txt from the experimental branch before changing the driver:

- The trace contains 4,464 lines, 2,155 AI telemetry rows, and 431 player telemetry rows. The player stayed at the same world position throughout the capture and all 431 player rows reported LINE_ERROR=999.00; the player therefore never acquired a valid route projection in this session.
- Existing lane-change logic completed multiple passes, but it only considered a leader after route projection succeeded and waited for the normal follow trigger. Consequently, an unprojected player could not be selected as a leader or reliably block a passing lane.
- The trace contains 12 OVERTAKE_BEGIN, 10 OVERTAKE_END, and 1,823 repeated OVERTAKE_DENIED right_lane_occupied messages. The repeated denial messages were generated on successive update frames and obscured useful trace information.

Implemented on this experimental branch:

- Added a bounded local world-space fallback for car index 0 only. When the player has no valid spline projection, the AI projects the player's position onto its own current route tangent and can recognize the player as a nearby leader or as an obstruction in the destination lane. Failed projections for other AI cars remain fail-closed to avoid treating a car on another route branch as a same-lane obstacle.
- Lane occupancy now considers vehicle body widths, reserved AI lane targets, and nearby side-by-side traffic. It checks both front and rear approach windows before a lane change. The same physical player fallback is used when checking the destination lane.
- Added an urgent pass trigger for fast AI: when a slower leader is within a speed-dependent approach distance and the AI has a meaningful potential speed advantage, it may begin the pass immediately instead of waiting for the normal follow timer. It still requires the passing lane to be clear and the complete vehicle footprint to fit the safe corridor over a 40 m forward preview.
- During an urgent pass, lateral lane-offset transition and steering response are temporarily faster, while the existing steering limit, stability/traction controls, predictive corridor guard, and normal pass-clearance rules remain active. The steering is deliberately not hard-locked to full input: a permanent full-lock command would undermine control and could cause a spin or boundary departure.
- The AI continues to hold the passing lane until rear-bumper clearance is established, then checks the return lane before moving back. If the player is the pass target and has no route projection, return clearance is measured in local route-tangent space.
- Added a one-second retry cooldown and one-shot denial logging so an unavailable passing lane no longer floods the trace every physics update.

Validation performed: fetched the updated file from GitHub after each write; confirmed all expected urgent-pass/player-fallback/clearance code is present; a structural Lua block and delimiter scan reports balanced blocks and delimiters. This is not a Lua compiler test. No Assetto Corsa/CSP runtime test has been performed. The new behavior remains experimental and must be checked in-game for steering response, lane clearance, player avoidance, and return-to-lane behavior. The trace's player was stationary and outside valid route projection, so it does not itself demonstrate that an AI successfully avoided a moving player.

Latest code commit: 34a2430d11481a9c6431490778667610f7a94f0d. The urgent trigger now uses a 25–60 m speed-dependent lead window, requires at least 5 m/s potential speed advantage, and rejects passes whose estimated relative-distance-to-clear exceeds 130 m. The best-known-good baseline, PR #6, and main remain untouched.


## 2026-11-25 unified traffic and native player autopilot (experimental)

Reviewed the latest supplied `RoutePilot/routepilot_trace.txt` from the predictive-boundary branch:

- The log contains 1,460 lines and 234 player telemetry rows. All player rows are still marked `MODE=PLAYER`; the player's measured speed ranges from 0 to 34.12 m/s. Therefore this capture demonstrates that AI cars can detect and pass the player, but it does not contain a test of the player's native Ctrl+8 AI state.
- The trace still initializes cars 1, 3 and 5 as `SLOW cruise=40.0kmh` and cars 2 and 4 as `FAST cruise=170.0kmh`. Source inspection confirmed that the 40 km/h class was created in `newPersonality()` and that `p.slow` also blocked those cars from initiating overtakes.
- The same log records six overtake starts, five completions and six denials. Existing player avoidance was retained.

Implemented on the new experimental branch `routepilot/unified-traffic-player-autopilot-2026-11-25`:

- The app now checks the existing `car.isAIControlled` state for car index 0. When the native Assetto Corsa Ctrl+8 autopilot activates that state, car 0 is passed through the same `Driver.update()` controller as every other AI car. It receives the recorded route follower, speed planner, lane logic, traffic checks, steering, throttle and braking. No RoutePilot button or custom hotkey was added.
- When the player is not in native AI mode, car 0 remains passive telemetry only; RoutePilot does not override manual steering, throttle or brake. Telemetry distinguishes `MODE=PLAYER` from `MODE=AI_AUTOPILOT`, and avoids writing duplicate rows.
- Removed the alternating index-based SLOW/FAST personality. Every controlled car now uses the same configured `speedMax` ceiling (default 170 km/h) and curve/braking planner; the artificial 40 km/h limit and the fast-only overtake gate are gone. Actual acceleration and attainable speed remain dependent on each car's physics.
- Added a local passing-lane reservation: while an AI car is in an overtake, another car within `max(140 m, speed × 4.5 s)` cannot start a competing pass. This serializes overtakes in a nearby convoy while allowing separated groups to pass independently.
- During an overtake, the controller also checks the vehicle ahead in the passing lane and applies the same safe-following-speed calculation. When the car's own lane is blocked and a vehicle is also ahead in the passing lane, its target speed is limited by the safer of the two lane leaders. If the passing lane is unavailable, the car follows the leader in its own lane instead of attempting a conflicting move.
- Existing front/rear/side occupancy checks, vehicle footprint checks, safe corridor checks, rear-bumper clearance before returning, player fallback detection, projection-loss braking and traction control remain in place.

Source-level validation: the edited app and driver were fetched back from GitHub; no old SLOW/FAST/40 km/h class references remain in the driver; player index 0 is included only when the game reports it as AI-controlled; the new pass reservation and both-lane following-speed paths are present; Lua block and delimiter scans are balanced. This is not a Lua compiler test. No Assetto Corsa/CSP runtime is available here, so Ctrl+8 adoption, player line tracking, convoy serialization and braking behavior still require an in-game test. In particular, the next trace should show `PLAYER_AUTOPILOT_ADOPT`, `CAR_INIT id=0`, and `MODE=AI_AUTOPILOT` while Ctrl+8 is active.

Code commits: app `ff37a0aa3f1c8bb94666beecece632cf39868f98`; driver `8a1466cfdf15a066d8aebd1e2af6874f1fa48381`; two-lane braking follow-up `40c9e2a3345b2104ebaf9e1a7e9291bdffdd78ab`. The best-known-good baseline, PR #6, the previous predictive-boundary branch, and `main` remain untouched.


## 2026-11-25 multi-lane emergency avoidance (experimental)

Reviewed the trace supplied from `routepilot/unified-traffic-player-autopilot-2026-11-25`:

- 1,630 lines, 263 telemetry rows per car (cars 0–5), four `OVERTAKE_BEGIN`, two `OVERTAKE_END`, nine `OVERTAKE_DENIED`, and one explicit `RESCUE id=1` at 233.89 s. The five AI cars did move during the capture (maximum sampled speeds: 23.86–32.31 m/s); the trace does not show all five cars remaining stopped for the entire session. It does show only two completed passes out of four starts and repeated denials.
- The denial reasons were `passing_lane_not_safe` and `right_lane_occupied`. Source review confirmed the controller still had a hard-coded right-lane target, a 5.4 m two-lane gate, and a single passing offset. Urgent mode only increased steering response; it did not search another offset if the right-lane check failed.
- The explicit rescue at 233.89 s matches the unconditional rescue rule: any AI car below 1.5 m/s for 3.5 s was teleported, even if it was queued behind traffic.
- All 263 player rows were `MODE=PLAYER`; there were no `PLAYER_AUTOPILOT_ADOPT` or `MODE=AI_AUTOPILOT` events in this trace. This session therefore does not validate Ctrl+8 adoption. The player projection was invalid (`LINE_ERROR=999`) in 168 of 263 rows.

Implemented on `routepilot/multi-lane-emergency-evasion-2026-11-25`:

- Replaced the fixed right-lane maneuver with a dynamic search across the complete predicted safe lateral corridor. Candidate offsets are sampled every 0.35 m, can be on either side of the current lane, and must clear the target vehicle's body width plus margin.
- Every candidate is checked against physical vehicle positions, front/rear/side occupancy, other AI passing offsets, and the predicted road-boundary corridor. Failed spline projection no longer makes another car invisible to occupancy checks: a bounded local world-space projection is used for nearby cars, not just car 0.
- Removed the global 140 m passing reservation. A reservation now blocks only a nearby car trying to use substantially the same lateral passing corridor; it covers at least 80 m or 3 seconds of travel. This allows genuinely separate free corridors to be used concurrently while serializing cars attempting the same passing line.
- Overtakes can start earlier (45–105 m decision window), including urgent close-gap passes, with a lower 3 m/s estimated pace advantage threshold and a 180 m pass-distance feasibility ceiling. A car at bumper distance can still be detected: leader detection now tolerates a small negative center-to-center gap corresponding to overlapping vehicle footprints.
- Urgent steering multiplies the path-following command by 2.6, allows the full normalized `-1..+1` steering range, raises steering response to 10/s, and raises lateral target transition to 5 m/s. This gives the controller full available steering authority when the selected safe trajectory demands it; it does not hold the wheel at a blind constant lock that could drive into a wall or another car.
- If no passing offset is safe, the car computes a safe following speed from the nearest leaders across the entire available corridor, rather than considering only a fixed second lane.
- Rescue now defers while any vehicle is detected ahead in the safe corridor within 100 m. This prevents the previous 1.5 m/s-for-3.5 s rule from teleporting a car merely because it is waiting in a queue. Recovery remains available when the car is slow and no nearby traffic blocks the route.
- Pass completion uses local route-tangent position as a fallback if the target loses spline projection, and the target being overtaken is excluded from the passing-lane lead-speed limiter so it cannot make the overtaker brake behind the car it is trying to pass.

Static validation after fetching the committed files back from GitHub: Lua block-token stacks are balanced in both edited scripts; delimiters are balanced; the driver has no remaining fixed-right-lane or legacy denial paths; dynamic candidate search, both-direction pass offsets, full-range emergency steering, all-corridor braking, traffic-aware rescue deferral, and target-clearance fallback are present. No Lua/LuaJIT compiler is installed in this environment, and no Assetto Corsa/CSP runtime test was performed. This version remains experimental until tested in-game. The next trace should be checked for `OVERTAKE_BEGIN from=... to=...`, `RESCUE_DEFER`, `OVERTAKE_END`, absence of queue-triggered `RESCUE`, and whether the player is actually detected in the lane being occupied.

Code commits in this branch: dynamic passing offsets `44ac2f8e2cc40e52b279110ec4236a3a3578cd06`; pass activation and traffic-aware rescue `5cece84af7626ec6c69f3879ebfcae0be8ec96c3`; projection-loss clearance fallback and emergency steering configuration `eb7bf085f1ab784458668c9f51bd117cc9e62a58`; same-lane reservation and rescue distance adjustment `b738eb3fc1ebd02dfcda0889e816b26691b391ef`; active-pass target exclusion `7784648fa92e5f236da08597799ff0cd633ac674`; first-frame source-lane preservation `dc784c4b7281729100d7bd2e5122cf742c4ee4a0`. The known-good baseline, earlier experimental branches, PR #6 and `main` remain unchanged.


Emergency full-lock refinement: for a validated urgent pass where the leader is within 8 m center-to-center, `filterSteering()` now applies a brief 80 ms full-range (`-1` or `+1`) steering pulse in the sign of the computed safe-path steering request, then returns to the path-following controller. The pulse is not applied if the steering request is effectively zero. The normal emergency steering gain/rate and 5 m/s lateral transition remain active after the pulse. This is the explicit bumper-distance escape response requested; it still depends on `findPassingOffset()` finding a physically clear corridor first.

Latest driver commits: bumper-distance pulse `09bb989e0207fb0613353bc6d71fb43c1a5edba9`; zero-request guard `1397fbf0ccf1ecbadff4a393a41b0ce19943026a`. These final refinements also remain untested in the game.


## 2026-11-26 straight-line stability and GDD audit pass

Working branch: routepilot/straight-line-stability-audit-2026-11-26, based on routepilot/ideal-controller-foundation-2026-11-26. This is a separate branch; main and the previous working branch are untouched.

Implemented:
- Removed duplicate curvature feed-forward from pure-pursuit steering.
- Added temporal continuity cost to the racing-line optimizer and fail-closed handling when all candidate offsets are blocked.
- Removed emergency full-lock pulses and emergency steering-gain multiplication.
- Increased speed-scaled preview to 85 m and synchronized the plan's origin with actual route progress; replan interval scales with speed.
- Made plan sampling beyond the actual horizon return nil, allowing the speed planner and path follower to fall back to route geometry instead of targeting the final stale plan point.
- Replaced continuous 0.30 m passing-offset search with discrete logical lane slots.
- Made lane-transition distance speed- and lateral-acceleration-dependent; infeasible passes are rejected.
- Corrected current path-offset telemetry and activated the previously unused settled-state steering deadband.
- Removed unused stability calculation and unused racing-line helper.

Open GDD gaps are recorded in ENGINEERING_AUDIT.md: default/inferred road bounds, the not-yet-time-optimal path/speed planner, distributed traffic arbitration, immediate full braking on projection loss, and unbenchmarked multi-car performance.

Validation is static only. GitHub files were fetched back after the edits; no Lua compiler and no Assetto Corsa/CSP runtime test were available. Do not treat this branch as proven stable until the straight-line and subsequent traffic tests confirm it.
