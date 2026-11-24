# RoutePilot development status

Last updated: 2026-11-24

## Current implementation

- Working branch: `routepilot/two-lane-traffic-2026-11-24`
- Base: `main` at `fcf5b04bb767638b90a7fc1ed3a069a54089aeac`
- Pull request: [#4 — deterministic independent two-lane AI traffic](https://github.com/rerorerttt/assetto-corsa/pull/4)
- `main` has not been modified.
- Existing route recording, boundary recording, route/bounds save and load, map, world-line drawing and logger are retained.

## Traffic and speed model

- The route map stores corridor geometry only: safe lateral bounds and the two permitted lane centres. It does not classify corners/straights and contains no precomputed speed limits.
- Every driver update scans the recorded route ahead locally, at 3 m intervals. Curvature is estimated from adjacent route samples; the target speed follows the local lateral-acceleration limit and a braking envelope to the curve. A straight section requests up to 320 km/h/full available acceleration; the car slows only for curvature or traffic.
- Steering starts with a speed-adaptive 20–110 m preview, but checks the route every 3 m. When accumulated heading change reaches 0.10 rad, the far target is pulled back to the bend entry rather than steering along a chord through the corner.
- Actual maximum speed remains determined by the car's powertrain, gearing, drag and grip. No whole-track speed plan or fixed speed classification remains.

## Lane changes and traffic safety

- Traffic occupancy is measured in the observing car's current route-segment coordinate frame, so two cars are compared against the same local lateral axis. Desired offsets are used only as merge reservations, never as physical positions.
- A closing car requests a lane change at approximately 10 m or earlier when time-to-collision falls below 3 seconds. Forward and rear clearance scale with relative speed and the 2.3-second transition time.
- A lane change is permitted only when the adjacent lane's forward and rear clearance windows are clear and the intermediate swept path between lane centres is unoccupied. Other AI cars' current and target lane centres count as reservations, preventing simultaneous conflicting merges.
- If the destination becomes occupied during a merge, the car starts one return transition to its original lane. Return progress is not reset every frame; the controller continues safe following-speed control while returning. After completion, the lane centre is snapped to the canonical centre for the selected lane.
- If the adjacent lane is occupied, the car remains in its current lane and reduces speed using the leader's speed, available gap and configured braking deceleration. It retries on later updates instead of forcing a merge.
- The target car centre is clamped at least 1.10 m inside recorded bounds. If the actual centre leaves this safety envelope, the controller applies stronger inward steering and reduces speed; a stronger reduction is applied near the recorded edge. Telemetry records `LAT_OFF`, `BOUNDARY_ERROR` and `BOUNDARY_HARD`.
- Traction control remains enabled where CSP exposes the per-car API; driven-wheel slip telemetry and throttle limiting remain active. Throttle is cut while braking.

## SDK and heading

- `ac.overrideCarControls(carIndex)` returns `ac.CarControlsInput`.
- `physics.setAICarPosition(carIndex, pos, dir)` accepts a direction vector; `nil` aligns to the AI spline.
- `physics.preventAIFromRetiring(carIndex)` keeps active AI cars from retiring when called periodically.
- `physics.setAssists({tractionControl='on'}, carIndex)` is used for per-car traction control.
- Spawn and recovery continue to pass the empirically corrected negated route tangent. The next runtime log must confirm positive `HEADING_DOT` and that the physical LEFT/RIGHT labels match the displayed positions.

## Latest log diagnosis (2026-11-24)

- The supplied trace contained 4,478 `LANE_CHANGE_ABORT` events but only 10 `LANE_CHANGE_START` events. The abort branch reset the return transition to progress zero every update, trapping cars between lane centres.
- Traffic detection used other cars' commanded offsets in place of their actual route-relative positions. This could mark the wrong lane occupied or miss a car whose real position differed from its stored target.
- Reconstructing lateral position from the saved route and telemetry segment showed positions from approximately -5.27 m to +10.00 m against recorded bounds of [-3.5 m, +3.5 m]; 345 of 1,500 telemetry samples were outside the 1.10 m vehicle-centre safety margin.
- The Nevada trace reached 32.41 m/s actual speed and 50.07 m/s requested speed. The new corner planner raises assumed lateral acceleration and braking deceleration; straight-line speed remains full throttle with no software speed cap.

## Validation status

- Source changes are committed to the working branch only; `main` remains untouched.
- Static Lua block and delimiter checks pass for `RoutePilot.lua` and `driver.lua`; the repeated-abort state reset has been removed, actual-position lane occupancy is used, boundary recovery is enabled, and the previous fixed 40 km/h target is absent. The new implementation has not yet been runtime-validated.
- Assetto Corsa with CSP 0.3.2-preview2 has not been run in this editing session; runtime behavior remains unverified.

## Next runtime checks

Use the saved 7 m corridor with at least four AI cars. Confirm the physical LEFT/RIGHT spawn positions, then observe cars accelerating without a fixed straight-line speed target, braking before corners, initiating a lane change when closing on a leader, rejecting a merge when the adjacent lane is occupied, and braking safely while waiting. Inspect `LANE_CHANGE_START`, `LANE_CHANGE_DONE`, `LANE_CHANGE_ABORT`, `LANE_CHANGE_RETURNED`, `TRAFFIC_GAP`, `TRAFFIC_BLOCKED`, `LAT_OFF`, `BOUNDARY_ERROR`, `BOUNDARY_HARD`, `TARGET`, `GAS`, `BRAKE`, `WHEEL_SLIP`, `TC_FACTOR`, `HEADING_DOT` and any `DRIVER_ERROR` entries. Repeat on a narrow single-file corridor to confirm that cars remain on the centreline and still have no artificial 40 km/h cap.

Do not merge PR #4 until these runtime checks pass.


## Local path-control revision (2026-11-24)

- Removed the whole-track speed classification and the driver's route-wide speed planner. The track map now contains corridor/lane geometry only.
- Replaced speed planning with a live forward scan of the recorded route at 3 m intervals. Curvature determines the local corner speed; the same preview applies a braking envelope based on distance to the detected bend.
- Steering checks every 3 m in its speed-adaptive preview. Once accumulated heading change reaches 0.10 rad, it shortens the target to the bend entry so the car does not aim at a distant point through a curved section.
- Traffic occupancy now measures every other car's actual lateral position in the current car's route-segment coordinate frame. It no longer compares lateral offsets calculated from unrelated per-car segments. Desired lane offsets remain reservations for merge-conflict checks, not substitutes for physical position.
- Lane changes are triggered only when the car is actually closing on a slower leader (more than 1 m/s), not merely because the gap is under 10 m. The full intermediate band between source and destination lane centres is checked for nearby cars; cars already in the source lane do not falsely block their own overtake.
- Lane-centre tracking now treats crossing the divider into the other lane as a lane violation, even when the car remains inside the overall track boundaries. If a narrow single-lane segment temporarily sets the role to SINGLE, the roster-assigned LEFT/RIGHT role is restored when two lanes become available again.
- These changes are committed to the working branch only. Runtime behavior still requires an Assetto Corsa/CSP test; GitHub comparison confirms the trace file is present and contains roughly 4,326 added lines, but the file-content connector returned an empty payload even for line-range requests; individual runtime events therefore could not be inspected in this pass.

## Regression diagnosis and corrective changes (2026-11-24)

- The live trace confirmed every car was forced to SINGLE with zero lane offset. The new corridor formula placed the two candidate centres exactly at the 2.35 m threshold; floating-point rounding made the strict comparison fail. The map now uses the established symmetric centres (up to ±1.75 m) and a tolerance when deciding whether two lanes fit.
- The trace also showed route-projection mismatches: for example, car 2 was reported on segment 158 while its logged world position was closest to segment 224 in the committed route file; cars 3–5 showed the same opposite-side projection pattern. Spawn now carries the exact source segment through the placement operation and seeds the driver's first projection from that segment. Unspawned cars, including the Ctrl+8 player, use an explicit global nearest-segment search once at initialization.
- The 6–18 m steering lookahead introduced in the previous change has been reverted to the established speed-adaptive 20–110 m lookahead to remove the short-horizon steering oscillation.
- The speed map now reserves 70 km/h for detected hairpins only. Ordinary bends retain the 160 km/h target; short straights target 160 km/h and straights of at least 250 m target 320 km/h. Traffic and boundary recovery can still lower the target when necessary.
- Removed the manual “Рассчитать участки” control. Route-map calculation remains automatic after line/bounds recording or line loading. Removed the “Остановить ИИ” toggle; the one-way “Запустить ИИ” action is shown only when AI control is disabled.
- These are source-level corrections based on the uploaded trace. Assetto Corsa runtime validation is still required.


## Committed overtaking and steering revision (2026-11-24)

- Replaced repeated frame-by-frame merge reconsideration with an explicit overtake state: IDLE -> PASSING -> RETURNING -> IDLE. The driver checks the slower leader, the destination lane ahead and behind, and the recorded lane corridor before committing.
- A blocked or geometrically unavailable destination lane means the car stays in its lane and follows the leader's safe-speed envelope. It does not force a merge or alternate between lanes.
- Once a merge starts, it completes as one smooth 2.3-second transition; the destination is not repeatedly re-evaluated in a way that resets or reverses the transition. The car returns only after the overtaken car is at least 14 m behind and the original lane has the required front/rear clearance.
- Merge clearance uses relative speed over the transition time, with minimum front/rear gaps. The whole transition is checked against the recorded safe corridor at 5 m samples, including vehicle-width clearance; a single-lane section rejects the merge.
- Steering now prefers one distant target and shortens its preview in 15 m steps only when the target chord would leave the assigned lane or recorded corridor. Removed the accumulated-small-turn threshold that shortened the target at every small heading change.
- Removed the 320 km/h straight-line software target. With no detected corner constraint, the controller requests full available throttle; corner curvature and the braking envelope remain the speed constraints. Traffic following and boundary recovery can still reduce speed for safety.
- Source delimiter scan passes after these changes. This is not a Lua runtime or Assetto Corsa validation. The new state machine and target selection still require in-game testing with a multi-car field, both occupied and free adjacent lanes, and narrow/wide recorded corridors.


## Runtime freeze diagnosis and fix (2026-11-24)

- The latest supplied routepilot_trace.txt contains 1,997 identical DRIVER_ERROR entries: driver.lua:517: attempt to perform arithmetic on field VEHICLE_HALF_WIDTH (a nil value).
- The exception occurs during steering setup before normal control can run. The main loop calls releaseCar() after each exception, clears the per-car state, and initializes the same AI again on the next update. This explains the repeated CAR_INIT messages and cars remaining stationary; it is not evidence of a traffic deadlock.
- Driver.setup() now normalizes the configuration and supplies defaults for MAX_LATERAL_ACCEL, BRAKE_ACCEL, VEHICLE_HALF_WIDTH, and BOUNDARY_CLEARANCE. The driver therefore no longer depends on the caller having the newest configuration table.
- The fix is source-level only. Verify in the next runtime trace that DRIVER_ERROR is absent and cars begin producing normal steering/throttle telemetry before evaluating overtaking behavior.


## Isolated player-car line test (2026-11-24)

- Replaced the multi-AI launch action with an isolated player-car autopilot test. Ctrl+8 toggles RoutePilot control of car index 0 using the documented CSP ac.ControlButton binding; the UI button provides the same action.
- The player car is assigned the recorded LEFT lane once. The test does not spawn, teleport, steer, or otherwise take control of AI opponents.
- On activation, the route direction is checked against the player's current heading; if reversed, route points and signed boundary arrays are reversed together, then the lane map is rebuilt.
- Driver single-car test mode bypasses traffic scanning, overtaking, and lane changes. The car follows the saved route with the existing speed-adaptive lookahead and local corner braking; the existing recorded-corridor preview check remains active, but the lane-specific half-road clamp is disabled so the distant route target is not rejected solely for belonging to the opposite side of the local lane classification.
- The control override is applied to player index 0 with ac.overrideCarControls. This is a CSP input override; it does not change Assetto Corsa's internal car.isAIControlled flag. No unsupported API is claimed to convert the player entity into an engine AI.
- Runtime validation is pending. Confirm Ctrl+8 acquisition/release, steering/throttle, lane selection, and fresh telemetry in-game.
