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

- Traffic occupancy is measured from each active car's actual position projected onto its nearest route segment; the controller no longer substitutes another car's stored desired offset for its real position.
- A closing car requests a lane change at approximately 10 m or earlier when time-to-collision falls below 3 seconds. Forward and rear clearance scale with relative speed and the 2.3-second transition time.
- A lane change is permitted only when the adjacent lane's forward and rear clearance windows and the swept transition corridor are clear. Other AI cars' current and target lane centres count as reservations, preventing simultaneous conflicting merges.
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
- These changes are committed to the working branch only. Runtime behavior still requires an Assetto Corsa/CSP test; the current GitHub trace blob is empty, so the newly supplied URL did not provide readable trace lines in this inspection.

## Regression diagnosis and corrective changes (2026-11-24)

- The live trace confirmed every car was forced to SINGLE with zero lane offset. The new corridor formula placed the two candidate centres exactly at the 2.35 m threshold; floating-point rounding made the strict comparison fail. The map now uses the established symmetric centres (up to ±1.75 m) and a tolerance when deciding whether two lanes fit.
- The trace also showed route-projection mismatches: for example, car 2 was reported on segment 158 while its logged world position was closest to segment 224 in the committed route file; cars 3–5 showed the same opposite-side projection pattern. Spawn now carries the exact source segment through the placement operation and seeds the driver's first projection from that segment. Unspawned cars, including the Ctrl+8 player, use an explicit global nearest-segment search once at initialization.
- The 6–18 m steering lookahead introduced in the previous change has been reverted to the established speed-adaptive 20–110 m lookahead to remove the short-horizon steering oscillation.
- The speed map now reserves 70 km/h for detected hairpins only. Ordinary bends retain the 160 km/h target; short straights target 160 km/h and straights of at least 250 m target 320 km/h. Traffic and boundary recovery can still lower the target when necessary.
- Removed the manual “Рассчитать участки” control. Route-map calculation remains automatic after line/bounds recording or line loading. Removed the “Остановить ИИ” toggle; the one-way “Запустить ИИ” action is shown only when AI control is disabled.
- These are source-level corrections based on the uploaded trace. Assetto Corsa runtime validation is still required.
