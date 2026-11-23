# RoutePilot development status

Last updated: 2026-11-24

## Current implementation

- Working branch: `routepilot/two-lane-traffic-2026-11-24`
- Base: `main` at `fcf5b04bb767638b90a7fc1ed3a069a54089aeac`
- Pull request: [#4 — deterministic independent two-lane AI traffic](https://github.com/rerorerttt/assetto-corsa/pull/4)
- `main` has not been modified.
- Existing route recording, boundary recording, route/bounds save and load, map, world-line drawing and logger are retained.

## Traffic and speed model

- AI cars are initially distributed alternately between physical LEFT and RIGHT lanes. In the current CSP spawn coordinate basis, negative route-relative lateral offset is physical LEFT; spawn labels, driver roles and map indicators use that same convention.
- Corridors at least 6 m wide use two canonical lane centres, normally 1.75 m from the route centreline. Narrower corridors remain single-file. At rest and after each completed merge, the lane centre is re-derived from the current lane role and the recorded bounds; a partial merge offset can never become the next lane's permanent centre.
- Neither lane has a fixed speed limit. With no corner or traffic constraint, the controller requests full throttle. The aggressive planner uses 17.5 m/s² lateral acceleration and 14 m/s² braking assumptions to allow later braking and higher corner speeds. Actual maximum speed is still determined by the car's powertrain, gearing, drag and grip; there is no artificial 200/320 km/h cap.
- At 320 km/h the corner-planning preview expands to approximately the estimated stopping distance plus 45 m. Active AI route projection is restricted to a local segment window to prevent snapping across nearby parallel track sections.
- The speed controller no longer assigns a 40 km/h target to either lane.

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

## Validation status

- Source changes are committed to the working branch only; `main` remains untouched.
- Static Lua block and delimiter checks pass for `RoutePilot.lua` and `driver.lua`; the repeated-abort state reset has been removed, actual-position lane occupancy is used, boundary recovery is enabled, and the previous fixed 40 km/h target is absent. The new implementation has not yet been runtime-validated.
- Assetto Corsa with CSP 0.3.2-preview2 has not been run in this editing session; runtime behavior remains unverified.

## Next runtime checks

Use the saved 7 m corridor with at least four AI cars. Confirm the physical LEFT/RIGHT spawn positions, then observe cars accelerating without a fixed straight-line speed target, braking before corners, initiating a lane change when closing on a leader, rejecting a merge when the adjacent lane is occupied, and braking safely while waiting. Inspect `LANE_CHANGE_START`, `LANE_CHANGE_DONE`, `LANE_CHANGE_ABORT`, `LANE_CHANGE_RETURNED`, `TRAFFIC_GAP`, `TRAFFIC_BLOCKED`, `LAT_OFF`, `BOUNDARY_ERROR`, `BOUNDARY_HARD`, `TARGET`, `GAS`, `BRAKE`, `WHEEL_SLIP`, `TC_FACTOR`, `HEADING_DOT` and any `DRIVER_ERROR` entries. Repeat on a narrow single-file corridor to confirm that cars remain on the centreline and still have no artificial 40 km/h cap.

Do not merge PR #4 until these runtime checks pass.
