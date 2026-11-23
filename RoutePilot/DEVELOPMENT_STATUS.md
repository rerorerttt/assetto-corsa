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
- Corridors at least 6 m wide use two lane centres, normally 1.75 m from the route centreline. Narrower corridors remain single-file.
- Neither lane has a fixed speed limit. With no corner or traffic constraint, the controller requests full throttle. Actual maximum speed is left to the car's powertrain, gearing, drag and grip.
- Both lanes use the same curvature-based corner-speed planner. The planning horizon expands with speed and estimated stopping distance; braking is based on the corner's curvature and distance.
- The speed controller no longer assigns a 40 km/h target to either lane.

## Lane changes and traffic safety

- Cars inspect all active cars, including the player, using their position relative to the current route tangent.
- A closing car requests a lane change at approximately 10 m or earlier when time-to-collision falls below the configured threshold.
- A lane change is permitted only when the adjacent lane's forward and rear clearance windows and the swept transition corridor are clear. Other AI cars' target offsets count as lane reservations to reduce simultaneous conflicting merges.
- Lane changes interpolate smoothly between the existing lane centres. Cars continue following the recorded route and stay in the new lane until another obstacle requires a change.
- If the adjacent lane is occupied, the car remains in its current lane and reduces speed using the leader's speed, available gap and configured braking deceleration. It retries on later updates instead of forcing a merge.
- Traction control remains enabled where CSP exposes the per-car API; driven-wheel slip telemetry and throttle limiting remain active. Throttle is cut while braking.

## SDK and heading

- `ac.overrideCarControls(carIndex)` returns `ac.CarControlsInput`.
- `physics.setAICarPosition(carIndex, pos, dir)` accepts a direction vector; `nil` aligns to the AI spline.
- `physics.preventAIFromRetiring(carIndex)` keeps active AI cars from retiring when called periodically.
- `physics.setAssists({tractionControl='on'}, carIndex)` is used for per-car traction control.
- Spawn and recovery continue to pass the empirically corrected negated route tangent. The next runtime log must confirm positive `HEADING_DOT` and that the physical LEFT/RIGHT labels match the displayed positions.

## Validation status

- Source changes are committed to the working branch only; `main` remains untouched.
- Static source review is still required after the lane-change implementation. No claim of in-game validation is made.
- Assetto Corsa with CSP 0.3.2-preview2 has not been run in this editing session.

## Next runtime checks

Use the saved 7 m corridor with at least four AI cars. Confirm the physical LEFT/RIGHT spawn positions, then observe cars accelerating without a fixed straight-line speed target, braking before corners, initiating a lane change when closing on a leader, rejecting a merge when the adjacent lane is occupied, and braking safely while waiting. Inspect `LANE_CHANGE_START`, `LANE_CHANGE_DONE`, `TRAFFIC_GAP`, `TRAFFIC_BLOCKED`, `TARGET`, `GAS`, `BRAKE`, `WHEEL_SLIP`, `TC_FACTOR`, `HEADING_DOT` and any `DRIVER_ERROR` entries. Repeat on a narrow single-file corridor to confirm that cars remain on the centreline and still have no artificial 40 km/h cap.

Do not merge PR #4 until these runtime checks pass.
