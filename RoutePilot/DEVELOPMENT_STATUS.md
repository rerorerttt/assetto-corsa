# RoutePilot development status

Last updated: 2026-11-24

## Current implementation

- Working branch: `routepilot/two-lane-traffic-2026-11-24`
- Base: `main` at `fcf5b04bb767638b90a7fc1ed3a069a54089aeac`
- Pull request: [#4 — deterministic independent two-lane AI traffic](https://github.com/rerorerttt/assetto-corsa/pull/4)
- `main` has not been modified.
- The existing RoutePilot interface, route recording, boundary recording, route/bounds save and load, map, world-line drawing and logger are retained.

## Traffic rules

- Active AI cars are assigned in roster order: first, third, fifth, etc. to the left/fast lane; second, fourth, sixth, etc. to the right/40 km/h lane.
- Assignments are stored by car index and remain stable while cars are temporarily inactive. Newly appearing cars are assigned to the less populated active lane.
- A recorded corridor narrower than 6 m is treated as one lane: offset 0 m and target speed 40 km/h for all AI cars.
- A corridor at least 6 m wide uses fixed lane offsets of +1.75 m (left/fast) and -1.75 m (right/40 km/h), clamped to the recorded boundaries.
- Right lane target: 40 km/h using PI speed control.
- Left lane: no fixed speed ceiling. It uses full throttle when no corner requires braking; the route-curvature planner supplies a temporary target speed only when a corner is ahead.
- Cars do not sense or react to other cars during normal driving. No following, yielding, overtaking decisions or inter-car braking. The faster left lane passes the slower right lane naturally.
- Steering uses a filtered, rate-limited look-ahead target on the assigned lane offset.
- Startup placement and stuck recovery both place cars at their lane offset.
- Before the initial spawn, route point order is checked against the player's forward vector. If the player is on the recorded route and the dot product is negative, route points are reversed and signed bounds are mirrored (`newLeft = -oldRight`, `newRight = -oldLeft`), so the lane axis follows actual travel direction.

## SDK and heading

Checked the SDK in `lua-sdk/ac_apps/README.md`:
- `ac.overrideCarControls(carIndex)` returns `ac.CarControlsInput`.
- `physics.setAICarPosition(carIndex, pos, dir)` accepts a direction vector; `nil` aligns to the AI spline.
- `physics.preventAIFromRetiring(carIndex)` keeps active AI cars from retiring when called periodically.

- `physics.setAssists({tractionControl='on'}, carIndex)` is documented for per-car assists. The driver applies it once per AI car, then reads driven-wheel `slipRatio` from `car.wheels` and limits throttle if wheelspin rises.
- Steering lookahead and rate limits are more conservative; throttle rise is ramped, wheelspin throttle cut is faster, and the right-lane PI integral range is expanded after the previous log showed a 35.5 km/h plateau.

## Repository track data

`RoutePilot/line_nevadadrift_bounds.txt` on `main` contains 230 entries, each with bounds `-3.50 3.50` (7.00 m total), matching the two-lane layout.

## Validation status

- Static block and delimiter-balance checks passed for `RoutePilot.lua` and `driver.lua`.
- Confirmed the new driver contains no car-following, leader sensing or overtaking logic.
- Confirmed existing UI, route recording, bounds recording, map and world-line hooks remain present.
- Static review only after the latest controller changes; not yet run inside Assetto Corsa with CSP 0.3.2-preview2. Do not claim in-game validation. Confirm the `ROUTE_DIRECTION_OK/REVERSED` event, `HEADING_DOT` near +1, left-fast/right-40 physical placement, `WHEEL_SLIP` and `TC_FACTOR` response, stable 40 km/h speed hold, and smooth steering in a live session.

## Next runtime checks

1. Start a session with the saved 7 m Nevada Drift bounds and at least four AI cars.
2. Confirm the log reports `ROUTE_DIRECTION_OK` or `ROUTE_DIRECTION_REVERSED` appropriately, then confirm cars appear at +1.75 m (left/fast) and -1.75 m (right/40 km/h) with `HEADING_DOT` positive and near +1.
3. Confirm right-lane cars settle at 40 km/h and remain there; inspect `WHEEL_SLIP` and `TC_FACTOR` during launches.
4. Confirm left-lane cars accelerate beyond 40 km/h, brake only for route curvature and pass right-lane cars without changing lane.
5. Repeat with the 3 m width preset: all AI cars should stay on the centreline at 40 km/h.
6. Inspect `routepilot_trace.txt` for `LANE_ASSIGN`, `SPAWN_LANE`, `CAR_INIT`, `CAR=` telemetry and any `DRIVER_ERROR` entries.

Do not merge PR #4 until these runtime checks pass.
