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
- Left lane: no 40 km/h limiter; target speed is determined by the route-curvature planner, with a 300 km/h ceiling and braking for corners.
- Cars do not sense or react to other cars during normal driving. No following, yielding, overtaking decisions or inter-car braking. The faster left lane passes the slower right lane naturally.
- Steering uses a filtered look-ahead target on the assigned lane offset.
- Startup placement and stuck recovery both place cars at their lane offset.

## SDK and heading

Checked the SDK in `lua-sdk/ac_apps/README.md`:
- `ac.overrideCarControls(carIndex)` returns `ac.CarControlsInput`.
- `physics.setAICarPosition(carIndex, pos, dir)` accepts a direction vector; `nil` aligns to the AI spline.
- `physics.preventAIFromRetiring(carIndex)` keeps active AI cars from retiring when called periodically.

The current spawn and recovery code passes the recorded route's forward tangent. This matches the original RoutePilot controller at commit `ad07f8d3469a5be81e1f3efb36ba5b15e9e36f7f`; a later version had inverted this vector.

## Repository track data

`RoutePilot/line_nevadadrift_bounds.txt` on `main` contains 230 entries, each with bounds `-3.50 3.50` (7.00 m total), matching the two-lane layout.

## Validation status

- Static block and delimiter-balance checks passed for `RoutePilot.lua` and `driver.lua`.
- Confirmed the new driver contains no car-following, leader sensing or overtaking logic.
- Confirmed existing UI, route recording, bounds recording, map and world-line hooks remain present.
- Not yet run inside Assetto Corsa with CSP 0.3.2-preview2. Do not claim in-game validation. Steering response, lane-side sign, speed-hold accuracy and actual AI spawn behavior still require a live session test.

## Next runtime checks

1. Start a session with the saved 7 m Nevada Drift bounds and at least four AI cars.
2. Confirm cars appear alternately at +1.75 m and -1.75 m from the recorded line, with no reversed heading.
3. Confirm right-lane cars settle at 40 km/h and remain there.
4. Confirm left-lane cars accelerate beyond 40 km/h, brake only for route curvature and pass right-lane cars without changing lane.
5. Repeat with the 3 m width preset: all AI cars should stay on the centreline at 40 km/h.
6. Inspect `routepilot_trace.txt` for `LANE_ASSIGN`, `SPAWN_LANE`, `CAR_INIT`, `CAR=` telemetry and any `DRIVER_ERROR` entries.

Do not merge PR #4 until these runtime checks pass.
