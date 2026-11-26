# RoutePilot engineering audit — 2026-11-26

## Audited baseline

- Repository: `rerorerttt/assetto-corsa`
- Baseline: `main`
- Baseline HEAD at audit start: `a397f184e58c86016fcf8a6383c5d1b0e7b533d8`
- Working branch created from that exact `main` HEAD: `routepilot/ideal-controller-foundation-2026-11-26`
- Scope inspected: `GAME_DESIGN_DOCUMENT.md`, `README.md`, `DEVELOPMENT_STATUS.md`, `RoutePilot.lua`, `driver.lua`, `route_geometry.lua`, `racing_line.lua`, `logger.lua`, and Advanced Gamepad Assist's `assist.lua`, `AGALib.lua`, and `CarPerformanceData.lua`. The repository's bundled CSP Lua SDK definitions were also checked for the car, wheel, and matrix fields used by the new telemetry module.

This is a source audit. No Assetto Corsa session has been run as part of this audit, so no handling, FPS, or collision-free behaviour is claimed as runtime-verified.

## What is already present

- Route recording, per-track save/load, boundary recording, map/world-line display, AI adoption, spawn positioning, and stuck-car rescue are in `RoutePilot.lua`.
- `route_geometry.lua` builds a Catmull–Rom route, samples it by approximate arc length, and projects cars onto a continuity-constrained route segment.
- `racing_line.lua` contains a short-horizon dynamic-programming offset search. It evaluates curvature and smoothness while applying the recorded corridor bounds.
- `driver.lua` combines path following, speed planning, traffic detection, pass reservation, overtake state, throttle/brake control, telemetry, and recovery. The current path follower uses pure-pursuit curvature with a curvature feed-forward term and additional boundary/slip corrections.
- The driver learns rough per-car braking, acceleration, and lateral-acceleration limits from observed motion. These estimates are useful inputs, but they are not yet a calibrated tyre/vehicle model.

## Main architectural problems

1. **The vehicle controller is not yet a vehicle-dynamics controller.** Before this branch, yaw rate was inferred from the change in the car's look vector, while slip angle was inferred from the velocity of the car's reference point. This misses the different velocity directions at the front and rear axles, which are essential during drift and countersteer.
2. **The AGA control law cannot be copied directly.** Advanced Gamepad Assist is a human-input assist. It calibrates steering from a gamepad, processes the player's input, and explicitly skips AI-controlled cars. Its useful transferable foundation is the measured front/rear axle velocity, load-weighted tyre slip, target front slip, steering-lock awareness, self-steer and yaw damping. RoutePilot needs to turn those measurements into an autonomous path-tracking controller, not add AGA's output blindly to the current steering command.
3. **The racing-line objective is incomplete.** The current optimiser primarily minimises geometric curvature and lateral-offset variation. It does not yet solve a complete time-optimal trajectory with a coupled speed/braking profile, tyre-force limits, and vehicle-specific response.
4. **Traffic avoidance and racing-line optimisation are coupled too loosely.** The optimiser converts nearby traffic into blocked lateral intervals at sampled stations. If all sampled candidates are blocked, its fallback still inserts an edge candidate; that fallback needs to be made explicitly safe rather than assuming the edge is clear.
5. **Overtaking does not follow the design document's discrete lane rule.** `findPassingOffset()` searches lateral offsets in 0.30 m increments and ranks arbitrary offsets. The GDD specifies a fixed set of adjacent logical lanes, not a continuous search for a convenient gap. This needs to be replaced with lane-slot selection and whole-manoeuvre reservation.
6. **Route-loss handling is too abrupt.** When projection is lost, the current driver sets gas to zero and brake to full. The GDD instead requires controlled continuation on a verified last-safe path where possible, followed by cautious reacquisition; a hard stop is only appropriate when no safe continuation exists.
7. **The boundary fallback needs a clear data-validity state.** If a bounds file is absent or mismatched, `loadBounds()` creates a default corridor from the width preset. The GDD says not to invent road boundaries. The application must distinguish measured/saved bounds from a user-confirmed width assumption and must not silently treat an assumption as recorded geometry.
8. **The control and traffic logic are concentrated in one large driver module.** Route progress, trajectory selection, obstacle checks, overtaking, stability, speed control, and recovery share mutable per-car state. This makes regressions likely and makes it difficult to prove that a new behaviour cannot invalidate another one.
9. **The speed planner is curvature-based, not yet time-optimal.** It estimates speed limits from curvature and braking response, but the path and speed profile are not optimised together. The target is a forward/backward feasible speed profile over the chosen path, with measured braking/acceleration and conservative tyre-force use.
10. **Performance has not been demonstrated at the GDD target.** The optimiser and traffic prediction need measured per-frame/per-car timings on long routes and larger grids. Do not remove safety checks to improve FPS; cache shared route data and update each car's plan at controlled intervals instead.

## Advanced Gamepad Assist: what to reuse

AGA's `getVehicleData()` computes front and rear axle velocities from the car's local linear/angular velocity and the physical positions of the wheel hubs. It also calculates load-weighted front/rear tyre slip angles and normalised slip. Its steering correction uses rear-axle velocity direction for self-steer, yaw-rate damping, front-axle slip limits, and a countersteer-aware transition. Its car-performance module estimates a target front slip angle from tyre data and learns it from observed behaviour.

These are valuable measurements and principles. The complete AGA function is not a drop-in AI controller: it depends on gamepad calibration, user input, AGA-specific smoothing state, and player-only update flow. RoutePilot must calculate a desired path/yaw response first, then use measured axle and tyre state to regulate the steering command without fighting that target.

## First implementation step on this branch

Added `vehicle_dynamics.lua` and connected it to `driver.lua`. It measures, per car:

- front and rear axle velocity angles from actual wheel-hub positions transformed into the car's physics-local frame;
- front/rear tyre slip angles and normalised slip, weighted by wheel load;
- local yaw rate, wheelbase, and steering-lock metadata.

The module is measurement-only. It does not change steering, throttle, braking, path selection, overtaking, or recovery. New fields are appended to the existing telemetry as `AXLE_VALID`, `FRONT_AXLE_RAD`, `REAR_AXLE_RAD`, `FRONT_TIRE_DEG`, `REAR_TIRE_DEG`, `FRONT_NDSLIP`, `REAR_NDSLIP`, `YAW_RATE_RAD`, `WHEELBASE_M`, and `STEER_LOCK_DEG`.

The module uses fields verified in the repository's CSP SDK definitions: `ac.StateCar.transform`, `worldToLocal`/matrix point transforms, `localVelocity`, `localAngularVelocity`, `steerLock`, and `ac.StateWheel.position`, `load`, `slipAngle`, and `ndSlip`. It follows AGA's physical calculation `v(point) = v(CG) + omega × r`. No AGA steering correction has been enabled yet.

## Implementation order

1. **Validate the new measurements first.** In-game logs must show finite, correctly signed front/rear axle angles, yaw rate, tyre slip, and wheelbase on straight running, steady corners, and controlled drift. Confirm the values for several different cars. No steering output changes until this is verified.
2. **Build a standalone autonomous vehicle controller.** Separate path tracking from stability regulation. Calculate a desired curvature/yaw response from the chosen trajectory; regulate it using axle velocity direction, yaw rate, measured tyre slip, steering lock, and a per-car learned response. Use bounded corrections and explicit saturation/anti-oscillation logic.
3. **Validate single-car control before traffic.** Test high-speed straight-line stability, fast bends, hairpins, S-bends, braking, acceleration, and controlled drift. Compare steering, yaw, axle slip, lateral error, and speed against the baseline.
4. **Replace the current racing-line heuristic with a coupled path/speed optimiser.** Use the actual corridor and full vehicle footprint, smooth curvature and curvature rate, tyre-force limits, braking distance, and a forward/backward feasible speed profile. Plan multiple bends together.
5. **Replace continuous-offset passing with discrete lane slots.** A traffic coordinator must reserve the lane and space-time corridor for the full lane change, side-by-side pass, and return. Conflicting requests are arbitrated deterministically.
6. **Make route-loss and recovery state machines explicit.** Keep the last safe path only while its corridor remains verified. Otherwise transition to a bounded emergency path/braking state, then reacquire route progress using distance, heading, and topology checks.
7. **Benchmark and regression-test.** Measure CPU time and FPS with 1, 5, 10, and 20+ cars, long routes, intersections, narrowings, and dense traffic. Keep the existing record/save/load and teleport recovery mechanisms unless a measured defect requires a focused change.

## Acceptance rule

A source change is not a successful driving change until the user tests it in Assetto Corsa and supplies the resulting trace. Static inspection can establish API usage, state transitions, and code structure; it cannot establish that the car handles correctly or that traffic is collision-free.
