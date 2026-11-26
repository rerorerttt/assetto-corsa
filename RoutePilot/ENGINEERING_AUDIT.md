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

## Follow-up analysis of the user's Drag and traffic traces

The Drag trace supplied after the first implementation shows a distinct steering-feedback failure. At approximately 16.4 s the car is still near 46 m/s with a small steering command; at 17.4 s the yaw rate and axle slip begin growing. By 18.4 s the route curvature feed-forward is only about 0.0023, but the applied steering is 0.120. It then alternates to -0.213, +0.322, -0.408 and +0.524 while the car departs the recorded corridor. The source explains the discrepancy: `calculateSteering()` added `p.slipAngle * 0.18` directly to the path-following steering output. This is not a yaw-rate damping controller and can feed an emerging sideslip back into steering. That direct term has now been removed. The path follower still steers from its preview geometry and the boundary controller; no unvalidated axle-based correction has been enabled.

The same trace shows a further diagnostic limitation: the old telemetry logged steering curvature and filtered steering but not the raw steering target, body slip angle, or estimated yaw rate. The trace now includes `STEER_TARGET`, `SLIP_ANGLE_DEG`, and `YAW_EST_RAD`, in addition to the previously added axle measurements, so the next run can separate path-following demand from stability feedback.

The supplied traffic trace contains five `OVERTAKE_BEGIN` events but only three matching `OVERTAKE_END` events. Two passes remain active at the end of the trace. The code also declared `PASS_TIMEOUT=8.0` but never used it, and an active pass had no blocked-progress exit. The pass state now tracks a deadline scaled to the estimated pass distance and a sustained near-zero-progress timer. It aborts and returns to the original lane only after that lane passes the live occupancy check; otherwise it retains the safety hold rather than cutting across another car. Aborts are logged as `OVERTAKE_ABORT reason=blocked|timeout`, with `PASS_BLOCKED` and `PASS_TIME` in telemetry.

These are source-level corrections, not proof of in-game stability. The next Drag run should confirm that steering remains close to the path-following demand on the straight and that yaw rate/slip no longer grow into alternating full corrections. The next traffic run should show every `OVERTAKE_BEGIN` ending in either `OVERTAKE_END` or a safe `OVERTAKE_ABORT`; an abort can still wait if the original lane is occupied.


## 2026-11-26 full source audit and straight-line stability correction

The user's report confirmed that removing the direct body-slip steering term did not remove the oscillation. A broader audit against GAME_DESIGN_DOCUMENT.md identified additional control-loop defects that the previous change did not touch:

- Pure-pursuit curvature was added to a second feed-forward term from the same planned path. This can double-count the bend and over-command steering. The duplicate term has been removed; pure pursuit is now the sole path-curvature demand.
- The racing-line optimizer could change its selected lateral offsets at each replan without any cost for departing from the previous plan. A temporal path-continuity penalty now discourages plan-to-plan lateral jumps.
- The preview used a plan-origin advance clamped to 30 m, while a car at 500 km/h can travel about 90 m in 0.65 s. This could make the controller aim at a stale point behind the car. Plan advance is now used in full, the planner refresh interval scales with speed to keep the plan origin within 20 m, and samples beyond the plan horizon return nil rather than silently clamping to the final point.
- The old fixed 28 m maximum preview was too short at extreme speed. The speed-scaled preview now reaches 85 m.
- Emergency passing could apply a 180 ms full-lock steering pulse and multiply steering gain by 2.6. Both interventions have been removed. Passing uses the same bounded steering authority as normal path following.
- Passing offset selection used arbitrary 0.30 m candidates rather than the GDD's discrete lane slots. It now selects among 1/2/3/4 logical slots for 3/7/11/15 m corridors.
- Passing transitions previously used a fixed 6/22 m path regardless of speed. The transition now expands with speed and lateral-acceleration allowance; a pass is rejected if the transition cannot be completed before the target.
- When traffic blocked every candidate offset, the optimiser manufactured an edge candidate even if it remained inside a blocked interval. It now fails closed with NO_PATH; the driver does not convert this traffic-only result into an unconditional full stop. A blocked-plan retry is rate-limited rather than recalculated every frame.
- The old STEER_DEAD constant was declared but unused. It now suppresses only tiny steering residue when cross-track, heading and lateral-velocity errors are already settled.
- p.laneOffsetCurrent previously recorded the preview offset as if it were the car's current path offset. It now samples the plan at the car's current progress, separating current lane occupancy from the future preview target.
- Unused stability calculations and the unused local racing-line cost helper were removed. The axle/yaw/slip measurements remain for diagnostics; no unvalidated axle countersteer law has been enabled.

The current steering law still uses a fixed curvature-to-normalized-steer gain. The repository SDK exposes steering-lock metadata, but the relationship between that value and front-wheel steering angle is not sufficiently established here to replace the gain with a bicycle-model conversion safely. That calibration remains an explicit limitation, not a guessed formula.

## GDD compliance findings still open

- RoutePilot.lua still silently creates default symmetric bounds when a bounds file is absent, and addPoint() initializes new points with the default width preset. This conflicts with GDD §4.2 unless the preset is treated as an explicit user-confirmed corridor. Existing route files and user-selected presets must be preserved while this is corrected.
- The racing-line optimiser remains a geometric short-horizon curvature/smoothness optimiser, not the GDD's full time-optimal coupled path/speed solver.
- Traffic arbitration is still distributed through per-car update state rather than a dedicated deterministic coordinator. Discrete lane slots are now used, but a full atomic space-time reservation coordinator remains unfinished.
- Route-loss handling still applies immediate full braking. The GDD calls for controlled last-safe-path continuation and cautious reacquisition where verified.
- Per-frame performance has not been benchmarked with 1/5/10/20+ cars. The speed-scaled replan interval increases planner work at very high speed and must be profiled in CSP.

Static checks only. No Lua/LuaJIT compiler or Assetto Corsa/CSP runtime is available in this environment. None of the steering, high-speed preview, lane-transition, collision-avoidance or FPS changes is claimed as in-game verified. The next test must begin with one car on a genuinely straight Drag route, then repeat at increasing speed, and only after that test corners and traffic.


## 2026-11-26 GDD cross-check: planner failure versus physical blockage

The GDD's governing constraints are explicit: planner errors must not strand cars on a passable road; the car should continue on a verified safe path where possible; and the full vehicle footprint must remain inside confirmed road bounds. The supplied trace showed the previous implementation conflated two different conditions:

- `DP NO_PATH`: the dynamic-programming candidate graph did not contain a complete sequence of accepted candidates;
- `NO_SAFE_PATH`: no tested path, including a deterministic fallback, satisfies the full-body road-boundary checks.

Those conditions are not equivalent. The driver previously treated the first as the second and forced target speed to zero.

The current correction separates them in OPEN mode by trying the current lateral corridor and then the route-centre corridor through the existing full-body-validated planner. The fallback does not bypass road bounds, does not remove body-sweep checks, and does not apply during PASS, RETURN, YIELD or TRAFFIC_HOLD where changing the reserved corridor could create a traffic conflict. Only failure of the permitted geometry-validated paths leaves `racingPlanBlocked=true`.

This closes the specific false-stop path identified in the supplied trace at source level. It does not close the broader GDD route-loss requirement: projection loss still invokes the existing immediate brake fail-safe pending a separately validated last-safe-path/reacquisition state machine. It also does not establish a globally time-optimal racing line; the current optimizer remains a local short-horizon approximation.

Static validation only. The new driver source was fetched back from GitHub after commit. No Lua compiler, Assetto Corsa session, CSP runtime, or FPS benchmark was available. In-game acceptance must verify that fallback events occur on passable OPEN road, that hard stops remain when the full-body corridor is genuinely infeasible, and that PASS/YIELD/RETURN reservations are unchanged.
