# RoutePilot racing trajectory planner — design and diagnosis
Date: 2026-11-25
Branch: routepilot/multi-lane-emergency-evasion-2026-11-25
Scope: replace the current single-offset swept-path planner with a receding-horizon racing trajectory planner. Lua/CSP only; do not use native AI fastline/spline commands or teleportation as driving.

## Diagnosis from the current branch

The current root app is RoutePilot 0.10.0. `driver.lua` already uses Catmull–Rom arc-length geometry and pure-pursuit curvature steering, but its path search is not a true trajectory search. `calculateSteering()` chooses one terminal lateral offset and evaluates a single quintic transition from the current offset to that value, then holds that offset. It validates the path over at least `PLAN_HORIZON=170 m` at 2 m intervals. Candidate offsets are generated from the union of local bounds over the whole horizon, not from a sequence of locally feasible lateral choices.

When all candidates fail the boundary test, the driver sets `boundaryInfeasible=true`, clears the path, commands `gas=0` and `brake=1`, and returns zero steering. `Driver.update()` then forces `targetSpeed=0`. There is no shorter-horizon retry, no multi-knot alternative, and no recovery path that preserves forward motion while remaining inside the hard boundary envelope. Thus a future infeasible constant-offset path can stop a car now, even when a nearby free corridor exists. This is the confirmed source-level deadlock mechanism; it is not evidence that a physical obstacle occupies the road.

The current root trace contains 1,327 lines and 205 one-second telemetry samples for each car 0–5. AI average sampled speeds are: car 1 9.2 m/s, car 2 11.0 m/s, car 3 11.0 m/s, car 4 8.2 m/s, car 5 13.7 m/s. The corresponding samples with `PLAN_FAIL=1` are 11, 14, 27, 63 and 24; samples below 1 m/s are 36, 12, 23, 57 and 23. Car 4 is the clearest failure: 63/205 planner-failure samples and 57/205 near-stationary samples. Failure rows show `PLAN_REJECT=13/0`, `PLAN_CLEAR=0.00`, `GAS=0`, `BRAKE=1`. These measurements agree with the source-level failure path.

The AI can detect the manually driven player: the trace contains `CAR=4 OVERTAKE_BEGIN target=0` at 200.50 s and `OVERTAKE_END` at 208.64 s. Therefore player detection is not wholly absent. It is unreliable as a practical behavior because the path planner can fail while trying to execute the selected pass. Manual player controls remain intentionally untouched; car 0 is only driven by RoutePilot after native AC Ctrl+8 marks it AI-controlled.

The speed issue is separate from the deadlock. The current speed controller follows the recorded route curvature and the selected offset path, with a conservative initial lateral-acceleration target of 6 m/s² and a measured grip estimator. A trajectory planner must select a path first, then derive a speed profile from that path's curvature and measured braking/grip. Raising a global speed cap or throttle gain before this separation is correct would reproduce boundary departures.

## Target architecture

### 1. Frenet state and route reference

Keep the recorded Catmull–Rom route as the reference geometry, not as a sequence of steering targets. Project each car to continuous route progress (s), signed lateral offset (d), heading error, longitudinal speed and lateral velocity. Keep projection continuity and heading-aware reacquisition. Bounds remain the hard physical corridor and must be transformed consistently into the spline's local normal frame.

### 2. Receding-horizon path lattice

Replace the one-terminal-offset family with a set of continuous paths (d(s)) made from several lateral knots. Plan over a time/distance horizon of roughly 2.5–4 seconds, with a minimum useful preview and a distance that expands with speed; do not reject the current path merely because a constant offset is infeasible 170 m away. Use stations spaced about 3–5 m and lateral samples about 0.25–0.4 m apart. Candidate paths must include:
- continuation of the current path/racing line;
- gradual moves toward any locally free part of the corridor;
- passing paths on either side of a blocker;
- return paths only after the passed vehicle is fully clear.

Connect knots with quintic segments (continuous position, slope and curvature). Search sequences using dynamic programming/beam search rather than enumerating only one final offset. Penalize lateral jerk, curvature, curvature change, unnecessary lateral movement and deviation from the desired racing line. Reward forward progress and paths that permit higher speed. Do not hard-code left/right lanes.

### 3. Space-time obstacle prediction and cooperative reservations

Treat every other car, including the manual player, as a moving body with length and width. Predict its near-term position using measured velocity/heading and, when available, its current RoutePilot path. Check candidate ego paths against the other cars' predicted swept footprints over time, not only against static lateral offsets or a single lane-occupancy test. Include rear traffic and opposite-direction traffic.

AI cars must share a small route-level coordinator. Each car submits its best few candidate paths; resolve conflicts deterministically using time-to-conflict, urgency/closing speed, progress and a stable tie-break. Reserve only the conflicting space-time tube ((s,d,t)), not an entire lane or a distant section of road. If a reservation conflicts, try another candidate immediately. Independent free corridors must remain usable concurrently. A stopped player is an obstacle to route around, not a reason to stop the whole traffic stream.

### 4. Speed profile from the selected trajectory

For every selected path, calculate curvature along the path and a local speed ceiling from available lateral grip. Estimate usable grip from the car's observed response and wheel slip; do not leave 6 m/s² as a permanent universal ceiling. Run a backward braking pass from upcoming corners/obstacles and a forward acceleration pass using the car's measured acceleration and braking capability. Respect a friction ellipse so longitudinal acceleration is reduced only to the extent required by lateral demand. No arbitrary 170 km/h cap and no speed reduction for cars in a physically separate corridor.

The controller should track the selected continuous path with pure-pursuit curvature plus curvature feed-forward and bounded steering-rate filtering. Throttle should follow the resulting speed profile; traction control should intervene on measured wheelspin, not become a permanent low-speed governor.

### 5. No-deadlock fallback

Planner failure must not immediately mean `gas=0, brake=1, steer=0`. Retry in this order: (a) retain/revalidate the current path over a short emergency horizon; (b) shorten the planning horizon and rebuild candidates; (c) reduce the requested speed enough to make a boundary-safe path feasible; (d) use the safest locally feasible path while logging the exact reason. Full braking is reserved for a predicted imminent collision or a genuine inability to remain within the hard boundary envelope. Never cross the saved bounds, never use blind full-lock steering, and never teleport as a substitute for normal driving. Replan every update while keeping the chosen path stable enough to avoid left-right oscillation.

A car is considered blocked only when its speed/progress indicates no motion and a predicted vehicle conflict actually occupies its reachable corridor. A planner bookkeeping failure or a free-road candidate rejection must not be classified as traffic blockage. Rescue is a last resort after the existing timeout and only at a validated clear position.

## Required telemetry

Log one concise planner decision per car at a lower rate than the physics update, with:
- planning horizon, number of generated/valid candidates and selected candidate ID;
- rejection counts separated into boundary, vehicle collision, reservation, curvature/steering feasibility and route projection;
- minimum body-to-boundary and body-to-car clearances, with the sample distance/time where each minimum occurs;
- selected path's lateral knots, maximum curvature, curvature rate/jerk and speed profile;
- speed-limit reason (corner, braking, obstacle, grip, projection or fallback);
- TTC and identity of the blocking vehicle, if any;
- fallback tier and whether the road was actually blocked.

This must distinguish "no path because of a wall", "no path because of another car", "bad projection", and "planner bug". The present `PLAN_FAIL` boolean is insufficient.

## Validation gates

1. Empty route with one AI: continuous lap, no unexplained stops, no boundary-footprint violation; establish baseline speed profile.
2. Stationary player on the route: every AI must pass on any genuinely free side, including close-range starts; no AI queue if a free corridor exists.
3. Two AI cars approaching the same blocker: coordinated, sequential or spatially separated passes without deadlock.
4. Opposite-direction traffic on the oval: predict closing speed and resolve passing space-time conflicts.
5. Narrow sections and high-curvature bends: full vehicle footprint remains within saved bounds; speed changes occur before the bend, not after entering it.
6. Six or more AI cars: no cascading reservation deadlock; free sections remain at racing pace.

Release criteria: zero boundary departures and collisions in the test set; zero persistent stops on a free reachable road; no full-brake planner-failure loop; completed passes recorded with start/end events; average speed and lap time reported alongside safety metrics. Source-level checks are not runtime validation. Do not call the planner safe or racing-ready until it has been tested in Assetto Corsa/CSP and the resulting trace has been reviewed.

## Non-negotiable constraints carried forward

- Lua/CSP app only; do not switch to native AI fastline/spline control.
- No teleportation as a driving method.
- Never let any part of the vehicle reach or cross the recorded road boundary.
- No blind full-lock pulse or emergency bypass of the hard boundary guard.
- No artificial global speed cap; maximize speed subject to the selected trajectory, grip and braking distance.
- AI must route around the player and other cars whenever a physically safe corridor exists.
- Preserve the known-good baseline and keep experimental work isolated on this branch.


## Implemented first correction — 2026-11-25

The first runtime-code correction is committed in `driver.lua`.

- Removed the requirement that one constant lateral target offset must be valid across the old 170 m horizon.
- Candidate lateral offsets are now generated from the current vehicle-footprint-safe interval. Each candidate is still checked against the recorded left/right boundaries over every sample of its chosen horizon; the change does not relax the hard boundary check.
- Added receding-horizon retries: the planner tries a speed-dependent horizon first (24–90 m), then 65, 45, 30, 20 and 14 m. A distant infeasible section therefore cannot by itself force an immediate stop when a shorter safe trajectory exists.
- Added a denser local lateral candidate lattice and telemetry fields `PLAN_HORIZON` and `PLAN_FAIL_REASON`.
- Reordered the update pipeline so steering/path validation happens before path-speed calculation, and overtake logic consumes the current frame's lane offset rather than the previous frame's value.

This is a targeted deadlock correction, not yet the complete multi-knot space-time racing planner described above. Moving-vehicle trajectory prediction, coordinated time-space reservations, multi-knot lateral beam search and calibrated grip/speed optimization remain outstanding. The Lua code has been committed, but no Assetto Corsa/CSP runtime test has been performed in this environment. The release gates above remain mandatory.
