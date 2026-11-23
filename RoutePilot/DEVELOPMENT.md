# RoutePilot development state

Updated: 2026-11-24  
Working branch: `routepilot/native-ai-fix-2026-11-24`  
Target CSP: `0.3.2-preview2`

## Primary requirement

A fast racing field that follows the recorded route, stays inside the saved corridor, overtakes cleanly, does not weave without a reason, avoids contact with other cars and the player, and recovers only when genuinely stuck. Faster cars may pass slower cars; the slower car must not be instructed to brake, move aside, yield or reduce speed for a pass.

## Repository material reviewed

- `lua-sdk/ac_apps/README.md`: CSP physics API contracts for custom AI splines, car placement, offsets, caution, aggression and track raycasts.
- `AISplineEditor/AISpline.lua` and `AISplineEditor/AISplineEditor.lua`: version-7 spline binary layout, payload indexing, surface snapping and the forward-vector coordinate frame used by side limits.
- `Verve/Verve.lua`, `Verve/lib/racecraft.lua`, `Verve/lib/recovery.lua`: native AI racecraft patterns, spline-offset usage, AI-awareness preservation, recovery limitations and CSP API usage.
- `comfy_map/comfy_map.lua`: map rendering and UI composition patterns. RoutePilot's map is drawn directly inside `windowMain`; it is not a separate floating map window.
- `RoutePilot/RoutePilot_Back.lua` on `feature/arcade-traffic`: previous manual-control implementation using `ac.overrideCarControls`. It is retained as a historical reference, not loaded by the native-AI app.
- `RoutePilot/Задачи грок.txt`: numbered requirements and subsequent corrections.
- Current `RoutePilot/routepilot_trace.txt` and `line_nevadadrift.txt`: post-change stationary-car trace and route-height anomalies.

## Confirmed SDK contracts

- `physics.setAISpline(carIndex, filename, aiHintsFilename)` loads a custom spline in the background and returns a boolean.
- `physics.setAICarPosition(carIndex, position, nil)` aligns the car along its assigned AI spline.
- `physics.setAISplineAbsoluteOffset(carIndex, offset, overrideAiAwareness)` uses a lateral offset in metres; negative is left and positive is right. Keep `overrideAiAwareness=false`.
- `physics.setAIAggression` accepts 0–1; the SDK says 0.95 corresponds to 100% aggression after the launcher's multiplier.
- `physics.setAICaution` accepts 0–16, with 1 as the default.
- `physics.raycastTrack` returns a hit distance or -1 and can write the hit point and normal into supplied vectors.
- `physics.setAIStopCounter(carIndex, 0)` clears CSP's post-incident AI brake wait; `physics.setAINoInput(carIndex, false, false)` restores normal AI input after a teleport.
- `physics.setAIThrottleLimit(carIndex, 1.0)` removes a throttle cap; `physics.setAITopSpeed(carIndex, 1e9)` removes an artificial top-speed cap.
- No permanent `fast_lane.ai`, `ac_start` or `ac_time` files are required for this runtime custom-spline path. RoutePilot writes a version-7 spline and passes its filename to `setAISpline`.

## Root causes found in the latest main-branch test

1. `spline_builder.lua` wrote `payload.gas = 0.0` and `payload.brake = 0.0` for every point. The trace shows cars moving before spawn and remaining at 0 m/s after placement on the custom spline. The new builder derives throttle/brake hints from the curvature-limited speed profile.
2. The builder multiplied speed by 3.6 before writing the version-7 payload. The speed channel is represented in m/s; the new builder writes m/s and converts the UI's km/h limit only once.
3. Surface snapping previously cast from 10 m above a route point for 20 m and did not check the raycast return value. On multi-level geometry this can hit the wrong deck or leave a bad height untouched. The new builder casts from 1.5 m above the recorded car position for 8 m, validates distance/normal/height delta, interpolates failed samples and aborts if more than 40% of samples fail.
4. Route projection used X/Z only, which is ambiguous on bridges and underpasses. It now includes world height.
5. Previous lane assignment placed the fastest group in the left lane and slower groups to the right. That blocks the intended passing model. Cars prefer the rightmost valid lane; if the player occupies that corridor, they choose the next safe lane to the left. If no lane fits clear of the player, they remain on the recorded centreline. Only a faster follower can reserve the adjacent left lane, pass, and return after 15 m of clearance. The player's actual position remains an obstacle during pass and return checks.

## Current code layout

- `RoutePilot.lua`: UI, recording, persistence, route projection, map and 3D rendering, spawn/recovery orchestration.
- `spline_builder.lua`: version-7 binary spline writer, surface validation, speed profile and pedal hints.
- `driver.lua`: native AI assignment, right-lane cruise, left-side passing reservation, telemetry and stuck recovery.
- `logger.lua`: trace file management.
- `RoutePilot_Back.lua`: legacy reference only; do not load alongside the new app.

## Current implementation

- The version-7 binary structure matches `AISplineEditor/AISpline.lua`; the point and payload arrays retain the same structure and ordering.
- Speed hints are generated from route curvature. The UI speed limit is in km/h, internal speed is m/s, lateral-acceleration target is 10.5 m/s², and the braking envelope uses 12 m/s².
- Throttle/brake hints are derived from the change in squared target speed between adjacent samples: full throttle on acceleration/constant-speed sections and proportional brake hints where the planned speed falls.
- Route samples are raycast locally from just above the recorded car position. Failed samples are interpolated in height only; X/Z are preserved. The route is rejected if too many surface samples fail.
- Side-width magnitudes are mapped to the same forward-vector frame used by AISplineEditor, including asymmetric recorded boundaries.
- Route matching includes world height so cars on overlapping track levels do not get projected onto the wrong deck.
- Cars prefer the rightmost valid lane, but the player's actual occupied corridor takes priority; if it blocks the right lane, AI chooses the next safe lane left or stays on the centreline if no lane fits. Only a faster follower can move left; the leader is never commanded to yield or slow. A shared reservation allows one active pass at a time. The pass begins with a 120 m trigger horizon and the follower does not return until at least 15 m ahead and the home lane is clear.
- The map, lane selector, route/boundary recording controls, clear button and 3D line preview are already in the main RoutePilot window. Collision-mesh discovery/recording is deliberately disabled; the saved corridor and native CSP awareness are the safety mechanisms.
- Spawn and recovery use `physics.setAICarPosition(index, position, nil)`, then clear the AI stop counter and parked-input state. Stuck recovery searches forward for a free route position before teleporting.

## Current changes on this branch

- Fixed spline speed units and replaced the all-zero pedal profile with curvature-derived throttle/brake hints.
- Added local track-surface snapping, hit validation and height interpolation.
- Corrected asymmetric side-width mapping to the AISplineEditor coordinate frame.
- Made route projection height-aware for multi-level tracks.
- Replaced pace-sorted default lanes with a right-lane cruise / left-lane passing policy.
- Increased pass lookahead and occupancy horizon for high-speed traffic, increased post-pass clearance and kept a single pass reservation and made initial lane assignment/player occupancy checks aware of the player's actual position.
- Applied CSP's documented aggression value (0.95) and the default AI caution (1.0) for safe following distance; removed artificial throttle/top-speed caps and reset CSP's post-teleport stop/parked-input state.
- Updated `README.md` and this file with the new model and validation caveat.

## Required in-game validation

This branch has not been run inside Assetto Corsa from this environment. Do not treat the implementation as proven until these checks pass on the user's CSP 0.3.2-preview2 install:

1. Start with 3–5 AI cars and confirm each accelerates after placement. In the trace, `AI_SPLINE_ASSIGNED` must be followed by increasing `SPEED`, not repeated `SPEED=0.00`.
2. Check `AI_SPLINE_BUILD_OK`: `surfaceFallbacks` should be low and no `AI_SPLINE_BUILD_FAILED` should occur. Inspect the rendered line at elevation changes and track crossings.
3. Confirm every car receives the rightmost home lane. The passing car should log `PASS_START`, move left smoothly, pass without changing the leader's controls, reach the clearance threshold, then log `PASS_COMPLETE` and `PASS_RELEASE`.
4. Check the 3D line and both boundary lines at walls, poles, kerbs, bridges and underpasses. A saved corridor is only as safe as its recorded limits; no automatic wall/column collision map is implemented.
5. Inspect wheel loads, route error, speed, rescue frequency and contacts in a replay before increasing traffic density.

A zero-collision guarantee cannot be made for arbitrary mod cars and track meshes. If any check fails, use the new trace to isolate the failing layer before changing the driving model again.
