# RoutePilot — persistent development handoff

Last reviewed: 2026-11-23
Active branch: `routepilot-traffic-coordinator`
Latest code changes: arcade pace/overtake tuning (`773605a1f32b44a9e4c01a0a0a452a7a5cd8bc6b`), explicit main-window flag (`0ec3d824cd0d2e02e3e1a3347a57a416fb4ce57a`), embedded radar child region (`35a675f9143a4c9516dda980aee4ceb0a4ca03d2`).
Main implementation: `RoutePilot/RoutePilot.lua`
Latest supplied telemetry: `RoutePilot/routepilot_trace.txt`
CSP SDK snapshot: `lua-sdk/`, especially `lua-sdk/ac_apps/README.md`.

## Product goal

RoutePilot is an arcade-style race controller for player-recorded routes, including oval and crossover layouts. It should produce high-speed, decisive, coordinated traffic rather than cautious road-following. Cars should know their paired pass/yield or crossing right-of-way decision before entering a conflict. Preserve the existing CSP app manifest, UI, route recording/loading, recovery and logger. Gearbox/transmission remain under vehicle control.

## Findings from the latest supplied trace

The latest trace is about 55 seconds and contains 275 AI samples (55 per car, five AI cars). The five-car mean of per-car average speeds is about 15.59 m/s (56 km/h); the maximum sample is 33.01 m/s (119 km/h). Mean target speed is about 28.65 m/s (103 km/h), so actual speed is far below the controller's requested target. There are 26 samples with brake > 0.05. Role samples: none=150, pass=50, yield=50, yield_player=9, meet_right=16. These role counts prove pair assignments occur, not that an overtake is completed. The trace contains emergency brake commands up to 0.75 while cars have pass/yield roles; the old collision predictor was overriding the coordinator.

The supplied `line_nevadadrift.txt` has 277 plain XYZ rows and no obstacle metadata. This confirms that no collision map had previously been generated or stored with that route.

## Current implementation changes

The current source contains the following first major arcade/traffic revision:

- Default speed setting is 240 km/h; UI range is 100–320 km/h. AI pace multipliers vary deterministically by car index (approximately 0.925–1.21), creating a meaningful speed spread and a reason to pass.
- Curvature planning uses a 24 m/s² lateral-acceleration assumption and a 26 m/s minimum corner target before pace scaling. This is intentionally arcade-oriented and must be validated against the actual recorded route.
- Drift throttle no longer collapses to 5% at high slip; at maximum slip severity it retains 72% of the calculated throttle.
- Same-direction passes can be initiated by a faster assigned pace even before a large measured closing speed develops. The pass/yield offset is 3.25 m, lateral offset response is faster, and third-car lane rejection is limited to a 9 m longitudinal window.
- Mutual reserved pairs are exempt from normal following and predictive collision braking while separated by more than 2.5 m. The emergency/predicted collision lateral envelope is tighter than the previous 2.3 m test, to reduce false braking at nearby route crossings.
- Perpendicular/crossing AI conflicts now receive deterministic `cross_go` / `cross_yield` roles. The higher-pace car (car index breaks ties) owns the crossing; the yielding car moves right and receives a reduced target speed. The reservation is retained while the pair is close.
- Coordinator transitions are logged as `TRAFFIC_PLAN`; releases of pass roles are logged as `PASS_RELEASE`. These records make assignment changes visible in the next trace.

These are code changes, not proof of successful racing. A completed pass must be established from position/progress over time, not by counting `PASS_ROLE=pass` samples.

## Static collision-map scanner

The scanner is now implemented in `RoutePilot.lua` using the SDK-documented `physics.raycastTrack(pos, dir, length, point, normal, includeNoise, backfaceCulling)` API.

- On route load, a missing or checksum-mismatched cache starts an incremental scan. It samples each route point in eight horizontal directions at three heights (0.55, 1.10 and 1.65 m), with 9 m rays and a 48-ray-per-frame budget.
- The nearest hit per route-point/direction is stored with its hit point, normal, height and distance.
- The versioned cache is appended to the route file as comment rows: `#RP_OBSTACLE_V1 version checksum count` and `#RP_OBS ...`. The existing XYZ parser ignores these records, so older route files remain readable.
- Cache reuse requires matching version and route-coordinate checksum. The UI exposes a rescan action and displays whether a map is ready or scanning.
- Cached static hit points are consulted when checking an AI passing corridor.

Important: the scanner runs inside Assetto Corsa. It cannot raycast the game world from GitHub or this development environment. The repository copy of `RoutePilot/line_nevadadrift.txt` therefore remains XYZ-only until the app runs the scan locally and the resulting runtime route file is uploaded/committed. Do not add a fake zero-obstacle header to the repository copy. After the user runs the app, inspect the log for `OBSTACLE_SCAN_FINISH` and `OBSTACLE_SCAN_SAVED`, then retrieve the generated local `line_nevadadrift.txt` to include the actual cache.

SDK limitation: `physics.raycastTrack()` tests physics surfaces. It does not guarantee detection of visible render-only KN5 meshes or objects without collision geometry. Do not claim every visible wall or pole is detected.

## Remaining work

1. Validate Lua syntax and CSP runtime behaviour. No Lua interpreter or Assetto Corsa runtime is available in this development environment.
2. Test the new high-speed profile with one AI car on the route. Check that it accelerates beyond the old 10–24 m/s range and does not oscillate or leave the line in tight corners.
3. Test a pack with at least 5 AI cars. Verify actual completed passes, not just role assignment; record pass start, target, pass side, clear/release, and route progress.
4. Test same-direction passes with a third car in the passing lane, perpendicular crossing conflicts, head-on meetings, and an AI passing the player.
5. Inspect the generated obstacle cache from Nevada. Confirm ray count, cache save/load, obstacle locations and that passing corridors are rejected only when physically blocked. If the map is empty, determine whether the route has no nearby collision surfaces or the ray geometry/heights need adjustment.
6. Review false emergency braking. The tightened collision envelope is a compromise, not a substitute for testing swept future vehicle envelopes. Never disable all third-party collision checks.
7. Continue adding plan rejection reasons, maneuver IDs, predicted meeting times and pass completion telemetry.

## SDK APIs confirmed

- `physics.raycastTrack(...)`: raycasts physics surfaces and optionally writes hit point/normal; shorter rays are faster. The SDK documents it as available even when `physics.allowed()` is false.
- `physics.setAICarPosition(carIndex, pos, dir)`: AI repositioning.
- `ac.overrideCarControls(carIndex)`: control override used for steering, throttle and brake. The app deliberately does not override gears.
- `ac.getCarColliders()` returns car colliders; it is not a documented enumerator for static track KN5 meshes.
- Stock AI spline/track-coordinate functions do not replace custom-route geometry for an oval crossover.

## Latest user test review — 2026-11-23, after route/cache upload

Current source file SHA after UI/coordinator correction: `1d92b67d6616918d67d211882b6791b68fb2f108`. README and this handoff were then synchronized in later documentation-only commits.

Reviewed branch snapshot after the user's upload commit `8d7e9930cff67fb8981ed6d553d82bc456b812a6`; the uploaded route file now contains a version-2 cache (`#RP_OBSTACLE_V1 2 209560670 452`) with 452 obstacle hits. The uploaded trace contains 800 AI telemetry samples across five cars, approximately 160 seconds of driving. Aggregate mean speed is 8.71 m/s (31.4 km/h) against mean target speed 20.93 m/s (75.3 km/h); 114 samples have brake >0.05, 322 samples have target speed <=5 m/s, 226 samples have line error >5 m, and 37 recovery successes are recorded. The log has 852 `TRAFFIC_REJECT` events, 33 `TRAFFIC_PLAN` events and 12 `PASS_RELEASE` events. No new obstacle scan occurred during this session: the version-2 cache was loaded from the uploaded route file.

Confirmed code defects from this review:

- Both the oncoming-traffic conflict block and the perpendicular/crossing conflict block were disabled by literal `if false and ...` guards. They have now been re-enabled in the latest source commit. Verify head-on and crossing behaviour carefully; the logic has not been tested in-game after reactivation.
- The map was wrapped in `ui.childWindow()`. To remove any possibility of a separate child/root surface, the latest source now draws the radar directly into the `windowMain()` context. This is the strongest way to make it part of the app's main content region. Verify the screenshot after moving/resizing the app.
- The maximum-speed slider has been removed from the UI. Internal default remains 240 km/h; no user-facing speed control should be reintroduced unless explicitly requested.
- The obstacle cache is being treated too broadly. `applyLiveObstacleGuard()` searches every cached hit in world space without first restricting hits to the route segment currently occupied by the car. `trafficLaneIsClear()` likewise scans every cached hit for each candidate point. This can let nearby but unrelated sections of an oval/crossover, or side geometry, veto a manoeuvre. Before changing thresholds, implement route-local obstacle association using each hit's `routeIndex`, current projected route progress/segment and travel direction. Preserve real nearby barriers; do not simply disable all obstacle protection.
- The live ray fan currently reduces 33 rays to the minimum distance per direction and applies aggressive speed limits. Telemetry often reports `LIVE_OBSTACLE` around 2–3 m together with target speed 4–5 m/s. Separate forward-facing hits from side-ray hits, retain hit point/normal and log the ray angle/height responsible for each speed reduction. A side wall must not cause a crawl unless the predicted swept vehicle envelope intersects it.
- The current local nearest-segment projection searches only a short forward window and two segments backward. If a car has already drifted off the route or has reversed direction, this may preserve the wrong segment. Improve projection using route continuity, velocity/heading agreement, direction sign and bounded recovery reacquisition; do not globally snap to a nearby opposite leg of the oval.
- The latest trace demonstrates severe instability: average actual speed is less than half the requested target; many samples have high line error; cars repeatedly receive low target speeds and recoveries. The prior controller tuning is not validated. Treat the user's requested behaviour as aggressive, smooth racing/drifting with decisive passing, not cautious road-following. Avoid abrupt steering sign changes and avoid using the current car heading alone to select an unrelated route segment.

## Required workflow for every future chat

At the start of every new chat about RoutePilot, read this file first, then fetch the live branch head and inspect the current `RoutePilot.lua`, `manifest.ini`, `logger.lua`, `line_nevadadrift.txt`, `routepilot_trace.txt`, and any newly uploaded screenshot/files. Do not rely on a previous assistant summary or stale commit SHA. Explicitly compare the live branch HEAD against the commit recorded here, identify user uploads newer than the last code commit, and analyze the newest trace before editing. The assistant must maintain this document as its persistent handoff: add dated notes describing each code change, discovered defect, relevant SDK/API fact, telemetry result, validation performed, and unresolved issue; update the README if user-facing behaviour changes. Commit these notes to GitHub after each meaningful work session so a new chat can resume from the exact state without asking the user to repeat the project history. Never state that in-game behaviour is fixed or tested until a new user-run test and trace confirm it.

## Development rules

Before editing, fetch the current branch head and re-read the current Lua source, manifest, logger, route file and latest trace. Work on `routepilot-traffic-coordinator` unless explicitly told otherwise. Preserve the UI, route recording/loading, recovery and logger. Verify APIs against the checked-in SDK docs. Update this handoff whenever implementation, file format, known issues or validation status changes. Never call a change in-game tested until the user has run it and supplied a fresh trace.


## Latest user test and collision-system shutdown — 2026-11-23

The user uploaded a newer trace. Current reviewed trace SHA: `f4c9a2a573f40f6026391ea5f7d950a0ef651a2a`, 2,953 lines, 505 telemetry samples across five AI cars. Aggregate sample mean speed is 10.31 m/s (37.1 km/h) versus mean target speed 23.27 m/s (83.8 km/h). There are 64 samples with brake >0.05, 181 samples with target speed <=5 m/s, 82 forward-obstacle readings below 4 m, 98 left-clearance readings below 4 m, and 91 right-clearance readings below 4 m. The log contains 2,315 `TRAFFIC_REJECT`, 76 `TRAFFIC_PLAN`, 12 `PASS_RELEASE`, and 14 `RECOVERY_SUCCESS` events. Role assignment or a release event still does not prove a completed overtake.

The map/live-guard data is not trustworthy enough to keep in control authority. In particular, the live sensor can report a clear forward ray (for example `LIVE_OBSTACLE=18`) while a side ray reports very small clearance; the old guard could reduce speed or steer based on these side-only contacts. Static map hits were also used globally in the pass corridor without validating route-local association. The user explicitly requested disabling collision detection until a correct method is established.

Latest code commit: `6f747712392299b51c65fcc465d23efd3e8e4aa3` (disable unverified obstacle sensing and inline route map). In this revision:
- `applyLiveObstacleGuard()` returns its input steering and target speed unchanged; it no longer calls `physics.raycastTrack()`.
- Static obstacle hits no longer gate pass reservations in `trafficLaneIsClear()`; the incomplete-map veto was removed as well.
- Runtime obstacle scanning is no longer called from the update loop; loading/saving routes no longer starts a scan or imports the old cache into active state.
- The scan button, collision-map status and green cached-hit markers were removed from the UI. Old `#RP_OBS` comment rows remain harmlessly parse-compatible in route files but are ignored by the controller.
- AI-to-AI route conflict checks, pass/yield reservations and head-on/crossing coordination remain enabled. This is not a global disable of vehicle-to-vehicle coordination or collision prediction.
- The route radar is drawn inline by the only manifest window callback, with a bounded canvas height. There is one `[WINDOW_...]` entry in `manifest.ini`, no `ui.childWindow()`, and no separate map window in the Lua source. Verify in-game that the map is visually inside the same app panel; CSP host window docking cannot be proven from source alone.

Do not re-enable track obstacle scanning or live raycast avoidance until the scanner is redesigned and its hits are visually validated against the track. A future implementation should classify ray hits by direction and height, validate hit points against route-local geometry and the vehicle's swept envelope, and record the exact ray/hit point responsible for every control intervention. Do not use a raw minimum distance from side rays as a forward hazard. Continue to preserve the current AI-to-AI coordination while debugging route tracking and speed control.

The newest trace also shows the overall controller remains unstable and slow; disabling obstacle sensing is a diagnostic isolation step, not a claim that speed, line tracking or passing are fixed. The user must run the new build and upload a fresh trace/screenshot before further conclusions.


## Rollback after startup regression — 2026-11-23

The attempted change from global `function windowMain(dt)` to `function script.windowMain(dt)` did not restore the app in the user's game and must not be treated as a verified fix. The source was restored byte-for-byte to the immediately preceding code revision, commit `32b30b667f390e5e3695f7a5a8d9d1e5d2e97d4b`, by rollback commit `c44a3054b0155e9f45124913c25ebbf27041e6d6`. This restores the prior global `windowMain(dt)` declaration while retaining the user's existing AI/coordinator and obstacle-sensing diagnostic changes. No further runtime or UI changes are allowed until the first CSP Lua error and an in-game result are available. Future changes must be isolated, syntax-checked, diff-reviewed, and documented; do not infer a working state from SDK examples alone.



## Arcade racing and embedded radar revision — 2026-11-23

The user confirmed that the rollback build launches and runs. The next requested direction is deliberately simple arcade racing: high pace, visible speed differences, decisive overtakes, and no timid procession. This revision keeps the proven global `function windowMain(dt)` callback unchanged; do not switch it to `script.windowMain` without a separate in-game reproduction on the user's exact CSP build.

### Embedded radar UI

- The manifest still defines exactly one `[WINDOW_...]` section and now explicitly marks it `FLAGS = MAIN`.
- The radar is rendered inside `ui.childWindow("##route_map", ...)`, a documented ImGui child region of the existing main window. It is not a second manifest window and cannot be independently dragged as a top-level app window.
- The child region contains an `ui.invisibleButton` that establishes the map canvas rectangle before drawing. Keep this item; `ui.getItemRectMin/Max()` must refer to the canvas item, not to a preceding control.
- SDK references: official Lua Apps manifest/window documentation (first `WINDOW_...` is main by default; `FLAGS = MAIN` explicitly marks it): https://github.com/ac-custom-shaders-patch/acc-lua-sdk/wiki/Lua-apps ; CSP SDK child-window examples use `ui.childWindow(id, size, function() ... end)`: https://github.com/ac-custom-shaders-patch/app-csp-defaults/blob/main/ext_controls/src/UI/windows/window_sdk.lua and https://github.com/ac-custom-shaders-patch/acc-lua-internal/blob/main/included-apps/CspDebug/CspDebug.lua.

### Arcade pace and overtaking

- `MAX_LATERAL_ACCEL`: 11.5 -> 20.0 m/s²; `LATERAL_ACCEL_LIMIT`: 17 -> 27 m/s². This raises the corner-speed plan and delays the high-slip throttle reduction.
- `MIN_CORNER_SPEED`: 9 -> 14 m/s. Tight bends no longer force the controller into the previous very low corner-speed floor.
- `BRAKE_ACCEL`: 12 -> 16 m/s²; `SPEED_GAIN`: 0.55 -> 0.85. Throttle response and the planner's braking-distance estimate are now more arcade-oriented.
- Steering response: `STEER_RATE` 2.4 -> 3.6 and `STEER_FILTER` 0.18 -> 0.14. Lane offset response: `TRAFFIC_OFFSET_RATE` 3 -> 5.
- Overtake planning horizon: 42 -> 60 m. A pass can now be requested with a real closing speed above 0.65 m/s or a deterministic pace advantage above 2.5%, provided the follower has at least the computed setup distance. The old 2.5 m/s closing-speed gate prevented most small-but-real pace differences from turning into passes.
- Existing centralized reservation, third-car lane checks, player-yield rules, oncoming/crossing coordination, and immediate-overlap braking remain in place. Track raycast/static obstacle avoidance remains disabled pending a validated implementation.

### Validation status and next test

Source-level review confirms the working global main callback and update callback were preserved, the manifest has one explicit main window, the radar is inside a child region, and the overtaking gate uses the new pace/closing-speed condition. The SDK examples confirm the child-window call shape. No Lua interpreter or Assetto Corsa runtime is available in this development environment, so syntax/runtime and actual racing performance are not claimed as tested. On the next in-game run, inspect that the map is visually contained inside the RoutePilot panel, then check that AI accelerate harder through bends and complete actual passes. Upload a fresh `routepilot_trace.txt` if they still crawl or fail to pass; inspect target speed, gas/brake, pass roles, route progress and pass completion before tuning further.
