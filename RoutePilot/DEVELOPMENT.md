# RoutePilot — persistent development handoff

Last reviewed: 2026-11-23
Active development branch: `routepilot-traffic-coordinator`
Reviewed HEAD: `62fc5f5901c91f1408ea1335b4e13b12e2d1ab38`
Main implementation: `RoutePilot/RoutePilot.lua`
Latest reviewed telemetry: `RoutePilot/routepilot_trace.txt`
CSP SDK snapshot in this repository: `lua-sdk/` (especially `lua-sdk/ac_apps/README.md`).

## Product goal

RoutePilot is an arcade-style, high-energy AI race controller that follows a player-recorded route. Cars should drive continuously and quickly, make decisive overtakes, coordinate passing/yielding before manoeuvres, and safely handle opposite-direction traffic on oval/crossover layouts. Reliability and fun take priority over realistic hesitation. Do not turn the pack into slow road traffic. Preserve the working app UI, route recording/loading, recovery, logging, and CSP app structure.

## Current implementation facts

- RoutePilot 0.4 controls steering, throttle and brake using `ac.overrideCarControls`; gearbox/transmission are intentionally left to the car.
- Route points are saved as plain XYZ rows; the existing parser ignores non-XYZ rows, which permits backward-compatible metadata/comment records if the format is deliberately versioned.
- `updateTrafficCoordinator()` builds one snapshot, tries to reserve same-direction pass/yield pairs and oncoming pairs, then publishes offsets.
- `trafficLaneIsClear()` currently rejects a planned lane if any third car is within 28 m longitudinally and within the lateral clearance. With a starting grid gap of 22 m this can reject most useful passes before they begin.
- Same-direction pass initiation currently requires closing speed > 1.2 m/s or a stopped leader. All cars share the same configured cruise speed (140 km/h), so a pack following the same curvature often has no reason to initiate a pass.
- The reviewed baseline used a 1.35 m pass offset against a 2.30 m lateral following/collision corridor, so a car could remain inside the same-lane envelope while attempting to pass. HEAD `62fc5f5` raises the shared traffic offset to 2.65 m and reduces the third-car lane rejection longitudinal window from 28 m to 18 m. These are first-pass tuning changes and still require in-game validation.
- HEAD `62fc5f5` raises steering response, relaxes early slip-based throttle suppression, raises curvature/braking acceleration assumptions, increases default speed limit from 140 to 180 km/h, and gives odd/even AI indices different pace multipliers (1.08 / 0.96). The pace multiplier affects both cruise cap and curvature target, creating deliberate speed spread so faster followers have a reason to pass. These are arcade tuning values, not verified safe limits.
- `carAhead()` combines same-direction following and predicted collision avoidance. A valid pass must be exempted from following/braking for the whole reserved manoeuvre, while true third-party and oncoming collision checks remain active.
- `planTargetSpeed()` scans 40 route vertices and limits speed from curvature using `MAX_LATERAL_ACCEL = 5.5` m/s² and `BRAKE_ACCEL = 5.0` m/s². This is conservative for the requested arcade race feel; the speed planner needs a deliberate high-speed/arcade profile rather than simply raising throttle.
- Baseline telemetry at `a84ee9a` has 977 lines, 951 car samples: roles none=761, pass=85, yield=85, yield_player=20; 117 samples with brake > 0.05; 307 with gas < 0.2; 65 with target speed < 1 m/s; maximum logged speed 30.94 m/s (~111 km/h). This is pre-tuning evidence; collect a fresh trace after testing `62fc5f5` before judging the changes.
- The old `RoutePilot/README.md` describes v0.2.1 and is stale; update it when behaviour and file format are revised.

## Required traffic redesign

1. Calculate race progress and direction for every active car from the custom route, not only world-space proximity. On an oval/crossover, distinguish cars travelling in the same direction from genuinely oncoming cars.
2. Run one deterministic coordinator over a single snapshot. It should assign manoeuvre IDs and reserve future corridors for the full sequence: approach, move-out, pass, clear, return. Every participant receives the same decision before control is applied.
3. Make overtaking proactive. Use deliberate per-car pace/aggression variation or race-position pressure so a pack does not settle at identical speed and follow forever. Do not wait exclusively for naturally occurring closing speed.
4. For same-direction passes, choose a free side and make the leader yield to the opposite side. Keep the overtaker on the chosen side until its rear is clearly ahead; then return both cars smoothly. Do not repeatedly cancel/reselect a pass every frame.
5. For oncoming conflicts, coordinate both AI cars to keep to their own right, with a shared meeting reservation and a clear release condition. A car in the opposite direction must never be treated as a same-direction leader to follow.
6. Collision prediction must use each car's planned future position, not only its current offset. A reserved pair must not trigger mutual following/emergency braking, but all third-party collision checks remain enabled.
7. Reject a manoeuvre only when the future corridor is genuinely occupied or blocked by track geometry. A blanket 28 m exclusion zone is incompatible with a 22 m race-pack spacing.
8. Add telemetry for decision creation/rejection and reason, manoeuvre ID, selected side, predicted time/distance to pass, lane occupancy, obstacle clearance, route direction/progress, and whether following/braking was suppressed for a valid reservation.
9. Test the coordinator in simulation/unit-style geometry cases: same-direction pack, two cars nose-to-tail, three-car pass queue, oncoming crossover, opposing cars arriving simultaneously, and a third car occupying the passing lane.

## Track geometry and obstacle detection (SDK-verified)

The CSP SDK documentation in `lua-sdk/ac_apps/README.md` exposes `physics.raycastTrack(pos, dir, length, point, normal, includeNoise, backfaceCulling)`. It casts against physics surfaces and returns hit distance or -1. The SDK specifically notes shorter rays are faster. This API is available even when `physics.allowed()` is false; in that case the SDK says only `physics.raycastTrack()` remains available.

This is the preferred first implementation for obstacle awareness; do not attempt to parse every KN5 manually in Lua. KN5 files may contain multiple meshes and separate track/extension objects, but a raycast against the physics scene can test collision-enabled wall/pole geometry regardless of which KN5 supplied it.

Important limitation: the documented API guarantees hits on physics surfaces, not every visible render-only mesh. A wall/pole without a collision surface cannot be guaranteed detectable by this method. Do not claim full KN5 visual-mesh enumeration unless another documented CSP API is found.

Recommended implementation:
- At route load, sample a corridor around the complete custom route, including the intended passing widths on both sides.
- Cast short horizontal/oblique rays at multiple chassis heights (low bumper, body, upper body) and in several directions. Use hit distance and contact point/normal to classify persistent barriers and narrow obstacles; filter out road/ground hits by ray direction/height.
- Build a compact route-relative obstacle map with segment/progress, lateral offset, vertical band, hit position, normal and clearance.
- Save it as versioned metadata associated with the route file (or a companion per-track obstacle file); preserve compatibility with existing XYZ-only route files.
- On later loads, reuse the cached map and invalidate/rescan when the track/layout identity or route geometry checksum changes. Offer a rescan action because CSP/track updates can alter collision meshes.
- During overtaking, test the complete planned swept vehicle envelope against the cached obstacle map and use a few live raycasts as a safety confirmation. Do not raycast the entire world every frame.
- Log scan duration, ray count, detected obstacle count, cache hit/miss and skipped/failed scan reason.

## CSP SDK APIs confirmed in the checked-in docs

- `physics.raycastTrack(...)`: physics-surface raycast; primary static obstacle query.
- `physics.allowed()`: reports whether script can affect physics; raycastTrack remains available when false.
- `physics.setAICarPosition(carIndex, pos, dir)`: AI repositioning; in CSP 0.3.0-preview124+ `dir=nil` aligns to stock AI spline.
- `physics.setAIAggression(carIndex, aggression)`: AI aggression 0..1; SDK notes 0.95 corresponds to 100% after launcher multiplier.
- `physics.setAICaution(carIndex, caution)`: alters AI following distance, experimental; do not rely on it as the main RoutePilot traffic controller.
- `ac.worldCoordinateToTrack()`, `ac.trackCoordinateToWorld()`, `ac.worldCoordinateToTrackProgress()`, `ac.getTrackAISplineSides()`: stock AI spline/track coordinates and boundaries. They are not substitutes for custom-route geometry on an oval crossover.
- `ac.getCarColliders()` returns car colliders, not a documented enumerator for static track KN5 meshes.

## Latest code change and validation status

Commit `62fc5f5901c91f1408ea1335b4e13b12e2d1ab38` applies the first arcade-pace/overtake tuning pass described above. The source was re-fetched after the write and the intended constants and code paths were confirmed present. No Lua interpreter/static parser or Assetto Corsa runtime validation has been performed in this environment; the changes must be tested in-game. Do not describe them as proven fixes until a new trace confirms higher sustained speed, actual completed passes, no repeated braking, and safe oncoming meetings.

## Development rules

- Before editing, fetch the latest branch head and read the current RoutePilot source, manifest, logger, route format and latest trace. Never assume an earlier chat's code is still HEAD.
- Work on `routepilot-traffic-coordinator` unless the user explicitly changes branch.
- Preserve the existing manifest/UI and working route recording/loading/recovery unless a change is required and verified.
- Verify every API against the checked-in CSP SDK docs or an authoritative CSP source. Never invent Lua APIs.
- Keep this file current whenever architecture, verified APIs, known bugs, format, or test results change. Update README/changelog alongside user-facing behaviour changes.
- Do not report a change as tested in-game unless the user has run it and supplied results. Clearly distinguish code review, static checks and in-game validation.
