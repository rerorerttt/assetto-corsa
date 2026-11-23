# RoutePilot development notes

Updated: 2026-11-23
Branch: `main`
Implementation: RoutePilot 1.0 native CSP AI spline

## Current architectural decision

The previous RoutePilot versions attempted to steer every AI car from Lua by repeatedly overriding steering, throttle and brake. The recorded traces showed unstable tracking, large route-projection errors, cars stopping and repeated render errors. The older `RoutePilot_Back.lua` on `feature/arcade-traffic` is retained as historical reference, not as the active controller.

The new implementation uses the native AC/CSP AI controller:

1. The player records a closed racing line and left/right limits.
2. `spline_builder.lua` serializes those samples into the version-7 binary AI spline format used by `AISplineEditor/AISpline.lua`.
3. Each AI car receives that file through `physics.setAISpline(carIndex, filename, nil)`.
4. The native game AI owns steering, throttle, braking, clutch and gearbox.
5. RoutePilot assigns pace-ranked lanes and changes only a faster follower's spline offset when an adjacent lane is clear.
6. CSP's own AI awareness remains enabled: calls to `physics.setAISplineAbsoluteOffset` pass `false` for `overrideAiAwareness`.

This removes the source of conflicting Lua steering and throttle corrections rather than trying to tune another version of the same controller.

## SDK audit

Source of truth: [lua-sdk/ac_apps/lib.lua](../lua-sdk/ac_apps/lib.lua).

- `physics.setAISpline(carIndex, filename, aiHintsFilename)` loads a per-car spline in the background. Passing `nil` as filename resets the car to its default spline. The function returns a boolean; RoutePilot only considers the spline accepted when the result is `true`, retries failures once per second, and waits at least 0.8 seconds before spawning.
- `physics.setAISplineAbsoluteOffset(carIndex, offset, overrideAiAwareness)` uses meters; negative is left and positive is right. Actual offset is limited by track width. RoutePilot passes `false` so CSP awareness is not explicitly overridden.
- `physics.setAICarPosition(carIndex, pos, nil)` aligns the car along its current AI spline. This replaces the earlier ambiguous manually constructed direction vector.
- `physics.setAILevel` accepts 0–2. `physics.setAIAggression` accepts 0–1; the SDK notes that launcher aggression is multiplied by 0.95, so 0.95 corresponds to full launcher aggression.
- `physics.preventAIFromRetiring` must be called repeatedly to keep an AI from retiring while stationary; RoutePilot calls it every two seconds.
- `physics.raycastTrack` is used only when building a spline to obtain a road-surface normal. There is no wall/column collision scan, collision map or obstacle raycast system.
- `ac.CarState.aabbSize` provides the car's bounding-box size and is used to estimate the half-width needed for side-by-side lane clearance.

## Reference applications

- [AISplineEditor/AISpline.lua](../AISplineEditor/AISpline.lua) defines the version-7 point and payload structures and writes the header, point array and payload array with `shared/utils/binary`. `spline_builder.lua` mirrors that layout: version 7, point count, 64-bit header value, 20-byte points, 72-byte payloads and the trailing integer.
- [Verve/lib/racecraft.lua](../Verve/lib/racecraft.lua) demonstrates CSP-native spline-offset racecraft and deliberately keeps `overrideAiAwareness=false`. RoutePilot uses the absolute-offset API so the offsets can be checked against the recorded meter-based boundaries.
- [comfy_map/comfy_map.lua](../comfy_map/comfy_map.lua) uses app-window UI patterns. RoutePilot does not create a second map window: it draws the map directly in `windowMain`, so moving/resizing the app moves the map with it.
- The task list is [Задачи грок.txt](./Задачи%20грок.txt). The previous feature-branch trace contained large line errors for one AI car and repeated `DRAW3D_ERROR` events; the main-branch trace supplied with the task had no AI cars in the sampled session (`ai=0`), so it could not validate AI driving.

## Current behavior

### Native spline generation

- RoutePilot stores the editable route and bounds as text files per track/layout.
- The native spline is generated alongside those files. The filename includes a timestamp/revision so CSP is asked to load a new spline after edits. The builder rejects a start/end gap greater than 30 m to prevent a long diagonal closure segment.
- Payload speed hints are derived from horizontal curvature. Straight sections are capped at 300 km/h; the curve target uses 10.5 m/s² lateral acceleration, and a two-pass cyclic braking envelope uses 12 m/s² deceleration.
- Throttle and brake hints are left neutral. The game AI remains responsible for actual inputs.
- Left and right side widths come from the user's signed boundary arrays. The spline builder stores their positive magnitudes in the native payload.
- Surface normals are sampled downward from each route point. These rays do not search for obstacles.

### Lanes and overtaking

- Width presets are 3, 7, 11 and 15 metres, representing one through four lanes.
- Default is preset 1 (3 m), preserving the requested initial selection. Select preset 2 (7 m) or wider to enable adjacent-lane passing; one lane intentionally disables passing.
- AI cars receive a small deterministic pace spread around their existing AI level (0.94–1.06 multiplier, clamped to 0.70–1.15) and 0.95 aggression.
- Cars are sorted by pace: the fastest group receives the leftmost lane; progressively slower groups receive lanes to the right.
- The leader is not assigned a pass-specific offset, speed reduction, throttle reduction or brake command. If a faster follower catches a car in its own lane, it may move to an adjacent lane only when the recorded limits and traffic-occupancy check leave enough room.
- A pass is complete when the follower is more than seven metres ahead; it then returns to its assigned home lane.
- The occupancy check includes cars ahead and behind, and treats a car facing against the recorded route direction as an oncoming obstruction.
- The pass planner is intentionally conservative. If the recorded width does not fit the car's bounding boxes plus margin, the pass is refused.

### Spawn and recovery

- Spawn waits for a stable AI roster and for every AI to report that the generated spline was accepted.
- AI cars are spaced along the recorded loop. A nil direction argument asks CSP to align each car to its assigned spline.
- A car below 0.8 m/s for 10 seconds after startup is treated as stuck. Recovery searches forward along the route for a position clear of other cars, then teleports with a nil direction so the spline controls heading.
- CSP does not expose a single airborne boolean in the inspected car state. RoutePilot uses the four `ac.StateWheel.load` values instead: all four below 120 N while the car is moving for 10 continuous seconds triggers the same free-position recovery. This is a heuristic, not a definitive flight-state API.

### Map and rendering

- The map is drawn in the main app window and fills the available area below the controls.
- The map uses the same averaged-tangent boundary vertices as the 3D overlay. No artificial pixel separation is added to the red lines.
- Boundary-only recording can start at any point inside the existing corridor. It rewrites only the boundary on the player's side, preserves the opposite edge, and clamps both edges to the old corridor. Recording stops and saves if the player leaves the old limits.
- During route recording, 3D lines preview the existing route a short distance ahead and behind the car without drawing the unfinished end-to-start closure. A completed route can be displayed at full distance.
- The map's 1/2/3/4-lane selector shows the active preset. The speed and following-distance sliders and the AI-disable button are absent.
- The 3D overlay draws short upcoming walls of blue chevrons across corners, aligned with the local route tangent. During recording, arrow rendering is limited to the preview range to avoid drawing the unfinished closure.

## Files changed in RoutePilot 1.0

- `RoutePilot.lua`: native spline build/load lifecycle, stable spawn, safe route recovery, same-window map, accurate continuous boundaries and 3D preview.
- `spline_builder.lua`: version-7 binary spline writer, curvature speed hints, braking envelope, road-side widths and surface normals.
- `driver.lua`: native spline assignment, pace profile, stable lane assignment, follower-only adjacent-lane passing, telemetry and stuck recovery.
- `manifest.ini`: version 1.0.0 and larger main window.
- `README.md`: installation and operating notes.

## Validation status

Source was inspected against the repository SDK and reference apps. This environment does not provide the Assetto Corsa runtime or LuaJIT compiler, so the implementation has not been syntax-compiled or driven in-game. Do not treat source-level inspection as a successful race test. The first in-game test should use a short AI field on the exact track/layout used to record the route, then inspect `routepilot_trace.txt` for `AI_SPLINE_ASSIGNED`, `SPAWN_FINISH`, `LANE_ASSIGN`, `PASS_START`, `PASS_COMPLETE`, `PASS_BLOCKED` and `RECOVERY_ATTEMPT`.

A no-accident guarantee is not possible across arbitrary mod cars and track meshes. The design avoids direct Lua actuator conflicts, refuses passes that do not fit the saved corridor, and preserves native CSP AI awareness; it cannot make malformed route boundaries or arbitrary car physics safe by itself.
