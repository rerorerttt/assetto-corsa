# RoutePilot 1.0 — Native CSP Racing AI

RoutePilot records a player-driven racing line and its left/right road limits, then converts them into a native Assetto Corsa/CSP version-7 AI spline. The game AI, not a Lua steering loop, controls steering, throttle, braking and gearbox.

## Installation

Copy the complete `RoutePilot` directory into:

```
assettocorsa/apps/lua/RoutePilot/
```

Keep all five runtime files together: `manifest.ini`, `RoutePilot.lua`, `driver.lua`, `spline_builder.lua` and `logger.lua`. Enable **Arcade AI** in Content Manager's Lua Apps list. The app targets CSP 0.3.2-preview2 and uses its native `physics.setAISpline` and `physics.setAISplineAbsoluteOffset` APIs.

## First run

1. Open RoutePilot on the track/layout you want to use.
2. Select the road width: 1, 2, 3 or 4 cars. The default is 1 lane, as specified in the task list. Select 2 lanes (7 m) or more to enable adjacent-lane passing; one lane intentionally disables passing.
3. Record a complete lap with **Запись линии**, then stop recording.
4. If necessary, record the road limits separately with **Запись границ** from any point inside the existing corridor. Only the boundary on the player's side is rewritten; it cannot expand beyond the previously saved corridor. If the car leaves the old limits, recording stops.
5. Press **Сохранить**. RoutePilot writes the text route, bounds and a generated native `.ai` spline for this track/layout. The first and last recorded points must be within 30 m so the AI line cannot close through a long artificial shortcut.
6. Add AI cars to the session. RoutePilot assigns the generated spline, waits for CSP to accept it, then spaces the AI cars along the recorded route. No separate start-autopilot button is required.

Routes and bounds are stored per track/layout. The map is drawn directly inside the main RoutePilot window and uses the same boundary geometry as the 3D track overlay.

## Driving model

- **Native AI spline:** RoutePilot writes the version-7 binary structure used by `AISplineEditor`. The native game AI follows that spline and retains its normal vehicle physics and gearbox logic.
- **Speed profile:** the spline receives curvature-derived speed hints, a 300 km/h straight-line cap, a 10.5 m/s² lateral-acceleration target and a 12 m/s² braking envelope. Actual speed still depends on the car, tyres, setup and track.
- **Pace variation:** each AI receives a small deterministic variation around its configured AI level (94–106%, clamped to 0.70–1.15) and maximum CSP AI aggression (1.0). This creates faster and slower traffic without overriding the throttle.
- **Lanes:** cars are ranked by pace. Faster groups are assigned to the leftmost lane; slower groups use the lanes to the right. Lane centres are calculated from the selected 3/7/11/15 m road width and are accepted only when the saved boundaries leave enough room.
- **Overtaking:** a faster follower can move to an adjacent free lane, pass, then return to its assigned lane. The leader is never given a pass-specific steering, throttle, brake, speed or lateral-offset command. CSP AI awareness remains enabled because the offset API is called with `overrideAiAwareness=false`.
- **Boundaries:** the generated spline stores left/right widths from the user's boundary recording. Lane assignment and passing are refused if the requested corridor does not fit those limits.
- **Spawn and recovery:** `physics.setAICarPosition(index, position, nil)` lets CSP align the car with its assigned AI spline. A car moving below 0.8 m/s for 10 seconds after startup is considered stuck. A car with all four wheel loads below 120 N while moving is treated as airborne after 10 seconds. Recovery searches forward for a free route position before teleporting.
- **No obstacle scan:** RoutePilot does not scan, record or infer wall/column collision meshes. Native CSP/AC AI awareness remains responsible for its built-in traffic response.

## Logs

The app writes `routepilot_trace.txt` in its app folder. Important events include `AI_SPLINE_BUILD_OK`, `AI_SPLINE_ASSIGNED`, `SPAWN_CAR`, `LANE_ASSIGN`, `PASS_START`, `PASS_COMPLETE`, `PASS_BLOCKED`, `PASS_RELEASE` and `RECOVERY_ATTEMPT`. Each periodic car line contains world position, speed, AI level, route projection error, lane, lateral offset and pass state.

## Validation status and limits

This is a source-level implementation based on the CSP SDK and the repository's `AISplineEditor` and `Verve` examples. It has **not** been run inside Assetto Corsa in this environment. A zero-accident guarantee is not physically possible: car dimensions, track mesh, AI behaviour, weather and mod-car physics vary. Start with a small number of AI cars and inspect the trace before increasing traffic density.

The 3D overlay also draws a short upcoming wall of blue chevrons across sharper corners, following the route direction. During recording, arrows are limited to the short preview range. Track-boundary data must be recorded accurately; RoutePilot does not infer hidden walls or poles.
