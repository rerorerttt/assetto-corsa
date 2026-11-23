# RoutePilot — Native CSP Racing AI

RoutePilot records a player-driven route and road limits, then writes a native version-7 AI spline and assigns it to Assetto Corsa AI cars through CSP. It does not generate a track's permanent `fast_lane.ai`, `ac_start` or `ac_time` files: `physics.setAISpline()` loads a custom spline for each AI car at runtime.

## Installation

Copy the complete `RoutePilot` directory into:

```
assettocorsa/apps/lua/RoutePilot/
```

Keep the runtime files together: `manifest.ini`, `RoutePilot.lua`, `driver.lua`, `spline_builder.lua` and `logger.lua`. Enable RoutePilot in Content Manager's Lua Apps list. The target environment is CSP 0.3.2-preview2.

## First run

1. Open RoutePilot on the required track/layout.
2. Select the road width: 1, 2, 3 or 4 cars (3, 7, 11 or 15 m). One lane disables passing; two or more lanes permit a left-side pass.
3. Record a complete lap with **Запись линии**, then stop recording.
4. Record road limits separately with **Запись границ** if the default corridor is not accurate. Record both sides carefully around walls, poles and narrow sections.
5. Press **Сохранить**. RoutePilot stores the route and bounds per track/layout and builds a native `.ai` spline. A closed route must have a start/end gap no greater than 30 m.
6. Add AI cars. RoutePilot assigns the custom spline, waits for CSP's background load, then places the AI cars along the route.

## Driving model

- **Native AI:** CSP/Assetto Corsa owns steering, throttle, braking and gearbox. RoutePilot does not write steering or pedal overrides.
- **Version-7 spline:** the binary layout matches `AISplineEditor/AISpline.lua`. The spline's speed channel is written in m/s. Throttle and brake hints are derived from the curvature-limited speed profile; the previous all-zero gas/brake values were invalid for a moving AI.
- **Speed planning:** corner speed is calculated from route curvature using a 10.5 m/s² lateral-acceleration target. A backward braking envelope uses 12 m/s². The user speed limit is entered in km/h and converted to m/s before it is written to the spline.
- **Surface height:** each route point is raycast from just above the recorded car position. This avoids a long ray hitting another deck on a multi-level track. Failed hits are interpolated from valid neighbouring road samples; if more than 40% of samples fail, spline generation aborts instead of silently creating a malformed route.
- **3D projection:** route matching includes world height, so cars on an overpass are not projected onto a lower road with the same X/Z coordinates.
- **Cruising lane:** cars prefer the rightmost valid lane; if the player occupies that corridor, they choose the next safe lane to the left. If no lane fits clear of the player, they remain on the recorded centreline rather than forcing a side-by-side overlap. Cars are not distributed by speed into different default lanes.
- **Overtaking:** only a faster follower may move left. A pass is reserved before the leader is reached, the passing corridor is checked against road bounds, nearby traffic and the player's actual position, and the follower returns only after it is at least 15 m ahead and the home lane is clear. The leader receives no pass-specific steering, throttle, brake, speed or lateral-offset command. Only one pass is reserved at a time.
- **AI profile:** each car receives a small deterministic pace variation around its configured AI level. CSP aggression is set to 0.95 (the SDK's value for 100% aggression); AI caution is set to 0.5. Artificial throttle and top-speed caps are removed while the car's physical limits remain. After spawn/recovery, CSP's post-incident stop counter and parked-input state are cleared.
- **Recovery:** a car below 0.8 m/s for 10 seconds is considered stuck. Recovery searches forward for a free route position before teleporting. This is a fallback, not a substitute for a valid route or functioning native AI.
- **Boundaries:** the spline side widths use the same lateral coordinate frame as AISplineEditor. Lane assignment and passing are refused when the requested vehicle footprint does not fit inside the recorded limits.

## Why there is no Fastline/AC start-time dependency

The app writes the version-7 spline and calls CSP's documented `physics.setAISpline(carIndex, filename, nil)`. The API accepts a custom spline filename and returns a boolean; the track's permanent `fast_lane.ai`, `ac_start` and `ac_time` files are not generated or edited. `setAICarPosition(carIndex, position, nil)` aligns a teleported car to its assigned AI spline.

## Logs

The app writes `routepilot_trace.txt` in its app folder. Important events include `AI_SPLINE_BUILD_OK`, `AI_SPLINE_ASSIGNED`, `SPAWN_CAR`, `LANE_ASSIGN`, `PASS_START`, `PASS_COMPLETE`, `PASS_BLOCKED`, `PASS_RELEASE` and `RECOVERY_ATTEMPT`. Periodic car records include world position, speed, AI level, route projection error, lane, lateral offset and pass state.

## Validation

This is a source-level correction and has not yet been run inside Assetto Corsa in this environment. A zero-accident guarantee is not physically possible across arbitrary mod cars, track meshes, weather and vehicle setups. First test with 3–5 cars on a known-good recorded route; verify that speed rises after spawn, then check lane assignment, pass start/completion, wheel loads and route error before increasing traffic density.

The 3D overlay draws the upcoming route, boundaries and blue corner chevrons in the main app. RoutePilot does not infer hidden wall/column collision meshes; the recorded corridor and native CSP/AC awareness remain the safety limits.
