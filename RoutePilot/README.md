# RoutePilot — two independent AI traffic lanes

RoutePilot records a player-driven route and its road limits, then builds native CSP version-7 AI splines. Each AI car receives one lane-specific spline. The native game AI handles steering, throttle, braking and gearbox; RoutePilot does not run a traffic negotiation or overtaking planner.

## Installation

Copy the complete `RoutePilot` directory into `assettocorsa/apps/lua/RoutePilot/`. Keep `manifest.ini`, `RoutePilot.lua`, `driver.lua`, `spline_builder.lua` and `logger.lua` together. Enable **RoutePilot** in Content Manager's Lua Apps list. Target CSP version: 0.3.2-preview2.

## Setup

Record a complete closed lap with **Запись линии**, stop recording and press **Сохранить**. The road-width selector controls the corridor width: 3 m creates one centered lane with no passing; 7 m or wider creates two independent lanes. With two lanes, odd-numbered AI car indices are assigned to the left fast lane and even-numbered indices to the right slow lane. Cars are placed directly on their assigned lane at startup.

## Traffic behavior

- **Left lane:** its own offset spline, with curvature-derived speed hints and no artificial top-speed cap. The car's natural engine and physics limits remain.
- **Right lane:** its own offset spline with a 40 km/h speed target and a hard 40 km/h top-speed cap via `physics.setAITopSpeed`.
- **No traffic planner:** cars are never ranked by pace, asked to pass, told to yield, or dynamically moved between lanes. There are no follower brake commands, shared pass reservations or traffic occupancy checks.
- **Lane geometry:** the lane centre lines are generated from the recorded route using a horizontal tangent offset. The two lane splines are built separately, and each spline's side limits are restricted to its own half of the recorded corridor.
- **Native steering:** CSP's native AI follows the lane-specific spline. RoutePilot does not inject per-frame steering corrections, which avoids the old oscillating pursuit controller.
- **Spawn and recovery:** `physics.setAICarPosition(index, position, nil)` aligns each car to its own assigned spline. The spawn position uses the same lateral offset as that spline. Stuck/airborne recovery keeps the car in its original lane.

The 40 km/h value is both the native spline speed target and the CSP top-speed ceiling. Real vehicle speed can still fall below 40 km/h when grip, a corner, damage or the physics model prevents maintaining the target; the top-speed API is a ceiling, not a cruise-control actuator.

## SDK basis

The implementation follows `lua-sdk/ac_apps/lib.lua` and the version-7 structures in `AISplineEditor/AISpline.lua`. It uses `physics.setAISpline`, `physics.setAITopSpeed`, `physics.setAILevel`, `physics.setAIAggression`, `physics.setAICarPosition` and `physics.preventAIFromRetiring`.

## Logs and validation

The app writes `routepilot_trace.txt` in its app folder. Relevant events: `AI_LANES_BUILD_OK`, `AI_SPLINE_ASSIGNED`, `LANE_ASSIGN`, `SPAWN_BEGIN`, `SPAWN_FINISH` and `RECOVERY_ATTEMPT`.

This branch has been inspected against the SDK and source code, but cannot be launched in the Assetto Corsa runtime from this environment. It has not yet passed an in-game test. Do not treat a successful GitHub commit or source inspection as proof of race stability.