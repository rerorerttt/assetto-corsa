# RoutePilot 0.2.1

CSP Lua app for Assetto Corsa. Records a route from the player's car and uses it to control AI cars without relying on the track's stock AI fast lane.

## Install

Copy the `RoutePilot` folder into:

`assettocorsa/apps/lua/RoutePilot/`

The folder must contain:
- `manifest.ini`
- `RoutePilot.lua`

Enable RoutePilot in Content Manager's Lua Apps list and open it in a session.

## First use

1. Drive the desired route with the player car.
2. Press **Начать запись маршрута** before driving.
3. Drive one full lap and press **Остановить запись**.
4. Keep **Замкнутый маршрут (круг)** enabled for a lap route.
5. Press **Сохранить маршрут этой трассы**.
6. Add AI cars to the session and press **Запустить автопилот**.

Routes are stored separately by full track ID using CSP `ac.storage` and are autoloaded when the app starts on a track with a saved route.

## Current scope

- Records player-car world X/Z coordinates and speed.
- Stores and loads routes per track/layout, with autoload on app start.
- Uses a Pure Pursuit-style steering controller and a curvature/braking speed planner.
- Car-following collision avoidance (v0.2.0): each AI car checks for the nearest car ahead in its own lane (lateral offset within 2.4 m) and limits its target speed to a safe value based on the gap and the leader's speed. Below the minimum gap (adjustable, 3–15 m) the car stops completely.
- Controls only steering, throttle and brake. No gear, clutch, reverse or stock AI spline commands are issued.
- Applies to active AI-controlled cars with index 1 and above.

## Changelog

- 0.2.1: fixed route autoload — Lua apps have no `script.onSessionStart` callback, so autoload now happens on the first `script.update` call.
- 0.2.0: car-following collision avoidance, minimum-distance slider, slowing-car counter.

## Traffic behavior (alternating left-lane / right-lane passing)

- All AI cars are assigned to the left lane initially.
- AI order alternates deterministically: car index 1 = 40 km/h, index 2 = fast class (uses the existing app speed limit and curve planner), index 3 = 40 km/h, and so on.
- Only fast-class cars initiate overtakes. They detect a slower car in the left lane, check that the right lane has enough recorded width and is clear ahead/behind, move to the right lane, and remain there until their rear has cleared the target by approximately two car lengths.
- After that clearance, the car checks that the left lane is clear and returns to it. If it is not clear, the car stays in the right lane and retries; it does not cut back across another car.
- Passing requires a corridor wide enough for two lanes (at least 2.7 m on each side of the corridor midpoint). On narrower routes, cars remain single-file.
- Lane centers are derived from the saved route bounds. With a symmetric 7 m corridor, the nominal centers are +1.75 m (left) and -1.75 m (right) from the recorded route.

## Known limitations

- Car-following only prevents rear-ending the car directly ahead in the same lane; side-by-side contact during overtakes is not handled (no overtaking logic yet).
- This is an initial controller implementation. It must be tested in-game on a clear track before use in traffic. The controller does not yet perform overtaking, pit handling, or automatic recovery from a spin/off-track situation.
