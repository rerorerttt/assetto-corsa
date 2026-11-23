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

- Records and replays the player's route per track/layout.
- Keeps the existing boundary recorder and width presets (3, 7, 11 and 15 m).
- Deterministic traffic assignment by active roster order: the first AI car uses the left lane, the second the right lane, then the assignment alternates. Existing assignments remain stable if cars are added or temporarily disappear.
- With a 7 m or wider recorded corridor, the two virtual lane centres are fixed at +1.75 m (left/fast) and -1.75 m (right/40 km/h) from the route centreline. The offsets use the route's local tangent, so the lanes follow bends without visible lane markings.
- The right lane targets 40 km/h. The left lane has no fixed 40 km/h limiter and uses the route-curvature planner to brake for corners.
- A 3 m corridor is treated as a single lane: all AI cars follow the centreline at 40 km/h; no passing lane is created.
- Cars do not inspect, follow, yield to, brake for, or overtake one another. Passing happens naturally because the left lane is faster.
- Steering uses a filtered, rate-limited look-ahead controller to hold each car on its assigned lateral offset.
- Before spawning, the route point order is checked against the player's forward vector. If the saved route runs backwards, points and signed bounds are reversed together; inference is skipped if the player is more than 15 m from the route.
- AI cars are initially placed at their assigned lane centres and retain that assignment during recovery.
- Per-car factory traction control is forced on through CSP `physics.setAssists()`. A second throttle limiter monitors driven-wheel slip ratio and reduces throttle rapidly when wheelspin appears; throttle application is ramped to avoid abrupt launches.
- Right-lane PI control has a wider integral range to avoid settling below the 40 km/h target.
- Only steering, throttle and brake are overridden. Gear selection remains with the car's normal AI/automatic drivetrain.

## Changelog

- 0.2.1: fixed route autoload — Lua apps have no `script.onSessionStart` callback, so autoload now happens on the first `script.update` call.
- 0.2.0: car-following collision avoidance, minimum-distance slider, slowing-car counter.

## Known limitations

- Car-following only prevents rear-ending the car directly ahead in the same lane; side-by-side contact during overtakes is not handled (no overtaking logic yet).
- This is an initial controller implementation. It must be tested in-game on a clear track before use in traffic. The controller does not yet perform overtaking, pit handling, or automatic recovery from a spin/off-track situation.
