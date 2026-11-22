# RoutePilot 0.1.0

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

Routes are stored separately by full track ID using CSP `ac.storage`.

## Current scope

- Records player-car world X/Z coordinates and speed.
- Stores and loads routes per track/layout.
- Uses a Pure Pursuit-style steering controller and a curvature/braking speed planner.
- Controls only steering, throttle and brake. No gear, clutch, reverse or stock AI spline commands are issued.
- Applies to active AI-controlled cars with index 1 and above.

This is an initial controller implementation. It must be tested in-game on a clear track before use in traffic. The controller does not yet perform overtaking, collision avoidance, pit handling, or automatic recovery from a spin/off-track situation.
