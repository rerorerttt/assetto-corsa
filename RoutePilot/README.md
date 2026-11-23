# RoutePilot 0.2.2

CSP Lua app for Assetto Corsa. Records a route from the player's car and uses it to control AI cars without relying on the track's stock AI fast lane.

## Install

Copy the `RoutePilot` folder into:

`assettocorsa/apps/lua/RoutePilot/`

Enable RoutePilot in Content Manager's Lua Apps list and open it in a session.

## First use

Drive and save a route, keep the loop option enabled for a lap route, record the track boundaries, then add AI cars to the session. The existing route recording, loading, map, world-line display and controls remain part of the app.

## Traffic and driving model

- Active AI cars are initially distributed alternately between the physical LEFT and RIGHT lanes. Negative lateral offset is physical LEFT in the current spawn coordinate basis; the map and spawn log use the same convention.
- A recorded corridor at least 6 m wide uses two lane centres, normally 1.75 m to either side of the route centreline. A corridor narrower than 6 m remains single-file.
- Neither lane has a fixed speed target or artificial straight-line speed ceiling. On a straight with no traffic constraint, the controller requests full throttle; actual top speed is determined by the car, gearing, drag and available grip.
- Both lanes use the same route-curvature planner. It estimates corner speed from curvature and available lateral acceleration, then begins braking according to the distance to the corner. The look-ahead horizon expands with speed and stopping distance.
- Cars inspect nearby traffic. A car closing on a vehicle in its current lane requests a lane change when the gap is around 10 m or the time-to-collision falls below 3 seconds. The target lane must be clear ahead, behind and through the lateral transition corridor; another AI car's requested target lane is treated as reserved. If the destination becomes occupied mid-transition, the controller aborts back toward the original lane.
- A lane change is a smooth transition between the two fixed lane centres. After completing it, the car continues following the same recorded route in the new lane; it does not generate a new racing line or choose a complex overtaking trajectory.
- If the adjacent lane is occupied, the car stays in its lane and calculates a safe following speed from the leader's speed, remaining gap and braking capability. It brakes rather than forcing a merge. The lane is checked again on subsequent updates.
- Factory traction control is requested per car. A second wheel-slip throttle limiter reduces gas if driven wheels spin, and throttle is released quickly whenever braking is commanded.
- Steering uses filtered look-ahead control with a longer speed-dependent look-ahead for high-speed stability. Gear selection remains with the car's normal drivetrain.

## Logging and validation

`routepilot_trace.txt` includes lane assignments, lane-change start/completion events, current/target lane offsets, traffic gap/block status, speed targets, brake/throttle, wheel slip and heading alignment. Static source checks do not replace an in-game test with the current car pack, track and CSP build. The latest implementation has not yet been runtime-validated in Assetto Corsa.
