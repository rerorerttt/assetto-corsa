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
- Neither lane has a fixed speed target or artificial straight-line speed ceiling. On a straight with no traffic constraint, the controller requests full throttle. The corner planner now assumes 17.5 m/s² lateral acceleration and 14 m/s² braking for an aggressive late-braking profile; actual speed still depends on the car's engine, gearing, drag and tyre grip. The controller does not impose a 200 or 320 km/h limiter.
- At 320 km/h the corner planner expands its preview to roughly the full calculated stopping distance plus a margin. Route projection for active AI is restricted to a local segment window so it cannot snap to a nearby parallel section of a hairpin.
- Every AI car receives a stable LEFT/RIGHT lane assignment. Lane changes use only the two canonical lane centres; the target centre is derived from the assigned lane geometry, never by negating a partially completed transition offset. Traffic occupancy is measured from each car's actual position projected onto its nearest route segment. A car closing on a vehicle in its current lane requests a lane change when the gap is around 10 m or the time-to-collision falls below 3 seconds. The target lane must be clear ahead, behind and through the lateral transition corridor; active lane-change targets are reservations, preventing two cars from claiming the same gap. Rear clearance scales with the closing speed rather than using a fixed oversized exclusion zone.
- If the destination becomes occupied mid-transition, the car starts one return transition to its original lane. That return is not repeatedly restarted each physics frame. The controller maintains safe following speed while blocked or returning.
- A lane change is a smooth transition between the two fixed lane centres. After completing it, the car continues following the same recorded route in the new lane; it does not generate a new racing line or choose a complex overtaking trajectory.
- If the adjacent lane is occupied, the car stays in its lane and calculates a safe following speed from the leader's speed, remaining gap and braking capability. It brakes rather than forcing a merge. The lane is checked again on subsequent updates.
- Factory traction control is requested per car. A second wheel-slip throttle limiter reduces gas if driven wheels spin, and throttle is released quickly whenever braking is commanded.
- Steering uses filtered look-ahead control with a longer speed-dependent look-ahead for high-speed stability. The desired car centre is clamped at least 1.10 m inside the recorded boundaries; when the actual car centre leaves that safety envelope, steering recovery is strengthened and speed is reduced, with a stronger speed reduction near the recorded edge. Gear selection remains with the car's normal drivetrain.

## Logging and validation

`routepilot_trace.txt` includes lane assignments, lane-change start/completion/abort-return events, current/target lane offsets, actual lateral position (`LAT_OFF`), boundary recovery error/status, traffic gap/block status, speed targets, brake/throttle, wheel slip and heading alignment. Static source checks do not replace an in-game test with the current car pack, track and CSP build. The latest implementation has not yet been runtime-validated in Assetto Corsa.
