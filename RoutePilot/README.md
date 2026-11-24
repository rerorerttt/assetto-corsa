# RoutePilot 0.9.x

CSP Lua app for Assetto Corsa. Records a route and its permitted corridor, then controls AI cars along discrete virtual lanes derived from that corridor.

## Install

Copy the `RoutePilot` folder into:

`assettocorsa/apps/lua/RoutePilot/`

Enable RoutePilot in Content Manager's Lua Apps list and open it in a session.

## Route and lane model

- Record the desired route and both corridor boundaries. Routes and bounds are saved per track/layout.
- A narrow corridor remains single-file. A corridor wide enough for two vehicle envelopes receives two virtual lane centres, normally at approximately ±1.75 m from the recorded route.
- Each AI car keeps a current lane assignment. When it closes on a slower car ahead, it checks the adjacent lane's front and rear clearance, the full lateral transition band, the recorded road corridor and other cars' in-progress lane reservations.
- A lane change is committed only after all checks pass. The car moves directly between the two lane rails over a fixed transition interval; if the destination is occupied or the corridor narrows, it stays in its lane and follows a safe speed.
- Vehicle width and length are read from CSP's `car.aabbSize` and projected onto the route axes. Clearance therefore uses the actual footprints of both cars rather than one fixed width.
- After passing, a car remains in its new lane. It does not immediately return; the next obstacle scan starts a new decision cycle.
- Steering validates the pure-pursuit preview chord against intermediate route-boundary samples. If the chord would cut across a bend or leave the permitted lane/corridor, the preview is shortened.
- Leaving the permitted corridor is treated as a driving failure. The driver does not teleport a car back onto the road as an off-track recovery.

## Controls

RoutePilot overrides steering, throttle and brake. Gear selection and drivetrain behavior remain with the normal Assetto Corsa/CSP AI drivetrain. CSP Lua API version targeted by this implementation: 0.3.2-preview2.

## Validation

Source-level checks are recorded in `DEVELOPMENT_STATUS.md`. The implementation still requires live Assetto Corsa testing with several cars, different vehicle dimensions, occupied and free adjacent lanes, narrow corridors and tight corners. Do not treat source-level checks as proof of collision-free in-game behavior.


The existing stuck-car rescue is intentionally preserved: after 5 seconds below 1.5 m/s, the driver calls `H.teleportHome()`, which selects a clear recovery position. The spawn update also no longer calls the undefined `alignRouteDirectionToPlayer()` global that previously caused `UPDATE_ERROR` every frame; the recorded route direction is retained.
