# RoutePilot frame-wide traffic coordination and immediate passing

## Frame transaction

Every simulation update uses one immutable roster/projection snapshot. Before any AI receives controls, RoutePilot refreshes route progress for every active AI, updates each car's free-running racing plan and free target speed, then builds all feasible pass requests. Requests are ranked deterministically by predicted pass ETA, pace advantage, route progress and stable car ID. The coordinator commits corridor reservations in one batch; a later request sees every earlier reservation from the same frame. Per-car controller updates cannot independently start a pass.

## Synchronized pack dispersal

A pass request identifies the slower leader and the compact same-direction train occupying its discrete logical lane. The coordinator assigns the train to the source slot and the faster car to a different adjacent slot. All train members receive the same transaction owner, a fixed slot target, their actual starting offset and a predicted smooth transition distance. The pass is committed only after the complete swept path is checked against the road bounds, vehicle footprints, opposing traffic and every other reservation. The path prediction models the yielding cars moving from their current offsets toward the assigned slot; it does not assume an instantaneous lateral teleport.

A successful transaction emits `TRAFFIC_BATCH_COMMIT` and `OVERTAKE_BEGIN`. A denied transaction emits `TRAFFIC_BATCH_DENY` with the denial reason. Yielding members report `TRAFFIC_STATE=YIELDING`; the passing car reports `TRAFFIC_STATE=PASSING`. The group reservation is released only after the passing car has cleared the final group member and the return corridor is safe.

## Physical limits

The one-second target is the decision/allocation deadline, not a command to teleport a vehicle sideways. Lateral movement remains smooth and is bounded by the existing grip-aware transition planner. The road uses only the fixed discrete slots derived from the recorded width; a one-slot road cannot generate a passing slot. If a legal adjacent slot or a fully clear predicted swept path does not exist, the car follows predictably rather than squeezing through. Engine output remains governed by the existing speed and tyre-slip controller; grip loss or a real occupied corridor can still require throttle reduction or braking.

## Validation status

Source changes are committed on this experimental branch. This environment has no Lua/LuaJIT compiler and no Assetto Corsa/CSP runtime, so syntax compilation and in-game collision clearance, steering response, throughput and FPS are not verified. The first in-game test must capture `TRAFFIC_BATCH_COMMIT`, `TRAFFIC_BATCH_DENY`, `OVERTAKE_BEGIN`, `YIELDING`, `OVERTAKE_END`, speed, throttle, brake, lane/offset and route progress for 2-, 3- and 4-slot widths, including simultaneous pass requests and an occupied adjacent lane.
