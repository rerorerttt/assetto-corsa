# RoutePilot frame-wide traffic state coordination

This branch introduces a frame-wide coordination pass while retaining the existing geometric path planner and full-body safety checks.

## Frame order

1. `RoutePilot.lua` gathers the complete active AI roster.
2. `Driver.prepareFrame()` projects every active AI car onto the recorded route before any car receives new controls. The resulting progress, lateral position and segment are stored for the current frame.
3. The roster is arbitrated deterministically. Already active PASS/RETURN reservations remain ahead of new requests. New pending pass requests are ranked by measured longitudinal acceleration and current speed; FIFO request age is no longer allowed to outrank vehicle performance.
4. The normal per-car controller consumes the common frame projection and performs its state transition and control write.

This removes the most important order dependency: a car processed early no longer sees a mixture of current-frame and previous-frame route projections for the other cars.

## Traffic states and reservation rules

The controller's observable states remain `OPEN`, `FOLLOW`, `WAIT_QUEUE`, `WAIT_YIELD`, `YIELDING`, `PASSING`, `RETURNING` and `ROUTE_LOST`. A car in `OPEN` receives the racing-line target speed without a traffic speed cap. `FOLLOW` is entered only when a vehicle overlaps the current driving corridor under the existing route-progress, direction and physical lateral-separation checks. A car in a separate usable lane must not reduce the subject car's speed.

A pending yield reservation can be pre-empted by a materially stronger eligible requester before the pass begins. An active pass is never pre-empted. Stale pending reservations are invalidated when the target is no longer active, is no longer ahead, or has moved beyond the bounded request horizon. This prevents a distant former target from keeping a car in `WAIT_QUEUE`.

## Planner failure and recovery

When a reserved PASS/RETURN/YIELD path cannot be optimized, the planner now tries a stationary path in the car's measured current corridor. It uses the same full-body swept-path and recorded road-boundary checks as every other path. It does not jump lanes or widen the road envelope. A hard stop remains possible only when the current corridor also fails the geometric validation.

A locked route projection may be reacquired globally only inside the existing 8 m acquisition radius and with the existing heading-direction validation. The stuck-recovery timer no longer triggers a teleport while the car is in a known queue/yield/pass transaction, has a nearby current-lane leader, has lost route projection, or has a blocked geometric plan.

## Validation status and acceptance tests

Static source review confirms that the frame preparation call precedes sorting and control updates, the FIFO comparator is removed, pending reservations can be performance-pre-empted, and the planner fallback retains the full-body validator. The environment used for this change does not contain Lua/LuaJIT or the Assetto Corsa/CSP runtime, so syntax compilation, runtime behavior, collision clearance and FPS have not been verified here.

In-game acceptance must cover: one car on Drag at increasing speed; two cars with the slower car ahead; a faster car requesting the same leader after a weaker requester; a vehicle in an adjacent lane that must not affect following speed; PASS-to-RETURN when the target corridor narrows; a blocked yield corridor; route projection loss and reacquisition; and 1/5/10/20-car FPS measurements. Every run should inspect `TRAFFIC_STATE`, `TRAFFIC_GAP`, `TARGET_SPEED`, `BRAKE`, `PLAN_BLOCKED`, `YIELD_PREEMPT`, `ROUTE_REACQUIRE` and `RESCUE_ATTEMPT` in the trace.
