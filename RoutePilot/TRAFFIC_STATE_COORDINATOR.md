# RoutePilot frame-wide traffic coordination and immediate passing

## Frame order

1. The main app gathers the active AI roster.
2. `Driver.prepareFrame()` projects every active AI car onto the recorded route before any car receives new controls. Route progress and lateral position are shared for the frame.
3. Each car computes its own unconstrained route speed target from its racing line, curvature, braking estimate and grip estimate. This value is stored as `freeTargetSpeed` before traffic logic applies any FOLLOW cap.
4. Traffic control compares that independent pace and the learned acceleration estimate with the car ahead. If the subject car has a real performance advantage, it immediately searches for a free adjacent corridor and starts the pass in the same update when the complete swept path is clear.

## Passing rules

There is no `YIELD_REQUEST`, `WAIT_YIELD`, `WAIT_QUEUE`, leader acknowledgement, or requirement for the car being passed to move aside in the new pass decision path. A pass is initiated directly through the existing road-boundary, swept-footprint, lane-occupancy and reservation checks. A successful pass reserves its selected offset for the duration of the manoeuvre; the reservation prevents two cars from claiming the same corridor simultaneously. The passing car keeps its own free-running target speed instead of being capped by the target it is passing.

FOLLOW is only the fallback when the car ahead occupies the current corridor and no safe adjacent passing corridor is currently available, or when the free-pace/acceleration model does not predict a feasible pass. The controller retries a denied corridor scan at 10 Hz. The retry interval is not a negotiation or a wait for another car: it is only a throttle on repeated geometric checks. Cars in a separate usable lane do not reduce the subject car's speed.

The performance comparison deliberately does not use only current speed difference: that value is already reduced by FOLLOW. It compares the subject car's free route speed with the leader's last-frame free route speed and combines this with the learned longitudinal acceleration estimates. The pass-distance estimate includes the actual longitudinal gap and both vehicle lengths.

## Diagnostics

`OVERTAKE_BEGIN` confirms that a pass was actually committed. `OVERTAKE_SCAN` records a denied attempt with gap, each car's free target speed, acceleration estimates, predicted advantage, estimated pass distance and the geometric denial reason. `TRAFFIC_STATE=FOLLOW` with `TRAFFIC_REASON=pass_corridor_unavailable` means a faster car was detected but the complete swept path did not pass the road/traffic/reservation checks.

## Validation status

The source has been updated and reviewed for the direct pass path. This environment has no Lua/LuaJIT executable and no Assetto Corsa/CSP runtime, so Lua compilation and in-game dynamics, overtaking frequency, collision clearance and FPS cannot be truthfully claimed as verified. In-game acceptance must check: a faster car behind a slower one on a multi-lane section; two cars attempting the same corridor simultaneously; a pass through a narrowing; a car occupying the adjacent lane; and 1/5/10/20-car FPS. Inspect `OVERTAKE_SCAN`, `OVERTAKE_BEGIN`, `OVERTAKE_END`, `TRAFFIC_REASON`, `TARGET_SPEED` and `BRAKE`.
