# RoutePilot development status

Updated: 2026-11-24

## Active traffic model

This branch is based directly on `main` and replaces the previous generic/random overtake behavior with the requested minimal two-lane rule set.

- Every AI starts in the left lane, whose center is derived from the saved left/right route bounds.
- AI cars alternate by AI car index: odd indices (1, 3, 5, ...) are capped at 40 km/h; even indices (2, 4, 6, ...) use the app's existing `speedMax` and curvature/braking planner.
- Only fast-class cars overtake.
- A fast car detects a slower vehicle ahead in its current left lane, checks that the right lane exists and is clear in a longitudinal window both ahead and behind, then moves to the right lane.
- It stays in the right lane until its front has advanced by the combined vehicle lengths (approximately two car bodies) beyond the target. It then checks the left lane; if occupied, it remains right and retries on later updates. It returns left only when that lane is clear.
- Passing is disabled unless each half of the saved corridor is at least 2.7 m wide. With symmetric 7 m bounds, lane centers are +1.75 m and -1.75 m from the recorded route. On narrower single-lane corridors, all cars stay centered on the recorded route instead of being shifted into a fictitious left lane.
- The existing stuck-car rescue and `teleportHome()` mechanism from `main` were not modified.

## Files changed

- `RoutePilot/driver.lua`: deterministic slow/fast assignment, left-lane target, right-lane pass, two-body clearance before return, and lane occupancy checks.
- `RoutePilot/README.md`: documents the behavior.

## Validation status

Static source inspection only. This branch has not been executed in Assetto Corsa/CSP. Required in-game checks:

1. Confirm all AI cars adopt the controller and initially settle into the left lane.
2. Confirm speed pattern by AI index: 1=40 km/h, 2=fast, 3=40 km/h, 4=fast.
3. Confirm fast cars pass only using the right lane and do not enter it when occupied.
4. Confirm a fast car remains right until it has cleared the target by approximately two vehicle lengths, then returns left only when clear.
5. Confirm single-lane/narrow bounds disable passing.
6. Confirm the existing stuck recovery still teleports a car after its original timeout.

Do not treat static checks as runtime validation. Preserve existing working spawn, AI adoption, steering, and rescue behavior unless a reproducible defect requires a targeted change.
