# RoutePilot development notes

Updated: 2026-11-24
Branch: `routepilot/two-independent-lanes-2026-11-24`
Implementation: two independent native CSP AI lane splines

## Architecture

1. The player records one closed centre route and its left/right corridor limits.
2. `spline_builder.lua` writes native version-7 CSP AI spline files using the exact structure from `AISplineEditor/AISpline.lua`.
3. For the 3 m preset, one centered spline is generated. For 7/11/15 m presets, two distinct lane-centre splines are generated at offsets `-width/4` and `+width/4`.
4. Odd AI car indices use the left/fast spline; even indices use the right/slow spline. The assignment is stable for the life of the car and is not reconsidered based on traffic.
5. The right spline contains a 40 km/h speed target and `physics.setAITopSpeed(index, 40)` supplies a hard top-speed ceiling. The left spline uses curvature-derived speed hints with a 1000 km/h hint ceiling and `physics.setAITopSpeed(index, 1e9)` to remove the top-speed restriction.
6. CSP native AI owns steering, throttle, braking, clutch and gearbox. RoutePilot no longer calculates pass gaps, checks lane occupancy, reserves passing corridors, changes spline offsets or issues traffic-related braking.

## SDK references

Source of truth: `lua-sdk/ac_apps/lib.lua`.

- `physics.setAISpline(carIndex, filename, aiHintsFilename)` assigns a custom spline to an individual AI car. The driver retries failed assignments and waits 1.5 seconds after a successful return before spawn.
- `physics.setAITopSpeed(carIndex, limit)` sets the AI top speed in km/h. The SDK explicitly documents `1e9` as a way to remove the limit.
- `physics.setAICarPosition(carIndex, pos, nil)` places an AI car and aligns its direction to its currently assigned AI spline.
- `physics.setAILevel` accepts 0–2; `physics.setAIAggression` accepts 0–1.
- `physics.preventAIFromRetiring` must be called repeatedly to keep a stationary AI from retiring.
- `physics.raycastTrack` is used only for road-surface snapping and normal sampling.
- The spline point/payload structure and binary header match `AISplineEditor/AISpline.lua`.

## Geometry and spawn

The builder offsets each route sample along its horizontal tangent, using the CSP convention that negative lateral offset is left. Each lane's native side widths are set to half the selected corridor width, so each AI spline describes only its own half-corridor. Spawn and recovery use the same offset formula, preventing cars from being teleported to the centreline and then having to cross into their lane.

The width presets remain 3, 7, 11 and 15 m. Preset 1 is a single centred lane. Presets 2–4 produce two lanes with half-widths 1.75, 2.75 and 3.75 m respectively.

## Limits

The 40 km/h spline value is a target and `setAITopSpeed(40)` is a ceiling. CSP can still drive below 40 km/h when vehicle dynamics, grip, cornering, damage or native safety logic require it. The SDK exposes a top-speed cap, not a cruise-control command that can guarantee exactly 40 km/h under every condition.

The two-lane architecture removes RoutePilot's own traffic reactions. It does not replace the underlying CSP/AC AI implementation or disable all built-in collision/physics behavior. No source-only validation can guarantee identical behavior across all mod cars and tracks.

## Validation status

Source has been checked against the SDK declarations and the reference spline writer. This environment has no Assetto Corsa runtime, so the branch has not been compiled by CSP/LuaJIT or tested in a live session. In-game validation is still required.