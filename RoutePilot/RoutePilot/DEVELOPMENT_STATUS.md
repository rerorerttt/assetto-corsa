# RoutePilot development status

Last updated: 2026-11-24

## Experimental branch: single-car kinematic path test

- Branch: `routepilot/kinematic-single-car-2026-11-24`
- Base: current `main`; the default branch is not modified.
- Scope: only the first active AI car is adopted by RoutePilot and placed along its recorded lane.
- Existing route recording, boundary recording, save/load, map, world-line rendering and UI are retained from `main`.
- Other AI cars are not teleported or controlled by RoutePilot in this experiment.
- The selected AI car receives full throttle, zero steering input and zero braking through `ac.overrideCarControls`. AC retains engine and gearbox simulation.
- Each update, RoutePilot projects the car onto the recorded route, advances by its current physical speed multiplied by `dt`, applies the assigned lane offset, and calls `physics.setAICarPosition` with the direction convention used by the existing spawn code.
- There is no route-curvature speed planner, tyre-slip limiter, steering controller or artificial speed cap. The car's engine/gearbox physics determines its acceleration; the path position is externally constrained to the recorded line.

## SDK facts verified

- `physics.setAICarPosition(carIndex, pos, dir)` is documented in `lua-sdk/ac_apps/lib.lua` as changing the position of an AI car.
- `ac.overrideCarControls(carIndex)` returns `ac.CarControlsInput`; `combineAxis=false` overrides original pedal and handbrake values.
- `physics.preventAIFromRetiring(carIndex)` must be called regularly to keep a non-moving AI from retiring.
- `physics.setCarVelocity` is intentionally not used: the SDK documents that it invalidates the current lap time.
- No API for disabling tyre/suspension physics while retaining the native engine was found in the inspected SDK. This experiment therefore does not claim that physics is disabled: it constrains AI position while the normal engine/gearbox physics remains active.

## Important runtime limitations to verify

- The SDK documents `setAICarPosition` as changing AI position, but does not explicitly document per-physics-frame path animation as a supported use. This must be tested in CSP 0.3.2-preview2.
- It is not yet verified whether repeated position updates preserve natural engine sound, gear changes, wheel rotation, suspension animation and collision behavior without visible snapping.
- Progress uses the car's actual `speedMs`. If the native AI does not accelerate under overridden full throttle, the path will not advance; the log will expose this instead of substituting a fabricated speed.
- The current direction sign follows the convention in the existing RoutePilot spawn code; confirm heading in-game.
- Static review confirms that only the first active AI is spawned and updated, the existing UI/recording/map functions remain in the application file, and the kinematic driver uses native throttle controls plus `setAICarPosition` without `setCarVelocity`.
- This branch has not been run inside Assetto Corsa. Do not merge into main before a live test.

## First runtime test

1. Load the saved route and its bounds.
2. Start with one AI car; confirm only that car is adopted and placed on the assigned lane.
3. Check that it accelerates under full throttle and continues moving while remaining on the line through corners.
4. Listen for native engine/gearbox sound and inspect gear, RPM, speed and wheel animation.
5. Confirm no repeated visible teleporting, heading flips, disappearing car, retirement or Lua errors in `routepilot_trace.txt`.
6. Only after the single-car test passes, consider extending the same controller to additional cars.
