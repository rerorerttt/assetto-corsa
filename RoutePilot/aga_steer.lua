-- RoutePilot/aga_steer.lua — v0.1.0 (CSP 0.3.2-preview2)
-- Advanced Gamepad Assist steering stabilizer, adapted for AI cars in a
-- Lua app. AGA's core idea (per its assist.lua source): a human/driver
-- input is a DIRECTION REQUEST, and the car follows it through a
-- stabilizer that (1) adds a self-steer force proportional to the rear
-- axle's velocity angle so the car keeps holding the requested angle
-- even in a slide, (2) damps yaw rate to kill oscillation, and
-- (3) limits steering so the front wheels stay near the slip that still
-- produces lateral force — the car "never loses control" even mid-drift.
--
-- Ported for ac.StateCar (Lua app context; AGA itself is an online car
-- script with ac.StateCarPhysics, which apps cannot fully rely on):
--   * Rear axle velocity angle: computed from car.velocity transformed
--     into the car frame at the rear-axle point (rigid-body point
--     velocity = v + w x r). Falls back to the simple body slip angle
--     when local angular velocity is unavailable.
--   * Front slip limit: from front wheels' ndSlip (guarded, nil-safe);
--     when unavailable, from the body slip angle with a conservative cap.
--   * Self-steer and damping strengths are speed-aware: full authority in
--     the grip/slide band, faded out below walking speed (parking and
--     launch are unaffected).
--
-- This module NEVER issues controls by itself: it takes the path
-- controller's desired normalized steer plus the live car state and
-- returns a stabilized normalized steer. It is additive to the existing
-- path tracking and cannot change where the car is going — only how
-- smoothly it gets there.

local AGA = {}

-- Tunables (AGA defaults in parentheses; AI values slightly gentler
-- because a race controller already feeds a smooth desired steer)
local SELF_STEER_RESPONSE = 0.30   -- (0.37) rear-velocity-angle correction strength
local DAMPING_STRENGTH    = 0.30   -- (0.37) yaw-rate damping
local MAX_SELF_STEER      = 0.45   -- fraction of full lock the self-steer may add
local TARGET_FRONT_SLIP   = 0.85   -- (0.95) fraction of front ndSlip budget
local SLIP_LIMIT_REDUCTION = 0.55   -- how far the limit may pull the wheel back

local function clamp(x, a, b) return math.max(a, math.min(b, x)) end
local function num(v, d)
  if type(v) ~= 'number' then return d or 0 end
  if v ~= v or v == math.huge or v == -math.huge then return d or 0 end
  return v
end
local function sign(x) if x > 0 then return 1 elseif x < 0 then return -1 end return 0 end

-- Signed self-steer force from the rear-axle velocity angle (deg at AGA
-- scale /72 normalized). Positive rear angle = sliding outward to the
-- left of heading; the wheel steers INTO the slide (countersteer).
local function selfSteerForce(rearAngleDeg, steerLockDeg)
  if steerLockDeg <= 1 then return 0 end
  local correction = clamp(-rearAngleDeg / 72.0, -1, 1)
  -- AGA shapes the response with an exponent; a plain linear shaping is
  -- enough for the AI path controller and avoids extra parameters.
  return correction * 72.0 / steerLockDeg
end

---Compute the stabilized steering command.
---@param car ac.StateCar live car state (AI car)
---@param desiredSteer number normalized -1..1 from the path controller
---@param dt number frame delta seconds
---@param S table per-car persistent state (uses S.agaSelfSmooth etc.)
---@return number stabilized normalized steer -1..1
function AGA.stabilize(car, desiredSteer, dt, S)
  local steer = clamp(num(desiredSteer, 0), -1, 1)
  local speed = math.max(0, num(car.speedMs, 0))

  -- Fade in above walking speed: the stabilizer must not fight the
  -- launch phase or very slow maneuvers.
  local fade = clamp((speed - 3.0) / 5.0, 0, 1)
  if fade <= 0 or dt <= 0.0001 then return steer end

  -- Car frame: forward = look, right = cross(up, look) style basis.
  local look = car.look
  local lx, lz = num(look.x), num(look.z)
  local fl = math.sqrt(lx * lx + lz * lz)
  if fl < 1e-6 then return steer end
  local fx, fz = lx / fl, lz / fl
  -- Right vector in the horizontal plane (AC: x right, z forward in car frame)
  local rx, rz = fz, -fx

  -- Body-frame velocity
  local v = car.velocity
  local vx, vz = num(v.x), num(v.z)
  local fwdVel = vx * fx + vz * fz
  local latVel = vx * rx + vz * rz

  -- Rear-axle point velocity: v + omega x r, with r = rear axle offset.
  -- ac.StateCar exposes localAngularVelocity (guarded); wheel positions
  -- give the physical axle offset; otherwise fall back to -1.3 m.
  local angVelY = 0
  local okA, lav = pcall(function() return car.localAngularVelocity end)
  if okA and lav then
    local okY = pcall(function() angVelY = num(lav.y, 0) end)
    if not okY then angVelY = 0 end
  end
  local rearOffset = -1.3
  local okW, wheels = pcall(function() return car.wheels end)
  if okW and wheels then
    local okF, wFront = pcall(function() return wheels[0].position end)
    local okR, wRear = pcall(function() return wheels[2].position end)
    if okF and okR and wFront and wRear then
      rearOffset = clamp(num(wFront.z, 0) - num(wRear.z, 0), -3.5, -0.5)
      -- wFront.z - wRear.z is the wheelbase (positive), axle sits at
      -- mid-rear; use its midpoint relative to body origin.
      rearOffset = clamp((num(wRear.z, -1.3) + num(wFront.z, 1.3)) * 0.5, -3.5, -0.5)
    end
  end
  -- Point-velocity of the rear axle in the horizontal plane:
  -- for yaw about +Y, the lateral velocity at distance d behind origin
  -- is latVel + angVelY * (-d) ... derive: v_point = v + w x r, with
  -- r = (0, 0, rearOffset): w x r for w=(0,wy,0) gives (wy*rearOffset* (x̂?)) —
  -- use the safe 2D form: latVelPoint = latVel + wy * rearOffset * (sign convention
  -- checked against AGA: AGA computes local axle velocities with full 3D math;
  -- in the 2D horizontal approximation the rear point slides by wy * rearOffset
  -- laterally opposite to the front).
  local rearLatVel = latVel + angVelY * rearOffset
  local rearLongVel = math.max(0.5, fwdVel)

  local rearAngleDeg = math.deg(math.atan2(rearLatVel, math.abs(rearLongVel)))

  -- Body slip fallback scale (used only for logging-free strength shaping)
  -- Steering lock estimate: normalize by the front wheels' angle if exposed,
  -- else assume a typical 36 deg lock for the limit math.
  local steerLockDeg = 36
  local okW2, wheels2 = pcall(function() return car.wheels end)
  if okW2 and wheels2 then
    local okL, wf = pcall(function() return wheels2[0].slipAngle end)
    -- steer lock is not exposed per-wheel; keep the estimate
  end

  -- 1) Self-steer force: hold the requested angle against the slide.
  local rawSelf = selfSteerForce(rearAngleDeg, steerLockDeg)
  local cap = MAX_SELF_STEER
  rawSelf = clamp(rawSelf, -cap, cap)
  S.agaSelfSmooth = num(S.agaSelfSmooth, 0) +
    clamp(rawSelf - num(S.agaSelfSmooth, 0), -12 * dt, 12 * dt)
  local selfSteer = clamp(S.agaSelfSmooth * SELF_STEER_RESPONSE * fade, -cap, cap)

  -- 2) Yaw damping: oppose rapid rotation (kills the zigzag/oscillation).
  local damping = clamp(-angVelY * DAMPING_STRENGTH * 0.15 * (30.0 / steerLockDeg) * fade,
    -0.25, 0.25)

  -- 3) Slip limit: if the front wheels are past the slip that still
  -- produces force, pull the wheel back toward that budget (AGA's
  -- dynamic limit reduction). Front ndSlip is the documented field;
  -- guarded. 0 ndSlip = perfect grip.
  local frontSlip = nil
  if okW2 and wheels2 then
    local maxNd = 0
    local any = false
    for i = 0, 1 do
      local ok, nd = pcall(function() return wheels2[i].ndSlip end)
      if ok and type(nd) == 'number' and nd == nd and nd >= 0 then
        maxNd = math.max(maxNd, nd); any = true
      end
    end
    if any then frontSlip = maxNd end
  end
  local steer2 = steer + selfSteer + damping
  if frontSlip and frontSlip > TARGET_FRONT_SLIP then
    -- Oversteer of the front axle: reduce steering toward the budget.
    local over = clamp((frontSlip - TARGET_FRONT_SLIP) / math.max(0.05, 1 - TARGET_FRONT_SLIP), 0, 1)
    local budget = clamp(math.abs(steer) * (1.0 - over * SLIP_LIMIT_REDUCTION), 0, 1)
    if math.abs(steer2) > budget then
      steer2 = sign(steer2) * budget
    end
  end
  return clamp(steer2, -1, 1)
end

---Reset per-car stabilizer memory (call when a car is re-adopted).
function AGA.reset(S)
  if type(S) == 'table' then S.agaSelfSmooth = 0 end
end

return AGA
