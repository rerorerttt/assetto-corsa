-- RoutePilot/vehicle_dynamics.lua
-- AGA-inspired autonomous stability correction.
-- Uses the actual rear-axle velocity (body-local velocity plus angular
-- velocity at the rear axle) rather than treating body heading as motion.
local M = {}

local function clamp(v, lo, hi)
  return math.max(lo, math.min(hi, v))
end

local function safe(v, fallback)
  if type(v) ~= "number" or v ~= v or v == math.huge or v == -math.huge then
    return fallback or 0
  end
  return v
end

local function rearAxleOffset(car, p)
  if p.rearAxleZ then return p.rearAxleZ end
  local ok, z = pcall(function()
    local inverse = car.transform:inverse()
    local left = inverse:transformPoint(car.wheels[2].position)
    local right = inverse:transformPoint(car.wheels[3].position)
    return (left.z + right.z) * 0.5
  end)
  if ok and type(z) == "number" and z == z then
    p.rearAxleZ = z
  else
    -- Conservative fallback only for cars whose wheel transforms are absent.
    p.rearAxleZ = -math.max(0.7, safe(car.aabbSize and car.aabbSize.z, 4.5) * 0.25)
  end
  return p.rearAxleZ
end

function M.correctSteering(car, p, pathSteer, speed, dt)
  if not car or not p then return pathSteer end
  dt = clamp(safe(dt, 0.016), 0.001, 0.05)
  speed = math.max(0, safe(speed))
  local localVelocity = car.localVelocity
  local angular = car.localAngularVelocity
  if not localVelocity or not angular then
    p.agaCorrection = 0
    return pathSteer
  end

  -- AGA obtains axle velocity from rigid-body point velocity:
  -- v(point) = v(COM) + angularVelocity x pointOffset.
  local rearZ = rearAxleOffset(car, p)
  local rearVx = safe(localVelocity.x) + safe(angular.y) * rearZ
  local rearVz = safe(localVelocity.z)
  local rearSpeed = math.sqrt(rearVx * rearVx + rearVz * rearVz)
  local rearAngle = 0
  if rearSpeed > 1.5 then
    rearAngle = math.atan2(rearVx, math.abs(rearVz))
  end

  -- Rear-axle slip supplies the self-steer direction; yaw-rate feedback
  -- damps rotation. The gains are deliberately bounded and smoothed so
  -- this layer stabilizes path tracking instead of replacing the planner.
  local yawRate = safe(p.yawRateFiltered, safe(p.yawRate))
  local raw = -rearAngle * 0.85 - yawRate * 0.10
  local slipMagnitude = math.abs(rearAngle)
  local correctionCap = 0.16 + clamp((slipMagnitude - 0.035) / 0.30, 0, 1) * 0.29
  local speedFade = clamp((speed - 3.0) / 7.0, 0, 1)
  raw = clamp(raw, -correctionCap, correctionCap) * speedFade

  local alpha = clamp(dt / (0.055 + dt), 0, 1)
  p.agaCorrectionFiltered = safe(p.agaCorrectionFiltered) +
    (raw - safe(p.agaCorrectionFiltered)) * alpha
  p.agaCorrection = p.agaCorrectionFiltered
  p.rearAxleSlip = rearAngle
  p.rearAxleSpeed = rearSpeed

  return clamp(pathSteer + p.agaCorrection, -1, 1)
end

return M
