-- RoutePilot/vehicle_dynamics.lua
-- Per-car measurements adapted from the axle-velocity model used by
-- Advanced Gamepad Assist. This module is measurement-only: it does not
-- alter steering, throttle or brake commands.
local VehicleDynamics = {}

local function finite(value)
  return type(value) == "number" and value == value and
    value ~= math.huge and value ~= -math.huge
end

local function safe(value, fallback)
  return finite(value) and value or (fallback or 0)
end

local function weightedPair(a, b, wa, wb)
  wa, wb = math.max(0, safe(wa)), math.max(0, safe(wb))
  local total = wa + wb
  if total < 1e-6 then return (safe(a) + safe(b)) * 0.5 end
  return (safe(a) * wa + safe(b) * wb) / total
end

function VehicleDynamics.newState()
  return {
    wheelLocal = { [0] = vec3(), [1] = vec3(), [2] = vec3(), [3] = vec3() },
    frontAxleAngle = 0,
    rearAxleAngle = 0,
    frontSlipDeg = 0,
    rearSlipDeg = 0,
    frontNdSlip = 0,
    rearNdSlip = 0,
    wheelbase = 0,
    yawRate = 0,
    frontLongSpeed = 0,
    rearLongSpeed = 0,
    valid = false
  }
end

-- Returns the supplied per-car state, updated in place, or nil when the
-- current car does not expose the physics data required for a valid sample.
function VehicleDynamics.measure(car, state)
  if not car or not state or car.physicsAvailable == false or
      not car.wheels or not car.transform or
      not car.localVelocity or not car.localAngularVelocity then
    return nil
  end

  local wheels = car.wheels
  for i = 0, 3 do
    if not wheels[i] or not wheels[i].position then return nil end
  end

  -- Match AGA's physical-coordinate method: transform each wheel hub into
  -- the car's physics-local frame, then calculate axle-point velocity as
  -- v(point) = v(CG) + omega x r.
  local inverse = car.transform:inverse()
  if not inverse then return nil end
  for i = 0, 3 do
    inverse:transformPointTo(state.wheelLocal[i], wheels[i].position)
  end

  local front = state.wheelLocal[0]
  local frontR = state.wheelLocal[1]
  local rear = state.wheelLocal[2]
  local rearR = state.wheelLocal[3]
  local frontX = (front.x + frontR.x) * 0.5
  local frontY = (front.y + frontR.y) * 0.5
  local frontZ = (front.z + frontR.z) * 0.5
  local rearX = (rear.x + rearR.x) * 0.5
  local rearY = (rear.y + rearR.y) * 0.5
  local rearZ = (rear.z + rearR.z) * 0.5

  local lv = car.localVelocity
  local av = car.localAngularVelocity
  local function pointVelocityX(x, y, z)
    return safe(lv.x) + safe(av.y) * z - safe(av.z) * y
  end
  local function pointVelocityZ(x, y, z)
    return safe(lv.z) + safe(av.x) * y - safe(av.y) * x
  end

  local frontVX = pointVelocityX(frontX, frontY, frontZ)
  local frontVZ = pointVelocityZ(frontX, frontY, frontZ)
  local rearVX = pointVelocityX(rearX, rearY, rearZ)
  local rearVZ = pointVelocityZ(rearX, rearY, rearZ)

  state.frontAxleAngle = math.atan2(frontVX, math.max(0.05, math.abs(frontVZ)))
  state.rearAxleAngle = math.atan2(rearVX, math.max(0.05, math.abs(rearVZ)))
  state.frontLongSpeed = frontVZ
  state.rearLongSpeed = rearVZ
  state.yawRate = safe(av.y)
  state.wheelbase = math.abs(frontZ - rearZ)

  state.frontSlipDeg = weightedPair(
    wheels[0].slipAngle, wheels[1].slipAngle,
    wheels[0].load, wheels[1].load)
  state.rearSlipDeg = weightedPair(
    wheels[2].slipAngle, wheels[3].slipAngle,
    wheels[2].load, wheels[3].load)
  state.frontNdSlip = weightedPair(
    wheels[0].ndSlip, wheels[1].ndSlip,
    wheels[0].load, wheels[1].load)
  state.rearNdSlip = weightedPair(
    wheels[2].ndSlip, wheels[3].ndSlip,
    wheels[2].load, wheels[3].load)

  state.steerLockDeg = safe(car.steerLock)
  state.valid = true
  return state
end

return VehicleDynamics
