-- RoutePilot/driver.lua — deterministic two-lane traffic (CSP 0.3.2-preview2)
-- Cars are assigned once by active roster order: first = left/fast,
-- second = right/40 km/h, then alternating. No car-to-car sensing, following,
-- yielding, overtaking or collision avoidance.
-- Each car follows its own fixed lateral offset from the recorded route.

local Driver = {}
local CFG, state, H

local FAST_MAX_SPEED = 300 / 3.6
local RIGHT_SPEED = 40 / 3.6
local STEER_LIMIT = 0.78
local STEER_GAIN = 0.92
local STEER_FILTER_TIME = 0.14
local STEER_RATE = 2.2
local LOOKAHEAD_BASE = 11.0
local LOOKAHEAD_SPEED = 0.62
local LOOKAHEAD_MAX = 42.0
local LANE_RESCUE_SPEED = 1.5
local LANE_RESCUE_TIME = 5.0

local function clamp(v, lo, hi)
  return math.max(lo, math.min(hi, v))
end

local function lerp(a, b, t)
  return a + (b - a) * t
end

local function normalize2(x, z)
  local length = math.sqrt(x * x + z * z)
  if length < 0.00001 then return 0, 1 end
  return x / length, z / length
end

local function signedAngle(ax, az, bx, bz)
  local dot = clamp(ax * bx + az * bz, -1, 1)
  return math.atan2(ax * bz - az * bx, dot)
end

local function safeNumber(v, fallback)
  if type(v) ~= "number" or v ~= v or v == math.huge or v == -math.huge then
    return fallback or 0
  end
  return v
end

function Driver.setup(cfg, st, helpers)
  CFG = cfg
  state = st
  H = helpers
end

local function newPersonality(carIndex)
  local laneOffset, twoLane = H.laneOffset(carIndex, 1)
  local laneRole = not twoLane and "SINGLE_40" or
    (laneOffset > 0 and "LEFT_FAST" or "RIGHT_40")
  local p = {
    segIdx = 1,
    laneRole = laneRole,
    laneOffset = laneOffset,
    twoLane = twoLane,
    steer = 0,
    steerFiltered = 0,
    gas = 0,
    brake = 0,
    speedIntegral = 0,
    slipAngle = 0,
    previousFx = nil,
    previousFz = nil,
    stuck = 0,
    targetSpeed = RIGHT_SPEED,
    lineDist = 0,
    latOff = 0,
    retireT = 2.0
  }
  state.p[carIndex] = p
  H.log("CAR_INIT id=%d lane=%s", carIndex, p.laneRole)
  return p
end

local function readVehicleDynamics(car, p, dt)
  local look = car.look
  local fx, fz = normalize2(look.x, look.z)
  local velocity = car.velocity
  local vx = safeNumber(velocity.x)
  local vz = safeNumber(velocity.z)
  local rightX, rightZ = fz, -fx
  local longitudinal = vx * fx + vz * fz
  local lateral = vx * rightX + vz * rightZ
  p.slipAngle = math.atan2(lateral, math.max(0.5, longitudinal))
  p.previousFx, p.previousFz = fx, fz
  return math.max(0, safeNumber(car.speedMs, longitudinal)), fx, fz
end

local function planFastLaneSpeed(p)
  local nodes = state.nodes
  local segments = H.segmentCount()
  if segments < 1 then return FAST_MAX_SPEED end

  local target = FAST_MAX_SPEED
  local distance = 0
  local current = p.segIdx
  local steps = 0

  while distance <= CFG.PLAN_HORIZON and steps < 240 do
    steps = steps + 1
    local curvature = H.getVertexCurvature(current)
    if curvature > CFG.CURVE_MIN then
      local cornerSpeed = math.sqrt(CFG.MAX_LATERAL_ACCEL / curvature)
      cornerSpeed = math.max(8.0, cornerSpeed)
      local brakingSpeed = math.sqrt(cornerSpeed * cornerSpeed +
        2 * CFG.BRAKE_ACCEL * distance)
      if brakingSpeed < target then target = brakingSpeed end
    end

    local a = nodes[current]
    local b = H.segEnd(current)
    distance = distance + math.sqrt((b.x - a.x) ^ 2 + (b.z - a.z) ^ 2)

    if state.loopRoute then
      current = (current % segments) + 1
    elseif current < segments then
      current = current + 1
    else
      break
    end
  end

  return clamp(target, 8.0, FAST_MAX_SPEED)
end

local function calculateSteering(carIndex, car, p, speed, fx, fz, dt)
  local seg, t, lineDistance = H.findNearestSeg(car.position, p.segIdx)
  p.segIdx = seg
  p.lineDist = lineDistance
  p.latOff = H.lateralOf(car.position, seg)

  local laneOffset, twoLane = H.laneOffset(carIndex, seg)
  p.laneOffset = laneOffset
  p.twoLane = twoLane
  p.laneRole = not twoLane and "SINGLE_40" or
    (laneOffset > 0 and "LEFT_FAST" or "RIGHT_40")

  local lookahead = clamp(LOOKAHEAD_BASE + speed * LOOKAHEAD_SPEED,
    LOOKAHEAD_BASE, LOOKAHEAD_MAX)

  local center = H.walkLine(seg, t, lookahead)
  local centerAhead = H.walkLine(seg, t, lookahead + 2.0)
  if not center then center = H.segEnd(seg) end
  if not centerAhead then centerAhead = center end

  local tx, tz = normalize2(centerAhead.x - center.x, centerAhead.z - center.z)
  local targetX = center.x - tz * laneOffset
  local targetZ = center.z + tx * laneOffset

  local dx = targetX - car.position.x
  local dz = targetZ - car.position.z
  local distance = math.sqrt(dx * dx + dz * dz)
  if distance < 0.001 then return p.steer end

  local angle = signedAngle(fx, fz, dx / distance, dz / distance)
  local lateralError = laneOffset - p.latOff
  local targetSteer = angle * STEER_GAIN + clamp(lateralError * 0.018, -0.10, 0.10)
  targetSteer = clamp(targetSteer, -STEER_LIMIT, STEER_LIMIT)

  if math.abs(angle) < 0.0015 and math.abs(lateralError) < 0.04 then
    targetSteer = 0
  end

  local alpha = clamp(dt / (STEER_FILTER_TIME + dt), 0, 1)
  p.steerFiltered = lerp(p.steerFiltered, targetSteer, alpha)
  local maxChange = STEER_RATE * dt
  p.steer = clamp(p.steer + clamp(p.steerFiltered - p.steer,
    -maxChange, maxChange), -STEER_LIMIT, STEER_LIMIT)
  return p.steer
end

local function calculateSpeedControls(p, speed, targetSpeed, dt)
  local error = targetSpeed - speed

  -- PI speed hold: stable 40 km/h in the right lane and predictable
  -- acceleration/braking for the fast lane. Anti-windup prevents delayed
  -- braking after long full-throttle sections.
  local drive = error * 0.18 + p.speedIntegral * 0.012
  if (drive > -1 and drive < 1) or (drive >= 1 and error < 0) or
      (drive <= -1 and error > 0) then
    p.speedIntegral = clamp(p.speedIntegral + error * dt, -18, 18)
    drive = error * 0.18 + p.speedIntegral * 0.012
  end

  p.gas = clamp(drive, 0, 1)
  p.brake = clamp(-drive * 0.85, 0, 1)
  if error >= 0 then p.brake = 0 end
  if error <= 0 then p.gas = 0 end
end

function Driver.update(carIndex, car, dt)
  local p = state.p[carIndex] or newPersonality(carIndex)

  p.retireT = p.retireT + dt
  if p.retireT >= 2.0 then
    p.retireT = 0
    if physics and physics.preventAIFromRetiring then
      pcall(physics.preventAIFromRetiring, carIndex)
    end
  end

  local speed, fx, fz = readVehicleDynamics(car, p, dt)

  if speed < LANE_RESCUE_SPEED then
    p.stuck = p.stuck + dt
    if p.stuck >= LANE_RESCUE_TIME then
      p.stuck = 0
      if H.teleportHome then H.teleportHome(carIndex, car) end
      return
    end
  else
    p.stuck = 0
  end

  local steer = calculateSteering(carIndex, car, p, speed, fx, fz, dt)

  local targetSpeed
  if not p.twoLane then
    targetSpeed = RIGHT_SPEED
  elseif p.laneOffset < 0 then
    targetSpeed = RIGHT_SPEED
  else
    targetSpeed = planFastLaneSpeed(p)
  end
  p.targetSpeed = targetSpeed
  calculateSpeedControls(p, speed, targetSpeed, dt)

  local ok, controls = pcall(ac.overrideCarControls, carIndex)
  if ok and controls then
    controls.combineAxis = false
    controls.steer = steer
    controls.gas = p.gas
    controls.brake = p.brake
    controls.handbrake = 0
  end
end

function Driver.telemetryLine(i, car, p)
  local left, right = H.boundsAt(p.segIdx)
  return string.format(
    "CAR=%d LANE=%s POS=(%.2f,%.2f,%.2f) SPEED=%.2f TARGET=%.2f STEER=%.3f GAS=%.3f BRAKE=%.3f LINE_ERROR=%.2f SEGMENT=%d LANE_OFFSET=%.2f BOUNDS=[%.1f,%.1f]",
    i, p.laneRole, car.position.x, car.position.y, car.position.z,
    safeNumber(car.speedMs), p.targetSpeed, p.steer, p.gas, p.brake,
    p.lineDist, p.segIdx, p.laneOffset, left, right)
end

return Driver
