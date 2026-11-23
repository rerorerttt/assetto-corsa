-- RoutePilot/driver.lua — v0.5.1 (CSP 0.3.2-preview2)
--
-- v0.5.1 fixes:
--  * ZIGZAG ROOT CAUSE: the pass offset was computed by rotating the
--    car->target vector 90 degrees — that vector flips sign when the target
--    is straight ahead, throwing the steering left/right every other frame.
--    The pass target is now computed in the STABLE line frame (lateral
--    coordinate from the line), so it cannot oscillate.
--  * NO TELEPORTS WHILE MOVING: rescue fires ONLY after the car has been
--    genuinely stuck (speed < 1.5 m/s for 3.5 s). Out-of-bounds cars are
--    steered back, not teleported.
--  * Blocker lateral offset is measured with a proper segment hint (the old
--    code used segment 1 for far-away cars — garbage values caused phantom
--    overtakes at spawn).

local Driver = {}

local CFG, state, H

local BOUND_MARGIN    = 1.0
local CENTER_PULL     = 0.35
local TRIGGER_TIME    = 0.6
local FORCE_TIME      = 2.5
local PASS_TIMEOUT    = 6.0
local PASS_WIDTH      = 2.6
local CAR_HALF        = 1.1
local RESCUE_SPEED    = 1.5
local RESCUE_TIME     = 3.5
local OUTSIDE_LIMIT   = 6.0

local function clamp(v, lo, hi) return math.max(lo, math.min(hi, v)) end
local function lerp(a, b, t) return a + (b - a) * t end
local function normalize2(x, z)
  local l = math.sqrt(x * x + z * z)
  if l < 0.00001 then return 0, 1 end
  return x / l, z / l
end
local function signedAngle(ax, az, bx, bz)
  local dot = clamp(ax * bx + az * bz, -1, 1)
  return math.atan2(ax * bz - az * bx, dot)
end
local function safeNumber(v, fb)
  if type(v) ~= "number" then return fb or 0 end
  if v ~= v or v == math.huge or v == -math.huge then return fb or 0 end
  return v
end
local function carRand(i, s)
  local n = math.sin(i * 127.1 + s * 311.7) * 43758.5453
  return n - math.floor(n)
end

function Driver.setup(cfg, st, helpers)
  CFG = cfg
  state = st
  H = helpers
end

local function newPersonality(carIndex)
  local pace = 0.86 + carRand(carIndex, 1) * 0.28
  local kmh = clamp(safeNumber(state.speedMax, 170) * pace, 80, 240)
  local p = {
    segIdx = 1, pace = pace, cruiseSpeed = kmh / 3.6,
    steer = 0, steerFiltered = 0, gas = 0, brake = 0,
    slipAngle = 0, lateralAccel = 0, yawRate = 0,
    previousFx = nil, previousFz = nil,
    stuck = 0, blockedT = 0, passT = 0,
    lineDist = 0, latOff = 0, targetSpeed = 0,
    slipSeverity = 0, stability = 1, lastCurve = 0,
    overtakeSide = 0, passLat = 0, blockerOff = 0, followTimer = 0,
    retireT = 2.0,
  }
  state.p[carIndex] = p
  H.log("CAR_INIT id=%d cruise=%.1fkmh pace=%.2f", carIndex, kmh, pace)
end

local function readVehicleDynamics(car, p, dt)
  local v = car.velocity
  local look = car.look
  local fx, fz = normalize2(look.x, look.z)
  local rx, rz = fz, -fx
  local vx, vz = safeNumber(v.x), safeNumber(v.z)
  local longVel = vx * fx + vz * fz
  local latVel = vx * rx + vz * rz
  p.slipAngle = math.atan2(latVel, math.max(0.5, longVel))
  p.yawRate = 0
  if p.previousFx and dt > 0.0001 then
    p.yawRate = signedAngle(p.previousFx, p.previousFz, fx, fz) / dt
  end
  local speed = math.max(0, safeNumber(car.speedMs, longVel))
  p.previousFx, p.previousFz = fx, fz
  p.lateralAccel = math.abs(p.yawRate * speed)
  return speed, fx, fz
end

local function calculateStability(p, speed)
  local slip = math.abs(p.slipAngle)
  local severity = 0
  if speed > CFG.SLIP_DAMP_MIN_SPEED and slip > CFG.SLIP_DAMP_MIN_ANGLE then
    local ss = clamp((slip - CFG.SLIP_START) / (CFG.SLIP_FULL - CFG.SLIP_START), 0, 1)
    local as = clamp((p.lateralAccel - CFG.MAX_LATERAL_ACCEL) / (CFG.MAX_LATERAL_ACCEL * 0.7), 0, 1)
    severity = math.max(ss, as) * 0.7
  end
  p.slipSeverity = severity
  p.stability = 1 - severity
end

local function planTargetSpeed(p)
  local nodes = state.nodes
  local segs = H.segmentCount()
  local target = p.cruiseSpeed
  local distance = 0
  local current = p.segIdx
  local steps = 0
  while distance <= CFG.PLAN_HORIZON and steps < 200 do
    steps = steps + 1
    local curvature = H.getVertexCurvature(current)
    if curvature > CFG.CURVE_MIN then
      local cs = math.max(CFG.MIN_APEX_SPEED, math.sqrt(CFG.MAX_LATERAL_ACCEL / curvature))
      local ss = math.sqrt(cs * cs + 2 * CFG.BRAKE_ACCEL * distance)
      if ss < target then target = ss end
    end
    local a = nodes[current]
    local b = H.segEnd(current)
    distance = distance + math.sqrt((b.x - a.x) ^ 2 + (b.z - a.z) ^ 2)
    if state.loopRoute then
      current = (current % segs) + 1
    elseif current < segs then
      current = current + 1
    else
      break
    end
  end
  return math.max(0, target)
end

local function linePointAtLateral(seg, t, ahead, lat)
  local pt = H.walkLine(seg, t, ahead)
  local a = state.nodes[seg]
  local b = H.segEnd(seg)
  local lx, lz = normalize2(b.x - a.x, b.z - a.z)
  local bx, bz = pt.x, pt.z
  return bx - lz * lat, bz + lx * lat
end

local function clampLat(seg, lat)
  local bl, br = H.boundsAt(seg)
  return clamp(lat, bl + BOUND_MARGIN, br - BOUND_MARGIN)
end

local function calculateSteering(car, p, speed, fx, fz)
  local seg, t, lineDist = H.findNearestSeg(car.position, p.segIdx)
  p.segIdx = seg
  p.lineDist = lineDist
  local latOff = H.lateralOf(car.position, seg)
  p.latOff = latOff

  local ahead = clamp(
    CFG.LOOKAHEAD_BASE + speed * CFG.LOOKAHEAD_SPEED,
    CFG.LOOKAHEAD_MIN, CFG.LOOKAHEAD_MAX)

  local wantLat = 0
  if p.overtakeSide ~= 0 then
    wantLat = p.passLat
  end
  wantLat = clampLat(seg, wantLat)

  local tx, tz = linePointAtLateral(seg, t, ahead, wantLat)

  local dx = tx - car.position.x
  local dz = tz - car.position.z
  local dist = math.sqrt(dx * dx + dz * dz)
  if dist < 0.001 then return p.steer end
  local steer = signedAngle(fx, fz, dx / dist, dz / dist) * CFG.STEER_GAIN
    - p.yawRate * 0.04

  if p.overtakeSide == 0 then
    local pull = clamp(latOff / 12, -1, 1) * CENTER_PULL
    steer = steer - pull
  end
  local bl, br = H.boundsAt(seg)
  if latOff > br + OUTSIDE_LIMIT then steer = -1 end
  if latOff < bl - OUTSIDE_LIMIT then steer = 1 end

  return clamp(steer, -CFG.STEER_LIMIT, CFG.STEER_LIMIT)
end

local function filterSteering(p, target, dt)
  local alpha = clamp(dt / (CFG.STEER_FILTER + dt), 0, 1)
  p.steerFiltered = lerp(p.steerFiltered, target, alpha)
  local maxChange = CFG.STEER_RATE * dt
  p.steer = clamp(p.steer + clamp(p.steerFiltered - p.steer, -maxChange, maxChange),
    -CFG.STEER_LIMIT, CFG.STEER_LIMIT)
  return p.steer
end

local function beginOvertake(carIndex, p, gap)
  p.followTimer = 0
  p.blockedT = 0
  p.passT = 0
  local seg = p.segIdx
  local bl, br = H.boundsAt(seg)
  local laneR = p.blockerOff + (CAR_HALF + PASS_WIDTH)
  local laneL = p.blockerOff - (CAR_HALF + PASS_WIDTH)
  local side
  if laneR <= br - BOUND_MARGIN and laneL >= bl + BOUND_MARGIN then
    side = (carRand(carIndex, 2) > 0.5) and 1 or -1
  elseif laneR <= br - BOUND_MARGIN then
    side = 1
  elseif laneL >= bl + BOUND_MARGIN then
    side = -1
  else
    return false
  end
  p.overtakeSide = side
  p.passLat = clampLat(seg, (side > 0) and laneR or laneL)
  H.log("CAR=%d OVERTAKE_BEGIN side=%d lane=%.1f gap=%.1f",
    carIndex, side, p.passLat, gap or -1)
  return true
end

local function updateOvertake(carIndex, car, p, speed, targetSpeed, dt)
  local gap, leaderSpeed, leaderIndex = H.carAheadFull(carIndex, car, 5.0)

  if leaderIndex then
    local other = ac.getCar(leaderIndex)
    if other then
      local bseg = select(1, H.findNearestSeg(other.position, p.segIdx))
      p.blockerOff = H.lateralOf(other.position, bseg)
    end
  end

  if p.overtakeSide ~= 0 then
    p.passT = p.passT + dt
    if not gap then
      p.overtakeSide = 0
      H.log("CAR=%d OVERTAKE_END clear", carIndex)
      return targetSpeed
    end
    if p.passT > PASS_TIMEOUT then
      p.overtakeSide = 0
      H.log("CAR=%d OVERTAKE_END timeout", carIndex)
      return targetSpeed
    end
    local bl, br = H.boundsAt(p.segIdx)
    local okWindow = (p.passLat > p.blockerOff and p.passLat <= br - 0.3)
      or (p.passLat < p.blockerOff and p.passLat >= bl + 0.3)
    if not okWindow then
      p.overtakeSide = 0
      H.log("CAR=%d OVERTAKE_END closed_window", carIndex)
      return targetSpeed
    end
    local safe = math.sqrt(math.max(0,
      (leaderSpeed or 0) * (leaderSpeed or 0) +
      2 * CFG.BRAKE_ACCEL * math.max(0, (gap or 0) - 5)))
    return math.min(math.max(targetSpeed, (leaderSpeed or 0) + 4), math.max(safe, 6))
  end

  if not gap then
    p.followTimer = 0
    p.blockedT = 0
    return targetSpeed
  end

  if (leaderSpeed or 0) < math.min(p.cruiseSpeed, targetSpeed + 3) - 2 then
    p.followTimer = p.followTimer + dt
    local safe = math.sqrt(math.max(0,
      (leaderSpeed or 0) * (leaderSpeed or 0) +
      2 * CFG.BRAKE_ACCEL * math.max(0, (gap or 0) - safeNumber(state.followGap, 8))))
    if p.followTimer > TRIGGER_TIME then
      if beginOvertake(carIndex, p, gap) then
        return math.max(targetSpeed, (leaderSpeed or 0) + 4)
      end
    end
    if speed < 3 then
      p.blockedT = p.blockedT + dt
      if p.blockedT > FORCE_TIME and beginOvertake(carIndex, p, gap) then
        return math.max(targetSpeed, 6)
      end
    end
    return math.min(targetSpeed, safe)
  end

  p.followTimer = 0
  p.blockedT = 0
  local safe = math.sqrt(math.max(0,
    (leaderSpeed or 0) * (leaderSpeed or 0) +
    2 * CFG.BRAKE_ACCEL * math.max(0, (gap or 0) - safeNumber(state.followGap, 8))))
  if (gap or 0) < safeNumber(state.followGap, 8) then
    return 0
  end
  return math.min(targetSpeed, safe)
end

local function calculateThrottle(p, speed, targetSpeed)
  local error = targetSpeed - speed
  local gas = clamp(error * CFG.GAS_GAIN, 0, CFG.MAX_GAS)
  local brake = clamp(-error * CFG.BRAKE_GAIN, 0, CFG.MAX_BRAKE)
  gas = gas * lerp(1.0, 0.15, p.slipSeverity)
  if error >= 0 then brake = 0 end
  p.gas = clamp(gas, 0, CFG.MAX_GAS)
  p.brake = clamp(brake, 0, CFG.MAX_BRAKE)
end

function Driver.update(carIndex, car, dt)
  local p = state.p[carIndex]
  if not p then
    newPersonality(carIndex)
    p = state.p[carIndex]
  end

  p.retireT = p.retireT + dt
  if p.retireT >= 2.0 then
    p.retireT = 0
    if physics and physics.preventAIFromRetiring then
      pcall(physics.preventAIFromRetiring, carIndex)
    end
  end

  local speed, fx, fz = readVehicleDynamics(car, p, dt)
  calculateStability(p, speed)

  if speed < RESCUE_SPEED then
    p.stuck = p.stuck + dt
    if p.stuck > RESCUE_TIME then
      p.stuck = 0
      if H.teleportHome then H.teleportHome(carIndex, car) end
      return
    end
  else
    p.stuck = 0
  end

  local targetSpeed = planTargetSpeed(p)
  targetSpeed = updateOvertake(carIndex, car, p, speed, targetSpeed, dt)
  p.targetSpeed = targetSpeed

  local steerTarget = calculateSteering(car, p, speed, fx, fz)
  local steer = filterSteering(p, steerTarget, dt)

  calculateThrottle(p, speed, targetSpeed)

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
  local bl, br = H.boundsAt(p.segIdx)
  return string.format(
    "CAR=%d POS=(%.2f,%.2f,%.2f) SPEED=%.2f STEER=%.3f GAS=%.3f BRAKE=%.3f TARGET_SPEED=%.2f LINE_ERROR=%.2f SEGMENT=%d PACE=%.2f BOUNDS=[%.1f,%.1f] OFF=%.1f OT_SIDE=%d",
    i, car.position.x, car.position.y, car.position.z,
    safeNumber(car.speedMs), p.steer, p.gas, p.brake, p.targetSpeed,
    p.lineDist, p.segIdx, p.pace, bl, br, p.latOff, p.overtakeSide)
end

return Driver
