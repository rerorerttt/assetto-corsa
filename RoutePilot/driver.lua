-- RoutePilot/driver.lua
-- Per-car racing traffic controller for RoutePilot 0.4.2.
--
-- v0.4.2 — NFS-Underground-style traffic:
--  * INSTANT avoidance: the moment ANY car (or the player) is ahead in the
--    cone, the driver commits to a pass in the same frame — no 1.2 s timer.
--    The side is picked from LIVE left/right raycasts (more room wins), so
--    the car always knows it will fit.
--  * NEVER PUSHES: if both sides are blocked, the car brakes behind the
--    obstacle and can stop completely (the old version held speed+5 and
--    shoved slow cars and the player around).
--  * ARCADE CORRIDOR SPEED: the corridor cap is scaled arcade (half*1.8),
--    the floor is 8 m/s — narrow sections no longer crawl at 40 km/h.
--  * Pass ends only when fully ahead of the target.
-- v0.4.1: curvature from the smoothed line (full speed on straights),
--    wall-ahead braking, heading auto-correction, preventAIFromRetiring.

local Driver = {}

local CFG, state, H

local function clamp(v, lo, hi)
  return math.max(lo, math.min(hi, v))
end

local function lerp(a, b, t)
  return a + (b - a) * t
end

local function normalize2(x, z)
  local l = math.sqrt(x * x + z * z)
  if l < 0.00001 then return 0, 1 end
  return x / l, z / l
end

local function signedAngle(ax, az, bx, bz)
  local dot = clamp(ax * bx + az * bz, -1, 1)
  local cross = ax * bz - az * bx
  return math.atan2(cross, dot)
end

local function safeNumber(v, fallback)
  if type(v) ~= "number" then return fallback or 0 end
  if v ~= v or v == math.huge or v == -math.huge then return fallback or 0 end
  return v
end

local function carRand(carIndex, salt)
  local n = math.sin(carIndex * 127.1 + salt * 311.7) * 43758.5453
  return n - math.floor(n)
end

function Driver.setup(cfg, st, helpers)
  CFG = cfg
  state = st
  H = helpers
end

------------------------------------------------------------
-- personality
------------------------------------------------------------

local function newPersonality(carIndex)
  local pace = 0.85 + carRand(carIndex, 1) * 0.30
  local kmh = clamp(state.speedMax * pace * 0.95, 80, 280)
  local p = {
    segIdx = 1,
    pace = pace,
    cruiseSpeed = kmh / 3.6,
    steer = 0,
    steerFiltered = 0,
    gas = 0,
    brake = 0,
    slipAngle = 0,
    lateralVelocity = 0,
    lateralAccel = 0,
    yawRate = 0,
    previousFx = nil,
    previousFz = nil,
    stuck = 0,
    lineDist = 0,
    targetX = 0,
    targetZ = 0,
    targetSpeed = 0,
    slipSeverity = 0,
    stability = 1,
    lastCurve = 0,
    correctTimer = 6.0,
    overtakeSide = 0,
    overtakeTarget = nil,
    blockTimer = 0,
    retireT = 2.0,
  }
  state.p[carIndex] = p
  H.log("CAR_INIT id=%d cruise=%.1fkmh pace=%.2f", carIndex, kmh, pace)
end

------------------------------------------------------------
-- dynamics
------------------------------------------------------------

local function readVehicleDynamics(car, p, dt)
  local velocity = car.velocity
  local look = car.look
  local fx, fz = normalize2(look.x, look.z)
  local rx, rz = fz, -fx
  local vx = safeNumber(velocity.x)
  local vz = safeNumber(velocity.z)
  local longVel = vx * fx + vz * fz
  local latVel = vx * rx + vz * rz
  local slipAngle = math.atan2(latVel, math.max(0.5, longVel))
  local yawRate = 0
  if p.previousFx and dt > 0.0001 then
    yawRate = signedAngle(p.previousFx, p.previousFz, fx, fz) / dt
  end
  local speed = math.max(0, safeNumber(car.speedMs, longVel))
  p.previousFx, p.previousFz = fx, fz
  p.lateralVelocity = latVel
  p.slipAngle = slipAngle
  p.yawRate = yawRate
  p.lateralAccel = math.abs(yawRate * speed)
  return speed, fx, fz
end

local function calculateStability(p, speed)
  local slip = math.abs(p.slipAngle)
  local severity = 0
  if speed > CFG.SLIP_DAMP_MIN_SPEED and slip > CFG.SLIP_DAMP_MIN_ANGLE then
    local slipSeverity = clamp((slip - CFG.SLIP_START) / (CFG.SLIP_FULL - CFG.SLIP_START), 0, 1)
    local accelSeverity = clamp((p.lateralAccel - CFG.MAX_LATERAL_ACCEL) / (CFG.MAX_LATERAL_ACCEL * 0.6), 0, 1)
    severity = math.max(slipSeverity, accelSeverity)
  end
  p.slipSeverity = severity
  p.stability = 1 - severity
  return severity
end

------------------------------------------------------------
-- speed planner (racing tempo; arcade corridor cap)
------------------------------------------------------------

local function planTargetSpeed(p)
  local nodes = state.nodes
  local segs = H.segmentCount()
  local target = p.cruiseSpeed
  local distance = 0
  local current = p.segIdx
  local maxCurve = 0
  local steps = 0
  while distance <= CFG.PLAN_HORIZON and steps < 200 do
    steps = steps + 1
    local curvature = H.getVertexCurvature(current)
    if curvature > CFG.CURVE_MIN then
      maxCurve = math.max(maxCurve, curvature)
      local cornerSpeed = math.max(CFG.MIN_APEX_SPEED, math.sqrt(CFG.MAX_LATERAL_ACCEL / curvature))
      local safeSpeed = math.sqrt(cornerSpeed * cornerSpeed + 2 * CFG.BRAKE_ACCEL * distance)
      if safeSpeed < target then
        target = safeSpeed
      end
    end
    -- arcade corridor cap: wide half -> no cap; tight half -> arcade slowdown
    local half = H.corridorAt(current)
    if half < 8 then
      local cap = math.max(8.0, half * 1.8)
      local safeCap = math.sqrt(cap * cap + 2 * CFG.BRAKE_ACCEL * distance)
      if safeCap < target then
        target = safeCap
      end
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
  p.lastCurve = maxCurve
  return math.max(0, target)
end

------------------------------------------------------------
-- steering
------------------------------------------------------------

local function headingMatchesLine(fx, fz, seg)
  local a = state.nodes[seg]
  local b = H.segEnd(seg)
  local dx, dz = normalize2(b.x - a.x, b.z - a.z)
  return (fx * dx + fz * dz) > -0.1
end

local function clampToCorridor(seg, tx, tz)
  local a = state.nodes[seg]
  local b = H.segEnd(seg)
  local lx, lz = normalize2(b.x - a.x, b.z - a.z)
  local relX, relZ = tx - a.x, tz - a.z
  local along = relX * lx + relZ * lz
  local lat = relX * -lz + relZ * lx
  local half = math.max(2, H.corridorAt(seg) - 1.5)
  if lat > half then lat = half end
  if lat < -half then lat = -half end
  return a.x + lx * along + (-lz) * lat,
         a.z + lz * along + lx * lat
end

local function calculateSteering(car, p, speed, fx, fz, dt)
  local seg, t, lineDist = H.findNearestSegment(car, p)
  p.segIdx = seg
  p.lineDist = lineDist

  if p.correctTimer > 0 then
    p.correctTimer = p.correctTimer - dt
    if not headingMatchesLine(fx, fz, seg) then
      return (fz >= 0) and 1 or -1
    end
  end

  local lookAhead = clamp(
    CFG.LOOKAHEAD_BASE + speed * CFG.LOOKAHEAD_SPEED,
    CFG.LOOKAHEAD_MIN,
    CFG.LOOKAHEAD_MAX)
  local point = H.walkLine(seg, t, lookAhead)
  local targetX, targetZ
  if point then
    targetX, targetZ = point.x, point.z
  else
    local b = H.segEnd(seg)
    targetX, targetZ = b.x, b.z
  end

  -- lateral shift while passing an obstacle
  if p.overtakeSide ~= 0 then
    local shift = math.min(CFG.OVERTAKE_LATERAL, math.max(3.0, H.corridorAt(seg) - 1.5))
    local nx, nz = normalize2(-(targetZ - car.position.z), targetX - car.position.x)
    targetX = targetX + nx * p.overtakeSide * shift
    targetZ = targetZ + nz * p.overtakeSide * shift
  end

  targetX, targetZ = clampToCorridor(seg, targetX, targetZ)
  p.targetX = targetX
  p.targetZ = targetZ

  local dx = targetX - car.position.x
  local dz = targetZ - car.position.z
  local distance = math.sqrt(dx * dx + dz * dz)
  if distance < 0.001 then
    return p.steer
  end
  local angle = signedAngle(fx, fz, dx / distance, dz / distance)
  local steer = angle * CFG.STEER_GAIN
    + p.slipAngle * 0.62
    - p.yawRate * 0.05

  -- hard pull back if far off the line
  if p.overtakeSide == 0 and lineDist > math.max(10, H.corridorAt(seg)) then
    local a = state.nodes[seg]
    local lx, lz = normalize2(targetX - a.x, targetZ - a.z)
    local side = (car.position.x - a.x) * -lz + (car.position.z - a.z) * lx
    if side > 0 then
      steer = steer - 0.5
    else
      steer = steer + 0.5
    end
  end

  return clamp(steer, -CFG.STEER_LIMIT, CFG.STEER_LIMIT)
end

local function filterSteering(p, target, dt)
  local alpha = clamp(dt / (CFG.STEER_FILTER + dt), 0, 1)
  local filtered = lerp(p.steerFiltered, target, alpha)
  local maxChange = CFG.STEER_RATE * dt
  local delta = clamp(filtered - p.steer, -maxChange, maxChange)
  p.steer = clamp(p.steer + delta, -CFG.STEER_LIMIT, CFG.STEER_LIMIT)
  p.steerFiltered = filtered
  return p.steer
end

------------------------------------------------------------
-- NFS-style obstacle avoidance (instant decision)
------------------------------------------------------------

local function updateOvertake(carIndex, car, p, speed, targetSpeed, dt)
  local gap, leaderSpeed, leaderIndex = H.carAhead(carIndex, car, CFG.AVOID_LATERAL)

  -- currently passing: finish only when fully ahead of the target
  if p.overtakeSide ~= 0 then
    local other = p.overtakeTarget and ac.getCar(p.overtakeTarget) or nil
    if other and other.isActive then
      local dx = other.position.x - car.position.x
      local dz = other.position.z - car.position.z
      local fx, fz = normalize2(car.look.x, car.look.z)
      local forward = dx * fx + dz * fz
      if forward < -CFG.OVERTAKE_CLEAR_AHEAD then
        p.overtakeSide = 0
        p.overtakeTarget = nil
        H.log("CAR=%d OVERTAKE_END", carIndex)
      end
      -- keep the racing pace through the pass
      return math.max(targetSpeed, math.min(p.cruiseSpeed, (leaderSpeed or 0) + 8))
    end
    -- target vanished: done
    p.overtakeSide = 0
    p.overtakeTarget = nil
  end

  if not gap then
    p.blockTimer = 0
    return targetSpeed
  end

  -- obstacle ahead (car OR player, any speed): decide INSTANTLY
  local needPass = (leaderSpeed or 0) + 2 < speed or (gap or 0) < 30
  if not needPass then
    return targetSpeed
  end

  -- pick the side with more live room (rays from THIS car, right now)
  local roomL, roomR = H.liveRoomLeftRight(car)
  local side = 0
  if roomL >= CFG.PASS_MIN_ROOM and roomL >= roomR then
    side = -1
  elseif roomR >= CFG.PASS_MIN_ROOM then
    side = 1
  end

  if side ~= 0 then
    -- commit: full racing pace through the pass
    p.overtakeTarget = leaderIndex
    p.overtakeSide = side
    p.blockTimer = 0
    H.log("CAR=%d OVERTAKE_BEGIN side=%d gap=%.1f roomL=%.1f roomR=%.1f leader=%d",
      carIndex, side, gap or -1, roomL, roomR, leaderIndex or -1)
    return math.max(targetSpeed, math.min(p.cruiseSpeed, (leaderSpeed or 0) + 8))
  end

  -- both sides blocked: brake behind and be ready to stop — never push
  local safeSpeed = math.sqrt(math.max(0,
    (leaderSpeed or 0) * (leaderSpeed or 0) +
    2 * CFG.BRAKE_ACCEL * math.max(0, (gap or 0) - state.followGap)))
  if (gap or 0) < state.followGap then
    return 0
  end
  return math.min(targetSpeed, safeSpeed)
end

------------------------------------------------------------
-- throttle / brake
------------------------------------------------------------

local function calculateThrottle(p, speed, targetSpeed)
  local error = targetSpeed - speed
  local gas = clamp(error * CFG.GAS_GAIN, 0, CFG.MAX_GAS)
  local brake = clamp(-error * CFG.BRAKE_GAIN, 0, CFG.MAX_BRAKE)
  gas = gas * lerp(1.0, 0.05, p.slipSeverity)
  if error >= 0 then
    brake = 0
  end
  p.gas = clamp(gas, 0, CFG.MAX_GAS)
  p.brake = clamp(brake, 0, CFG.MAX_BRAKE)
  return p.gas, p.brake
end

------------------------------------------------------------
-- main per-car update
------------------------------------------------------------

function Driver.update(carIndex, car, dt)
  local p = state.p[carIndex]
  if not p then
    newPersonality(carIndex)
    p = state.p[carIndex]
  end

  -- keep CSP from retiring (despawning) this AI car
  p.retireT = p.retireT + dt
  if p.retireT >= 2.0 then
    p.retireT = 0
    if physics and physics.preventAIFromRetiring then
      pcall(physics.preventAIFromRetiring, carIndex)
    end
  end

  local speed, fx, fz = readVehicleDynamics(car, p, dt)
  calculateStability(p, speed)

  local targetSpeed = planTargetSpeed(p)

  -- wall ahead: brake before hitting it
  if H.wallAhead then
    local wallDist = H.wallAhead(car)
    if wallDist and wallDist < 70 then
      local vStop = math.sqrt(math.max(0, 2 * CFG.BRAKE_ACCEL * (wallDist - 3)))
      if vStop < targetSpeed then
        targetSpeed = vStop
      end
    end
  end

  targetSpeed = updateOvertake(carIndex, car, p, speed, targetSpeed, dt)
  p.targetSpeed = targetSpeed

  local steerTarget = calculateSteering(car, p, speed, fx, fz, dt)
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
  return string.format(
    "CAR=%d POS=(%.2f,%.2f,%.2f) SPEED=%.2f STEER=%.3f GAS=%.3f BRAKE=%.3f SLIP=%.4f STABILITY=%.3f CURVATURE=%.5f TARGET_SPEED=%.2f LINE_ERROR=%.2f SEGMENT=%d PACE=%.2f CORR=%.1f OT_SIDE=%d",
    i, car.position.x, car.position.y, car.position.z,
    safeNumber(car.speedMs), p.steer, p.gas, p.brake,
    p.slipAngle, p.stability, p.lastCurve, p.targetSpeed,
    p.lineDist, p.segIdx, p.pace,
    H.corridorAt and H.corridorAt(p.segIdx) or -1,
    p.overtakeSide)
end

return Driver
