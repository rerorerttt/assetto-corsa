-- RoutePilot/driver.lua
-- Per-car racing traffic controller for RoutePilot 0.4.
--
-- The main script sets up context (CFG, state, helpers) via Driver.setup()
-- and then calls Driver.update(carIndex, car, dt) every frame for each AI car.
--
-- Key behaviors (v0.4):
--  * RACING TEMPO: plans speed by scanning the line ahead for corners and
--    braking exactly as late as the arcade braking power allows; per-car
--    pace (0.85..1.15) makes fast cars overtake slow ones naturally.
--  * LOW-SPEED FIX: slip-based throttle damping engages only above 8 m/s
--    and above 4 degrees of real slip (the old version saw "slip" in the
--    noise at walking speed and crawled).
--  * HEADING FIX after teleport: if the car faces backwards along the
--    line, it does a steady full-lock turn instead of oscillating.
--  * OVERTAKING: instead of queuing behind a slower car, the car shifts
--    laterally off the line and passes, then returns to the line.
--  * COLLISION SAFETY: hard braking only when the space ahead is really
--    occupied (checks every car including the player); the slow car keeps
--    its line and does not brake for the passer.

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

-- deterministic per-car random
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
-- line geometry helpers (provided by the main script through H)
-- H.segEnd(index)           -> node at segment end
-- H.segmentCount()          -> number of segments
-- H.walkLine(seg, t, dist)   -> point ahead on the line
-- H.flatDist(a, b)           -> flat distance
-- H.getVertexCurvature(i)    -> curvature of vertex i
-- H.findNearestSegment(car, p) -> seg, t, distToLine, segs
-- H.carAhead(carIndex, car, lateralWidth) -> gap, leaderSpeed
------------------------------------------------------------

------------------------------------------------------------
-- personality
------------------------------------------------------------

local function newPersonality(carIndex)
  local span = math.max(0, state.speedMax - state.speedMin)
  local pace = 0.85 + carRand(carIndex, 1) * 0.30
  local kmh = clamp(state.speedMin + span * 0.5, 40, 260) * pace
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
    correctTimer = 6.0,          -- heading auto-correction window
    overtakeSide = 0,
    overtakeTarget = nil,
    followTimer = 0,
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
  -- FIX: only real slip at real speeds damps the throttle
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
-- speed planner (racing tempo)
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
    if curvature > 0.0008 then
      maxCurve = math.max(maxCurve, curvature)
      local cornerSpeed = math.max(CFG.MIN_APEX_SPEED, math.sqrt(CFG.MAX_LATERAL_ACCEL / curvature))
      local safeSpeed = math.sqrt(cornerSpeed * cornerSpeed + 2 * CFG.BRAKE_ACCEL * distance)
      if safeSpeed < target then
        target = safeSpeed
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

local function calculateSteering(car, p, speed, fx, fz, dt)
  local seg, t, lineDist = H.findNearestSegment(car, p)
  p.segIdx = seg
  p.lineDist = lineDist

  -- heading auto-correction right after teleport
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
  p.targetX = targetX
  p.targetZ = targetZ

  -- lateral shift while overtaking
  if p.overtakeSide ~= 0 then
    local nx, nz = normalize2(-(targetZ - car.position.z), targetX - car.position.x)
    targetX = targetX + nx * p.overtakeSide * CFG.OVERTAKE_LATERAL
    targetZ = targetZ + nz * p.overtakeSide * CFG.OVERTAKE_LATERAL
  end

  local dx = targetX - car.position.x
  local dz = targetZ - car.position.z
  local distance = math.sqrt(dx * dx + dz * dz)
  if distance < 0.001 then
    return p.steer
  end
  local angle = signedAngle(fx, fz, dx / distance, dz / distance)
  local steer = angle * CFG.STEER_GAIN + p.slipAngle * 0.62

  -- keep the car from straying far from the line (except when overtaking)
  if p.overtakeSide == 0 and lineDist > CFG.LINE_HARD_PULL then
    local a = state.nodes[seg]
    local b = H.segEnd(seg)
    local lx, lz = normalize2(b.x - a.x, b.z - a.z)
    local side = (car.position.x - a.x) * -lz + (car.position.z - a.z) * lx
    if side > 0 then
      steer = steer - 0.4
    else
      steer = steer + 0.4
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
-- overtaking decision
------------------------------------------------------------

local function updateOvertake(carIndex, car, p, speed, targetSpeed)
  local gap, leaderSpeed = H.carAhead(carIndex, car, CFG.FOLLOW_LATERAL)

  if p.overtakeSide ~= 0 then
    -- passing in progress: end it when well clear of the target
    if p.overtakeTarget then
      local other = ac.getCar(p.overtakeTarget)
      if other and other.isActive then
        local dx = other.position.x - car.position.x
        local dz = other.position.z - car.position.z
        local fx, fz = normalize2(car.look.x, car.look.z)
        local forward = dx * fx + dz * fz
        local lateral = math.abs(dx * fz - dz * fx)
        if forward < -CFG.OVERTAKE_CLEAR_AHEAD or lateral > CFG.OVERTAKE_LATERAL * 2 then
          p.overtakeSide = 0
          p.overtakeTarget = nil
          H.log("CAR=%d OVERTAKE_END", carIndex)
        end
        return targetSpeed
      end
    end
    p.overtakeSide = 0
    p.overtakeTarget = nil
    return targetSpeed
  end

  if not gap then
    p.followTimer = 0
    return targetSpeed
  end

  -- a slower car ahead within following distance
  if leaderSpeed + 3 < speed and gap < 60 then
    p.followTimer = p.followTimer + 1 / 60
    if p.followTimer > CFG.OVERTAKE_TRIGGER_TIME then
      p.followTimer = 0
      p.overtakeTarget = H.carAheadIndex and H.carAheadIndex(carIndex, car, CFG.FOLLOW_LATERAL * 2)
      p.overtakeSide = (carRand(carIndex, 2) > 0.5) and 1 or -1
      H.log("CAR=%d OVERTAKE_BEGIN side=%d gap=%.1f", carIndex, p.overtakeSide, gap or -1)
    end
    -- do NOT brake behind a slower car — keep the pace and go around
    return math.max(targetSpeed, leaderSpeed + 5)
  end

  -- same-speed or faster car ahead: classic safe following
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
  -- progressive power cut only on real slides
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

  local speed, fx, fz = readVehicleDynamics(car, p, dt)
  calculateStability(p, speed)

  local targetSpeed = planTargetSpeed(p)
  targetSpeed = updateOvertake(carIndex, car, p, speed, targetSpeed)
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
    "CAR=%d POS=(%.2f,%.2f,%.2f) SPEED=%.2f STEER=%.3f GAS=%.3f BRAKE=%.3f SLIP=%.4f STABILITY=%.3f CURVATURE=%.5f TARGET_SPEED=%.2f LINE_ERROR=%.2f SEGMENT=%d PACE=%.2f OT_SIDE=%d",
    i, car.position.x, car.position.y, car.position.z,
    safeNumber(car.speedMs), p.steer, p.gas, p.brake,
    p.slipAngle, p.stability, p.lastCurve, p.targetSpeed,
    p.lineDist, p.segIdx, p.pace, p.overtakeSide)
end

return Driver
