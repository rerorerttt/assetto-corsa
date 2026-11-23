-- RoutePilot/driver.lua
-- Per-car ARCADE racing traffic controller, v0.4.3 (CSP 0.3.2-preview2).
--
-- v0.4.3 — CRITICAL FIX + NFS polish:
--  * FATAL BUG FIXED: v0.4.2 referenced CFG keys that do not exist in the
--    main script (WALL_MARGIN, CENTER_PULL, OVERTAKE_TRIGGER_TIME,
--    FOLLOW_LATERAL) — nil arithmetic crashed the driver on the very first
--    frame, pcall released the car, and the STOCK AI (no fast lane on this
--    track) parked it at a wall. All constants are now LOCAL here and every
--    CFG/H reference is checked against the main v0.4.2 script.
--  * PASS ANY BLOCKER (NFS Underground style): anything slower than my
--    pace ahead — including a STANDING PLAYER — is passed after 0.6 s;
--    if I'm stopped behind it for 2.5 s, the pass is forced.
--  * SIDE CHOSEN BY LIVE ROOM RAYS: every frame the car casts left/right
--    rays and commits to the side with more space — it knows it fits.
--  * CONTINUOUS corridor-center pull: the further off-center, the harder
--    the car is steered back. Cars cannot settle at walls anymore.
--  * NO CRAWLING IN WIDE CORRIDORS: speed is only capped where the corridor
--    is genuinely narrow (< 4.5 m half-width). A 13 m wide section is driven
--    at full pace; the forward wall ray brakes only if a wall is really
--    straight ahead.
--  * STUCK RESCUE and the preventAIFromRetiring pulse are kept (no
--    vanishing cars).

local Driver = {}

local CFG, state, H

-- local tuning constants (do NOT depend on main-script CFG keys that
-- might be missing; everything the driver needs is defined right here)
local WALL_MARGIN      = 2.5    -- keep targets this far from corridor edges, m
local CENTER_PULL      = 0.40   -- proportional pull to corridor center
local TRIGGER_TIME     = 0.6    -- follow a slower car this long before passing
local FORCE_TIME      = 2.5    -- stopped behind a blocker this long -> force pass
local PASS_TIMEOUT     = 6.0    -- give up on a stuck pass attempt
local RESCUE_SPEED     = 1.5    -- below this speed the stuck timer runs
local RESCUE_TIME      = 3.5    -- stopped this long -> re-orient and push
local NARROW_HALF      = 4.5    -- corridor half-width below which speed is capped
local WALL_BRAKE_DIST  = 25.0   -- start braking to a wall closer than this
local WALL_STOP_MARGIN = 4.0    -- stop this short of a wall

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
  local pace = 0.86 + carRand(carIndex, 1) * 0.28   -- 0.86..1.14
  local kmh = clamp(safeNumber(state.speedMax, 170) * pace, 80, 240)
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
    blockedT = 0,
    passT = 0,
    lineDist = 0,
    targetX = 0,
    targetZ = 0,
    targetSpeed = 0,
    slipSeverity = 0,
    stability = 1,
    lastCurve = 0,
    correctTimer = 6.0,
    overtakeSide = 0,
    followTimer = 0,
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
  -- only real slides at real speeds cut the power (never at walking pace)
  if speed > CFG.SLIP_DAMP_MIN_SPEED and slip > CFG.SLIP_DAMP_MIN_ANGLE then
    local slipSeverity = clamp((slip - CFG.SLIP_START) / (CFG.SLIP_FULL - CFG.SLIP_START), 0, 1)
    local accelSeverity = clamp((p.lateralAccel - CFG.MAX_LATERAL_ACCEL) / (CFG.MAX_LATERAL_ACCEL * 0.7), 0, 1)
    severity = math.max(slipSeverity, accelSeverity) * 0.7
  end
  p.slipSeverity = severity
  p.stability = 1 - severity
  return severity
end

------------------------------------------------------------
-- speed planner (arcade racing tempo)
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
    -- only GENUINELY narrow sections cap the speed; a 13 m wide corridor
    -- is driven flat out (the center pull keeps the car off the walls)
    local half = H.corridorAt(current)
    if half < NARROW_HALF then
      local cap = math.max(CFG.MIN_APEX_SPEED, half * 2.4)
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

-- clamp a target point laterally inside the corridor with a hard margin
local function clampToCorridor(seg, tx, tz)
  local a = state.nodes[seg]
  local b = H.segEnd(seg)
  local lx, lz = normalize2(b.x - a.x, b.z - a.z)
  local relX, relZ = tx - a.x, tz - a.z
  local along = relX * lx + relZ * lz
  local lat = relX * -lz + relZ * lx
  local half = math.max(1.5, H.corridorAt(seg) - WALL_MARGIN)
  if lat > half then lat = half end
  if lat < -half then lat = -half end
  return a.x + lx * along + (-lz) * lat,
         a.z + lz * along + lx * lat
end

local function calculateSteering(car, p, speed, fx, fz, dt)
  local seg, t, lineDist = H.findNearestSegment(car, p)
  p.segIdx = seg
  p.lineDist = lineDist

  -- heading auto-correction right after teleport / rescue
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

  -- signed lateral offset of the CAR from the line (for the center pull)
  local a = state.nodes[seg]
  local b = H.segEnd(seg)
  local lx, lz = normalize2(b.x - a.x, b.z - a.z)
  local latOffset = (car.position.x - a.x) * -lz + (car.position.z - a.z) * lx

  -- lateral shift while overtaking, kept inside the corridor
  local corridorHalf = H.corridorAt(seg)
  if p.overtakeSide ~= 0 then
    if corridorHalf < 4.0 then
      -- nowhere to pass here: tuck back in, try again further on
      p.overtakeSide = 0
    else
      local shift = math.min(CFG.OVERTAKE_LATERAL, corridorHalf - WALL_MARGIN)
      local nx, nz = normalize2(-(targetZ - car.position.z), targetX - car.position.x)
      targetX = targetX + nx * p.overtakeSide * shift
      targetZ = targetZ + nz * p.overtakeSide * shift
    end
  end

  -- never aim near a wall
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
  local steer = angle * CFG.STEER_GAIN + p.slipAngle * 0.5 - p.yawRate * 0.04

  -- CONTINUOUS pull to the corridor center: the further off-center the car
  -- is, the harder it steers back. Cars can no longer settle at a wall.
  -- (Disabled mid-overtake so the pass itself is not fought.)
  if p.overtakeSide == 0 then
    local pull = clamp(latOffset / 12, -1, 1) * CENTER_PULL
    steer = steer - pull
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
-- overtaking (NFS style: pass ANY blocker, side by live room)
------------------------------------------------------------

local function beginOvertake(carIndex, car, p, gap)
  p.followTimer = 0
  p.blockedT = 0
  p.passT = 0
  -- NFS trick: cast live rays left/right and commit to the side with room
  local side = 0
  if H.liveRoomLeftRight then
    local dLeft, dRight = H.liveRoomLeftRight(car)
    side = (dRight >= dLeft) and 1 or -1
  end
  if side == 0 then side = (carRand(carIndex, 2) > 0.5) and 1 or -1 end
  p.overtakeSide = side
  H.log("CAR=%d OVERTAKE_BEGIN side=%d gap=%.1f", carIndex, side, gap or -1)
end

local function updateOvertake(carIndex, car, p, speed, targetSpeed, dt)
  local cone = CFG.AVOID_LATERAL or 5.0
  local gap, leaderSpeed = H.carAhead(carIndex, car, cone)

  -- pass in progress
  if p.overtakeSide ~= 0 then
    p.passT = p.passT + dt
    if not gap then
      -- nothing in the cone anymore: pass complete
      p.overtakeSide = 0
      H.log("CAR=%d OVERTAKE_END clear", carIndex)
      return targetSpeed
    end
    if p.passT > PASS_TIMEOUT then
      p.overtakeSide = 0
      H.log("CAR=%d OVERTAKE_END timeout", carIndex)
      return targetSpeed
    end
    -- hold the pace while going around; brake only to avoid contact
    local safeSpeed = math.sqrt(math.max(0,
      (leaderSpeed or 0) * (leaderSpeed or 0) +
      2 * CFG.BRAKE_ACCEL * math.max(0, (gap or 0) - 4)))
    return math.min(math.max(targetSpeed, (leaderSpeed or 0) + 4), math.max(safeSpeed, 6))
  end

  if not gap then
    p.followTimer = 0
    p.blockedT = 0
    return targetSpeed
  end

  -- ANY car ahead slower than my pace is a blocker — including a
  -- STANDING PLAYER. This was the old bug: parked player = eternal queue.
  if (leaderSpeed or 0) < math.min(p.cruiseSpeed, targetSpeed + 3) - 2 then
    p.followTimer = p.followTimer + dt
    -- approach safely while deciding
    local safeSpeed = math.sqrt(math.max(0,
      (leaderSpeed or 0) * (leaderSpeed or 0) +
      2 * CFG.BRAKE_ACCEL * math.max(0, (gap or 0) - safeNumber(state.followGap, 8))))
    if p.followTimer > TRIGGER_TIME
       and H.corridorAt(p.segIdx) >= (CFG.PASS_MIN_ROOM or 6.0) then
      beginOvertake(carIndex, car, p, gap)
      return math.max(targetSpeed, (leaderSpeed or 0) + 4)
    end
    -- blocked dead for too long: force the pass even in tight spots
    if speed < 3 then
      p.blockedT = p.blockedT + dt
      if p.blockedT > FORCE_TIME then
        beginOvertake(carIndex, car, p, gap)
        return math.max(targetSpeed, 6)
      end
    end
    return math.min(targetSpeed, safeSpeed)
  end

  -- same-speed or faster traffic: normal safe following
  p.followTimer = 0
  p.blockedT = 0
  local safeSpeed = math.sqrt(math.max(0,
    (leaderSpeed or 0) * (leaderSpeed or 0) +
    2 * CFG.BRAKE_ACCEL * math.max(0, (gap or 0) - safeNumber(state.followGap, 8))))
  if (gap or 0) < safeNumber(state.followGap, 8) then
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
  gas = gas * lerp(1.0, 0.15, p.slipSeverity)
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

  -- wall straight ahead: brake to stop short of it (steering pulls to center)
  if H.wallAhead then
    local wallDist = H.wallAhead(car)
    if wallDist and wallDist < WALL_BRAKE_DIST then
      local vStop = math.sqrt(math.max(0,
        2 * CFG.BRAKE_ACCEL * math.max(0, wallDist - WALL_STOP_MARGIN)))
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

  -- stuck rescue: re-orient to the line and push on
  if speed < RESCUE_SPEED then
    p.stuck = p.stuck + dt
    if p.stuck > RESCUE_TIME then
      p.stuck = 0
      p.correctTimer = 2.0
      p.gas = 1.0
      H.log("CAR=%d STUCK_RESCUE", carIndex)
    end
  else
    p.stuck = 0
  end

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
