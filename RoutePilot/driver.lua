-- RoutePilot/driver.lua
-- Per-car ARCADE racing traffic controller, v0.5.0 (CSP 0.3.2-preview2).
--
-- v0.5.0 — adapted to PLAYER-RECORDED TRACK BOUNDS:
--  * No raycasts, no corridor scanning. Left/right bounds come from the
--    helper boundsAt(seg) (player-recorded with the line, ± slider width).
--  * STEERING: pure pursuit along the AI line with a smooth proportional
--    pull to the corridor center. No zigzags: low gain, long lookahead,
--    yaw-rate damping, slow steer rate. The car may drive CLOSE to a
--    bound (targets clamp with only 1.5 m margin) but never aims past it.
--  * OVERTAKING RULES (physical, guaranteed to fit):
--      1. A pass is committed ONLY if the gap between the blocker's
--         lateral offset and the bound on that side is at least 2.5 m
--         (a car is ~2 m wide) — measured from live positions, so the
--         car KNOWS it fits before it moves.
--      2. The side with more room is picked.
--      3. Passing cars keep their pace; blockers never brake for a passer.
--      4. If the pass window vanished (bounds narrowed), the car tucks
--         back in and follows safely instead of forcing through.
--  * STUCK/OUT-OF-BOUNDS: after 3.5 s stuck or if outside the bounds,
--    the car asks H.teleportHome() and is placed back onto the line.
--  * preventAIFromRetiring pulse kept (no vanishing cars).

local Driver = {}

local CFG, state, H

-- local tuning constants
local BOUND_MARGIN    = 1.5    -- targets keep this from a bound, m
local CENTER_PULL     = 0.35   -- proportional pull to line center
local CENTER_PULL_MAX = 0.45   -- max pull strength
local TRIGGER_TIME    = 0.6    -- follow a slower car this long, then pass
local FORCE_TIME      = 2.5    -- stopped behind a blocker this long -> force
local PASS_TIMEOUT    = 6.0
local PASS_WIDTH      = 2.5    -- min lateral clearance for a committed pass
local RESCUE_SPEED    = 1.5
local RESCUE_TIME     = 3.5
local OUTSIDE_MARGIN  = 2.0    -- this far beyond a bound -> rescue teleport

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
  local pace = 0.86 + carRand(carIndex, 1) * 0.28
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
    lateralAccel = 0,
    yawRate = 0,
    previousFx = nil,
    previousFz = nil,
    stuck = 0,
    blockedT = 0,
    passT = 0,
    lineDist = 0,
    latOff = 0,
    targetSpeed = 0,
    slipSeverity = 0,
    stability = 1,
    lastCurve = 0,
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
    local accelSeverity = clamp((p.lateralAccel - CFG.MAX_LATERAL_ACCEL) / (CFG.MAX_LATERAL_ACCEL * 0.7), 0, 1)
    severity = math.max(slipSeverity, accelSeverity) * 0.7
  end
  p.slipSeverity = severity
  p.stability = 1 - severity
  return severity
end

------------------------------------------------------------
-- speed planner
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

-- clamp a target point laterally INSIDE the bounds (hard margin)
local function clampToBounds(seg, tx, tz)
  local a = state.nodes[seg]
  local b = H.segEnd(seg)
  local lx, lz = normalize2(b.x - a.x, b.z - a.z)
  local relX, relZ = tx - a.x, tz - a.z
  local along = relX * lx + relZ * lz
  local lat = relX * -lz + relZ * lx
  local bl, br = H.boundsAt(seg)
  local hi = math.max(bl + 0.5, br - BOUND_MARGIN)
  local lo = math.min(br - 0.5, bl + BOUND_MARGIN)
  if lat > hi then lat = hi end
  if lat < lo then lat = lo end
  return a.x + lx * along + (-lz) * lat,
         a.z + lz * along + lx * lat
end

local function calculateSteering(car, p, speed, fx, fz)
  local seg, t, lineDist = H.findNearestSeg(car.position, p.segIdx)
  p.segIdx = seg
  p.lineDist = lineDist
  local latOff = H.lateralOf(car.position, seg)
  p.latOff = latOff

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

  -- lateral shift while overtaking (clamped inside the bounds)
  if p.overtakeSide ~= 0 then
    local nx, nz = normalize2(-(targetZ - car.position.z), targetX - car.position.x)
    targetX = targetX + nx * p.overtakeSide * p.passShift
    targetZ = targetZ + nz * p.overtakeSide * p.passShift
  end

  -- never aim beyond a bound
  targetX, targetZ = clampToBounds(seg, targetX, targetZ)

  local dx = targetX - car.position.x
  local dz = targetZ - car.position.z
  local distance = math.sqrt(dx * dx + dz * dz)
  if distance < 0.001 then
    return p.steer
  end
  local angle = signedAngle(fx, fz, dx / distance, dz / distance)
  local steer = angle * CFG.STEER_GAIN - p.yawRate * 0.04

  -- smooth proportional pull to the line center (kills zigzags, keeps
  -- the car from camping near a bound)
  if p.overtakeSide == 0 then
    local pull = clamp(latOff / 12, -1, 1) * CENTER_PULL
    if pull > CENTER_PULL_MAX then pull = CENTER_PULL_MAX end
    if pull < -CENTER_PULL_MAX then pull = -CENTER_PULL_MAX end
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
-- overtaking (physical rules; pass only where it FITS)
------------------------------------------------------------

local function beginOvertake(carIndex, car, p, gap)
  p.followTimer = 0
  p.blockedT = 0
  p.passT = 0
  -- pick the side with more room and compute the physical shift
  local seg = p.segIdx
  local bl, br = H.boundsAt(seg)
  local myOff = H.lateralOf(car.position, seg)
  -- side 1 = right (positive), -1 = left
  local roomRight = br - math.max(myOff, safeNumber(p.blockerOff, 0))
  local roomLeft = math.min(myOff, safeNumber(p.blockerOff, 0)) - bl
  local side, shift
  if roomRight >= roomLeft then
    side = 1
    shift = clamp(roomRight - 1.2, 1.0, 6.0)
  else
    side = -1
    shift = clamp(roomLeft - 1.2, 1.0, 6.0)
  end
  p.overtakeSide = side
  p.passShift = shift
  H.log("CAR=%d OVERTAKE_BEGIN side=%d shift=%.1f gap=%.1f", carIndex, side, shift, gap or -1)
end

local function updateOvertake(carIndex, car, p, speed, targetSpeed, dt)
  local gap, leaderSpeed, leaderIndex, leaderOff =
    H.carAheadFull(carIndex, car, 5.0)

  -- pass in progress
  if p.overtakeSide ~= 0 then
    p.passT = p.passT + dt
    -- is the window still physically open?
    local seg = p.segIdx
    local bl, br = H.boundsAt(seg)
    local windowOpen
    if p.overtakeSide > 0 then
      windowOpen = (br - safeNumber(leaderOff, p.latOff)) >= PASS_WIDTH
    else
      windowOpen = (safeNumber(leaderOff, p.latOff) - bl) >= PASS_WIDTH
    end
    if not gap then
      p.overtakeSide = 0
      H.log("CAR=%d OVERTAKE_END clear", carIndex)
      return targetSpeed
    end
    if p.passT > PASS_TIMEOUT or not windowOpen then
      p.overtakeSide = 0
      p.passT = 0
      H.log("CAR=%d OVERTAKE_END %s", carIndex, windowOpen and "timeout" or "closed_window")
      return targetSpeed
    end
    -- keep the pace around the blocker; brake only to avoid contact
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

  p.blockerOff = leaderOff

  -- ANY slower car ahead (including a STANDING PLAYER) is a blocker
  if (leaderSpeed or 0) < math.min(p.cruiseSpeed, targetSpeed + 3) - 2 then
    p.followTimer = p.followTimer + dt
    local safeSpeed = math.sqrt(math.max(0,
      (leaderSpeed or 0) * (leaderSpeed or 0) +
      2 * CFG.BRAKE_ACCEL * math.max(0, (gap or 0) - safeNumber(state.followGap, 8))))
    -- commit only where the pass physically fits (>= PASS_WIDTH beside the blocker)
    local bl, br = H.boundsAt(p.segIdx)
    local fitsRight = (br - safeNumber(leaderOff, p.latOff)) >= PASS_WIDTH
    local fitsLeft = (safeNumber(leaderOff, p.latOff) - bl) >= PASS_WIDTH
    if p.followTimer > TRIGGER_TIME and (fitsRight or fitsLeft) then
      beginOvertake(carIndex, car, p, gap)
      return math.max(targetSpeed, (leaderSpeed or 0) + 4)
    end
    -- stopped dead behind a blocker for too long: try to squeeze through
    if speed < 3 then
      p.blockedT = p.blockedT + dt
      if p.blockedT > FORCE_TIME and (fitsRight or fitsLeft) then
        beginOvertake(carIndex, car, p, gap)
        return math.max(targetSpeed, 6)
      end
    end
    return math.min(targetSpeed, safeSpeed)
  end

  -- same-speed or faster traffic: safe following
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

  -- OUT OF BOUNDS -> rescue teleport back onto the line
  local seg = H.findNearestSeg(car.position, p.segIdx)
  local off = H.lateralOf(car.position, seg)
  local bl, br = H.boundsAt(seg)
  if off > br + OUTSIDE_MARGIN or off < bl - OUTSIDE_MARGIN then
    if H.teleportHome then
      H.teleportHome(carIndex, car)
    end
    return
  end

  -- STUCK -> rescue
  if speed < RESCUE_SPEED then
    p.stuck = p.stuck + dt
    if p.stuck > RESCUE_TIME then
      p.stuck = 0
      if H.teleportHome then
        H.teleportHome(carIndex, car)
      end
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
  local seg = p.segIdx
  local bl, br = H.boundsAt(seg)
  return string.format(
    "CAR=%d POS=(%.2f,%.2f,%.2f) SPEED=%.2f STEER=%.3f GAS=%.3f BRAKE=%.3f SLIP=%.4f TARGET_SPEED=%.2f LINE_ERROR=%.2f SEGMENT=%d PACE=%.2f BOUNDS=[%.1f,%.1f] OFF=%.1f OT_SIDE=%d",
    i, car.position.x, car.position.y, car.position.z,
    safeNumber(car.speedMs), p.steer, p.gas, p.brake,
    p.slipAngle, p.targetSpeed,
    p.lineDist, seg, p.pace, bl, br, p.latOff, p.overtakeSide)
end

return Driver
