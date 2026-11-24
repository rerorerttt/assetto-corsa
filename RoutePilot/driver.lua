-- RoutePilot/driver.lua — v0.9.0 (CSP 0.3.2-preview2)
-- FUNDAMENTAL RETHINK:
--  * The user trace showed ZERO CAR_INIT lines: the main script never adopted
--    the cars (spawn gate blocked driving forever), so the cars drove on the
--    STOCK game AI — that is where the zigzags, burnouts, wall-hunting and
--    ignoring the player came from. The main script now adopts every AI car
--    instantly; this driver only does the driving.
--  * STEERING DEADBAND: on a straight the angle error is tiny — steering 0
--    instead of micro-correcting left/right every frame (zigzag source).
--  * DECISIVE OVERTAKES: side is picked ONCE from the free room and LOCKED
--    for the whole pass; no flapping between sides mid-manoeuvre.
--  * SOFT START: gas capped at low speed (no burnout launch).
--  * Route planned tens of meters ahead (lookahead 14 + 0.8*speed, max 50).

local Driver = {}
local CFG, state, H

local BOUND_MARGIN = 0.4
local REQ_SEP      = 2.8    -- center-to-center gap needed to pass, m
local TRIGGER_TIME = 0.5
local FORCE_TIME   = 2.0
local PASS_TIMEOUT = 8.0
local RESCUE_SPEED = 1.5
local RESCUE_TIME  = 3.5
local STEER_DEAD   = 0.001  -- only suppress numerical noise; never freeze real cornering input

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
  -- Cache route arc length once: traffic gaps are measured along the route,
  -- not against each car's instantaneous world-space heading.
  local count = H.segmentCount()
  state.routeLength, state.routeCum, state.routeSegLength = 0, {}, {}
  for i = 1, count do
    state.routeCum[i] = state.routeLength
    local a, b = state.nodes[i], H.segEnd(i)
    local length = math.sqrt((b.x-a.x)^2 + (b.z-a.z)^2)
    state.routeSegLength[i] = length
    state.routeLength = state.routeLength + length
  end
end

local function routeProjection(car, hintSeg)
  local seg, t, distance = H.findNearestSeg(car.position, hintSeg or 1)
  local lengths, cumulative = state.routeSegLength, state.routeCum
  if not lengths or not lengths[seg] then return nil end
  local progress = cumulative[seg] + lengths[seg] * t
  local lateral = H.lateralOf(car.position, seg)
  return progress, lateral, seg, t, distance
end

-- Signed distance from own progress to another car. Positive means ahead.
local function routeGap(ownProgress, otherProgress)
  local gap = otherProgress - ownProgress
  local total = state.routeLength or 0
  if state.loopRoute and total > 0 then
    if gap > total * 0.5 then gap = gap - total end
    if gap < -total * 0.5 then gap = gap + total end
  end
  return gap
end

local function newPersonality(carIndex)
  -- AI order is deterministic: 1=40 km/h, 2=fast, 3=40 km/h, etc.
  local slow = (carIndex % 2) == 1
  local kmh = slow and 40 or clamp(safeNumber(state.speedMax, 170), 80, 240)
  local p = {
    segIdx = 1, slow = slow, cruiseSpeed = kmh / 3.6,
    steer = 0, steerFiltered = 0, gas = 0, brake = 0,
    slipAngle = 0, lateralAccel = 0, yawRate = 0,
    previousFx = nil, previousFz = nil,
    stuck = 0, blockedT = 0, passT = 0,
    lineDist = 0, latOff = 0, targetSpeed = 0,
    slipSeverity = 0, stability = 1,
    overtakeSide = 0, passLat = 0, blockerOff = 0, followTimer = 0,
    laneOffset = nil, passTargetIndex = nil, retireT = 2.0,
  }
  state.p[carIndex] = p
  H.log("CAR_INIT id=%d lane=LEFT class=%s cruise=%.1fkmh",
    carIndex, slow and "SLOW" or "FAST", kmh)
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
    severity = math.max(ss, as)
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

local function clampLat(seg, lat)
  local bl, br = H.boundsAt(seg)
  return clamp(lat, bl + BOUND_MARGIN, br - BOUND_MARGIN)
end

local function smoothPathSample(seg, t)
  local count = #state.nodes
  local function nodeAt(i)
    if state.loopRoute then
      return state.nodes[((i - 1) % count) + 1]
    end
    return state.nodes[clamp(i, 1, count)]
  end
  local p0, p1, p2, p3 = nodeAt(seg - 1), nodeAt(seg), nodeAt(seg + 1), nodeAt(seg + 2)
  local u, u2, u3 = t, t*t, t*t*t
  local function catmull(a, b, c, d)
    return 0.5 * ((2*b) + (-a+c)*u + (2*a-5*b+4*c-d)*u2 + (-a+3*b-3*c+d)*u3)
  end
  local function derivative(a, b, c, d)
    return 0.5 * ((-a+c) + 2*(2*a-5*b+4*c-d)*u + 3*(-a+3*b-3*c+d)*u2)
  end
  local x = catmull(p0.x,p1.x,p2.x,p3.x)
  local z = catmull(p0.z,p1.z,p2.z,p3.z)
  local tx, tz = normalize2(
    derivative(p0.x,p1.x,p2.x,p3.x),
    derivative(p0.z,p1.z,p2.z,p3.z))
  return x, z, tx, tz
end

local function linePointAtLateral(seg, t, ahead, lat)
  local _, targetSeg, targetT = H.walkLine(seg, t, ahead)
  if not targetSeg then targetSeg, targetT = seg, 1 end
  local x, z, tx, tz = smoothPathSample(targetSeg, targetT or 0)
  return x - tz * lat, z + tx * lat, tx, tz
end

local function calculateSteering(car, p, speed, fx, fz, dt)
  local seg, t, lineDist = H.findNearestSeg(car.position, p.segIdx)
  p.segIdx, p.lineDist = seg, lineDist
  p.latOff = H.lateralOf(car.position, seg)
  local progress = routeProjection(car, seg)
  p.routeProgress = progress

  local ahead = clamp(CFG.LOOKAHEAD_BASE + speed * CFG.LOOKAHEAD_SPEED,
    CFG.LOOKAHEAD_MIN, CFG.LOOKAHEAD_MAX)
  local bl, br = H.boundsAt(seg)
  local mid, width = (bl + br) * 0.5, br - bl
  local leftCenter = width >= 5.4 and ((mid + br) * 0.5) or mid
  local rightCenter = (bl + mid) * 0.5
  p.laneOffset = p.overtakeSide == 0 and leftCenter or rightCenter
  local laneTarget = clampLat(seg, p.laneOffset or leftCenter)
  p.laneOffsetCurrent = p.laneOffsetCurrent or laneTarget
  local maxStep = 1.5 * math.max(0, dt or 0)
  p.laneOffsetCurrent = p.laneOffsetCurrent +
    clamp(laneTarget - p.laneOffsetCurrent, -maxStep, maxStep)

  local tx, tz, pathTx, pathTz = linePointAtLateral(
    seg, t, ahead, p.laneOffsetCurrent)
  local dx, dz = tx - car.position.x, tz - car.position.z
  local targetDist = math.sqrt(dx*dx + dz*dz)
  if targetDist < 0.01 then return 0 end

  -- Stanley-style feedback: path heading plus signed cross-track correction.
  -- The correction is speed-aware, so it does not snap the car sideways at
  -- low speed or become too weak as speed rises.
  local headingError = signedAngle(fx, fz, pathTx, pathTz)
  local crossTrack = p.laneOffsetCurrent - p.latOff
  local crossTrackAngle = math.atan2(2.0 * crossTrack, math.max(3.0, speed + 2.0))
  local previewAngle = signedAngle(fx, fz, dx / targetDist, dz / targetDist)
  local angle = headingError * 0.35 + crossTrackAngle * 0.65
  -- Preview is a bounded anticipatory term; it prevents corner-entry delay
  -- without allowing a single distant vertex to dominate steering.
  angle = angle + clamp(previewAngle - angle, -0.12, 0.12) * 0.35
  p.headingError, p.crossTrackError = headingError, crossTrack
  local steer = angle * CFG.STEER_GAIN + p.slipAngle * 0.30
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

-- free room from a lateral offset to each bound, at a segment
local function roomAt(seg, off)
  local bl, br = H.boundsAt(seg)
  return off - bl, br - off
end

-- The road is split into two fixed 3 m lanes when the recorded corridor
-- is wide enough. Positive lateral offset is the left lane.
local function laneCenters(seg)
  local bl, br = H.boundsAt(seg)
  local mid = (bl + br) * 0.5
  local left = (mid + br) * 0.5
  local right = (bl + mid) * 0.5
  if (mid - bl < 2.7 or br - mid < 2.7) then return nil, nil end
  return left, right
end

-- Checks traffic in a specified lane, both ahead and behind. This is used
-- before entering the passing lane and before returning to the left lane.
local function laneOccupied(carIndex, car, laneOffset, ignoreIndex)
  local sim = ac.getSim()
  if not sim then return true end
  local own = state.p[carIndex]
  local ownProgress = own and own.routeProgress
  if ownProgress == nil then
    ownProgress = routeProjection(car, own and own.segIdx or 1)
  end
  if ownProgress == nil then return true end
  for i = 0, sim.carsCount - 1 do
    if i ~= carIndex and i ~= ignoreIndex then
      local other = ac.getCar(i)
      if other and other.isActive then
        local progress, lateral = routeProjection(other, own.segIdx)
        if progress ~= nil then
          local longitudinal = routeGap(ownProgress, progress)
          if math.abs(lateral - laneOffset) < 1.35 and
             longitudinal > -18 and longitudinal < 48 then
            return true
          end
        end
      end
    end
  end
  return false
end

local function carAheadInLane(carIndex, car, laneOffset)
  local sim = ac.getSim()
  local own = state.p[carIndex]
  if not sim or not own then return nil, nil, nil end
  local ownProgress = own.routeProgress
  if ownProgress == nil then
    ownProgress = routeProjection(car, own.segIdx)
    own.routeProgress = ownProgress
  end
  if ownProgress == nil then return nil, nil, nil end
  local bestGap, bestSpeed, bestIndex = nil, nil, nil
  for i = 0, sim.carsCount - 1 do
    if i ~= carIndex then
      local other = ac.getCar(i)
      if other and other.isActive then
        local progress, lateral = routeProjection(other, own.segIdx)
        if progress ~= nil and math.abs(lateral - laneOffset) < 1.35 then
          local gap = routeGap(ownProgress, progress)
          if gap > 0.5 and gap < 90 and (not bestGap or gap < bestGap) then
            bestGap = gap
            bestSpeed = math.max(0, safeNumber(other.speedMs))
            bestIndex = i
          end
        end
      end
    end
  end
  return bestGap, bestSpeed, bestIndex
end

local function beginOvertake(carIndex, car, p, gap, leaderIndex)
  local leftCenter, rightCenter = laneCenters(p.segIdx)
  if not leftCenter or p.slow then return false end
  if laneOccupied(carIndex, car, rightCenter, nil) then
    H.log("CAR=%d OVERTAKE_DENIED right_lane_occupied", carIndex)
    return false
  end
  p.overtakeSide = -1
  p.passLat = rightCenter
  p.laneOffset = rightCenter
  p.passTargetIndex = leaderIndex
  p.passT = 0
  p.followTimer = 0
  p.blockedT = 0
  H.log("CAR=%d OVERTAKE_BEGIN target=%d from=LEFT to=RIGHT gap=%.1f",
    carIndex, leaderIndex or -1, gap or -1)
  return true
end

local function updateOvertake(carIndex, car, p, speed, targetSpeed, dt)
  local ownProgress = routeProjection(car, p.segIdx)
  p.routeProgress = ownProgress
  local leftCenter, rightCenter = laneCenters(p.segIdx)
  local gap, leaderSpeed, leaderIndex = carAheadInLane(
    carIndex, car, leftCenter or 0)

  if p.overtakeSide ~= 0 then
    p.passT = p.passT + dt
    local target = p.passTargetIndex and ac.getCar(p.passTargetIndex) or nil
    local passedClearance = false
    if target and target.isActive then
      local targetProgress = routeProjection(target, p.segIdx)
      local currentProgress = ownProgress or routeProjection(car, p.segIdx)
      local passedDistance = (currentProgress and targetProgress)
        and routeGap(targetProgress, currentProgress) or -math.huge
      local ownLength = (car.aabbSize and car.aabbSize.z) or 4.5
      local otherLength = (target.aabbSize and target.aabbSize.z) or 4.5
      passedClearance = passedDistance >= ownLength + otherLength
    else
      -- A removed/despawned target is not a reason to cut back blindly:
      -- the full lane occupancy check below is still mandatory.
      passedClearance = true
    end

    if passedClearance and leftCenter and
       not laneOccupied(carIndex, car, leftCenter, p.passTargetIndex) then
      p.overtakeSide = 0
      p.passTargetIndex = nil
      p.laneOffset = leftCenter
      p.passT = 0
      p.followTimer = 0
      p.blockedT = 0
      H.log("CAR=%d OVERTAKE_END return=LEFT clearance=2_car_lengths", carIndex)
      return targetSpeed
    end

    -- Stay in the right lane until the complete rear-bumper clearance exists.
    p.laneOffset = rightCenter or p.passLat
    p.passLat = p.laneOffset
    return targetSpeed
  end

  if not leftCenter then return targetSpeed end
  p.laneOffset = leftCenter

  if not gap or not leaderIndex then
    p.followTimer = 0
    p.blockedT = 0
    return targetSpeed
  end

  local slower = (leaderSpeed or 0) < p.cruiseSpeed - 1.0
  if slower then
    p.followTimer = p.followTimer + dt
    local safe = math.sqrt(math.max(0,
      (leaderSpeed or 0) * (leaderSpeed or 0) +
      2 * CFG.BRAKE_ACCEL * math.max(0, gap - safeNumber(state.followGap, 8))))
    if not p.slow and p.followTimer >= TRIGGER_TIME then
      if beginOvertake(carIndex, car, p, gap, leaderIndex) then
        return targetSpeed
      end
    end
    return math.min(targetSpeed, safe)
  end

  p.followTimer = 0
  p.blockedT = 0
  local safe = math.sqrt(math.max(0,
    (leaderSpeed or 0) * (leaderSpeed or 0) +
    2 * CFG.BRAKE_ACCEL * math.max(0, gap - safeNumber(state.followGap, 8))))
  return math.min(targetSpeed, safe)
end

local function calculateThrottle(p, speed, targetSpeed)
  local error = targetSpeed - speed
  local gas = clamp(error * CFG.GAS_GAIN, 0, CFG.MAX_GAS)
  local brake = clamp(-error * CFG.BRAKE_GAIN, 0, CFG.MAX_BRAKE)
  gas = gas * lerp(1.0, 0.15, p.slipSeverity)
  -- SOFT START: no burnout launches from a standstill
  if speed < 5 then
    gas = math.min(gas, 0.6)
  end
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
  local progress = routeProjection(car, p.segIdx)
  if progress ~= nil then p.routeProgress = progress end
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
  local bl, br = H.boundsAt(p.segIdx)
  return string.format(
    "CAR=%d POS=(%.2f,%.2f,%.2f) SPEED=%.2f STEER=%.3f GAS=%.3f BRAKE=%.3f TARGET_SPEED=%.2f LINE_ERROR=%.2f SEGMENT=%d PACE=%.2f BOUNDS=[%.1f,%.1f] OFF=%.1f OT_SIDE=%d CROSS=%.2f HEAD=%.3f",
    i, car.position.x, car.position.y, car.position.z,
    safeNumber(car.speedMs), p.steer, p.gas, p.brake, p.targetSpeed,
    p.lineDist, p.segIdx, (p.pace or 0), bl, br, p.latOff, p.overtakeSide,
    p.crossTrackError or 0, p.headingError or 0)
end

return Driver