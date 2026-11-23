-- RoutePilot/driver.lua — v0.10.0 (CSP 0.3.2-preview2)
-- LANE-BASED RETHINK (user's design):
--  * The width preset defines N FIXED LANES (1..4). Every car is assigned a
--    home lane by pace: SLOW cars in the RIGHT lane, FAST cars in the LEFT
--    lane. A car drives its rail and does not wander.
--  * 1 lane = no overtakes (single file). 2+ lanes: a faster car changes to
--    a free lane ONCE, decisively, and only when that lane is actually clear
--    of cars for the whole pass distance; then it may return home.
--  * HARD BOUNDS: steering targets are clamped inside the bounds AND an
--    edge-correction term pushes back if the car is near a bound. Crossing
--    the recorded bounds (walls/pillars) is now impossible by steering.
--  * PATH SMOOTHING: the pursuit point is averaged over three samples of
--    the line ahead — no per-node wiggle, no zigzag. Deadband keeps the
--    wheel still on straights.
--  * No standstill overtakes at spawn (cars must be moving to overtake).

local Driver = {}
local CFG, state, H

local BOUND_MARGIN = 0.5
local EDGE_ZONE    = 1.2    -- m from a bound where correction kicks in
local EDGE_GAIN    = 0.5
local PASS_CLEAR   = 14.0   -- m of clear road needed in the target lane
local TRIGGER_TIME = 0.4
local PASS_TIMEOUT = 10.0
local RESCUE_SPEED = 1.5
local RESCUE_TIME  = 3.5
local STEER_DEAD   = 0.012

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

-- lane centers for N lanes (lateral offsets from the AI line)
local function laneCenters()
  local n = clamp(safeNumber(state.laneCount, 1), 1, 4)
  local width = safeNumber(state.trackWidth, 7)
  local half = width / 2
  local centers = {}
  if n == 1 then
    centers[1] = 0
    return centers
  end
  local usable = (half - BOUND_MARGIN) * 2
  local step = usable / n
  for i = 1, n do
    centers[i] = -usable / 2 + step * (i - 0.5)
  end
  return centers
end

-- lane lateral offset of a car position, rounded to the nearest lane index
local function nearestLaneIndex(off, centers)
  local best, bestD = 1, math.huge
  for i = 1, #centers do
    local d = math.abs(off - centers[i])
    if d < bestD then best, bestD = i, d end
  end
  return best
end

local function newPersonality(carIndex)
  local pace = 0.86 + carRand(carIndex, 1) * 0.28
  local kmh = clamp(safeNumber(state.speedMax, 170) * pace, 80, 240)
  local centers = laneCenters()
  -- home lane: fast cars left (low index = negative offset = left), slow right
  local homeLane
  if pace >= 1.0 then
    homeLane = 1 + math.floor(carRand(carIndex, 3) * math.min(2, #centers))
  else
    homeLane = #centers - math.floor(carRand(carIndex, 4) * math.min(2, math.max(1, #centers - 1)))
  end
  homeLane = clamp(homeLane, 1, #centers)
  local p = {
    segIdx = 1, pace = pace, cruiseSpeed = kmh / 3.6,
    homeLane = homeLane, curLane = homeLane,
    steer = 0, steerFiltered = 0, gas = 0, brake = 0,
    slipAngle = 0, lateralAccel = 0, yawRate = 0,
    previousFx = nil, previousFz = nil,
    stuck = 0, passT = 0,
    lineDist = 0, latOff = 0, targetSpeed = 0,
    slipSeverity = 0, stability = 1,
    overtakeSide = 0, passLat = 0, blockerOff = 0, followTimer = 0,
    retireT = 2.0,
  }
  state.p[carIndex] = p
  H.log("CAR_INIT id=%d cruise=%.1fkmh pace=%.2f lane=%d/%d",
    carIndex, kmh, pace, homeLane, #centers)
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

local function linePointAtLateral(seg, t, ahead, lat)
  local pt = H.walkLine(seg, t, ahead)
  if not pt then
    pt = H.segEnd(seg)
  end
  local a = state.nodes[seg]
  local b = H.segEnd(seg)
  local lx, lz = normalize2(b.x - a.x, b.z - a.z)
  return pt.x - lz * lat, pt.z + lx * lat
end

-- SMOOTHED pursuit target: average of three points along the line ahead
-- (kills per-node wiggle: the car cuts corners smoothly, never zigzags)
local function smoothedTarget(seg, t, ahead, lat)
  local pts = { ahead * 0.6, ahead, ahead * 1.4 }
  local sx, sz, cnt = 0, 0, 0
  for k = 1, 3 do
    local tx, tz = linePointAtLateral(seg, t, pts[k], lat)
    sx = sx + tx
    sz = sz + tz
    cnt = cnt + 1
  end
  return sx / cnt, sz / cnt
end

local function calculateSteering(car, p, speed, fx, fz)
  local seg, t, lineDist = H.findNearestSeg(car.position, p.segIdx)
  p.segIdx = seg
  p.lineDist = lineDist
  p.latOff = H.lateralOf(car.position, seg)

  local ahead = clamp(
    CFG.LOOKAHEAD_BASE + speed * CFG.LOOKAHEAD_SPEED,
    CFG.LOOKAHEAD_MIN, CFG.LOOKAHEAD_MAX)

  -- rail: home lane or the passing lane
  local centers = laneCenters()
  local wantLat = centers[clamp(p.curLane, 1, #centers)]
  if p.overtakeSide ~= 0 then
    wantLat = p.passLat
  end
  wantLat = clampLat(seg, wantLat)

  local tx, tz = smoothedTarget(seg, t, ahead, wantLat)

  local dx = tx - car.position.x
  local dz = tz - car.position.z
  local dist = math.sqrt(dx * dx + dz * dz)
  if dist < 0.001 then return p.steer end

  local angle = signedAngle(fx, fz, dx / dist, dz / dist)

  -- deadband: a straight needs no steering
  if math.abs(angle) < STEER_DEAD then
    angle = 0
  end

  local steer = angle * CFG.STEER_GAIN + p.slipAngle * 0.62

  -- HARD BOUNDS: edge correction pushes the car away from a bound
  local bl, br = H.boundsAt(seg)
  local off = p.latOff
  if off < bl + EDGE_ZONE then
    steer = steer + (bl + EDGE_ZONE - off) * EDGE_GAIN
  elseif off > br - EDGE_ZONE then
    steer = steer - (off - (br - EDGE_ZONE)) * EDGE_GAIN
  end

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

-- is a lane clear of other cars for PASS_CLEAR meters ahead of us?
local function laneClear(carIndex, car, lat, centers)
  local sim = ac.getSim()
  if not sim then return true end
  local fx, fz = normalize2(car.look.x, car.look.z)
  for i = 0, sim.carsCount - 1 do
    if i ~= carIndex then
      local other = ac.getCar(i)
      if other and other.isActive then
        local dx = other.position.x - car.position.x
        local dz = other.position.z - car.position.z
        local forward = dx * fx + dz * fz
        if forward > -2 and forward < PASS_CLEAR then
          local oseg = select(1, H.findNearestSeg(other.position, 1))
          local ooff = H.lateralOf(other.position, oseg)
          local oLane = nearestLaneIndex(ooff, centers)
          local mLane = nearestLaneIndex(lat, centers)
          if oLane == mLane then
            return false
          end
        end
      end
    end
  end
  return true
end

-- DECISIVE single lane change to a CLEAR adjacent lane
local function beginOvertake(carIndex, p, car, gap, centers)
  p.followTimer = 0
  p.passT = 0
  local from = clamp(p.curLane, 1, #centers)
  -- try left first (fast side), then right
  local targetLane
  if from > 1 and laneClear(carIndex, car, centers[from - 1], centers) then
    targetLane = from - 1
  elseif from < #centers and laneClear(carIndex, car, centers[from + 1], centers) then
    targetLane = from + 1
  end
  if not targetLane then return false end
  p.overtakeSide = (targetLane < from) and -1 or 1
  p.passLat = clampLat(p.segIdx, centers[targetLane])
  p.curLane = targetLane
  H.log("CAR=%d LANE_CHANGE from=%d to=%d lat=%.1f gap=%.1f",
    carIndex, from, targetLane, p.passLat, gap or -1)
  return true
end

local function updateOvertake(carIndex, car, p, speed, targetSpeed, dt)
  local centers = laneCenters()
  if #centers < 2 then
    -- single lane: NEVER overtake, keep the queue with a safe gap
    p.overtakeSide = 0
    local gap, leaderSpeed = H.carAheadFull(carIndex, car, 5.0)
    if gap then
      local safe = math.sqrt(math.max(0,
        (leaderSpeed or 0) * (leaderSpeed or 0) +
        2 * CFG.BRAKE_ACCEL * math.max(0, (gap or 0) - safeNumber(state.followGap, 8))))
      return math.min(targetSpeed, safe)
    end
    return targetSpeed
  end

  if p.overtakeSide ~= 0 then
    -- passing: keep the lane; return home when clear ahead
    p.passT = p.passT + dt
    local gap, leaderSpeed = H.carAheadFull(carIndex, car, 4.0)
    if p.passT > PASS_TIMEOUT then
      local from = p.curLane
      if from ~= p.homeLane and laneClear(carIndex, car, centers[p.homeLane], centers) then
        p.curLane = p.homeLane
        H.log("CAR=%d LANE_RETURN home=%d", carIndex, p.homeLane)
      end
      p.overtakeSide = 0
      p.passT = 0
      return targetSpeed
    end
    if gap then
      local safe = math.sqrt(math.max(0,
        (leaderSpeed or 0) * (leaderSpeed or 0) +
        2 * CFG.BRAKE_ACCEL * math.max(0, (gap or 0) - 5)))
      return math.min(math.max(targetSpeed, (leaderSpeed or 0) + 4), math.max(safe, 6))
    end
    return targetSpeed
  end

  local gap, leaderSpeed = H.carAheadFull(carIndex, car, 4.5)
  if not gap then
    p.followTimer = 0
    return targetSpeed
  end

  -- blocker: slower car (or standing player) ahead in OUR lane
  if (leaderSpeed or 0) < math.min(p.cruiseSpeed, targetSpeed + 3) - 2
     and speed > 3 then
    p.followTimer = p.followTimer + dt
    if p.followTimer > TRIGGER_TIME then
      if beginOvertake(carIndex, p, car, gap, centers) then
        return math.max(targetSpeed, (leaderSpeed or 0) + 4)
      end
    end
    local safe = math.sqrt(math.max(0,
      (leaderSpeed or 0) * (leaderSpeed or 0) +
      2 * CFG.BRAKE_ACCEL * math.max(0, (gap or 0) - safeNumber(state.followGap, 8))))
    return math.min(targetSpeed, safe)
  end

  p.followTimer = 0
  local safe = math.sqrt(math.max(0,
    (leaderSpeed or 0) * (leaderSpeed or 0) +
    2 * CFG.BRAKE_ACCEL * math.max(0, (gap or 0) - safeNumber(state.followGap, 8))))
  return math.min(targetSpeed, safe)
end

local function calculateThrottle(p, speed, targetSpeed)
  local error = targetSpeed - speed
  local gas = clamp(error * CFG.GAS_GAIN, 0, CFG.MAX_GAS)
  local brake = clamp(-error * CFG.BRAKE_GAIN, 0, CFG.MAX_BRAKE)
  gas = gas * lerp(1.0, 0.15, p.slipSeverity)
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
    "CAR=%d POS=(%.2f,%.2f,%.2f) SPEED=%.2f STEER=%.3f GAS=%.3f BRAKE=%.3f TARGET_SPEED=%.2f LINE_ERROR=%.2f SEGMENT=%d PACE=%.2f LANE=%d/%d BOUNDS=[%.1f,%.1f] OFF=%.1f OT=%d",
    i, car.position.x, car.position.y, car.position.z,
    safeNumber(car.speedMs), p.steer, p.gas, p.brake, p.targetSpeed,
    p.lineDist, p.segIdx, p.pace, p.curLane, p.homeLane, bl, br, p.latOff, p.overtakeSide)
end

return Driver
