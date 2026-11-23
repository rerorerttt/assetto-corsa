-- RoutePilot/driver.lua — v0.11.0 RAILS AI (CSP 0.3.2-preview2)
-- A textbook PURE PURSUIT controller (bicycle model):
--    ld = clamp(K_DD * v, LD_MIN, LD_MAX)     -- lookahead distance
--    alpha = angle(heading -> target)
--    delta = atan(2 * L * sin(alpha) / ld)     -- front wheel angle
--    steer = delta / STEER_LOCK                -- normalized -1..1
-- Pure pursuit is self-stabilizing: it cannot zigzag by construction. The
-- target is ONE point on the rail ld meters ahead. No averaging, no
-- edge-correction terms (the v0.10 three-point average cut corners outside
-- the bounds and fought the edge term -> the giant zigzags).
-- Burnouts are killed with wheel slip control (car.wheels[2]/[3].slipRatio,
-- guarded) plus a standstill gas cap.
-- Blockers are cars in OUR OWN lane only (v0.10 saw diagonal neighbors on
-- the 7 m track as blockers and braked the whole queue to a stop).
-- Lanes: 1..4 from the width preset; a faster car changes to a CLEAR lane
-- once, decisively.

local Driver = {}
local CFG, state, H

-- pure pursuit constants
local WHEELBASE  = 2.6    -- m, typical AC car
local K_DD       = 0.55   -- lookahead seconds
local LD_MIN     = 6.0    -- m
local LD_MAX     = 35.0   -- m
local STEER_LOCK = 0.6    -- rad, front wheel lock for normalization

-- bounds / lanes
local BOUND_MARGIN = 0.5
local PASS_CLEAR   = 18.0
local TRIGGER_TIME = 0.4
local PASS_TIMEOUT = 10.0
local FOLLOW_LAT   = 2.2   -- blocker must be within this lateral distance of OUR rail
local MIN_GAP      = 6.0
local RESCUE_SPEED = 1.5
local RESCUE_TIME  = 3.5

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

local function laneCenters()
  local n = clamp(safeNumber(state.laneCount, 1), 1, 4)
  local width = safeNumber(state.trackWidth, 3)
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

local function newPersonality(carIndex)
  local pace = 0.86 + carRand(carIndex, 1) * 0.28
  local kmh = clamp(safeNumber(state.speedMax, 170) * pace, 80, 240)
  local centers = laneCenters()
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
    steer = 0, gas = 0, brake = 0,
    slipAngle = 0,
    stuck = 0, passT = 0,
    lineDist = 0, latOff = 0, targetSpeed = 0,
    overtakeSide = 0, followTimer = 0,
    retireT = 2.0,
  }
  state.p[carIndex] = p
  H.log("CAR_INIT id=%d cruise=%.1fkmh pace=%.2f lane=%d/%d",
    carIndex, kmh, pace, homeLane, #centers)
end

local function readHeading(car, p, dt)
  local look = car.look
  local fx, fz = normalize2(look.x, look.z)
  local v = car.velocity
  local rx, rz = fz, -fx
  local vx, vz = safeNumber(v.x), safeNumber(v.z)
  local longVel = vx * fx + vz * fz
  local latVel = vx * rx + vz * rz
  p.slipAngle = math.atan2(latVel, math.max(0.5, longVel))
  local speed = math.max(0, safeNumber(car.speedMs, longVel))
  return speed, fx, fz
end

-- driven wheel slip (guarded; nil if unavailable)
local function wheelSlip(car)
  local okW, wheels = pcall(function() return car.wheels end)
  if not okW or type(wheels) ~= "table" then return nil end
  local maxSlip = nil
  for i = 2, 3 do
    local okW2, wheel = pcall(function() return wheels[i] end)
    if okW2 and wheel then
      local okS, sr = pcall(function() return wheel.slipRatio end)
      if okS and type(sr) == "number" and sr == sr then
        local a = math.abs(sr)
        if not maxSlip or a > maxSlip then maxSlip = a end
      end
    end
  end
  return maxSlip
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

-- PURE PURSUIT steering. One target point on the rail, bicycle model.
local function calculateSteering(car, p, speed, fx, fz)
  local seg, t, lineDist = H.findNearestSeg(car.position, p.segIdx)
  p.segIdx = seg
  p.lineDist = lineDist
  p.latOff = H.lateralOf(car.position, seg)

  local ld = clamp(K_DD * math.max(speed, 5.0), LD_MIN, LD_MAX)

  -- rail lateral offset (clamped inside the bounds)
  local centers = laneCenters()
  local rail = centers[clamp(p.curLane, 1, #centers)]
  local bl, br = H.boundsAt(seg)
  rail = clamp(rail, bl + BOUND_MARGIN, br - BOUND_MARGIN)

  -- target: the path point ld meters ahead, shifted onto the rail
  local a = state.nodes[seg]
  local b = H.segEnd(seg)
  local lx, lz = normalize2(b.x - a.x, b.z - a.z)
  local pt = H.walkLine(seg, t, ld)
  local tx, tz
  if pt then
    tx = pt.x - lz * rail
    tz = pt.z + lx * rail
  else
    -- always-valid fallback: the segment end on the rail (NEVER a full lock)
    tx = b.x - lz * rail
    tz = b.z + lx * rail
  end

  local dx = tx - car.position.x
  local dz = tz - car.position.z
  local dist = math.sqrt(dx * dx + dz * dz)
  if dist < 1.0 then return p.steer end

  -- alpha: heading-to-target angle
  local alpha = signedAngle(fx, fz, dx / dist, dz / dist)

  -- bicycle-model pure pursuit
  local delta = math.atan(2.0 * WHEELBASE * math.sin(alpha) / ld)

  -- normalize to the steering input; slight slip damping on top
  local steer = delta / STEER_LOCK + p.slipAngle * 0.2

  return clamp(steer, -CFG.STEER_LIMIT, CFG.STEER_LIMIT)
end

local function filterSteering(p, target, dt)
  -- rails need fast, direct steering: minimal filtering, bounded rate
  local alpha = clamp(dt / (0.08 + dt), 0, 1)
  p.steer = clamp(lerp(p.steer, target, alpha),
    -CFG.STEER_LIMIT, CFG.STEER_LIMIT)
  return p.steer
end

-- is the given rail clear of other cars for PASS_CLEAR meters ahead?
local function railClear(carIndex, car, railLat)
  local sim = ac.getSim()
  if not sim then return true end
  local fx, fz = normalize2(car.look.x, car.look.z)
  local seg = select(1, H.findNearestSeg(car.position, 1))
  for i = 0, sim.carsCount - 1 do
    if i ~= carIndex then
      local other = ac.getCar(i)
      if other and other.isActive then
        local dx = other.position.x - car.position.x
        local dz = other.position.z - car.position.z
        local forward = dx * fx + dz * fz
        if forward > -3 and forward < PASS_CLEAR then
          local oseg = select(1, H.findNearestSeg(other.position, seg))
          local ooff = H.lateralOf(other.position, oseg)
          if math.abs(ooff - railLat) < FOLLOW_LAT then
            return false
          end
        end
      end
    end
  end
  return true
end

-- blocker ahead IN OUR OWN RAIL (diagonal neighbors don't count)
local function blockerAhead(carIndex, car, railLat)
  local sim = ac.getSim()
  if not sim then return nil, nil end
  local fx, fz = normalize2(car.look.x, car.look.z)
  local bestGap, bestSpeed = nil, nil
  for i = 0, sim.carsCount - 1 do
    if i ~= carIndex then
      local other = ac.getCar(i)
      if other and other.isActive then
        local dx = other.position.x - car.position.x
        local dz = other.position.z - car.position.z
        local forward = dx * fx + dz * fz
        if forward > 0 and forward < 60 then
          local oseg = select(1, H.findNearestSeg(other.position, 1))
          local ooff = H.lateralOf(other.position, oseg)
          if math.abs(ooff - railLat) < FOLLOW_LAT then
            if not bestGap or forward < bestGap then
              bestGap = forward
              bestSpeed = math.max(0, safeNumber(other.speedMs))
            end
          end
        end
      end
    end
  end
  return bestGap, bestSpeed
end

-- decisive single lane change into a CLEAR adjacent lane
local function beginOvertake(carIndex, p, car, gap, centers)
  p.followTimer = 0
  p.passT = 0
  local from = clamp(p.curLane, 1, #centers)
  local targetLane
  if from > 1 and railClear(carIndex, car, centers[from - 1]) then
    targetLane = from - 1
  elseif from < #centers and railClear(carIndex, car, centers[from + 1]) then
    targetLane = from + 1
  end
  if not targetLane then return false end
  p.overtakeSide = (targetLane < from) and -1 or 1
  p.curLane = targetLane
  H.log("CAR=%d LANE_CHANGE from=%d to=%d gap=%.1f",
    carIndex, from, targetLane, gap or -1)
  return true
end

local function updateOvertake(carIndex, car, p, speed, targetSpeed, dt)
  local centers = laneCenters()
  local railLat = centers[clamp(p.curLane, 1, #centers)]

  -- passing: hold the lane; return home when the pass is done
  if p.overtakeSide ~= 0 then
    p.passT = p.passT + dt
    local gap, leaderSpeed = blockerAhead(carIndex, car, railLat)
    if p.passT > PASS_TIMEOUT or (not gap and p.passT > 1.0) then
      p.overtakeSide = 0
      p.passT = 0
      if p.curLane ~= p.homeLane and railClear(carIndex, car, centers[p.homeLane]) then
        p.curLane = p.homeLane
        H.log("CAR=%d LANE_RETURN home=%d", carIndex, p.homeLane)
      end
      return targetSpeed
    end
    if gap then
      local safe = math.sqrt(math.max(0,
        (leaderSpeed or 0) * (leaderSpeed or 0) +
        2 * CFG.BRAKE_ACCEL * math.max(0, gap - MIN_GAP)))
      return math.min(math.max(targetSpeed, (leaderSpeed or 0) + 3), math.max(safe, 6))
    end
    return targetSpeed
  end

  local gap, leaderSpeed = blockerAhead(carIndex, car, railLat)
  if not gap then
    p.followTimer = 0
    return targetSpeed
  end

  -- slower blocker in our rail: follow at a safe speed; change lanes when moving
  if (leaderSpeed or 0) < math.min(p.cruiseSpeed, targetSpeed + 3) - 2 then
    local safe = math.sqrt(math.max(0,
      (leaderSpeed or 0) * (leaderSpeed or 0) +
      2 * CFG.BRAKE_ACCEL * math.max(0, gap - MIN_GAP)))
    if speed > 5 then
      p.followTimer = p.followTimer + dt
      if p.followTimer > TRIGGER_TIME and #centers >= 2 then
        if beginOvertake(carIndex, p, car, gap, centers) then
          return math.max(targetSpeed, (leaderSpeed or 0) + 3)
        end
      end
    end
    return math.min(targetSpeed, safe)
  end

  p.followTimer = 0
  return targetSpeed
end

local function calculateThrottle(p, car, speed, targetSpeed)
  local error = targetSpeed - speed
  local gas = clamp(error * CFG.GAS_GAIN, 0, CFG.MAX_GAS)

  -- ANTI-BURNOUT: cut gas when the driven wheels slip
  local slip = wheelSlip(car)
  if slip and slip > 0.3 then
    gas = gas * clamp(1.0 - (slip - 0.3) * 1.5, 0.15, 1.0)
  end
  -- standstill: gentle launch, never a burnout
  if speed < 3 then
    gas = math.min(gas, 0.45)
  end

  local brake = clamp(-error * CFG.BRAKE_GAIN, 0, CFG.MAX_BRAKE)
  if error >= 0 then brake = 0 end
  -- smooth in traffic: never slam to a full stop from far away
  if brake > 0.5 and speed > 5 then brake = 0.5 end
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

  local speed, fx, fz = readHeading(car, p, dt)

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

  calculateThrottle(p, car, speed, targetSpeed)

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
  local bl, br2 = H.boundsAt(p.segIdx)
  return string.format(
    "CAR=%d POS=(%.2f,%.2f,%.2f) SPEED=%.2f STEER=%.3f GAS=%.3f BRAKE=%.3f TARGET_SPEED=%.2f LINE_ERROR=%.2f SEGMENT=%d PACE=%.2f LANE=%d/%d BOUNDS=[%.1f,%.1f] OFF=%.1f OT=%d",
    i, car.position.x, car.position.y, car.position.z,
    safeNumber(car.speedMs), p.steer, p.gas, p.brake, p.targetSpeed,
    p.lineDist, p.segIdx, p.pace, p.curLane, p.homeLane, bl, br2, p.latOff, p.overtakeSide)
end

return Driver
