-- RoutePilot/driver.lua — two-lane race traffic (CSP 0.3.2-preview2)
-- Both lanes are unrestricted on straights. Cars brake for route curvature
-- and traffic, and change to the adjacent lane only when its swept corridor is clear.

local Driver = {}
local CFG, state, H

local LANE_CHANGE_TIME = 1.65
local LANE_CHANGE_COOLDOWN = 2.0
local LANE_TRIGGER_DISTANCE = 10.0
local LANE_TRIGGER_TTC = 2.2
local LANE_FRONT_CLEAR = 32.0
local LANE_REAR_CLEAR = 15.0
local LANE_SAFETY_GAP = 5.0
local LANE_HALF_WIDTH = 1.45
local LANE_CHANGE_RATE = 2.1
-- Conservative, rate-limited steering prevents large snap corrections.
local STEER_LIMIT = 0.68
local STEER_GAIN = 0.78
local STEER_FILTER_TIME = 0.22
local STEER_RATE = 1.5
local LOOKAHEAD_BASE = 16.0
local LOOKAHEAD_SPEED = 0.55
local LOOKAHEAD_MAX = 75.0
local LANE_RESCUE_SPEED = 1.5
local LANE_RESCUE_TIME = 5.0
local GAS_RISE_RATE = 1.8
local GAS_RELEASE_RATE = 5.0
local GAS_SPIN_CUT_RATE = 10.0

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

local function ensureTractionAssist(carIndex,p)
  if p.tcApplied then return end
  if physics and type(physics.setAssists)=="function" then
    local ok,result=pcall(physics.setAssists,
      {tractionControl="on",editable=false},carIndex)
    p.tcApplied=true
    if ok then
      H.log("TC_ASSIST id=%d applied=%s",carIndex,tostring(result))
    else
      H.log("TC_ASSIST_ERROR id=%d err=%s",carIndex,tostring(result))
    end
  else
    p.tcApplied=true
    H.log("TC_ASSIST_UNAVAILABLE id=%d",carIndex)
  end
end

local function measureDrivenWheelSlip(car)
  local wheels=car.wheels
  if not wheels then return 0 end
  local tractionType=safeNumber(car.tractionType,-1)
  local indices
  if tractionType==0 then
    indices={2,3}                 -- RWD
  elseif tractionType==1 then
    indices={0,1}                 -- FWD
  else
    indices={0,1,2,3}             -- AWD or unknown: fail-safe
  end
  local maximum=0
  for i=1,#indices do
    local wheel=wheels[indices[i]]
    if wheel then
      local slip=math.abs(safeNumber(wheel.slipRatio,0))
      if slip>maximum then maximum=slip end
    end
  end
  return maximum
end

local function applyTractionLimiter(car,p,dt)
  p.wheelSlip=measureDrivenWheelSlip(car)
  -- Factory TC is forced on above; this second layer catches cars without
  -- usable factory TC and trims throttle before sustained wheelspin develops.
  local factor=1.0
  if p.wheelSlip>0.10 then
    factor=clamp(1.0-(p.wheelSlip-0.10)*1.75,0.22,1.0)
  end
  p.tractionFactor=factor
  local requested=math.min(p.gas,factor)
  local rate
  if p.brake > 0.02 then
    requested=0
    rate=GAS_SPIN_CUT_RATE
  elseif requested<p.gasOutput then
    rate=(p.wheelSlip>0.14) and GAS_SPIN_CUT_RATE or GAS_RELEASE_RATE
  else
    rate=GAS_RISE_RATE
  end
  p.gasOutput=p.gasOutput+clamp(requested-p.gasOutput,-rate*dt,rate*dt)
  p.gas=clamp(p.gasOutput,0,1)
end

local function laneRoleForOffset(offset, twoLane)
  if not twoLane then return "SINGLE" end
  return offset < 0 and "LEFT" or "RIGHT"
end

local function newPersonality(carIndex)
  local laneOffset, twoLane = H.laneOffset(carIndex, 1)
  local p = {
    segIdx = 1,
    laneRole = laneRoleForOffset(laneOffset, twoLane),
    laneOffset = laneOffset,
    laneStartOffset = laneOffset,
    laneTargetOffset = laneOffset,
    laneChangeProgress = 1.0,
    laneCooldown = 0,
    twoLane = twoLane,
    steer = 0,
    steerFiltered = 0,
    gas = 0,
    gasOutput = 0,
    brake = 0,
    speedIntegral = 0,
    wheelSlip = 0,
    tractionFactor = 1.0,
    tcApplied = false,
    slipAngle = 0,
    previousFx = nil,
    previousFz = nil,
    stuck = 0,
    targetSpeed = math.huge,
    lineDist = 0,
    latOff = 0,
    retireT = 2.0,
    trafficGap = math.huge,
    trafficBlocked = false,
    laneChanges = 0
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

local function planRaceSpeed(p, speed)
  local nodes = state.nodes
  local segments = H.segmentCount()
  if segments < 1 then return nil end

  -- Dynamic look-ahead includes the full stopping distance at the current speed.
  local horizon = math.max(CFG.PLAN_HORIZON,
    speed * speed / (2 * CFG.BRAKE_ACCEL) + 45.0)
  horizon = math.min(horizon, 1400.0)

  local target = math.huge
  local distance = 0
  local current = p.segIdx
  local steps = 0
  while distance <= horizon and steps < 2400 do
    steps = steps + 1
    local curvature = H.getVertexCurvature(current)
    if curvature > CFG.CURVE_MIN then
      local cornerSpeed = math.sqrt(CFG.MAX_LATERAL_ACCEL / curvature)
      local brakingSpeed = math.sqrt(cornerSpeed * cornerSpeed +
        2 * CFG.BRAKE_ACCEL * math.max(0, distance - 3.0))
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
  if target == math.huge then return nil end
  return target
end

local function laneOffsetAtPosition(car, ownTangentX, ownTangentZ)
  local dx = car.position.x
  local dz = car.position.z
  local lateral = dx * -ownTangentZ + dz * ownTangentX
  return lateral
end

local function scanTraffic(carIndex, car, p, speed, tx, tz)
  local sim = ac.getSim()
  if not sim then return nil end
  local ownLateral = H.lateralOf(car.position, p.segIdx)
  local nearest, nearestGap = nil, math.huge
  for i = 0, sim.carsCount - 1 do
    if i ~= carIndex then
      local other = ac.getCar(i)
      if other and other.isActive then
        local dx = other.position.x - car.position.x
        local dz = other.position.z - car.position.z
        local longitudinal = dx * tx + dz * tz
        local lateral = dx * -tz + dz * tx
        local otherP = state.p[i]
        local otherLaneCoordinate = otherP and otherP.laneOffset or
          (ownLateral + lateral)
        local sameLane = math.abs(otherLaneCoordinate - p.laneOffset) <= LANE_HALF_WIDTH
        if longitudinal > 0.5 and sameLane and longitudinal < nearestGap then
          nearestGap = longitudinal
          nearest = {
            index = i,
            gap = longitudinal,
            speed = math.max(0, safeNumber(other.speedMs, 0)),
            closing = speed - math.max(0, safeNumber(other.speedMs, 0))
          }
        end
      end
    end
  end
  return nearest
end

local function adjacentLaneIsClear(carIndex, car, p, targetOffset, tx, tz)
  local sim = ac.getSim()
  if not sim then return false end
  local ownLateral = H.lateralOf(car.position, p.segIdx)
  for i = 0, sim.carsCount - 1 do
    if i ~= carIndex then
      local other = ac.getCar(i)
      if other and other.isActive then
        local dx = other.position.x - car.position.x
        local dz = other.position.z - car.position.z
        local longitudinal = dx * tx + dz * tz
        local lateral = dx * -tz + dz * tx
        local otherP = state.p[i]
        local otherSpeed = math.max(0, safeNumber(other.speedMs, 0))
        local otherLaneCoordinate = otherP and otherP.laneOffset or
          (ownLateral + lateral)
        local reservedOffset = otherP and
          (otherP.laneTargetOffset or otherP.laneOffset) or otherLaneCoordinate
        local inTargetLane = math.abs(otherLaneCoordinate - targetOffset) <= LANE_HALF_WIDTH
        local reservingTarget = otherP and
          math.abs(reservedOffset - targetOffset) <= LANE_HALF_WIDTH
        local frontClear = math.max(LANE_FRONT_CLEAR,
          math.max(0, safeNumber(car.speedMs, 0) - otherSpeed) * 1.5 + 10.0)
        local rearClear = math.max(LANE_REAR_CLEAR,
          math.max(0, otherSpeed - safeNumber(car.speedMs, 0)) * 1.5 + 10.0)
        local inTransitionSweep = math.abs(longitudinal) < 10.0 and
          otherLaneCoordinate >= math.min(p.laneOffset, targetOffset) - LANE_HALF_WIDTH and
          otherLaneCoordinate <= math.max(p.laneOffset, targetOffset) + LANE_HALF_WIDTH
        if (inTargetLane or reservingTarget) and
          longitudinal >= -rearClear and longitudinal <= frontClear then
          return false, i, longitudinal
        end
        if inTransitionSweep then return false, i, longitudinal end
      end
    end
  end
  return true
end

local function updateLaneChange(carIndex, car, p, speed, tx, tz, dt)
  p.laneCooldown = math.max(0, p.laneCooldown - dt)
  p.trafficBlocked = false
  local front = scanTraffic(carIndex, car, p, speed, tx, tz)
  p.trafficGap = front and front.gap or math.huge

  if p.twoLane and p.laneChangeProgress >= 1.0 and p.laneCooldown <= 0 and front then
    local closing = math.max(0, front.closing)
    local ttc = closing > 0.1 and front.gap / closing or math.huge
    local trigger = front.gap <= LANE_TRIGGER_DISTANCE or
      ttc <= LANE_TRIGGER_TTC
    if trigger then
      local targetOffset = -p.laneTargetOffset
      local clear, blockedBy, blockedGap =
        adjacentLaneIsClear(carIndex, car, p, targetOffset, tx, tz)
      if clear then
        p.laneStartOffset = p.laneOffset
        p.laneTargetOffset = targetOffset
        p.laneChangeProgress = 0
        p.laneCooldown = LANE_CHANGE_COOLDOWN
        p.laneChanges = p.laneChanges + 1
        p.laneRole = laneRoleForOffset(targetOffset, true)
        H.log("LANE_CHANGE_START id=%d from=%.2f to=%.2f gap=%.2f closing=%.2f ttc=%.2f",
          carIndex, p.laneStartOffset, targetOffset, front.gap, closing, ttc)
      else
        p.trafficBlocked = true
        p.blockedBy = blockedBy
        p.blockedGap = blockedGap
      end
    end
  end

  if p.laneChangeProgress < 1.0 then
    p.laneChangeProgress = math.min(1.0,
      p.laneChangeProgress + dt / LANE_CHANGE_TIME)
    local t = p.laneChangeProgress
    local smooth = t * t * (3.0 - 2.0 * t)
    p.laneOffset = p.laneStartOffset +
      (p.laneTargetOffset - p.laneStartOffset) * smooth
    if p.laneChangeProgress >= 1.0 then
      p.laneOffset = p.laneTargetOffset
      H.log("LANE_CHANGE_DONE id=%d lane=%s changes=%d",
        carIndex, p.laneRole, p.laneChanges)
    end
  end

  -- Recheck the target lane while crossing; a newly arriving car blocks
  -- acceleration until the transition is complete.
  if p.laneChangeProgress < 1.0 then
    local clear = adjacentLaneIsClear(carIndex, car, p,
      p.laneTargetOffset, tx, tz)
    if not clear then p.trafficBlocked = true end
  end

  -- If the neighboring lane is unavailable, stay in lane and brake to a
  -- speed that can be stopped behind the leading vehicle.
  if front and p.trafficBlocked then
    local available = math.max(0, front.gap - LANE_SAFETY_GAP)
    local safeSpeed = math.sqrt(front.speed * front.speed +
      2 * CFG.BRAKE_ACCEL * available)
    return safeSpeed, front
  end
  if front and p.laneChangeProgress < 0.55 then
    local available = math.max(0, front.gap - LANE_SAFETY_GAP)
    return math.sqrt(front.speed * front.speed +
      2 * CFG.BRAKE_ACCEL * available), front
  end
  return nil, front
end

local function calculateSteering(carIndex, car, p, speed, fx, fz, dt)
  local seg, t, lineDistance = H.findNearestSeg(car.position, p.segIdx)
  p.segIdx = seg
  p.lineDist = lineDistance
  p.latOff = H.lateralOf(car.position, seg)

  local baseOffset, twoLane = H.laneOffset(carIndex, seg)
  p.twoLane = twoLane
  if not twoLane then
    p.laneOffset = 0
    p.laneTargetOffset = 0
    p.laneStartOffset = 0
    p.laneRole = "SINGLE"
  else
    -- Preserve the chosen lane while following the route; the lane controller
    -- changes this offset only after a verified safe gap.
    if p.laneTargetOffset == nil then
      p.laneOffset = baseOffset
      p.laneTargetOffset = baseOffset
      p.laneStartOffset = baseOffset
    end
    p.laneRole = laneRoleForOffset(p.laneTargetOffset, true)
  end

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
    -- A larger integral range is needed to overcome each car's real drag
    -- at 40 km/h; the old +/-18 clamp caused a permanent 35 km/h plateau.
    p.speedIntegral = clamp(p.speedIntegral + error * dt, -60, 60)
    drive = error * 0.18 + p.speedIntegral * 0.012
  end

  p.gas = clamp(drive, 0, 1)
  p.brake = clamp(-drive * 0.85, 0, 1)
  if error >= 0 then p.brake = 0 end
  if error <= 0 then p.gas = 0 end
end

function Driver.update(carIndex, car, dt)
  local p = state.p[carIndex] or newPersonality(carIndex)
  ensureTractionAssist(carIndex,p)

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

  local seg = select(1, H.findNearestSeg(car.position, p.segIdx))
  p.segIdx = seg
  local a = state.nodes[seg]
  local b = H.segEnd(seg)
  local tx, tz = normalize2(b.x - a.x, b.z - a.z)

  local trafficSpeed, front = updateLaneChange(
    carIndex, car, p, speed, tx, tz, dt)

  local steer = calculateSteering(carIndex, car, p, speed, fx, fz, dt)
  local cornerSpeed = planRaceSpeed(p, speed)
  local targetSpeed = cornerSpeed
  if trafficSpeed then
    targetSpeed = targetSpeed and math.min(targetSpeed, trafficSpeed) or trafficSpeed
  end

  if targetSpeed then
    p.targetSpeed = targetSpeed
    calculateSpeedControls(p, speed, targetSpeed, dt)
  else
    -- No corner or traffic constraint: maximum available throttle, no speed cap.
    p.targetSpeed = math.huge
    p.speedIntegral = 0
    p.gas = 1
    p.brake = 0
  end

  applyTractionLimiter(car,p,dt)

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
  local a=state.nodes[p.segIdx]
  local b=H.segEnd(p.segIdx)
  local tx,tz=normalize2(b.x-a.x,b.z-a.z)
  local look=car.look
  local headingDot=look.x*tx+look.z*tz
  local targetText=p.targetSpeed==math.huge and "OPEN" or string.format("%.2f",p.targetSpeed)
  local gapText=p.trafficGap==math.huge and "INF" or string.format("%.2f",p.trafficGap)
  return string.format(
    "CAR=%d LANE=%s POS=(%.2f,%.2f,%.2f) SPEED=%.2f TARGET=%s STEER=%.3f GAS=%.3f BRAKE=%.3f WHEEL_SLIP=%.3f TC_FACTOR=%.2f HEADING_DOT=%.3f LINE_ERROR=%.2f SEGMENT=%d LANE_OFFSET=%.2f LANE_TARGET=%.2f LANE_PROGRESS=%.2f LANE_CHANGES=%d TRAFFIC_GAP=%s TRAFFIC_BLOCKED=%s BOUNDS=[%.1f,%.1f]",
    i, p.laneRole, car.position.x, car.position.y, car.position.z,
    safeNumber(car.speedMs), targetText, p.steer, p.gas, p.brake,
    p.wheelSlip, p.tractionFactor, headingDot, p.lineDist, p.segIdx,
    p.laneOffset, p.laneTargetOffset or p.laneOffset, p.laneChangeProgress or 1,
    p.laneChanges or 0, gapText, tostring(p.trafficBlocked), left, right)
end

return Driver
