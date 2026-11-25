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
local Geometry = require("route_geometry")
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
  CFG, state, H = cfg, st, helpers
  state.routeGeometry = Geometry.build(state.nodes, state.loopRoute, 8)
  state.routeLength = state.routeGeometry and state.routeGeometry.total or 0
  state.routeMetricCount = #state.nodes
  state.routeMetricLoop = state.loopRoute
  state.playerTelemetry = nil
end

local function refreshRouteMetrics()
  if state.routeGeometry and state.routeMetricCount == #state.nodes and
     state.routeMetricLoop == state.loopRoute then return end
  state.routeGeometry = Geometry.build(state.nodes, state.loopRoute, 8)
  state.routeLength = state.routeGeometry and state.routeGeometry.total or 0
  state.routeMetricCount = #state.nodes
  state.routeMetricLoop = state.loopRoute
  state.playerTelemetry = nil
  for _, p in pairs(state.p) do
    p.routeLocked = false
    p.routeProgress = nil
  end
end

local function routeProjection(car, hintSeg, allowGlobal, hintProgress, speed, dt)
  refreshRouteMetrics()
  if not state.routeGeometry then return nil end
  local result = Geometry.project(state.routeGeometry, car.position,
    hintProgress, car.look, speed or safeNumber(car.speedMs), dt or 0.05,
    allowGlobal ~= true and hintProgress ~= nil)
  if not result then return nil end
  return result.s, result.lateral, result.seg, result.t, result.distance,
    result.tx, result.tz
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
  -- All traffic cars use the fast class; passing is handled by the separate adjacent-lane controller.
  local slow = false
  local kmh = clamp(safeNumber(state.speedMax, 170), 80, 240)
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
  refreshRouteMetrics()
  local route=state.routeGeometry
  if not route or not p.routeProgress then return math.min(p.cruiseSpeed, 11.0) end
  local target=p.cruiseSpeed
  local distance=0
  while distance<=CFG.PLAN_HORIZON do
    local sample=Geometry.sample(route,p.routeProgress+distance)
    if not sample then break end
    local curvature=math.abs(sample.k or 0)
    if curvature>CFG.CURVE_MIN then
      local curveSpeed=math.sqrt(CFG.MAX_LATERAL_ACCEL/curvature)
      local safeSpeed=math.sqrt(curveSpeed*curveSpeed+2*CFG.BRAKE_ACCEL*distance)
      if safeSpeed<target then target=safeSpeed end
    end
    distance=distance+5.0
    if not route.loop and p.routeProgress+distance>=route.total then break end
  end
  return math.max(0,target)
end

local function clampLat(seg, lat)
  local bl, br = H.boundsAt(seg)
  return clamp(lat, bl + BOUND_MARGIN, br - BOUND_MARGIN)
end

local function calculateSteering(car, p, speed, fx, fz, dt)
  refreshRouteMetrics()
  local route=state.routeGeometry
  if not route then
    p.lineDist=0; p.crossTrackError=0; p.headingError=0
    p.pathCurvature=0; p.steerCurvature=0
    return 0
  end

  -- Project onto the continuous spline, then aim at a point measured by
  -- arc distance. The controller never aims at the next raw recorded node.
  local projection=Geometry.project(route,car.position,p.routeProgress,car.look,
    speed,dt,p.routeLocked and p.routeProgress~=nil)
  if not projection then
    p.projectionLost=true; p.targetSpeed=0; p.gas=0; p.brake=1; p.lineDist=999
    return 0
  end
  p.projectionLost=false; p.routeLocked=true
  p.routeProgress=projection.s; p.segIdx=projection.seg
  p.lineDist=projection.distance; p.latOff=projection.lateral

  -- Speed-dependent preview starts steering before the bend. Pure pursuit
  -- supplies the geometric turn command; spline curvature adds anticipation.
  local ahead=clamp(CFG.LOOKAHEAD_BASE+speed*CFG.LOOKAHEAD_SPEED,
    CFG.LOOKAHEAD_MIN,CFG.LOOKAHEAD_MAX)
  local bl,br=H.boundsAt(p.segIdx)
  local mid,width=(bl+br)*0.5,br-bl
  local leftCenter=width>=5.4 and ((mid+br)*0.5) or mid
  local rightCenter=(bl+mid)*0.5
  p.laneOffset=p.overtakeSide==0 and leftCenter or rightCenter
  local laneTarget=clampLat(p.segIdx,p.laneOffset or leftCenter)
  p.laneOffsetCurrent=p.laneOffsetCurrent or laneTarget
  local maxStep=1.5*math.max(0,dt or 0)
  p.laneOffsetCurrent=p.laneOffsetCurrent+
    clamp(laneTarget-p.laneOffsetCurrent,-maxStep,maxStep)

  local current=Geometry.sample(route,projection.s)
  local future=Geometry.sample(route,projection.s+ahead)
  local curvePreview=Geometry.sample(route,projection.s+ahead*0.55)
  if not current or not future or not curvePreview then return 0 end
  local targetX=future.x-future.tz*p.laneOffsetCurrent
  local targetZ=future.z+future.tx*p.laneOffsetCurrent
  local dx,dz=targetX-car.position.x,targetZ-car.position.z
  local targetDist=math.sqrt(dx*dx+dz*dz)
  if targetDist<0.5 then return 0 end

  -- Pure-pursuit curvature from the preview point, plus modest curvature
  -- feed-forward to begin rotating into the bend before heading error grows.
  local alpha=signedAngle(fx,fz,dx/targetDist,dz/targetDist)
  local pursuitCurvature=2.0*math.sin(alpha)/targetDist
  local previewCurvature=safeNumber(curvePreview.k)
  local desiredCurvature=pursuitCurvature+
    previewCurvature*CFG.CURVATURE_FEEDFORWARD

  p.headingError=signedAngle(fx,fz,future.tx,future.tz)
  p.crossTrackError=p.laneOffsetCurrent-projection.lateral
  p.pathCurvature=safeNumber(current.k)
  p.steerCurvature=desiredCurvature
  p.projectionProgressDelta=projection.delta or 0

  -- Small slip-angle damping is secondary; the spline preview is the
  -- primary steering signal.
  local steer=desiredCurvature*CFG.CURVATURE_STEER_GAIN+p.slipAngle*0.18
  return clamp(steer,-CFG.STEER_LIMIT,CFG.STEER_LIMIT)
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
    ownProgress = routeProjection(car, own and own.segIdx or 1, not (own and own.routeLocked), own and own.routeProgress, car.speedMs, 0.05)
  end
  if ownProgress == nil then return true end
  for i = 0, sim.carsCount - 1 do
    if i ~= carIndex and i ~= ignoreIndex then
      local other = ac.getCar(i)
      if other and other.isActive then
        local otherState = state.p[i]
        local progress, lateral, otherSeg = routeProjection(other,
          otherState and otherState.segIdx or own.segIdx,
          not (otherState and otherState.routeLocked),
          otherState and otherState.routeProgress, other.speedMs, 0.05)
        if otherState then
          if progress ~= nil then otherState.routeProgress = progress end
          otherState.routeLocked = progress ~= nil
          if otherSeg then otherState.segIdx = otherSeg end
        end
        if progress ~= nil then
          local longitudinal = routeGap(ownProgress, progress)
          local targetOffset = otherState and
            (otherState.laneOffset or otherState.laneOffsetCurrent) or lateral
          -- Occupancy is based on physical body overlap, not a narrow
          -- centreline threshold. This also detects the player on the route
          -- centre when the AI's nominal lane centre is offset to the left.
          local ownHalfWidth = math.max(0.75,
            safeNumber(car.aabbSize and car.aabbSize.x, 2.0) * 0.5)
          local otherHalfWidth = math.max(0.75,
            safeNumber(other.aabbSize and other.aabbSize.x, 2.0) * 0.5)
          local lateralClearance = ownHalfWidth + otherHalfWidth + 0.25
          local reserved = math.abs(targetOffset - laneOffset) < lateralClearance
          local actual = math.abs(lateral - laneOffset) < lateralClearance
          local otherSpeed = math.max(0, safeNumber(other.speedMs))
          local ownSpeed = math.max(0, safeNumber(car.speedMs))
          local rearWindow = math.max(18, (ownSpeed + otherSpeed) * 0.375 + 8)
          local frontWindow = math.max(48, math.max(ownSpeed, otherSpeed) * 1.2 + 10)
          if (actual or reserved) and
             longitudinal > -rearWindow and longitudinal < frontWindow then
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
    ownProgress = routeProjection(car, own.segIdx, not own.routeLocked, own.routeProgress, car.speedMs, 0.05)
    own.routeProgress = ownProgress
  end
  if ownProgress == nil then return nil, nil, nil end
  local bestGap, bestSpeed, bestIndex = nil, nil, nil
  for i = 0, sim.carsCount - 1 do
    if i ~= carIndex then
      local other = ac.getCar(i)
      if other and other.isActive then
        local otherState = state.p[i]
        local progress, lateral, otherSeg = routeProjection(other,
          otherState and otherState.segIdx or own.segIdx,
          not (otherState and otherState.routeLocked),
          otherState and otherState.routeProgress, other.speedMs, 0.05)
        if otherState then
          if progress ~= nil then otherState.routeProgress = progress end
          otherState.routeLocked = progress ~= nil
          if otherSeg then otherState.segIdx = otherSeg end
        end
        local targetOffset = otherState and
          (otherState.laneOffset or otherState.laneOffsetCurrent) or lateral
        local ownHalfWidth = math.max(0.75,
          safeNumber(car.aabbSize and car.aabbSize.x, 2.0) * 0.5)
        local otherHalfWidth = math.max(0.75,
          safeNumber(other.aabbSize and other.aabbSize.x, 2.0) * 0.5)
        local lateralClearance = ownHalfWidth + otherHalfWidth + 0.25
        if progress ~= nil and
           (math.abs(lateral - laneOffset) < lateralClearance or
            math.abs(targetOffset - laneOffset) < lateralClearance) then
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
  local ownProgress, _, ownSeg = routeProjection(car, p.segIdx, not p.routeLocked, p.routeProgress, speed, dt)
  if ownProgress == nil then
    p.projectionLost = true
    return 0
  end
  p.routeLocked = true
  if ownSeg then p.segIdx = ownSeg end
  p.routeProgress = ownProgress
  local leftCenter, rightCenter = laneCenters(p.segIdx)
  local gap, leaderSpeed, leaderIndex = carAheadInLane(
    carIndex, car, leftCenter or 0)

  if p.overtakeSide ~= 0 then
    p.passT = p.passT + dt
    local target = p.passTargetIndex and ac.getCar(p.passTargetIndex) or nil
    local passedClearance = false
    if target and target.isActive then
      local targetState = state.p[p.passTargetIndex]
      local targetProgress, _, targetSeg = routeProjection(target,
        targetState and targetState.segIdx or p.segIdx,
        not (targetState and targetState.routeLocked),
        targetState and targetState.routeProgress, target.speedMs, 0.05)
      if targetState then
        if targetProgress ~= nil then targetState.routeProgress = targetProgress end
        targetState.routeLocked = targetProgress ~= nil
        if targetSeg then targetState.segIdx = targetSeg end
      end
      local currentProgress = ownProgress or routeProjection(car, p.segIdx, false, p.routeProgress, speed, dt)
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

  if not leftCenter then
    if not p.slow and not p.overtakeDisabledLogged then
      local bl, br = H.boundsAt(p.segIdx)
      H.log("CAR=%d OVERTAKE_DISABLED corridor_too_narrow width=%.2f required=5.40",
        carIndex, br - bl)
      p.overtakeDisabledLogged = true
    end
    return targetSpeed
  end
  p.overtakeDisabledLogged = false
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

local function readWheelSlip(car, p)
  -- Documented CSP StateCar.wheels[0..3] fields. Positive longitudinal
  -- slip is wheelspin; negative braking slip does not trigger torque cut.
  local maxSlip=0
  local wheels=car and car.wheels
  if wheels then
    for i=0,3 do
      local wheel=wheels[i]
      if wheel then
        local ratio=safeNumber(wheel.slipRatio)
        local speedDiff=safeNumber(wheel.speedDifference)
        local normalizedDiff=speedDiff/math.max(3.0,safeNumber(car.speedMs))
        maxSlip=math.max(maxSlip,ratio,normalizedDiff)
      end
    end
  end
  p.wheelSlip=math.max(0,maxSlip)
  return p.wheelSlip
end

local function calculateThrottle(p, speed, targetSpeed, dt)
  local error=targetSpeed-speed
  local gas=clamp(error*CFG.GAS_GAIN,0,CFG.MAX_GAS)
  local brake=clamp(-error*CFG.BRAKE_GAIN,0,CFG.MAX_BRAKE)

  -- Fast torque cut on wheelspin, slow recovery to avoid repeated grip/spin.
  local slip=math.max(0,safeNumber(p.wheelSlip))
  local slipAmount=clamp((slip-CFG.TC_SLIP_START)/
    math.max(0.01,CFG.TC_SLIP_FULL-CFG.TC_SLIP_START),0,1)
  local desiredScale=1.0-slipAmount*CFG.TC_MAX_CUT
  p.tractionScale=p.tractionScale or 1.0
  if desiredScale<p.tractionScale then
    p.tractionScale=desiredScale
  else
    p.tractionScale=math.min(desiredScale,
      p.tractionScale+math.max(0,dt or 0)*CFG.TC_RECOVERY_RATE)
  end

  -- Leave grip reserve for cornering instead of applying full drive torque
  -- while lateral acceleration is already close to the route limit.
  local lateralUse=clamp(math.abs(safeNumber(p.lateralAccel))/
    math.max(0.1,CFG.MAX_LATERAL_ACCEL),0,1)
  local cornerScale=1.0-CFG.TC_CORNER_CUT*lateralUse*lateralUse
  gas=gas*p.tractionScale*cornerScale

  if speed<5 then gas=math.min(gas,0.6) end
  if error>=0 then brake=0 end
  p.gas=clamp(gas,0,CFG.MAX_GAS)
  p.brake=clamp(brake,0,CFG.MAX_BRAKE)
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
  local progress, _, projectedSeg = routeProjection(car, p.segIdx, not p.routeLocked, p.routeProgress, speed, dt)
  if progress ~= nil then
    p.routeLocked = true
    p.routeProgress = progress
    p.projectionLost = false
    if projectedSeg then p.segIdx = projectedSeg end
  else
    p.projectionLost = true
  end
  calculateStability(p, speed)
  readWheelSlip(car,p)

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

  local targetSpeed = p.projectionLost and 0 or planTargetSpeed(p)
  targetSpeed = updateOvertake(carIndex, car, p, speed, targetSpeed, dt)
  p.targetSpeed = targetSpeed

  local steerTarget = calculateSteering(car, p, speed, fx, fz, dt)
  if p.projectionLost then targetSpeed=0 end
  local steer = filterSteering(p, steerTarget, dt)

  calculateThrottle(p, speed, targetSpeed, dt)
  if p.projectionLost then p.gas=0; p.brake=1 end

  local ok, controls = pcall(ac.overrideCarControls, carIndex)
  if ok and controls then
    controls.combineAxis = false
    controls.steer = steer
    controls.gas = p.gas
    controls.brake = p.brake
    controls.handbrake = 0
  end
end

-- Observe the human-driven car without touching its controls.
-- Called every update frame so route progress remains continuous; the shared
-- one-second logger writes the latest snapshot using the AI telemetry schema.
function Driver.observePlayer(car, dt, steerInput)
  if not car or not car.isActive then return nil end
  refreshRouteMetrics()
  if not state.routeGeometry then return nil end
  local p=state.playerTelemetry
  if not p then
    p={
      segIdx=1, routeLocked=false, routeProgress=nil,
      previousFx=nil, previousFz=nil,
      steer=0, gas=0, brake=0, targetSpeed=-1,
      lineDist=0, latOff=0, crossTrackError=0,
      headingError=0, pathCurvature=0,
      overtakeSide=0, laneOffsetCurrent=0, pace=0,
      slipAngle=0, lateralAccel=0, yawRate=0,
      slipSeverity=0, stability=1, wheelSlip=0, tractionScale=1,
      steerCurvature=0
    }
    state.playerTelemetry=p
  end

  local speed,fx,fz=readVehicleDynamics(car,p,dt or 0.016)
  calculateStability(p,speed)
  readWheelSlip(car,p)
  local progress,lateral,seg,_,distance=routeProjection(
    car,p.segIdx,not p.routeLocked,p.routeProgress,speed,dt or 0.016)
  if progress~=nil then
    p.routeLocked=true
    p.routeProgress=progress
    p.segIdx=seg or p.segIdx
    p.lineDist=distance or 0
    p.latOff=lateral or 0
    p.crossTrackError=-(lateral or 0)
    local sample=Geometry.sample(state.routeGeometry,progress)
    if sample then
      p.headingError=signedAngle(fx,fz,sample.tx,sample.tz)
      p.pathCurvature=safeNumber(sample.k)
    end
  else
    p.routeLocked=false
    p.lineDist=999
    p.crossTrackError=0
    p.headingError=0
    p.pathCurvature=0
  end

  p.steer=safeNumber(steerInput)
  p.gas=clamp(safeNumber(car.gas),0,1)
  p.brake=clamp(safeNumber(car.brake),0,1)
  p.targetSpeed=-1 -- not applicable to a manually driven car
  p.overtakeSide=0
  p.laneOffsetCurrent=0
  p.pace=0
  p.player=true
  return p
end

function Driver.telemetryLine(i, car, p)
  local bl, br = H.boundsAt(p.segIdx)
  local steerLimit=p.player and 0.99 or (CFG.STEER_LIMIT-0.01)
  local mode=p.player and " MODE=PLAYER" or ""
  return string.format(
    "CAR=%d POS=(%.2f,%.2f,%.2f) SPEED=%.2f STEER=%.3f GAS=%.3f BRAKE=%.3f TARGET_SPEED=%.2f LINE_ERROR=%.2f SEGMENT=%d PACE=%.2f BOUNDS=[%.1f,%.1f] OFF=%.1f OT_SIDE=%d CROSS=%.2f HEAD=%.3f CURVE=%.4f ROUTE=%.1f LANE=%.2f SAT=%d WHEEL_SLIP=%.3f TC_SCALE=%.2f STEER_CURV=%.4f%s",
    i, car.position.x, car.position.y, car.position.z,
    safeNumber(car.speedMs), p.steer, p.gas, p.brake, p.targetSpeed,
    p.lineDist, p.segIdx, (p.pace or 0), bl, br, p.latOff, p.overtakeSide,
    p.crossTrackError or 0, p.headingError or 0, p.pathCurvature or 0,
    p.routeProgress or 0, p.laneOffsetCurrent or 0,
    math.abs(p.steer or 0) >= steerLimit and 1 or 0,
    safeNumber(p.wheelSlip), safeNumber(p.tractionScale,1),
    safeNumber(p.steerCurvature), mode)
end

return Driver