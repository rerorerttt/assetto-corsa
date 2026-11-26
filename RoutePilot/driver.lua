-- RoutePilot/driver.lua — v0.9.1 (CSP 0.3.2-preview2)
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
local RacingLine = require("racing_line")
local okVehicleDynamics, VehicleDynamics = pcall(require, "vehicle_dynamics")
if not okVehicleDynamics or type(VehicleDynamics) ~= "table" or
    type(VehicleDynamics.measure) ~= "function" then
  VehicleDynamics = nil
end
local CFG, state, H

local BOUND_MARGIN = 0.4
local REQ_SEP      = 2.8    -- center-to-center gap needed to pass, m
local TRIGGER_TIME = 0.5
local FORCE_TIME   = 2.0
local PASS_TIMEOUT = 8.0
local RESCUE_SPEED = 1.5
local RESCUE_TIME  = 10.0
local STEER_DEAD   = 0.002  -- suppress only sub-resolution steering noise

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
  state.trafficRequestSerial=state.trafficRequestSerial or 0
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

local INITIAL_PROJECTION_MAX = 8.0
local function initialProjectionValid(result, heading)
  if not result or safeNumber(result.distance, math.huge) > INITIAL_PROJECTION_MAX then
    return false
  end
  if heading then
    local hx,hz=normalize2(heading.x,heading.z)
    if hx*result.tx+hz*result.tz < -0.15 then return false end
  end
  return true
end

local function routeProjection(car, hintSeg, allowGlobal, hintProgress, speed, dt)
  refreshRouteMetrics()
  if not state.routeGeometry then return nil end
  local locked=allowGlobal ~= true and hintProgress ~= nil
  local result = Geometry.project(state.routeGeometry, car.position,
    hintProgress, car.look, speed or safeNumber(car.speedMs), dt or 0.05, locked)
  if not result then return nil end
  if not locked and not initialProjectionValid(result,car.look) then return nil end
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
  -- No artificial cruise-speed ceiling: the route planner and vehicle physics
  -- determine speed. Curvature and measured braking capability remain limits.
  local p = {
    segIdx = 1, cruiseSpeed = math.huge, brakeAccelEstimate = CFG.BRAKE_ACCEL,
    accelEstimate = 4.0, lateralAccelLimit = CFG.MAX_LATERAL_ACCEL,
    previousSpeed = nil, playerAutopilot = carIndex == 0,
    steer = 0, steerFiltered = 0, gas = 0, brake = 0,
    vehicleDynamicsState = VehicleDynamics and VehicleDynamics.newState() or nil,
    slipAngle = 0, lateralAccel = 0, yawRate = 0,
    previousFx = nil, previousFz = nil,
    lateralVelocityFiltered = 0,
    yawRateFiltered = 0,
    raceOffsetRaw = 0, raceOffsetCurrent = 0,
    pursuitCurvature = 0, boundarySteer = 0, lookahead = 0,
    stuck = 0, blockedT = 0, passT = 0,
    lineDist = 0, latOff = 0, targetSpeed = 0,
    overtakeSide = 0, passLat = 0, blockerOff = 0, followTimer = 0,
    laneOffset = nil, passTargetIndex = nil, retireT = 2.0,
    overtakeRetryT = 0, emergencyPass = false, returningFromPass = false,
    trafficPriority = 0,
    yieldRequestBy = nil, yieldOffset = nil, yieldReady = false,
    yieldRequestTimer = 0, pendingYieldTarget = nil,
    pendingPassOffset = nil, pendingYieldTimer = 0,
    pendingYieldGap = nil, pendingYieldSpeed = nil, pendingPassDistance = nil,
    pendingYieldGroup = nil, passGroupTargets = nil,
  }
  local spawnHint=state.spawnHints and state.spawnHints[carIndex]
  if spawnHint and state.routeGeometry then
    local seededProgress=Geometry.distanceAt(
      state.routeGeometry,spawnHint.segment,spawnHint.parameter)
    if seededProgress~=nil then
      p.routeProgress=seededProgress
      p.routeLocked=true
      p.spawnSeeded=true
      p.segIdx=spawnHint.segment
      H.log("ROUTE_SEED id=%d seg=%d t=%.3f progress=%.2f",
        carIndex,spawnHint.segment,spawnHint.parameter,seededProgress)
    else
      H.log("ROUTE_SEED_FAILED id=%d seg=%d t=%.3f",
        carIndex,spawnHint.segment,spawnHint.parameter)
    end
    state.spawnHints[carIndex]=nil
  end
  state.p[carIndex] = p
  H.log("CAR_INIT id=%d lane=LEFT cruise=PHYSICS_LIMITED",
    carIndex)
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
  if VehicleDynamics then
    p.vehicleDynamicsState = p.vehicleDynamicsState or VehicleDynamics.newState()
    local measured = VehicleDynamics.measure(car, p.vehicleDynamicsState)
    if measured then
      p.frontAxleSlip = measured.frontAxleAngle
      p.rearAxleSlip = measured.rearAxleAngle
      p.frontTireSlipDeg = measured.frontSlipDeg
      p.rearTireSlipDeg = measured.rearSlipDeg
      p.frontNdSlip = measured.frontNdSlip
      p.rearNdSlip = measured.rearNdSlip
      p.measuredYawRate = measured.yawRate
      p.measuredWheelbase = measured.wheelbase
      p.measuredSteerLock = measured.steerLockDeg
      p.vehicleDynamicsValid = true
    else
      p.vehicleDynamicsValid = false
    end
  end
  local speed = math.max(0, safeNumber(car.speedMs, longVel))
  -- Learn this car's real braking response from measured speed loss while
  -- the previous frame was braking. Ignore noisy samples and filter changes.
  if p.previousSpeed ~= nil and dt > 0.002 then
    if p.brake > 0.15 and speed < p.previousSpeed then
      local measured = (p.previousSpeed - speed) / dt
      if measured >= 1.5 and measured <= 25.0 then
        local estimate = clamp(measured, 2.5, 22.0)
        p.brakeAccelEstimate = lerp(
          safeNumber(p.brakeAccelEstimate, CFG.BRAKE_ACCEL), estimate, 0.12)
      end
    elseif p.gas > 0.2 and speed > p.previousSpeed then
      local measured = (speed - p.previousSpeed) / dt
      if measured >= 0.3 and measured <= 18.0 then
        local estimate = clamp(measured, 0.5, 18.0)
        p.accelEstimate = lerp(safeNumber(p.accelEstimate,4.0),estimate,0.08)
      end
    end
  end
  p.previousSpeed = speed
  p.previousFx, p.previousFz = fx, fz
  p.lateralAccel = math.abs(p.yawRate * speed)
  -- Learn usable cornering grip from actual vehicle response. Increase the
  -- limit only while the car is stable; reduce it when slip indicates loss.
  local grip=clamp(safeNumber(p.lateralAccelLimit,CFG.MAX_LATERAL_ACCEL),7.5,20.0)
  if speed>12.0 and math.abs(p.slipAngle)<0.08 and
     safeNumber(p.wheelSlip)<0.20 and p.lateralAccel>grip*0.70 then
    grip=lerp(grip,clamp(p.lateralAccel*1.18,grip,20.0),0.015)
  elseif speed>12.0 and
     (math.abs(p.slipAngle)>0.15 or safeNumber(p.wheelSlip)>0.30) and
     p.lateralAccel>3.0 then
    grip=lerp(grip,clamp(p.lateralAccel*0.95,7.5,grip),0.08)
  end
  p.lateralAccelLimit=grip
  return speed, fx, fz
end

local function planTargetSpeed(p,speed)
  refreshRouteMetrics()
  local route=state.routeGeometry
  if not route or not p.routeProgress then return 11.0 end
  local target=p.cruiseSpeed
  local brakeAccel=clamp(safeNumber(p.brakeAccelEstimate,CFG.BRAKE_ACCEL),2.5,22.0)
  local currentSpeed=math.max(0,safeNumber(speed))
  local brakingDistance=currentSpeed*currentSpeed/(2*brakeAccel)
  local horizon=math.max(CFG.PLAN_HORIZON,brakingDistance+currentSpeed*1.5+35.0)
  if p.racingPlanBlockedDistance then
    local stoppingRoom=math.max(0,p.racingPlanBlockedDistance-10.0)
    local corridorSpeed=math.sqrt(2.0*brakeAccel*stoppingRoom)
    target=math.min(target,corridorSpeed)
  end
  local distance=0
  while distance<=horizon do
    local planAdvance=p.racingPlanStartProgress and routeGap(p.racingPlanStartProgress,p.routeProgress) or 0
    local planDistance=distance+math.max(0,planAdvance)
    local planned=p.racingPlan and planDistance<=p.racingPlan.horizon and
      RacingLine.sample(p.racingPlan,planDistance) or nil
    local roadSample=Geometry.sample(route,p.routeProgress+distance)
    local sample=planned or roadSample
    if not sample or not roadSample then break end
    -- On an open run, use the recorded road curvature for braking.
    -- The optimizer's first plan stations can contain a short-lived geometric
    -- kink while the path is being rebuilt; treating that single value as a
    -- real corner can command a large speed drop on a visually straight.
    -- During a pass/return, the actual offset path is authoritative because
    -- the car is deliberately leaving the reference line.
    -- Braking must use the same optimized path that steering follows.
    local curvature=math.abs(safeNumber(planned and planned.k or roadSample.k))
    if curvature>math.max(CFG.CURVE_MIN,0.0018) then
      local lateralLimit=clamp(safeNumber(p.lateralAccelLimit,CFG.MAX_LATERAL_ACCEL),7.5,20.0)*0.90
      local curveSpeed=math.sqrt(lateralLimit/curvature)
      local brakingDistanceAvailable=math.max(0,distance-currentSpeed*0.25)
      local safeSpeed=math.sqrt(curveSpeed*curveSpeed+
        2*brakeAccel*brakingDistanceAvailable)
      if safeSpeed<target then target=safeSpeed end
    end
    distance=distance+5.0
    if not route.loop and p.routeProgress+distance>=route.total then break end
  end
  return math.max(0,target)
end

local function routeBoundsAtSample(sample)
  local count=#state.nodes
  local i=sample and sample.seg or 1
  if count<2 then return H.boundsAt(i) end
  if i<1 then i=1 elseif i>count then i=count end
  local j=i<count and i+1 or (state.loopRoute and 1 or i)
  local a,b=state.nodes[i],state.nodes[j]
  local bl0,br0=H.boundsAt(i)
  local bl1,br1=H.boundsAt(j)
  if not a or not b or i==j then return bl0,br0 end

  -- Saved bounds were measured against the raw node-to-node chord. Transform
  -- those boundary points into the smoothed spline's lateral frame before
  -- using them as a steering constraint.
  local rawTx,rawTz=normalize2(b.x-a.x,b.z-a.z)
  local rawNx,rawNz=-rawTz,rawTx
  local u=clamp(safeNumber(sample.t),0,1)
  local centerX=lerp(a.x,b.x,u)
  local centerZ=lerp(a.z,b.z,u)
  local left=lerp(bl0,bl1,u)
  local right=lerp(br0,br1,u)
  local splineNx,splineNz=-sample.tz,sample.tx
  local leftOffset=(centerX+rawNx*left-sample.x)*splineNx+
    (centerZ+rawNz*left-sample.z)*splineNz
  local rightOffset=(centerX+rawNx*right-sample.x)*splineNx+
    (centerZ+rawNz*right-sample.z)*splineNz
  return math.min(leftOffset,rightOffset),math.max(leftOffset,rightOffset)
end

local function safeCorridorForPreview(route, progress, ahead, car, fx, fz)
  local size=car and car.aabbSize
  local halfWidth=math.max(0.75,safeNumber(size and size.x,2.0)*0.5)
  local halfLength=math.max(1.5,safeNumber(size and size.z,4.5)*0.5)
  local safeMin,safeMax=-math.huge,math.huge
  local maxEnvelope=0
  local first=-halfLength
  local last=ahead+halfLength
  local distance=first
  while distance<=last+0.001 do
    local sample=Geometry.sample(route,progress+distance)
    if sample then
      local angle=signedAngle(fx,fz,sample.tx,sample.tz)
      local envelope=halfWidth*math.abs(math.cos(angle))+
        halfLength*math.abs(math.sin(angle))+BOUND_MARGIN
      local bl,br=routeBoundsAtSample(sample)
      safeMin=math.max(safeMin,bl+envelope)
      safeMax=math.min(safeMax,br-envelope)
      maxEnvelope=math.max(maxEnvelope,envelope)
    end
    distance=distance+2.5
  end
  local deficit=math.max(0,safeMin-safeMax)
  if deficit>0 then
    -- No single offset fits the complete preview corridor. The midpoint
    -- minimizes the worst-case boundary violation; speed control handles it.
    local center=(safeMin+safeMax)*0.5
    safeMin,safeMax=center,center
  end
  return safeMin,safeMax,maxEnvelope,deficit
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
  local locked=p.routeLocked and p.routeProgress~=nil
  local projection=Geometry.project(route,car.position,p.routeProgress,car.look,
    speed,dt,locked)
  if not projection or
     (not locked and not initialProjectionValid(projection,car.look)) or
     (p.spawnSeeded and projection.distance>8.0) then
    p.projectionLost=true; p.targetSpeed=0; p.gas=0; p.brake=1; p.lineDist=999
    return 0
  end
  p.projectionLost=false; p.routeLocked=true
  p.routeProgress=projection.s; p.segIdx=projection.seg
  p.lineDist=projection.distance; p.latOff=projection.lateral

  -- Preview the complete vehicle footprint through the steering horizon.
  -- The common safe lateral interval forces the target to remain inside both
  -- saved boundaries before the car reaches a narrowing or a bend.
  local ahead=clamp(CFG.LOOKAHEAD_BASE+speed*CFG.LOOKAHEAD_SPEED,
    CFG.LOOKAHEAD_MIN,CFG.LOOKAHEAD_MAX)
  local current=Geometry.sample(route,projection.s)
  local future=Geometry.sample(route,projection.s+ahead)
  local curvePreview=Geometry.sample(route,projection.s+ahead*0.55)
  if not current or not future or not curvePreview then return 0 end

  local halfWidth=math.max(0.75,safeNumber(car.aabbSize and car.aabbSize.x,2.0)*0.5)
  local bl,br=routeBoundsAtSample(current)
  local safeMin,safeMax=bl+halfWidth+BOUND_MARGIN,br-halfWidth-BOUND_MARGIN
  local boundaryDeficit=math.max(0,safeMin-safeMax)
  if boundaryDeficit>0 then
    local center=(safeMin+safeMax)*0.5
    safeMin,safeMax=center,center
  end
  local bodyEnvelope=halfWidth+BOUND_MARGIN
  local laneLeft,laneRight=H.boundsAt(p.segIdx)
  local roadCenter=(laneLeft+laneRight)*0.5
  -- Ordinary driving and the post-pass return target the racing corridor,
  -- not the legacy left-lane slot. A discrete lane offset is active only
  -- while a pass or an explicit yield reservation is in progress.
  if p.overtakeSide~=0 then
    p.laneOffset=p.passLat
  elseif p.yieldRequestBy~=nil and p.yieldOffset~=nil then
    p.laneOffset=p.yieldOffset
  else
    p.baseLaneOffset=p.baseLaneOffset or roadCenter
    p.laneOffset=p.baseLaneOffset
  end
  local laneTarget=p.laneOffset or roadCenter
  local planAdvance=p.racingPlanStartProgress and routeGap(p.racingPlanStartProgress,projection.s) or 0
  local currentRace=p.racingPlan and planAdvance<=p.racingPlan.horizon and
    RacingLine.sample(p.racingPlan,math.max(0,planAdvance)) or nil
  local targetPlanDistance=ahead+math.max(0,planAdvance)
  local raceTarget=p.racingPlan and targetPlanDistance<=p.racingPlan.horizon and
    RacingLine.sample(p.racingPlan,targetPlanDistance) or nil
  if raceTarget then
    p.raceOffsetRaw=raceTarget.offset
    p.raceOffsetCurrent=currentRace and currentRace.offset or raceTarget.offset
    p.laneOffsetCurrent=p.raceOffsetCurrent
  else
    local maxStep=(p.emergencyPass and 12.0 or 1.8)*math.max(0,dt or 0)
    local previousOffset=p.laneOffsetCurrent or laneTarget
    p.laneOffsetCurrent=previousOffset+
      clamp(laneTarget-previousOffset,-maxStep,maxStep)
  end
  if p.returningFromPass and math.abs(projection.lateral-laneTarget)<0.35 then
    p.returningFromPass=false
  end

  local targetX=raceTarget and raceTarget.x or
    (future.x-future.tz*p.laneOffsetCurrent)
  local targetZ=raceTarget and raceTarget.z or
    (future.z+future.tx*p.laneOffsetCurrent)
  local dx,dz=targetX-car.position.x,targetZ-car.position.z
  local targetDist=math.sqrt(dx*dx+dz*dz)
  p.steeringTargetX,p.steeringTargetZ=targetX,targetZ
  p.steeringTargetDistance=targetDist
  p.steeringTargetSource=raceTarget and "RACING_PLAN" or "ROUTE_SPLINE"
  p.steeringTargetPlanDistance=targetPlanDistance
  p.steeringTargetOffset=raceTarget and raceTarget.offset or p.laneOffsetCurrent
  if targetDist<0.5 then
    p.pursuitSteer=0; p.boundaryCurvatureSteer=0; p.rawSteer=0
    p.steeringAlpha=0
    return 0
  end

  -- Pure-pursuit curvature from the preview point, plus modest curvature
  -- feed-forward to begin rotating into the bend before heading error grows.
  local alpha=signedAngle(fx,fz,dx/targetDist,dz/targetDist)
  local pursuitCurvature=2.0*math.sin(alpha)/targetDist
  p.steeringAlpha=alpha
  -- Pure pursuit already calculates the curvature needed to reach this
  -- preview point. Adding the planner's curvature again double-counts the
  -- same bend and can create an oversteer/oscillation feedback loop.
  local desiredCurvature=pursuitCurvature

  -- Predict lateral drift over the preview time and steer back before the
  -- vehicle footprint reaches a saved boundary.
  local velocity=car.velocity
  local lateralVelocity=safeNumber(velocity.x)*(-current.tz)+
    safeNumber(velocity.z)*current.tx
  local previewTime=clamp(ahead/math.max(5.0,speed),0.35,1.25)
  local predictedLateral=projection.lateral+lateralVelocity*previewTime
  local safePredicted=clamp(predictedLateral,safeMin,safeMax)
  local boundaryError=safePredicted-predictedLateral
  local boundaryDistance=math.max(6.0,ahead*0.50)
  -- Boundary recovery is deliberately a low-authority damping term.
  -- The previous high-gain prediction feedback changed sign every time the
  -- lateral velocity crossed zero, producing full-lock left/right oscillation
  -- on straights. Main path tracking remains responsible for normal steering.
  local boundaryCurvature=clamp(1.2*boundaryError/
    (boundaryDistance*boundaryDistance),-0.06,0.06)
  desiredCurvature=desiredCurvature+boundaryCurvature
  local velocityAlpha=clamp(dt/(0.12+math.max(0,dt)),0,1)
  p.lateralVelocityFiltered=lerp(
    safeNumber(p.lateralVelocityFiltered),lateralVelocity,velocityAlpha)
  -- Lateral-velocity damping is boundary protection, not a permanent
  -- straight-line steering input. Fade it in only as the predicted vehicle
  -- footprint approaches either safe corridor edge.
  local predictedClearance=math.min(
    safePredicted-safeMin,safeMax-safePredicted)
  local boundaryRisk=clamp((1.5-predictedClearance)/1.5,0,1)
  local lateralDamping=p.lateralVelocityFiltered*0.035*boundaryRisk
  local boundarySteer=clamp(
    boundaryError*0.075-lateralDamping,-0.22,0.22)
  p.boundaryRisk=boundaryRisk
  p.boundarySteer=boundarySteer
  p.boundaryCurvature=boundaryCurvature
  p.lookahead=ahead
  p.pursuitCurvature=pursuitCurvature
  p.lateralVelocity=lateralVelocity
  p.previewTime=previewTime
  local k15=Geometry.sample(route,projection.s+15.0)
  local k30=Geometry.sample(route,projection.s+30.0)
  local k50=Geometry.sample(route,projection.s+50.0)
  p.roadCurvature0=safeNumber(current.k)
  p.roadCurvature15=safeNumber(k15 and k15.k)
  p.roadCurvature30=safeNumber(k30 and k30.k)
  p.roadCurvature50=safeNumber(k50 and k50.k)
  p.roadStraight=math.max(math.abs(p.roadCurvature0),
    math.abs(p.roadCurvature15),math.abs(p.roadCurvature30),
    math.abs(p.roadCurvature50))<0.0012

  p.headingError=signedAngle(fx,fz,future.tx,future.tz)
  p.crossTrackError=p.laneOffsetCurrent-projection.lateral
  p.pathCurvature=safeNumber(currentRace and currentRace.k or current.k)
  p.steerCurvature=desiredCurvature
  p.projectionProgressDelta=projection.delta or 0
  p.boundaryError=boundaryError
  p.boundarySafeMin=safeMin
  p.boundarySafeMax=safeMax
  p.boundaryEnvelope=bodyEnvelope
  p.boundaryDeficit=boundaryDeficit
  p.boundaryEmergency=math.abs(boundaryError)>0.45 or boundaryDeficit>0.15

  -- Body slip angle is deliberately not fed directly into steering:
  -- its sign depends on the velocity convention and can reinforce a slide.
  -- Yaw-rate feedback below uses the explicitly aligned physical yaw signal.
  -- Close the yaw-rate feedback loop around the path-following controller.
  -- route curvature and signedAngle use the X/Z convention; CSP's measured
  -- localAngularVelocity.y has the opposite sign, so convert it once here.
  -- When axle telemetry is unavailable, use the heading-derived estimate,
  -- which already uses the route convention.
  local yawActual=p.vehicleDynamicsValid and
    -safeNumber(p.measuredYawRate) or safeNumber(p.yawRate)
  local yawAlpha=clamp(dt/(0.08+math.max(0,dt)),0,1)
  p.yawRateFiltered=lerp(safeNumber(p.yawRateFiltered),yawActual,yawAlpha)
  local yawTarget=desiredCurvature*math.max(0,speed)
  local yawError=yawTarget-p.yawRateFiltered
  local yawSpeedFade=clamp((speed-8.0)/12.0,0,1)
  local yawFeedbackCurvature=0
  if yawSpeedFade>0 then
    yawFeedbackCurvature=clamp(
      yawError*1.6/math.max(12.0,speed)*yawSpeedFade,-0.055,0.055)
  end
  local pursuitSteer=pursuitCurvature*CFG.CURVATURE_STEER_GAIN
  local boundaryCurvatureSteer=boundaryCurvature*CFG.CURVATURE_STEER_GAIN
  local yawStabilitySteer=yawFeedbackCurvature*CFG.CURVATURE_STEER_GAIN
  local rawSteer=pursuitSteer+boundaryCurvatureSteer+
    yawStabilitySteer+boundarySteer
  local deadbandActive=false
  if math.abs(rawSteer)<STEER_DEAD and
     math.abs(p.crossTrackError or 0)<0.03 and
     math.abs(p.headingError or 0)<0.001 and
     math.abs(p.lateralVelocityFiltered or 0)<0.03 then
    rawSteer=0
    deadbandActive=true
  end
  p.pursuitSteer=pursuitSteer
  p.boundaryCurvatureSteer=boundaryCurvatureSteer
  p.yawStabilitySteer=yawStabilitySteer
  p.yawRateTarget=yawTarget
  p.yawRateFeedbackError=yawError
  p.rawSteer=rawSteer
  p.deadbandActive=deadbandActive
  return clamp(rawSteer,-CFG.STEER_LIMIT,CFG.STEER_LIMIT)
end

local function filterSteering(p, target, dt, collisionEmergency)
  local alpha = clamp(dt / (CFG.STEER_FILTER + dt), 0, 1)
  p.steerFiltered = collisionEmergency and target or
    lerp(p.steerFiltered, target, alpha)
  -- A predicted collision needs faster steering authority than an ordinary
  -- lane change. The input remains bounded by STEER_LIMIT and the emergency
  -- slew is capped per frame to avoid an instantaneous full-lock snap.
  local maxChange=collisionEmergency and math.max(CFG.STEER_RATE*dt,
    math.min(0.45,16.0*dt)) or CFG.STEER_RATE*dt
  local requestedChange=p.steerFiltered-p.steer
  local appliedChange=clamp(requestedChange,-maxChange,maxChange)
  p.steerRateLimited=math.abs(requestedChange)>maxChange+1e-6
  p.steerRateLimitDemand=requestedChange
  p.steerRateLimitApplied=appliedChange
  p.steer = clamp(p.steer+appliedChange,-CFG.STEER_LIMIT,CFG.STEER_LIMIT)
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
local function relativeToRoute(position, sample)
  if not position or not sample then return nil, nil end
  local dx=safeNumber(position.x)-sample.x
  local dz=safeNumber(position.z)-sample.z
  local longitudinal=dx*sample.tx+dz*sample.tz
  local lateral=dx*(-sample.tz)+dz*sample.tx
  return longitudinal,lateral
end

local function laneOccupied(carIndex,car,laneOffset,ignoreIndex)
  local sim=ac.getSim()
  if not sim then return true end
  local own=state.p[carIndex]
  local ownProgress=own and own.routeProgress
  if ownProgress==nil then
    ownProgress=routeProjection(car,own and own.segIdx or 1,
      not (own and own.routeLocked),own and own.routeProgress,car.speedMs,0.05)
  end
  if ownProgress==nil then return true end
  local ownSample=Geometry.sample(state.routeGeometry,ownProgress)
  if not ownSample then return true end

  local ownSpeed=math.max(0,safeNumber(car.speedMs))
  local ownHalfWidth=math.max(0.75,safeNumber(car.aabbSize and car.aabbSize.x,2.0)*0.5)
  for i=0,sim.carsCount-1 do
    local ignored=i==ignoreIndex
    if type(ignoreIndex)=="table" then
      for j=1,#ignoreIndex do
        if ignoreIndex[j]==i then ignored=true;break end
      end
    end
    if i~=carIndex and not ignored then
      local other=ac.getCar(i)
      if other and other.isActive then
        local otherState=state.p[i]
        local progress,lateral,otherSeg=routeProjection(other,
          otherState and otherState.segIdx or own.segIdx or 1,
          not (otherState and otherState.routeLocked),
          otherState and otherState.routeProgress,other.speedMs,0.05)
        if otherState then
          if progress~=nil then otherState.routeProgress=progress end
          otherState.routeLocked=progress~=nil
          if otherSeg then otherState.segIdx=otherSeg end
        end

        local longitudinal,actualLateral
        local otherFx,otherFz=normalize2(other.look.x,other.look.z)
        local otherRouteSample=progress and Geometry.sample(state.routeGeometry,progress)
        local directionDot=otherRouteSample and
          (otherFx*otherRouteSample.tx+otherFz*otherRouteSample.tz) or 1.0
        if progress~=nil and directionDot>=-0.25 then
          longitudinal=routeGap(ownProgress,progress)
          actualLateral=lateral
        else
          -- Opposing-direction traffic on a shared route is measured in the
          -- local forward frame, not by same-direction loop progress.
          local dx=safeNumber(other.position.x)-ownSample.x
          local dz=safeNumber(other.position.z)-ownSample.z
          if dx*dx+dz*dz<=120.0*120.0 then
            longitudinal,actualLateral=relativeToRoute(other.position,ownSample)
          end
        end

        if longitudinal~=nil and actualLateral~=nil then
          local otherSpeed=math.max(0,safeNumber(other.speedMs))
          local otherHalfWidth=math.max(0.75,
            safeNumber(other.aabbSize and other.aabbSize.x,2.0)*0.5)
          local separation=ownHalfWidth+otherHalfWidth+0.35
          local actualGap=math.abs(actualLateral-laneOffset)
          local otherTarget=otherState and
            (otherState.overtakeSide~=0 or otherState.returningFromPass or otherState.yieldRequestBy~=nil) and
            safeNumber(otherState.yieldOffset or otherState.laneOffset,actualLateral) or actualLateral
          local reservedGap=math.abs(otherTarget-laneOffset)
          local currentOffset=own and safeNumber(own.laneOffsetCurrent,
            own.laneOffset or laneOffset) or laneOffset
          local sweepMin=math.min(currentOffset,laneOffset)
          local sweepMax=math.max(currentOffset,laneOffset)
          local sweepGap=actualLateral<sweepMin and sweepMin-actualLateral or
            (actualLateral>sweepMax and actualLateral-sweepMax or 0)

          -- Occupancy is based on physical overlap and the complete lateral
          -- transition, not a blanket reservation around parallel traffic.
          local closingSpeed=directionDot < -0.25 and
            (ownSpeed+otherSpeed) or math.max(0,ownSpeed-otherSpeed)
          local rearClosingSpeed=directionDot < -0.25 and 0 or
            math.max(0,otherSpeed-ownSpeed)
          local frontWindow=math.max(14.0,closingSpeed*1.0+6.0)
          local rearWindow=math.max(10.0,rearClosingSpeed*1.0+5.0)
          local longitudinalOverlap=longitudinal>-rearWindow and
            longitudinal<frontWindow
          local bodyConflict=actualGap<separation and longitudinalOverlap

          -- An active pass reserves only its actual swept corridor and only
          -- near enough to create a time/space conflict.
          local reservationConflict=otherState and
            (otherState.overtakeSide~=0 or otherState.returningFromPass or otherState.yieldRequestBy~=nil) and
            reservedGap<separation+0.25 and
            longitudinal>-math.max(12.0,otherSpeed*0.8+5.0) and
            longitudinal<math.max(22.0,ownSpeed*1.2+7.0)

          -- Do not let the steering transition cut through another vehicle.
          local crossingWindow=math.max(6.0,(ownSpeed+otherSpeed)*0.18+3.0)
          local sideConflict=math.abs(longitudinal)<crossingWindow and
            (sweepGap<separation+0.25 or reservedGap<separation+0.45)

          if bodyConflict or reservationConflict or sideConflict then
            return true
          end
        end
      end
    end
  end
  return false
end

-- Detect an imminent physical overlap independently from lane following.
-- A car that is already alongside/behind must never become a longitudinal
-- leader; it is handled here as a short emergency steering manoeuvre instead.
local function collisionEvasion(carIndex,car,p,speed)
  local sim=ac.getSim()
  if not sim or not state.routeGeometry or p.routeProgress==nil then return nil end
  local fx,fz=normalize2(car.look.x,car.look.z)
  local rx,rz=-fz,fx
  local ownHalfLength=math.max(1.5,safeNumber(car.aabbSize and car.aabbSize.z,4.5)*0.5)
  local ownHalfWidth=math.max(0.75,safeNumber(car.aabbSize and car.aabbSize.x,2.0)*0.5)
  local ownVelocity=car.velocity or {x=fx*speed,z=fz*speed}
  local ovx,ovz=safeNumber(ownVelocity.x),safeNumber(ownVelocity.z)
  local nearestTime=nil
  local nearestIndex=nil
  local nearestLateral=0
  local nearestLongitudinal=0
  local nearestHalfWidth=0
  for i=0,sim.carsCount-1 do
    if i~=carIndex then
      local other=ac.getCar(i)
      if other and other.isActive then
        local dx=safeNumber(other.position.x)-safeNumber(car.position.x)
        local dz=safeNumber(other.position.z)-safeNumber(car.position.z)
        local otherVelocity=other.velocity or {x=0,z=0}
        local rvx=safeNumber(otherVelocity.x)-ovx
        local rvz=safeNumber(otherVelocity.z)-ovz
        local longitudinal=dx*fx+dz*fz
        local lateral=dx*rx+dz*rz
        local relativeSpeedSq=rvx*rvx+rvz*rvz
        local closestTime=relativeSpeedSq>0.25 and
          clamp(-(dx*rvx+dz*rvz)/relativeSpeedSq,0,0.90) or 0
        local predictedLongitudinal=(dx+rvx*closestTime)*fx+
          (dz+rvz*closestTime)*fz
        local predictedLateral=(dx+rvx*closestTime)*rx+
          (dz+rvz*closestTime)*rz
        local otherHalfLength=math.max(1.5,
          safeNumber(other.aabbSize and other.aabbSize.z,4.5)*0.5)
        local otherHalfWidth=math.max(0.75,
          safeNumber(other.aabbSize and other.aabbSize.x,2.0)*0.5)
        local ofx,ofz=normalize2(other.look.x,other.look.z)
        local orx,orz=-ofz,ofx
        -- Project the other vehicle's oriented rectangle into our local frame.
        local otherLongExtent=otherHalfLength*math.abs(ofx*fx+ofz*fz)+
          otherHalfWidth*math.abs(orx*fx+orz*fz)
        local otherLatExtent=otherHalfLength*math.abs(ofx*rx+ofz*rz)+
          otherHalfWidth*math.abs(orx*rx+orz*rz)
        local longitudinalLimit=ownHalfLength+otherLongExtent+0.65
        local lateralLimit=ownHalfWidth+otherLatExtent+0.45
        local currentOverlap=math.abs(longitudinal)<longitudinalLimit and
          math.abs(lateral)<lateralLimit
        local predictedOverlap=math.abs(predictedLongitudinal)<longitudinalLimit and
          math.abs(predictedLateral)<lateralLimit
        local closing=-(dx*rvx+dz*rvz)
        if predictedOverlap and (currentOverlap or closing>0.5) and
           (nearestTime==nil or closestTime<nearestTime) then
          nearestTime=closestTime
          nearestIndex=i
          nearestLateral=predictedLateral
          nearestLongitudinal=predictedLongitudinal
          nearestHalfWidth=otherLatExtent
        end
      end
    end
  end
  if nearestIndex==nil then return nil end

  local sample=Geometry.sample(state.routeGeometry,p.routeProgress)
  if not sample then return nil end
  local leftBound,rightBound=routeBoundsAtSample(sample)
  local currentLateral=safeNumber(p.latOff)
  local leftRoom=rightBound-currentLateral-ownHalfWidth-0.35
  local rightRoom=currentLateral-leftBound-ownHalfWidth-0.35
  local requiredShift=ownHalfWidth+nearestHalfWidth+0.55
  local canLeft=leftRoom>=requiredShift and
    not laneOccupied(carIndex,car,currentLateral+requiredShift,nearestIndex)
  local canRight=rightRoom>=requiredShift and
    not laneOccupied(carIndex,car,currentLateral-requiredShift,nearestIndex)
  if not canLeft and not canRight then
    p.collisionEvasionSide=0
    p.collisionEvasionTarget=nearestIndex
    if not p.collisionEvasionBlockedLogged or
       p.collisionEvasionLoggedTarget~=nearestIndex then
      H.log("COLLISION_EVASION_BLOCKED id=%d other=%d ttc=%.3f long=%.2f lat=%.2f",
        carIndex,nearestIndex,nearestTime,nearestLongitudinal,nearestLateral)
      p.collisionEvasionBlockedLogged=true
      p.collisionEvasionLoggedTarget=nearestIndex
    end
    return nil,true
  end
  p.collisionEvasionBlockedLogged=false

  local side
  -- Keep the selected escape direction for the duration of this threat;
  -- changing sides while the vehicles are still converging would create
  -- exactly the steering oscillation the emergency manoeuvre is meant to avoid.
  if p.collisionEvasionSide==1 and canLeft then side=1
  elseif p.collisionEvasionSide==-1 and canRight then side=-1
  elseif canLeft and canRight then
    if nearestLateral>0.15 then side=-1
    elseif nearestLateral< -0.15 then side=1
    else side=leftRoom>=rightRoom and 1 or -1 end
  else
    side=canLeft and 1 or -1
  end
  local severity=clamp(1.0-nearestTime/0.90,0,1)
  local magnitude=clamp(0.62+severity*0.24,0.62,0.86)
  p.collisionEvasionSide=side
  p.collisionEvasionTarget=nearestIndex
  p.collisionEvasionTTC=nearestTime
  p.collisionEvasionOffset=currentLateral+side*requiredShift
  if not p.collisionEvasionLogged or
     p.collisionEvasionLoggedTarget~=nearestIndex or
     p.collisionEvasionLoggedSide~=side then
    H.log("COLLISION_EVASION id=%d other=%d side=%s ttc=%.3f long=%.2f lat=%.2f target=%.2f",
      carIndex,nearestIndex,side>0 and "LEFT" or "RIGHT",nearestTime,
      nearestLongitudinal,nearestLateral,p.collisionEvasionOffset)
    p.collisionEvasionLogged=true
    p.collisionEvasionLoggedTarget=nearestIndex
    p.collisionEvasionLoggedSide=side
  end
  return side*magnitude
end

local function carAheadInLane(carIndex,car,laneOffset,ignoreIndex)
  local sim=ac.getSim()
  local own=state.p[carIndex]
  if not sim or not own then return nil,nil,nil,nil end
  local ownProgress=own.routeProgress
  if ownProgress==nil then
    ownProgress=routeProjection(car,own.segIdx,not own.routeLocked,
      own.routeProgress,car.speedMs,0.05)
    own.routeProgress=ownProgress
  end
  if ownProgress==nil then return nil,nil,nil,nil end
  local ownSample=Geometry.sample(state.routeGeometry,ownProgress)
  if not ownSample then return nil,nil,nil,nil end
  local bestGap,bestSpeed,bestIndex,bestLateral=nil,nil,nil,nil
  local ownSpeed=math.max(0,safeNumber(car.speedMs))
  for i=0,sim.carsCount-1 do
    if i~=carIndex and i~=ignoreIndex then
      local other=ac.getCar(i)
      if other and other.isActive then
        local otherState=state.p[i]
        local progress,lateral,otherSeg=routeProjection(other,
          otherState and otherState.segIdx or own.segIdx or 1,
          not (otherState and otherState.routeLocked),
          otherState and otherState.routeProgress,other.speedMs,0.05)
        if otherState then
          if progress~=nil then otherState.routeProgress=progress end
          otherState.routeLocked=progress~=nil
          if otherSeg then otherState.segIdx=otherSeg end
        end
        local gap,actualLateral
        local otherFx,otherFz=normalize2(other.look.x,other.look.z)
        local otherRouteSample=progress and Geometry.sample(state.routeGeometry,progress)
        local directionDot=otherRouteSample and
          (otherFx*otherRouteSample.tx+otherFz*otherRouteSample.tz) or 1.0
        if progress~=nil and directionDot>=-0.25 then
          gap=routeGap(ownProgress,progress);actualLateral=lateral
        else
          local dx=safeNumber(other.position.x)-ownSample.x
          local dz=safeNumber(other.position.z)-ownSample.z
          if dx*dx+dz*dz<=120.0*120.0 then
            gap,actualLateral=relativeToRoute(other.position,ownSample)
          end
        end
        local halfLengths=(safeNumber(car.aabbSize and car.aabbSize.z,4.5)+
          safeNumber(other.aabbSize and other.aabbSize.z,4.5))*0.5
        local lateralSeparation=(math.max(1.5,safeNumber(car.aabbSize and car.aabbSize.x,2.0))+
          math.max(1.5,safeNumber(other.aabbSize and other.aabbSize.x,2.0)))*0.5+0.4
        local otherSpeed=math.max(0,safeNumber(other.speedMs))
        local detectionHorizon=directionDot < -0.25 and
          (ownSpeed+otherSpeed)*2.0+30.0 or ownSpeed*3.0+30.0
        -- Same-direction traffic is a leader only when it is genuinely
        -- ahead. The former negative-gap allowance made two overlapping cars
        -- select each other as leaders, producing reciprocal zero-speed locks.
        -- Oncoming traffic uses local forward projection and must also be ahead.
        if gap and actualLateral and gap>0.5 and
           gap<math.max(110.0,detectionHorizon) and
           math.abs(actualLateral-laneOffset)<lateralSeparation and
           (not bestGap or gap<bestGap) then
          bestGap=gap
          local measuredOtherSpeed=math.max(0,safeNumber(other.speedMs))
          bestSpeed=directionDot < -0.25 and -measuredOtherSpeed or measuredOtherSpeed
          bestIndex=i;bestLateral=actualLateral
        end
      end
    end
  end
  return bestGap,bestSpeed,bestIndex,bestLateral
end

local function updateRacingPlan(carIndex,car,p,speed,dt)
  refreshRouteMetrics()
  if not state.routeGeometry or p.routeProgress==nil then
    p.racingPlan=nil
    p.racingPlanBlocked=false
    p.racingPlanTrafficBlocked=false
    p.racingPlanTrafficBlockedDistance=nil
    p.racingPlanBlockedDistance=nil
    return
  end
  local bl,br=H.boundsAt(p.segIdx)
  local mid=(bl+br)*0.5
  -- The saved return target is the neutral racing corridor. Lane centres
  -- are selected only by the explicit passing/yield state machine.
  local base=p.baseLaneOffset
  if base==nil then
    base=mid
    p.baseLaneOffset=base
  end

  p.racingPlanTimer=math.max(0,safeNumber(p.racingPlanTimer)-dt)
  local forcedMode=nil
  if p.overtakeSide~=0 then forcedMode="PASS"
  elseif p.returningFromPass then forcedMode="RETURN"
  elseif p.yieldRequestBy~=nil and p.yieldOffset~=nil then forcedMode="YIELD" end
  local mode=forcedMode or "OPEN"
  local fixedOffset=nil
  local fixedTransitionDistance=22.0
  -- Normal racing always uses the complete road corridor, even when other
  -- cars are nearby. Traffic spacing and conflict decisions belong to
  -- updateTrafficControl(); converting every nearby car into a fixed-offset
  -- TRAFFIC_HOLD path was locking the optimizer to stale lane offsets and
  -- producing NO_PATH at the start grid. Only an explicitly reserved
  -- PASS/RETURN/YIELD manoeuvre may constrain the racing-line corridor.
  if p.racingPlanTimer>0 and p.racingPlanMode==mode then
    return
  end
  if mode=="PASS" then
    fixedOffset=safeNumber(p.passLat,p.laneOffset or base)
  elseif mode=="RETURN" then
    fixedOffset=safeNumber(p.baseLaneOffset,base)
  elseif mode=="YIELD" then
    fixedOffset=safeNumber(p.yieldOffset,base)
  end
  if fixedOffset~=nil then
    local lateralDelta=math.abs(fixedOffset-safeNumber(p.latOff,base))
    local lateralLimit=clamp(
      safeNumber(p.lateralAccelLimit,CFG.MAX_LATERAL_ACCEL)*0.65,4.0,12.0)
    fixedTransitionDistance=math.max(22.0,
      math.max(0,speed)*math.sqrt(6.0*lateralDelta/lateralLimit))
  end
  -- In OPEN running the optimizer uses the complete road width.
  -- Only explicit PASS/RETURN/YIELD reservations constrain the path.

  local brakeAccel=clamp(safeNumber(p.brakeAccelEstimate,CFG.BRAKE_ACCEL),2.5,22.0)
  local v=math.max(0,speed)
  local horizon=clamp(math.max(CFG.PLAN_HORIZON,v*v/(2*brakeAccel)+v*1.5+35.0),CFG.PLAN_HORIZON,135.0)
  local planFailure=nil
  -- Traffic is deliberately not fed into the geometric racing-line optimizer.
  -- Following, passing and collision reservations have one owner below:
  -- updateTrafficControl(). A blocked obstacle grid must never turn a valid
  -- following situation into a planner NO_PATH hard stop.
  local previousPlan=p.racingPlanMode==mode and p.racingPlan or nil
  local previousAdvance=previousPlan and p.racingPlanStartProgress and
    math.max(0,routeGap(p.racingPlanStartProgress,p.routeProgress)) or 0
  local nextPlan
  -- If a reserved PASS/RETURN/YIELD path cannot be optimized, first hold the
  -- car's measured current corridor. This is still a full-body swept-path
  -- validation; it does not jump to another lane or relax road boundaries.
  -- A planner search failure therefore cannot strand a car when its present
  -- corridor remains physically drivable.
  -- In OPEN mode the preferred offset is the physical road midpoint, not
  -- the legacy left-lane slot. Discrete lane slots are traffic reservations
  -- for passing/yielding only; they must not bias ordinary racing-line speed.
  local preferredOffset=mode=="OPEN" and mid or base
  nextPlan,planFailure=RacingLine.plan(
    state.routeGeometry,Geometry.sample,p.routeProgress,car,routeBoundsAtSample,
    safeNumber(p.latOff,base),preferredOffset,fixedOffset,horizon,7.5,
    previousPlan,previousAdvance,fixedTransitionDistance,p.lateralAccelLimit)

  -- A failed dynamic-programming search is not proof that the road is
  -- physically blocked. In ordinary OPEN running, retry on the road centre
  -- (the GDD's neutral fallback) using the same oriented full-body swept-path
  -- validation. Do not retry on a stale lane offset: lane slots exist only
  -- during explicit passing/yield manoeuvres. Reserved PASS/RETURN/YIELD
  -- paths are never moved to another corridor by this fallback.
  local originalFailure=planFailure or (not nextPlan and "NO_PLAN" or nil)
  local fallbackKind=nil
  if not nextPlan and mode=="OPEN" then
    local fallbackFailure=nil
    nextPlan,fallbackFailure=RacingLine.plan(
      state.routeGeometry,Geometry.sample,p.routeProgress,car,routeBoundsAtSample,
      safeNumber(p.latOff,base),mid,mid,horizon,7.5,
      nil,0,fixedTransitionDistance,p.lateralAccelLimit)
    if nextPlan then
      fallbackKind="ROAD_CENTRE"
    else
      planFailure=fallbackFailure or originalFailure or "NO_SAFE_FALLBACK"
    end
  end

  if not nextPlan and mode~="OPEN" then
    local holdOffset=safeNumber(p.latOff,base)
    local holdFailure=nil
    nextPlan,holdFailure=RacingLine.plan(
      state.routeGeometry,Geometry.sample,p.routeProgress,car,routeBoundsAtSample,
      holdOffset,holdOffset,holdOffset,horizon,7.5,
      nil,0,fixedTransitionDistance,p.lateralAccelLimit)
    if nextPlan then
      fallbackKind="HOLD_CURRENT_CORRIDOR"
      planFailure=nil
    else
      planFailure=holdFailure or planFailure or "NO_SAFE_CURRENT_CORRIDOR"
    end
  end

  if nextPlan ~= p.racingPlan then
    p.racingPlanGeneration = (p.racingPlanGeneration or 0) + 1
  end
  p.racingPlan=nextPlan
  p.racingPlanStartProgress=p.routeProgress
  -- A hard stop is reserved for the case where the optimizer and its
  -- geometry-validated OPEN road-centre fallback both fail, or an explicitly
  -- reserved PASS/RETURN/YIELD corridor cannot contain the complete vehicle body.
  p.racingPlanBlocked=nextPlan==nil
  p.racingPlanFailure=p.racingPlanBlocked and
    (planFailure or originalFailure or "NO_SAFE_PATH") or nil
  p.racingPlanFallback=fallbackKind
  p.racingPlanTrafficBlocked=false
  p.racingPlanTrafficBlockedDistance=nil
  p.racingPlanBlockedDistance=p.racingPlan and
    p.racingPlan.blockedDistance or nil
  if fallbackKind and (p.racingPlanLastFallback~=fallbackKind or
      p.racingPlanLastFallbackReason~=originalFailure) then
    H.log("RACE_LINE_FALLBACK id=%d mode=%s type=%s cause=%s",
      carIndex,mode,fallbackKind,tostring(originalFailure))
  end
  p.racingPlanLastFallback=fallbackKind
  p.racingPlanLastFallbackReason=originalFailure
  if p.racingPlanBlocked then
    H.log("RACE_LINE_BLOCKED id=%d reason=HARD_STOP_NO_SAFE_PATH cause=%s",
      carIndex,tostring(p.racingPlanFailure))
  end
  if p.racingPlanMode~=mode then
    H.log("RACE_LINE id=%d mode=%s base=%.2f target=%.2f curve=%.4f horizon=%.1f",
      carIndex,mode,base,safeNumber(p.racingPlan and p.racingPlan.points[2] and
      p.racingPlan.points[2].offset,base),
      safeNumber(p.racingPlan and p.racingPlan.curvature[2]),horizon)
  end
  p.racingPlanMode=mode
  -- Keep the plan's origin close enough to the car that the preview point
  -- remains inside the sampled horizon even at extreme speed.
  p.racingPlanTimer=math.min(0.65,20.0/math.max(30.0,speed))
  if p.racingPlan then
    p.racingPlanCurvature=safeNumber(p.racingPlan.curvature[2])
    local feature=p.racingPlan.feature and p.racingPlan.feature[2] or "UNKNOWN"
    if feature~=p.racingPlanFeature then
      p.racingPlanFeature=feature
      H.log("RACE_FEATURE id=%d type=%s curvature=%.5f horizon=%.1f",
        carIndex,tostring(feature),p.racingPlanCurvature,
        safeNumber(p.racingPlan.horizon))
    end
  end
end

local function passingReservationBlocked(carIndex,p,targetOffset,speed)
  if p.routeProgress==nil then return true end
  local range=math.max(16.0,math.max(0,speed)*1.0+8.0)
  local ownCar=ac.getCar(carIndex)
  if not ownCar then return true end
  local ownWidth=math.max(1.5,safeNumber(ownCar.aabbSize and ownCar.aabbSize.x,2.0))
  for otherIndex,other in pairs(state.p) do
    if otherIndex~=carIndex and
       (other.overtakeSide~=0 or other.returningFromPass or other.yieldRequestBy~=nil) and
       other.routeProgress~=nil and (other.laneOffset~=nil or other.yieldOffset~=nil) then
      local gap=math.abs(routeGap(p.routeProgress,other.routeProgress))
      local otherCar=ac.getCar(otherIndex)
      local otherWidth=math.max(1.5,safeNumber(otherCar and otherCar.aabbSize and otherCar.aabbSize.x,2.0))
      local lateralSeparation=(ownWidth+otherWidth)*0.5+0.5
      if gap<range and math.abs(safeNumber(other.yieldOffset,other.laneOffset)-targetOffset)<lateralSeparation then
        return true,otherIndex,gap
      end
    end
  end
  return false
end

local function safeFollowingSpeed(gap,leaderSpeed,brakeAccel)
  local available=math.max(0,gap-math.max(0,safeNumber(state.followGap,8)))
  local decel=clamp(safeNumber(brakeAccel,CFG.BRAKE_ACCEL),2.5,22.0)
  local reactionTerm=decel*0.45
  local leader=safeNumber(leaderSpeed)
  if leader<0 then
    -- Include reaction distance as well as braking distance for an oncoming
    -- vehicle; the result is the maximum speed that can still stop in gap.
    local opposingSpeed=-leader
    local radicand=2*decel*available+
      2*opposingSpeed*reactionTerm+reactionTerm*reactionTerm
    return math.max(0,math.sqrt(math.max(0,radicand))-
      opposingSpeed-reactionTerm)
  end
  -- Solve v*t_reaction + (v^2-v_leader^2)/(2*a) <= available.
  return math.max(0,math.sqrt(math.max(0,leader*leader+
    2*decel*available+reactionTerm*reactionTerm))-reactionTerm)
end

-- Predict the complete lateral pass, the side-by-side phase and the
-- return to the original line. A candidate is accepted only when the whole
-- swept vehicle footprint remains inside the road and clear of every car.
local function passingPathClear(carIndex,car,p,leaderIndex,source,targetOffset,
    passDistance,speed)
  local route=state.routeGeometry
  if not route or p.routeProgress==nil then return false,"no_route" end
  local ownHalfWidth=math.max(0.75,safeNumber(car.aabbSize and car.aabbSize.x,2.0)*0.5)
  local ownHalfLength=math.max(1.5,safeNumber(car.aabbSize and car.aabbSize.z,4.5)*0.5)
  local fx,fz=normalize2(car.look.x,car.look.z)
  local traffic={}
  local sim=ac.getSim()
  if not sim then return false,"no_sim" end

  for i=0,sim.carsCount-1 do
    if i~=carIndex then
      local other=ac.getCar(i)
      if other and other.isActive then
        local otherState=state.p[i]
        local op,ol=routeProjection(other,
          otherState and otherState.segIdx or p.segIdx,
          not (otherState and otherState.routeLocked),
          otherState and otherState.routeProgress,other.speedMs,0.05)
        if op==nil then
          local dx=other.position.x-car.position.x
          local dz=other.position.z-car.position.z
          if dx*dx+dz*dz<80.0*80.0 then
            return false,"near_vehicle_unprojected"
          end
        else
          local os=Geometry.sample(route,op)
          local ofx,ofz=normalize2(other.look.x,other.look.z)
          local direction=os and (ofx*os.tx+ofz*os.tz) or 1.0
          local signedSpeed=math.max(0,safeNumber(other.speedMs))
          if direction < -0.25 then signedSpeed=-signedSpeed end
          local otherHalfWidth=math.max(0.75,
            safeNumber(other.aabbSize and other.aabbSize.x,2.0)*0.5)
          local otherHalfLength=math.max(1.5,
            safeNumber(other.aabbSize and other.aabbSize.z,4.5)*0.5)
          local otherOffset=ol or 0
          if otherState and
             (otherState.overtakeSide~=0 or otherState.returningFromPass or otherState.yieldRequestBy~=nil) then
            otherOffset=safeNumber(otherState.yieldOffset or otherState.laneOffset,otherOffset)
          end
          traffic[#traffic+1]={
            index=i,progress=op,lateral=otherOffset,speed=signedSpeed,
            halfWidth=otherHalfWidth,halfLength=otherHalfLength
          }
        end
      end
    end
  end

  local lateralDelta=math.abs(targetOffset-source)
  local lateralLimit=clamp(
    safeNumber(p.lateralAccelLimit,CFG.MAX_LATERAL_ACCEL)*0.65,4.0,12.0)
  -- A lane change is a spatial manoeuvre. Its minimum distance grows with
  -- speed so the lateral acceleration remains bounded.
  local transitionOut=math.max(22.0,
    math.max(0,speed)*math.sqrt(6.0*lateralDelta/lateralLimit))
  if safeNumber(passDistance)<transitionOut then
    return false,"insufficient_transition_distance"
  end
  local transitionBack=transitionOut
  local totalDistance=math.max(20.0,passDistance)+transitionBack
  local step=3.0
  local distance=0.0
  while distance<=totalDistance+0.001 do
    local sample=Geometry.sample(route,p.routeProgress+distance)
    if not sample then return false,"route_ends" end

    local outT=clamp(distance/transitionOut,0,1)
    outT=outT*outT*(3.0-2.0*outT)
    local backT=clamp((distance-passDistance)/transitionBack,0,1)
    backT=backT*backT*(3.0-2.0*backT)
    local offset
    if distance<=passDistance then
      offset=source+(targetOffset-source)*outT
    else
      offset=targetOffset+(source-targetOffset)*backT
    end

    local angle=signedAngle(fx,fz,sample.tx,sample.tz)*
      (1.0-clamp(distance/12.0,0,1))
    local envelope=ownHalfWidth*math.abs(math.cos(angle))+
      ownHalfLength*math.abs(math.sin(angle))+BOUND_MARGIN+0.12
    local left,right=routeBoundsAtSample(sample)
    if offset<left+envelope or offset>right-envelope then
      return false,"road_boundary"
    end

    local predictionSpeed=math.max(5.0,speed)
    local predictionAccel=clamp(safeNumber(p.accelEstimate,4.0),0.5,18.0)
    local time=(math.sqrt(predictionSpeed*predictionSpeed+
      2.0*predictionAccel*distance)-predictionSpeed)/predictionAccel
    local ownProgress=p.routeProgress+distance
    for _,other in ipairs(traffic) do
      local predictedOther=other.progress+other.speed*time
      local longitudinal=routeGap(ownProgress,predictedOther)
      local lateralGap=math.abs(offset-other.lateral)
      local longitudinalClearance=ownHalfLength+other.halfLength+
        math.max(1.0,math.abs(speed-other.speed)*0.15)
      local lateralClearance=ownHalfWidth+other.halfWidth+0.45
      if math.abs(longitudinal)<longitudinalClearance and
         lateralGap<lateralClearance then
        return false,"predicted_vehicle_conflict"
      end
    end
    distance=distance+step
  end
  return true
end

local function findPassingOffset(carIndex,car,p,leaderIndex,leaderLateral,
    speed,passDistance)
  if p.routeProgress==nil then
    p.lastPassDeniedReason="no_route_projection"
    return nil
  end
  if leaderLateral==nil then
    p.lastPassDeniedReason="leader_lateral_unknown"
    return nil
  end
  local fx,fz=normalize2(car.look.x,car.look.z)
  local safeMin,safeMax=safeCorridorForPreview(state.routeGeometry,p.routeProgress,
    math.max(18.0,math.max(0,speed)*0.8+12.0),car,fx,fz)
  if not safeMin or not safeMax or safeMax-safeMin<1.15 then
    p.lastPassDeniedReason="insufficient_safe_road_width"
    return nil
  end
  local defaultCenter=laneCenters(p.segIdx)
  local bl,br=H.boundsAt(p.segIdx)
  local source=safeNumber(p.latOff,
    p.laneOffsetCurrent or p.laneOffset or p.baseLaneOffset or
    defaultCenter or ((bl+br)*0.5))
  local target=ac.getCar(leaderIndex)
  local ownWidth=math.max(1.5,safeNumber(car.aabbSize and car.aabbSize.x,2.0))
  local targetWidth=math.max(1.5,safeNumber(target and target.aabbSize and target.aabbSize.x,2.0))
  local separation=(ownWidth+targetWidth)*0.5+0.55
  local candidates={}
  -- Passing uses discrete logical lane slots from the GDD (3/7/11/15 m
  -- road widths produce 1/2/3/4 slots), never arbitrary 0.30 m offsets.
  local corridorWidth=br-bl
  local laneCount=math.max(1,math.floor((corridorWidth+1.0)/4.0))
  local slotWidth=corridorWidth/laneCount
  if laneCount<2 or slotWidth<ownWidth+0.55 then
    p.lastPassDeniedReason="no_adjacent_lane"
    return nil
  end
  local sourceSlot=1
  for i=2,laneCount do
    local a=math.abs((bl+(i-0.5)*slotWidth)-source)
    local b=math.abs((bl+(sourceSlot-0.5)*slotWidth)-source)
    if a<b then sourceSlot=i end
  end
  for i=1,laneCount do
    local offset=bl+(i-0.5)*slotWidth
    if i~=sourceSlot and math.abs(offset-leaderLateral)>=separation then
      candidates[#candidates+1]=offset
    end
  end
  table.sort(candidates,function(a,b)return math.abs(a-source)<math.abs(b-source) end)
  local lastReason="no_candidate"
  p.lastPassReservationOwner=nil
  for _,offset in ipairs(candidates) do
    local reserved,reservationOwner=passingReservationBlocked(carIndex,p,offset,speed)
    if not reserved and not laneOccupied(carIndex,car,offset,leaderIndex) then
      local clear,reason=passingPathClear(carIndex,car,p,leaderIndex,
        source,offset,passDistance or 60.0,speed)
      if clear then return offset end
      lastReason=reason or lastReason
    else
      if reserved then p.lastPassReservationOwner=reservationOwner end
      lastReason=reserved and "reserved_corridor" or "occupied_corridor"
    end
  end
  p.lastPassDeniedReason=lastReason
  return nil
end

local function chooseYieldOffset(leaderIndex,leaderCar,leaderState,passOffset,requesterIndex,ignoreGroup)
  if not leaderCar or not leaderState or leaderState.routeProgress==nil then return nil end
  local seg=leaderState.segIdx or 1
  local bl,br=H.boundsAt(seg)
  local width=br-bl
  local laneCount=math.max(1,math.floor((width+1.0)/4.0))
  local slotWidth=width/laneCount
  local lateral=safeNumber(leaderState.latOff,leaderState.laneOffsetCurrent or 0)
  -- The passing slot is authoritative. When a racing-line car sits on the
  -- boundary between logical slots, nearest-centre rounding is ambiguous;
  -- choose the closest different slot as the yielding slot, never the
  -- already-reserved passing slot.
  local passSlot=clamp(math.floor((passOffset-bl)/math.max(0.1,slotWidth))+1,1,laneCount)
  local slot=nil
  for i=1,laneCount do
    if i~=passSlot and (slot==nil or
       math.abs(bl+(i-0.5)*slotWidth-lateral)<
       math.abs(bl+(slot-0.5)*slotWidth-lateral)) then slot=i end
  end
  if slot==nil then return nil end
  local slotLow=bl+(slot-1)*slotWidth
  local slotHigh=slotLow+slotWidth
  local halfWidth=math.max(0.75,safeNumber(leaderCar.aabbSize and leaderCar.aabbSize.x,2.0)*0.5)
  local safeLow=math.max(bl+halfWidth+BOUND_MARGIN,slotLow+halfWidth+0.12)
  local safeHigh=math.min(br-halfWidth-BOUND_MARGIN,slotHigh-halfWidth-0.12)
  if safeHigh-safeLow<0.24 then return nil end
  local direction=passOffset>lateral and -1 or 1
  local desired=clamp(lateral+direction*0.42,safeLow,safeHigh)
  if math.abs(desired-lateral)<0.22 then return nil end
  local fx,fz=normalize2(leaderCar.look.x,leaderCar.look.z)
  local corridorLow,corridorHigh=safeCorridorForPreview(state.routeGeometry,leaderState.routeProgress,18.0,leaderCar,fx,fz)
  if not corridorLow or not corridorHigh or desired<corridorLow or desired>corridorHigh then return nil end
  if laneOccupied(leaderIndex,leaderCar,desired,ignoreGroup or requesterIndex) then return nil end
  return desired
end
local function listContains(list,value)
  if type(list)~="table" then return false end
  for i=1,#list do if list[i]==value then return true end end
  return false
end

local function pendingGroupReady(p,ownerIndex)
  local group=p.pendingYieldGroup
  if type(group)~="table" or #group==0 then return false end
  for i=1,#group do
    local member=state.p[group[i]]
    local car=ac.getCar(group[i])
    if not member or not car or not car.isActive or
       member.yieldRequestBy~=ownerIndex or not member.yieldReady then
      return false
    end
  end
  return true
end

local function releaseYieldGroup(ownerIndex,group)
  if type(group)~="table" then return end
  for i=1,#group do
    local member=state.p[group[i]]
    if member and member.yieldRequestBy==ownerIndex then
      member.yieldRequestBy=nil;member.yieldOffset=nil
      member.yieldReady=false;member.yieldRequestTimer=0
    end
  end
end

-- Collect a compact single-file train in the same logical lane. A pack pass
-- is allowed only when every member can yield into the same adjacent slot.
local function collectYieldGroup(carIndex,car,p,leaderIndex,maxRange)
  local sim=ac.getSim()
  local leaderState=state.p[leaderIndex]
  local leaderCar=ac.getCar(leaderIndex)
  if not sim or not leaderState or not leaderCar then return {leaderIndex},0 end
  local ownProgress=p.routeProgress
  local leaderProgress=leaderState.routeProgress
  if ownProgress==nil or leaderProgress==nil then return {leaderIndex},0 end
  local leaderGap=routeGap(ownProgress,leaderProgress)
  if leaderGap<=0 then return {leaderIndex},0 end
  local leaderSeg=leaderState.segIdx or p.segIdx or 1
  local left,right=H.boundsAt(leaderSeg)
  local laneCount=math.max(1,math.floor(((right-left)+1.0)/4.0))
  local slotWidth=(right-left)/laneCount
  local leaderLat=safeNumber(leaderState.latOff,0)
  local leaderSlot=1
  for i=2,laneCount do
    if math.abs(left+(i-0.5)*slotWidth-leaderLat)<
       math.abs(left+(leaderSlot-0.5)*slotWidth-leaderLat) then leaderSlot=i end
  end
  local candidates={{index=leaderIndex,gap=leaderGap}}
  for i=0,sim.carsCount-1 do
    if i~=carIndex and i~=leaderIndex then
      local other=ac.getCar(i)
      local os=state.p[i]
      if other and other.isActive and other.isAIControlled and os and
         os.overtakeSide==0 and not os.returningFromPass and
         os.yieldRequestBy==nil and os.pendingYieldTarget==nil and
         #candidates<8 then
        local progress,lateral,seg=routeProjection(other,os.segIdx or p.segIdx,
          not os.routeLocked,os.routeProgress,other.speedMs,0.05)
        if progress~=nil and lateral~=nil then
          local sample=Geometry.sample(state.routeGeometry,progress)
          local ofx,ofz=normalize2(other.look.x,other.look.z)
          local sameDirection=sample and
            (ofx*sample.tx+ofz*sample.tz)>=-0.25
          local gap=routeGap(ownProgress,progress)
          if sameDirection and gap>leaderGap and gap<=maxRange then
            local ol,orr=H.boundsAt(seg or os.segIdx or leaderSeg)
            local lanes=math.max(1,math.floor(((orr-ol)+1.0)/4.0))
            local width=(orr-ol)/lanes
            local slot=1
            for si=2,lanes do
              if math.abs(ol+(si-0.5)*width-lateral)<
                 math.abs(ol+(slot-0.5)*width-lateral) then slot=si end
            end
            if slot==leaderSlot and math.abs(lateral-leaderLat)<math.min(slotWidth,width)*0.38 then
              candidates[#candidates+1]={index=i,gap=gap}
            end
          end
        end
      end
    end
  end
  table.sort(candidates,function(a,b)return a.gap<b.gap end)
  local group={leaderIndex}
  local previous=leaderGap
  for i=2,#candidates do
    local item=candidates[i]
    if item.gap-previous>28.0 then break end
    group[#group+1]=item.index
    previous=item.gap
  end
  return group,previous
end

local function safeFollowingCurrentLane(carIndex,car,p,currentGap,currentSpeed)
  -- Follow only the leader in the car's current corridor. A vehicle in a
  -- separate usable lane must never reduce this car's target speed.
  return safeFollowingSpeed(currentGap,currentSpeed,p.brakeAccelEstimate)
end

local function beginOvertake(carIndex,car,p,gap,leaderIndex,leaderLateral,speed,passDistance,forcedOffset)
  if p.overtakeRetryT>0 then return false end
  local offset=forcedOffset or findPassingOffset(carIndex,car,p,leaderIndex,leaderLateral,speed,passDistance)
  if forcedOffset~=nil then
    local reserved=passingReservationBlocked(carIndex,p,offset,speed)
    local clear,reason=passingPathClear(carIndex,car,p,leaderIndex,
      p.laneOffsetCurrent or p.latOff or 0,offset,passDistance or 60.0,speed)
    if reserved or laneOccupied(carIndex,car,offset,leaderIndex) or not clear then
      p.lastPassDeniedReason=reason or "yielded_corridor_no_longer_clear"
      return false
    end
  end
  if offset==nil then
    p.overtakeRetryT=0.25
    if not p.overtakeDeniedLogged then
      H.log("CAR=%d OVERTAKE_DENIED reason=%s reserved_by=%d gap=%.1f",
        carIndex,tostring(p.lastPassDeniedReason or "no_free_passing_offset"),
        safeNumber(p.lastPassReservationOwner,-1),gap or -1)
      p.overtakeDeniedLogged=true
    end
    return false
  end
  local defaultCenter=laneCenters(p.segIdx)
  local bl,br=H.boundsAt(p.segIdx)
  local source=p.laneOffsetCurrent or p.laneOffset or p.baseLaneOffset or
    defaultCenter or ((bl+br)*0.5)
  p.overtakeDeniedLogged=false
  p.passFromOffset=source;p.passLat=offset;p.laneOffset=offset
  p.passGroupTargets=p.pendingYieldGroup or {leaderIndex}
  p.pendingYieldGroup=nil
  p.overtakeSide=offset<source and -1 or 1
  p.passTargetIndex=leaderIndex
  p.passT=0
  p.passBlockedT=0
  p.passDeadline=math.max(PASS_TIMEOUT,
    (math.max(20.0,safeNumber(passDistance,60.0))/
      math.max(8.0,speed))*2.0+4.0)
  p.followTimer=0;p.blockedT=0
  p.emergencyPass=p.emergencyPassRequest==true;p.emergencyPassRequest=false
  p.pendingYieldTarget=nil;p.pendingPassOffset=nil;p.pendingYieldTimer=0;p.pendingYieldOrder=nil
  p.pendingYieldGap=nil;p.pendingYieldSpeed=nil;p.pendingPassDistance=nil
  H.log("CAR=%d OVERTAKE_BEGIN target=%d group=%d from=%.2f to=%.2f gap=%.1f mode=%s",
    carIndex,leaderIndex or -1,#(p.passGroupTargets or {}),source,offset,gap or -1,
    p.emergencyPass and "URGENT" or "NORMAL")
  return true
end

local function updateTrafficControl(carIndex, car, p, speed, targetSpeed, dt)
  p.overtakeRetryT=math.max(0,safeNumber(p.overtakeRetryT)-dt)
  p.trafficAction="OPEN"
  p.trafficReason="clear_path"
  p.trafficLeaderIndex=-1
  p.trafficGap=-1
  p.trafficLeaderSpeed=0
  p.trafficTargetOffset=safeNumber(p.laneOffsetCurrent,p.latOff or 0)
  if p.overtakeSide~=0 then
    p.trafficPriority=1000000+math.max(0,100-safeNumber(p.passT))
    p.trafficAction="PASSING"
    p.trafficReason="reserved_pass"
    p.trafficLeaderIndex=safeNumber(p.passTargetIndex,-1)
    p.trafficTargetOffset=safeNumber(p.passLat,p.laneOffset or 0)
  elseif p.returningFromPass then
    p.trafficPriority=900000
    p.trafficAction="RETURNING"
    p.trafficReason="pass_completed"
  else
    p.trafficPriority=0
  end
  local ownProgress, ownLateral, ownSeg = routeProjection(car, p.segIdx, not p.routeLocked, p.routeProgress, speed, dt)
  if ownProgress == nil then
    p.projectionLost = true
    p.trafficPriority=0
    p.trafficAction="ROUTE_LOST"
    p.trafficReason="projection_failed"
    return 0
  end
  p.routeLocked = true
  if ownSeg then p.segIdx = ownSeg end
  p.routeProgress = ownProgress
  if ownLateral~=nil then p.latOff=ownLateral end
  if p.yieldRequestBy~=nil then
    local requester=state.p[p.yieldRequestBy]
    local requesterCar=ac.getCar(p.yieldRequestBy)
    local requestedByPending=requester and
      (requester.pendingYieldTarget==carIndex or
       listContains(requester.pendingYieldGroup,carIndex))
    local requestedByActivePass=requester and requester.overtakeSide~=0 and
      listContains(requester.passGroupTargets,carIndex)
    if not requester or not requesterCar or not requesterCar.isActive or
       (not requestedByPending and not requestedByActivePass) or
       (requester.overtakeSide~=0 and not requestedByActivePass) or
       p.overtakeSide~=0 or p.returningFromPass then
      p.yieldRequestBy=nil;p.yieldOffset=nil;p.yieldReady=false;p.yieldRequestTimer=0
    else
      p.yieldRequestTimer=safeNumber(p.yieldRequestTimer)+dt
      p.laneOffset=p.yieldOffset
      local lateralError=math.abs(safeNumber(p.latOff)-safeNumber(p.yieldOffset))
      local lateralSpeed=math.abs(safeNumber(p.lateralVelocityFiltered))
      p.yieldReady=lateralError<=0.28 and lateralSpeed<=0.65
      p.trafficAction="YIELDING"
      p.trafficReason=p.yieldReady and "yield_position_confirmed" or "moving_to_yield_position"
      p.trafficLeaderIndex=p.yieldRequestBy;p.trafficTargetOffset=safeNumber(p.yieldOffset)
      if p.yieldRequestTimer>8.0 and not requestedByActivePass then
        releaseYieldGroup(p.yieldRequestBy,requester.pendingYieldGroup)
        requester.pendingYieldGroup=nil
        requester.pendingYieldTarget=nil;requester.pendingPassOffset=nil;requester.pendingYieldTimer=0;requester.pendingYieldOrder=nil
        requester.pendingYieldGap=nil;requester.pendingYieldSpeed=nil;requester.pendingPassDistance=nil
        p.yieldRequestBy=nil;p.yieldOffset=nil;p.yieldReady=false;p.yieldRequestTimer=0
        H.log("CAR=%d YIELD_ABORT reason=timeout",carIndex)
      end
    end
  end
  if p.returningFromPass then
    p.trafficPriority=900000
    return targetSpeed
  end
  local leftCenter,rightCenter=laneCenters(p.segIdx)
  local bl,br=H.boundsAt(p.segIdx)
  local defaultOffset=p.baseLaneOffset or leftCenter or ((bl+br)*0.5)
  local currentLaneOffset=p.overtakeSide~=0 and (p.passLat or p.laneOffset) or
    (p.laneOffsetCurrent or p.laneOffset or defaultOffset)
  local gap,leaderSpeed,leaderIndex,leaderLateral=carAheadInLane(
    carIndex,car,currentLaneOffset or 0)
  p.trafficLeaderIndex=leaderIndex or -1
  p.trafficGap=gap or -1
  p.trafficLeaderSpeed=leaderSpeed or 0
  if p.pendingYieldTarget~=nil then
    local pendingState=state.p[p.pendingYieldTarget]
    local pendingCar=ac.getCar(p.pendingYieldTarget)
    local pendingGap=pendingState and pendingState.routeProgress and
      routeGap(p.routeProgress,pendingState.routeProgress) or nil
    local pendingOwner=pendingState and pendingState.yieldRequestBy
    local pendingOwnerState=pendingOwner~=nil and state.p[pendingOwner] or nil
    local ownerStillOwnsRequest=pendingOwner==carIndex or
      (pendingOwnerState and
       (pendingOwnerState.pendingYieldTarget==p.pendingYieldTarget or
        (pendingOwnerState.overtakeSide~=0 and
         listContains(pendingOwnerState.passGroupTargets,p.pendingYieldTarget))))
    local ownerAvailable=pendingOwner==nil and p.pendingYieldTarget==leaderIndex
    local reservationValid=pendingState and pendingCar and pendingCar.isActive and
      (ownerStillOwnsRequest or ownerAvailable) and pendingGap and
      pendingGap>0 and pendingGap<math.max(120.0,speed*5.0+50.0)
    if not reservationValid then
      releaseYieldGroup(carIndex,p.pendingYieldGroup or {p.pendingYieldTarget})
      p.pendingYieldGroup=nil
      p.pendingYieldTarget=nil;p.pendingPassOffset=nil;p.pendingYieldTimer=0;p.pendingYieldOrder=nil
      p.pendingYieldGap=nil;p.pendingYieldSpeed=nil;p.pendingPassDistance=nil
    end
  end
  if p.yieldRequestBy~=nil and p.yieldOffset~=nil then
    p.trafficAction="YIELDING"
    p.trafficReason=p.yieldReady and "yield_position_confirmed" or "moving_to_yield_position"
    if gap~=nil then return math.min(targetSpeed,safeFollowingSpeed(gap,leaderSpeed or 0,p.brakeAccelEstimate)) end
    return targetSpeed
  end

  -- Once a convoy has accepted a group yield request, keep that transaction
  -- alive even after the first car moves out of the original lane and is no
  -- longer returned by the lane-leader query.
  if p.pendingYieldTarget~=nil and p.pendingYieldTarget~=leaderIndex then
    local targetIndex=p.pendingYieldTarget
    local targetState=state.p[targetIndex]
    local targetCar=ac.getCar(targetIndex)
    if targetState and targetCar and targetCar.isActive then
      local pendingProgress=targetState.routeProgress
      local pendingGap=pendingProgress and routeGap(p.routeProgress,pendingProgress) or
        safeNumber(p.pendingYieldGap,12.0)
      if targetState.yieldRequestBy==carIndex then
        if pendingGroupReady(p,carIndex) then
          if beginOvertake(carIndex,car,p,safeNumber(p.pendingYieldGap,12.0),
            targetIndex,targetState.latOff,speed,p.pendingPassDistance,p.pendingPassOffset) then
            p.trafficPriority=1000000;p.trafficAction="PASSING"
            p.trafficReason="group_yield_confirmed";p.trafficLeaderIndex=targetIndex
            p.trafficTargetOffset=safeNumber(p.passLat)
            return targetSpeed
          end
          releaseYieldGroup(carIndex,p.pendingYieldGroup)
          p.pendingYieldGroup=nil
          p.pendingYieldTarget=nil;p.pendingPassOffset=nil;p.pendingYieldTimer=0
          p.pendingYieldOrder=nil;p.pendingYieldGap=nil;p.pendingYieldSpeed=nil
          p.pendingPassDistance=nil
        else
          p.trafficAction="WAIT_YIELD";p.trafficReason="group_not_yet_clear"
          p.trafficLeaderIndex=targetIndex;p.trafficGap=pendingGap
          return math.min(targetSpeed,safeFollowingSpeed(pendingGap,
            targetCar.speedMs,p.brakeAccelEstimate))
        end
      else
        p.trafficAction="WAIT_QUEUE";p.trafficReason="earlier_pass_has_slot"
        p.trafficLeaderIndex=targetIndex;p.trafficGap=pendingGap
        return math.min(targetSpeed,safeFollowingSpeed(pendingGap,
          targetCar.speedMs,p.brakeAccelEstimate))
      end
    end
  end

  if p.overtakeSide ~= 0 then
    p.passT = p.passT + dt
    local rightGap,rightLeaderSpeed=carAheadInLane(
      carIndex,car,p.passLat or p.laneOffset or currentLaneOffset or 0,
      p.passTargetIndex)
    local passSafeSpeed = targetSpeed
    p.trafficGap=rightGap or gap or -1
    p.trafficLeaderIndex=p.passTargetIndex or leaderIndex or -1
    if rightGap then
      p.trafficLeaderSpeed=rightLeaderSpeed or 0
      passSafeSpeed = math.min(passSafeSpeed,
        safeFollowingSpeed(rightGap, rightLeaderSpeed or 0,p.brakeAccelEstimate))
    end

    -- A pass can become blocked after its initial corridor reservation
    -- (another car may enter the lane or the target may stop). Do not keep
    -- the AI committed to that lane indefinitely. Abort only after sustained
    -- near-zero progress and only when the original lane is physically clear.
    if speed < 3.0 and passSafeSpeed < 2.5 then
      p.passBlockedT = safeNumber(p.passBlockedT) + dt
    else
      p.passBlockedT = 0
    end

    local returnOffset=p.passFromOffset or p.baseLaneOffset or defaultOffset
    local passExpired=p.passT >= safeNumber(p.passDeadline,PASS_TIMEOUT)
    local passDeadlocked=p.passBlockedT >= 2.5
    if (passExpired or passDeadlocked) and
       not laneOccupied(carIndex,car,returnOffset,nil) then
      local yielded=state.p[p.passTargetIndex]
      p.overtakeSide=0
      p.passTargetIndex=nil
      p.emergencyPass=false
      p.returningFromPass=true
      p.laneOffset=returnOffset
      p.baseLaneOffset=returnOffset
      p.passFromOffset=nil
      p.passLat=nil
      p.passT=0
      p.passBlockedT=0
      p.passDeadline=nil
      p.followTimer=0
      p.blockedT=0
      p.overtakeRetryT=2.0
      releaseYieldGroup(carIndex,p.passGroupTargets)
      p.passGroupTargets=nil
      if yielded and yielded.yieldRequestBy==carIndex then
        yielded.yieldRequestBy=nil;yielded.yieldOffset=nil;yielded.yieldReady=false;yielded.yieldRequestTimer=0
      end
      H.log("CAR=%d OVERTAKE_ABORT reason=%s return=%.2f",
        carIndex,passDeadlocked and "blocked" or "timeout",returnOffset)
      return targetSpeed
    end

    local clearanceTargetIndex=p.passGroupTargets and
      p.passGroupTargets[#p.passGroupTargets] or p.passTargetIndex
    local target = clearanceTargetIndex and ac.getCar(clearanceTargetIndex) or nil
    local passedClearance = false
    if target and target.isActive then
      local targetState = state.p[clearanceTargetIndex]
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
      local ownSample=Geometry.sample(state.routeGeometry,currentProgress or ownProgress)
      local targetRouteSample=targetProgress and
        Geometry.sample(state.routeGeometry,targetProgress)
      local targetFx,targetFz=normalize2(target.look.x,target.look.z)
      local targetDirectionDot=targetRouteSample and
        (targetFx*targetRouteSample.tx+targetFz*targetRouteSample.tz) or 1.0
      local passedDistance
      if targetProgress and targetDirectionDot>=-0.25 then
        passedDistance=routeGap(targetProgress,currentProgress or ownProgress)
      else
        local targetLongitudinal=relativeToRoute(target.position,ownSample)
        passedDistance=targetLongitudinal~=nil and -targetLongitudinal or -math.huge
      end
      local ownLength = (car.aabbSize and car.aabbSize.z) or 4.5
      local otherLength = (target.aabbSize and target.aabbSize.z) or 4.5
      passedClearance = passedDistance >= ownLength + otherLength
    else
      -- A removed/despawned target is not a reason to cut back blindly:
      -- the full lane occupancy check below is still mandatory.
      passedClearance = true
    end

    if passedClearance and
       not laneOccupied(carIndex,car,returnOffset,p.passTargetIndex) then
      local yielded=state.p[p.passTargetIndex]
      p.overtakeSide=0
      p.passTargetIndex=nil
      p.emergencyPass=false
      p.returningFromPass=true
      p.laneOffset=returnOffset
      p.baseLaneOffset=returnOffset
      p.passFromOffset=nil
      p.passLat=nil
      p.passT=0
      p.passBlockedT=0
      p.passDeadline=nil
      p.followTimer=0
      p.blockedT=0
      releaseYieldGroup(carIndex,p.passGroupTargets)
      p.passGroupTargets=nil
      if yielded and yielded.yieldRequestBy==carIndex then
        yielded.yieldRequestBy=nil;yielded.yieldOffset=nil;yielded.yieldReady=false;yielded.yieldRequestTimer=0
      end
      H.log("CAR=%d OVERTAKE_END return=%.2f clearance=2_car_lengths",
        carIndex,returnOffset)
      return targetSpeed
    end

    -- Hold the selected free offset until the target is fully behind us.
    p.laneOffset=p.passLat
    return passSafeSpeed
  end

  p.overtakeDisabledLogged=false
  if p.overtakeSide==0 then
    p.baseLaneOffset=p.baseLaneOffset or defaultOffset
    p.laneOffset=p.baseLaneOffset
  end

  if not gap or not leaderIndex then
    if p.pendingYieldTarget~=nil then
      local targetIndex=p.pendingYieldTarget
      if not p.pendingYieldOrder then state.trafficRequestSerial=(state.trafficRequestSerial or 0)+1;p.pendingYieldOrder=state.trafficRequestSerial end
      local targetState=state.p[targetIndex]
      local targetCar=ac.getCar(targetIndex)
      if targetState and targetCar and targetCar.isActive then
        if targetState.yieldRequestBy==nil then
          local pendingOffset=p.pendingPassOffset
          if pendingOffset==nil and p.overtakeRetryT<=0 then
            pendingOffset=findPassingOffset(carIndex,car,p,targetIndex,targetState.latOff,speed,p.pendingPassDistance)
            if pendingOffset==nil then p.overtakeRetryT=0.25 end
          end
          p.pendingPassOffset=pendingOffset
          local yieldOffset=pendingOffset and chooseYieldOffset(targetIndex,targetCar,targetState,pendingOffset,carIndex) or nil
          if pendingOffset~=nil and yieldOffset==nil then p.overtakeRetryT=0.25 end
          if yieldOffset~=nil then
            targetState.yieldRequestBy=carIndex;targetState.yieldOffset=yieldOffset
            targetState.yieldReady=false;targetState.yieldRequestTimer=0
            H.log("CAR=%d YIELD_REQUEST target=%d yield=%.2f pass=%.2f",carIndex,targetIndex,yieldOffset,safeNumber(p.pendingPassOffset))
          end
        end
        if targetState.yieldRequestBy==carIndex and pendingGroupReady(p,carIndex) then
          p.emergencyPassRequest=false
          if beginOvertake(carIndex,car,p,safeNumber(p.pendingYieldGap,12.0),targetIndex,targetState.latOff,speed,p.pendingPassDistance,p.pendingPassOffset) then
            p.trafficPriority=1000000;p.trafficAction="PASSING"
            p.trafficReason="yield_confirmed";p.trafficLeaderIndex=targetIndex
            p.trafficTargetOffset=safeNumber(p.passLat)
            return targetSpeed
          end
          releaseYieldGroup(carIndex,p.pendingYieldGroup)
          p.pendingYieldGroup=nil
          targetState.yieldRequestBy=nil;targetState.yieldOffset=nil;targetState.yieldReady=false
          p.pendingYieldTarget=nil;p.pendingPassOffset=nil;p.pendingYieldTimer=0
          p.pendingYieldOrder=nil;p.pendingYieldGap=nil;p.pendingYieldSpeed=nil
          p.pendingPassDistance=nil
        else
          local targetProgress=routeProjection(targetCar,targetState.segIdx,not targetState.routeLocked,targetState.routeProgress,targetCar.speedMs,dt)
          local queueGap=targetProgress and routeGap(p.routeProgress,targetProgress) or safeNumber(p.pendingYieldGap,12.0)
          p.trafficAction=targetState.yieldRequestBy==carIndex and "WAIT_YIELD" or
            (targetState.yieldRequestBy~=nil and "WAIT_QUEUE" or "FOLLOW")
          p.trafficReason=targetState.yieldRequestBy==carIndex and "leader_not_yet_clear" or
            (targetState.yieldRequestBy~=nil and "earlier_pass_has_slot" or "no_safe_pass_corridor")
          p.trafficLeaderIndex=targetIndex;p.trafficGap=queueGap
          return math.min(targetSpeed,safeFollowingSpeed(queueGap,targetCar.speedMs,p.brakeAccelEstimate))
        end
      end
      p.pendingYieldTarget=nil;p.pendingPassOffset=nil;p.pendingYieldTimer=0;p.pendingYieldOrder=nil
      p.pendingYieldGap=nil;p.pendingYieldSpeed=nil;p.pendingPassDistance=nil
    end
    p.followTimer = 0
    p.blockedT = 0
    p.trafficPriority=0
    p.trafficAction="OPEN"
    p.trafficReason="no_leader_in_path"
    return targetSpeed
  end
  p.trafficAction="FOLLOW"
  p.trafficReason="leader_in_path"

  local leaderPace=math.max(0,leaderSpeed or 0)
  local acceleration=clamp(safeNumber(p.accelEstimate,4.0),0.5,18.0)
  local leaderState=state.p[leaderIndex]
  local leaderAcceleration=clamp(safeNumber(
    leaderState and leaderState.accelEstimate,acceleration),0.5,18.0)
  -- Compare predicted speeds, not our acceleration against a fictitious
  -- stationary leader: equal cars at equal speed must not trigger passes.
  local relativeAcceleration=acceleration-leaderAcceleration
  local speedAdvantage=speed-leaderPace+relativeAcceleration*1.5
  local slower=speedAdvantage>=1.5
  if slower then
    local targetCar=ac.getCar(leaderIndex)
    local ownLength=math.max(3.0,safeNumber(car.aabbSize and car.aabbSize.z,4.5))
    local targetLength=math.max(3.0,safeNumber(targetCar and targetCar.aabbSize and targetCar.aabbSize.z,4.5))
    local clearanceDistance=ownLength+targetLength+6.0
    -- Include the actual distance to the leader. Estimating only the
    -- side-by-side clearance made the planner return to the source lane
    -- before the target was passed when the initial gap was large.
    local requiredClosure=math.max(0,safeNumber(gap))+clearanceDistance
    local relativeSpeed=speed-leaderPace
    local passTime=math.huge
    local estimatedPassDistance=math.huge
    if relativeSpeed>=1.0 then
      passTime=requiredClosure/relativeSpeed
      estimatedPassDistance=speed*passTime
    elseif relativeAcceleration>0.25 then
      passTime=(-relativeSpeed+
        math.sqrt(relativeSpeed*relativeSpeed+
          2.0*relativeAcceleration*requiredClosure))/relativeAcceleration
      estimatedPassDistance=speed*passTime+
        0.5*acceleration*passTime*passTime
    end
    local urgentGap=clamp(22.0+speed*1.25,32.0,85.0)
    local canActuallyPass=speedAdvantage>=1.5 and estimatedPassDistance<=260.0 and
      speed>=(leaderSpeed or 0)-4.0
    local urgent=canActuallyPass and gap<=urgentGap
    local passWindow=math.max(65.0,48.0+speed*2.0)
    p.followTimer=p.followTimer+dt
    local passRequestReady=(p.pendingYieldTarget==leaderIndex or (canActuallyPass and gap<=passWindow)) and
      p.overtakeSide==0 and p.overtakeRetryT<=0
    if passRequestReady then
      -- Requests are ranked by urgency before cars are updated. The next
      -- frame's deterministic ordering lets the most urgent valid request
      -- reserve first, instead of granting every simultaneous request to
      -- whichever car happens to have the lowest simulation index.
      p.trafficPriority=10000+
        clamp(passWindow-gap,0,passWindow)*10+
        clamp(speedAdvantage,0,50)*2+
        (urgent and 1000 or 0)
    else
      p.trafficPriority=0
    end
    if passRequestReady then
      local leaderState=state.p[leaderIndex]
      local leaderCar=ac.getCar(leaderIndex)
      if leaderState and leaderCar and leaderCar.isActive and
         leaderState.yieldRequestBy~=nil and leaderState.yieldRequestBy~=carIndex then
        -- A pending reservation is not an active pass. A materially stronger
        -- eligible car may take over that request before the yield manoeuvre
        -- begins; once the pass starts, its reservation is never pre-empted.
        local ownerIndex=leaderState.yieldRequestBy
        local ownerState=state.p[ownerIndex]
        local ownerCar=ac.getCar(ownerIndex)
        if ownerState and ownerCar and ownerCar.isActive and
           ownerState.overtakeSide==0 and not ownerState.returningFromPass then
          local ownScore=clamp(safeNumber(p.accelEstimate,4.0),0.5,18.0)*100.0+
            math.max(0,safeNumber(car.speedMs))*2.0
          local ownerScore=clamp(safeNumber(ownerState.accelEstimate,4.0),0.5,18.0)*100.0+
            math.max(0,safeNumber(ownerCar.speedMs))*2.0
          if ownScore>ownerScore+25.0 then
            releaseYieldGroup(ownerIndex,ownerState.pendingYieldGroup or {leaderIndex})
            ownerState.pendingYieldGroup=nil
            ownerState.pendingYieldTarget=nil;ownerState.pendingPassOffset=nil
            ownerState.pendingYieldTimer=0;ownerState.pendingYieldOrder=nil
            ownerState.pendingYieldGap=nil;ownerState.pendingYieldSpeed=nil
            ownerState.pendingPassDistance=nil
            leaderState.yieldRequestBy=nil;leaderState.yieldOffset=nil
            leaderState.yieldReady=false;leaderState.yieldRequestTimer=0
            H.log("YIELD_PREEMPT leader=%d old=%d new=%d old_score=%.1f new_score=%.1f",
              leaderIndex,ownerIndex,carIndex,ownerScore,ownScore)
          end
        end
      end
      if not leaderState or not leaderCar or not leaderCar.isActive or
         not leaderCar.isAIControlled or leaderState.overtakeSide~=0 or
         leaderState.returningFromPass or leaderState.pendingYieldTarget~=nil then
        p.lastPassDeniedReason="leader_not_yield_capable"
      elseif leaderState.yieldRequestBy~=nil and leaderState.yieldRequestBy~=carIndex then
        p.pendingYieldTarget=leaderIndex;p.pendingYieldGap=gap
        if not p.pendingYieldOrder then state.trafficRequestSerial=(state.trafficRequestSerial or 0)+1;p.pendingYieldOrder=state.trafficRequestSerial end
        p.pendingYieldSpeed=leaderSpeed or 0;p.pendingPassDistance=estimatedPassDistance
        p.trafficAction="WAIT_QUEUE";p.trafficReason="earlier_pass_has_slot"
        p.trafficLeaderIndex=leaderIndex
        return math.min(targetSpeed,safeFollowingCurrentLane(carIndex,car,p,gap,leaderSpeed or 0))
      else
        local passOffset=p.pendingYieldTarget==leaderIndex and p.pendingPassOffset or nil
        if passOffset==nil and p.overtakeRetryT<=0 then
          passOffset=findPassingOffset(carIndex,car,p,leaderIndex,leaderLateral,speed,estimatedPassDistance)
          if passOffset==nil then p.overtakeRetryT=0.25 end
        end
        if passOffset~=nil then
          if leaderState.yieldRequestBy==nil or leaderState.yieldRequestBy==carIndex then
            if leaderState.yieldRequestBy==nil then
              local group,farGap=collectYieldGroup(carIndex,car,p,leaderIndex,
                math.min(110.0,math.max(45.0,estimatedPassDistance+20.0)))
              local groupPassDistance=estimatedPassDistance
              if #group>1 then
                local farState=state.p[group[#group]]
                local farCar=ac.getCar(group[#group])
                local ownLength=math.max(3.0,safeNumber(car.aabbSize and car.aabbSize.z,4.5))
                local farLength=math.max(3.0,safeNumber(farCar and farCar.aabbSize and farCar.aabbSize.z,4.5))
                local required=farGap+ownLength+farLength+6.0
                local relative=speed-(farCar and safeNumber(farCar.speedMs) or leaderPace)
                local farAcceleration=clamp(safeNumber(
                  farState and farState.accelEstimate,leaderAcceleration),0.5,18.0)
                local relativeAccel=acceleration-farAcceleration
                local t=math.huge
                if relative>=1.0 then
                  t=required/relative
                elseif relativeAccel>0.25 then
                  t=(-relative+math.sqrt(relative*relative+
                    2.0*relativeAccel*required))/relativeAccel
                end
                local groupDistanceEstimate=1000.0
                if t<math.huge and t==t then
                  groupDistanceEstimate=speed*t+0.5*acceleration*t*t
                end
                groupPassDistance=math.max(groupPassDistance,
                  math.min(1000.0,groupDistanceEstimate))
                local groupOffset=groupPassDistance<=300.0 and
                  findPassingOffset(carIndex,car,p,leaderIndex,
                    leaderLateral,speed,groupPassDistance) or nil
                if groupOffset~=nil then passOffset=groupOffset
                else group={leaderIndex};groupPassDistance=estimatedPassDistance end
              end
              local assigned={}
              local groupOK=true
              for gi=1,#group do
                local memberIndex=group[gi]
                local member=state.p[memberIndex]
                local memberCar=ac.getCar(memberIndex)
                local memberYield=member and memberCar and
                  chooseYieldOffset(memberIndex,memberCar,member,passOffset,carIndex,group) or nil
                if not member or not memberCar or not memberCar.isActive or
                   (member.yieldRequestBy~=nil and member.yieldRequestBy~=carIndex) or
                   memberYield==nil then
                  groupOK=false
                  break
                end
                assigned[#assigned+1]={index=memberIndex,offset=memberYield}
              end
              if not groupOK and #group>1 then
                -- If the full pack cannot yield into one common safe slot,
                -- downgrade to the nearest leader instead of abandoning a
                -- valid ordinary one-car overtake.
                group={leaderIndex};groupPassDistance=estimatedPassDistance
                passOffset=findPassingOffset(carIndex,car,p,leaderIndex,
                  leaderLateral,speed,groupPassDistance)
                assigned={}
                local member=state.p[leaderIndex]
                local memberCar=ac.getCar(leaderIndex)
                local memberYield=passOffset and member and memberCar and
                  chooseYieldOffset(leaderIndex,memberCar,member,passOffset,carIndex,group) or nil
                groupOK=member~=nil and memberCar~=nil and memberCar.isActive and
                  member.yieldRequestBy==nil and memberYield~=nil
                if groupOK then assigned[1]={index=leaderIndex,offset=memberYield} end
              end
              if groupOK then
                for gi=1,#assigned do
                  local item=assigned[gi]
                  local member=state.p[item.index]
                  member.yieldRequestBy=carIndex;member.yieldOffset=item.offset
                  member.yieldReady=false;member.yieldRequestTimer=0
                  H.log("CAR=%d YIELD_REQUEST target=%d yield=%.2f pass=%.2f group=%d",
                    carIndex,item.index,item.offset,passOffset,#group)
                end
                p.pendingYieldGroup=group
                p.pendingYieldTarget=leaderIndex;p.pendingPassOffset=passOffset;p.pendingYieldTimer=0
                if not p.pendingYieldOrder then
                  state.trafficRequestSerial=(state.trafficRequestSerial or 0)+1
                  p.pendingYieldOrder=state.trafficRequestSerial
                end
                p.pendingYieldGap=gap;p.pendingYieldSpeed=leaderSpeed or 0
                p.pendingPassDistance=groupPassDistance
              else
                releaseYieldGroup(carIndex,assigned and (function()
                  local ids={} for _,v in ipairs(assigned) do ids[#ids+1]=v.index end return ids
                end)() or group)
                p.pendingYieldGroup=nil
                p.lastPassDeniedReason="group_yield_unavailable";p.overtakeRetryT=0.25
              end
            end
            if leaderState.yieldRequestBy==carIndex then
              p.pendingYieldTimer=safeNumber(p.pendingYieldTimer)+dt
              if pendingGroupReady(p,carIndex) then
                p.emergencyPassRequest=urgent
                if beginOvertake(carIndex,car,p,gap,leaderIndex,leaderLateral,speed,p.pendingPassDistance or estimatedPassDistance,p.pendingPassOffset) then
                  p.trafficPriority=1000000;p.trafficAction="PASSING"
                  p.trafficReason="yield_confirmed";p.trafficLeaderIndex=leaderIndex
                  p.trafficTargetOffset=safeNumber(p.passLat)
                  return targetSpeed
                end
                releaseYieldGroup(carIndex,p.pendingYieldGroup)
                p.pendingYieldGroup=nil
                leaderState.yieldRequestBy=nil;leaderState.yieldOffset=nil;leaderState.yieldReady=false;leaderState.yieldRequestTimer=0
                p.pendingYieldTarget=nil;p.pendingPassOffset=nil;p.pendingYieldTimer=0;p.pendingYieldOrder=nil;p.pendingYieldGap=nil;p.pendingYieldSpeed=nil;p.pendingPassDistance=nil
              else
                p.trafficAction="WAIT_YIELD";p.trafficReason="leader_not_yet_clear"
                p.trafficLeaderIndex=leaderIndex;p.trafficTargetOffset=safeNumber(p.pendingPassOffset)
                if p.pendingYieldTimer>8.5 then
                  releaseYieldGroup(carIndex,p.pendingYieldGroup)
                  p.pendingYieldGroup=nil
                  leaderState.yieldRequestBy=nil;leaderState.yieldOffset=nil;leaderState.yieldReady=false;leaderState.yieldRequestTimer=0
                  p.pendingYieldTarget=nil;p.pendingPassOffset=nil;p.pendingYieldTimer=0;p.pendingYieldOrder=nil
                  p.pendingYieldGap=nil;p.pendingYieldSpeed=nil;p.pendingPassDistance=nil
                  p.lastPassDeniedReason="yield_timeout"
                else
                  return math.min(targetSpeed,safeFollowingCurrentLane(carIndex,car,p,gap,leaderSpeed or 0))
                end
              end
            else
              p.lastPassDeniedReason="leader_has_no_safe_yield_space"
              p.trafficAction="FOLLOW";p.trafficReason="leader_has_no_safe_yield_space"
              return math.min(targetSpeed,safeFollowingCurrentLane(carIndex,car,p,gap,leaderSpeed or 0))
            end
          else
            p.lastPassDeniedReason="yield_slot_owned_by_other_car"
          end
        end
      end
      p.trafficAction="FOLLOW"
      p.trafficReason=p.lastPassDeniedReason or "yield_not_available"
      p.emergencyPassRequest=false
    elseif p.pendingYieldTarget~=nil then
      local oldLeader=state.p[p.pendingYieldTarget]
      if oldLeader and oldLeader.yieldRequestBy==carIndex then
        oldLeader.yieldRequestBy=nil;oldLeader.yieldOffset=nil;oldLeader.yieldReady=false;oldLeader.yieldRequestTimer=0
      end
      releaseYieldGroup(carIndex,p.pendingYieldGroup or {p.pendingYieldTarget})
      p.pendingYieldGroup=nil
      p.pendingYieldTarget=nil;p.pendingPassOffset=nil;p.pendingYieldTimer=0
    end
    local safe=safeFollowingCurrentLane(carIndex,car,p,gap,leaderSpeed or 0)
    return math.min(targetSpeed,safe)
  end

  p.followTimer = 0
  p.blockedT = 0
  p.trafficPriority=0
  local safe = safeFollowingSpeed(gap, leaderSpeed or 0,p.brakeAccelEstimate)
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
  local lateralLimit=clamp(safeNumber(p.lateralAccelLimit,CFG.MAX_LATERAL_ACCEL),4.0,20.0)
  local lateralUse=clamp(math.abs(safeNumber(p.lateralAccel))/
    math.max(0.1,lateralLimit),0,1)
  local cornerScale=1.0-math.min(0.10,CFG.TC_CORNER_CUT*0.45)*lateralUse*lateralUse
  gas=gas*p.tractionScale*cornerScale

  if speed<5 then gas=math.min(gas,0.82) end
  if error>=0 then brake=0 end
  p.gas=clamp(gas,0,CFG.MAX_GAS)
  p.brake=clamp(brake,0,CFG.MAX_BRAKE)
end

-- High-rate dynamics trace. This is observation-only: it never changes controls.
-- Stable running is sampled at 10 Hz; suspected instability is sampled at 20 Hz.
local function logDynamicsTrace(carIndex,car,p,dt,controlWriteOk)
  if not state.steeringDiagnostics then return end
  local speed=math.max(0,safeNumber(car.speedMs))
  if speed<15.0 then return end
  local velocity=car.velocity
  local localVelocity=car.localVelocity
  local angular=car.localAngularVelocity
  local look=car.look
  local slip=math.abs(safeNumber(p.slipAngle))
  local lateral=math.abs(safeNumber(p.lateralVelocity))
  local yaw=math.abs(safeNumber(p.measuredYawRate))
  local steering=math.abs(safeNumber(p.steerTarget))
  local unstable=slip>0.035 or lateral>0.8 or yaw>0.20 or steering>0.05
  local interval=unstable and 0.05 or 0.10
  p.dynTraceTimer=(p.dynTraceTimer or 0)+math.max(0,safeNumber(dt))
  if p.dynTraceTimer+1e-6<interval then return end
  p.dynTraceTimer=p.dynTraceTimer-interval
  if p.dynTraceTimer>interval then p.dynTraceTimer=0 end

  H.log("DYN_TRACE id=%d v=%.3f dt=%.4f pos=(%.3f,%.3f,%.3f) look=(%.5f,%.5f,%.5f) worldV=(%.4f,%.4f,%.4f) localV=(%.4f,%.4f,%.4f) omega=(%.5f,%.5f,%.5f) route=%.3f lateral=%.4f cross=%.4f head=%.5f roadK=%.6f slip=%.6f latV=%.5f yawLook=%.5f yawPhys=%.5f axleF=%.5f axleR=%.5f wheelSlip=%.5f pp=%.5f boundK=%.5f bound=%.5f raw=%.5f target=%.5f filtered=%.5f rateDemand=%.5f rateApplied=%.5f final=%.5f gas=%.4f brake=%.4f targetSpeed=%.3f offset=%.4f planOffset=%.4f planGen=%d planMode=%s controlOK=%d",
    carIndex,speed,math.max(0,safeNumber(dt)),
    safeNumber(car.position and car.position.x),safeNumber(car.position and car.position.y),
    safeNumber(car.position and car.position.z),
    safeNumber(look and look.x),safeNumber(look and look.y),safeNumber(look and look.z),
    safeNumber(velocity and velocity.x),safeNumber(velocity and velocity.y),safeNumber(velocity and velocity.z),
    safeNumber(localVelocity and localVelocity.x),safeNumber(localVelocity and localVelocity.y),
    safeNumber(localVelocity and localVelocity.z),
    safeNumber(angular and angular.x),safeNumber(angular and angular.y),safeNumber(angular and angular.z),
    safeNumber(p.routeProgress),safeNumber(p.latOff),safeNumber(p.crossTrackError),
    safeNumber(p.headingError),safeNumber(p.roadCurvature0),safeNumber(p.slipAngle),
    safeNumber(p.lateralVelocity),safeNumber(p.yawRate),safeNumber(p.measuredYawRate),
    safeNumber(p.frontAxleSlip),safeNumber(p.rearAxleSlip),safeNumber(p.wheelSlip),
    safeNumber(p.pursuitSteer),safeNumber(p.boundaryCurvatureSteer),safeNumber(p.boundarySteer),
    safeNumber(p.rawSteer),safeNumber(p.steerTarget),safeNumber(p.steerFiltered),
    safeNumber(p.steerRateLimitDemand),safeNumber(p.steerRateLimitApplied),
    safeNumber(p.steer),safeNumber(p.gas),safeNumber(p.brake),safeNumber(p.targetSpeed),
    safeNumber(p.laneOffsetCurrent),safeNumber(p.raceOffsetRaw),
    safeNumber(p.racingPlanGeneration),tostring(p.racingPlanMode or "NONE"),
    controlWriteOk and 1 or 0)
end

local function logSteeringDiagnostics(carIndex,car,p,dt,controlWriteOk)
  if not state.steeringDiagnostics then return end
  dt=math.max(0,safeNumber(dt))
  p.steerDiagTimer=(p.steerDiagTimer or 0)+dt
  p.steerEventCooldown=math.max(0,(p.steerEventCooldown or 0)-dt)
  local flags={}
  local function flag(name) flags[#flags+1]=name end
  local target=safeNumber(p.steerTarget)
  local final=safeNumber(p.steer)
  local previousTarget=p.diagPreviousTarget
  local previousFinal=p.diagPreviousFinal
  local targetOffset=safeNumber(p.steeringTargetOffset)
  local targetAlpha=safeNumber(p.steeringAlpha)
  local previousOffset=p.diagPreviousTargetOffset
  local previousAlpha=p.diagPreviousTargetAlpha
  local targetSource=p.steeringTargetSource or "NONE"

  if p.roadStraight and math.abs(target)>0.01 and
     not p.diagWasSteeringOnStraight then flag("STEER_ON_STRAIGHT") end
  if previousTarget and previousTarget*target<0 and
     math.abs(previousTarget)>0.02 and math.abs(target)>0.02 then flag("TARGET_SIGN_REVERSAL") end
  if previousFinal and previousFinal*final<0 and
     math.abs(previousFinal)>0.02 and math.abs(final)>0.02 then flag("FINAL_SIGN_REVERSAL") end
  if previousOffset~=nil and math.abs(targetOffset-previousOffset)>0.20 then flag("TARGET_OFFSET_JUMP") end
  if previousAlpha~=nil and math.abs(targetAlpha-previousAlpha)>0.015 then flag("TARGET_ANGLE_JUMP") end
  if p.diagPreviousTargetSource~=nil and p.diagPreviousTargetSource~=targetSource then flag("TARGET_SOURCE_CHANGED") end
  if p.diagPreviousPlanGeneration~=nil and
     p.diagPreviousPlanGeneration~=(p.racingPlanGeneration or 0) then flag("RACING_PLAN_REBUILT") end

  local expectedProgress=math.max(0,safeNumber(car.speedMs))*dt
  local progressDelta=safeNumber(p.frameProgressDelta)
  local steerProjectionDelta=safeNumber(p.projectionProgressDelta)
  if p.routeLocked and dt>0 and
     math.abs(progressDelta-expectedProgress)>math.max(1.0,expectedProgress*0.85) then flag("PROJECTION_STEP_MISMATCH") end
  if p.steerRateLimited then flag("STEER_RATE_LIMITED") end
  if p.projectionLost then flag("PROJECTION_LOST") end
  if p.overtakeSide and p.overtakeSide~=0 then flag("OVERTAKE_STEERING_CONTEXT") end
  if p.returningFromPass then flag("RETURN_FROM_OVERTAKE") end
  if not controlWriteOk then flag("CONTROL_WRITE_FAILED") end
  if math.abs(safeNumber(p.pursuitSteer))>0.01 then flag("PURE_PURSUIT_ACTIVE") end
  if math.abs(safeNumber(p.boundaryCurvatureSteer))>0.01 then flag("BOUNDARY_CURVATURE_ACTIVE") end
  if math.abs(safeNumber(p.boundarySteer))>0.01 then flag("BOUNDARY_STEER_ACTIVE") end

  local yawTarget=safeNumber(p.steerCurvature)*math.max(0,safeNumber(car.speedMs))
  -- AC's localAngularVelocity.y uses the opposite sign to the X/Z
  -- signed-angle convention used by route curvature and pure pursuit.
  -- Keep the raw physics value for telemetry; align only this comparison.
  local yawActual=p.measuredYawRate~=nil and
    -safeNumber(p.measuredYawRate) or safeNumber(p.yawRate)
  p.yawRateTarget=yawTarget
  p.yawRateError=yawTarget-yawActual
  if safeNumber(car.speedMs)>15 and math.abs(p.yawRateError)>0.35 then flag("YAW_RATE_MISMATCH") end
  p.diagWasSteeringOnStraight=p.roadStraight and math.abs(target)>0.01

  if #flags>0 and p.steerEventCooldown<=0 then
    H.log("STEER_EVENT id=%d flags=%s v=%.2f target=%.4f final=%.4f raw=%.4f pp=%.4f boundK=%.4f bound=%.4f cross=%.3f head=%.4f latV=%.3f yaw=%.4f yawTarget=%.4f routeDelta=%.3f steerProjDelta=%.3f expected=%.3f targetOffset=%.3f targetAlpha=%.5f planGen=%d",
      carIndex,table.concat(flags,"+"),safeNumber(car.speedMs),target,final,
      safeNumber(p.rawSteer),safeNumber(p.pursuitSteer),safeNumber(p.boundaryCurvatureSteer),
      safeNumber(p.boundarySteer),safeNumber(p.crossTrackError),safeNumber(p.headingError),
      safeNumber(p.lateralVelocity),yawActual,yawTarget,progressDelta,steerProjectionDelta,
      expectedProgress,targetOffset,targetAlpha,safeNumber(p.racingPlanGeneration))
    p.steerEventCooldown=0.20
  end

  local interval=0.50
  if p.roadStraight and (math.abs(target)>0.004 or math.abs(final)>0.004) then interval=0.10
  elseif math.abs(target)>0.004 or math.abs(final)>0.004 then interval=0.25 end
  if p.steerDiagTimer>=interval then
    p.steerDiagTimer=p.steerDiagTimer-interval
    H.log("STEER_DIAG id=%d v=%.2f dt=%.3f straight=%d k0=%.6f k15=%.6f k30=%.6f k50=%.6f progress=%.2f projDist=%.3f routeDelta=%.3f steerProjDelta=%.3f lateral=%.3f cross=%.3f head=%.5f latV=%.3f slip=%.4f yawEst=%.4f yawPhys=%.4f yawTarget=%.4f yawErr=%.4f axleF=%.4f axleR=%.4f look=%.1f targetSrc=%s targetXY=(%.2f,%.2f) targetDist=%.2f alpha=%.5f targetOffset=%.3f planGen=%d planMode=%s ppK=%.6f ppSteer=%.5f boundErr=%.4f boundK=%.6f boundKSteer=%.5f boundSteer=%.5f yawFB=%.5f yawErr=%.4f raw=%.5f deadband=%d target=%.5f filtered=%.5f rateDemand=%.5f rateApplied=%.5f rateLimited=%d final=%.5f lane=%.3f raceOffset=%.3f pass=%d returning=%d controlOK=%d",
      carIndex,safeNumber(car.speedMs),dt,p.roadStraight and 1 or 0,
      safeNumber(p.roadCurvature0),safeNumber(p.roadCurvature15),safeNumber(p.roadCurvature30),safeNumber(p.roadCurvature50),
      safeNumber(p.routeProgress),safeNumber(p.lineDist),progressDelta,steerProjectionDelta,
      safeNumber(p.latOff),safeNumber(p.crossTrackError),safeNumber(p.headingError),
      safeNumber(p.lateralVelocity),safeNumber(p.slipAngle),safeNumber(p.yawRate),safeNumber(p.measuredYawRate),
      safeNumber(p.yawRateTarget),safeNumber(p.yawRateError),safeNumber(p.frontAxleSlip),safeNumber(p.rearAxleSlip),
      safeNumber(p.lookahead),tostring(targetSource),safeNumber(p.steeringTargetX),safeNumber(p.steeringTargetZ),
      safeNumber(p.steeringTargetDistance),targetAlpha,targetOffset,safeNumber(p.racingPlanGeneration),
      tostring(p.racingPlanMode or "NONE"),safeNumber(p.pursuitCurvature),safeNumber(p.pursuitSteer),
      safeNumber(p.boundaryError),safeNumber(p.boundaryCurvature),safeNumber(p.boundaryCurvatureSteer),
      safeNumber(p.boundarySteer),safeNumber(p.yawStabilitySteer),
       safeNumber(p.yawRateFeedbackError),safeNumber(p.rawSteer),
       p.deadbandActive and 1 or 0,target,
      safeNumber(p.steerFiltered),safeNumber(p.steerRateLimitDemand),safeNumber(p.steerRateLimitApplied),
      p.steerRateLimited and 1 or 0,final,safeNumber(p.laneOffsetCurrent),safeNumber(p.raceOffsetRaw),
      safeNumber(p.overtakeSide),p.returningFromPass and 1 or 0,controlWriteOk and 1 or 0)
  end
  p.diagPreviousTarget=target
  p.diagPreviousFinal=final
  p.diagPreviousTargetOffset=targetOffset
  p.diagPreviousTargetAlpha=targetAlpha
  p.diagPreviousTargetSource=targetSource
  p.diagPreviousPlanGeneration=p.racingPlanGeneration or 0
end

-- Build one coherent route-projection snapshot for every active AI car before
-- any controller writes controls. This makes traffic decisions independent of
-- the order in which cars are subsequently processed.
function Driver.prepareFrame(sim,dt)
  if not sim then return end
  refreshRouteMetrics()
  state.trafficFrame=(state.trafficFrame or 0)+1
  local frame=state.trafficFrame
  for i=0,sim.carsCount-1 do
    local car=ac.getCar(i)
    if car and car.isActive and car.isAIControlled then
      local p=state.p[i]
      if not p then newPersonality(i);p=state.p[i] end
      local progress,lateral,seg,parameter,distance,tx,tz=
        routeProjection(car,p.segIdx,not p.routeLocked,p.routeProgress,
          math.max(0,safeNumber(car.speedMs)),dt or 0.05)
      p.framePreviousRouteProgress=p.routeProgress
      p.framePreparedFrame=frame
      p.frameProjection={valid=progress~=nil,progress=progress,lateral=lateral,
        seg=seg,parameter=parameter,distance=distance,tx=tx,tz=tz}
      if progress~=nil then
        p.routeLocked=true
        p.routeProgress=progress
        if lateral~=nil then p.latOff=lateral end
        if seg then p.segIdx=seg end
      end
    end
  end
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
  local prepared=p.framePreparedFrame==(state.trafficFrame or -1) and p.frameProjection or nil
  local previousRouteProgress=prepared and p.framePreviousRouteProgress or p.routeProgress
  local progress, projectedLateral, projectedSeg, projectionParameter, projectionDistance
  if prepared then
    if prepared.valid then
      progress,projectedLateral,projectedSeg,projectionParameter,projectionDistance=
        prepared.progress,prepared.lateral,prepared.seg,prepared.parameter,prepared.distance
    end
  else
    progress,projectedLateral,projectedSeg,projectionParameter,projectionDistance=
      routeProjection(car,p.segIdx,not p.routeLocked,p.routeProgress,speed,dt)
  end
  -- A locked projection can fail after a small transient departure. Permit a
  -- topology reacquisition only when the global candidate is still within the
  -- existing 8 m acquisition radius and points in the vehicle's travel
  -- direction; never jump to a distant parallel section.
  if progress==nil and p.routeLocked then
    local rp,rl,rs,_,rd=routeProjection(car,p.segIdx,true,nil,speed,dt)
    if rp~=nil and rd~=nil and rd<=8.0 then
      progress,projectedLateral,projectedSeg,projectionDistance=rp,rl,rs,rd
      H.log("ROUTE_REACQUIRE id=%d distance=%.2f seg=%d",carIndex,rd,rs or -1)
    end
  end
  if p.spawnSeeded and
     (projectionDistance==nil or projectionDistance>8.0) then
    progress=nil
  end
  if progress ~= nil then
    p.frameProgressDelta=previousRouteProgress~=nil and routeGap(previousRouteProgress,progress) or 0
    p.routeLocked = true
    p.routeProgress = progress
    p.latOff = projectedLateral or p.latOff
    p.projectionLost = false
    if projectedSeg then p.segIdx = projectedSeg end
  else
    p.frameProgressDelta=0
    p.projectionLost = true
  end
  readWheelSlip(car,p)

  p.rescueRetryT=math.max(0,safeNumber(p.rescueRetryT)-dt)
  if speed<RESCUE_SPEED then
    p.stuck=p.stuck+dt
  else
    p.stuck=0
  end
  -- A low-speed car in an active traffic transaction is waiting, not stuck.
  -- Recovery is reserved for a genuine no-progress condition with no current
  -- lane leader, no outstanding yield/pass reservation and a valid route plan.
  local trafficWait=p.trafficAction=="WAIT_QUEUE" or
    p.trafficAction=="WAIT_YIELD" or p.trafficAction=="YIELDING" or
    p.pendingYieldTarget~=nil or p.yieldRequestBy~=nil or
    p.overtakeSide~=0 or p.returningFromPass
  local trafficGap=safeNumber(p.trafficGap,-1)
  local nearbyLaneLeader=trafficGap>=0 and
    trafficGap<math.max(30.0,speed*2.0+10.0)
  if p.stuck>=RESCUE_TIME and p.rescueRetryT<=0 and
     not trafficWait and not nearbyLaneLeader and
     not p.projectionLost then
    p.stuck=0
    p.rescueRetryT=5.0
    local rescued=H.teleportHome and H.teleportHome(carIndex,car) or false
    H.log("RESCUE_ATTEMPT id=%d success=%s",carIndex,tostring(rescued))
    if rescued then return end
  end

  if not p.projectionLost then updateRacingPlan(carIndex,car,p,speed,dt) end
  -- A failed optimizer is not a physical obstacle. If route projection is
  -- still valid, steering falls back to the recorded spline and speed planning
  -- uses its curvature instead of commanding a stationary vehicle.
  local targetSpeed = p.projectionLost and 0 or planTargetSpeed(p,speed)
  targetSpeed = updateTrafficControl(carIndex, car, p, speed, targetSpeed, dt)
  -- Overtake state can change in this frame; rebuild the steering path before
  -- issuing controls so the first lateral movement follows the reserved side.
  if not p.projectionLost then updateRacingPlan(carIndex,car,p,speed,dt) end
  p.targetSpeed = targetSpeed

  p.boundaryEmergency=false
  local steerTarget = calculateSteering(car, p, speed, fx, fz, dt)
  local evasiveSteer,evasionBlocked=collisionEvasion(carIndex,car,p,speed)
  p.collisionEvasionActive=evasiveSteer~=nil
  p.collisionEvasionBlocked=evasionBlocked==true
  if evasiveSteer~=nil then steerTarget=evasiveSteer
  elseif not p.collisionEvasionBlocked then
    p.collisionEvasionLogged=false
    p.collisionEvasionTarget=nil
    p.collisionEvasionSide=0
  end
  p.steerTarget = steerTarget
  if p.projectionLost or p.collisionEvasionBlocked then targetSpeed=0 end
  -- Boundary correction is a steering task, not a reason to stop.
  -- The predictive footprint guard already bends the target path back into
  -- the corridor. Do not convert lateral correction magnitude into a speed
  -- cap: that caused repeated full stops even with zero corridor deficit.
  p.targetSpeed=targetSpeed
  local steer = filterSteering(p, steerTarget, dt,p.collisionEvasionActive)

  calculateThrottle(p, speed, targetSpeed, dt)
  if p.projectionLost or p.collisionEvasionBlocked then p.gas=0; p.brake=1 end

  local ok, controls = pcall(ac.overrideCarControls, carIndex)
  if ok and controls then
    controls.combineAxis = false
    controls.steer = steer
    controls.gas = p.gas
    controls.brake = p.brake
    controls.handbrake = 0
  end
  p.controlWriteOk=ok and controls~=nil
  logSteeringDiagnostics(carIndex,car,p,dt,p.controlWriteOk)
  logDynamicsTrace(carIndex,car,p,dt,p.controlWriteOk)
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
  p.trafficPriority=0
  p.trafficAction="PLAYER"
  p.trafficReason="manual_control"
  p.trafficLeaderIndex=-1
  p.trafficGap=-1
  p.trafficLeaderSpeed=0
  p.trafficTargetOffset=safeNumber(p.latOff)
  p.pace=0
  p.player=true

  -- Manual player driving does not pass through Driver.update(), so the
  -- AI-only steering diagnostics above would otherwise never run for CAR=0.
  -- Keep the ordinary one-second CAR= snapshot unchanged and emit a separate,
  -- higher-rate observation record when the Dev switch is enabled.
  if state.steeringDiagnostics then
    local diagDt=math.max(0,safeNumber(dt,0.016))
    p.playerDiagTimer=(p.playerDiagTimer or 0)+diagDt
    local localVelocity=car.localVelocity
    local angular=car.localAngularVelocity
    local velocity=car.velocity
    local speedAbs=math.abs(safeNumber(car.speedMs))
    local slip=math.abs(safeNumber(p.slipAngle))
    local lateral=math.abs(safeNumber(p.lateralVelocity))
    local yaw=math.abs(safeNumber(p.measuredYawRate))
    local unstable=slip>0.035 or lateral>0.8 or yaw>0.20 or
      math.abs(safeNumber(steerInput))>0.05
    local interval=unstable and 0.05 or 0.10
    if p.playerDiagTimer+1e-6>=interval then
      p.playerDiagTimer=p.playerDiagTimer-interval
      if p.playerDiagTimer>interval then p.playerDiagTimer=0 end
      H.log("PLAYER_DYN_TRACE id=0 v=%.3f dt=%.4f pos=(%.3f,%.3f,%.3f) look=(%.5f,%.5f,%.5f) worldV=(%.4f,%.4f,%.4f) localV=(%.4f,%.4f,%.4f) omega=(%.5f,%.5f,%.5f) route=%.3f lateral=%.4f cross=%.4f head=%.5f roadK=%.6f slip=%.6f latV=%.5f yawLook=%.5f yawPhys=%.5f wheelSlip=%.5f steerInput=%.5f gas=%.4f brake=%.4f projection=%d",
        speedAbs,diagDt,
        safeNumber(car.position and car.position.x),safeNumber(car.position and car.position.y),
        safeNumber(car.position and car.position.z),
        safeNumber(car.look and car.look.x),safeNumber(car.look and car.look.y),
        safeNumber(car.look and car.look.z),
        safeNumber(velocity and velocity.x),safeNumber(velocity and velocity.y),
        safeNumber(velocity and velocity.z),
        safeNumber(localVelocity and localVelocity.x),safeNumber(localVelocity and localVelocity.y),
        safeNumber(localVelocity and localVelocity.z),
        safeNumber(angular and angular.x),safeNumber(angular and angular.y),
        safeNumber(angular and angular.z),
        safeNumber(p.routeProgress),safeNumber(p.latOff),safeNumber(p.crossTrackError),
        safeNumber(p.headingError),safeNumber(p.pathCurvature),safeNumber(p.slipAngle),
        safeNumber(p.lateralVelocity),safeNumber(p.yawRate),safeNumber(p.measuredYawRate),
        safeNumber(p.wheelSlip),safeNumber(steerInput),safeNumber(car.gas),safeNumber(car.brake),
        p.routeLocked and 1 or 0)
    end
  end
  return p
end

function Driver.telemetryLine(i, car, p)
  local bl, br = H.boundsAt(p.segIdx)
  local steerLimit=p.player and 0.99 or (CFG.STEER_LIMIT-0.01)
  local mode=p.player and " MODE=PLAYER" or
    (p.playerAutopilot and " MODE=AI_AUTOPILOT" or "")
  return string.format(
    "CAR=%d POS=(%.2f,%.2f,%.2f) SPEED=%.2f STEER=%.3f GAS=%.3f BRAKE=%.3f TARGET_SPEED=%.2f LINE_ERROR=%.2f SEGMENT=%d PACE=%.2f BOUNDS=[%.1f,%.1f] OFF=%.1f OT_SIDE=%d CROSS=%.2f HEAD=%.3f CURVE=%.4f ROUTE=%.1f LANE=%.2f SAT=%d WHEEL_SLIP=%.3f TC_SCALE=%.2f STEER_CURV=%.4f BOUNDARY=%.2f SAFE=[%.2f,%.2f] BODY=%.2f DEFICIT=%.2f BRAKE_ACC=%.2f DRIVE_ACC=%.2f GRIP=%.2f STUCK=%.1f AXLE_VALID=%d FRONT_AXLE_RAD=%.3f REAR_AXLE_RAD=%.3f FRONT_TIRE_DEG=%.2f REAR_TIRE_DEG=%.2f FRONT_NDSLIP=%.3f REAR_NDSLIP=%.3f YAW_RATE_RAD=%.3f WHEELBASE_M=%.2f STEER_LOCK_DEG=%.1f STEER_TARGET=%.3f SLIP_ANGLE_DEG=%.2f YAW_EST_RAD=%.3f PASS_BLOCKED=%.1f PASS_TIME=%.1f LOOKAHEAD=%.1f PP_CURV=%.5f BOUND_STEER=%.4f LAT_VEL=%.3f PREVIEW_OFFSET=%.2f PLAN_BLOCKED=%d PLAN_BLOCK_DIST=%.1f TRAFFIC_STATE=%s TRAFFIC_TARGET=%d TRAFFIC_GAP=%.1f TRAFFIC_LEADER_SPEED=%.1f TRAFFIC_PRIORITY=%.0f TRAFFIC_OFFSET=%.2f TRAFFIC_REASON=%s%s",
    i, car.position.x, car.position.y, car.position.z,
    safeNumber(car.speedMs), p.steer, p.gas, p.brake, p.targetSpeed,
    p.lineDist, p.segIdx, (p.pace or 0), bl, br, p.latOff, p.overtakeSide,
    p.crossTrackError or 0, p.headingError or 0, p.pathCurvature or 0,
    p.routeProgress or 0, p.laneOffsetCurrent or 0,
    math.abs(p.steer or 0) >= steerLimit and 1 or 0,
    safeNumber(p.wheelSlip), safeNumber(p.tractionScale,1),
    safeNumber(p.steerCurvature), safeNumber(p.boundaryError),
    safeNumber(p.boundarySafeMin), safeNumber(p.boundarySafeMax),
    safeNumber(p.boundaryEnvelope),safeNumber(p.boundaryDeficit),
    safeNumber(p.brakeAccelEstimate,CFG.BRAKE_ACCEL),
    safeNumber(p.accelEstimate,0),safeNumber(p.lateralAccelLimit,CFG.MAX_LATERAL_ACCEL),
    safeNumber(p.stuck),
    p.vehicleDynamicsValid and 1 or 0,
    safeNumber(p.frontAxleSlip), safeNumber(p.rearAxleSlip),
    safeNumber(p.frontTireSlipDeg), safeNumber(p.rearTireSlipDeg),
    safeNumber(p.frontNdSlip), safeNumber(p.rearNdSlip),
    safeNumber(p.measuredYawRate), safeNumber(p.measuredWheelbase),
    safeNumber(p.measuredSteerLock),
    safeNumber(p.steerTarget), math.deg(safeNumber(p.slipAngle)),
    safeNumber(p.yawRate), safeNumber(p.passBlockedT), safeNumber(p.passT),
    safeNumber(p.lookahead),safeNumber(p.pursuitCurvature),
    safeNumber(p.boundarySteer),safeNumber(p.lateralVelocityFiltered),
    safeNumber(p.raceOffsetRaw),
    p.racingPlanBlocked and 1 or 0,
    safeNumber(p.racingPlanBlockedDistance),
    tostring(p.trafficAction or "UNKNOWN"),
    safeNumber(p.trafficLeaderIndex,-1),safeNumber(p.trafficGap,-1),
    safeNumber(p.trafficLeaderSpeed),safeNumber(p.trafficPriority),
    safeNumber(p.trafficTargetOffset),tostring(p.trafficReason or "unknown"),mode)
end

return Driver