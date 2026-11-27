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
local RESCUE_SPEED = 1.5
local RESCUE_TIME  = 10.0
local OFFTRACK_TIME = 10.0
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

local function routeProjection(car, hintSeg, allowGlobal, hintProgress, speed, dt, recovery)
  refreshRouteMetrics()
  if not state.routeGeometry then return nil end
  local locked=allowGlobal ~= true and hintProgress ~= nil
  local projectionSpeed=speed or safeNumber(car.speedMs)
  local projectionDt=dt or 0.05
  local result = Geometry.project(state.routeGeometry, car.position,
    hintProgress, car.look, projectionSpeed, projectionDt, locked)
  if not result and locked then
    -- A temporary local-window miss must not turn into a full-brake command.
    -- Reacquire globally only when the candidate is nearby, heading-compatible
    -- and topologically close to the last known route progress.
    local candidate=Geometry.project(state.routeGeometry,car.position,nil,
      car.look,projectionSpeed,projectionDt,false)
    if candidate and candidate.distance<=12.0 then
      local hx,hz=normalize2(car.look.x,car.look.z)
      local alignment=hx*candidate.tx+hz*candidate.tz
      local delta=candidate.s-hintProgress
      if state.loopRoute then
        local total=state.routeLength or 0
        if total>0 and delta>total*0.5 then delta=delta-total end
        if total>0 and delta< -total*0.5 then delta=delta+total end
      end
      local continuityLimit=math.max(15.0,projectionSpeed*projectionDt*4.0+8.0)
      if alignment>=-0.05 and math.abs(delta)<=continuityLimit then
        result=candidate
      end
    end
  end
  if not result then return nil end
  if recovery and safeNumber(result.distance,math.huge)>100.0 then return nil end
  if not locked and not recovery and not initialProjectionValid(result,car.look) then return nil end
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
    stuck = 0, blockedT = 0,
    lineDist = 0, latOff = 0, targetSpeed = 0,
    overtakeSide = 0, passTargetIndex = nil, retireT = 2.0,
    trafficPriority = 0, coordinatedLane = nil, homeLane = nil,
    coordinatorOffset = nil, coordinatorTargetLane = nil,
    coordinatorFromLane = nil, coordinatorChangeT = 0,
    yieldingFor = nil, coordinatorReturn = false,
    emergencyEscape = false, emergencyTargetOffset = nil,
    emergencyObstacleIndex = nil, emergencySteerFullLock = false,
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
  local recovery=p.offTrackRecovering==true
  local recoveryHint=p.routeProgress
  local allowGlobal=not recovery and not locked
  if recoveryHint==nil then allowGlobal=true end
  local ps,pl,pg,pt,pd,tx,tz=routeProjection(
    car,p.segIdx,allowGlobal,recoveryHint,speed,dt,recovery)
  local projection=ps and {s=ps,lateral=pl,seg=pg,t=pt,distance=pd,tx=tx,tz=tz} or nil
  if not projection or (p.spawnSeeded and projection.distance>8.0) then
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
  p.baseLaneOffset=safeNumber(p.coordinatorOffset,
    p.baseLaneOffset or roadCenter)
  p.laneOffset=p.baseLaneOffset
  local laneTarget=p.laneOffset or roadCenter
  local corridorMin,corridorMax=safeCorridorForPreview(
    route,projection.s,ahead,car,fx,fz)
  if corridorMin and corridorMax then
    laneTarget=clamp(laneTarget,corridorMin,corridorMax)
  end
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
    local maxStep=1.8*math.max(0,dt or 0)
    local previousOffset=p.laneOffsetCurrent or laneTarget
    p.laneOffsetCurrent=previousOffset+
      clamp(laneTarget-previousOffset,-maxStep,maxStep)
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
  if p.emergencyEscape and p.emergencyTargetOffset~=nil then
    local escapeTarget=clamp(p.emergencyTargetOffset,
      corridorMin or safeMin,corridorMax or safeMax)
    p.emergencyTargetOffset=escapeTarget
    local escapeError=escapeTarget-projection.lateral
    local escapeDirection=escapeError>=0 and 1 or -1
    local towardSpeed=lateralVelocity*escapeDirection
    if math.abs(escapeError)<=0.45 then
      p.emergencyEscape=false
      p.emergencyTargetOffset=nil
      p.emergencyObstacleIndex=nil
      p.emergencySteerFullLock=false
      p.emergencyUntil=0
      p.racingPlan=nil
      p.racingPlanTimer=0
    else
      -- Continuous lateral position/velocity controller. The previous
      -- 1.0 m/s hard switch alternated between full lock and normal steering
      -- whenever lateral speed crossed the threshold, causing steering kicks.
      -- Proportional position demand plus velocity damping gives one smooth,
      -- progressively reducing steering command throughout the escape.
      rawSteer=clamp(escapeError*0.32-lateralVelocity*0.12,-0.62,0.62)
      p.emergencySteerFullLock=false
    end
  else
    p.emergencySteerFullLock=false
  end
  p.rawSteer=rawSteer
  p.deadbandActive=deadbandActive
  return clamp(rawSteer,-CFG.STEER_LIMIT,CFG.STEER_LIMIT)
end

local function filterSteering(p, target, dt)
  local alpha = clamp(dt / (CFG.STEER_FILTER + dt), 0, 1)
  p.steerFiltered = lerp(p.steerFiltered, target, alpha)
  -- Emergency evasion temporarily increases steering slew rate; the normal
  -- steering target, stability limits and absolute steering limit remain active.
  local emergencyActive=safeNumber(p.emergencyUntil)>0
  if emergencyActive then
    p.emergencyUntil=math.max(0,safeNumber(p.emergencyUntil)-dt)
  end
  local maxChange=CFG.STEER_RATE*dt*
    ((p.emergencyEscape or emergencyActive) and 1.8 or 1.0)
  local requestedChange=p.steerFiltered-p.steer
  local appliedChange=clamp(requestedChange,-maxChange,maxChange)
  p.steerRateLimited=math.abs(requestedChange)>maxChange+1e-6
  p.steerRateLimitDemand=requestedChange
  p.steerRateLimitApplied=appliedChange
  p.steer = clamp(p.steer+appliedChange,-CFG.STEER_LIMIT,CFG.STEER_LIMIT)
  return p.steer
end

local function relativeToRoute(position, sample)
  if not position or not sample then return nil, nil end
  local dx=safeNumber(position.x)-sample.x
  local dz=safeNumber(position.z)-sample.z
  local longitudinal=dx*sample.tx+dz*sample.tz
  local lateral=dx*(-sample.tz)+dz*sample.tx
  return longitudinal,lateral
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
        if gap and actualLateral and gap>-halfLengths-2.0 and
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
  if p.offTrackRecovering then
    p.racingPlan=nil
    p.racingPlanBlocked=false
    p.racingPlanTrafficBlocked=false
    p.racingPlanBlockedDistance=nil
    p.racingPlanTargetOffset=nil
    return
  end
  if p.emergencyEscape then
    p.racingPlan=nil
    p.racingPlanBlocked=false
    p.racingPlanTrafficBlocked=false
    p.racingPlanBlockedDistance=nil
    p.racingPlanTargetOffset=nil
    return
  end
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
  -- The coordinator supplies a discrete lane centre for every managed car.
  -- The trajectory planner smooths the transition but does not choose lanes.
  local base=safeNumber(p.coordinatorOffset,p.baseLaneOffset or mid)
  p.baseLaneOffset=base
  p.laneOffset=base
  p.racingPlanTimer=math.max(0,safeNumber(p.racingPlanTimer)-dt)
  local mode="LANE"
  local fixedOffset=base
  local fixedTransitionDistance=22.0
  if p.racingPlanTimer>0 and p.racingPlanMode==mode and
      p.racingPlanTargetOffset~=nil and
      math.abs(p.racingPlanTargetOffset-fixedOffset)<0.05 then
    return
  end
  if fixedOffset~=nil then
    local lateralDelta=math.abs(fixedOffset-safeNumber(p.latOff,base))
    local lateralLimit=clamp(
      safeNumber(p.lateralAccelLimit,CFG.MAX_LATERAL_ACCEL)*0.65,4.0,12.0)
    fixedTransitionDistance=math.max(
      safeNumber(p.emergencyUntil)>0 and 4.0 or 22.0,
      math.max(0,speed)*math.sqrt(6.0*lateralDelta/lateralLimit)*
        (safeNumber(p.emergencyUntil)>0 and 0.45 or 1.0))
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
  -- Fixed lane offset is the preferred path; the optimiser may smooth its
  -- geometry but cannot move the car into another logical lane.
  local preferredOffset=base
  nextPlan,planFailure=RacingLine.plan(
    state.routeGeometry,Geometry.sample,p.routeProgress,car,routeBoundsAtSample,
    safeNumber(p.latOff,base),preferredOffset,fixedOffset,horizon,7.5,
    previousPlan,previousAdvance,fixedTransitionDistance,p.lateralAccelLimit)

  -- A failed dynamic-programming search is not proof that the road is blocked.
  -- The controller falls back to the recorded route spline and the lane target
  -- is clamped to the full-body safe corridor in calculateSteering().
  local originalFailure=planFailure or (not nextPlan and "NO_PLAN" or nil)
  local fallbackKind=nil
  if not nextPlan then
    -- The spline follower below is the deterministic fallback. An optimiser
    -- search failure is not evidence that the road is blocked and must never
    -- command a stationary car on a verified route.
    fallbackKind="ROUTE_SPLINE"
    planFailure=planFailure or originalFailure or "NO_PLAN"
  end

  if nextPlan ~= p.racingPlan then
    p.racingPlanGeneration = (p.racingPlanGeneration or 0) + 1
  end
  p.racingPlan=nextPlan
  p.racingPlanStartProgress=p.routeProgress
  -- Search failure falls back to the recorded route spline. Only NO_FIT means
  -- the vehicle physically cannot fit the immediate corridor and may stop.
  p.racingPlanBlocked=nextPlan==nil and planFailure=="NO_FIT"
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
  p.racingPlanTargetOffset=fixedOffset
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

local function updateTrafficControl(carIndex,car,p,speed,targetSpeed,dt)
  local laneOffset=safeNumber(p.coordinatorOffset,p.baseLaneOffset or p.latOff or 0)
  p.trafficTargetOffset=laneOffset
  local gap,leaderSpeed,leaderIndex,leaderLateral=
    carAheadInLane(carIndex,car,laneOffset)
  p.trafficLeaderIndex=leaderIndex or -1
  p.trafficGap=gap or -1
  p.trafficLeaderSpeed=leaderSpeed or 0
  if p.emergencyEscape then
    p.trafficAction="EMERGENCY_ESCAPE"
    p.trafficReason="physical_lateral_clearance"
    p.trafficPriority=1100000
    -- FOLLOW braking against the obstacle being escaped would prevent any
    -- lateral movement. Keep a controlled crawl until the target line is met.
    return clamp(math.max(0,safeNumber(gap,8.0))*0.55,2.5,5.0)
  end
  if p.passTargetIndex~=nil then
    p.trafficAction="PASSING"
    p.trafficReason="coordinator_lane_reserved"
    p.trafficPriority=1000000
  elseif p.yieldingFor~=nil then
    p.trafficAction="YIELDING"
    p.trafficReason="proactive_lane_release"
    p.trafficPriority=900000
  else
    p.trafficAction=gap and "FOLLOW" or "OPEN"
    p.trafficReason=gap and "same_lane_vehicle" or "assigned_lane_clear"
    p.trafficPriority=0
  end
  if gap~=nil then
    local safeSpeed=safeFollowingSpeed(gap,leaderSpeed or 0,p.brakeAccelEstimate)
    targetSpeed=math.min(targetSpeed,safeSpeed)
  end
  -- Keep the coordinator's no-slot decision authoritative even if the
  -- ordinary lane-based detector misses an off-line or partially displaced car.
  if p.coordinatorConflictSpeed~=nil then
    targetSpeed=math.min(targetSpeed,p.coordinatorConflictSpeed)
    p.trafficAction="CONFLICT_BRAKE"
    p.trafficReason="no_safe_lane_reserved"
  end
  return targetSpeed
end

-- One deterministic traffic pass runs before any car receives controls.
-- Lane 1 is the rightmost slot (negative lateral offset); the highest slot
-- is the leftmost passing lane. Every slot is derived from the recorded bounds.
local function coordinatorLaneGeometry(seg,car)
  local left,right=H.boundsAt(seg)
  local width=right-left
  if width<=0.1 then return {left=(left+right)*0.5,right=(left+right)*0.5,
    count=1,centers={(left+right)*0.5}} end
  local count=clamp(math.floor((width+1.0)/4.0),1,4)
  local carWidth=math.max(1.5,safeNumber(car and car.aabbSize and car.aabbSize.x,2.0))
  while count>1 and width/count<carWidth+0.55 do count=count-1 end
  local slotWidth=width/count
  local centers={}
  for lane=1,count do centers[lane]=left+(lane-0.5)*slotWidth end
  return {left=left,right=right,count=count,slotWidth=slotWidth,centers=centers}
end

local function coordinatorGap(a,b)
  if not a or not b or a.p.routeProgress==nil or b.p.routeProgress==nil then return nil end
  local ownSample=Geometry.sample(state.routeGeometry,a.p.routeProgress)
  if not ownSample then return nil end
  local bx,bz=normalize2(b.car.look.x,b.car.look.z)
  local directionDot=bx*ownSample.tx+bz*ownSample.tz
  if directionDot>=-0.25 then
    return routeGap(a.p.routeProgress,b.p.routeProgress)
  end
  local longitudinal=relativeToRoute(b.car.position,ownSample)
  return longitudinal
end

local function coordinatorSameDirection(a,b)
  local sample=Geometry.sample(state.routeGeometry,a.p.routeProgress)
  if not sample then return true end
  local fx,fz=normalize2(b.car.look.x,b.car.look.z)
  return fx*sample.tx+fz*sample.tz>=-0.25
end

local function coordinatorLaneFree(owner,targetLane,records,window)
  local center=owner.lanes.centers[targetLane]
  if center==nil then return false end
  for _,other in ipairs(records) do
    if other.index~=owner.index and other.p.routeProgress~=nil then
      local gap=coordinatorGap(owner,other)
      if gap~=nil then
        local otherHalf=math.max(1.5,
          safeNumber(other.car.aabbSize and other.car.aabbSize.z,4.5)*0.5)
        local ownHalf=math.max(1.5,
          safeNumber(owner.car.aabbSize and owner.car.aabbSize.z,4.5)*0.5)
        local ownLateralHalf=math.max(0.75,
          safeNumber(owner.car.aabbSize and owner.car.aabbSize.x,2.0)*0.5)
        local otherLateralHalf=math.max(0.75,
          safeNumber(other.car.aabbSize and other.car.aabbSize.x,2.0)*0.5)
        local targetLateralOverlap=math.abs(center-
          safeNumber(other.p.latOff,center))<
          ownLateralHalf+otherLateralHalf+0.35
        local lateralDelta=math.abs(center-
          safeNumber(owner.p.latOff,owner.p.coordinatorOffset or center))
        local lateralLimit=clamp(
          safeNumber(owner.p.lateralAccelLimit,CFG.MAX_LATERAL_ACCEL)*0.65,4.0,12.0)
        local transitionDistance=math.max(22.0,
          owner.speed*math.sqrt(6.0*lateralDelta/lateralLimit))
        local transitionTime=transitionDistance/math.max(8.0,owner.speed)
        local dynamicWindow=math.max(window or 8.0,
          ownHalf+otherHalf+math.abs(owner.speed-other.speed)*transitionTime+5.0)
        local otherLane=other.p.coordinatedLane
        if (otherLane==targetLane or targetLateralOverlap) and
            math.abs(gap)<dynamicWindow then
          return false
        end
        -- Reserve both the destination and the lane being vacated until the
        -- full vehicle body has completed its lateral transition.
        if other.p.coordinatorFromLane==targetLane and
            math.abs(gap)<dynamicWindow+4.0 then return false end
        if other.p.coordinatorTargetLane==targetLane and
            math.abs(gap)<dynamicWindow+4.0 then return false end
      end
    end
  end
  return true
end

-- Emergency lane checks use predicted collision time, not the normal
-- reservation horizon. A distant car travelling away from the escape slot
-- must not make a physically empty adjacent lane appear occupied.
local function coordinatorEmergencyLaneFree(owner,targetLane,records)
  local center=owner.lanes.centers[targetLane]
  if center==nil then return false,"NO_LANE_GEOMETRY" end
  local ownHalf=math.max(1.5,
    safeNumber(owner.car.aabbSize and owner.car.aabbSize.z,4.5)*0.5)
  local ownLatHalf=math.max(0.75,
    safeNumber(owner.car.aabbSize and owner.car.aabbSize.x,2.0)*0.5)
  local lateralDelta=math.abs(center-safeNumber(owner.p.latOff,center))
  local lateralAccel=clamp(
    safeNumber(owner.p.lateralAccelLimit,CFG.MAX_LATERAL_ACCEL)*0.45,2.0,8.0)
  local transitionTime=clamp(math.sqrt(2.0*math.max(0.15,lateralDelta)/lateralAccel),
    0.65,2.5)
  for _,other in ipairs(records) do
    if other.index~=owner.index and other.p.routeProgress~=nil then
      local gap=coordinatorGap(owner,other)
      if gap~=nil then
        local otherHalf=math.max(1.5,
          safeNumber(other.car.aabbSize and other.car.aabbSize.z,4.5)*0.5)
        local otherLatHalf=math.max(0.75,
          safeNumber(other.car.aabbSize and other.car.aabbSize.x,2.0)*0.5)
        local physicalOverlap=math.abs(center-safeNumber(other.p.latOff,center))<
          ownLatHalf+otherLatHalf+0.35
        local reserved=(other.p.coordinatedLane==targetLane or
          other.p.coordinatorFromLane==targetLane or
          other.p.coordinatorTargetLane==targetLane)
        if physicalOverlap or (other.managed and reserved) then
          local minGap=ownHalf+otherHalf+3.0
          local closing
          local sameDirection=coordinatorSameDirection(owner,other)
          if sameDirection then
            closing=gap>=0 and owner.speed-other.speed or other.speed-owner.speed
          else
            closing=owner.speed+other.speed
          end
          local timeToConflict=math.huge
          if math.abs(gap)<=minGap then
            timeToConflict=0
          elseif closing>0.5 then
            timeToConflict=math.max(0,math.abs(gap)-minGap)/closing
          end
          if timeToConflict<=transitionTime+0.65 then
            return false,string.format("PREDICTED_OCCUPANCY_%d",other.index),
              gap,closing,timeToConflict
          end
        end
      end
    end
  end
  return true
end

local function coordinatorEmergencyMove(record,targetLane,records,otherIndex)
  local p=record.p
  local current=p.coordinatedLane or 1
  if targetLane<1 or targetLane>record.lanes.count or targetLane==current then
    return false
  end
  local laneFree,rejectReason,rejectGap,rejectClosing,rejectTTC=
    coordinatorEmergencyLaneFree(record,targetLane,records)
  if not laneFree then
    H.log("TRAFFIC_EMERGENCY_LANE_REJECT id=%d lane=%d reason=%s gap=%.2f closing=%.2f ttc=%.2f",
      record.index,targetLane,tostring(rejectReason),safeNumber(rejectGap,999),
      safeNumber(rejectClosing),safeNumber(rejectTTC,999))
    return false
  end
  p.coordinatorFromLane=current
  p.coordinatorTargetLane=targetLane
  p.coordinatedLane=targetLane
  p.coordinatorOffset=record.lanes.centers[targetLane]
  p.baseLaneOffset=p.coordinatorOffset
  p.laneOffset=p.coordinatorOffset
  -- A separate physical escape controller owns steering until the adjacent
  -- lane is reached; ordinary racing-line smoothing must not cancel it.
  p.emergencyEscape=true
  p.emergencyTargetOffset=p.coordinatorOffset
  p.emergencyObstacleIndex=otherIndex
  p.emergencySteerFullLock=false
  p.coordinatorChangeT=0.10
  p.emergencyUntil=1.25
  p.emergencyBlockedT=0
  p.emergencyRescueReady=false
  p.passTargetIndex=nil
  p.yieldingFor=nil
  p.coordinatorReturn=false
  p.overtakeSide=targetLane>current and 1 or -1
  p.racingPlanTimer=0
  p.racingPlanTargetOffset=nil
  p.racingPlan=nil
  H.log("TRAFFIC_EMERGENCY_EVASION id=%d from=%d to=%d obstacle=%d speed=%.2f",
    record.index,current,targetLane,otherIndex or -1,record.speed)
  return true
end

local function coordinatorMove(record,targetLane,records,reason,otherIndex)
  -- Emergency escape owns its lane reservation until the physical manoeuvre
  -- finishes. Ordinary PASS/YIELD/RETURN must never overwrite that decision.
  if record.p.emergencyEscape then return false end
  local current=record.p.coordinatedLane or 1
  if targetLane<1 or targetLane>record.lanes.count or
      math.abs(targetLane-current)~=1 then return false end
  if not coordinatorLaneFree(record,targetLane,records,8.0) then return false end
  record.p.coordinatorFromLane=current
  record.p.coordinatorTargetLane=targetLane
  record.p.coordinatedLane=targetLane
  record.p.coordinatorOffset=record.lanes.centers[targetLane]
  record.p.baseLaneOffset=record.p.coordinatorOffset
  record.p.laneOffset=record.p.coordinatorOffset
  record.p.coordinatorChangeT=3.0
  if reason=="PASS" then
    record.p.passTargetIndex=otherIndex
    record.p.yieldingFor=nil
    record.p.coordinatorReturn=false
    record.p.overtakeSide=targetLane>current and 1 or -1
  elseif reason=="YIELD" then
    record.p.passTargetIndex=nil
    record.p.yieldingFor=otherIndex
    record.p.coordinatorReturn=false
    record.p.overtakeSide=0
  else
    record.p.passTargetIndex=nil
    record.p.yieldingFor=nil
    record.p.overtakeSide=0
  end
  record.p.racingPlanTimer=0
  record.p.racingPlanTargetOffset=nil
  H.log("TRAFFIC_LANE_CHANGE id=%d from=%d to=%d reason=%s other=%d",
    record.index,current,targetLane,reason,otherIndex or -1)
  return true
end

function Driver.prepareTraffic(sim,driveOrder,dt)
  refreshRouteMetrics()
  if not sim or not state.routeGeometry then return end
  local records={}
  local managedSet={}
  for _,index in ipairs(driveOrder or {}) do
    managedSet[index]=true
  end
  for _,index in ipairs(driveOrder or {}) do
    local car=ac.getCar(index)
    if car and car.isActive then
      local p=state.p[index]
      if not p then newPersonality(index);p=state.p[index] end
      local speed=math.max(0,safeNumber(car.speedMs))
      local progress,lateral,seg=routeProjection(car,p.segIdx,
        not p.routeLocked,p.routeProgress,speed,dt)
      if progress~=nil then
        p.routeProgress=progress;p.routeLocked=true
        p.segIdx=seg or p.segIdx;p.latOff=lateral or p.latOff
        p.projectionLost=false
        if p.coordinatorFromLane and
            math.abs((lateral or p.latOff or 0)-
              safeNumber(p.coordinatorOffset,0))<0.30 then
          p.coordinatorFromLane=nil
        end
        local lanes=coordinatorLaneGeometry(p.segIdx,car)
        if p.coordinatedLane~=nil then
          p.coordinatedLane=clamp(p.coordinatedLane,1,lanes.count)
          p.homeLane=clamp(p.homeLane or p.coordinatedLane,1,lanes.count)
          p.coordinatorOffset=lanes.centers[p.coordinatedLane]
          if p.emergencyEscape then
            p.emergencyTargetOffset=p.coordinatorOffset
          end
        end
        p.coordinatorLaneCount=lanes.count
        p.coordinatorChangeT=math.max(0,safeNumber(p.coordinatorChangeT)-dt)
        records[#records+1]={index=index,car=car,p=p,lanes=lanes,
          speed=speed,accel=clamp(safeNumber(p.accelEstimate,4.0),0.5,18.0),
          managed=true}
      else
        p.projectionLost=true
        p.trafficAction="ROUTE_LOST"
        p.trafficReason="coordinator_projection_failed"
      end
    end
  end
  -- Include the manually controlled player and any non-managed active car as
  -- read-only obstacles. The coordinator may route AI around them, never move them.
  for index=0,sim.carsCount-1 do
    if not managedSet[index] then
      local car=ac.getCar(index)
      if car and car.isActive then
        local progress,lateral,seg=routeProjection(car,1,true,nil,
          math.max(0,safeNumber(car.speedMs)),dt)
        if progress~=nil then
          local lanes=coordinatorLaneGeometry(seg or 1,car)
          local lane,best=1,math.huge
          for n=1,lanes.count do
            local error=math.abs(lanes.centers[n]-(lateral or 0))
            if error<best then best=error;lane=n end
          end
          records[#records+1]={index=index,car=car,
            p={routeProgress=progress,routeLocked=true,segIdx=seg or 1,
              latOff=lateral or 0,coordinatedLane=lane,
              coordinatorTargetLane=lane},
            lanes=lanes,speed=math.max(0,safeNumber(car.speedMs)),
            accel=0,managed=false}
        end
      end
    end
  end
  if #records==0 then return end
  -- Clear the per-frame coordinator safety cap before resolving this frame.
  for _,rec in ipairs(records) do
    if rec.managed then rec.p.coordinatorConflictSpeed=nil end
  end

  -- Initialise the field by measured acceleration rank, then place nearby
  -- cars in the least-conflicted slot. This is not tied to car IDs or spawn order.
  local unassigned={}
  for _,rec in ipairs(records) do
    if rec.managed and rec.p.coordinatedLane==nil then
      unassigned[#unassigned+1]=rec
    end
  end
  table.sort(unassigned,function(a,b)
    if math.abs(a.accel-b.accel)>0.08 then return a.accel>b.accel end
    return a.p.routeProgress<b.p.routeProgress
  end)
  local assigned={}
  for _,rec in ipairs(records) do
    if not rec.managed or rec.p.coordinatedLane~=nil then
      assigned[#assigned+1]=rec
    end
  end
  for rank,rec in ipairs(unassigned) do
    local preferred=rec.lanes.count
    if #unassigned>1 then
      preferred=clamp(rec.lanes.count-
        math.floor((rank-1)*rec.lanes.count/#unassigned),1,rec.lanes.count)
    end
    local bestLane,bestCost=preferred,math.huge
    for lane=1,rec.lanes.count do
      local nearby=0
      for _,other in ipairs(assigned) do
        if other.p.coordinatedLane==lane then
          local gap=coordinatorGap(rec,other)
          if gap and math.abs(gap)<32.0 then nearby=nearby+1 end
        end
      end
      local cost=nearby*100.0+math.abs(lane-preferred)*1.5
      if lane==rec.p.coordinatedLane then cost=cost-0.5 end
      if cost<bestCost then bestCost=cost;bestLane=lane end
    end
    rec.p.coordinatedLane=bestLane
    rec.p.homeLane=bestLane
    rec.p.coordinatorOffset=rec.lanes.centers[bestLane]
    rec.p.baseLaneOffset=rec.p.coordinatorOffset
    rec.p.laneOffset=rec.p.coordinatorOffset
    rec.p.coordinatorTargetLane=bestLane
    assigned[#assigned+1]=rec
    H.log("TRAFFIC_LANE_INIT id=%d lane=%d/%d accel=%.2f",
      rec.index,bestLane,rec.lanes.count,rec.accel)
  end

  -- Existing assignments are retained unless a real predicted conflict exists.
  -- Resolve the closest/highest-closing pairs first so both cars see one decision.
  table.sort(records,function(a,b)
    if a.p.routeProgress~=b.p.routeProgress then
      return a.p.routeProgress<b.p.routeProgress
    end
    return a.index<b.index
  end)
  local handled={}
  for i=1,#records do
    local a=records[i]
    for j=i+1,#records do
      local b=records[j]
      local gap=coordinatorGap(a,b)
      if gap~=nil then
        local follower,leader,distance
        if gap>=0 then follower,leader,distance=a,b,gap
        else follower,leader,distance=b,a,-gap end
        local closing=follower.speed-leader.speed+
          math.max(0.0,follower.accel-leader.accel)*0.35
        local followerHalf=math.max(1.5,
          safeNumber(follower.car.aabbSize and follower.car.aabbSize.z,4.5)*0.5)
        local leaderHalf=math.max(1.5,
          safeNumber(leader.car.aabbSize and leader.car.aabbSize.z,4.5)*0.5)
        local slotWidth=math.max(2.8,safeNumber(follower.lanes.slotWidth,3.5))
        local function transitionTime(rec)
          local limit=clamp(
            safeNumber(rec.p.lateralAccelLimit,CFG.MAX_LATERAL_ACCEL)*0.65,4.0,12.0)
          local distance=math.max(22.0,rec.speed*math.sqrt(6.0*slotWidth/limit))
          return distance/math.max(8.0,rec.speed)
        end
        local maneuverTime=math.max(transitionTime(follower),transitionTime(leader))
        local trigger=followerHalf+leaderHalf+
          math.max(5.0,math.max(0,closing)*maneuverTime+5.0)
        local sameDirection=coordinatorSameDirection(follower,leader)
        local sameLane=follower.p.coordinatedLane==leader.p.coordinatedLane
        local conflictDistance=sameDirection and distance<trigger or
          (not sameDirection and math.abs(gap)<math.max(5.0,
            (follower.speed+leader.speed)*1.5+5.0))
        if sameLane and conflictDistance and
            not follower.p.emergencyEscape and not leader.p.emergencyEscape and
            not handled[follower.index..":"..leader.index] then
          handled[follower.index..":"..leader.index]=true
          local lane=follower.p.coordinatedLane
          local moved=false
          local emergencyCandidate=sameDirection and
            distance<math.max(14.0,follower.speed*0.95+5.0) and
            (leader.speed<math.max(1.5,follower.speed-0.8) or distance<6.0)
          if emergencyCandidate then
            -- Let the dedicated escape controller below own stopped obstacles;
            -- ordinary PASS/YIELD must not seize the lane first.
            moved=true
          elseif not sameDirection then
            -- Head-on conflicts are handled only by the emergency controller
            -- below. Do not start a normal lane transition here: it can be
            -- too slow and can send both cars toward the same lane.
          elseif follower.managed and leader.managed then
            -- A slower leading car yields right; otherwise the faster follower
            -- takes the adjacent left lane. Only one adjacent slot is changed.
            if closing>0.5 and lane>1 and leader.p.coordinatorChangeT<=0 then
              moved=coordinatorMove(leader,lane-1,records,"YIELD",follower.index)
            end
            if not moved and lane<follower.lanes.count and
                follower.p.coordinatorChangeT<=0 then
              moved=coordinatorMove(follower,lane+1,records,"PASS",leader.index)
            end
            if not moved and lane>1 and leader.p.coordinatorChangeT<=0 then
              moved=coordinatorMove(leader,lane-1,records,"YIELD",follower.index)
            end
          elseif follower.managed then
            -- The leading obstacle is not controlled by RoutePilot.
            if lane<follower.lanes.count and follower.p.coordinatorChangeT<=0 then
              moved=coordinatorMove(follower,lane+1,records,"PASS",leader.index)
            end
          elseif leader.managed then
            -- A non-managed faster vehicle behind: RoutePilot's car yields
            -- into any available adjacent slot, never by controlling the player.
            if leader.p.coordinatorChangeT<=0 then
              if lane>1 then
                moved=coordinatorMove(leader,lane-1,records,"YIELD",follower.index)
              elseif lane<leader.lanes.count then
                moved=coordinatorMove(leader,lane+1,records,"YIELD",follower.index)
              end
            end
          end
          if not moved and sameDirection and follower.managed then
            -- A failed lane reservation is a mandatory braking decision,
            -- not permission to continue at the previous target speed.
            local safeSpeed=safeFollowingSpeed(distance,leader.speed,
              follower.p.brakeAccelEstimate)
            follower.p.coordinatorConflictSpeed=math.min(
              safeNumber(follower.p.coordinatorConflictSpeed,math.huge),
              safeSpeed)
          end
          if not moved and math.abs(gap)<5.0 then
            H.log("TRAFFIC_CONFLICT_NO_SLOT follower=%d leader=%d gap=%.2f lane=%d",
              follower.index,leader.index,math.abs(gap),lane)
          end
        end
      end
    end
  end

  -- Emergency escape has priority over ordinary passing/yielding. It checks
  -- every lane, not only the adjacent one, and never waits for the 3 s normal
  -- lane-change cooldown. A stopped/slow leader triggers the escape early.
  for _,rec in ipairs(records) do
    if rec.managed and not rec.p.emergencyEscape then
      local p=rec.p
      local current=p.coordinatedLane or 1
      local nearestGap,nearestIndex,nearestSpeed=nil,nil,nil
      local nearestOncoming,nearestManaged=false,false
      for _,other in ipairs(records) do
        if other.index~=rec.index then
          local sameDirection=coordinatorSameDirection(rec,other)
          local gap=coordinatorGap(rec,other)
          local ownHalf=math.max(0.75,
            safeNumber(rec.car.aabbSize and rec.car.aabbSize.x,2.0)*0.5)
          local otherHalf=math.max(0.75,
            safeNumber(other.car.aabbSize and other.car.aabbSize.x,2.0)*0.5)
          local lateralOverlap=math.abs(
            safeNumber(other.p.latOff)-safeNumber(p.latOff))<
            ownHalf+otherHalf+0.35
          local candidateGap=sameDirection and gap or
            (gap and math.abs(gap) or nil)
          if lateralOverlap and candidateGap and
              (not sameDirection or gap>=-2.0) and
              (not nearestGap or candidateGap<nearestGap) then
            nearestGap=candidateGap
            nearestIndex=other.index
            nearestSpeed=other.speed
            nearestOncoming=not sameDirection
            nearestManaged=other.managed==true
          end
        end
      end
      local closing=nearestOncoming and (rec.speed+safeNumber(nearestSpeed)) or
        (rec.speed-safeNumber(nearestSpeed))
      local dangerDistance=math.max(14.0,
        (nearestOncoming and closing or rec.speed)*0.95+5.0)
      local emergencyDanger=nearestGap~=nil and nearestGap<dangerDistance and
        (nearestOncoming or nearestSpeed<math.max(1.5,rec.speed-0.8) or
          nearestGap<6.0)
      if emergencyDanger then
        local candidates={}
        -- For a head-on pair, exactly one managed car initiates the lateral
        -- escape. The other brakes and holds its lane; this prevents both cars
        -- from choosing the same empty lane at the same time.
        local mayEscape=not nearestOncoming or not nearestManaged or
          rec.index<nearestIndex
        if mayEscape then
          for lane=1,rec.lanes.count do
            if math.abs(lane-current)==1 then
              candidates[#candidates+1]={
                lane=lane,
                cost=math.abs(rec.lanes.centers[lane]-
                  safeNumber(p.latOff,rec.lanes.centers[current]))
              }
            end
          end
        end
        table.sort(candidates,function(a,b) return a.cost<b.cost end)
        local chosen=nil
        for _,candidate in ipairs(candidates) do
          if coordinatorEmergencyMove(rec,candidate.lane,records,nearestIndex) then
            chosen=candidate.lane
            break
          end
        end
        if chosen then
          p.emergencyBlockedT=0
          p.emergencyRescueReady=false
          p.emergencyWaitLogged=false
        else
          p.emergencyBlockedT=safeNumber(p.emergencyBlockedT)+math.max(0,dt or 0)
          p.emergencyRescueReady=p.emergencyBlockedT>=10.0
          p.coordinatorConflictSpeed=0.0
          p.trafficAction="EMERGENCY_WAIT"
          p.trafficReason=nearestOncoming and "oncoming_no_safe_lane" or
            "all_lanes_occupied"
          if not p.emergencyWaitLogged then
            p.emergencyWaitLogged=true
            H.log("TRAFFIC_EMERGENCY_NO_LANE id=%d obstacle=%d gap=%.2f oncoming=%s",
              rec.index,nearestIndex or -1,nearestGap,tostring(nearestOncoming))
          end
        end
      else
        p.emergencyBlockedT=0
        p.emergencyRescueReady=false
        p.emergencyWaitLogged=false
      end
    end
  end

  -- Complete passes and return yielding cars only after the faster vehicle
  -- has cleared the full pair of vehicle lengths and the home slot is free.
  for _,rec in ipairs(records) do
    local p=rec.p
    if rec.managed and not p.emergencyEscape and p.passTargetIndex~=nil then
      local target
      for _,other in ipairs(records) do
        if other.index==p.passTargetIndex then target=other;break end
      end
      local gap=target and coordinatorGap(rec,target) or nil
      local clearance=math.max(8.0,
        safeNumber(rec.car.aabbSize and rec.car.aabbSize.z,4.5)*0.5+
        safeNumber(target and target.car.aabbSize and target.car.aabbSize.z,4.5)*0.5+3.0)
      if not target or (gap and gap< -clearance) then
        p.passTargetIndex=nil;p.overtakeSide=0
        p.coordinatorReturn=true
      end
    end
    if rec.managed and not p.emergencyEscape and p.yieldingFor~=nil then
      local faster
      for _,other in ipairs(records) do
        if other.index==p.yieldingFor then faster=other;break end
      end
      local gap=faster and coordinatorGap(rec,faster) or nil
      if not faster or (gap and gap>8.0) then
        p.yieldingFor=nil;p.coordinatorReturn=true
      end
    end
    if rec.managed and not p.emergencyEscape and p.coordinatorReturn and p.homeLane and
        p.coordinatedLane~=p.homeLane and p.coordinatorChangeT<=0 then
      local step=p.homeLane>p.coordinatedLane and 1 or -1
      if coordinatorLaneFree(rec,p.coordinatedLane+step,records,8.0) then
        coordinatorMove(rec,p.coordinatedLane+step,records,"RETURN",nil)
      end
      if p.coordinatedLane==p.homeLane then p.coordinatorReturn=false end
    end
    if rec.managed then
      p.coordinatorOffset=rec.lanes.centers[p.coordinatedLane] or p.coordinatorOffset
      p.baseLaneOffset=p.coordinatorOffset
      p.laneOffset=p.coordinatorOffset
    end
  end
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
  local cornerScale=1.0-CFG.TC_CORNER_CUT*lateralUse*lateralUse
  gas=gas*p.tractionScale*cornerScale

  if speed<5 then gas=math.min(gas,0.6) end
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
      safeNumber(p.overtakeSide),0,controlWriteOk and 1 or 0)
  end
  p.diagPreviousTarget=target
  p.diagPreviousFinal=final
  p.diagPreviousTargetOffset=targetOffset
  p.diagPreviousTargetAlpha=targetAlpha
  p.diagPreviousTargetSource=targetSource
  p.diagPreviousPlanGeneration=p.racingPlanGeneration or 0
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
  local previousRouteProgress=p.routeProgress
  local recovering=p.offTrackRecovering==true or safeNumber(p.offTrackT)>0
  local projectionHint=p.routeProgress
  local allowGlobal=not recovering and not p.routeLocked
  if projectionHint==nil then allowGlobal=true end
  local progress, projectedLateral, projectedSeg, _, projectionDistance =
    routeProjection(car,p.segIdx,allowGlobal,projectionHint,speed,dt,recovering)
  if progress==nil and not p.spawnSeeded and previousRouteProgress~=nil then
    -- Off-track recovery may use a nearby global projection, but only when
    -- heading and route-progress continuity agree. This avoids jumping to a
    -- different branch at a crossing while still allowing a car to rejoin.
    local candidate=Geometry.project(state.routeGeometry,car.position,nil,
      car.look,speed,dt,false)
    if candidate and candidate.distance<=30.0 then
      local hx,hz=normalize2(car.look.x,car.look.z)
      local alignment=hx*candidate.tx+hz*candidate.tz
      local delta=candidate.s-previousRouteProgress
      if state.loopRoute then
        local total=state.routeLength or 0
        if total>0 and delta>total*0.5 then delta=delta-total end
        if total>0 and delta< -total*0.5 then delta=delta+total end
      end
      local continuityLimit=math.max(25.0,speed*dt*6.0+12.0)
      if alignment>=0.0 and math.abs(delta)<=continuityLimit then
        progress=candidate.s
        projectedLateral=candidate.lateral
        projectedSeg=candidate.seg
        projectionDistance=candidate.distance
        p.routeReacquired=true
      end
    end
  end
  if p.spawnSeeded and not recovering and
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

  local outsideBounds=false
  if progress~=nil and projectedLateral~=nil then
    local left,right=H.boundsAt(p.segIdx)
    outsideBounds=projectedLateral<left or projectedLateral>right
  end
  if outsideBounds or p.projectionLost then
    p.offTrackT=safeNumber(p.offTrackT)+dt
    p.offTrackRecovering=true
    if not p.offTrackRecoveryLogged then
      p.offTrackRecoveryLogged=true
      H.log("OFFTRACK_RECOVERY_BEGIN id=%d distance=%.2f lateral=%.2f",
        carIndex,safeNumber(projectionDistance,999),safeNumber(projectedLateral,999))
    end
  else
    if p.offTrackRecovering then
      H.log("OFFTRACK_RECOVERY_COMPLETE id=%d time=%.2f lateral=%.2f",
        carIndex,safeNumber(p.offTrackT),safeNumber(projectedLateral))
    end
    p.offTrackT=0
    p.offTrackRecovering=false
    p.offTrackRecoveryLogged=false
  end

  p.rescueRetryT=math.max(0,safeNumber(p.rescueRetryT)-dt)
  -- An emergency lane reservation being unavailable is not a recovery
  -- failure. Teleporting here hid the traffic-planning problem and made an
  -- identical obstacle scenario appear random. Keep the car on the road and
  -- let prepareTraffic retry the adjacent-lane reservation every frame.
  if p.emergencyRescueReady then
    p.emergencyRescueReady=false
    if not p.emergencyWaitLongLogged then
      p.emergencyWaitLongLogged=true
      H.log("TRAFFIC_EMERGENCY_WAIT_CONTINUE id=%d blocked=%.2f",
        carIndex,safeNumber(p.emergencyBlockedT))
    end
  elseif safeNumber(p.emergencyBlockedT)<=0 then
    p.emergencyWaitLongLogged=false
  end
  local blockingGap=nil
  if not p.projectionLost then
    blockingGap=select(1,carAheadInLane(carIndex,car,
      safeNumber(p.coordinatorOffset,p.latOff or 0)))
  end
  local trafficBlocked=blockingGap~=nil and blockingGap<18.0
  if speed<RESCUE_SPEED and safeNumber(car.gas)>=0.55 and
      not trafficBlocked and not p.projectionLost and
      not p.offTrackRecovering and safeNumber(p.emergencyBlockedT)<=0 then
    p.stuck=p.stuck+dt
  else
    p.stuck=0
  end
  local rescueReason=nil
  if safeNumber(p.emergencyBlockedT)>=10.0 then rescueReason="traffic_blocked_10s"
  elseif p.offTrackT>=OFFTRACK_TIME then rescueReason="offtrack_recovery_failed_10s"
  elseif p.stuck>=RESCUE_TIME and safeNumber(car.gas)>=0.55 and
      not p.offTrackRecovering and safeNumber(p.emergencyBlockedT)<=0 then
    rescueReason="stuck_gas_10s" end
  if rescueReason and p.rescueRetryT<=0 then
    p.stuck=0
    p.rescueRetryT=5.0
    local rescued=H.teleportHome and H.teleportHome(carIndex,car) or false
    H.log("RESCUE_ATTEMPT id=%d reason=%s success=%s elapsed=%.2f",
      carIndex,rescueReason,tostring(rescued),
      rescueReason=="traffic_blocked_10s" and safeNumber(p.emergencyBlockedT) or safeNumber(p.offTrackT))
    if rescued then return end
  end

  if not p.projectionLost then updateRacingPlan(carIndex,car,p,speed,dt) end
  local targetSpeed = (p.projectionLost or p.racingPlanBlocked) and 0 or planTargetSpeed(p,speed)
  if p.offTrackRecovering and not p.projectionLost then
    targetSpeed=math.min(targetSpeed,8.0)
  end
  targetSpeed = updateTrafficControl(carIndex, car, p, speed, targetSpeed, dt)
  -- Ordinary optimiser failures use the route-spline fallback; only NO_FIT stops.
  -- Overtake state can change in this frame; rebuild the steering path before
  -- issuing controls so the first lateral movement follows the reserved side.
  if not p.projectionLost then updateRacingPlan(carIndex,car,p,speed,dt) end
  p.targetSpeed = targetSpeed

  p.boundaryEmergency=false
  local steerTarget = calculateSteering(car, p, speed, fx, fz, dt)
  p.steerTarget = steerTarget
  if p.projectionLost then targetSpeed=0 end
  -- Boundary correction is a steering task, not a reason to stop.
  -- The predictive footprint guard already bends the target path back into
  -- the corridor. Do not convert lateral correction magnitude into a speed
  -- cap: that caused repeated full stops even with zero corridor deficit.
  p.targetSpeed=targetSpeed
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
    "CAR=%d POS=(%.2f,%.2f,%.2f) SPEED=%.2f STEER=%.3f GAS=%.3f BRAKE=%.3f TARGET_SPEED=%.2f LINE_ERROR=%.2f SEGMENT=%d PACE=%.2f BOUNDS=[%.1f,%.1f] OFF=%.1f OT_SIDE=%d CROSS=%.2f HEAD=%.3f CURVE=%.4f ROUTE=%.1f LANE=%.2f SAT=%d WHEEL_SLIP=%.3f TC_SCALE=%.2f STEER_CURV=%.4f BOUNDARY=%.2f SAFE=[%.2f,%.2f] BODY=%.2f DEFICIT=%.2f BRAKE_ACC=%.2f DRIVE_ACC=%.2f GRIP=%.2f STUCK=%.1f AXLE_VALID=%d FRONT_AXLE_RAD=%.3f REAR_AXLE_RAD=%.3f FRONT_TIRE_DEG=%.2f REAR_TIRE_DEG=%.2f FRONT_NDSLIP=%.3f REAR_NDSLIP=%.3f YAW_RATE_RAD=%.3f WHEELBASE_M=%.2f STEER_LOCK_DEG=%.1f STEER_TARGET=%.3f SLIP_ANGLE_DEG=%.2f YAW_EST_RAD=%.3f PASS_BLOCKED=%.1f PASS_TIME=%.1f LOOKAHEAD=%.1f PP_CURV=%.5f BOUND_STEER=%.4f LAT_VEL=%.3f PREVIEW_OFFSET=%.2f PLAN_BLOCKED=%d PLAN_BLOCK_DIST=%.1f TRAFFIC_STATE=%s TRAFFIC_TARGET=%d TRAFFIC_GAP=%.1f TRAFFIC_LEADER_SPEED=%.1f TRAFFIC_PRIORITY=%.0f TRAFFIC_OFFSET=%.2f TRAFFIC_REASON=%s%s LANE_SLOT=%d/%d OFFTRACK_T=%.1f",
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
    safeNumber(p.trafficTargetOffset),tostring(p.trafficReason or "unknown"),mode,
    safeNumber(p.coordinatedLane,-1),safeNumber(p.coordinatorLaneCount,1),
    safeNumber(p.offTrackT))
end

return Driver