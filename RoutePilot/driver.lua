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
local TrajectoryOptimizer = require("trajectory_optimizer")
local CFG, state, H

local BOUND_MARGIN = 0.4
local REQ_SEP      = 2.8    -- center-to-center gap needed to pass, m
local TRIGGER_TIME = 0.5
local FORCE_TIME   = 2.0
local PASS_TIMEOUT = 8.0
local RESCUE_SPEED = 1.5
local RESCUE_TIME  = 10.0
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
  state.globalTrajectory = nil
  state.trajectoryCache = {}
  state.globalTrajectoryBuildFailed = nil
end

local function refreshRouteMetrics()
  if state.routeGeometry and state.routeMetricCount == #state.nodes and
     state.routeMetricLoop == state.loopRoute then return end
  state.routeGeometry = Geometry.build(state.nodes, state.loopRoute, 8)
  state.routeLength = state.routeGeometry and state.routeGeometry.total or 0
  state.routeMetricCount = #state.nodes
  state.routeMetricLoop = state.loopRoute
  state.playerTelemetry = nil
  state.globalTrajectory = nil
  state.trajectoryCache = {}
  state.globalTrajectoryBuildFailed = nil
  for _, p in pairs(state.p) do
    p.trajectoryPlan = nil
    p.trajectoryBundle = nil
    p.globalSpeedProfile = nil
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
    slipAngle = 0, lateralAccel = 0, yawRate = 0,
    previousFx = nil, previousFz = nil,
    stuck = 0, blockedT = 0, passT = 0,
    lineDist = 0, latOff = 0, targetSpeed = 0,
    slipSeverity = 0, stability = 1,
    overtakeSide = 0, passLat = 0, blockerOff = 0, followTimer = 0,
    laneOffset = nil, passTargetIndex = nil, retireT = 2.0,
    overtakeRetryT = 0, emergencyPass = false, returningFromPass = false,
  }
  local spawnHint=state.spawnHints and state.spawnHints[carIndex]
  if spawnHint and type(spawnHint.lateralOffset)=="number" then
    -- Preserve the grid lane after teleport; otherwise every car defaults
    -- to the same left lane and immediately converges across the road.
    p.baseLaneOffset=spawnHint.lateralOffset
    p.laneOffset=spawnHint.lateralOffset
    p.laneOffsetCurrent=spawnHint.lateralOffset
  end
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
  H.log("CAR_INIT id=%d lane_offset=%.2f cruise=PHYSICS_LIMITED",
    carIndex,safeNumber(p.baseLaneOffset,0))
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

local function calculateStability(p, speed)
  local slip = math.abs(p.slipAngle)
  local severity = 0
  if speed > CFG.SLIP_DAMP_MIN_SPEED and slip > CFG.SLIP_DAMP_MIN_ANGLE then
    local ss = clamp((slip - CFG.SLIP_START) / (CFG.SLIP_FULL - CFG.SLIP_START), 0, 1)
    local grip=clamp(safeNumber(p.lateralAccelLimit,CFG.MAX_LATERAL_ACCEL),4.0,20.0)
    local as = clamp((p.lateralAccel - grip) / (grip * 0.7), 0, 1)
    severity = math.max(ss, as)
  end
  p.slipSeverity = severity
  p.stability = 1 - severity
end

local function planTargetSpeed(p,speed)
  refreshRouteMetrics()
  if p.trajectoryPlan and p.overtakeSide==0 and not p.returningFromPass then
    local profile=TrajectoryOptimizer.sample(p.trajectoryPlan,p.routeProgress or 0,p.globalSpeedProfile)
    if profile then return math.max(0,profile.targetSpeed) end
  end
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
    local planned=p.racingPlan and RacingLine.sample(p.racingPlan,distance+clamp(planAdvance,0,30.0)) or nil
    local sample=planned or Geometry.sample(route,p.routeProgress+distance)
    if not sample then break end
    local curvature=math.abs(safeNumber(planned and planned.k or sample.k))
    -- Ignore curvature below the noise floor of a route recorded at 3 m.
    -- The optimizer's path curvature, not the reference-line curvature,
    -- determines braking on an open road.
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

local function representativeCar()
  if ac and ac.getSim then
    local sim=ac.getSim()
    if sim then
      for i=1,sim.carsCount-1 do
        local car=ac.getCar(i)
        if car and car.isActive then return car end
      end
      local player=ac.getCar(0)
      if player and player.isActive then return player end
    end
  end
  return nil
end

local function trajectorySizeKey(car)
  local size=car and car.aabbSize
  local w=math.max(0.75,safeNumber(size and size.x,2.0)*0.5)
  local l=math.max(1.5,safeNumber(size and size.z,4.5)*0.5)
  return string.format("%.1f:%.1f",math.floor(w*10+0.5)/10,
    math.floor(l*10+0.5)/10),w,l
end

function Driver.rebuildTrajectory()
  refreshRouteMetrics()
  if not state.routeGeometry then
    state.globalTrajectory=nil
    state.globalTrajectoryBuildFailed="NO_ROUTE"
    return false,"NO_ROUTE"
  end
  state.trajectoryCache={}
  for _,p in pairs(state.p) do
    p.trajectoryPlan=nil
    p.trajectoryBundle=nil
    p.globalSpeedProfile=nil
    p.racingPlan=nil
    p.racingPlanComputed=false
  end
  local car=representativeCar()
  local key=trajectorySizeKey(car)
  local plan,reason=TrajectoryOptimizer.build(state.routeGeometry,Geometry.sample,
    state.nodes,state.loopRoute,H.boundsAt,car,{
      spacing=math.max(4.5,state.routeGeometry.total/1800),
      lateralAccel=10.0,brakeAccel=8.0,accel=4.0
    })
  state.globalTrajectory=plan
  if plan then state.globalTrajectoryBuildFailed=nil
  else state.globalTrajectoryBuildFailed=reason end
  if plan then
    state.trajectoryCache[key]=plan
    H.log("GLOBAL_TRAJECTORY_READY points=%d alternatives=%d length=%.1f best_time=%.2f anchors=%s",
      #plan.points,plan.alternativeCount or 1,plan.routeLength or 0,
      plan.estimatedTime or 0,tostring(plan.anchorFraction))
    state.status=string.format("Гоночная траектория: %d варианта, %.1f с",
      plan.alternativeCount or 1,plan.estimatedTime or 0)
    for i,alternative in ipairs(plan.alternatives or {}) do
      H.log("GLOBAL_TRAJECTORY_ALT index=%d anchor=%.2f time=%.2f",
        i,alternative.anchorFraction or 0,alternative.estimatedTime or 0)
    end
    return true
  end
  H.log("GLOBAL_TRAJECTORY_FAILED reason=%s",tostring(reason))
  state.status=string.format("Оптимизация не удалась: %s",tostring(reason))
  return false,reason
end

local function trajectoryForCar(car)
  if not state.globalTrajectory and not state.globalTrajectoryBuildFailed then
    Driver.rebuildTrajectory()
  end
  local key=trajectorySizeKey(car)
  local cached=state.trajectoryCache and state.trajectoryCache[key]
  if cached~=nil then return cached or nil end
  if not state.routeGeometry then return nil end
  local plan,reason=TrajectoryOptimizer.build(state.routeGeometry,Geometry.sample,
    state.nodes,state.loopRoute,H.boundsAt,car,{
      spacing=4.5,lateralAccel=10.0,brakeAccel=8.0,accel=4.0
    })
  if plan then
    state.trajectoryCache=state.trajectoryCache or {}
    state.trajectoryCache[key]=plan
    H.log("VEHICLE_TRAJECTORY_READY size=%s points=%d alternatives=%d time=%.2f",
      key,#plan.points,plan.alternativeCount or 1,plan.estimatedTime or 0)
  else
    state.trajectoryCache=state.trajectoryCache or {}
    state.trajectoryCache[key]=false
    H.log("VEHICLE_TRAJECTORY_FAILED size=%s reason=%s",key,tostring(reason))
  end
  return plan
end

local function hasNearbyTraffic(carIndex,car)
  local sim=ac.getSim()
  if not sim then return false end
  local rangeSq=30.0*30.0
  local fx,fz=normalize2(car.look.x,car.look.z)
  for i=0,sim.carsCount-1 do
    if i~=carIndex then
      local other=ac.getCar(i)
      if other and other.isActive then
        local dx=other.position.x-car.position.x
        local dz=other.position.z-car.position.z
        local distanceSq=dx*dx+dz*dz
        local forward=dx*fx+dz*fz
        -- Cars ahead and cars alongside count as traffic; a distant rear car
        -- must not make the driver abandon its fastest clear-road trajectory.
        if distanceSq<rangeSq and forward>-6.0 then return true end
      end
    end
  end
  return false
end

local function assignTrajectory(p,car,carIndex)
  p.trajectoryBundle=trajectoryForCar(car)
  if p.trajectoryBundle then
    local preferred=nil
    if hasNearbyTraffic(carIndex,car) then
      preferred=type(p.latOff)=="number" and p.latOff or p.baseLaneOffset
    end
    p.trajectoryPlan,p.globalSpeedProfile=TrajectoryOptimizer.select(
      p.trajectoryBundle,p.lateralAccelLimit,p.brakeAccelEstimate,
      p.accelEstimate,preferred,p.routeProgress or 0)
  else
    p.trajectoryPlan=nil
    p.globalSpeedProfile=nil
  end
  p.globalProfileTimer=2.0
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
  local mid,width=(laneLeft+laneRight)*0.5,laneRight-laneLeft
  local leftCenter=width>=5.4 and ((mid+laneRight)*0.5) or mid
  if p.overtakeSide==0 then
    p.baseLaneOffset=p.baseLaneOffset or leftCenter
    p.laneOffset=p.baseLaneOffset
  else
    p.laneOffset=p.passLat
  end
  local laneTarget=p.laneOffset or leftCenter
  local planAdvance=p.racingPlanStartProgress and routeGap(p.racingPlanStartProgress,projection.s) or 0
  local raceTarget=p.racingPlan and RacingLine.sample(p.racingPlan,ahead+clamp(planAdvance,0,30.0)) or nil
  if not raceTarget and p.trajectoryPlan and p.overtakeSide==0 and not p.returningFromPass then
    raceTarget=TrajectoryOptimizer.sample(p.trajectoryPlan,projection.s+ahead,p.globalSpeedProfile)
  end
  if raceTarget then
    p.laneOffsetCurrent=raceTarget.offset
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
  if targetDist<0.5 then return 0 end

  -- Pure-pursuit curvature from the preview point, plus modest curvature
  -- feed-forward to begin rotating into the bend before heading error grows.
  local alpha=signedAngle(fx,fz,dx/targetDist,dz/targetDist)
  local pursuitCurvature=2.0*math.sin(alpha)/targetDist
  local previewCurvature=safeNumber(raceTarget and raceTarget.k or curvePreview.k)
  local desiredCurvature=pursuitCurvature+
    previewCurvature*CFG.CURVATURE_FEEDFORWARD

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
  local boundarySteer=clamp(boundaryError*0.075-lateralVelocity*0.035,-0.22,0.22)

  p.headingError=signedAngle(fx,fz,future.tx,future.tz)
  p.crossTrackError=p.laneOffsetCurrent-projection.lateral
  local currentRace= p.racingPlan and RacingLine.sample(p.racingPlan,0) or nil
  if not currentRace and p.trajectoryPlan and p.overtakeSide==0 and not p.returningFromPass then
    currentRace=TrajectoryOptimizer.sample(p.trajectoryPlan,projection.s,p.globalSpeedProfile)
  end
  p.pathCurvature=safeNumber(currentRace and currentRace.k or current.k)
  p.steerCurvature=desiredCurvature
  p.projectionProgressDelta=projection.delta or 0
  p.boundaryError=boundaryError
  p.boundarySafeMin=safeMin
  p.boundarySafeMax=safeMax
  p.boundaryEnvelope=bodyEnvelope
  p.boundaryDeficit=boundaryDeficit
  p.boundaryEmergency=math.abs(boundaryError)>0.45 or boundaryDeficit>0.15

  -- Small slip-angle damping is secondary; the spline preview is the
  -- primary steering signal.
  local steer=desiredCurvature*CFG.CURVATURE_STEER_GAIN+p.slipAngle*0.18
  if p.emergencyPass then
    steer=steer*CFG.EMERGENCY_STEER_GAIN
    return clamp(steer+boundarySteer,-1.0,1.0)
  end
  return clamp(steer+boundarySteer,-CFG.STEER_LIMIT,CFG.STEER_LIMIT)
end

local function filterSteering(p, target, dt)
  if safeNumber(p.emergencySteerBoostT)>0 and math.abs(target)>0.02 then
    p.emergencySteerBoostT=math.max(0,p.emergencySteerBoostT-dt)
    local fullLock=target>0 and 1.0 or -1.0
    p.steer=fullLock
    p.steerFiltered=fullLock
    return fullLock
  else
    p.emergencySteerBoostT=0
  end
  local alpha = clamp(dt / (CFG.STEER_FILTER + dt), 0, 1)
  p.steerFiltered = lerp(p.steerFiltered, target, alpha)
  local steerRate=p.emergencyPass and math.max(CFG.STEER_RATE,CFG.EMERGENCY_STEER_RATE) or CFG.STEER_RATE
  local maxChange=steerRate*dt
  local steerLimit=p.emergencyPass and 1.0 or CFG.STEER_LIMIT
  p.steer = clamp(p.steer + clamp(p.steerFiltered - p.steer, -maxChange, maxChange),
    -steerLimit, steerLimit)
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
            (otherState.overtakeSide~=0 or otherState.returningFromPass) and
            safeNumber(otherState.laneOffset,actualLateral) or actualLateral
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
            (otherState.overtakeSide~=0 or otherState.returningFromPass) and
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

local function hasTrafficInCorridor(carIndex,car,p,baseOffset)
  local sim=ac.getSim()
  if not sim or not p.routeProgress then return true end
  local ownSample=Geometry.sample(state.routeGeometry,p.routeProgress)
  if not ownSample then return true end
  local bl,br=H.boundsAt(p.segIdx)
  local corridorMin,corridorMax=bl-1.0,br+1.0
  for i=0,sim.carsCount-1 do
    if i~=carIndex then
      local other=ac.getCar(i)
      if other and other.isActive then
        local otherState=state.p[i]
        local progress,lateral=routeProjection(other,
          otherState and otherState.segIdx or p.segIdx,
          not (otherState and otherState.routeLocked),
          otherState and otherState.routeProgress,other.speedMs,0.05)
        if otherState and progress~=nil then
          otherState.routeProgress=progress
          otherState.routeLocked=true
        end
        local otherFx,otherFz=normalize2(other.look.x,other.look.z)
        local otherSample=progress and Geometry.sample(state.routeGeometry,progress)
        local directionDot=otherSample and
          (otherFx*otherSample.tx+otherFz*otherSample.tz) or 1.0
        local gap,off
        if progress~=nil and directionDot>=-0.25 then
          gap=routeGap(p.routeProgress,progress)
          off=lateral
        else
          gap,off=relativeToRoute(other.position,ownSample)
        end
        if gap~=nil and off~=nil and gap>-18.0 and gap<105.0 and
           off>=corridorMin and off<=corridorMax then
          return true
        end
      end
    end
  end
  return false
end

local function buildTrafficObstacles(carIndex,car,p,speed)
  local sim=ac.getSim()
  if not sim or not state.routeGeometry or p.routeProgress==nil then return {} end
  local obstacles={}
  local ownHalf=math.max(0.75,safeNumber(car.aabbSize and car.aabbSize.x,2.0)*0.5)
  for i=0,sim.carsCount-1 do
    if i~=carIndex then
      local other=ac.getCar(i)
      if other and other.isActive then
        local otherState=state.p[i]
        local op=otherState and otherState.routeProgress
        local ol=otherState and otherState.latOff
        -- Every active AI is updated by RoutePilot each frame. Reuse its
        -- current projection; only project an unadopted car once as fallback.
        if op==nil or ol==nil then
          op,ol=routeProjection(other,
            otherState and otherState.segIdx or p.segIdx,
            not (otherState and otherState.routeLocked),
            otherState and otherState.routeProgress,
            safeNumber(other.speedMs),0.05)
        end
        if op~=nil and ol~=nil then
          local otherSpeed=math.max(0,safeNumber(other.speedMs))
          local half=math.max(0.75,safeNumber(other.aabbSize and other.aabbSize.x,2.0)*0.5)
          local longitudinalGap=routeGap(p.routeProgress,op)
          -- Only cars that can occupy the same piece of road when we reach
          -- the sample become obstacles. Rear traffic is handled by its own
          -- planner and does not force us into a fixed lane.
          if longitudinalGap>=-7.0 and longitudinalGap<=120.0 then
            obstacles[#obstacles+1]={
              progress=op,lateral=ol,speed=otherSpeed,
              halfWidth=ownHalf+half+0.65
            }
          end
        end
      end
    end
  end
  return obstacles
end

local function updateRacingPlan(carIndex,car,p,speed,dt)
  refreshRouteMetrics()
  if not state.routeGeometry or p.routeProgress==nil then
    p.racingPlan=nil
    p.racingPlanBlocked=false
    p.racingPlanBlockedDistance=nil
    return
  end
  local bl,br=H.boundsAt(p.segIdx)
  local mid=(bl+br)*0.5
  local base=p.baseLaneOffset
  if base==nil then
    base=(br-bl)>=5.4 and ((mid+br)*0.5) or mid
    p.baseLaneOffset=base
  end

  p.racingPlanTimer=math.max(0,safeNumber(p.racingPlanTimer)-dt)
  local forcedMode=nil
  if p.overtakeSide~=0 then forcedMode="PASS"
  elseif p.returningFromPass then forcedMode="RETURN" end
  if forcedMode==nil and p.trajectoryPlan then
    p.racingPlan=nil
    p.racingPlanBlocked=false
    p.racingPlanBlockedDistance=nil
    p.racingPlanMode="OPEN"
    p.racingPlanComputed=true
    p.racingPlanTimer=0.40
    return
  end
  -- Cache failed plans too. Previously nil plans bypassed the timer and the
  -- optimizer ran twice per frame, logging NO_PATH thousands of times.
  if p.racingPlanComputed and p.racingPlanTimer>0 then
    if forcedMode==nil and
       (p.racingPlanMode=="OPEN" or p.racingPlanMode=="TRAFFIC") then return end
    if forcedMode~=nil and p.racingPlanMode==forcedMode then return end
  end

  local fixedOffset=nil
  local mode=forcedMode or "OPEN"
  if mode=="PASS" then
    fixedOffset=safeNumber(p.passLat,p.laneOffset or base)
  elseif mode=="RETURN" then
    fixedOffset=safeNumber(p.baseLaneOffset,base)
  end
  -- Traffic does not reserve a permanent lane. In open running the optimizer
  -- may use the complete physical road width; only an active, validated pass
  -- or its return phase temporarily reserves a passing corridor.

  local brakeAccel=clamp(safeNumber(p.brakeAccelEstimate,CFG.BRAKE_ACCEL),2.5,22.0)
  local v=math.max(0,speed)
  local horizon=clamp(math.max(CFG.PLAN_HORIZON,v*v/(2*brakeAccel)+v*1.5+35.0),CFG.PLAN_HORIZON,150.0)
  local planFailure=nil
  local obstacles=fixedOffset==nil and
    buildTrafficObstacles(carIndex,car,p,speed) or nil
  local function obstacleAt(sample,distance)
    local blocked={}
    local travelTime=distance/math.max(8.0,speed)
    for _,o in ipairs(obstacles or {}) do
      local otherProgress=o.progress+o.speed*travelTime
      local gap=routeGap(sample.s,otherProgress)
      if gap>=-6.0 and gap<=7.0 then
        blocked[#blocked+1]={o.lateral-o.halfWidth,o.lateral+o.halfWidth}
      end
    end
    return blocked
  end
  p.racingPlan,planFailure=RacingLine.plan(
    state.routeGeometry,Geometry.sample,p.routeProgress,car,routeBoundsAtSample,
    safeNumber(p.latOff,base),base,fixedOffset,horizon,7.5,
    fixedOffset==nil and obstacleAt or nil)
  local planFallback=false
  -- If the global search has no connected candidate chain, keep a validated
  -- constant-offset corridor and let following/overtake logic handle traffic.
  -- A search failure is not itself a physical obstacle and must not command
  -- a full stop on an otherwise drivable route.
  if planFailure=="NO_PATH" and fixedOffset==nil then
    p.racingPlan,planFailure=RacingLine.plan(
      state.routeGeometry,Geometry.sample,p.routeProgress,car,routeBoundsAtSample,
      safeNumber(p.latOff,base),base,base,horizon,7.5,nil)
    planFallback=p.racingPlan~=nil
  end
  p.racingPlanStartProgress=p.routeProgress
  p.racingPlanComputed=true
  p.racingPlanFallback=planFallback
  p.racingPlanBlocked=planFailure=="NO_FIT" or planFailure=="NO_PATH"
  p.racingPlanBlockedDistance=p.racingPlan and
    p.racingPlan.blockedDistance or nil
  if p.racingPlanBlocked then
    H.log("RACE_LINE_BLOCKED id=%d reason=%s",carIndex,tostring(planFailure))
  elseif planFallback then
    H.log("RACE_LINE_FALLBACK id=%d mode=%s reason=NO_PATH",carIndex,mode)
  end
  if p.racingPlanMode~=mode then
    H.log("RACE_LINE id=%d mode=%s base=%.2f target=%.2f curve=%.4f horizon=%.1f",
      carIndex,mode,base,safeNumber(p.racingPlan and p.racingPlan.points[2] and
      p.racingPlan.points[2].offset,base),
      safeNumber(p.racingPlan and p.racingPlan.curvature[2]),horizon)
  end
  p.racingPlanMode=mode
  p.racingPlanTimer=0.40
  if p.racingPlan then
    p.racingPlanCurvature=safeNumber(p.racingPlan.curvature[2])
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
       (other.overtakeSide~=0 or other.returningFromPass) and
       other.routeProgress~=nil and other.laneOffset~=nil then
      local gap=math.abs(routeGap(p.routeProgress,other.routeProgress))
      local otherCar=ac.getCar(otherIndex)
      local otherWidth=math.max(1.5,safeNumber(otherCar and otherCar.aabbSize and otherCar.aabbSize.x,2.0))
      local lateralSeparation=(ownWidth+otherWidth)*0.5+0.5
      if gap<range and math.abs(other.laneOffset-targetOffset)<lateralSeparation then
        return true,otherIndex,gap
      end
    end
  end
  return false
end

local function safeFollowingSpeed(gap,leaderSpeed,brakeAccel)
  local available=math.max(0,gap-math.max(0,safeNumber(state.followGap,8)))
  local decel=clamp(safeNumber(brakeAccel,CFG.BRAKE_ACCEL),2.5,22.0)
  local leader= safeNumber(leaderSpeed)
  if leader<0 then
    -- Negative leader speed marks an oncoming vehicle. Its velocity adds to
    -- closing speed, so compute the own speed that can stop before impact.
    local opposingSpeed=-leader
    return math.max(0,math.sqrt(opposingSpeed*opposingSpeed+
      2*decel*available)-opposingSpeed)
  end
  return math.sqrt(math.max(0,leader*leader+2*decel*available))
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
             (otherState.overtakeSide~=0 or otherState.returningFromPass) then
            otherOffset=safeNumber(otherState.laneOffset,otherOffset)
          end
          traffic[#traffic+1]={
            index=i,progress=op,lateral=otherOffset,speed=signedSpeed,
            halfWidth=otherHalfWidth,halfLength=otherHalfLength
          }
        end
      end
    end
  end

  local urgent=p.emergencyPassRequest==true
  local transitionOut=urgent and 6.0 or 22.0
  local transitionBack=22.0
  local totalDistance=math.max(20.0,passDistance)+transitionBack
  local step=urgent and 1.5 or 3.0
  local distance=0.0
  while distance<=totalDistance+0.001 do
    local sample=Geometry.sample(route,p.routeProgress+distance)
    if not sample then return false,"route_ends" end

    local outT=clamp(distance/transitionOut,0,1)
    if urgent then
      outT=1.0-(1.0-outT)*(1.0-outT)*(1.0-outT)
    else
      outT=outT*outT*(3.0-2.0*outT)
    end
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
  if p.routeProgress==nil or leaderLateral==nil then return nil end
  local fx,fz=normalize2(car.look.x,car.look.z)
  local safeMin,safeMax=safeCorridorForPreview(state.routeGeometry,p.routeProgress,
    math.max(18.0,math.max(0,speed)*0.8+12.0),car,fx,fz)
  if not safeMin or not safeMax or safeMax-safeMin<1.15 then return nil end
  local defaultCenter=laneCenters(p.segIdx)
  local bl,br=H.boundsAt(p.segIdx)
  local source=p.laneOffsetCurrent or p.laneOffset or p.baseLaneOffset or
    defaultCenter or ((bl+br)*0.5)
  local target=ac.getCar(leaderIndex)
  local ownWidth=math.max(1.5,safeNumber(car.aabbSize and car.aabbSize.x,2.0))
  local targetWidth=math.max(1.5,safeNumber(target and target.aabbSize and target.aabbSize.x,2.0))
  local separation=(ownWidth+targetWidth)*0.5+0.55
  local candidates={}
  local step=0.30
  for i=0,math.floor((safeMax-safeMin)/step) do
    local offset=safeMin+i*step
    if math.abs(offset-source)>=1.15 and
       math.abs(offset-leaderLateral)>=separation then
      candidates[#candidates+1]=offset
    end
  end
  table.sort(candidates,function(a,b)return math.abs(a-source)<math.abs(b-source) end)
  local lastReason="no_candidate"
  for _,offset in ipairs(candidates) do
    local reserved=passingReservationBlocked(carIndex,p,offset,speed)
    if not reserved and not laneOccupied(carIndex,car,offset,leaderIndex) then
      local clear,reason=passingPathClear(carIndex,car,p,leaderIndex,
        source,offset,passDistance or 60.0,speed)
      if clear then return offset end
      lastReason=reason or lastReason
    else
      lastReason=reserved and "reserved_corridor" or "occupied_corridor"
    end
  end
  p.lastPassDeniedReason=lastReason
  return nil
end

local function safeFollowingCurrentLane(carIndex,car,p,currentGap,currentSpeed)
  -- Follow only the leader in the car's current corridor. A vehicle in a
  -- separate usable lane must never reduce this car's target speed.
  return safeFollowingSpeed(currentGap,currentSpeed,p.brakeAccelEstimate)
end

local function beginOvertake(carIndex,car,p,gap,leaderIndex,leaderLateral,speed,passDistance)
  if p.overtakeRetryT>0 then return false end
  local offset=findPassingOffset(carIndex,car,p,leaderIndex,leaderLateral,speed,passDistance)
  if offset==nil then
    p.overtakeRetryT=0.25
    if not p.overtakeDeniedLogged then
      H.log("CAR=%d OVERTAKE_DENIED reason=%s gap=%.1f",carIndex,tostring(p.lastPassDeniedReason or "no_free_passing_offset"),gap or -1)
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
  p.overtakeSide=offset<source and -1 or 1
  p.passTargetIndex=leaderIndex;p.passT=0;p.followTimer=0;p.blockedT=0
  p.emergencyPass=p.emergencyPassRequest==true;p.emergencyPassRequest=false
  p.emergencySteerBoostT=(p.emergencyPass and gap<=8.0) and 0.18 or 0
  H.log("CAR=%d OVERTAKE_BEGIN target=%d from=%.2f to=%.2f gap=%.1f mode=%s",
    carIndex,leaderIndex or -1,source,offset,gap or -1,
    p.emergencyPass and "URGENT" or "NORMAL")
  return true
end

local function updateOvertake(carIndex, car, p, speed, targetSpeed, dt)
  p.overtakeRetryT=math.max(0,safeNumber(p.overtakeRetryT)-dt)
  local ownProgress, _, ownSeg = routeProjection(car, p.segIdx, not p.routeLocked, p.routeProgress, speed, dt)
  if ownProgress == nil then
    p.projectionLost = true
    return 0
  end
  p.routeLocked = true
  if ownSeg then p.segIdx = ownSeg end
  p.routeProgress = ownProgress
  if p.returningFromPass then return targetSpeed end
  local leftCenter,rightCenter=laneCenters(p.segIdx)
  local bl,br=H.boundsAt(p.segIdx)
  local defaultOffset=p.baseLaneOffset or leftCenter or ((bl+br)*0.5)
  local currentLaneOffset=p.overtakeSide~=0 and (p.passLat or p.laneOffset) or
    (p.laneOffsetCurrent or p.laneOffset or defaultOffset)
  local gap,leaderSpeed,leaderIndex,leaderLateral=carAheadInLane(
    carIndex,car,currentLaneOffset or 0)

  if p.overtakeSide ~= 0 then
    p.passT = p.passT + dt
    local rightGap,rightLeaderSpeed=carAheadInLane(
      carIndex,car,p.passLat or p.laneOffset or currentLaneOffset or 0,
      p.passTargetIndex)
    local passSafeSpeed = targetSpeed
    if rightGap then
      passSafeSpeed = math.min(passSafeSpeed,
        safeFollowingSpeed(rightGap, rightLeaderSpeed or 0,p.brakeAccelEstimate))
    end
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

    local returnOffset=p.passFromOffset or p.baseLaneOffset or defaultOffset
    if passedClearance and
       not laneOccupied(carIndex,car,returnOffset,p.passTargetIndex) then
      p.overtakeSide=0
      p.passTargetIndex=nil
      p.emergencyPass=false
      p.returningFromPass=true
      p.laneOffset=returnOffset
      p.baseLaneOffset=returnOffset
      p.passFromOffset=nil
      p.passLat=nil
      p.passT=0
      p.followTimer=0
      p.blockedT=0
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
    p.followTimer = 0
    p.blockedT = 0
    return targetSpeed
  end

  local leaderPace=math.max(0,leaderSpeed or 0)
  local acceleration=clamp(safeNumber(p.accelEstimate,4.0),0.5,18.0)
  -- Predict one second of available acceleration rather than treating an
  -- unbounded cruise target as an instantaneous speed advantage.
  local speedAdvantage=speed-leaderPace+acceleration
  local slower=speedAdvantage>=3.0
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
    local passTime
    local estimatedPassDistance
    if relativeSpeed>=3.0 then
      passTime=requiredClosure/relativeSpeed
      estimatedPassDistance=speed*passTime
    else
      passTime=(-relativeSpeed+
        math.sqrt(relativeSpeed*relativeSpeed+2.0*acceleration*requiredClosure))/
        acceleration
      estimatedPassDistance=speed*passTime+
        0.5*acceleration*passTime*passTime
    end
    local urgentGap=clamp(16.0+speed*1.0,25.0,70.0)
    local canActuallyPass=speedAdvantage>=3.0 and estimatedPassDistance<=180.0 and
      speed>=(leaderSpeed or 0)-4.0
    local urgent=canActuallyPass and gap<=urgentGap
    local passWindow=math.max(45.0,32.0+speed*1.5)
    p.followTimer=p.followTimer+dt
    if canActuallyPass and gap<=passWindow and p.overtakeSide==0 and
       p.overtakeRetryT<=0 then
      p.emergencyPassRequest=urgent
      if beginOvertake(carIndex,car,p,gap,leaderIndex,leaderLateral,speed,estimatedPassDistance) then
        return targetSpeed
      end
      p.emergencyPassRequest=false
    end
    local safe=safeFollowingCurrentLane(carIndex,car,p,gap,leaderSpeed or 0)
    return math.min(targetSpeed,safe)
  end

  p.followTimer = 0
  p.blockedT = 0
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
  local progress, projectedLateral, projectedSeg, _, projectionDistance =
    routeProjection(car,p.segIdx,not p.routeLocked,p.routeProgress,speed,dt)
  if p.spawnSeeded and
     (projectionDistance==nil or projectionDistance>8.0) then
    progress=nil
  end
  if progress ~= nil then
    p.routeLocked = true
    p.routeProgress = progress
    p.latOff = projectedLateral or p.latOff
    p.projectionLost = false
    if projectedSeg then p.segIdx = projectedSeg end
  else
    p.projectionLost = true
  end
  calculateStability(p, speed)
  readWheelSlip(car,p)

  if not p.trajectoryBundle then assignTrajectory(p,car,carIndex) end
  p.globalProfileTimer=math.max(0,safeNumber(p.globalProfileTimer)-dt)
  if p.trajectoryBundle and p.globalProfileTimer<=0 then
    local preferred=nil
    if hasNearbyTraffic(carIndex,car) then
      preferred=type(p.latOff)=="number" and p.latOff or p.baseLaneOffset
    end
    p.trajectoryPlan,p.globalSpeedProfile=TrajectoryOptimizer.select(
      p.trajectoryBundle,p.lateralAccelLimit,p.brakeAccelEstimate,
      p.accelEstimate,preferred,p.routeProgress or 0)
    p.globalProfileTimer=2.0
  end

  p.rescueRetryT=math.max(0,safeNumber(p.rescueRetryT)-dt)
  if speed<RESCUE_SPEED then
    p.stuck=p.stuck+dt
  else
    p.stuck=0
  end
  -- Traffic ahead is not a reason to disable recovery forever. A car that
  -- cannot make progress for ten seconds gets one safe respawn attempt.
  if p.stuck>=RESCUE_TIME and p.rescueRetryT<=0 then
    p.stuck=0
    p.rescueRetryT=5.0
    local rescued=H.teleportHome and H.teleportHome(carIndex,car) or false
    H.log("RESCUE_ATTEMPT id=%d success=%s",carIndex,tostring(rescued))
    if rescued then return end
  end

  if not p.projectionLost then updateRacingPlan(carIndex,car,p,speed,dt) end
  local targetSpeed = (p.projectionLost or p.racingPlanBlocked) and 0 or planTargetSpeed(p,speed)
  local previousOvertakeSide=p.overtakeSide
  local previousReturning=p.returningFromPass
  targetSpeed = updateOvertake(carIndex, car, p, speed, targetSpeed, dt)
  if p.racingPlanBlocked then targetSpeed=0 end
  -- Rebuild immediately only when the pass/return state actually changes.
  -- Normal plans are refreshed by their timer once per frame, not twice.
  if not p.projectionLost and
     (p.overtakeSide~=previousOvertakeSide or
      p.returningFromPass~=previousReturning) then
    updateRacingPlan(carIndex,car,p,speed,dt)
  end
  if p.racingPlanBlocked then targetSpeed=0 end
  p.targetSpeed = targetSpeed

  p.boundaryEmergency=false
  local steerTarget = calculateSteering(car, p, speed, fx, fz, dt)
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
  local mode=p.player and " MODE=PLAYER" or
    (p.playerAutopilot and " MODE=AI_AUTOPILOT" or "")
  return string.format(
    "CAR=%d POS=(%.2f,%.2f,%.2f) SPEED=%.2f STEER=%.3f GAS=%.3f BRAKE=%.3f TARGET_SPEED=%.2f LINE_ERROR=%.2f SEGMENT=%d PACE=%.2f BOUNDS=[%.1f,%.1f] OFF=%.1f OT_SIDE=%d CROSS=%.2f HEAD=%.3f CURVE=%.4f ROUTE=%.1f LANE=%.2f SAT=%d WHEEL_SLIP=%.3f TC_SCALE=%.2f STEER_CURV=%.4f BOUNDARY=%.2f SAFE=[%.2f,%.2f] BODY=%.2f DEFICIT=%.2f BRAKE_ACC=%.2f DRIVE_ACC=%.2f GRIP=%.2f STUCK=%.1f%s",
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
    safeNumber(p.stuck),mode)
end

return Driver