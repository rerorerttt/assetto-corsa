-- RoutePilot/driver.lua — v0.10.0 experimental trajectory planner (CSP 0.3.2-preview2)
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
  -- Start the field split across both usable halves of the recorded road.
  -- This prevents every AI from queueing on the same centreline and then
  -- requesting simultaneous passes into the same reserved corridor.
  local laneLeft,laneRight=H.boundsAt(p.segIdx or 1)
  local laneMid=(laneLeft+laneRight)*0.5
  local laneOffset=laneMid
  if laneMid-laneLeft>=2.7 and laneRight-laneMid>=2.7 then
    laneOffset=(carIndex%2==1) and
      ((laneMid+laneRight)*0.5) or ((laneLeft+laneMid)*0.5)
  end
  p.baseLaneOffset=laneOffset
  p.laneOffset=laneOffset
  state.p[carIndex] = p
  H.log("CAR_INIT id=%d lane=%s offset=%.2f cruise=PHYSICS_LIMITED",
    carIndex,laneOffset>laneMid and "LEFT" or
      (laneOffset<laneMid and "RIGHT" or "CENTRE"),laneOffset)
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
  local grip=clamp(safeNumber(p.lateralAccelLimit,CFG.MAX_LATERAL_ACCEL),4.0,20.0)
  if speed>12.0 and math.abs(p.slipAngle)<0.08 and
     safeNumber(p.wheelSlip)<0.20 and p.lateralAccel>grip*0.70 then
    grip=lerp(grip,clamp(p.lateralAccel*1.18,grip,20.0),0.015)
  elseif speed>12.0 and
     (math.abs(p.slipAngle)>0.15 or safeNumber(p.wheelSlip)>0.30) and
     p.lateralAccel>3.0 then
    grip=lerp(grip,clamp(p.lateralAccel*0.95,4.0,grip),0.08)
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
  local route=state.routeGeometry
  if not route or not p.routeProgress then return 11.0 end
  local target=p.cruiseSpeed
  local brakeAccel=clamp(safeNumber(p.brakeAccelEstimate,CFG.BRAKE_ACCEL),2.5,22.0)
  -- At high speed, inspect beyond the braking distance plus response margin.
  local currentSpeed=math.max(0,safeNumber(speed))
  local brakingDistance=currentSpeed*currentSpeed/(2*brakeAccel)
  local horizon=math.max(CFG.PLAN_HORIZON,brakingDistance+currentSpeed*1.5+35.0)
  local distance=0
  while distance<=horizon do
    local sample=Geometry.sample(route,p.routeProgress+distance)
    if not sample then break end
    local curvature=math.abs(sample.k or 0)
    if curvature>CFG.CURVE_MIN then
      local lateralLimit=clamp(safeNumber(p.lateralAccelLimit,CFG.MAX_LATERAL_ACCEL),4.0,20.0)
      local curveSpeed=math.sqrt(lateralLimit/curvature)
      local brakingDistanceAvailable=math.max(0,distance-currentSpeed*0.35)
      local safeSpeed=math.sqrt(curveSpeed*curveSpeed+
        2*brakeAccel*brakingDistanceAvailable)
      if safeSpeed<target then target=safeSpeed end
    end
    distance=distance+5.0
    if not route.loop and p.routeProgress+distance>=route.total then break end
  end
  local pathSamples=p.plannedPathSamples
  if pathSamples then
    for i=1,#pathSamples do
      local item=pathSamples[i]
      local curvature=math.abs(safeNumber(item.curvature))
      local distance=math.max(0,safeNumber(item.distance))
      if curvature>CFG.CURVE_MIN then
        local lateralLimit=clamp(safeNumber(p.lateralAccelLimit,CFG.MAX_LATERAL_ACCEL),4.0,20.0)
        local curveSpeed=math.sqrt(lateralLimit/curvature)
        local available=math.max(0,distance-currentSpeed*0.35)
        local safeSpeed=math.sqrt(curveSpeed*curveSpeed+2*brakeAccel*available)
        if safeSpeed<target then target=safeSpeed end
      end
    end
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
  -- Empty intersection means no constant-offset corridor exists.
  if deficit>0 then return nil,nil,maxEnvelope,deficit end
  return safeMin,safeMax,maxEnvelope,0
end

-- Quintic profile with the measured initial lateral velocity and zero
-- terminal lateral velocity/acceleration. Using the same function for
-- validation and steering prevents the controller from following a different
-- path than the one the boundary checker approved.
local function offsetProfile(startOffset,targetOffset,transitionLength,initialSlope,distance)
  local length=math.max(1.0,transitionLength)
  local u=clamp(distance/length,0,1)
  local m=clamp(initialSlope,-0.45,0.45)*length
  local d=targetOffset-startOffset-m
  local a=10*d+4*m
  local b=-15*d-7*m
  local c=6*d+3*m
  local u2,u3,u4,u5=u*u,u*u*u,u*u*u*u,u*u*u*u*u
  local offset=startOffset+m*u+a*u3+b*u4+c*u5
  local slope=(m+3*a*u2+4*b*u3+5*c*u4)/length
  local second=(6*a*u+12*b*u2+20*c*u3)/(length*length)
  return offset,slope,second
end

-- Evaluate one smooth lateral trajectory. The quintic profile starts with
-- measured lateral velocity and ends with zero lateral velocity/acceleration.
local function evaluateOffsetPath(route, progress, startOffset, targetOffset,
    transitionLength, horizon, halfWidth, halfLength, fx, fz, initialSlope)
  local samples,minClearance,maxCurvature,maxEnvelope={},math.huge,0,0
  local distance=-halfLength
  while distance<=horizon+halfLength+0.001 do
    local sample=Geometry.sample(route,progress+distance)
    if not sample then return nil,"route_end",minClearance,maxCurvature,maxEnvelope end
    local offset,slope,second=offsetProfile(
      startOffset,targetOffset,transitionLength,initialSlope,distance)
    local pathAngle=math.atan(slope)
    local initialAngle=signedAngle(sample.tx,sample.tz,fx,fz)
    -- Until the vehicle has aligned with the planned path, retain the larger
    -- of its measured heading error and the planned path angle in the footprint.
    local angleMagnitude=math.max(math.abs(pathAngle),
      math.abs(initialAngle)*(1-clamp(math.max(0,distance)/8.0,0,1)))
    local envelope=halfWidth*math.abs(math.cos(angleMagnitude))+
      halfLength*math.abs(math.sin(angleMagnitude))+BOUND_MARGIN
    maxEnvelope=math.max(maxEnvelope,envelope)
    local bl,br=routeBoundsAtSample(sample)
    local clearance=math.min(offset-bl-envelope,br-envelope-offset)
    minClearance=math.min(minClearance,clearance)
    if clearance < -0.015 then
      return nil,"boundary",minClearance,maxCurvature,maxEnvelope,
        distance,offset,bl,br,envelope
    end
    local baseCurvature=math.abs(safeNumber(sample.k))
    -- A lateral offset changes the radius of the road-relative path. Account
    -- for the normal-offset scale so an inside-line path is not assigned the
    -- centerline's more generous radius.
    local normalScale=math.max(0.25,1.0-baseCurvature*math.abs(offset))
    local curvature=baseCurvature/normalScale+
      math.abs(second)/(normalScale*normalScale*normalScale*
        math.pow(1+slope*slope,1.5))+
      2.0*baseCurvature*math.abs(slope)/(normalScale*normalScale)
    maxCurvature=math.max(maxCurvature,curvature)
    if distance>=0 then
      samples[#samples+1]={distance=distance,curvature=curvature}
    end
    distance=distance+2.0
  end
  return samples,nil,minClearance,maxCurvature,maxEnvelope
end

local function calculateSteering(car, p, speed, fx, fz, dt)
  refreshRouteMetrics()
  local route=state.routeGeometry
  p.plannedPathSamples=nil
  p.boundaryInfeasible=false
  p.plannerFirstBoundaryFailure=nil
  p.plannerFallback="none"
  if not route then
    p.lineDist=0; p.crossTrackError=0; p.headingError=0
    p.pathCurvature=0; p.steerCurvature=0
    return 0
  end
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

  local ahead=clamp(CFG.LOOKAHEAD_BASE+speed*CFG.LOOKAHEAD_SPEED,
    CFG.LOOKAHEAD_MIN,CFG.LOOKAHEAD_MAX)
  local current=Geometry.sample(route,projection.s)
  local future=Geometry.sample(route,projection.s+ahead)
  local curvePreview=Geometry.sample(route,projection.s+ahead*0.55)
  if not current or not future or not curvePreview then return 0 end

  local size=car and car.aabbSize
  local halfWidth=math.max(0.75,safeNumber(size and size.x,2.0)*0.5)
  local halfLength=math.max(1.5,safeNumber(size and size.z,4.5)*0.5)
  -- Track the measured car offset for occupancy/pass-clearance checks.
  -- The future steering target is sampled from the selected quintic profile.
  local startOffset=projection.lateral
  p.laneOffsetCurrent=startOffset
  local velocity=car.velocity
  local lateralVelocity=safeNumber(velocity.x)*(-current.tz)+
    safeNumber(velocity.z)*current.tx
  -- Do not extrapolate lateral velocity into a large synthetic path slope
  -- while launching or recovering at low speed. The previous 0.45 clamp,
  -- multiplied by an 18–48 m quintic transition, could create multi-metre
  -- lateral overshoot and reject every otherwise-safe candidate.
  local initialSlope=0
  if speed>8.0 then
    initialSlope=clamp(lateralVelocity/math.max(8.0,speed),-0.12,0.12)
  end
  local bl,br=routeBoundsAtSample(current)
  local desired=p.overtakeSide==0 and
    safeNumber(p.baseLaneOffset,(bl+br)*0.5) or
    safeNumber(p.passLat,p.laneOffset or startOffset)
  local transitionLength=clamp(math.max(18.0,speed*1.5),18.0,48.0)
  local brakeAccel=clamp(safeNumber(p.brakeAccelEstimate,CFG.BRAKE_ACCEL),2.5,22.0)
  local brakingDistance=speed*speed/(2*brakeAccel)
  -- Receding horizon: plan far enough to react at the current speed, but do
  -- not require one constant lateral offset to remain valid for 170 metres.
  -- A failed distant section must never freeze a car while its near path is clear.
  local requestedHorizon=clamp(math.max(ahead+halfLength,
    speed*2.25+14.0,brakingDistance*0.65+12.0),24.0,90.0)
  local horizonTrials={requestedHorizon}
  for _,candidateHorizon in ipairs({65.0,45.0,30.0,20.0,14.0}) do
    if candidateHorizon<requestedHorizon-1.0 then
      horizonTrials[#horizonTrials+1]=candidateHorizon
    end
  end

  -- Candidate offsets are constrained by the CURRENT vehicle footprint.
  -- Future boundaries are checked on every trajectory sample below. Taking
  -- the intersection of every road section over the whole horizon was wrong:
  -- it rejected valid paths whenever the road bends or changes width.
  local headingErrorNow=signedAngle(current.tx,current.tz,fx,fz)
  local currentEnvelope=halfWidth*math.abs(math.cos(headingErrorNow))+
    halfLength*math.abs(math.sin(headingErrorNow))+BOUND_MARGIN
  local currentLeft,currentRight=routeBoundsAtSample(current)
  local minOffset=currentLeft+currentEnvelope
  local maxOffset=currentRight-currentEnvelope
  if maxOffset<minOffset then
    p.boundaryInfeasible=true; p.gas=0; p.brake=1
    p.plannerMinClearance=maxOffset-minOffset
    p.plannerFailureReason="current_footprint"
    return 0
  end

  local candidates={}
  local function addCandidate(value)
    value=clamp(value,minOffset,maxOffset)
    for i=1,#candidates do
      if math.abs(candidates[i]-value)<0.10 then return end
    end
    candidates[#candidates+1]=value
  end
  addCandidate(desired)
  addCandidate(startOffset)
  addCandidate((minOffset+maxOffset)*0.5)
  -- Dense local lateral lattice; every candidate is subsequently checked
  -- against the complete car footprint and all future boundary samples.
  local width=maxOffset-minOffset
  local steps=math.min(24,math.max(1,math.ceil(width/0.25)))
  for i=0,steps do addCandidate(minOffset+width*i/steps) end

  local best,bestCost,bestSamples=nil,math.huge,nil
  local bestClearance,bestCurvature,bestEnvelope=-math.huge,math.huge,0
  local bestHorizon=0
  local rejectBoundary,rejectOther=0,0
  for h=1,#horizonTrials do
    local horizon=horizonTrials[h]
    local horizonBest,horizonCost,horizonSamples=nil,math.huge,nil
    local horizonClearance,horizonCurvature,horizonEnvelope=-math.huge,math.huge,0
    for i=1,#candidates do
      local candidate=candidates[i]
      local samples,reason,clearance,maxCurvature,envelope,failDistance,
        failOffset,failLeft,failRight,failEnvelope=evaluateOffsetPath(
        route,projection.s,startOffset,candidate,transitionLength,horizon,
        halfWidth,halfLength,fx,fz,initialSlope)
      if samples then
        local cost=math.abs(candidate-desired)+
          math.abs(candidate-startOffset)*0.10+maxCurvature*2.0+
          0.04/math.max(0.10,clearance+0.10)
        if cost<horizonCost then
          horizonBest,horizonCost,horizonSamples=candidate,cost,samples
          horizonClearance,horizonCurvature,horizonEnvelope=
            clearance,maxCurvature,envelope
        end
      elseif reason=="boundary" then
        rejectBoundary=rejectBoundary+1
        if not p.plannerFirstBoundaryFailure then
          p.plannerFirstBoundaryFailure={distance=failDistance,offset=failOffset,
            left=failLeft,right=failRight,envelope=failEnvelope,clearance=clearance}
        end
      else rejectOther=rejectOther+1 end
    end
    if horizonBest~=nil then
      best,bestCost,bestSamples=horizonBest,horizonCost,horizonSamples
      bestClearance,bestCurvature,bestEnvelope=
        horizonClearance,horizonCurvature,horizonEnvelope
      bestHorizon=horizon
      break
    end
  end

  -- Guaranteed recovery attempts before declaring the car blocked. First
  -- hold the present lateral corridor with zero synthetic lateral velocity;
  -- if that corridor narrows ahead, make one smooth move toward the route
  -- centre. Both fallbacks retain the same full-footprint boundary test.
  if best==nil then
    local samples,reason,clearance,maxCurvature,envelope=
      evaluateOffsetPath(route,projection.s,startOffset,startOffset,1.0,12.0,
        halfWidth,halfLength,fx,fz,0)
    if samples then
      best=startOffset;bestSamples=samples;bestClearance=clearance
      bestCurvature=maxCurvature;bestEnvelope=envelope;bestHorizon=12.0
      p.plannerFallback="hold_current_offset"
    else
      samples,reason,clearance,maxCurvature,envelope=
        evaluateOffsetPath(route,projection.s,startOffset,0.0,12.0,12.0,
          halfWidth,halfLength,fx,fz,0)
      if samples then
        best=0.0;bestSamples=samples;bestClearance=clearance
        bestCurvature=maxCurvature;bestEnvelope=envelope;bestHorizon=12.0
        p.plannerFallback="recover_to_route_centre"
      end
    end
  end

  p.plannerRejectBoundary=rejectBoundary
  p.plannerRejectOther=rejectOther
  p.plannerHorizon=bestHorizon
  p.plannerFailureReason=nil
  if best==nil then
    p.boundaryInfeasible=true
    p.boundaryCandidateMin=minOffset; p.boundaryCandidateMax=maxOffset
    p.boundaryEnvelope=currentEnvelope
    p.boundaryDeficit=0
    p.plannerMinClearance=-math.huge
    p.plannedPathSamples=nil
    p.plannerFailureReason="no_boundary_safe_local_trajectory"
    p.gas=0; p.brake=1
    if not p.plannerFailureLogged then
      local failure=p.plannerFirstBoundaryFailure or {}
      H.log("PLAN_BLOCK id=%d reason=%s speed=%.2f bounds=[%.2f,%.2f] reject=%d/%d first=d%.1f off=%.2f road=[%.2f,%.2f] env=%.2f clear=%.2f",
        p.carIndex or -1,p.plannerFailureReason,speed,minOffset,maxOffset,
        rejectBoundary,rejectOther,safeNumber(failure.distance,-1),
        safeNumber(failure.offset),safeNumber(failure.left),
        safeNumber(failure.right),safeNumber(failure.envelope),
        safeNumber(failure.clearance))
      p.plannerFailureLogged=true
    end
    return 0
  end

  p.plannerFailureLogged=false
  p.plannerFailureReason=nil
  p.plannedPathSamples=bestSamples
  p.plannedTargetOffset=best
  p.plannerHorizon=bestHorizon
  p.plannerMinClearance=bestClearance
  p.plannerMaxCurvature=bestCurvature
  p.boundaryCandidateMin=minOffset; p.boundaryCandidateMax=maxOffset
  p.boundaryEnvelope=bestEnvelope
  p.boundaryDeficit=0
  local previewOffset=offsetProfile(
    startOffset,best,transitionLength,initialSlope,ahead)
  if p.returningFromPass and math.abs(previewOffset-best)<0.25 then
    p.returningFromPass=false
  end

  local targetX=future.x-future.tz*previewOffset
  local targetZ=future.z+future.tx*previewOffset
  local dx,dz=targetX-car.position.x,targetZ-car.position.z
  local targetDist=math.sqrt(dx*dx+dz*dz)
  if targetDist<0.5 then return 0 end
  local alpha=signedAngle(fx,fz,dx/targetDist,dz/targetDist)
  local pursuitCurvature=2.0*math.sin(alpha)/targetDist
  local desiredCurvature=pursuitCurvature+
    safeNumber(curvePreview.k)*CFG.CURVATURE_FEEDFORWARD
  local previewTime=clamp(ahead/math.max(5.0,speed),0.35,1.25)
  local predictedLateral=projection.lateral+lateralVelocity*previewTime
  local boundaryError=best-predictedLateral
  local boundaryDistance=math.max(8.0,ahead*0.65)
  desiredCurvature=desiredCurvature+
    clamp(2.5*boundaryError/(boundaryDistance*boundaryDistance),-0.15,0.15)

  p.headingError=signedAngle(fx,fz,future.tx,future.tz)
  p.crossTrackError=previewOffset-projection.lateral
  p.pathCurvature=safeNumber(current.k)
  p.steerCurvature=desiredCurvature
  p.projectionProgressDelta=projection.delta or 0
  p.boundaryError=boundaryError
  p.boundaryEmergency=false
  return clamp(desiredCurvature*CFG.CURVATURE_STEER_GAIN+
    p.slipAngle*0.18,-CFG.STEER_LIMIT,CFG.STEER_LIMIT)
end

local function filterSteering(p, target, dt)
  -- No blind full-lock pulse: even an urgent pass must follow the selected
  -- collision-free path and remain under the same steering limit as cruising.
  p.emergencySteerBoostT=0
  local alpha = clamp(dt / (CFG.STEER_FILTER + dt), 0, 1)
  p.steerFiltered = lerp(p.steerFiltered, target, alpha)
  local steerRate=p.emergencyPass and math.max(CFG.STEER_RATE,5.5) or CFG.STEER_RATE
  local maxChange=steerRate*dt
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
        local dx=safeNumber(other.position.x)-ownSample.x
        local dz=safeNumber(other.position.z)-ownSample.z
        local worldDistanceSq=dx*dx+dz*dz
        local directionDot=otherFx*ownSample.tx+otherFz*ownSample.tz
        if worldDistanceSq<=65.0*65.0 then
          -- In the near field, use the actual relative position. This includes
          -- the manually driven player even when their route projection fails.
          longitudinal,actualLateral=relativeToRoute(other.position,ownSample)
        elseif progress~=nil and directionDot>=-0.25 then
          longitudinal=routeGap(ownProgress,progress)
          actualLateral=lateral
        elseif worldDistanceSq<=120.0*120.0 then
          longitudinal,actualLateral=relativeToRoute(other.position,ownSample)
        end

        if longitudinal~=nil and actualLateral~=nil then
          local otherSpeed=math.max(0,safeNumber(other.speedMs))
          local otherHalfWidth=math.max(0.75,
            safeNumber(other.aabbSize and other.aabbSize.x,2.0)*0.5)
          -- The road-boundary envelope already includes a 0.4 m margin.
          -- Keep only a small additional car-to-car clearance so a real
          -- two-car corridor is not rejected by double-counted margins.
          local separation=ownHalfWidth+otherHalfWidth+0.08
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
        local dx=safeNumber(other.position.x)-ownSample.x
        local dz=safeNumber(other.position.z)-ownSample.z
        local worldDistanceSq=dx*dx+dz*dz
        local directionDot=otherFx*ownSample.tx+otherFz*ownSample.tz
        if worldDistanceSq<=65.0*65.0 then
          gap,actualLateral=relativeToRoute(other.position,ownSample)
        elseif progress~=nil and directionDot>=-0.25 then
          gap=routeGap(ownProgress,progress);actualLateral=lateral
        elseif worldDistanceSq<=120.0*120.0 then
          gap,actualLateral=relativeToRoute(other.position,ownSample)
        end
        local halfLengths=(safeNumber(car.aabbSize and car.aabbSize.z,4.5)+
          safeNumber(other.aabbSize and other.aabbSize.z,4.5))*0.5
        local lateralSeparation=(math.max(1.5,safeNumber(car.aabbSize and car.aabbSize.x,2.0))+
          math.max(1.5,safeNumber(other.aabbSize and other.aabbSize.x,2.0)))*0.5+0.4
        local otherSpeed=math.max(0,safeNumber(other.speedMs))
        local detectionHorizon=directionDot < -0.25 and
          (ownSpeed+otherSpeed)*2.0+30.0 or ownSpeed*3.0+30.0
        if gap and actualLateral and gap>-1.0 and
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

local function findPassingOffset(carIndex,car,p,leaderIndex,leaderLateral,speed)
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
  local separation=(ownWidth+targetWidth)*0.5+0.08
  local candidates={}
  local candidateCount,reservedCount,occupiedCount=0,0,0
  local step=0.20
  for i=0,math.floor((safeMax-safeMin)/step) do
    local offset=safeMin+i*step
    if math.abs(offset-source)>=1.15 and math.abs(offset-leaderLateral)>=separation then
      candidates[#candidates+1]=offset
    end
  end
  candidateCount=#candidates
  table.sort(candidates,function(a,b)return math.abs(a-source)<math.abs(b-source) end)
  for _,offset in ipairs(candidates) do
    local reserved=passingReservationBlocked(carIndex,p,offset,speed)
    if reserved then
      reservedCount=reservedCount+1
    elseif laneOccupied(carIndex,car,offset,leaderIndex) then
      occupiedCount=occupiedCount+1
    else
      p.passSearchDiag=string.format("safe=[%.2f,%.2f] candidates=%d reserved=%d occupied=%d",
        safeMin,safeMax,candidateCount,reservedCount,occupiedCount)
      return offset
    end
  end
  p.passSearchDiag=string.format("safe=[%.2f,%.2f] candidates=%d reserved=%d occupied=%d",
    safeMin,safeMax,candidateCount,reservedCount,occupiedCount)
  return nil
end

local function safeFollowingCurrentLane(carIndex,car,p,currentGap,currentSpeed)
  -- Follow only the leader in the car's current corridor. A vehicle in a
  -- separate usable lane must never reduce this car's target speed.
  return safeFollowingSpeed(currentGap,currentSpeed,p.brakeAccelEstimate)
end

local function beginOvertake(carIndex,car,p,gap,leaderIndex,leaderLateral,speed)
  if p.overtakeRetryT>0 then return false end
  local offset=findPassingOffset(carIndex,car,p,leaderIndex,leaderLateral,speed)
  if offset==nil then
    p.overtakeRetryT=0.25
    if not p.overtakeDeniedLogged then
      H.log("CAR=%d OVERTAKE_DENIED no_free_passing_offset gap=%.1f %s",
        carIndex,gap or -1,p.passSearchDiag or "no_diagnostics")
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
  p.emergencySteerBoostT=0 -- retained as a telemetry/state field; blind full-lock is disabled
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
    if p.emergencyPass and target and target.isActive then
      local ownSample=Geometry.sample(state.routeGeometry,ownProgress)
      if ownSample then
        local targetLongitudinal,targetLateral=relativeToRoute(target.position,ownSample)
        local ownHalfWidth=math.max(0.75,safeNumber(car.aabbSize and car.aabbSize.x,2.0)*0.5)
        local targetHalfWidth=math.max(0.75,safeNumber(target.aabbSize and target.aabbSize.x,2.0)*0.5)
        local lateralClearance=ownHalfWidth+targetHalfWidth+0.08
        local currentOffset=safeNumber(p.laneOffsetCurrent,p.passLat)
        -- Until the vehicle has moved fully alongside the blocker, control
        -- closing speed as well as steering. Keep a small rolling speed so
        -- full-lock steering can actually move the car out of the lane.
        if targetLongitudinal and targetLongitudinal>0 and targetLateral and
           math.abs(targetLateral-currentOffset)<lateralClearance then
          local ownHalfLength=math.max(1.5,safeNumber(car.aabbSize and car.aabbSize.z,4.5)*0.5)
          local targetHalfLength=math.max(1.5,safeNumber(target.aabbSize and target.aabbSize.z,4.5)*0.5)
          local available=math.max(0,targetLongitudinal-ownHalfLength-targetHalfLength-0.8)
          local decel=clamp(safeNumber(p.brakeAccelEstimate,CFG.BRAKE_ACCEL),2.5,22.0)
          local evasionSpeed=math.max(5.0,math.sqrt(2.0*decel*available))
          passSafeSpeed=math.min(passSafeSpeed,evasionSpeed)
        end
      end
    end
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
      local passedDistance=-math.huge
      if ownSample then
        local dx=safeNumber(target.position.x)-ownSample.x
        local dz=safeNumber(target.position.z)-ownSample.z
        local worldDistanceSq=dx*dx+dz*dz
        if worldDistanceSq<=50.0*50.0 then
          -- Close-range pass completion must use actual world geometry.
          -- Route progress can wrap at the start/finish and falsely report
          -- that a bumper-to-bumper blocker is already behind us.
          local targetLongitudinal=relativeToRoute(target.position,ownSample)
          passedDistance=targetLongitudinal~=nil and -targetLongitudinal or -math.huge
        elseif targetProgress and targetDirectionDot>=-0.25 then
          passedDistance=routeGap(targetProgress,currentProgress or ownProgress)
        end
      end
      local ownLength = math.max(3.0,safeNumber(car.aabbSize and car.aabbSize.z,4.5))
      local otherLength = math.max(3.0,safeNumber(target.aabbSize and target.aabbSize.z,4.5))
      local requiredClearance=(ownLength+otherLength)*0.5+1.5
      local reachedPassingLine=math.abs(safeNumber(p.laneOffsetCurrent,p.passLat)-p.passLat)<0.45
      passedClearance = passedDistance >= requiredClearance and reachedPassingLine
    else
      -- A removed/despawned target is not a reason to cut back blindly:
      -- the full lane occupancy check below is still mandatory.
      passedClearance = math.abs(safeNumber(p.laneOffsetCurrent,p.passLat)-p.passLat)<0.45
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
  local immediateBlock=gap<=12.0 and leaderPace<=math.max(2.0,speed*0.15)
  local slower=speedAdvantage>=3.0 or immediateBlock
  if slower then
    local targetCar=ac.getCar(leaderIndex)
    local ownLength=math.max(3.0,safeNumber(car.aabbSize and car.aabbSize.z,4.5))
    local targetLength=math.max(3.0,safeNumber(targetCar and targetCar.aabbSize and targetCar.aabbSize.z,4.5))
    local clearanceDistance=ownLength+targetLength+6.0
    local estimatedPassDistance
    if speed-leaderPace>=3.0 then
      estimatedPassDistance=clearanceDistance*
        math.max(speed,leaderPace+1.0)/(speed-leaderPace)
    else
      local passTime=math.sqrt(2.0*clearanceDistance/acceleration)
      estimatedPassDistance=math.max(speed,leaderPace)*passTime+
        0.5*acceleration*passTime*passTime
    end
    local urgentGap=clamp(16.0+speed*1.0,25.0,70.0)
    local canActuallyPass=speedAdvantage>=3.0 and estimatedPassDistance<=180.0 and
      speed>=(leaderSpeed or 0)-4.0
    canActuallyPass=canActuallyPass or immediateBlock
    local urgent=canActuallyPass and (gap<=urgentGap or immediateBlock)
    local passWindow=math.max(45.0,32.0+speed*1.5)
    p.followTimer=p.followTimer+dt
    if canActuallyPass and gap>=-1.0 and gap<=passWindow and p.overtakeSide==0 and
       p.overtakeRetryT<=0 then
      p.emergencyPassRequest=urgent
      if beginOvertake(carIndex,car,p,gap,leaderIndex,leaderLateral,speed) then
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
        -- speedDifference is an absolute wheel/vehicle speed delta on
        -- several CSP physics builds; dividing by near-zero vehicle speed
        -- made launch slip appear enormous and cut almost all engine torque.
        local normalizedDiff=speedDiff/math.max(8.0,safeNumber(car.speedMs))
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

  -- Keep launch torque available; traction control now handles real
  -- wheelspin without the old permanent low-speed 60% throttle ceiling.
  if speed<5 then gas=math.min(gas,0.85) end
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

  p.carIndex=carIndex
  local speed, fx, fz = readVehicleDynamics(car, p, dt)
  local progress, _, projectedSeg, _, projectionDistance =
    routeProjection(car,p.segIdx,not p.routeLocked,p.routeProgress,speed,dt)
  if p.spawnSeeded and
     (projectionDistance==nil or projectionDistance>8.0) then
    progress=nil
  end
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

  -- Select and validate the path BEFORE deriving its speed profile. The old
  -- order evaluated speed twice and could apply a stale path's limit.
  local steerTarget = calculateSteering(car, p, speed, fx, fz, dt)
  local pathSpeedCap = p.projectionLost and 0 or planTargetSpeed(p,speed)
  local trafficSpeedCap = updateOvertake(carIndex, car, p, speed, pathSpeedCap, dt)
  local targetSpeed = trafficSpeedCap
  if p.projectionLost or p.boundaryInfeasible then targetSpeed=0 end
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
    "CAR=%d POS=(%.2f,%.2f,%.2f) SPEED=%.2f STEER=%.3f GAS=%.3f BRAKE=%.3f TARGET_SPEED=%.2f LINE_ERROR=%.2f SEGMENT=%d PACE=%.2f BOUNDS=[%.1f,%.1f] OFF=%.1f OT_SIDE=%d CROSS=%.2f HEAD=%.3f CURVE=%.4f ROUTE=%.1f LANE=%.2f SAT=%d WHEEL_SLIP=%.3f TC_SCALE=%.2f STEER_CURV=%.4f BOUNDARY=%.2f CANDIDATE_RANGE=[%.2f,%.2f] BODY=%.2f DEFICIT=%.2f BRAKE_ACC=%.2f DRIVE_ACC=%.2f GRIP=%.2f STUCK=%.1f PLAN_OFFSET=%.2f PLAN_CLEAR=%.2f PLAN_CURVE=%.4f PLAN_REJECT=%d/%d PLAN_FAIL=%d PLAN_HORIZON=%.1f PLAN_FAIL_REASON=%s%s",
    i, car.position.x, car.position.y, car.position.z,
    safeNumber(car.speedMs), p.steer, p.gas, p.brake, p.targetSpeed,
    p.lineDist, p.segIdx, (p.pace or 0), bl, br, p.latOff, p.overtakeSide,
    p.crossTrackError or 0, p.headingError or 0, p.pathCurvature or 0,
    p.routeProgress or 0, p.laneOffsetCurrent or 0,
    math.abs(p.steer or 0) >= steerLimit and 1 or 0,
    safeNumber(p.wheelSlip), safeNumber(p.tractionScale,1),
    safeNumber(p.steerCurvature), safeNumber(p.boundaryError),
    safeNumber(p.boundaryCandidateMin), safeNumber(p.boundaryCandidateMax),
    safeNumber(p.boundaryEnvelope),safeNumber(p.boundaryDeficit),
    safeNumber(p.brakeAccelEstimate,CFG.BRAKE_ACCEL),
    safeNumber(p.accelEstimate,0),safeNumber(p.lateralAccelLimit,CFG.MAX_LATERAL_ACCEL),
    safeNumber(p.stuck),safeNumber(p.plannedTargetOffset),
    safeNumber(p.plannerMinClearance),safeNumber(p.plannerMaxCurvature),
    safeNumber(p.plannerRejectBoundary),safeNumber(p.plannerRejectOther),
    p.boundaryInfeasible and 1 or 0,safeNumber(p.plannerHorizon),
    tostring(p.plannerFailureReason or "none"),mode)
end

return Driver