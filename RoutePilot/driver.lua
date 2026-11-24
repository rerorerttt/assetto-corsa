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
  -- Every participant uses the same configured cruise ceiling. Actual speed
  -- differences come from the car physics, not an index-based class.
  local kmh = clamp(safeNumber(state.speedMax, 170), 80, 240)
  local p = {
    segIdx = 1, cruiseSpeed = kmh / 3.6, playerAutopilot = carIndex == 0,
    steer = 0, steerFiltered = 0, gas = 0, brake = 0,
    slipAngle = 0, lateralAccel = 0, yawRate = 0,
    previousFx = nil, previousFz = nil,
    stuck = 0, blockedT = 0, passT = 0,
    lineDist = 0, latOff = 0, targetSpeed = 0,
    slipSeverity = 0, stability = 1,
    overtakeSide = 0, passLat = 0, blockerOff = 0, followTimer = 0,
    laneOffset = nil, passTargetIndex = nil, retireT = 2.0,
    overtakeRetryT = 0, emergencyPass = false,
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
  H.log("CAR_INIT id=%d lane=LEFT cruise=%.1fkmh",
    carIndex, kmh)
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

  local safeMin,safeMax,bodyEnvelope,boundaryDeficit=safeCorridorForPreview(
    route,projection.s,ahead,car,fx,fz)
  local bl,br=H.boundsAt(p.segIdx)
  local mid,width=(bl+br)*0.5,br-bl
  local leftCenter=width>=5.4 and ((mid+br)*0.5) or mid
  local rightCenter=(bl+mid)*0.5
  p.laneOffset=p.overtakeSide==0 and leftCenter or rightCenter
  local laneTarget=clamp(p.laneOffset or leftCenter,safeMin,safeMax)
  local laneChangeRate=p.emergencyPass and 2.6 or 1.5
  local maxStep=laneChangeRate*math.max(0,dt or 0)
  local previousOffset=p.laneOffsetCurrent
  if previousOffset==nil then previousOffset=laneTarget end
  p.laneOffsetCurrent=previousOffset+
    clamp(laneTarget-previousOffset,-maxStep,maxStep)

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

  -- Predict lateral drift over the preview time and steer back before the
  -- vehicle footprint reaches a saved boundary.
  local velocity=car.velocity
  local lateralVelocity=safeNumber(velocity.x)*(-current.tz)+
    safeNumber(velocity.z)*current.tx
  local previewTime=clamp(ahead/math.max(5.0,speed),0.35,1.25)
  local predictedLateral=projection.lateral+lateralVelocity*previewTime
  local safePredicted=clamp(predictedLateral,safeMin,safeMax)
  local boundaryError=safePredicted-predictedLateral
  local boundaryDistance=math.max(8.0,ahead*0.65)
  local boundaryCurvature=clamp(2.5*boundaryError/
    (boundaryDistance*boundaryDistance),-0.15,0.15)
  desiredCurvature=desiredCurvature+boundaryCurvature

  p.headingError=signedAngle(fx,fz,future.tx,future.tz)
  p.crossTrackError=p.laneOffsetCurrent-projection.lateral
  p.pathCurvature=safeNumber(current.k)
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
  return clamp(steer,-CFG.STEER_LIMIT,CFG.STEER_LIMIT)
end

local function filterSteering(p, target, dt)
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

local function laneOccupied(carIndex, car, laneOffset, ignoreIndex)
  local sim = ac.getSim()
  if not sim then return true end
  local own = state.p[carIndex]
  local ownProgress = own and own.routeProgress
  if ownProgress == nil then
    ownProgress = routeProjection(car, own and own.segIdx or 1, not (own and own.routeLocked), own and own.routeProgress, car.speedMs, 0.05)
  end
  if ownProgress == nil then return true end
  local ownSample=Geometry.sample(state.routeGeometry,ownProgress)
  if not ownSample then return true end
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
        local longitudinal,actualLateral
        if progress ~= nil then
          longitudinal=routeGap(ownProgress,progress)
          actualLateral=lateral
        elseif i==0 then
          -- The player may be outside the route-projection gate. Use physical
          -- position only for car 0; failed AI projections remain fail-closed.
          longitudinal,actualLateral=relativeToRoute(other.position,ownSample)
        end
        if longitudinal~=nil and actualLateral~=nil then
          local targetOffset=actualLateral
          local reserved=false
          if progress~=nil and otherState and otherState.laneOffset~=nil then
            targetOffset=otherState.laneOffset
            reserved=math.abs(targetOffset-laneOffset)<1.35
          end
          local ownWidth=math.max(1.5,safeNumber(car.aabbSize and car.aabbSize.x,2.0))
          local otherWidth=math.max(1.5,safeNumber(other.aabbSize and other.aabbSize.x,2.0))
          local lateralGap=math.abs(actualLateral-laneOffset)
          local bodyOverlap=lateralGap<(ownWidth+otherWidth)*0.5+0.55
          local sideWindow=math.max(8,math.max(0,safeNumber(car.speedMs))*0.30+4)
          local sideConflict=math.abs(longitudinal)<sideWindow and lateralGap<4.5
          local otherSpeed=math.max(0,safeNumber(other.speedMs))
          local ownSpeed=math.max(0,safeNumber(car.speedMs))
          local rearWindow=math.max(22,(ownSpeed+otherSpeed)*0.45+10)
          local frontWindow=math.max(55,math.max(ownSpeed,otherSpeed)*1.25+12)
          if (bodyOverlap or reserved or sideConflict) and
             longitudinal>-rearWindow and longitudinal<frontWindow then
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
  local ownSample=Geometry.sample(state.routeGeometry,ownProgress)
  if not ownSample then return nil,nil,nil end
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
        local gap,actualLateral
        if progress~=nil then
          gap=routeGap(ownProgress,progress)
          actualLateral=lateral
        elseif i==0 then
          -- Include the human car as a leader even when its spline projection
          -- is invalid; the route-tangent spatial test is local and bounded.
          gap,actualLateral=relativeToRoute(other.position,ownSample)
        end
        if gap and actualLateral and gap>0.5 and gap<90 and
           math.abs(actualLateral-laneOffset)<2.6 and
           (not bestGap or gap<bestGap) then
          bestGap=gap
          bestSpeed=math.max(0,safeNumber(other.speedMs))
          bestIndex=i
        end
      end
    end
  end
  return bestGap, bestSpeed, bestIndex
end

-- One nearby passing manoeuvre owns the passing lane reservation. Cars farther
-- away can pass independently; cars in the same convoy queue behind it.
local function passingReservationBlocked(carIndex, p, speed)
  if p.routeProgress == nil then return true end
  local reservationRange = math.max(140.0, math.max(0, speed) * 4.5)
  for otherIndex, other in pairs(state.p) do
    if otherIndex ~= carIndex and other.overtakeSide ~= 0 and
       other.routeProgress ~= nil then
      local gap = math.abs(routeGap(p.routeProgress, other.routeProgress))
      if gap < reservationRange then
        return true, otherIndex, gap
      end
    end
  end
  return false
end

local function safeFollowingSpeed(gap, leaderSpeed)
  local available = math.max(0, gap - math.max(0, safeNumber(state.followGap, 8)))
  return math.sqrt(math.max(0,
    leaderSpeed * leaderSpeed + 2 * CFG.BRAKE_ACCEL * available))
end

local function beginOvertake(carIndex,car,p,gap,leaderIndex,speed)
  local leftCenter,rightCenter=laneCenters(p.segIdx)
  if not leftCenter then return false end
  if p.overtakeRetryT>0 then return false end
  local fx,fz=normalize2(car.look.x,car.look.z)
  local passPreview=math.max(40.0,CFG.LOOKAHEAD_MAX)
  local safeMin,safeMax=safeCorridorForPreview(
    state.routeGeometry,p.routeProgress,passPreview,car,fx,fz)
  if not safeMin or not safeMax or rightCenter<safeMin+0.05 or
     rightCenter>safeMax-0.05 then
    p.overtakeRetryT=1.0
    if not p.overtakeDeniedLogged then
      H.log("CAR=%d OVERTAKE_DENIED passing_lane_not_safe",carIndex)
      p.overtakeDeniedLogged=true
    end
    return false
  end
  local reserved, reservedBy, reservedGap = passingReservationBlocked(carIndex,p,speed)
  if reserved then
    p.overtakeRetryT=0.8
    if not p.overtakeDeniedLogged then
      H.log("CAR=%d OVERTAKE_DENIED passing_slot_reserved by=%d gap=%.1f",
        carIndex,reservedBy or -1,reservedGap or -1)
      p.overtakeDeniedLogged=true
    end
    return false
  end
  if laneOccupied(carIndex,car,rightCenter,nil) then
    p.overtakeRetryT=1.0
    if not p.overtakeDeniedLogged then
      H.log("CAR=%d OVERTAKE_DENIED right_lane_occupied",carIndex)
      p.overtakeDeniedLogged=true
    end
    return false
  end
  p.overtakeDeniedLogged=false
  p.overtakeSide = -1
  p.passLat = rightCenter
  p.laneOffset = rightCenter
  p.passTargetIndex = leaderIndex
  p.passT = 0
  p.followTimer = 0
  p.blockedT = 0
  p.emergencyPass = p.emergencyPassRequest==true
  p.emergencyPassRequest=false
  H.log("CAR=%d OVERTAKE_BEGIN target=%d from=LEFT to=RIGHT gap=%.1f mode=%s",
    carIndex,leaderIndex or -1,gap or -1,p.emergencyPass and "URGENT" or "NORMAL")
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
  local leftCenter, rightCenter = laneCenters(p.segIdx)
  local gap, leaderSpeed, leaderIndex = carAheadInLane(
    carIndex, car, leftCenter or 0)

  if p.overtakeSide ~= 0 then
    p.passT = p.passT + dt
    local rightGap, rightLeaderSpeed = carAheadInLane(
      carIndex, car, rightCenter or p.laneOffset or p.passLat or -1.75)
    local passSafeSpeed = targetSpeed
    if rightGap then
      passSafeSpeed = math.min(passSafeSpeed,
        safeFollowingSpeed(rightGap, rightLeaderSpeed or 0))
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
      local passedDistance=(currentProgress and targetProgress)
        and routeGap(targetProgress,currentProgress) or -math.huge
      if passedDistance==-math.huge and p.passTargetIndex==0 then
        local ownSample=Geometry.sample(state.routeGeometry,currentProgress or ownProgress)
        local playerLongitudinal=relativeToRoute(target.position,ownSample)
        if playerLongitudinal~=nil then passedDistance=-playerLongitudinal end
      end
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
      p.emergencyPass=false
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
    return passSafeSpeed
  end

  if not leftCenter then
    if not p.overtakeDisabledLogged then
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

  local slower=(leaderSpeed or 0)<p.cruiseSpeed-1.0
  if slower then
    local speedAdvantage=p.cruiseSpeed-(leaderSpeed or 0)
    local targetCar=ac.getCar(leaderIndex)
    local ownLength=math.max(3.0,safeNumber(car.aabbSize and car.aabbSize.z,4.5))
    local targetLength=math.max(3.0,safeNumber(targetCar and targetCar.aabbSize and targetCar.aabbSize.z,4.5))
    local estimatedPassDistance=(ownLength+targetLength+8.0)*
      math.max(speed, (leaderSpeed or 0)+1.0)/math.max(1.0,speedAdvantage)
    local urgentGap=clamp(18.0+speed*0.9,25.0,60.0)
    local canActuallyPass=speedAdvantage>=5.0 and estimatedPassDistance<=130.0 and
      speed>=(leaderSpeed or 0)-3.0
    local urgent=canActuallyPass and gap<=urgentGap
    p.followTimer=p.followTimer+dt
    if urgent and p.overtakeSide==0 then
      p.emergencyPassRequest=true
      if beginOvertake(carIndex,car,p,gap,leaderIndex,speed) then
        return targetSpeed
      end
      p.emergencyPassRequest=false
    end
    local safe = safeFollowingSpeed(gap, leaderSpeed or 0)
    if p.followTimer>=TRIGGER_TIME and p.overtakeRetryT<=0 then
      p.emergencyPassRequest=false
      if beginOvertake(carIndex,car,p,gap,leaderIndex,speed) then
        return targetSpeed
      end
    end
    return math.min(targetSpeed, safe)
  end

  p.followTimer = 0
  p.blockedT = 0
  local safe = safeFollowingSpeed(gap, leaderSpeed or 0)
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
    "CAR=%d POS=(%.2f,%.2f,%.2f) SPEED=%.2f STEER=%.3f GAS=%.3f BRAKE=%.3f TARGET_SPEED=%.2f LINE_ERROR=%.2f SEGMENT=%d PACE=%.2f BOUNDS=[%.1f,%.1f] OFF=%.1f OT_SIDE=%d CROSS=%.2f HEAD=%.3f CURVE=%.4f ROUTE=%.1f LANE=%.2f SAT=%d WHEEL_SLIP=%.3f TC_SCALE=%.2f STEER_CURV=%.4f BOUNDARY=%.2f SAFE=[%.2f,%.2f] BODY=%.2f DEFICIT=%.2f%s",
    i, car.position.x, car.position.y, car.position.z,
    safeNumber(car.speedMs), p.steer, p.gas, p.brake, p.targetSpeed,
    p.lineDist, p.segIdx, (p.pace or 0), bl, br, p.latOff, p.overtakeSide,
    p.crossTrackError or 0, p.headingError or 0, p.pathCurvature or 0,
    p.routeProgress or 0, p.laneOffsetCurrent or 0,
    math.abs(p.steer or 0) >= steerLimit and 1 or 0,
    safeNumber(p.wheelSlip), safeNumber(p.tractionScale,1),
    safeNumber(p.steerCurvature), safeNumber(p.boundaryError),
    safeNumber(p.boundarySafeMin), safeNumber(p.boundarySafeMax),
    safeNumber(p.boundaryEnvelope),safeNumber(p.boundaryDeficit),mode)
end

return Driver