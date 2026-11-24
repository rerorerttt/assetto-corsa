-- RoutePilot/driver.lua — two-lane race traffic (CSP 0.3.2-preview2)
-- Both lanes are unrestricted on straights. Cars brake for route curvature
-- and traffic, and change to the adjacent lane only when its swept corridor is clear.

local Driver = {}
local CFG, state, H

local LANE_CHANGE_TIME = 2.3
local LANE_CHANGE_COOLDOWN = 2.0
local LANE_TRIGGER_DISTANCE = 10.0
local LANE_TRIGGER_TTC = 3.0
local LANE_FRONT_CLEAR = 12.0
local LANE_REAR_CLEAR = 8.0
local LANE_SAFETY_GAP = 7.0
local LANE_HALF_WIDTH = 1.25
local BOUNDARY_MARGIN = 1.10
local BOUNDARY_HARD_MARGIN = 0.35
-- High-speed look-ahead and smooth steering.
local STEER_LIMIT = 0.74
local STEER_GAIN = 0.92
local STEER_FILTER_TIME = 0.16
local STEER_RATE = 2.2
local LOOKAHEAD_BASE = 20.0
local LOOKAHEAD_SPEED = 0.82
local LOOKAHEAD_MAX = 110.0
local LANE_RESCUE_SPEED = 1.5
local LANE_RESCUE_TIME = 5.0
local GAS_RISE_RATE = 4.0
local GAS_RELEASE_RATE = 8.0
local GAS_SPIN_CUT_RATE = 14.0

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
  CFG = cfg or {}
  -- Keep the driver operational even if an older RoutePilot.lua supplies
  -- an incomplete configuration table.
  CFG.MAX_LATERAL_ACCEL = safeNumber(CFG.MAX_LATERAL_ACCEL, 17.5)
  CFG.BRAKE_ACCEL = safeNumber(CFG.BRAKE_ACCEL, 14.0)
  CFG.VEHICLE_HALF_WIDTH = safeNumber(CFG.VEHICLE_HALF_WIDTH, 1.05)
  CFG.BOUNDARY_CLEARANCE = safeNumber(CFG.BOUNDARY_CLEARANCE, 0.10)
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

local function laneCenterForRole(carIndex, seg, role)
  local baseOffset, twoLane = H.laneOffset(carIndex, seg)
  if not twoLane then return 0, false end
  local magnitude = math.abs(baseOffset)
  return role == "LEFT" and -magnitude or magnitude, true
end

local function newPersonality(carIndex, car)
  local initialSeg = (H.spawnSegment and H.spawnSegment(carIndex)) or
    select(1,H.findNearestSeg(car.position,nil,true))
  local laneOffset, twoLane = H.laneOffset(carIndex, initialSeg)
  local p = {
    segIdx = initialSeg,
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
    laneChanges = 0,
    boundaryError = 0,
    boundaryViolation = false,
    boundaryHard = false
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

-- Local preview only: inspect the road directly ahead on every update.
-- Curvature is measured from route samples, never from a prebuilt track speed map.
local function planLocalSpeed(p,speed)
  local target=math.huge
  local preview=clamp(math.max(45.0,speed*2.8+18.0),45.0,125.0)
  local distance=3.0
  while distance<=preview do
    local before=H.walkLine(p.segIdx,p.segT or 0,math.max(0,distance-3.0))
    local middle=H.walkLine(p.segIdx,p.segT or 0,distance)
    local after=H.walkLine(p.segIdx,p.segT or 0,distance+3.0)
    if not before or not middle or not after then break end
    local ax,az=normalize2(middle.x-before.x,middle.z-before.z)
    local bx,bz=normalize2(after.x-middle.x,after.z-middle.z)
    local angle=math.abs(signedAngle(ax,az,bx,bz))
    local curvature=angle/6.0
    if curvature>0.0015 then
      local curveSpeed=math.sqrt(CFG.MAX_LATERAL_ACCEL/curvature)
      local brakingSpeed=math.sqrt(curveSpeed*curveSpeed+
        2*CFG.BRAKE_ACCEL*math.max(0,distance-3.0))
      if brakingSpeed<target then target=brakingSpeed end
    end
    distance=distance+3.0
  end
  return target
end

local function scanTraffic(carIndex, car, p, speed, tx, tz)
  local sim=ac.getSim()
  if not sim then return nil end
  local nearest=nil
  local laneCenter=p.laneOffset
  for i=0,sim.carsCount-1 do
    if i~=carIndex then
      local other=ac.getCar(i)
      if other and other.isActive then
        local dx=other.position.x-car.position.x
        local dz=other.position.z-car.position.z
        local longitudinal=dx*tx+dz*tz
        local lateral=H.lateralOf(other.position,p.segIdx)
        local otherP=state.p[i]
        local inCurrent=math.abs(lateral-laneCenter)<=LANE_HALF_WIDTH
        if p.laneChangeProgress<1.0 then
          inCurrent=inCurrent or
            math.abs(lateral-p.laneStartOffset)<=LANE_HALF_WIDTH or
            math.abs(lateral-p.laneTargetOffset)<=LANE_HALF_WIDTH
        end
        if otherP and otherP.laneChangeProgress<1.0 then
          inCurrent=inCurrent or
            math.abs(lateral-otherP.laneStartOffset)<=LANE_HALF_WIDTH or
            math.abs(lateral-otherP.laneTargetOffset)<=LANE_HALF_WIDTH
        end
        if inCurrent and longitudinal>0.5 and
            (not nearest or longitudinal<nearest.gap) then
          local otherSpeed=math.max(0,safeNumber(other.speedMs,0))
          nearest={index=i,gap=longitudinal,speed=otherSpeed,
            closing=speed-otherSpeed}
        end
      end
    end
  end
  return nearest
end

-- Check the whole manoeuvre window in the destination lane. The source and
-- destination lane centres must both remain inside the recorded corridor.
local function laneChangePathFits(carIndex,p,targetOffset,speed)
  if not H.walkLine or not H.safeBoundsAt then return false end
  local distance=math.max(25.0,speed*LANE_CHANGE_TIME+12.0)
  for sample=0,distance,5.0 do
    local _,seg=H.walkLine(p.segIdx,p.segT or 0,sample)
    if not seg then return false end
    local safeMin,safeMax=H.safeBoundsAt(seg)
    local _,twoLane=H.laneOffset(carIndex,seg)
    if not twoLane then return false end
    local sourceSign=p.laneStartOffset<0 and -1 or 1
    local targetSign=targetOffset<0 and -1 or 1
    local assignedCenter=H.laneOffset(carIndex,seg)
    local laneMagnitude=math.abs(assignedCenter)
    local destination=targetSign<0 and -laneMagnitude or laneMagnitude
    local source=sourceSign<0 and -laneMagnitude or laneMagnitude
    -- safeBoundsAt() already includes the vehicle half-width and clearance.
    local margin=0.05
    if source<safeMin+margin or source>safeMax-margin or
       destination<safeMin+margin or destination>safeMax-margin then
      return false
    end
    local fraction=clamp(sample/distance,0,1)
    local swept=lerp(source,destination,fraction)
    if swept<safeMin+margin or swept>safeMax-margin then return false end
  end
  return true
end

-- A merge is accepted only if the destination lane is empty both ahead and
-- behind for the complete time needed to move the car across.
local function adjacentLaneIsClear(carIndex,car,p,targetOffset,tx,tz)
  local sim=ac.getSim()
  if not sim then return false end
  local ownSpeed=math.max(0,safeNumber(car.speedMs,0))
  local frontWindow=18.0
  local rearWindow=16.0
  for i=0,sim.carsCount-1 do
    if i~=carIndex then
      local other=ac.getCar(i)
      if other and other.isActive then
        local dx=other.position.x-car.position.x
        local dz=other.position.z-car.position.z
        local longitudinal=dx*tx+dz*tz
        local lateral=H.lateralOf(other.position,p.segIdx)
        local otherP=state.p[i]
        local occupiesTarget=math.abs(lateral-targetOffset)<=LANE_HALF_WIDTH
        if otherP and otherP.laneChangeProgress<1.0 then
          occupiesTarget=occupiesTarget or
            math.abs(otherP.laneStartOffset-targetOffset)<=LANE_HALF_WIDTH or
            math.abs(otherP.laneTargetOffset-targetOffset)<=LANE_HALF_WIDTH
        end
        local otherSpeed=math.max(0,safeNumber(other.speedMs,0))
        local dynamicFront=math.max(frontWindow,
          math.max(0,ownSpeed-otherSpeed)*LANE_CHANGE_TIME+12.0)
        local dynamicRear=math.max(rearWindow,
          math.max(0,otherSpeed-ownSpeed)*LANE_CHANGE_TIME+12.0)
        if occupiesTarget and longitudinal>=-dynamicRear and
            longitudinal<=dynamicFront then
          return false,i,longitudinal
        end
        -- Also reserve the entire lateral transition band. This prevents
        -- two cars from committing to intersecting merges simultaneously.
        local lo=math.min(p.laneOffset,targetOffset)+LANE_HALF_WIDTH
        local hi=math.max(p.laneOffset,targetOffset)-LANE_HALF_WIDTH
        if lateral>=lo and lateral<=hi and
            longitudinal>=-dynamicRear and longitudinal<=dynamicFront then
          return false,i,longitudinal
        end
      end
    end
  end
  return true
end

local function beginLaneTransition(p,targetOffset,targetRole,kind,carIndex,front)
  p.laneOriginalRole=p.laneRole
  p.laneStartOffset=p.laneOffset
  p.laneTargetOffset=targetOffset
  p.laneChangeProgress=0
  p.laneChangeAborting=false
  p.laneTransitionKind=kind
  p.laneTransitionTarget=targetRole
  p.laneChangeProgress=0
  if kind=="OVERTAKE" then
    p.overtakeState="PASSING"
    p.overtakeTargetIndex=front and front.index or nil
    p.returnRole=p.laneRole
  else
    p.overtakeState="RETURNING"
  end
  p.laneChanges=(p.laneChanges or 0)+1
  H.log("LANE_CHANGE_START id=%d kind=%s from=%s to=%s gap=%.2f",
    carIndex,kind,p.laneOriginalRole,targetRole,front and front.gap or -1)
end

local function updateLaneChange(carIndex,car,p,speed,tx,tz,dt)
  p.laneCooldown=math.max(0,(p.laneCooldown or 0)-dt)
  p.trafficBlocked=false
  local baseOffset,twoLane=H.laneOffset(carIndex,p.segIdx)
  p.twoLane=twoLane
  if not twoLane then
    p.laneRole="SINGLE"
    p.laneOffset,p.laneStartOffset,p.laneTargetOffset=baseOffset,baseOffset,baseOffset
    p.laneChangeProgress=1.0
    p.overtakeState="IDLE"
    p.overtakeTargetIndex=nil
    return nil,nil
  end
  if p.laneRole=="SINGLE" and p.laneChangeProgress>=1.0 then
    p.laneRole=laneRoleForOffset(baseOffset,true)
  end
  if p.laneChangeProgress>=1.0 then
    p.laneOffset=laneCenterForRole(carIndex,p.segIdx,p.laneRole)
    p.laneStartOffset=p.laneOffset
    p.laneTargetOffset=p.laneOffset
  end

  local front=scanTraffic(carIndex,car,p,speed,tx,tz)
  p.trafficGap=front and front.gap or math.huge
  local stateName=p.overtakeState or "IDLE"
  p.overtakeState=stateName

  -- Commit once: a blocked adjacent lane means stay behind the leader.
  if stateName=="IDLE" and p.laneChangeProgress>=1.0 and
      p.laneCooldown<=0 and front then
    local closing=math.max(0,front.closing)
    local ttc=closing>0.1 and front.gap/closing or math.huge
    if closing>0.8 and (front.gap<=18.0 or ttc<=4.0) then
      local current=laneCenterForRole(carIndex,p.segIdx,p.laneRole)
      local target=-current
      p.laneOffset=current
      p.laneStartOffset=current
      local fits=laneChangePathFits(carIndex,p,target,speed)
      local clear,blockedBy,blockedGap=false,-1,-1
      if fits then
        clear,blockedBy,blockedGap=adjacentLaneIsClear(
          carIndex,car,p,target,tx,tz)
      end
      if clear then
        local targetRole=laneRoleForOffset(target,true)
        beginLaneTransition(p,target,targetRole,"OVERTAKE",carIndex,front)
        H.log("OVERTAKE_COMMIT id=%d leader=%d gap=%.2f closing=%.2f ttc=%.2f",
          carIndex,front.index,front.gap,closing,ttc)
      else
        p.trafficBlocked=true
        p.blockedBy=blockedBy
        p.blockedGap=blockedGap
        p.laneCooldown=1.0
        H.log("OVERTAKE_WAIT id=%d leader=%d reason=%s blocked_by=%d gap=%.2f",
          carIndex,front.index,fits and "OCCUPIED" or "BOUNDARY",
          blockedBy or -1,blockedGap or -1)
      end
    end
  end

  -- Once committed, finish the transition without oscillating or repeatedly
  -- reconsidering the decision. The destination was reserved before launch.
  if p.laneChangeProgress<1.0 then
    p.laneChangeProgress=math.min(1.0,p.laneChangeProgress+dt/LANE_CHANGE_TIME)
    local t=p.laneChangeProgress
    local smooth=t*t*(3.0-2.0*t)
    p.laneOffset=p.laneStartOffset+
      (p.laneTargetOffset-p.laneStartOffset)*smooth
    if p.laneChangeProgress>=1.0 then
      p.laneOffset=p.laneTargetOffset
      p.laneRole=p.laneTransitionTarget or laneRoleForOffset(p.laneOffset,true)
      H.log("LANE_CHANGE_DONE id=%d kind=%s lane=%s",
        carIndex,p.laneTransitionKind or "UNKNOWN",p.laneRole)
      if p.laneTransitionKind=="RETURN" then
        p.overtakeState="IDLE"
        p.overtakeTargetIndex=nil
        p.returnRole=nil
        p.laneCooldown=1.0
      else
        p.overtakeState="PASSING"
      end
      p.laneTransitionKind=nil
      p.laneTransitionTarget=nil
    end
  end

  -- Return only after the passed car is clearly behind and the original lane
  -- has a full front/rear safety window. If it is occupied, remain in the
  -- overtaking lane; never dart back and forth.
  if p.overtakeState=="PASSING" and p.laneChangeProgress>=1.0 and
      p.laneCooldown<=0 and p.returnRole then
    local passed=not p.overtakeTargetIndex
    if p.overtakeTargetIndex then
      local other=ac.getCar(p.overtakeTargetIndex)
      if not other or not other.isActive then
        passed=true
      else
        local dx=other.position.x-car.position.x
        local dz=other.position.z-car.position.z
        local longitudinal=dx*tx+dz*tz
        passed=longitudinal < -14.0
      end
    end
    if passed then
      local returnOffset=laneCenterForRole(carIndex,p.segIdx,p.returnRole)
      local fits=laneChangePathFits(carIndex,p,returnOffset,speed)
      local clear,blockedBy,blockedGap=false,-1,-1
      if fits then
        clear,blockedBy,blockedGap=adjacentLaneIsClear(
          carIndex,car,p,returnOffset,tx,tz)
      end
      if clear then
        beginLaneTransition(p,returnOffset,p.returnRole,"RETURN",carIndex,nil)
      else
        p.trafficBlocked=true
        p.blockedBy=blockedBy
        p.blockedGap=blockedGap
      end
    end
  end

  if front then
    local available=math.max(0,front.gap-LANE_SAFETY_GAP)
    local safeSpeed=math.sqrt(front.speed*front.speed+
      2*CFG.BRAKE_ACCEL*available)
    return safeSpeed,front
  end
  return nil,front
end

local function calculateSteering(carIndex, car, p, speed, fx, fz, dt)
  local seg, t, lineDistance = H.findNearestSeg(car.position, p.segIdx, false)
  p.segIdx = seg
  p.segT = t
  p.lineDist = lineDistance
  p.latOff = H.lateralOf(car.position, seg)

  local baseOffset, twoLane = H.laneOffset(carIndex, seg)
  p.twoLane = twoLane
  if not twoLane then
    p.laneOffset = baseOffset
    p.laneTargetOffset = baseOffset
    p.laneStartOffset = baseOffset
    p.laneRole = "SINGLE"
  elseif p.laneChangeProgress >= 1.0 then
    if p.laneRole=="SINGLE" then p.laneRole=laneRoleForOffset(baseOffset,true) end
    local canonical = laneCenterForRole(carIndex, seg, p.laneRole)
    p.laneOffset, p.laneTargetOffset, p.laneStartOffset =
      canonical, canonical, canonical
  end
  local laneOffset = p.laneOffset

  local left, right = H.boundsAt(seg)
  local safeMin, safeMax
  if H.safeBoundsAt then
    safeMin,safeMax=H.safeBoundsAt(seg)
  else
    safeMin,safeMax=left+BOUNDARY_MARGIN,right-BOUNDARY_MARGIN
  end
  if safeMin > safeMax then
    safeMin, safeMax = (left + right) * 0.5, (left + right) * 0.5
  end
  local vehicleMargin=CFG.VEHICLE_HALF_WIDTH+CFG.BOUNDARY_CLEARANCE
  local laneMin,laneMax=safeMin,safeMax
  if twoLane and p.laneChangeProgress>=1.0 then
    if p.laneRole=="LEFT" then laneMax=math.min(laneMax,-vehicleMargin)
    else laneMin=math.max(laneMin,vehicleMargin) end
  end
  local targetOffset=clamp(laneOffset,laneMin,laneMax)
  p.boundaryError=0
  if p.latOff<laneMin then p.boundaryError=laneMin-p.latOff
  elseif p.latOff>laneMax then p.boundaryError=laneMax-p.latOff end
  p.laneViolation=math.abs(p.boundaryError)>0.05
  p.boundaryViolation=p.laneViolation
  p.boundaryHard=p.latOff<left+BOUNDARY_HARD_MARGIN or
    p.latOff>right-BOUNDARY_HARD_MARGIN

  local requestedLookahead=clamp(LOOKAHEAD_BASE+speed*LOOKAHEAD_SPEED,
    LOOKAHEAD_BASE,LOOKAHEAD_MAX)
  local function makeTarget(distance)
    local center=H.walkLine(seg,t,distance)
    local before=H.walkLine(seg,t,math.max(0,distance-2.5))
    local after=H.walkLine(seg,t,distance+2.5)
    if not center then return nil,nil end
    if not before then before=center end
    if not after then after=center end
    local tx,tz=normalize2(after.x-before.x,after.z-before.z)
    return center.x-tz*targetOffset,center.z+tx*targetOffset
  end
  local function previewPathFits(distance,targetX,targetZ)
    local dx,dz=targetX-car.position.x,targetZ-car.position.z
    local total=math.max(0.001,distance)
    for sample=5.0,distance-1.0,5.0 do
      local point,pointSeg=H.walkLine(seg,t,sample)
      local before=H.walkLine(seg,t,math.max(0,sample-2.5))
      local after=H.walkLine(seg,t,sample+2.5)
      if not point or not pointSeg or not before or not after then return false end
      local tangentX,tangentZ=normalize2(after.x-before.x,after.z-before.z)
      local fraction=sample/total
      local chordX=car.position.x+dx*fraction
      local chordZ=car.position.z+dz*fraction
      local lateral=(chordX-point.x)*-tangentZ+(chordZ-point.z)*tangentX
      local minOffset,maxOffset=H.safeBoundsAt(pointSeg)
      local _,sampleTwoLane=H.laneOffset(carIndex,pointSeg)
      if sampleTwoLane and p.laneChangeProgress>=1.0 and
          not p.boundaryViolation and
          (p.laneRole=="LEFT" or p.laneRole=="RIGHT") then
        if p.laneRole=="LEFT" then
          maxOffset=math.min(maxOffset,-vehicleMargin)
        else
          minOffset=math.max(minOffset,vehicleMargin)
        end
      end
      if lateral<minOffset+0.02 or lateral>maxOffset-0.02 then return false end
    end
    return true
  end
  -- Prefer one distant, stable target. Shorten it only in meaningful steps
  -- when the swept chord would leave the assigned lane or recorded corridor.
  local lookahead=requestedLookahead
  local targetX,targetZ=nil,nil
  while lookahead>=12.0 do
    targetX,targetZ=makeTarget(lookahead)
    if targetX and previewPathFits(lookahead,targetX,targetZ) then break end
    lookahead=lookahead-15.0
  end
  if not targetX or lookahead<12.0 then
    lookahead=12.0
    targetX,targetZ=makeTarget(lookahead)
  end
  p.previewDistance=lookahead

  local dx = targetX - car.position.x
  local dz = targetZ - car.position.z
  local distance = math.sqrt(dx * dx + dz * dz)
  if distance < 0.001 then return p.steer end

  local angle = signedAngle(fx, fz, dx / distance, dz / distance)
  local lateralError = targetOffset - p.latOff
  local lateralGain = p.boundaryViolation and 0.14 or 0.018
  local lateralLimit = p.boundaryViolation and 0.55 or 0.10
  local targetSteer = angle * STEER_GAIN +
    clamp(lateralError * lateralGain, -lateralLimit, lateralLimit)
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

  -- PI speed control tracks temporary corner and traffic targets.
  -- Anti-windup prevents delayed braking after long full-throttle sections.
  local drive = error * 0.18 + p.speedIntegral * 0.012
  if (drive > -1 and drive < 1) or (drive >= 1 and error < 0) or
      (drive <= -1 and error > 0) then
    -- The expanded integral range compensates for different drivetrain drag
    -- while tracking temporary corner and traffic speed targets.
    p.speedIntegral = clamp(p.speedIntegral + error * dt, -60, 60)
    drive = error * 0.18 + p.speedIntegral * 0.012
  end

  p.gas = clamp(drive, 0, 1)
  p.brake = clamp(-drive * 0.85, 0, 1)
  if error >= 0 then p.brake = 0 end
  if error <= 0 then p.gas = 0 end
end

function Driver.update(carIndex, car, dt)
  local p = state.p[carIndex] or newPersonality(carIndex, car)
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

  local seg, segT = H.findNearestSeg(car.position, p.segIdx, false)
  p.segIdx = seg
  p.segT = segT
  local a = state.nodes[seg]
  local b = H.segEnd(seg)
  local tx, tz = normalize2(b.x - a.x, b.z - a.z)

  local trafficSpeed, front
  trafficSpeed,front=updateLaneChange(
    carIndex,car,p,speed,tx,tz,dt)

  local steer = calculateSteering(carIndex, car, p, speed, fx, fz, dt)
  local cornerSpeed = planLocalSpeed(p, speed)
  local targetSpeed = cornerSpeed
  if trafficSpeed then
    targetSpeed = targetSpeed and math.min(targetSpeed, trafficSpeed) or trafficSpeed
  end

  if p.boundaryHard then
    targetSpeed = math.min(targetSpeed or 10.0, 10.0)
  elseif p.boundaryViolation then
    targetSpeed = math.min(targetSpeed or 22.0, 22.0)
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
  if carIndex==0 and not p.overrideLogged then
    H.log("PLAYER_OVERRIDE_RESULT ok=%s controls=%s steer=%.3f gas=%.3f brake=%.3f",
      tostring(ok),tostring(controls~=nil),steer,p.gas,p.brake)
    p.overrideLogged=true
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
    "CAR=%d LANE=%s POS=(%.2f,%.2f,%.2f) SPEED=%.2f TARGET=%s PREVIEW=%.1f SECTION=LOCAL STEER=%.3f GAS=%.3f BRAKE=%.3f WHEEL_SLIP=%.3f TC_FACTOR=%.2f HEADING_DOT=%.3f LINE_ERROR=%.2f SEGMENT=%d LAT_OFF=%.2f LANE_OFFSET=%.2f LANE_TARGET=%.2f LANE_PROGRESS=%.2f LANE_VIOLATION=%s LANE_CHANGES=%d TRAFFIC_GAP=%s TRAFFIC_BLOCKED=%s BOUNDARY_ERROR=%.2f BOUNDARY_HARD=%s BOUNDS=[%.1f,%.1f]",
    i, p.laneRole, car.position.x, car.position.y, car.position.z,
    safeNumber(car.speedMs), targetText, p.previewDistance or 0, p.steer, p.gas, p.brake,
    p.wheelSlip, p.tractionFactor, headingDot, p.lineDist, p.segIdx,
    p.latOff, p.laneOffset, p.laneTargetOffset or p.laneOffset, p.laneChangeProgress or 1,
    tostring(p.laneViolation), p.laneChanges or 0, gapText, tostring(p.trafficBlocked),
    p.boundaryError or 0, tostring(p.boundaryHard), left, right)
end

return Driver
