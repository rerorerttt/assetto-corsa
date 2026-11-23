-- RoutePilot/driver.lua — native CSP AI traffic controller.
-- Native AC AI follows the generated version-7 spline. This module does not
-- override steering, throttle, brake, clutch or gearbox.
-- Passing changes only the faster follower's spline offset; the leader is
-- never commanded to move aside or reduce speed.

local Driver = {}
local CFG, S, H

local PASS_TRIGGER_GAP = 120.0
local PASS_MIN_GAP = 6.0
local PASS_CLEAR_GAP = 15.0
local PASS_MAX_TIME = 18.0
local PASS_OFFSET_RATE = 5.0
local PASS_SEPARATION = 0.28
local PASS_EDGE_MARGIN = 0.12
local WIDTH_PRESETS = {3, 7, 11, 15}
local PASS_LOOKAHEAD = 140.0
local PASS_OCCUPANCY_RANGE = 100.0
local STUCK_SPEED = 0.8
local STUCK_TIME = 10.0

local function clamp(x, a, b) return math.max(a, math.min(b, x)) end
local function number(x, fallback)
  if type(x) ~= 'number' or x ~= x or x == math.huge or x == -math.huge then return fallback or 0 end
  return x
end
local function deterministic(i)
  local x = math.sin(i * 127.1 + 311.7) * 43758.5453
  return x - math.floor(x)
end
local function approach(value, target, step)
  if target > value + step then return value + step end
  if target < value - step then return value - step end
  return target
end
local function flatDistance(a, b)
  local dx, dz = b.x - a.x, b.z - a.z
  return math.sqrt(dx * dx + dz * dz)
end
local function halfWidth(car)
  local width = 2.0
  pcall(function() if car.aabbSize and car.aabbSize.x > 0.5 then width = car.aabbSize.x end end)
  return clamp(width * 0.5, 0.75, 1.35)
end

local function buildMetrics()
  local count = H.segmentCount()
  if count < 3 then return nil end
  local starts, lengths, total = {}, {}, 0
  for i = 1, count do
    local d = flatDistance(S.nodes[i], H.segEnd(i))
    if d < 0.05 then return nil end
    starts[i], lengths[i] = total, d
    total = total + d
  end
  if total < 50 then return nil end
  return {starts = starts, lengths = lengths, total = total, count = count}
end

local function projectCar(index, car, p, metrics)
  local hint = (p and p.segIdx) or S.trafficHints[index] or 1
  local seg, t, lineError = H.findNearestSeg(car.position, hint)
  if p then p.segIdx = seg end
  S.trafficHints[index] = seg
  local a, b = S.nodes[seg], H.segEnd(seg)
  local dx, dz = b.x - a.x, b.z - a.z
  local len = math.sqrt(dx * dx + dz * dz)
  if len < 0.001 then return nil end
  local tx, tz = dx / len, dz / len
  local look = car.look
  local headingDot = look and (look.x * tx + look.z * tz) or 0
  local progress = metrics.starts[seg] + metrics.lengths[seg] * t
  local lateral = H.lateralOf(car.position, seg)
  if p then
    p.lineDist, p.lateral, p.headingDot = lineError, lateral, headingDot
  end
  return {
    index = index, car = car, p = p, seg = seg, progress = progress,
    lateral = lateral, headingDot = headingDot, half = halfWidth(car),
    speed = math.max(0, number(car.speedMs, 0)),
    pace = p and p.targetLevel or (number(car.aiLevel,1.0)<0 and 1.0 or number(car.aiLevel,1.0)),
    x = car.position.x, z = car.position.z
  }
end

local function signedGap(a, b, length)
  local d = b.progress - a.progress
  if d > length * 0.5 then d = d - length end
  if d < -length * 0.5 then d = d + length end
  return d
end

local function laneLayout()
  local laneCount=clamp(math.floor(number(S.widthPreset,2)),1,4)
  local width=WIDTH_PRESETS[laneCount]
  local laneWidth=width/laneCount
  local offsets={}
  for lane=1,laneCount do offsets[lane]=-width*0.5+laneWidth*(lane-0.5) end
  return laneCount,laneWidth,offsets
end

local laneFits
local function playerBlocksLane(follower, player, target, metrics)
  if not player then return false end
  local gap=signedGap(follower,player,metrics.total)
  local longitudinalClose=gap>-15.0 and gap<PASS_OCCUPANCY_RANGE
  if player.headingDot<0.25 then
    local dx,dz=player.x-follower.x,player.z-follower.z
    longitudinalClose=dx*dx+dz*dz<50.0*50.0
  end
  return longitudinalClose and
    math.abs(player.lateral-target)<follower.half+player.half+PASS_SEPARATION
end

local function assignLanes(snapshots,metrics)
  local cars={}
  for _,snap in pairs(snapshots) do
    if snap.p and snap.p.splineLoaded then cars[#cars+1]=snap end
  end
  if #cars==0 then return false end

  -- Prefer the rightmost lane, but never assign a car into the player's
  -- occupied corridor. If the player blocks the right lane, use the next
  -- safe lane to the left; if no lane fits, remain on the recorded centreline.
  local laneCount,_,offsets=laneLayout()
  local player=snapshots[0]
  for _,snap in ipairs(cars) do
    local lane,home=0,0
    for candidate=laneCount,1,-1 do
      local target=offsets[candidate]
      if laneFits(snap.seg,target,snap.half,metrics) and
        not playerBlocksLane(snap,player,target,metrics) then
        lane,home=candidate,target
        break
      end
    end
    snap.p.laneIndex=lane
    snap.p.homeOffset=home
    snap.p.targetOffset=home
    snap.p.currentOffset=0
    snap.p.laneAssigned=true
    H.log('LANE_ASSIGN car=%d lane=%d offset=%.2f pace=%.3f playerBlock=%s',
      snap.index,lane,home,snap.p.targetLevel,tostring(lane==0))
  end
  S.lanesAssigned=true
  return true
end

laneFits=function(seg, target, half, metrics)
  local remaining, index = PASS_LOOKAHEAD, seg
  for _ = 1, metrics.count do
    local left, right = H.boundsAt(index)
    if target - half < left + PASS_EDGE_MARGIN or target + half > right - PASS_EDGE_MARGIN then return false end
    remaining = remaining - metrics.lengths[index]
    if remaining <= 0 then break end
    index = index % metrics.count + 1
  end
  return remaining <= 0
end

local function laneIsFree(follower, leader, target, snapshots, metrics)
  local leaderIndex=leader and leader.index or -1
  local leaderIsAI=leader and leader.p~=nil
  for index, other in pairs(snapshots) do
    -- An AI leader is already known to be in its home lane; the player is
    -- different: if the player is the leader, still treat the player's actual
    -- position as an obstacle in the proposed passing corridor.
    if index ~= follower.index and (index ~= leaderIndex or not leaderIsAI) then
      local gap = signedGap(follower, other, metrics.total)
      local longitudinalClose = math.abs(gap) < PASS_OCCUPANCY_RANGE
      if other.headingDot < 0.25 then
        local dx, dz = other.x - follower.x, other.z - follower.z
        longitudinalClose = dx * dx + dz * dz < 50 * 50
      end
      if longitudinalClose then
        local occupiedLat = other.lateral
        if other.p and other.p.laneAssigned then
          occupiedLat = other.p.passState and other.p.passState ~= 'RETURN' and
            other.p.targetOffset or (other.p.homeOffset or other.lateral)
        end
        if math.abs(occupiedLat - target) < follower.half + other.half + PASS_SEPARATION then return false end
      end
    end
  end
  return true
end

local function choosePassOffset(follower, leader, snapshots, metrics)
  local laneCount,_,offsets=laneLayout()
  if laneCount<2 or not follower.p or not follower.p.laneIndex or follower.p.laneIndex<1 then return nil,0 end
  local nextLane=follower.p.laneIndex-1
  if nextLane<1 then return nil,0 end

  -- Never pass on the right: use only the immediately adjacent left lane.
  local target=offsets[nextLane]
  if laneFits(follower.seg,target,follower.half,metrics) and
    laneIsFree(follower,leader,target,snapshots,metrics) then
    return target,-1
  end
  return nil,0
end

local function ensureCar(index, car, dt)
  local p = S.p[index]
  if not p then
    local baseLevel = number(car.aiLevel, 1.0)
    if baseLevel < 0 then baseLevel = 1.0 end
    local baseAggression = number(car.aiAggression, 1.0)
    if baseAggression < 0 then baseAggression = 1.0 end
    local pace = 0.94 + deterministic(index) * 0.12
    p = {
      segIdx = 1, baseLevel = baseLevel, baseAggression = baseAggression,
      pace = pace, targetLevel = clamp(baseLevel * pace, 0.0, 1.15),
      splinePath = nil, splineLoaded = false, splineAge = 0, splineRetry = 0, settingsApplied = false,
      currentOffset = 0, targetOffset = 0, passState = nil, passLeader = nil,
      passSide = 0, passTimer = 0, passLoggedTimeout = false,
      retireTimer = 0, stuckTimer = 0, airTimer = 0, lineDist = 0, lateral = 0
    }
    S.p[index] = p
    H.log('CAR_INIT id=%d aiLevel=%.3f pace=%.3f targetLevel=%.3f',
      index, baseLevel, pace, p.targetLevel)
  end

  p.splineAge = p.splineAge + dt
  p.splineRetry = math.max(0,(p.splineRetry or 0)-dt)
  if p.splinePath ~= S.aiSplinePath or (not p.splineLoaded and p.splineRetry<=0) then
    p.splinePath, p.splineLoaded, p.splineAge = S.aiSplinePath, false, 0
    p.splineRetry=1.0
    local ok, result = pcall(physics.setAISpline, index, S.aiSplinePath, nil)
    if ok and result == true then
      p.splineLoaded = true
      H.log('AI_SPLINE_ASSIGNED id=%d file=%s result=true', index, tostring(S.aiSplinePath))
    else
      H.log('AI_SPLINE_FAILED id=%d file=%s result=%s error=%s',
        index, tostring(S.aiSplinePath), tostring(result), tostring(ok and 'not accepted' or result))
    end
  end

  if not p.settingsApplied then
    local okLevel, errLevel = pcall(physics.setAILevel, index, p.targetLevel)
    -- CSP documents 0.95 as the value corresponding to 100% aggression.
    local okAgg, errAgg = pcall(physics.setAIAggression, index, 0.95)
    local okCaution, errCaution = true, nil
    local okThrottle, errThrottle = true, nil
    local okTopSpeed, errTopSpeed = true, nil
    if physics.setAICaution then
      okCaution, errCaution = pcall(physics.setAICaution, index, 0.5)
    end
    -- Remove artificial AI caps; physical car limits still apply.
    if physics.setAIThrottleLimit then
      okThrottle, errThrottle = pcall(physics.setAIThrottleLimit, index, 1.0)
    end
    if physics.setAITopSpeed then
      okTopSpeed, errTopSpeed = pcall(physics.setAITopSpeed, index, 1e9)
    end
    p.settingsApplied = okLevel and okAgg and okCaution and okThrottle and okTopSpeed
    H.log('AI_PROFILE id=%d level=%.3f aggression=0.95 caution=0.50 throttleLimit=1 topSpeed=unlimited levelOK=%s aggressionOK=%s cautionOK=%s throttleOK=%s topSpeedOK=%s',
      index, p.targetLevel, tostring(okLevel), tostring(okAgg), tostring(okCaution),
      tostring(okThrottle), tostring(okTopSpeed))
    if not okLevel then H.log('AI_PROFILE_ERROR id=%d kind=level err=%s', index, tostring(errLevel)) end
    if not okAgg then H.log('AI_PROFILE_ERROR id=%d kind=aggression err=%s', index, tostring(errAgg)) end
    if not okCaution then H.log('AI_PROFILE_ERROR id=%d kind=caution err=%s', index, tostring(errCaution)) end
    if not okThrottle then H.log('AI_PROFILE_ERROR id=%d kind=throttle_limit err=%s', index, tostring(errThrottle)) end
    if not okTopSpeed then H.log('AI_PROFILE_ERROR id=%d kind=top_speed err=%s', index, tostring(errTopSpeed)) end
  end
  return p
end

function Driver.setup(cfg, state, helpers)
  CFG, S, H = cfg, state, helpers
  S.trafficHints = S.trafficHints or {}
  S.passOwner=nil
end

function Driver.beginFrame(sim, dt)
  if sim.customAISplinesAllowed == false then
    S.customAIBlocked=true
    S.status='CSP запретил custom AI spline в этой сессии'
    if not S.customAIBlockedLogged then
      S.customAIBlockedLogged=true
      H.log('AI_SPLINE_BLOCKED reason=session_policy')
    end
    return
  end
  S.customAIBlocked=false
  S.customAIBlockedLogged=false
  if not S.aiSplineBuilt or not S.aiSplinePath then return end
  local metrics = buildMetrics()
  if not metrics then
    S.aiSplineBuilt = false
    S.status = 'Ошибка геометрии AI-линии'
    H.log('TRAFFIC_ABORT reason=invalid_route_metrics')
    return
  end
  S.routeMetrics = metrics
  local snapshots = {}

  for i = 0, sim.carsCount - 1 do
    local car = ac.getCar(i)
    if car and car.isActive and not car.isInPitlane and not car.isInPit and (i == 0 or car.isAIControlled) then
      local p = i > 0 and car.isAIControlled and ensureCar(i, car, dt) or nil
      local snap = projectCar(i, car, p, metrics)
      if snap then snapshots[i] = snap end
    end
  end

  if not S.enabled or not S.spawnDone or S.sessionTime < 2 then return end
  local needsLanes=not S.lanesAssigned
  if not needsLanes then
    for _,snap in pairs(snapshots) do
      if snap.p and snap.p.splineLoaded and not snap.p.laneAssigned then needsLanes=true break end
    end
  end
  if needsLanes then assignLanes(snapshots,metrics) end
  if S.passOwner then
    local owner=S.p[S.passOwner]
    if not owner or not owner.passState then S.passOwner=nil end
  end

  for i = 1, sim.carsCount - 1 do
    local p = S.p[i]
    if p and p.passState and p.passState ~= 'RETURN' then
      p.passTimer = p.passTimer + dt
      local follower, leader = snapshots[i], snapshots[p.passLeader]
      if not follower or not leader then
        p.passState, p.targetOffset = 'RETURN', p.homeOffset or 0
        H.log('PASS_ABORT car=%d leader=%s reason=car_unavailable', i, tostring(p.passLeader))
      else
        local gap = signedGap(follower, leader, metrics.total)
        local home=p.homeOffset or 0
        if gap < -PASS_CLEAR_GAP then
          if laneIsFree(follower,leader,home,snapshots,metrics) then
            p.passState, p.targetOffset = 'RETURN', home
            H.log('PASS_COMPLETE car=%d leader=%d gap=%.1f side=%d', i, p.passLeader, gap, p.passSide)
          elseif not p.returnBlockedLogged then
            p.returnBlockedLogged=true
            H.log('PASS_RETURN_BLOCKED car=%d leader=%d gap=%.1f home=%.2f',
              i,p.passLeader,gap,home)
          end
        elseif p.passTimer > PASS_MAX_TIME and gap > 12.0 then
          if laneIsFree(follower,leader,home,snapshots,metrics) then
            p.passState, p.targetOffset = 'RETURN', home
            H.log('PASS_ABORT car=%d leader=%d reason=timeout_no_progress gap=%.1f',
              i,p.passLeader,gap)
          elseif not p.passLoggedTimeout then
            p.passLoggedTimeout=true
            H.log('PASS_LONG car=%d leader=%d gap=%.1f offset=%.2f home_lane_blocked=true',
              i,p.passLeader,gap,p.currentOffset)
          end
        elseif p.passTimer > PASS_MAX_TIME and not p.passLoggedTimeout then
          p.passLoggedTimeout = true
          H.log('PASS_LONG car=%d leader=%d gap=%.1f offset=%.2f',
            i,p.passLeader,gap,p.currentOffset)
        end
      end
    end
  end

  for i = 1, sim.carsCount - 1 do
    local follower, p = snapshots[i], S.p[i]
    if (not S.passOwner or S.passOwner==i) and follower and p and p.splineLoaded and
      p.laneAssigned and not p.passState and follower.headingDot > 0.35 then
      local laneCount,laneWidth=laneLayout()
      local bestLeader, bestGap = nil, PASS_TRIGGER_GAP
      for j, other in pairs(snapshots) do
        if j ~= i and other.headingDot > 0.35 then
          local gap = signedGap(follower, other, metrics.total)
          local sameLane=false
          if other.p and other.p.laneAssigned then
            sameLane=other.p.laneIndex==p.laneIndex
          else
            sameLane=math.abs(other.lateral-(p.homeOffset or follower.lateral))<laneWidth*0.45
          end
          if gap > PASS_MIN_GAP and gap < bestGap and sameLane then
            bestLeader, bestGap = other, gap
          end
        end
      end
      if bestLeader and laneCount>1 then
        local faster = p.targetLevel > bestLeader.pace + 0.015 or follower.speed > bestLeader.speed + 0.8
        if faster then
          local target, side = choosePassOffset(follower, bestLeader, snapshots, metrics)
          if target then
            p.passState, p.passLeader, p.passSide = 'OUT', bestLeader.index, side
            S.passOwner=i
            p.passTimer, p.passLoggedTimeout, p.returnBlockedLogged, p.targetOffset = 0, false, false, target
            H.log('PASS_START car=%d leader=%d gap=%.1f side=%d target=%.2f speed=%.1f leaderSpeed=%.1f pace=%.3f leaderPace=%.3f',
              i, bestLeader.index, bestGap, side, target, follower.speed, bestLeader.speed, p.targetLevel, bestLeader.pace)
          else
            p.lastPassBlocked = (p.lastPassBlocked or 0) + dt
            if p.lastPassBlocked >= 2.0 then
              p.lastPassBlocked = 0
              H.log('PASS_BLOCKED car=%d leader=%d gap=%.1f reason=no_free_corridor', i, bestLeader.index, bestGap)
            end
          end
        end
      end
    end
  end
end

function Driver.update(index, car, dt)
  if S.customAIBlocked then return end
  local p = S.p[index]
  if not p then p = ensureCar(index, car, dt) end
  if not p.splineLoaded then return end

  p.retireTimer = p.retireTimer + dt
  if p.retireTimer >= 2.0 then
    p.retireTimer = 0
    if physics.preventAIFromRetiring then pcall(physics.preventAIFromRetiring, index) end
  end

  local previous = p.currentOffset
  p.currentOffset = approach(p.currentOffset, p.targetOffset, PASS_OFFSET_RATE * math.max(0, math.min(dt, 0.05)))
  if p.laneAssigned or p.passState or math.abs(p.currentOffset) > 0.02 or p.offsetApplied then
    local ok, err = pcall(physics.setAISplineAbsoluteOffset, index, p.currentOffset, false)
    if ok then
      p.offsetApplied = math.abs(p.currentOffset) > 0.02 or p.passState ~= nil
    else
      H.log('PASS_OFFSET_ERROR car=%d offset=%.2f err=%s', index, p.currentOffset, tostring(err))
      p.passState, p.targetOffset = 'RETURN', p.homeOffset or 0
    end
    if math.abs(previous - p.currentOffset) > 0.08 then
      H.log('PASS_OFFSET car=%d state=%s offset=%.2f target=%.2f',
        index, tostring(p.passState), p.currentOffset, p.targetOffset)
    end
  end

  if p.passState == 'RETURN' and math.abs(p.currentOffset-(p.homeOffset or 0)) < 0.05 then
    H.log('PASS_RELEASE car=%d leader=%s', index, tostring(p.passLeader))
    p.passState, p.passLeader, p.passSide = nil, nil, 0
    if S.passOwner==index then
      H.log('PASS_RESERVATION_RELEASE car=%d reason=returned_home',index)
      S.passOwner=nil
    end
    p.passTimer = 0
    p.targetOffset, p.currentOffset = p.homeOffset or 0, p.homeOffset or 0
    p.offsetApplied = math.abs(p.currentOffset)>0.02
    p.passLoggedTimeout, p.returnBlockedLogged = false, false
  end

  if S.spawnDone and S.sessionTime > 10 and not car.isInPitlane then
    local speed=number(car.speedMs,0)
    if speed < STUCK_SPEED then
      p.stuckTimer=p.stuckTimer+dt
    else
      p.stuckTimer=0
    end

    -- No single airborne flag exists in the CSP car state. Four wheel loads
    -- are available, so require all four wheels to remain unloaded for 10 s.
    local loadedWheels,hasWheelData=0,false
    pcall(function()
      if car.wheels then
        for wheelIndex=0,3 do
          local wheel=car.wheels[wheelIndex]
          if wheel then
            hasWheelData=true
            if number(wheel.load,0)>120 then loadedWheels=loadedWheels+1 end
          end
        end
      end
    end)
    if hasWheelData and loadedWheels==0 and speed>3.0 then
      p.airTimer=p.airTimer+dt
    else
      p.airTimer=0
    end

    local reason=nil
    if p.stuckTimer>=STUCK_TIME then
      p.stuckTimer=0
      reason='stuck'
    elseif p.airTimer>=10.0 then
      p.airTimer=0
      reason='airborne'
    end
    if reason then
      local ok=H.teleportHome and H.teleportHome(index,car)
      H.log('RECOVERY_ATTEMPT car=%d reason=%s success=%s pos=(%.2f,%.2f,%.2f) loadedWheels=%d',
        index,reason,tostring(ok),car.position.x,car.position.y,car.position.z,loadedWheels)
    end
  end
end

function Driver.isSplineReady(index)
  local p = S.p[index]
  return p ~= nil and p.splineLoaded and p.splineAge >= 1.5
end

function Driver.resetAfterTeleport(index)
  local p = S.p[index]
  if not p then return end
  p.passState, p.passLeader, p.passSide = nil, nil, 0
  if S.passOwner==index then
    H.log('PASS_RESERVATION_RELEASE car=%d reason=teleport',index)
    S.passOwner=nil
  end
  p.passTimer = 0
  p.targetOffset, p.currentOffset = p.homeOffset or 0, 0
  p.offsetApplied, p.stuckTimer, p.airTimer = false, 0, 0
  p.segIdx, p.lineDist, p.lateral = 1, 0, 0
  pcall(physics.setAISplineAbsoluteOffset, index, 0, false)
end

function Driver.release(index)
  local p = S.p[index]
  if S.passOwner==index then
    H.log('PASS_RESERVATION_RELEASE car=%d reason=car_release',index)
    S.passOwner=nil
  end
  if not p then return end
  pcall(physics.setAISplineAbsoluteOffset, index, 0, false)
  pcall(physics.setAISpline, index, nil, nil)
  if p.baseLevel then pcall(physics.setAILevel, index, p.baseLevel) end
  if p.baseAggression then pcall(physics.setAIAggression, index, p.baseAggression) end
  S.p[index], S.trafficHints[index] = nil, nil
end

function Driver.invalidateAll()
  for index in pairs(S.p) do Driver.release(index) end
  S.p, S.trafficHints, S.routeMetrics = {}, {}, nil
  S.passOwner=nil
  S.lanesAssigned=false
end

function Driver.telemetryLine(index, car, p)
  return string.format(
    'CAR=%d POS=(%.2f,%.2f,%.2f) SPEED=%.2f AI_LEVEL=%.3f PACE=%.3f SPLINE=%s LINE_ERROR=%.2f SEG=%d LAT=%.2f LANE=%d HOME=%.2f PASS=%s LEADER=%s OFFSET=%.2f TARGET_OFFSET=%.2f',
    index, car.position.x, car.position.y, car.position.z, number(car.speedMs, 0),
    p.targetLevel or -1, p.pace or -1, tostring(p.splineLoaded), number(p.lineDist, -1),
    p.segIdx or -1, number(p.lateral, 0), p.laneIndex or 0, number(p.homeOffset, 0),
    tostring(p.passState or 'NONE'), tostring(p.passLeader or '-'),
    number(p.currentOffset, 0), number(p.targetOffset, 0)
  )
end

return Driver
