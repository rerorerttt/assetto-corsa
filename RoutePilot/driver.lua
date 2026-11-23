-- RoutePilot/driver.lua
-- Two independent native CSP AI lanes. No traffic negotiation, passing planner,
-- follower braking, lane changes or per-frame lateral corrections.
-- Lane 1: fast left spline. Lane 2: fixed 40 km/h right spline.

local Driver = {}
local S, H

local STUCK_SPEED = 0.8
local STUCK_TIME = 10.0

local function number(x, fallback)
  if type(x) ~= 'number' or x ~= x or x == math.huge or x == -math.huge then return fallback or 0 end
  return x
end

local function laneForCar(index)
  if (S.aiLaneCount or 1) < 2 then return 1, 0 end
  local lane = (index % 2 == 1) and 1 or 2
  local half = number(S.aiLaneHalfWidth, 1.75)
  return lane, lane == 1 and -half or half
end

local function ensureCar(index, car, dt)
  local p = S.p[index]
  if not p then
    local lane, offset = laneForCar(index)
    local baseLevel = number(car.aiLevel, 1.0)
    if baseLevel < 0 then baseLevel = 1.0 end
    local baseAggression = number(car.aiAggression, 1.0)
    if baseAggression < 0 then baseAggression = 1.0 end
    p = {
      laneIndex = lane, laneOffset = offset,
      splinePath = nil, splineLoaded = false, splineAge = 0, splineRetry = 0,
      baseLevel = baseLevel, baseAggression = baseAggression,
      settingsApplied = false, retireTimer = 0, stuckTimer = 0, airTimer = 0,
      lineDist = 0, lateral = 0
    }
    S.p[index] = p
    H.log('LANE_ASSIGN car=%d lane=%s offset=%.2f speed=%s',
      index, lane == 1 and 'LEFT_FAST' or 'RIGHT_40', offset,
      lane == 1 and 'TRACK_LIMIT' or '40_KMH')
  end

  local paths = S.aiSplinePaths or {}
  local path = paths[p.laneIndex] or S.aiSplinePath
  p.splineAge = p.splineAge + dt
  p.splineRetry = math.max(0, (p.splineRetry or 0) - dt)

  if path and (p.splinePath ~= path or (not p.splineLoaded and p.splineRetry <= 0)) then
    p.splinePath, p.splineLoaded, p.splineAge = path, false, 0
    p.splineRetry = 1.0
    local ok, result = pcall(physics.setAISpline, index, path, nil)
    if ok and result == true then
      p.splineLoaded = true
      H.log('AI_SPLINE_ASSIGNED car=%d lane=%d file=%s result=true', index, p.laneIndex, tostring(path))
    else
      H.log('AI_SPLINE_FAILED car=%d lane=%d file=%s result=%s error=%s',
        index, p.laneIndex, tostring(path), tostring(result), tostring(ok and 'not accepted' or result))
    end
  end

  if not p.settingsApplied then
    local okLevel, errLevel = pcall(physics.setAILevel, index, 2.0)
    local okAgg, errAgg = pcall(physics.setAIAggression, index, 1.0)
    local topSpeed = p.laneIndex == 2 and 40 or 1000000000
    local okTop, errTop = pcall(physics.setAITopSpeed, index, topSpeed)
    p.settingsApplied = okLevel and okAgg and okTop
    H.log('AI_PROFILE car=%d level=2.00 aggression=1.00 topSpeed=%.0f levelOK=%s aggressionOK=%s topSpeedOK=%s',
      index, topSpeed, tostring(okLevel), tostring(okAgg), tostring(okTop))
    if not okLevel then H.log('AI_PROFILE_ERROR car=%d kind=level err=%s', index, tostring(errLevel)) end
    if not okAgg then H.log('AI_PROFILE_ERROR car=%d kind=aggression err=%s', index, tostring(errAgg)) end
    if not okTop then H.log('AI_PROFILE_ERROR car=%d kind=top_speed err=%s', index, tostring(errTop)) end
  end
  return p
end

function Driver.setup(_, state, helpers)
  S, H = state, helpers
  S.p = S.p or {}
  S.trafficHints = S.trafficHints or {}
end

function Driver.beginFrame(sim, dt)
  if sim.customAISplinesAllowed == false then
    S.customAIBlocked = true
    S.status = 'CSP запретил custom AI spline в этой сессии'
    if not S.customAIBlockedLogged then
      S.customAIBlockedLogged = true
      H.log('AI_SPLINE_BLOCKED reason=session_policy')
    end
    return
  end
  S.customAIBlocked = false
  S.customAIBlockedLogged = false
  if not S.aiSplineBuilt or not S.aiSplinePaths then return end

  for i = 1, sim.carsCount - 1 do
    local car = ac.getCar(i)
    if car and car.isActive and car.isAIControlled then
      ensureCar(i, car, dt)
    elseif S.p[i] then
      Driver.release(i)
    end
  end
end

function Driver.update(index, car, dt)
  if S.customAIBlocked then return end
  local p = S.p[index] or ensureCar(index, car, dt)
  if not p.splineLoaded then return end

  p.retireTimer = p.retireTimer + dt
  if p.retireTimer >= 2.0 then
    p.retireTimer = 0
    if physics.preventAIFromRetiring then pcall(physics.preventAIFromRetiring, index) end
  end

  if S.spawnDone and S.sessionTime > 10 and not car.isInPitlane then
    local speed = number(car.speedMs, 0)
    if speed < STUCK_SPEED then p.stuckTimer = p.stuckTimer + dt else p.stuckTimer = 0 end

    local loadedWheels, hasWheelData = 0, false
    pcall(function()
      if car.wheels then
        for wheelIndex = 0, 3 do
          local wheel = car.wheels[wheelIndex]
          if wheel then
            hasWheelData = true
            if number(wheel.load, 0) > 120 then loadedWheels = loadedWheels + 1 end
          end
        end
      end
    end)
    if hasWheelData and loadedWheels == 0 and speed > 3.0 then
      p.airTimer = p.airTimer + dt
    else
      p.airTimer = 0
    end

    local reason
    if p.stuckTimer >= STUCK_TIME then p.stuckTimer, reason = 0, 'stuck'
    elseif p.airTimer >= 10.0 then p.airTimer, reason = 0, 'airborne' end
    if reason then
      local ok = H.teleportHome and H.teleportHome(index, car)
      H.log('RECOVERY_ATTEMPT car=%d reason=%s success=%s lane=%d',
        index, reason, tostring(ok), p.laneIndex)
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
  p.stuckTimer, p.airTimer = 0, 0
end

function Driver.release(index)
  local p = S.p[index]
  if not p then return end
  pcall(physics.setAISpline, index, nil, nil)
  if p.baseLevel then pcall(physics.setAILevel, index, p.baseLevel) end
  if p.baseAggression then pcall(physics.setAIAggression, index, p.baseAggression) end
  if physics.setAITopSpeed then pcall(physics.setAITopSpeed, index, 1000000000) end
  S.p[index], S.trafficHints[index] = nil, nil
end

function Driver.invalidateAll()
  for index in pairs(S.p) do Driver.release(index) end
  S.p, S.trafficHints = {}, {}
end

function Driver.telemetryLine(index, car, p)
  return string.format(
    'CAR=%d POS=(%.2f,%.2f,%.2f) SPEED=%.2f SPLINE=%s LANE=%d LANE_OFFSET=%.2f',
    index, car.position.x, car.position.y, car.position.z,
    number(car.speedMs, 0), tostring(p.splineLoaded), p.laneIndex or 0,
    number(p.laneOffset, 0))
end

return Driver
