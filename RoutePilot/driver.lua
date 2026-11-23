-- RoutePilot/driver.lua — native CSP AI controller.
-- The recorded route, native spline builder, spawn teleport and recovery
-- remain owned by RoutePilot.lua/spline_builder.lua and are not modified here.
-- This module only assigns AI behaviour and fixed lateral lane offsets.

local Driver = {}
local CFG, S, H

local WIDTH_PRESETS = {3, 7, 11, 15}
local STUCK_SPEED = 0.8
local STUCK_TIME = 10.0
local LANE_RETRY = 1.0

local function clamp(x, a, b) return math.max(a, math.min(b, x)) end
local function number(x, fallback)
  if type(x) ~= 'number' or x ~= x or x == math.huge or x == -math.huge then
    return fallback or 0
  end
  return x
end
local function flatDistance(a, b)
  local dx, dz = b.x - a.x, b.z - a.z
  return math.sqrt(dx * dx + dz * dz)
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
  local seg, _, lineError = H.findNearestSeg(car.position, hint)
  if p then p.segIdx = seg end
  S.trafficHints[index] = seg
  local a, b = S.nodes[seg], H.segEnd(seg)
  local dx, dz = b.x - a.x, b.z - a.z
  local len = math.sqrt(dx * dx + dz * dz)
  if len < 0.001 then return nil end
  local tx, tz = dx / len, dz / len
  local lateral = H.lateralOf(car.position, seg)
  if p then
    p.lineDist, p.lateral = lineError, lateral
  end
  return {index = index, seg = seg, lateral = lateral}
end

local function laneForCar(index)
  local preset = clamp(math.floor(number(S.widthPreset, 1)), 1, 4)
  local roadWidth = WIDTH_PRESETS[preset]
  if roadWidth <= 3 then
    return 0, 0, 1.0, 1e9
  end

  -- CSP absolute spline offset: negative = left, positive = right.
  -- The two lane centres divide the existing recorded corridor in half.
  local laneHalf = roadWidth * 0.25
  if index % 2 == 1 then
    return 1, -laneHalf, 1.15, 1e9
  end
  return 2, laneHalf, 1.0, 40.0
end

local function ensureCar(index, car, dt)
  local p = S.p[index]
  if not p then
    local baseLevel = number(car.aiLevel, 1.0)
    if baseLevel < 0 then baseLevel = 1.0 end
    local baseAggression = number(car.aiAggression, 1.0)
    if baseAggression < 0 then baseAggression = 1.0 end
    local lane, laneOffset, targetLevel, topSpeed = laneForCar(index)
    p = {
      segIdx = 1,
      baseLevel = baseLevel,
      baseAggression = baseAggression,
      targetLevel = targetLevel,
      pace = targetLevel,
      laneIndex = lane,
      laneOffset = laneOffset,
      topSpeed = topSpeed,
      splinePath = nil,
      splineLoaded = false,
      splineAge = 0,
      splineRetry = 0,
      settingsApplied = false,
      speedApplied = false,
      offsetApplied = false,
      offsetRetry = 0,
      retireTimer = 0,
      stuckTimer = 0,
      airTimer = 0,
      lineDist = 0,
      lateral = 0
    }
    S.p[index] = p
    H.log('CAR_INIT id=%d lane=%d offset=%.2f targetLevel=%.3f topSpeed=%.1f',
      index, lane, laneOffset, targetLevel, topSpeed)
  end

  p.splineAge = p.splineAge + dt
  p.splineRetry = math.max(0, (p.splineRetry or 0) - dt)
  if p.splinePath ~= S.aiSplinePath or (not p.splineLoaded and p.splineRetry <= 0) then
    p.splinePath, p.splineLoaded, p.splineAge = S.aiSplinePath, false, 0
    p.splineRetry = 1.0
    p.settingsApplied, p.speedApplied, p.offsetApplied = false, false, false
    local ok, result = pcall(physics.setAISpline, index, S.aiSplinePath, nil)
    if ok and result == true then
      p.splineLoaded = true
      H.log('AI_SPLINE_ASSIGNED id=%d file=%s result=true',
        index, tostring(S.aiSplinePath))
    else
      H.log('AI_SPLINE_FAILED id=%d file=%s result=%s error=%s',
        index, tostring(S.aiSplinePath), tostring(result),
        tostring(ok and 'not accepted' or result))
    end
  end

  if p.splineLoaded and not p.settingsApplied then
    local okLevel, errLevel = pcall(physics.setAILevel, index, p.targetLevel)
    local okAgg, errAgg = pcall(physics.setAIAggression, index, 1.0)
    p.settingsApplied = okLevel and okAgg
    H.log('AI_PROFILE id=%d lane=%d level=%.3f aggression=1.00 levelOK=%s aggressionOK=%s',
      index, p.laneIndex, p.targetLevel, tostring(okLevel), tostring(okAgg))
    if not okLevel then H.log('AI_PROFILE_ERROR id=%d kind=level err=%s', index, tostring(errLevel)) end
    if not okAgg then H.log('AI_PROFILE_ERROR id=%d kind=aggression err=%s', index, tostring(errAgg)) end
  end

  if p.splineLoaded and not p.speedApplied then
    local ok, err = pcall(physics.setAITopSpeed, index, p.topSpeed)
    p.speedApplied = ok
    if ok then
      H.log('AI_SPEED_LIMIT id=%d lane=%d kmh=%.1f', index, p.laneIndex, p.topSpeed)
    else
      H.log('AI_SPEED_ERROR id=%d lane=%d err=%s', index, p.laneIndex, tostring(err))
    end
  end

  return p
end

function Driver.setup(cfg, state, helpers)
  CFG, S, H = cfg, state, helpers
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
  if not S.aiSplineBuilt or not S.aiSplinePath then return end

  local metrics = buildMetrics()
  if not metrics then
    S.status = 'Ошибка геометрии AI-линии'
    H.log('AI_CONTROL_WAIT reason=invalid_route_metrics')
    return
  end
  S.routeMetrics = metrics

  for i = 0, sim.carsCount - 1 do
    local car = ac.getCar(i)
    if i > 0 and car and car.isActive and car.isAIControlled and
      not car.isInPitlane and not car.isInPit then
      local p = ensureCar(i, car, dt)
      projectCar(i, car, p, metrics)
    end
  end
end

function Driver.update(index, car, dt)
  if S.customAIBlocked then return end
  local p = S.p[index]
  if not p then p = ensureCar(index, car, dt) end
  if not p.splineLoaded then return end

  -- Apply the fixed lane offset after spline assignment and after each
  -- teleport reset. No traffic planner, lane changes or per-frame steering.
  p.offsetRetry = math.max(0, (p.offsetRetry or 0) - dt)
  if not p.offsetApplied and p.offsetRetry <= 0 then
    p.offsetRetry = LANE_RETRY
    if math.abs(p.laneOffset) < 0.01 then
      p.offsetApplied = true
    else
      local ok, err = pcall(
        physics.setAISplineAbsoluteOffset, index, p.laneOffset, false)
      if ok then
        p.offsetApplied = true
        H.log('AI_LANE_FIXED id=%d lane=%d offset=%.2f',
          index, p.laneIndex, p.laneOffset)
      else
        H.log('AI_LANE_ERROR id=%d lane=%d offset=%.2f err=%s',
          index, p.laneIndex, p.laneOffset, tostring(err))
      end
    end
  end

  p.retireTimer = p.retireTimer + dt
  if p.retireTimer >= 2.0 then
    p.retireTimer = 0
    if physics.preventAIFromRetiring then
      pcall(physics.preventAIFromRetiring, index)
    end
  end

  if S.spawnDone and S.sessionTime > 10 and not car.isInPitlane then
    local speed = number(car.speedMs, 0)
    if speed < STUCK_SPEED then
      p.stuckTimer = p.stuckTimer + dt
    else
      p.stuckTimer = 0
    end

    local loadedWheels, hasWheelData = 0, false
    pcall(function()
      if car.wheels then
        for wheelIndex = 0, 3 do
          local wheel = car.wheels[wheelIndex]
          if wheel then
            hasWheelData = true
            if number(wheel.load, 0) > 120 then
              loadedWheels = loadedWheels + 1
            end
          end
        end
      end
    end)

    if hasWheelData and loadedWheels == 0 and speed > 3.0 then
      p.airTimer = p.airTimer + dt
    else
      p.airTimer = 0
    end

    local reason = nil
    if p.stuckTimer >= STUCK_TIME then
      p.stuckTimer = 0
      reason = 'stuck'
    elseif p.airTimer >= 10.0 then
      p.airTimer = 0
      reason = 'airborne'
    end
    if reason then
      local ok = H.teleportHome and H.teleportHome(index, car)
      H.log('RECOVERY_ATTEMPT car=%d reason=%s success=%s pos=(%.2f,%.2f,%.2f) loadedWheels=%d',
        index, reason, tostring(ok), car.position.x, car.position.y,
        car.position.z, loadedWheels)
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
  p.offsetApplied = false
  p.offsetRetry = 0
  p.stuckTimer, p.airTimer = 0, 0
  p.segIdx, p.lineDist, p.lateral = 1, 0, 0
  -- Do not touch the AI spline or teleport coordinates here. RoutePilot.lua
  -- owns the actual recovery position and orientation.
end

function Driver.release(index)
  local p = S.p[index]
  if not p then return end
  pcall(physics.setAISplineAbsoluteOffset, index, 0, false)
  pcall(physics.setAISpline, index, nil, nil)
  if p.baseLevel then pcall(physics.setAILevel, index, p.baseLevel) end
  if p.baseAggression then pcall(physics.setAIAggression, index, p.baseAggression) end
  if physics.setAITopSpeed then pcall(physics.setAITopSpeed, index, 1e9) end
  S.p[index], S.trafficHints[index] = nil, nil
end

function Driver.invalidateAll()
  for index in pairs(S.p) do Driver.release(index) end
  S.p, S.trafficHints, S.routeMetrics = {}, {}, nil
end

function Driver.telemetryLine(index, car, p)
  return string.format(
    'CAR=%d POS=(%.2f,%.2f,%.2f) SPEED=%.2f AI_LEVEL=%.3f SPLINE=%s LINE_ERROR=%.2f SEG=%d LAT=%.2f LANE=%d LANE_OFFSET=%.2f TOP_SPEED=%.1f',
    index, car.position.x, car.position.y, car.position.z,
    number(car.speedMs, 0), p.targetLevel or -1, tostring(p.splineLoaded),
    number(p.lineDist, -1), p.segIdx or -1, number(p.lateral, 0),
    p.laneIndex or 0, number(p.laneOffset, 0), number(p.topSpeed, 0)
  )
end

return Driver
