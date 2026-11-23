-- RoutePilot/driver.lua — native CSP AI traffic controller.
-- Native AC AI follows the generated version-7 spline. This module does not
-- override steering, throttle, brake, clutch or gearbox.
-- Passing changes only the faster follower's spline offset; the leader is
-- never commanded to move aside or reduce speed.

local Driver = {}
local CFG, S, H

local PASS_TRIGGER_GAP = 48.0
local PASS_MIN_GAP = 6.0
local PASS_CLEAR_GAP = 7.0
local PASS_MAX_TIME = 18.0
local PASS_OFFSET_RATE = 3.0
local PASS_SEPARATION = 0.38
local PASS_EDGE_MARGIN = 0.22
local PASS_LOOKAHEAD = 42.0
local PASS_OCCUPANCY_RANGE = 23.0
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
    speed = math.max(0, number(car.speedMs, 0)), pace = p and p.targetLevel or number(car.aiLevel, 1.0),
    x = car.position.x, z = car.position.z
  }
end

local function signedGap(a, b, length)
  local d = b.progress - a.progress
  if d > length * 0.5 then d = d - length end
  if d < -length * 0.5 then d = d + length end
  return d
end

local function laneFits(seg, target, half, metrics)
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
  for index, other in pairs(snapshots) do
    if index ~= follower.index and index ~= leader.index then
      local gap = signedGap(follower, other, metrics.total)
      local longitudinalClose = math.abs(gap) < PASS_OCCUPANCY_RANGE
      if other.headingDot < 0.25 then
        local dx, dz = other.x - follower.x, other.z - follower.z
        longitudinalClose = dx * dx + dz * dz < 42 * 42
      end
      if longitudinalClose then
        local occupiedLat = other.lateral
        if other.p and other.p.passState and other.p.passState ~= 'RETURN' then occupiedLat = other.p.targetOffset end
        if math.abs(occupiedLat - target) < follower.half + other.half + PASS_SEPARATION then return false end
      end
    end
  end
  return true
end

local function choosePassOffset(follower, leader, snapshots, metrics)
  local separation = follower.half + leader.half + PASS_SEPARATION
  local leftTarget, rightTarget = leader.lateral - separation, leader.lateral + separation
  local leftFits = laneFits(follower.seg, leftTarget, follower.half, metrics)
  local rightFits = laneFits(follower.seg, rightTarget, follower.half, metrics)
  local leftFree = leftFits and laneIsFree(follower, leader, leftTarget, snapshots, metrics)
  local rightFree = rightFits and laneIsFree(follower, leader, rightTarget, snapshots, metrics)
  if leftFree then return leftTarget, -1 end
  if rightFree then return rightTarget, 1 end
  return nil, 0
end

local function ensureCar(index, car, dt)
  local p = S.p[index]
  if not p then
    local baseLevel = number(car.aiLevel, 1.0)
    if baseLevel <= 0 then baseLevel = 1.0 end
    local baseAggression = number(car.aiAggression, 0.95)
    local pace = 0.94 + deterministic(index) * 0.12
    p = {
      segIdx = 1, baseLevel = baseLevel, baseAggression = baseAggression,
      pace = pace, targetLevel = clamp(baseLevel * pace, 0.70, 1.15),
      splinePath = nil, splineLoaded = false, splineAge = 0, settingsApplied = false,
      currentOffset = 0, targetOffset = 0, passState = nil, passLeader = nil,
      passSide = 0, passTimer = 0, passLoggedTimeout = false,
      retireTimer = 0, stuckTimer = 0, lineDist = 0, lateral = 0
    }
    S.p[index] = p
    H.log('CAR_INIT id=%d aiLevel=%.3f pace=%.3f targetLevel=%.3f',
      index, baseLevel, pace, p.targetLevel)
  end

  p.splineAge = p.splineAge + dt
  if p.splinePath ~= S.aiSplinePath then
    p.splinePath, p.splineLoaded, p.splineAge = S.aiSplinePath, false, 0
    local ok, result = pcall(physics.setAISpline, index, S.aiSplinePath, nil)
    local exists = false
    pcall(function() exists = io and io.fileExists and io.fileExists(S.aiSplinePath) or false end)
    if ok and (result ~= false or exists) then
      p.splineLoaded = true
      H.log('AI_SPLINE_ASSIGNED id=%d file=%s result=%s', index, tostring(S.aiSplinePath), tostring(result))
    else
      H.log('AI_SPLINE_FAILED id=%d file=%s result=%s', index, tostring(S.aiSplinePath), tostring(result))
    end
  end

  if not p.settingsApplied then
    local okLevel, errLevel = pcall(physics.setAILevel, index, p.targetLevel)
    local okAgg, errAgg = pcall(physics.setAIAggression, index, 0.95)
    p.settingsApplied = okLevel and okAgg
    H.log('AI_PROFILE id=%d level=%.3f aggression=0.95 levelOK=%s aggressionOK=%s',
      index, p.targetLevel, tostring(okLevel), tostring(okAgg))
    if not okLevel then H.log('AI_PROFILE_ERROR id=%d kind=level err=%s', index, tostring(errLevel)) end
    if not okAgg then H.log('AI_PROFILE_ERROR id=%d kind=aggression err=%s', index, tostring(errAgg)) end
  end
  return p
end

function Driver.setup(cfg, state, helpers)
  CFG, S, H = cfg, state, helpers
  S.trafficHints = S.trafficHints or {}
end

function Driver.beginFrame(sim, dt)
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

  for i = 1, sim.carsCount - 1 do
    local p = S.p[i]
    if p and p.passState and p.passState ~= 'RETURN' then
      p.passTimer = p.passTimer + dt
      local follower, leader = snapshots[i], snapshots[p.passLeader]
      if not follower or not leader then
        p.passState, p.targetOffset = 'RETURN', 0
        H.log('PASS_ABORT car=%d leader=%s reason=car_unavailable', i, tostring(p.passLeader))
      else
        local gap = signedGap(follower, leader, metrics.total)
        if gap < -PASS_CLEAR_GAP then
          p.passState, p.targetOffset = 'RETURN', 0
          H.log('PASS_COMPLETE car=%d leader=%d gap=%.1f side=%d', i, p.passLeader, gap, p.passSide)
        elseif p.passTimer > PASS_MAX_TIME and not p.passLoggedTimeout then
          p.passLoggedTimeout = true
          H.log('PASS_LONG car=%d leader=%d gap=%.1f offset=%.2f', i, p.passLeader, gap, p.currentOffset)
        end
      end
    end
  end

  for i = 1, sim.carsCount - 1 do
    local follower, p = snapshots[i], S.p[i]
    if follower and p and p.splineLoaded and not p.passState and follower.headingDot > 0.35 then
      local bestLeader, bestGap = nil, PASS_TRIGGER_GAP
      for j, other in pairs(snapshots) do
        if j ~= i and other.headingDot > 0.35 then
          local gap = signedGap(follower, other, metrics.total)
          if gap > PASS_MIN_GAP and gap < bestGap and math.abs(other.lateral - follower.lateral) < 5.5 then
            bestLeader, bestGap = other, gap
          end
        end
      end
      if bestLeader then
        local faster = p.targetLevel > bestLeader.pace + 0.015 or follower.speed > bestLeader.speed + 0.8
        if faster then
          local target, side = choosePassOffset(follower, bestLeader, snapshots, metrics)
          if target then
            p.passState, p.passLeader, p.passSide = 'OUT', bestLeader.index, side
            p.passTimer, p.passLoggedTimeout, p.targetOffset = 0, false, target
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
  if p.passState or math.abs(p.currentOffset) > 0.02 or p.offsetApplied then
    local ok, err = pcall(physics.setAISplineAbsoluteOffset, index, p.currentOffset, false)
    if ok then
      p.offsetApplied = math.abs(p.currentOffset) > 0.02 or p.passState ~= nil
    else
      H.log('PASS_OFFSET_ERROR car=%d offset=%.2f err=%s', index, p.currentOffset, tostring(err))
      p.passState, p.targetOffset = 'RETURN', 0
    end
    if math.abs(previous - p.currentOffset) > 0.08 then
      H.log('PASS_OFFSET car=%d state=%s offset=%.2f target=%.2f',
        index, tostring(p.passState), p.currentOffset, p.targetOffset)
    end
  end

  if p.passState == 'RETURN' and math.abs(p.currentOffset) < 0.05 then
    H.log('PASS_RELEASE car=%d leader=%s', index, tostring(p.passLeader))
    p.passState, p.passLeader, p.passSide = nil, nil, 0
    p.passTimer, p.targetOffset, p.currentOffset = 0, 0, 0
    p.offsetApplied, p.passLoggedTimeout = false, false
  end

  if S.spawnDone and S.sessionTime > 10 and not car.isInPitlane then
    if number(car.speedMs, 0) < STUCK_SPEED then
      p.stuckTimer = p.stuckTimer + dt
      if p.stuckTimer >= STUCK_TIME then
        p.stuckTimer = 0
        local ok = H.teleportHome and H.teleportHome(index, car)
        H.log('RECOVERY_ATTEMPT car=%d success=%s pos=(%.2f,%.2f,%.2f)',
          index, tostring(ok), car.position.x, car.position.y, car.position.z)
      end
    else
      p.stuckTimer = 0
    end
  end
end

function Driver.isSplineReady(index)
  local p = S.p[index]
  return p ~= nil and p.splineLoaded and p.splineAge >= 0.8
end

function Driver.resetAfterTeleport(index)
  local p = S.p[index]
  if not p then return end
  p.passState, p.passLeader, p.passSide = nil, nil, 0
  p.passTimer, p.targetOffset, p.currentOffset = 0, 0, 0
  p.offsetApplied, p.stuckTimer = false, 0
  p.segIdx, p.lineDist, p.lateral = 1, 0, 0
  pcall(physics.setAISplineAbsoluteOffset, index, 0, false)
end

function Driver.release(index)
  local p = S.p[index]
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
end

function Driver.telemetryLine(index, car, p)
  return string.format(
    'CAR=%d POS=(%.2f,%.2f,%.2f) SPEED=%.2f AI_LEVEL=%.3f PACE=%.3f SPLINE=%s LINE_ERROR=%.2f SEG=%d LAT=%.2f PASS=%s LEADER=%s OFFSET=%.2f TARGET_OFFSET=%.2f',
    index, car.position.x, car.position.y, car.position.z, number(car.speedMs, 0),
    p.targetLevel or -1, p.pace or -1, tostring(p.splineLoaded), number(p.lineDist, -1),
    p.segIdx or -1, number(p.lateral, 0), tostring(p.passState or 'NONE'),
    tostring(p.passLeader or '-'), number(p.currentOffset, 0), number(p.targetOffset, 0)
  )
end

return Driver
