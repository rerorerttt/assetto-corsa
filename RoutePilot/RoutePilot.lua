-- RoutePilot.lua
-- RoutePilot 0.3 - Stable Route Following Controller
-- CSP Lua Apps
-- Controls: steering, throttle, brake only.
-- Gearbox and transmission remain under vehicle control.

local okLogger, Logger = pcall(require, "logger")
if not okLogger or type(Logger) ~= "table" then
  Logger = nil
end

------------------------------------------------------------
-- CONFIGURATION
------------------------------------------------------------

local CFG = {
  LOOKAHEAD_BASE = 10.0,
  LOOKAHEAD_SPEED = 0.42,
  LOOKAHEAD_MIN = 10.0,
  LOOKAHEAD_MAX = 25.0,

  STEER_GAIN = 0.72,
  STEER_LIMIT = 0.75,
  STEER_RATE = 2.0,
  STEER_FILTER = 0.18,

  SLIP_START = 0.09,
  SLIP_FULL = 0.38,

  MAX_LATERAL_ACCEL = 5.5,
  LATERAL_ACCEL_LIMIT = 11.0,

  BRAKE_ACCEL = 5.0,
  MAX_BRAKE = 0.75,
  MAX_GAS = 0.90,

  SPEED_GAIN = 0.28,

  FOLLOW_LATERAL = 3.0,
  FOLLOW_DISTANCE = 8.0,

  RECORD_DISTANCE = 3.0,

  SPAWN_GAP = 22.0,
  SPAWN_MIN_GAP = 14.0,
  SPAWN_STABLE_TIME = 0.35,
  SPAWN_WAIT_TIMEOUT = 8.0,

  LOG_INTERVAL = 1.0,
  MAX_ROUTE_POINTS = 50000,
}

------------------------------------------------------------
-- STATE
------------------------------------------------------------

local state = {
  enabled = true,
  recording = false,
  loopRoute = true,

  speedMin = 60,
  speedMax = 140,

  followGap = 8.0,

  spawnDone = false,
  spawnPerformed = false,
  spawnWait = 0,
  spawnStable = 0,
  spawnLastRoster = "",

  nodes = {},
  p = {},

  status = "Готов",
  logAccum = 0,

  loaded = false,
}

local logger = nil

if Logger and type(Logger.new) == "function" then
  local ok, result = pcall(function()
    local l = Logger.new("routepilot_trace.txt", 6000)
    l:init()
    return l
  end)

  if ok then
    logger = result
  end
end

------------------------------------------------------------
-- LOGGING
------------------------------------------------------------

local function writeLog(fmt, ...)
  if not logger or not logger.log then return end
  pcall(logger.log, logger, fmt, ...)
end

local function flushLog()
  if logger and logger.flush then
    pcall(logger.flush, logger)
  end
end

------------------------------------------------------------
-- MATH
------------------------------------------------------------

local function clamp(v, lo, hi)
  return math.max(lo, math.min(hi, v))
end

local function lerp(a, b, t)
  return a + (b - a) * t
end

local function flatDist(a, b)
  local dx = b.x - a.x
  local dz = b.z - a.z
  return math.sqrt(dx * dx + dz * dz)
end

local function normalize2(x, z)
  local l = math.sqrt(x * x + z * z)

  if l < 0.00001 then
    return 0, 1
  end

  return x / l, z / l
end

local function signedAngle(ax, az, bx, bz)
  local dot = clamp(ax * bx + az * bz, -1, 1)
  local cross = ax * bz - az * bx

  return math.atan2(cross, dot)
end

local function safeNumber(v, fallback)
  if type(v) ~= "number" then
    return fallback or 0
  end

  if v ~= v or v == math.huge or v == -math.huge then
    return fallback or 0
  end

  return v
end

------------------------------------------------------------
-- TRACK AND FILES
------------------------------------------------------------

local function getTrackTag()
  local tid = (ac.getTrackID and ac.getTrackID()) or "track"
  local layout = (ac.getTrackLayout and ac.getTrackLayout()) or ""

  if layout ~= "" then
    return tostring(tid) .. "_" .. tostring(layout)
  end

  return tostring(tid)
end

local function getLineFilePath()
  local appDir = ""

  if ac.getFolder and ac.FolderID then
    appDir = ac.getFolder(ac.FolderID.ACApps) or ""
  end

  local tag = string.gsub(
    getTrackTag(),
    "[^%w_%-]",
    "_"
  )

  if appDir ~= "" then
    return appDir .. "/lua/RoutePilot/line_" .. tag .. ".txt"
  end

  return "line_" .. tag .. ".txt"
end

------------------------------------------------------------
-- ROUTE SERIALIZATION
------------------------------------------------------------

local function serializeNodes(nodes)
  local lines = {}

  for i = 1, #nodes do
    local n = nodes[i]

    lines[#lines + 1] = string.format(
      "%.4f %.4f %.4f",
      n.x,
      n.y,
      n.z
    )
  end

  return table.concat(lines, "\n")
end

local function parseNodes(raw)
  if type(raw) ~= "string" or raw == "" then
    return nil
  end

  local loaded = {}

  for line in string.gmatch(raw, "[^\r\n]+") do
    local xs, ys, zs = string.match(
      line,
      "^%s*([%-0-9%.]+)%s+([%-0-9%.]+)%s+([%-0-9%.]+)%s*$"
    )

    local x = tonumber(xs)
    local y = tonumber(ys)
    local z = tonumber(zs)

    if x and y and z then
      if x == x and y == y and z == z then
        loaded[#loaded + 1] = vec3(x, y, z)
      end
    end

    if #loaded >= CFG.MAX_ROUTE_POINTS then
      break
    end
  end

  if #loaded >= 2 then
    return loaded
  end

  return nil
end

------------------------------------------------------------
-- SAVE ROUTE
------------------------------------------------------------

local function saveLine()
  if #state.nodes < 2 then
    state.status = "Недостаточно точек"
    return
  end

  local data = serializeNodes(state.nodes)
  local wroteFile = false
  local wroteStorage = false

  local path = getLineFilePath()

  if io and io.open then
    local ok, f = pcall(io.open, path, "w")

    if ok and f then
      local result = pcall(function()
        f:write(data)
        f:close()
      end)

      wroteFile = result
    end
  end

  if ac.storage then
    local ok = pcall(
      ac.storage,
      "RP_LINE_DATA_" .. getTrackTag(),
      data
    )

    wroteStorage = ok
  end

  if wroteFile or wroteStorage then
    state.status = string.format(
      "Сохранено: %d точек",
      #state.nodes
    )

    writeLog(
      "SAVE_ROUTE points=%d file=%s storage=%s",
      #state.nodes,
      tostring(wroteFile),
      tostring(wroteStorage)
    )

    flushLog()
  else
    state.status = "Ошибка сохранения"
  end
end

------------------------------------------------------------
-- LOAD ROUTE
------------------------------------------------------------

local function loadLine()
  local loaded = nil
  local path = getLineFilePath()

  if io and io.open then
    local ok, f = pcall(io.open, path, "r")

    if ok and f then
      local success, raw = pcall(function()
        local content = f:read("*all")
        f:close()
        return content
      end)

      if success then
        loaded = parseNodes(raw)
      end
    end
  end

  if not loaded and ac.storage then
    local ok, raw = pcall(
      ac.storage,
      "RP_LINE_DATA_" .. getTrackTag()
    )

    if ok and raw then
      loaded = parseNodes(raw)
    end
  end

  if loaded then
    state.nodes = loaded
    state.p = {}
    state.loaded = true

    if not state.spawnPerformed then
      state.spawnDone = false
      state.spawnWait = 0
      state.spawnStable = 0
      state.spawnLastRoster = ""
    end

    state.status = string.format(
      "Загружено: %d точек",
      #loaded
    )

    writeLog(
      "LOAD_ROUTE points=%d track=%s",
      #loaded,
      getTrackTag()
    )

    flushLog()
    return true
  end

  state.status = "Линия не найдена"
  state.loaded = false

  return false
end

------------------------------------------------------------
-- STARTUP AI TELEPORT
------------------------------------------------------------

local function buildSpawnRouteMetrics()
  local count = #state.nodes
  if count < 2 then return nil end

  local segmentCount = state.loopRoute and count or count - 1
  local segments = {}
  local totalLength = 0

  for i = 1, segmentCount do
    local a = state.nodes[i]
    local b = state.nodes[(i % count) + 1]
    local length = flatDist(a, b)

    if length > 0.05 then
      segments[#segments + 1] = {
        a = a,
        b = b,
        start = totalLength,
        length = length
      }
      totalLength = totalLength + length
    end
  end

  if #segments < 2 or totalLength < CFG.SPAWN_MIN_GAP * 2 then
    return nil
  end

  return {
    segments = segments,
    length = totalLength,
    loop = state.loopRoute
  }
end

local function findRouteDistanceAtPosition(position, metrics)
  local bestDistanceSq = math.huge
  local bestRouteDistance = 0

  for i = 1, #metrics.segments do
    local seg = metrics.segments[i]
    local a, b = seg.a, seg.b

    local vx = b.x - a.x
    local vz = b.z - a.z
    local lenSq = vx * vx + vz * vz

    if lenSq > 0.0001 then
      local t = clamp(
        ((position.x - a.x) * vx + (position.z - a.z) * vz) / lenSq,
        0,
        1
      )

      local px = a.x + vx * t
      local pz = a.z + vz * t
      local dx = position.x - px
      local dz = position.z - pz
      local distanceSq = dx * dx + dz * dz

      if distanceSq < bestDistanceSq then
        bestDistanceSq = distanceSq
        bestRouteDistance = seg.start + seg.length * t
      end
    end
  end

  return bestRouteDistance, math.sqrt(bestDistanceSq)
end

local function sampleSpawnRoute(metrics, distance)
  if metrics.loop then
    distance = distance % metrics.length
  else
    distance = clamp(distance, 0, metrics.length)
  end

  for i = 1, #metrics.segments do
    local seg = metrics.segments[i]
    local finish = seg.start + seg.length

    if distance <= finish or i == #metrics.segments then
      local t = clamp((distance - seg.start) / seg.length, 0, 1)
      local a, b = seg.a, seg.b

      local position = vec3(
        lerp(a.x, b.x, t),
        lerp(a.y, b.y, t),
        lerp(a.z, b.z, t)
      )

      -- physics.setAICarPosition() expects the direction vector opposite
      -- to ac.CarState.look (the car's forward vector). Keep the spawned
      -- vehicle facing along the recorded route, not against it.
      local direction = vec3(
        a.x - b.x,
        a.y - b.y,
        a.z - b.z
      )

      if direction:length() < 0.0001 then
        return nil, nil
      end

      direction:normalize()
      return position, direction
    end
  end

  return nil, nil
end

local function collectActiveAICars(sim)
  local cars = {}

  -- Index 0 is the player's car. Only active AI cars are repositioned.
  for i = 1, sim.carsCount - 1 do
    local car = ac.getCar(i)

    if car and car.isActive and car.isAIControlled then
      cars[#cars + 1] = i
    end
  end

  return cars
end

local function teleportAICarsToRoute(aiCars)
  -- Latch before native calls: even a partial failure must not trigger
  -- a second mass teleport after the route is loaded again.
  state.spawnPerformed = true

  if type(physics) ~= "table" or type(physics.setAICarPosition) ~= "function" then
    state.status = "CSP: physics.setAICarPosition недоступна"
    writeLog("SPAWN_ABORT reason=setAICarPosition_unavailable")
    state.spawnDone = true
    flushLog()
    return
  end

  local metrics = buildSpawnRouteMetrics()
  if not metrics then
    state.status = "Ошибка: линия слишком короткая"
    writeLog("SPAWN_ABORT reason=invalid_route points=%d", #state.nodes)
    state.spawnDone = true
    flushLog()
    return
  end

  local count = #aiCars
  local gap = math.min(
    CFG.SPAWN_GAP,
    metrics.length / (count + 1)
  )

  -- Do not compress the grid below the safe minimum.
  if gap < CFG.SPAWN_MIN_GAP then
    state.status = "Недостаточно места для безопасной расстановки"
    writeLog(
      "SPAWN_ABORT reason=route_too_short cars=%d route=%.1f gap=%.1f",
      count,
      metrics.length,
      gap
    )
    state.spawnDone = true
    flushLog()
    return
  end

  local startDistance = 0

  if metrics.loop then
    local player = ac.getCar(0)
    if player then
      local nearestDistance, playerLineError =
        findRouteDistanceAtPosition(player.position, metrics)

      -- Keep the first AI car one full interval ahead of the player.
      startDistance = nearestDistance + gap

      writeLog(
        "SPAWN_ANCHOR player_line_error=%.2f route_distance=%.2f",
        playerLineError,
        nearestDistance
      )
    else
      startDistance = gap
    end
  end

  local moved = 0
  local failed = 0

  writeLog(
    "SPAWN_BEGIN cars=%d route=%.1f gap=%.1f start=%.1f loop=%s",
    count,
    metrics.length,
    gap,
    startDistance,
    tostring(metrics.loop)
  )

  -- All native teleports are issued in one update. The route controller
  -- resumes on the following frame, after the entire AI chain is positioned.
  for order = 1, count do
    local carIndex = aiCars[order]
    local routeDistance = startDistance + (order - 1) * gap
    local position, direction = sampleSpawnRoute(metrics, routeDistance)

    if position and direction then
      local ok, result = pcall(
        physics.setAICarPosition,
        carIndex,
        position,
        direction
      )

      if ok and result ~= false then
        moved = moved + 1
        state.p[carIndex] = nil

        writeLog(
          "SPAWN_CAR id=%d order=%d distance=%.2f pos=(%.2f,%.2f,%.2f) dir=(%.3f,%.3f,%.3f)",
          carIndex,
          order,
          routeDistance,
          position.x,
          position.y,
          position.z,
          direction.x,
          direction.y,
          direction.z
        )
      else
        failed = failed + 1
        writeLog(
          "SPAWN_CAR_FAILED id=%d order=%d error=%s",
          carIndex,
          order,
          tostring(result)
        )
      end
    else
      failed = failed + 1
      writeLog("SPAWN_CAR_FAILED id=%d order=%d reason=invalid_sample", carIndex, order)
    end
  end

  state.spawnDone = true

  if failed == 0 then
    state.status = string.format(
      "ИИ расставлен: %d машин, интервал %.0f м",
      moved,
      gap
    )
  else
    state.status = string.format(
      "Телепортировано %d из %d; ошибок: %d",
      moved,
      count,
      failed
    )
  end

  writeLog(
    "SPAWN_FINISH moved=%d failed=%d total=%d gap=%.2f",
    moved,
    failed,
    count,
    gap
  )

  flushLog()
end

local function updateStartupTeleport(sim, dt)
  if state.spawnDone then return true end
  if not state.loaded or state.recording or #state.nodes < 2 then return true end

  state.spawnWait = state.spawnWait + dt

  local aiCars = collectActiveAICars(sim)
  local count = #aiCars
  local rosterParts = {}

  for i = 1, count do
    rosterParts[i] = tostring(aiCars[i])
  end

  local roster = table.concat(rosterParts, ",")

  if count == 0 then
    state.spawnStable = 0
    state.spawnLastRoster = ""

    if state.spawnWait >= CFG.SPAWN_WAIT_TIMEOUT then
      state.spawnDone = true
      state.status = "ИИ-машины не обнаружены"
      writeLog("SPAWN_SKIP reason=no_active_ai timeout=%.1f", state.spawnWait)
      flushLog()
      return true
    end

    state.status = "Ожидание загрузки ИИ-машин..."
    return false
  end

  -- Compare actual car indices, not only the number of cars.
  if roster ~= state.spawnLastRoster then
    state.spawnLastRoster = roster
    state.spawnStable = 0
    state.status = string.format("Подготовка расстановки: %d ИИ", count)
    return false
  end

  state.spawnStable = state.spawnStable + dt

  if state.spawnStable < CFG.SPAWN_STABLE_TIME then
    state.status = string.format("Подготовка расстановки: %d ИИ", count)
    return false
  end

  teleportAICarsToRoute(aiCars)
  return false
end

------------------------------------------------------------
-- RECORD PLAYER ROUTE
------------------------------------------------------------

local function addPoint(pos)
  if #state.nodes >= CFG.MAX_ROUTE_POINTS then
    state.recording = false
    state.status = "Достигнут лимит точек"
    return
  end

  if #state.nodes == 0 or
     flatDist(state.nodes[#state.nodes], pos) >= CFG.RECORD_DISTANCE then

    state.nodes[#state.nodes + 1] = vec3(
      pos.x,
      pos.y + 0.1,
      pos.z
    )
  end
end

------------------------------------------------------------
-- CAR STATE
------------------------------------------------------------

local function newPersonality(carIndex)
  local span = math.max(0, state.speedMax - state.speedMin)

  local kmh = state.speedMin + span * 0.5

  local p = {
    segIdx = 1,

    cruiseSpeed = kmh / 3.6,

    steer = 0,
    steerFiltered = 0,

    gas = 0,
    brake = 0,

    slipAngle = 0,
    lateralVelocity = 0,
    lateralAccel = 0,
    yawRate = 0,

    previousFx = nil,
    previousFz = nil,
    previousLatVelocity = 0,

    stuck = 0,

    lineDist = 0,

    targetX = 0,
    targetZ = 0,

    targetSpeed = 0,

    slipSeverity = 0,
    stability = 1,

    lastCurve = 0,
  }

  local car = ac.getCar(carIndex)

  if car and #state.nodes >= 2 then
    local bestI = 1
    local bestD = math.huge

    for i = 1, #state.nodes do
      local d = flatDist(car.position, state.nodes[i])

      if d < bestD then
        bestD = d
        bestI = i
      end
    end

    p.segIdx = bestI
  end

  state.p[carIndex] = p

  writeLog(
    "CAR_INIT id=%d cruise=%.1fkmh segment=%d",
    carIndex,
    kmh,
    p.segIdx
  )
end

------------------------------------------------------------
-- ROUTE GEOMETRY
------------------------------------------------------------

local function getSegmentEnd(index, count)
  if state.loopRoute then
    return state.nodes[(index % count) + 1]
  end

  return state.nodes[index + 1]
end

local function findNearestSegment(car, p)
  local nodes = state.nodes
  local count = #nodes

  local segmentCount = state.loopRoute and count or count - 1

  local px = car.position.x
  local pz = car.position.z

  local bestSeg = clamp(p.segIdx, 1, segmentCount)
  local bestT = 0
  local bestDist = math.huge

  local function checkSegment(index)
    local a = nodes[index]
    local b = getSegmentEnd(index, count)

    local vx = b.x - a.x
    local vz = b.z - a.z

    local lenSq = vx * vx + vz * vz

    if lenSq < 0.00001 then
      return
    end

    local t = clamp(
      ((px - a.x) * vx + (pz - a.z) * vz) / lenSq,
      0,
      1
    )

    local cx = a.x + vx * t
    local cz = a.z + vz * t

    local dx = px - cx
    local dz = pz - cz

    local dSq = dx * dx + dz * dz

    if dSq < bestDist then
      bestDist = dSq
      bestSeg = index
      bestT = t
    end
  end

  local back = math.min(3, segmentCount - 1)
  local ahead = math.min(
    segmentCount - 1,
    math.max(15, math.floor(segmentCount * 0.20))
  )

  for offset = -back, ahead do
    local idx

    if state.loopRoute then
      idx = ((p.segIdx - 1 + offset) % segmentCount) + 1
    else
      idx = p.segIdx + offset

      if idx >= 1 and idx <= segmentCount then
        checkSegment(idx)
      end

      idx = nil
    end

    if idx then
      checkSegment(idx)
    end
  end

  -- Recover if the vehicle has moved far from the expected segment.
  if bestDist > 20 * 20 then
    for i = 1, segmentCount do
      checkSegment(i)
    end
  end

  p.segIdx = bestSeg

  return bestSeg, bestT, math.sqrt(bestDist), segmentCount
end

------------------------------------------------------------
-- LOOKAHEAD TARGET
------------------------------------------------------------

local function getLookaheadTarget(car, p, speed)
  local nodes = state.nodes
  local count = #nodes

  local seg, t = findNearestSegment(car, p)

  local a = nodes[seg]
  local b = getSegmentEnd(seg, count)

  local startX = lerp(a.x, b.x, t)
  local startZ = lerp(a.z, b.z, t)

  local lookAhead = clamp(
    CFG.LOOKAHEAD_BASE + speed * CFG.LOOKAHEAD_SPEED,
    CFG.LOOKAHEAD_MIN,
    CFG.LOOKAHEAD_MAX
  )

  local remaining = lookAhead
  local currentSeg = seg

  local targetX = b.x
  local targetZ = b.z

  local steps = 0
  local segmentCount = state.loopRoute and count or count - 1

  while remaining > 0 and steps < segmentCount do
    steps = steps + 1

    local sa = nodes[currentSeg]
    local sb = getSegmentEnd(currentSeg, count)

    local sx = steps == 1 and startX or sa.x
    local sz = steps == 1 and startZ or sa.z

    local dx = sb.x - sx
    local dz = sb.z - sz

    local length = math.sqrt(dx * dx + dz * dz)

    if length < 0.0001 then
      break
    end

    if length >= remaining then
      local ratio = remaining / length

      targetX = sx + dx * ratio
      targetZ = sz + dz * ratio

      remaining = 0
    else
      remaining = remaining - length

      targetX = sb.x
      targetZ = sb.z

      if state.loopRoute then
        currentSeg = currentSeg % segmentCount + 1
      elseif currentSeg < segmentCount then
        currentSeg = currentSeg + 1
      else
        break
      end
    end
  end

  return targetX, targetZ
end

------------------------------------------------------------
-- CURVATURE AND SPEED PLANNER
------------------------------------------------------------

local function getVertexCurvature(index)
  local nodes = state.nodes
  local count = #nodes

  local prevIndex = index - 1
  local nextIndex = index + 1

  if state.loopRoute then
    if prevIndex < 1 then prevIndex = count end
    if nextIndex > count then nextIndex = 1 end
  else
    if prevIndex < 1 or nextIndex > count then
      return 0
    end
  end

  local a = nodes[prevIndex]
  local b = nodes[index]
  local c = nodes[nextIndex]

  local v1x = b.x - a.x
  local v1z = b.z - a.z

  local v2x = c.x - b.x
  local v2z = c.z - b.z

  local l1 = math.sqrt(v1x * v1x + v1z * v1z)
  local l2 = math.sqrt(v2x * v2x + v2z * v2z)

  if l1 < 0.1 or l2 < 0.1 then
    return 0
  end

  local dot = clamp(
    (v1x * v2x + v1z * v2z) / (l1 * l2),
    -1,
    1
  )

  local angle = math.acos(dot)

  local averageLength = (l1 + l2) * 0.5

  return angle / math.max(0.1, averageLength)
end

local function planTargetSpeed(p, speed)
  local nodes = state.nodes
  local count = #nodes

  local segmentCount = state.loopRoute and count or count - 1

  local targetSpeed = p.cruiseSpeed
  local maxCurve = 0

  local distance = 0
  local current = p.segIdx

  -- Look ahead through upcoming corners.
  for offset = 0, 12 do
    local index = current + offset

    if state.loopRoute then
      index = ((index - 1) % count) + 1
    elseif index > segmentCount then
      break
    end

    local curvature = getVertexCurvature(index)

    maxCurve = math.max(maxCurve, curvature)

    if curvature > 0.001 then
      local cornerSpeed = math.sqrt(
        CFG.MAX_LATERAL_ACCEL / curvature
      )

      -- Allow braking distance before reaching the corner.
      local safeSpeed = math.sqrt(
        cornerSpeed * cornerSpeed +
        2 * CFG.BRAKE_ACCEL * distance
      )

      targetSpeed = math.min(targetSpeed, safeSpeed)
    end

    local a = nodes[index]
    local b = getSegmentEnd(index, count)

    distance = distance + flatDist(a, b)
  end

  p.lastCurve = maxCurve

  return math.max(0, targetSpeed)
end

------------------------------------------------------------
-- VEHICLE DYNAMICS
------------------------------------------------------------

local function readVehicleDynamics(car, p, dt)
  local velocity = car.velocity
  local look = car.look

  local fx, fz = normalize2(look.x, look.z)

  -- Local right direction.
  local rx = fz
  local rz = -fx

  local vx = safeNumber(velocity.x)
  local vz = safeNumber(velocity.z)

  local longitudinalVelocity = vx * fx + vz * fz
  local lateralVelocity = vx * rx + vz * rz

  -- Actual direction of travel versus body direction.
  local slipAngle = math.atan2(
    lateralVelocity,
    math.max(0.5, longitudinalVelocity)
  )

  local yawRate = 0

  if p.previousFx and dt > 0.0001 then
    local headingChange = signedAngle(
      p.previousFx,
      p.previousFz,
      fx,
      fz
    )

    yawRate = headingChange / dt
  end

  local speed = math.max(
    0,
    safeNumber(car.speedMs, longitudinalVelocity)
  )

  -- Approximate lateral acceleration from angular motion.
  local lateralAccel = math.abs(yawRate * speed)

  p.previousFx = fx
  p.previousFz = fz

  p.previousLatVelocity = lateralVelocity

  p.lateralVelocity = lateralVelocity
  p.slipAngle = slipAngle
  p.yawRate = yawRate
  p.lateralAccel = lateralAccel

  return speed, fx, fz
end

------------------------------------------------------------
-- SLIP DETECTION
------------------------------------------------------------

local function calculateStability(p)
  local slip = math.abs(p.slipAngle)

  local slipSeverity = clamp(
    (slip - CFG.SLIP_START) /
    (CFG.SLIP_FULL - CFG.SLIP_START),
    0,
    1
  )

  local accelerationSeverity = clamp(
    (p.lateralAccel - CFG.MAX_LATERAL_ACCEL) /
    (CFG.LATERAL_ACCEL_LIMIT - CFG.MAX_LATERAL_ACCEL),
    0,
    1
  )

  local severity = math.max(
    slipSeverity,
    accelerationSeverity
  )

  p.slipSeverity = severity
  p.stability = 1 - severity

  return severity
end

------------------------------------------------------------
-- PURE PURSUIT STEERING
------------------------------------------------------------

local function calculateSteering(car, p, speed, fx, fz)
  local targetX, targetZ = getLookaheadTarget(car, p, speed)

  p.targetX = targetX
  p.targetZ = targetZ

  local dx = targetX - car.position.x
  local dz = targetZ - car.position.z

  local distance = math.sqrt(dx * dx + dz * dz)

  if distance < 0.001 then
    return 0
  end

  local dirX = dx / distance
  local dirZ = dz / distance

  local angle = signedAngle(
    fx,
    fz,
    dirX,
    dirZ
  )

  -- Basic route following.
  local routeSteer = angle * CFG.STEER_GAIN

  -- Countersteering compensation.
  -- When the vehicle moves sideways, the controller compensates
  -- for the actual velocity direction instead of chasing the
  -- route with the body heading alone.
  local slipCompensation = p.slipAngle * 0.62

  local steer = routeSteer + slipCompensation

  return clamp(
    steer,
    -CFG.STEER_LIMIT,
    CFG.STEER_LIMIT
  )
end

------------------------------------------------------------
-- STEERING FILTER
------------------------------------------------------------

local function filterSteering(p, target, dt)
  local tau = CFG.STEER_FILTER

  local alpha = clamp(
    dt / (tau + dt),
    0,
    1
  )

  local filtered = lerp(
    p.steerFiltered,
    target,
    alpha
  )

  -- Limit steering speed to suppress oscillation.
  local maxChange = CFG.STEER_RATE * dt

  local delta = filtered - p.steer

  delta = clamp(
    delta,
    -maxChange,
    maxChange
  )

  p.steer = clamp(
    p.steer + delta,
    -CFG.STEER_LIMIT,
    CFG.STEER_LIMIT
  )

  p.steerFiltered = filtered

  return p.steer
end

------------------------------------------------------------
-- CAR FOLLOWING
------------------------------------------------------------

local function carAhead(carIndex, car)
  local fx, fz = normalize2(car.look.x, car.look.z)

  local sim = ac.getSim()

  if not sim then
    return nil, nil
  end

  local bestGap = nil
  local bestSpeed = nil

  for i = 0, sim.carsCount - 1 do
    if i ~= carIndex then
      local other = ac.getCar(i)

      if other and other.isActive then
        local dx = other.position.x - car.position.x
        local dz = other.position.z - car.position.z

        local forward = dx * fx + dz * fz

        if forward > 0 and forward < 80 then
          local lateral = math.abs(
            dx * fz - dz * fx
          )

          if lateral <= CFG.FOLLOW_LATERAL then
            if not bestGap or forward < bestGap then
              bestGap = forward
              bestSpeed = math.max(
                0,
                safeNumber(other.speedMs)
              )
            end
          end
        end
      end
    end
  end

  return bestGap, bestSpeed
end

------------------------------------------------------------
-- THROTTLE AND BRAKE CONTROLLER
------------------------------------------------------------

local function calculateThrottle(p, speed, targetSpeed, dt)
  local error = targetSpeed - speed

  local gas = clamp(
    error * CFG.SPEED_GAIN,
    0,
    CFG.MAX_GAS
  )

  local brake = clamp(
    -error * 0.30,
    0,
    CFG.MAX_BRAKE
  )

  local stability = p.stability

  -- Progressive throttle reduction during loss of grip.
  -- The more sideways the vehicle moves, the less power it receives.
  local throttleFactor = lerp(
    1.0,
    0.05,
    p.slipSeverity
  )

  gas = gas * throttleFactor

  -- Do not apply aggressive braking simply because the car is
  -- sliding. First reduce engine torque and allow the vehicle
  -- to regain stability.
  if p.slipSeverity > 0.80 then
    gas = math.min(gas, 0.08)
  end

  -- If speed is already below target, do not brake.
  if error >= 0 then
    brake = 0
  end

  p.gas = clamp(gas, 0, CFG.MAX_GAS)
  p.brake = clamp(brake, 0, CFG.MAX_BRAKE)

  return p.gas, p.brake
end

------------------------------------------------------------
-- RELEASE CONTROL
------------------------------------------------------------

local function releaseCar(carIndex)
  local ok, controls = pcall(
    ac.overrideCarControls,
    carIndex
  )

  if ok and controls then
    controls.combineAxis = false
    controls.steer = 0
    controls.gas = 0
    controls.brake = 0
    controls.handbrake = 0
  end

  state.p[carIndex] = nil
end

------------------------------------------------------------
-- MAIN AI CONTROL
------------------------------------------------------------

local function updateLineCar(carIndex, car, dt)
  local p = state.p[carIndex]

  if not p then
    newPersonality(carIndex)
    p = state.p[carIndex]
  end

  local speed, fx, fz = readVehicleDynamics(
    car,
    p,
    dt
  )

  -- 1. Detect vehicle instability.
  calculateStability(p)

  -- 2. Calculate steering using the recorded route.
  local steerTarget = calculateSteering(
    car,
    p,
    speed,
    fx,
    fz
  )

  -- 3. Smooth steering.
  local steer = filterSteering(
    p,
    steerTarget,
    dt
  )

  -- 4. Calculate speed required for upcoming corners.
  local targetSpeed = planTargetSpeed(p, speed)

  -- 5. Avoid the vehicle ahead.
  local gap, leaderSpeed = carAhead(
    carIndex,
    car
  )

  if gap then
    local safeSpeed = math.sqrt(
      math.max(
        0,
        leaderSpeed * leaderSpeed +
        2 * CFG.BRAKE_ACCEL *
        math.max(0, gap - state.followGap)
      )
    )

    targetSpeed = math.min(
      targetSpeed,
      safeSpeed
    )

    if gap < state.followGap then
      targetSpeed = 0
    end
  end

  p.targetSpeed = targetSpeed

  -- 6. Adapt throttle to grip and actual vehicle movement.
  local gas, brake = calculateThrottle(
    p,
    speed,
    targetSpeed,
    dt
  )

  -- 7. Recover from a stationary vehicle without changing
  -- gearbox, clutch or transmission state.
  if speed < 0.5 then
    p.stuck = p.stuck + dt
  else
    p.stuck = 0
  end

  -- 8. Send controls to the AI car.
  local ok, controls = pcall(
    ac.overrideCarControls,
    carIndex
  )

  if ok and controls then
    controls.combineAxis = false

    controls.steer = steer
    controls.gas = gas
    controls.brake = brake
    controls.handbrake = 0
  end

  p.lineDist = select(
    3,
    findNearestSegment(car, p)
  )

  p.lastSteerTarget = steerTarget

  -- Do not override gear, clutch or transmission.
end

------------------------------------------------------------
-- TELEMETRY
------------------------------------------------------------

local function recordSecondTelemetry(sim)
  local anyWritten = false

  for i = 0, sim.carsCount - 1 do
    local car = ac.getCar(i)
    local p = state.p[i]

    if car and car.isActive and car.isAIControlled and p then
      local pos = car.position
      local vel = car.velocity
      local look = car.look

      writeLog(
        "CAR=%d " ..
        "POS=(%.2f,%.2f,%.2f) " ..
        "SPEED=%.2f " ..
        "STEER=%.3f " ..
        "STEER_TARGET=%.3f " ..
        "GAS=%.3f " ..
        "BRAKE=%.3f " ..
        "SLIP=%.4f " ..
        "LAT_VEL=%.3f " ..
        "YAW_RATE=%.4f " ..
        "LAT_ACCEL=%.3f " ..
        "STABILITY=%.3f " ..
        "CURVATURE=%.5f " ..
        "TARGET_SPEED=%.2f " ..
        "LINE_ERROR=%.2f " ..
        "SEGMENT=%d " ..
        "VELOCITY=(%.2f,%.2f) " ..
        "LOOK=(%.2f,%.2f)",

        i,

        pos.x,
        pos.y,
        pos.z,

        safeNumber(car.speedMs),

        p.steer,
        p.lastSteerTarget or 0,

        p.gas,
        p.brake,

        p.slipAngle,
        p.lateralVelocity,
        p.yawRate,
        p.lateralAccel,

        p.stability,

        p.lastCurve,
        p.targetSpeed,
        p.lineDist,

        p.segIdx,

        vel.x,
        vel.z,

        look.x,
        look.z
      )

      anyWritten = true
    end
  end

  if not anyWritten and logger and logger.heartbeat then
    pcall(
      logger.heartbeat,
      logger,
      "nodes=" .. tostring(#state.nodes)
    )
  end

  flushLog()
end

------------------------------------------------------------
-- UPDATE
------------------------------------------------------------

function script.update(dt)
  if not dt or dt <= 0 then
    return
  end

  -- Avoid excessively large controller steps.
  dt = math.min(dt, 0.05)

  local sim = ac.getSim()

  if not sim then
    return
  end

  local player = ac.getCar(0)

  -- Record player route.
  if state.recording and player then
    addPoint(player.position)
  end

  -- Teleport the AI grid once, before the route-following controller starts.
  local startupReady = true
  if state.enabled and state.loaded and #state.nodes >= 2 then
    startupReady = updateStartupTeleport(sim, dt)
  end

  -- Disable AI control and release all overrides.
  if not state.enabled or #state.nodes < 2 then
    for idx in pairs(state.p) do
      releaseCar(idx)
    end
  elseif startupReady then
    for i = 0, sim.carsCount - 1 do
      local car = ac.getCar(i)

      if car and car.isActive and car.isAIControlled then
        updateLineCar(i, car, dt)
      elseif state.p[i] then
        releaseCar(i)
      end
    end
  else
    -- Preserve the existing player-car controller while AI slots settle.
    local playerCar = ac.getCar(0)
    if playerCar and playerCar.isActive and playerCar.isAIControlled then
      updateLineCar(0, playerCar, dt)
    end
  end

  -- Periodic telemetry.
  state.logAccum = state.logAccum + dt

  if state.logAccum >= CFG.LOG_INTERVAL then
    state.logAccum = state.logAccum - CFG.LOG_INTERVAL

    recordSecondTelemetry(sim)
  end
end

------------------------------------------------------------
-- UI
------------------------------------------------------------

function windowMain(dt)
  local player = ac.getCar(0)

  ui.text(string.format(
    "Точек: %d | %s",
    #state.nodes,
    state.status
  ))

  ui.text(string.format(
    "Автопилот: %s",
    state.enabled and "ВКЛ" or "ВЫКЛ"
  ))

  if logger and logger.path then
    ui.text("Лог: " .. tostring(logger.path))
  end

  if ui.button(
    state.recording and "Стоп запись" or "Запись линии",
    vec2(150, 24)
  ) then
    state.recording = not state.recording

    if state.recording then
      state.nodes = {}
      state.p = {}
      state.loaded = false
      state.status = "Запись линии..."
    else
      state.status = "Записано точек: " .. #state.nodes
    end
  end

  ui.sameLine()

  if ui.button(
    state.loopRoute and "Круг: ВКЛ" or "Круг: ВЫКЛ",
    vec2(-1, 24)
  ) then
    state.loopRoute = not state.loopRoute
    state.p = {}
  end

  if ui.button(
    state.enabled and "Отключить ИИ" or "Включить ИИ",
    vec2(-1, 24)
  ) then
    state.enabled = not state.enabled

    if not state.enabled then
      for idx in pairs(state.p) do
        releaseCar(idx)
      end
    end
  end

  local canvas = vec2(300, 125)

  ui.invisibleButton("line_radar", canvas)

  local p0 = ui.getItemRectMin()
  local p1 = ui.getItemRectMax()

  ui.drawRectFilled(
    p0,
    p1,
    rgbm(0.1, 0.1, 0.12, 1),
    4
  )

  ui.drawRect(
    p0,
    p1,
    rgbm(0.3, 0.3, 0.35, 1),
    4
  )

  if #state.nodes >= 2 then
    local cx = player and player.position.x or state.nodes[1].x
    local cz = player and player.position.z or state.nodes[1].z

    local scale = 1.2

    local mx = (p0.x + p1.x) * 0.5
    local my = (p0.y + p1.y) * 0.5

    local segmentCount = state.loopRoute and
      #state.nodes or (#state.nodes - 1)

    for i = 1, segmentCount do
      local a = state.nodes[i]
      local b = state.nodes[(i % #state.nodes) + 1]

      local x1 = mx + (a.x - cx) * scale
      local y1 = my + (a.z - cz) * scale

      local x2 = mx + (b.x - cx) * scale
      local y2 = my + (b.z - cz) * scale

      if x1 >= p0.x and x1 <= p1.x and
         y1 >= p0.y and y1 <= p1.y and
         x2 >= p0.x and x2 <= p1.x and
         y2 >= p0.y and y2 <= p1.y then

        ui.drawLine(
          vec2(x1, y1),
          vec2(x2, y2),
          rgbm(1.0, 0.85, 0.0, 0.9),
          2
        )
      end
    end

    ui.drawCircleFilled(
      vec2(mx, my),
      4.5,
      rgbm(0.0, 0.8, 1.0, 1.0),
      12
    )
  end

  if ui.button("Сохранить", vec2(96, 24)) then
    saveLine()
  end

  ui.sameLine()

  if ui.button("Загрузить", vec2(96, 24)) then
    loadLine()
  end

  ui.sameLine()

  if ui.button("Очистить", vec2(-1, 24)) then
    state.nodes = {}
    state.p = {}
    state.status = "Очищено"
    state.loaded = false
  end

  state.speedMin = ui.slider(
    "Мин. скорость",
    state.speedMin,
    40,
    180,
    "%.0f км/ч"
  )

  state.speedMax = ui.slider(
    "Макс. скорость",
    state.speedMax,
    60,
    260,
    "%.0f км/ч"
  )

  state.speedMin = math.min(
    state.speedMin,
    state.speedMax
  )

  state.followGap = ui.slider(
    "Дистанция до машины",
    state.followGap,
    3,
    20,
    "%.0f м"
  )
end

------------------------------------------------------------
-- RELEASE
------------------------------------------------------------

if ac.onRelease then
  ac.onRelease(function()
    for idx in pairs(state.p) do
      releaseCar(idx)
    end

    if logger and logger.shutdown then
      pcall(
        logger.shutdown,
        logger,
        "RoutePilot released"
      )
    end
  end)
end

------------------------------------------------------------
-- INITIALIZATION
------------------------------------------------------------

loadLine()

writeLog(
  "ROUTEPILOT INITIALIZED track=%s points=%d",
  getTrackTag(),
  #state.nodes
)

flushLog()