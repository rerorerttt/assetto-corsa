-- RoutePilot.lua
-- RoutePilot 0.4 - Route Following, Collision Avoidance and AI Recovery
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
  LOOKAHEAD_BASE = 14.0,
  LOOKAHEAD_SPEED = 0.82,
  LOOKAHEAD_MIN = 14.0,
  LOOKAHEAD_MAX = 52.0,

  STEER_GAIN = 0.48,
  STEER_LIMIT = 0.75,
  STEER_RATE = 2.4,
  STEER_FILTER = 0.18,

  SLIP_START = 0.16,
  SLIP_FULL = 0.62,

  MAX_LATERAL_ACCEL = 11.5,
  LATERAL_ACCEL_LIMIT = 17.0,

  BRAKE_ACCEL = 12.0,
  MAX_BRAKE = 0.75,
  MAX_GAS = 1.0,

  SPEED_GAIN = 0.55,
  MIN_CORNER_SPEED = 9.0,
  SCAN_RAY_LENGTH = 14.0,
  SCAN_RAYS_PER_FRAME = 64,
  SCAN_CACHE_VERSION = 2,

  FOLLOW_LATERAL = 3.0,
  FOLLOW_DISTANCE = 8.0,

  RECORD_DISTANCE = 3.0,

  SPAWN_GAP = 22.0,
  SPAWN_MIN_GAP = 14.0,
  SPAWN_STABLE_TIME = 0.35,
  SPAWN_WAIT_TIMEOUT = 8.0,

  RECOVERY_STUCK_TIME = 10.0,
  RECOVERY_MIN_MOVE = 3.0,
  RECOVERY_AIR_HEIGHT = 6.0,
  RECOVERY_AIR_TIME = 10.0,
  RECOVERY_OFF_ROUTE = 12.0,
  RECOVERY_OFF_ROUTE_TIME = 1.25,
  RECOVERY_COOLDOWN = 6.0,
  RECOVERY_SAFE_GAP = 14.0,
  RECOVERY_SEARCH_STEP = 16.0,
  RECOVERY_SEARCH_COUNT = 12,

  COLLISION_HORIZON = 4.0,
  COLLISION_RADIUS = 6.5,
  COLLISION_EMERGENCY_RADIUS = 3.2,

  OVERTAKE_DISTANCE = 42.0,
  OVERTAKE_REAR_DISTANCE = 30.0,
  OVERTAKE_MIN_ADVANTAGE = 0.5,
  OVERTAKE_OFFSET = 1.45,
  OVERTAKE_CLEARANCE = 2.55,
  OVERTAKE_RELEASE_DISTANCE = 14.0,
  OVERTAKE_OFFSET_RATE = 5.0,
  OVERTAKE_STUCK_TIME = 1.5,
  TRAFFIC_OFFSET = 1.65,
  TRAFFIC_LATERAL_CLEARANCE = 2.35,
  TRAFFIC_LONGITUDINAL_CLEARANCE = 3.2,
  TRAFFIC_LOOKAHEAD = 125.0,
  TRAFFIC_ONCOMING_TIME = 8.0,
  TRAFFIC_ONCOMING_RANGE = 9.0,
  TRAFFIC_OFFSET_RATE = 3.0,

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

  speedMax = 240,

  followGap = 5.0,
  playerSlowTimer = 0,
  trafficProjection = {},

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

  obstacleMap = {},
  obstacleReady = false,
  obstacleScanCursor = 1,
  obstacleScanBest = {},
  obstacleScanRays = 0,
  obstacleScanHits = 0,
  obstacleScanStarted = false,
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
-- STATIC TRACK COLLISION SCAN
-- Cache records are comments in the route file; the XYZ parser ignores them.
------------------------------------------------------------

local OBSTACLE_SCAN_DIRECTIONS = {
  { 1.0000, 0.0000 }, { 0.9239, 0.3827 }, { 0.7071, 0.7071 },
  { 0.3827, 0.9239 }, { 0.0000, 1.0000 }, { -0.3827, 0.9239 },
  { -0.7071, 0.7071 }, { -0.9239, 0.3827 }, { -1.0000, 0.0000 },
  { -0.9239, -0.3827 }, { -0.7071, -0.7071 }, { -0.3827, -0.9239 },
  { 0.0000, -1.0000 }, { 0.3827, -0.9239 }, { 0.7071, -0.7071 },
  { 0.9239, -0.3827 },
}
local OBSTACLE_SCAN_HEIGHTS = { 0.35, 0.80, 1.25, 1.70, 2.15 }

local function routeChecksum(nodes)
  local hash = 2166136261
  for i = 1, #nodes do
    local n = nodes[i]
    local x = math.floor(math.abs(n.x) * 1000)
    local y = math.floor(math.abs(n.y) * 1000)
    local z = math.floor(math.abs(n.z) * 1000)
    hash = (hash * 31 + x + y * 3 + z * 7 + i) % 2147483647
  end
  return math.floor(hash)
end

local function parseObstacleCache(raw, nodes)
  if type(raw) ~= "string" or type(nodes) ~= "table" then return false end
  local version, checksum, expectedCount = string.match(
    raw, "#RP_OBSTACLE_V1%s+(%d+)%s+(%d+)%s+(%d+)"
  )
  if tonumber(version) ~= CFG.SCAN_CACHE_VERSION or
     tonumber(checksum) ~= routeChecksum(nodes) then
    return false
  end

  local found = {}
  for line in string.gmatch(raw, "[^\r\n]+") do
    local idx, dir, height, distance, x, y, z, nx, ny, nz =
      string.match(line,
        "^#RP_OBS%s+(%d+)%s+(%d+)%s+([%-0-9%.]+)%s+([%-0-9%.]+)%s+([%-0-9%.]+)%s+([%-0-9%.]+)%s+([%-0-9%.]+)%s+([%-0-9%.]+)%s+([%-0-9%.]+)%s+([%-0-9%.]+)%s*$")
    if idx then
      found[#found + 1] = {
        routeIndex = tonumber(idx), direction = tonumber(dir),
        height = tonumber(height), distance = tonumber(distance),
        x = tonumber(x), y = tonumber(y), z = tonumber(z),
        nx = tonumber(nx), ny = tonumber(ny), nz = tonumber(nz),
      }
    end
  end

  if #found ~= tonumber(expectedCount) then return false end
  state.obstacleMap = found
  state.obstacleReady = true
  state.obstacleScanStarted = true
  return true
end

local function serializeRouteFile()
  local data = serializeNodes(state.nodes)
  if not state.obstacleReady then return data .. "\n" end

  local lines = { data }
  lines[#lines + 1] = string.format(
    "#RP_OBSTACLE_V1 %d %d %d",
    CFG.SCAN_CACHE_VERSION,
    routeChecksum(state.nodes),
    #state.obstacleMap
  )
  for i = 1, #state.obstacleMap do
    local o = state.obstacleMap[i]
    lines[#lines + 1] = string.format(
      "#RP_OBS %d %d %.2f %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
      o.routeIndex, o.direction, o.height, o.distance,
      o.x, o.y, o.z, o.nx, o.ny, o.nz
    )
  end
  return table.concat(lines, "\n") .. "\n"
end

local function beginObstacleScan()
  state.obstacleMap = {}
  state.obstacleReady = false
  state.obstacleScanCursor = 1
  state.obstacleScanBest = {}
  state.obstacleScanRays = 0
  state.obstacleScanHits = 0
  state.obstacleScanStarted = true
  writeLog("OBSTACLE_SCAN_BEGIN points=%d rays=%d length=%.1f",
    #state.nodes,
    #state.nodes * #OBSTACLE_SCAN_DIRECTIONS * #OBSTACLE_SCAN_HEIGHTS,
    CFG.SCAN_RAY_LENGTH)
end

local function saveObstacleCache()
  local path = getLineFilePath()
  if not io or not io.open then
    writeLog("OBSTACLE_SCAN_SAVE_FAILED reason=io_unavailable")
    return false
  end
  local ok, file = pcall(io.open, path, "w")
  if not ok or not file then
    writeLog("OBSTACLE_SCAN_SAVE_FAILED reason=open path=%s", tostring(path))
    return false
  end
  local success = pcall(function()
    file:write(serializeRouteFile())
    file:close()
  end)
  if success then
    writeLog("OBSTACLE_SCAN_SAVED path=%s obstacles=%d", path, #state.obstacleMap)
  else
    writeLog("OBSTACLE_SCAN_SAVE_FAILED reason=write")
  end
  flushLog()
  return success
end

local function updateObstacleScan()
  if not state.loaded or state.recording or state.obstacleReady or
     not state.obstacleScanStarted then return end
  if type(physics) ~= "table" or type(physics.raycastTrack) ~= "function" then
    state.obstacleScanStarted = false
    writeLog("OBSTACLE_SCAN_ABORT reason=raycastTrack_unavailable")
    return
  end

  local heightsCount = #OBSTACLE_SCAN_HEIGHTS
  local dirsCount = #OBSTACLE_SCAN_DIRECTIONS
  local tasksPerPoint = heightsCount * dirsCount
  local totalTasks = #state.nodes * tasksPerPoint
  local budget = CFG.SCAN_RAYS_PER_FRAME

  while budget > 0 and state.obstacleScanCursor <= totalTasks do
    local task = state.obstacleScanCursor - 1
    local pointIndex = math.floor(task / tasksPerPoint) + 1
    local localTask = task % tasksPerPoint
    local directionIndex = math.floor(localTask / heightsCount) + 1
    local heightIndex = (localTask % heightsCount) + 1
    local node = state.nodes[pointIndex]
    local direction = OBSTACLE_SCAN_DIRECTIONS[directionIndex]
    local height = OBSTACLE_SCAN_HEIGHTS[heightIndex]

    local origin = vec3(node.x, node.y + height, node.z)
    local ray = vec3(direction[1], 0, direction[2])
    local hitPoint = vec3(0, 0, 0)
    local hitNormal = vec3(0, 0, 0)
    local ok, distance = pcall(
      physics.raycastTrack,
      origin, ray, CFG.SCAN_RAY_LENGTH,
      hitPoint, hitNormal, false, false
    )
    state.obstacleScanRays = state.obstacleScanRays + 1

    if ok and type(distance) == "number" and distance > 0.25 and
       distance <= CFG.SCAN_RAY_LENGTH then
      state.obstacleScanHits = state.obstacleScanHits + 1
      local key = tostring(pointIndex) .. ":" .. tostring(directionIndex)
      local previous = state.obstacleScanBest[key]
      if not previous or distance < previous.distance then
        state.obstacleScanBest[key] = {
          routeIndex = pointIndex,
          direction = directionIndex,
          height = height,
          distance = distance,
          x = hitPoint.x, y = hitPoint.y, z = hitPoint.z,
          nx = hitNormal.x, ny = hitNormal.y, nz = hitNormal.z,
        }
      end
    end

    if heightIndex == heightsCount then
      local key = tostring(pointIndex) .. ":" .. tostring(directionIndex)
      local hit = state.obstacleScanBest[key]
      if hit then state.obstacleMap[#state.obstacleMap + 1] = hit end
      state.obstacleScanBest[key] = nil
    end

    state.obstacleScanCursor = state.obstacleScanCursor + 1
    budget = budget - 1
  end

  if state.obstacleScanCursor > totalTasks then
    state.obstacleReady = true
    state.status = string.format("Карта коллизий: %d объектов", #state.obstacleMap)
    writeLog("OBSTACLE_SCAN_FINISH rays=%d hits=%d objects=%d",
      state.obstacleScanRays, state.obstacleScanHits, #state.obstacleMap)
    saveObstacleCache()
  end
end

------------------------------------------------------------
-- SAVE ROUTE
------------------------------------------------------------

local function saveLine()
  if #state.nodes < 2 then
    state.status = "Недостаточно точек"
    return
  end

  local data = serializeRouteFile()
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
    state.loaded = true
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
  local rawData = nil
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
        rawData = raw
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
      rawData = raw
      loaded = parseNodes(raw)
    end
  end

  if loaded then
    state.nodes = loaded
    state.p = {}
    state.loaded = true
    state.obstacleMap = {}
    state.obstacleReady = false
    state.obstacleScanStarted = false

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
  local bestSignedOffset = 0

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
        bestSignedOffset = (dx * vz - dz * vx) / math.sqrt(lenSq)
      end
    end
  end

  return bestRouteDistance, math.sqrt(bestDistanceSq), bestSignedOffset
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

      local direction = vec3(
        b.x - a.x,
        b.y - a.y,
        b.z - a.z
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

  for order = 1, count do
    local carIndex = aiCars[order]
    local routeDistance = startDistance + (order - 1) * gap
    local position, direction = sampleSpawnRoute(metrics, routeDistance)

    if position and direction then
      -- Direction is inverted: the supplied vector must match the
      -- backward axis used by CSP's AI positioning call.
      local spawnDirection = vec3(-direction.x, -direction.y, -direction.z)

      local ok, result = pcall(
        physics.setAICarPosition,
        carIndex,
        position,
        spawnDirection
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
          spawnDirection.x,
          spawnDirection.y,
          spawnDirection.z
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
  local kmh = state.speedMax
  local car = ac.getCar(carIndex)

  local p = {
    segIdx = 1,

    cruiseSpeed = kmh / 3.6 * (0.94 + ((carIndex * 3) % 7) * 0.045),
    paceMultiplier = 0.94 + ((carIndex * 3) % 7) * 0.045,

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
    moveAnchor = car and vec3(car.position.x, car.position.y, car.position.z) or nil,
    moveTimer = 0,
    airTimer = 0,
    offRouteTimer = 0,
    recoveryCooldown = 0,
    recoveryCount = 0,

    lineDist = 0,
    routeY = car and safeNumber(car.position.y) or 0,
    collisionRisk = 0,

    targetX = 0,
    targetZ = 0,

    targetSpeed = 0,

    slipSeverity = 0,
    stability = 1,

    lastCurve = 0,

    routeDistance = 0,
    routeOffsetActual = 0,
    routeOffset = 0,
    desiredRouteOffset = 0,
    passTargetIndex = nil,
    passRole = nil,
    passSide = 0,
    slowTimer = 0,
  }

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
    "CAR_INIT id=%d cruise=%.1fkmh pace=%.2f segment=%d",
    carIndex,
    p.cruiseSpeed * 3.6,
    p.paceMultiplier,
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

  local back = math.min(2, segmentCount - 1)
  -- Keep projection local: a broad search can snap to a nearby parallel
  -- or opposite section of an oval and command a dangerous cross-track turn.
  local ahead = math.min(
    segmentCount - 1,
    math.max(8, math.floor(segmentCount * 0.06))
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

  -- Never snap an off-track car to an unrelated distant section of the
  -- oval. Keep its local route segment so recovery can return it safely.
  p.segIdx = bestSeg

  local routeA = nodes[bestSeg]
  local routeB = getSegmentEnd(bestSeg, count)
  local routeY = lerp(routeA.y, routeB.y, bestT)

  return bestSeg, bestT, math.sqrt(bestDist), segmentCount, routeY
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
  local targetDirX = 0
  local targetDirZ = 1

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

    targetDirX = dx / length
    targetDirZ = dz / length

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

  return targetX, targetZ, targetDirX, targetDirZ
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

  -- Examine a longer section of the route, not only the next few points.
  for offset = 0, 40 do
    local index = current + offset

    if state.loopRoute then
      index = ((index - 1) % count) + 1
    elseif index > segmentCount then
      break
    end

    local curvature = getVertexCurvature(index)

    maxCurve = math.max(maxCurve, curvature)

    if curvature > 0.001 then
      local cornerSpeed = math.max(
        CFG.MIN_CORNER_SPEED,
        math.sqrt(CFG.MAX_LATERAL_ACCEL / curvature)
      ) * (p.paceMultiplier or 1.0)

      -- Braking distance is calculated from the current position
      -- to the upcoming corner.
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

  return math.max(0, math.min(targetSpeed, p.cruiseSpeed))
end

------------------------------------------------------------
-- VEHICLE DYNAMICS
------------------------------------------------------------

local function readVehicleDynamics(car, p, dt)
  local velocity = car.velocity
  local look = car.look

  local fx, fz = normalize2(look.x, look.z)

  local rx = fz
  local rz = -fx

  local vx = safeNumber(velocity.x)
  local vz = safeNumber(velocity.z)

  local longitudinalVelocity = vx * fx + vz * fz
  local lateralVelocity = vx * rx + vz * rz

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
  local targetX, targetZ, routeFx, routeFz = getLookaheadTarget(car, p, speed)
  local routeOffset = clamp(safeNumber(p.routeOffset, 0), -CFG.TRAFFIC_OFFSET, CFG.TRAFFIC_OFFSET)
  local travelSign = p.trafficDirectionSign or
    ((routeFx * fx + routeFz * fz) >= 0 and 1 or -1)
  targetX = targetX + routeFz * routeOffset * travelSign
  targetZ = targetZ - routeFx * routeOffset * travelSign

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

  local routeSteer = angle * CFG.STEER_GAIN
  -- Keep the race line dominant; large slip compensation fed the steering
  -- oscillation seen in telemetry at high speed.
  local slipCompensation = p.slipAngle * 0.18

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
-- ROUTE-RELATIVE OVERTAKING AND YIELDING
------------------------------------------------------------

------------------------------------------------------------
-- CENTRAL TRAFFIC COORDINATOR
-- All AI decisions are calculated together before any car is controlled.
------------------------------------------------------------

local function trafficLaneIsClear(info, infos, plans, targetIndex, offset)
  -- Check the complete reserved lane, not one point beside the car.
  -- Other cars are projected forward in time; static hits are checked at
  -- every sample so a wall or a post cannot be hidden between two samples.
  local sampleSpeed = math.max(10.0, info.speed or 0)
  local candidateY = safeNumber(info.car.position.y)

  for step = 0, 8 do
    local distance = step * 5.0
    local travelTime = distance / sampleSpeed
    local candidateX = info.x + info.hx * distance + info.rightX * offset
    local candidateZ = info.z + info.hz * distance + info.rightZ * offset

    for otherIndex, other in pairs(infos) do
      if otherIndex ~= info.index and otherIndex ~= targetIndex then
        local otherPlan = plans[otherIndex]
        local otherOffset = otherPlan and otherPlan.offset or 0
        local otherX = other.x + other.vx * travelTime + other.rightX * otherOffset
        local otherZ = other.z + other.vz * travelTime + other.rightZ * otherOffset
        local dx = otherX - candidateX
        local dz = otherZ - candidateZ
        local along = dx * info.hx + dz * info.hz
        local lateral = math.abs(dx * info.hz - dz * info.hx)

        if math.abs(along) < 5.0 and lateral < 2.55 then
          return false, "traffic"
        end
      end
    end

  end

  return true, "clear"
end

local function getRouteGap(from, to)
  if not from or not to or
     type(from.routeProgress) ~= "number" or
     type(to.routeProgress) ~= "number" then
    return nil
  end

  local fromDirection = from.directionSign or from.trafficDirectionSign
  local toDirection = to.directionSign or to.trafficDirectionSign
  if not fromDirection or not toDirection or fromDirection ~= toDirection then
    return nil
  end

  local delta = (to.routeProgress - from.routeProgress) * fromDirection
  local totalLength = from.routeTotalLength or to.routeTotalLength or 0

  if state.loopRoute and totalLength > 0 then
    delta = delta % totalLength
    if delta > totalLength * 0.5 then
      delta = delta - totalLength
    end
  end

  return delta
end

local function updateTrafficCoordinator(sim, dt)
  local routeSegmentCount = state.loopRoute and #state.nodes or (#state.nodes - 1)
  local routeStarts = {}
  local routeTotalLength = 0

  for segment = 1, routeSegmentCount do
    routeStarts[segment] = routeTotalLength
    routeTotalLength = routeTotalLength +
      flatDist(state.nodes[segment], getSegmentEnd(segment, #state.nodes))
  end

  local infos = {}
  local plans = {}

  -- Build one consistent snapshot of every active car, including the player.
  for i = 0, sim.carsCount - 1 do
    local car = ac.getCar(i)

    if car and car.isActive then
      local p = nil

      if i ~= 0 and car.isAIControlled then
        p = state.p[i]
        if not p then
          newPersonality(i)
          p = state.p[i]
        end
      else
        p = state.trafficProjection[i]
        if not p then
          p = { segIdx = 1 }
          state.trafficProjection[i] = p
        end
      end

      local seg, t, lineError = findNearestSegment(car, p)
      local a = state.nodes[seg]
      local b = getSegmentEnd(seg, #state.nodes)

      local tx, tz = normalize2(b.x - a.x, b.z - a.z)
      local hx, hz = normalize2(car.look.x, car.look.z)
      local directionSign = (hx * tx + hz * tz) >= 0 and 1 or -1
      local dx = car.position.x - (a.x + (b.x - a.x) * t)
      local dz = car.position.z - (a.z + (b.z - a.z) * t)
      local routeOffsetActual = (dx * tz - dz * tx) * directionSign
      local routeProgress = (routeStarts[seg] or 0) + flatDist(a, b) * t

      p.routeProgress = routeProgress
      p.routeTotalLength = routeTotalLength
      p.routeOffsetActual = routeOffsetActual
      p.trafficDirectionSign = directionSign

      infos[i] = {
        index = i,
        car = car,
        p = p,
        x = safeNumber(car.position.x),
        z = safeNumber(car.position.z),
        vx = safeNumber(car.velocity.x),
        vz = safeNumber(car.velocity.z),
        speed = math.max(0, safeNumber(car.speedMs)),
        hx = hx,
        hz = hz,
        rightX = hz,
        rightZ = -hx,
        directionSign = directionSign,
        lineError = lineError,
        routeOffsetActual = routeOffsetActual,
        routeProgress = routeProgress,
        routeTotalLength = routeTotalLength,
        isPlayer = i == 0,
        isAI = i ~= 0 and car.isAIControlled == true,
      }

      if i ~= 0 and p then
        if infos[i].speed < 2.0 then
          p.slowTimer = (p.slowTimer or 0) + dt
        else
          p.slowTimer = 0
        end
      end
    end
  end

  local function reservePair(a, b, roleA, roleB, offsetA, offsetB)
    if not a or not b then return end
    if plans[a.index] or plans[b.index] then return end

    plans[a.index] = {
      target = b.index,
      role = roleA,
      offset = offsetA,
    }
    plans[b.index] = {
      target = a.index,
      role = roleB,
      offset = offsetB,
    }
  end

  -- Preserve an already agreed pass until the faster car is safely clear.
  for i = 1, sim.carsCount - 1 do
    local a = infos[i]
    local old = a and a.p
    if a and old and old.passTargetIndex ~= nil and old.passRole == "pass" then
      local b = infos[old.passTargetIndex]
      local bp = b and b.p

      if b and bp and bp.passTargetIndex == i and bp.passRole == "yield" then
        local headingDot = a.hx * b.hx + a.hz * b.hz
        local dx = b.x - a.x
        local dz = b.z - a.z
        local forward = dx * a.hx + dz * a.hz

        if headingDot > 0.55 and forward > -14.0 and forward < 90.0 then
          reservePair(a, b, "pass", "yield", -CFG.TRAFFIC_OFFSET, CFG.TRAFFIC_OFFSET)
        end
      end
    elseif a and old and old.passTargetIndex ~= nil and old.passRole == "meet_right" then
      local b = infos[old.passTargetIndex]
      local bp = b and b.p

      if b and bp and bp.passTargetIndex == i and bp.passRole == "meet_right" then
        local dx = b.x - a.x
        local dz = b.z - a.z
        local rvx = b.vx - a.vx
        local rvz = b.vz - a.vz
        local rv2 = rvx * rvx + rvz * rvz
        local tClosest = rv2 > 0.25 and
          -(dx * rvx + dz * rvz) / rv2 or 0
        local distance = math.sqrt(dx * dx + dz * dz)

        if distance < 75.0 and tClosest > -0.5 then
          reservePair(a, b, "meet_right", "meet_right",
            CFG.TRAFFIC_OFFSET, CFG.TRAFFIC_OFFSET)
        end
      end
    elseif a and old and old.passTargetIndex ~= nil and old.passRole == "cross_go" then
      local b = infos[old.passTargetIndex]
      local bp = b and b.p
      if b and bp and bp.passTargetIndex == i and bp.passRole == "cross_yield" then
        local dx = b.x - a.x
        local dz = b.z - a.z
        local distance = math.sqrt(dx * dx + dz * dz)
        if distance < 24.0 then
          reservePair(a, b, "cross_go", "cross_yield", 0, CFG.TRAFFIC_OFFSET)
        end
      end
    end
  end

  -- First priority: oncoming conflicts. Both AI cars keep to their own right.
  for i = 0, sim.carsCount - 1 do
    local a = infos[i]
    if a then
      for j = i + 1, sim.carsCount - 1 do
        local b = infos[j]
        if b and not plans[i] and not plans[j] then
          local headingDot = a.hx * b.hx + a.hz * b.hz

          if headingDot < -0.55 then
            local dx = b.x - a.x
            local dz = b.z - a.z
            local rvx = b.vx - a.vx
            local rvz = b.vz - a.vz
            local rv2 = rvx * rvx + rvz * rvz

            local tClosest = rv2 > 0.25 and clamp(
              -(dx * rvx + dz * rvz) / rv2,
              0,
              CFG.TRAFFIC_ONCOMING_TIME
            ) or 0

            local cx = dx + rvx * tClosest
            local cz = dz + rvz * tClosest
            local closest = math.sqrt(cx * cx + cz * cz)

            local dxAhead = dx * a.hx + dz * a.hz
            local otherAhead = -dx * b.hx - dz * b.hz
            local lateralA = math.abs(dx * a.hz - dz * a.hx)
            local lateralB = math.abs(dx * b.hz - dz * b.hx)
            local facingConflict = dxAhead > 0 and otherAhead > 0 and
              math.sqrt(dx * dx + dz * dz) < 85.0 and
              lateralA < 6.0 and lateralB < 6.0

            if ((rv2 > 0.25 and tClosest > 0.15 and
                 closest < CFG.TRAFFIC_ONCOMING_RANGE) or facingConflict) then
                if a.isPlayer and b.isAI then
                  plans[j] = {
                    target = i,
                    role = "yield_player",
                    offset = CFG.TRAFFIC_OFFSET,
                  }
                elseif b.isPlayer and a.isAI then
                  plans[i] = {
                    target = j,
                    role = "yield_player",
                    offset = CFG.TRAFFIC_OFFSET,
                  }
                elseif a.isAI and b.isAI then
                  reservePair(
                    a,
                    b,
                    "meet_right",
                    "meet_right",
                    CFG.TRAFFIC_OFFSET,
                    CFG.TRAFFIC_OFFSET
                  )
                end
              end
            end
          end
        end
      end
    end

  -- Second priority: same-direction traffic. Faster follower passes on the left;
  -- slower leader yields to the right. The player is never asked to move.
  for i = 0, sim.carsCount - 1 do
    local a = infos[i]
    if a then
      for j = i + 1, sim.carsCount - 1 do
        local b = infos[j]
        if b and not plans[i] and not plans[j] then
          local headingDot = a.hx * b.hx + a.hz * b.hz

          if headingDot > 0.72 then
            local dx = b.x - a.x
            local dz = b.z - a.z
            local forward = dx * a.hx + dz * a.hz
            local lateral = math.abs(dx * a.hz - dz * a.hx)

            if math.abs(forward) > 4.0 and
               math.abs(forward) < CFG.TRAFFIC_LOOKAHEAD and
               lateral < 5.5 then

              local follower, leader, gap
              if forward > 0 then
                follower, leader, gap = a, b, forward
              else
                follower, leader, gap = b, a, -forward
              end

              local closingSpeed = follower.speed - leader.speed
              local leaderStopped = leader.speed < 1.5 and
                (leader.isPlayer and state.playerSlowTimer > 1.5 or
                 leader.p and (leader.p.slowTimer or 0) > 1.5)
              local routeGap = getRouteGap(follower, leader)
              local routeLateral = math.abs(
                (follower.routeOffsetActual or 0) - (leader.routeOffsetActual or 0)
              )
              local routeAligned = routeGap and routeGap > 4.0 and
                routeGap < CFG.OVERTAKE_DISTANCE and
                math.abs(routeGap - gap) < 12.0 and routeLateral < 5.5
              -- Personality pace is not a physical reason to change lanes.
              -- Start the lane change early enough to finish it before reaching
              -- the leader. At high closing speed, a seven-metre trigger is too late.
              local setupDistance = math.max(7.0, math.abs(closingSpeed) * 0.65 + 4.0)
              local canPass = (closingSpeed > 2.5 and gap > setupDistance) or leaderStopped

              if routeAligned and gap > 7.0 and gap < CFG.OVERTAKE_DISTANCE and
                 lateral < 5.5 and canPass then
                if follower.isAI and leader.isAI then
                  reservePair(
                    follower,
                    leader,
                    "pass",
                    "yield",
                    -CFG.TRAFFIC_OFFSET,
                    CFG.TRAFFIC_OFFSET
                  )
                elseif follower.isPlayer and leader.isAI then
                  plans[leader.index] = {
                    target = follower.index,
                    role = "yield_player",
                    offset = CFG.TRAFFIC_OFFSET,
                  }
                end
              end
            end
          end
        end
      end
    end
  end

  -- Crossing-path conflicts (including 90-degree route intersections) are
  -- assigned a deterministic right-of-way before the cars reach the crossing.
  for i = 1, sim.carsCount - 1 do
    local a = infos[i]
    if a and a.isAI and not plans[i] then
      for j = i + 1, sim.carsCount - 1 do
        local b = infos[j]
        if b and b.isAI and not plans[j] then
          local headingDot = a.hx * b.hx + a.hz * b.hz
          if headingDot >= -0.55 and headingDot <= 0.72 then
            local dx = b.x - a.x
            local dz = b.z - a.z
            local rvx = b.vx - a.vx
            local rvz = b.vz - a.vz
            local rv2 = rvx * rvx + rvz * rvz
            if rv2 > 1.0 then
              local tClosest = -(dx * rvx + dz * rvz) / rv2
              if tClosest > 0.35 and tClosest < 5.0 then
                local cx = dx + rvx * tClosest
                local cz = dz + rvz * tClosest
                local closest = math.sqrt(cx * cx + cz * cz)
                if closest < 7.0 then
                  local paceA = a.p and a.p.paceMultiplier or 1.0
                  local paceB = b.p and b.p.paceMultiplier or 1.0
                  local priority, yielding
                  if paceA > paceB or (paceA == paceB and a.index < b.index) then
                    priority, yielding = a, b
                  else
                    priority, yielding = b, a
                  end
                  reservePair(priority, yielding,
                    "cross_go", "cross_yield",
                    0, CFG.TRAFFIC_OFFSET)
                end
              end
            end
          end
        end
      end
    end
  end

  -- Reject a coordinated manoeuvre if either reserved path is occupied by a
  -- third car. Never let two independent plans claim the same strip of road.
  local rejected = {}
  for index, plan in pairs(plans) do
    local info = infos[index]
    if info then
      local laneClear, rejectReason = trafficLaneIsClear(
        info,
        infos,
        plans,
        plan.target,
        plan.offset
      )
      if not laneClear then
        rejected[index] = true
        writeLog("TRAFFIC_REJECT car=%d target=%d reason=%s offset=%.2f",
          index, plan.target or -1, rejectReason or "unknown", plan.offset or 0)
        if plans[plan.target] and plans[plan.target].target == index then
          rejected[plan.target] = true
        end
      end
    end
  end

  for index in pairs(rejected) do
    plans[index] = nil
  end

  -- Publish all decisions together. The individual controllers only execute
  -- the coordinator's reservation; they no longer negotiate independently.
  for index, info in pairs(infos) do
    if info.isAI then
      local p = info.p
      local plan = plans[index]
      local desired = plan and plan.offset or 0

      local nextTarget = plan and plan.target or nil
      local nextRole = plan and plan.role or nil
      if p.lastTrafficTarget ~= nextTarget or p.lastTrafficRole ~= nextRole then
        if nextRole then
          writeLog("TRAFFIC_PLAN car=%d target=%d role=%s offset=%.2f pace=%.2f",
            index, nextTarget, nextRole, desired, p.paceMultiplier or 1.0)
        elseif p.lastTrafficRole == "pass" then
          writeLog("PASS_RELEASE car=%d target=%d", index, p.lastTrafficTarget or -1)
        end
        p.lastTrafficTarget = nextTarget
        p.lastTrafficRole = nextRole
      end
      p.passTargetIndex = nextTarget
      p.passRole = nextRole
      p.passSide = 0
      p.desiredRouteOffset = desired
      p.routeOffsetActual = info.routeOffsetActual
      p.trafficDirectionSign = info.directionSign
      p.trafficCoordinated = plan ~= nil
      p.passCorridorClear = plan ~= nil
      local alpha = clamp(dt * CFG.TRAFFIC_OFFSET_RATE, 0, 1)
      p.routeOffset = clamp(lerp(safeNumber(p.routeOffset, 0), desired, alpha),
        -CFG.TRAFFIC_OFFSET, CFG.TRAFFIC_OFFSET)
    end
  end
end

------------------------------------------------------------
-- CAR FOLLOWING AND COLLISION PREDICTION
------------------------------------------------------------

local function carAhead(carIndex, car, personality)
  local fx, fz = normalize2(car.look.x, car.look.z)
  local sim = ac.getSim()

  if not sim then
    return nil, nil, math.huge
  end

  local bestGap = nil
  local bestSpeed = nil
  local riskTarget = math.huge
  local riskCar = -1
  local riskTTC = -1
  local riskLong = 0
  local riskLat = 0

  local ownVx = safeNumber(car.velocity.x)
  local ownVz = safeNumber(car.velocity.z)
  local ownSpeed = math.max(0, safeNumber(car.speedMs))

  for i = 0, sim.carsCount - 1 do
    if i ~= carIndex then
      local other = ac.getCar(i)

      if other and other.isActive then
        local dx = safeNumber(other.position.x) - safeNumber(car.position.x)
        local dz = safeNumber(other.position.z) - safeNumber(car.position.z)
        local distance = math.sqrt(dx * dx + dz * dz)
        local forward = dx * fx + dz * fz
        local lateralSigned = dx * fz - dz * fx
        local lateral = math.abs(lateralSigned)

        local otherSpeed = math.max(0, safeNumber(other.speedMs))
        local ovx = safeNumber(other.velocity.x)
        local ovz = safeNumber(other.velocity.z)
        local otherLookX, otherLookZ = normalize2(other.look.x, other.look.z)
        local sameDirection = fx * otherLookX + fz * otherLookZ > 0.55

        local otherPersonality = state.p[i] or state.trafficProjection[i]
        local routeGap = getRouteGap(personality, otherPersonality)
        local sameRouteDirection = routeGap ~= nil
        local coordinatedPair =
          personality and otherPersonality and
          personality.passTargetIndex == i and
          otherPersonality.passTargetIndex == carIndex and
          personality.passRole ~= "yield_player" and
          otherPersonality.passRole ~= "yield_player"

        local reservedPassPair = coordinatedPair and sameRouteDirection and
          ((personality.passRole == "pass" and otherPersonality.passRole == "yield") or
           (personality.passRole == "yield" and otherPersonality.passRole == "pass"))
        local ownReservedOffset = personality and
          safeNumber(personality.desiredRouteOffset, personality.routeOffset or 0) or 0
        local otherReservedOffset = otherPersonality and
          safeNumber(otherPersonality.desiredRouteOffset, otherPersonality.routeOffset or 0) or 0
        local reservedLateral = otherReservedOffset - ownReservedOffset
        local relativeLongSpeed = math.abs((ownVx - ovx) * fx + (ownVz - ovz) * fz)
        local passSetupGap = math.max(3.2, relativeLongSpeed * 0.65 + 4.0)
        local reservedLaneSeparated = reservedPassPair and
          math.abs(reservedLateral) >= 2.65 and routeGap and
          math.abs(routeGap) > passSetupGap

        -- Immediate physical overlap is never waived, even for an agreed pass.
        if math.abs(forward) < CFG.TRAFFIC_LONGITUDINAL_CLEARANCE and
           lateral < 1.55 then
          riskTarget = 0
          riskCar = i
          riskTTC = 0
          riskLong = math.abs(forward)
          riskLat = lateral
        end

        -- Once the coordinator has verified both swept corridors, the pair
        -- must not treat each other as a same-lane leader and brake to a stop.
        if not reservedLaneSeparated then
          local routeLateral = personality and otherPersonality and
            math.abs((otherPersonality.routeOffsetActual or 0) -
                     (personality.routeOffsetActual or 0)) or lateral
          if routeGap and routeGap > 0 and routeGap < 110 and
             routeLateral < CFG.TRAFFIC_LATERAL_CLEARANCE then
            if not bestGap or routeGap < bestGap then
              bestGap = routeGap
              bestSpeed = otherSpeed
            end
          elseif not routeGap and sameDirection and forward > 0 and forward < 110 and
                 lateral < CFG.TRAFFIC_LATERAL_CLEARANCE then
            if not bestGap or forward < bestGap then
              bestGap = forward
              bestSpeed = otherSpeed
            end
          end

          -- Predict relative motion for every unrelated vehicle. A verified
          -- pass reservation supplies a different lane trajectory for this pair.
          if distance < 130 then
            local rvx = ownVx - ovx
            local rvz = ownVz - ovz
            local rv2 = rvx * rvx + rvz * rvz
            local closing = dx * rvx + dz * rvz

            if rv2 > 0.04 and closing > 0 then
              local t = clamp(
                closing / rv2,
                0,
                CFG.COLLISION_HORIZON
              )

              local cx = dx - rvx * t
              local cz = dz - rvz * t
              local closestLong = math.abs(cx * fx + cz * fz)
              local closestLat = math.abs(cx * fz - cz * fx)

              if closestLong < CFG.TRAFFIC_LONGITUDINAL_CLEARANCE and
                 closestLat < 1.65 then
                local lateralMargin = clamp(
                  (closestLat - 1.15) /
                  (CFG.TRAFFIC_LATERAL_CLEARANCE - 1.15),
                  0,
                  1
                )
                local longitudinalMargin = clamp(
                  (closestLong - 2.0) /
                  (CFG.TRAFFIC_LONGITUDINAL_CLEARANCE - 2.0),
                  0,
                  1
                )
                local clearance = math.max(lateralMargin, longitudinalMargin)
                local safe = ownSpeed * clearance
                if safe < riskTarget then
                  riskTarget = safe
                  riskCar = i
                  riskTTC = t
                  riskLong = closestLong
                  riskLat = closestLat
                end
              end
            end
          end
        end
      end
    end
  end

  if personality then
    personality.followGapActual = bestGap or -1
    personality.riskCar = riskCar
    personality.riskTTC = riskTTC
    personality.riskLong = riskLong
    personality.riskLat = riskLat
  end

  return bestGap, bestSpeed, riskTarget
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

  -- Keep power on during a drift. The old 5% throttle floor made
  -- every high-slip corner collapse into a crawl instead of an arcade slide.
  local throttleFactor = lerp(1.0, 0.72, p.slipSeverity)
  gas = gas * throttleFactor

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
-- AI RECOVERY: FREE ROUTE POSITION SEARCH
------------------------------------------------------------

local function isRecoverySpotFree(carIndex, position)
  local sim = ac.getSim()

  if not sim then
    return false
  end

  for i = 0, sim.carsCount - 1 do
    if i ~= carIndex then
      local other = ac.getCar(i)

      if other and other.isActive and
         flatDist(position, other.position) < CFG.RECOVERY_SAFE_GAP then
        return false
      end
    end
  end

  return true
end

local function findRecoverySpot(carIndex, car)
  local metrics = buildSpawnRouteMetrics()

  if not metrics then
    return nil, nil, nil
  end

  local base = findRouteDistanceAtPosition(car.position, metrics)

  -- Search forward on the route. Every candidate is checked against
  -- the player and every active AI car.
  for step = 1, CFG.RECOVERY_SEARCH_COUNT do
    local distance = base + CFG.RECOVERY_SEARCH_STEP * step

    if not metrics.loop then
      distance = clamp(distance, 0, metrics.length)
    end

    local position, direction = sampleSpawnRoute(metrics, distance)

    if position and direction and isRecoverySpotFree(carIndex, position) then
      return position, direction, distance
    end
  end

  -- On an open route, also search behind the current position.
  if not metrics.loop then
    for step = 1, CFG.RECOVERY_SEARCH_COUNT do
      local distance = base - CFG.RECOVERY_SEARCH_STEP * step

      if distance >= 0 then
        local position, direction = sampleSpawnRoute(metrics, distance)

        if position and direction and isRecoverySpotFree(carIndex, position) then
          return position, direction, distance
        end
      end
    end
  end

  return nil, nil, nil
end

------------------------------------------------------------
-- AI RECOVERY: TELEPORT AND CONTROLLER RESET
------------------------------------------------------------

local function recoverAICar(carIndex, car, reason)
  if type(physics) ~= "table" or type(physics.setAICarPosition) ~= "function" then
    writeLog(
      "RECOVERY_ABORT id=%d reason=%s api=unavailable",
      carIndex,
      reason
    )

    return false
  end

  local position, direction, routeDistance =
    findRecoverySpot(carIndex, car)

  if not position or not direction then
    writeLog(
      "RECOVERY_WAIT id=%d reason=%s no_free_route_spot",
      carIndex,
      reason
    )

    return false
  end

  -- Use the same direction convention as the startup grid.
  local spawnDirection = vec3(
    -direction.x,
    -direction.y,
    -direction.z
  )

  local ok, result = pcall(
    physics.setAICarPosition,
    carIndex,
    position,
    spawnDirection
  )

  if not ok or result == false then
    writeLog(
      "RECOVERY_FAILED id=%d reason=%s error=%s",
      carIndex,
      reason,
      tostring(result)
    )

    return false
  end

  local previousRecoveries =
    state.p[carIndex] and state.p[carIndex].recoveryCount or 0

  state.p[carIndex] = nil
  newPersonality(carIndex)

  local p = state.p[carIndex]

  if p then
    p.recoveryCooldown = CFG.RECOVERY_COOLDOWN
    p.recoveryCount = previousRecoveries + 1

    p.moveAnchor = vec3(
      position.x,
      position.y,
      position.z
    )

    p.moveTimer = 0
    p.airTimer = 0
    p.offRouteTimer = 0
  end

  state.status = string.format(
    "Восстановление AI #%d: %s",
    carIndex,
    reason
  )

  writeLog(
    "RECOVERY_SUCCESS id=%d reason=%s route_distance=%.2f pos=(%.2f,%.2f,%.2f)",
    carIndex,
    reason,
    routeDistance,
    position.x,
    position.y,
    position.z
  )

  return true
end

------------------------------------------------------------
-- AI RECOVERY: STUCK, AIRBORNE AND OFF-ROUTE DETECTION
------------------------------------------------------------

local function updateRecoveryState(carIndex, car, p, dt, collisionHold)
  p.recoveryCooldown = math.max(
    0,
    (p.recoveryCooldown or 0) - dt
  )

  local _, _, lineError, _, routeY =
    findNearestSegment(car, p)

  p.lineDist = lineError
  p.routeY = routeY or safeNumber(car.position.y)

  if not p.moveAnchor then
    p.moveAnchor = vec3(
      car.position.x,
      car.position.y,
      car.position.z
    )

    p.moveTimer = 0
  end

  -- Three metres of real chassis displacement resets the ten-second timer.
  p.moveTimer = (p.moveTimer or 0) + dt

  if flatDist(p.moveAnchor, car.position) >= CFG.RECOVERY_MIN_MOVE then
    p.moveAnchor = vec3(
      car.position.x,
      car.position.y,
      car.position.z
    )

    p.moveTimer = 0
  end

  -- Height is measured against the interpolated height of the route.
  local height =
    safeNumber(car.position.y) - safeNumber(p.routeY)

  if height > CFG.RECOVERY_AIR_HEIGHT then
    p.airTimer = (p.airTimer or 0) + dt
  else
    p.airTimer = 0
  end

  -- A severe route departure is recovered earlier than a normal stall.
  if lineError > CFG.RECOVERY_OFF_ROUTE then
    p.offRouteTimer = (p.offRouteTimer or 0) + dt
  else
    p.offRouteTimer = 0
  end

  if p.recoveryCooldown > 0 then
    return false
  end

  local reason = nil

  if p.airTimer >= CFG.RECOVERY_AIR_TIME then
    reason = "airborne_10s"

  elseif p.moveTimer >= CFG.RECOVERY_STUCK_TIME then
    reason = collisionHold and "blocked_10s" or "stuck_10s"

  elseif p.offRouteTimer >= CFG.RECOVERY_OFF_ROUTE_TIME then
    reason = "off_route"
  end

  if reason then
    return recoverAICar(carIndex, car, reason)
  end

  return false
end

------------------------------------------------------------
-- LIVE TRACK-EDGE / OBSTACLE GUARD
-- Rechecks the moving car's forward corridor every update. The cached
-- route obstacle map is useful for planning, but is never trusted alone.
------------------------------------------------------------

local obstacleApiWarningWritten = false

local function getTrackRayDistance(origin, direction, length)
  if type(physics) ~= "table" or type(physics.raycastTrack) ~= "function" then
    return nil
  end

  local hitPoint = vec3(0, 0, 0)
  local hitNormal = vec3(0, 0, 0)
  local ok, distance = pcall(
    physics.raycastTrack,
    origin, direction, length,
    hitPoint, hitNormal, false, false
  )

  if not ok or type(distance) ~= "number" or distance < 0 then
    return length
  end

  return clamp(distance, 0, length)
end

local function applyLiveObstacleGuard(car, p, steer, targetSpeed)
  -- Static and raycast obstacle detection is intentionally disabled pending a validated scanner.
  return steer, targetSpeed

  if type(physics) ~= "table" or type(physics.raycastTrack) ~= "function" then
    if not obstacleApiWarningWritten then
      obstacleApiWarningWritten = true
      writeLog("LIVE_GUARD_UNAVAILABLE reason=raycastTrack")
    end
    return steer, targetSpeed
  end

  local fx, fz = normalize2(car.look.x, car.look.z)
  -- Side and forward fan at two heights: low rays catch barriers and
  -- curbs; high rays catch posts and vertical wall faces.
  local angles = { -1.35, -1.05, -0.75, -0.45, -0.20, 0,
                    0.20, 0.45, 0.75, 1.05, 1.35 }
  local heights = { 0.35, 0.90, 1.50 }
  local distances = {}
  local rayLength = 18.0

  for i = 1, #angles do
    distances[i] = rayLength
  end

  for h = 1, #heights do
    local rayOrigin = vec3(
      safeNumber(car.position.x),
      safeNumber(car.position.y) + heights[h],
      safeNumber(car.position.z)
    )
    for i = 1, #angles do
      local angle = angles[i]
      local c, sn = math.cos(angle), math.sin(angle)
      local direction = vec3(
        fx * c - fz * sn,
        0,
        fx * sn + fz * c
      )
      local hitDistance = getTrackRayDistance(rayOrigin, direction, rayLength) or rayLength
      distances[i] = math.min(distances[i], hitDistance)
    end
  end

  local center = distances[6]
  local rightClear = math.min(distances[1], distances[2], distances[3], distances[4], distances[5])
  local leftClear = math.min(distances[7], distances[8], distances[9], distances[10], distances[11])
  local forwardClear = math.min(distances[5], center, distances[7])
  local correction = 0
  local staticCorrection = 0
  local speed = math.max(0, safeNumber(car.speedMs))

  -- Cached track hits must protect the normal racing line too, not only
  -- authorize overtakes. A post or wall recorded on the map is treated as
  -- a forward threat even when a live ray misses its narrow collision mesh.
  local staticThreatDistance = math.huge
  local staticThreatSide = 0
  if state.obstacleReady then
    local carX = safeNumber(car.position.x)
    local carY = safeNumber(car.position.y)
    local carZ = safeNumber(car.position.z)
    for i = 1, #state.obstacleMap do
      local obstacle = state.obstacleMap[i]
      if math.abs(safeNumber(obstacle.y) - carY) < 2.6 then
        local dx = obstacle.x - carX
        local dz = obstacle.z - carZ
        local along = dx * fx + dz * fz
        local lateral = dx * fz - dz * fx
        if along > -1.5 and along < 18.0 and math.abs(lateral) < 3.4 and
           along < staticThreatDistance then
          staticThreatDistance = along
          staticThreatSide = lateral
        end
      end
    end
  end

  if staticThreatDistance < 14.0 then
    if staticThreatSide >= 0 then
      staticCorrection = -0.36
    else
      staticCorrection = 0.36
    end
    if staticThreatDistance < 9.0 then
      targetSpeed = math.min(targetSpeed, math.max(9.0, speed * 0.72))
    end
    if staticThreatDistance < 4.0 then
      targetSpeed = math.min(targetSpeed, math.max(5.0, speed * 0.42))
    end
    if staticThreatDistance < 2.0 then
      targetSpeed = math.min(targetSpeed, 4.0)
    end
  end

  if center < 9.0 then
    if rightClear > leftClear + 0.35 then
      correction = -0.34
    elseif leftClear > rightClear + 0.35 then
      correction = 0.34
    else
      -- No proven side corridor: keep a crawl, but do not issue a full stop
      -- unless the nose and both escape sides are physically boxed in.
      targetSpeed = math.min(targetSpeed, 6.0)
    end
    targetSpeed = math.min(targetSpeed, math.max(8.0, center * 1.45))
  else
    -- Side-only contacts are important: they are the usual signature of a
    -- car scraping a wall or post while trying to steer back to the route.
    if leftClear < 2.6 and rightClear > leftClear + 0.45 then
      correction = -0.28
      targetSpeed = math.min(targetSpeed, math.max(8.0, speed * 0.75))
    elseif rightClear < 2.6 and leftClear > rightClear + 0.45 then
      correction = 0.28
      targetSpeed = math.min(targetSpeed, math.max(8.0, speed * 0.75))
    end
  end

  if leftClear < 2.0 and steer > 0 then steer = 0 end
  if rightClear < 2.0 and steer < 0 then steer = 0 end

  correction = clamp(correction + staticCorrection, -0.55, 0.55)
  steer = clamp(steer + correction, -CFG.STEER_LIMIT, CFG.STEER_LIMIT)

  if staticThreatDistance < 14.0 then
    p.staticObstacleDistance = staticThreatDistance
  else
    p.staticObstacleDistance = -1
  end

  if center < 2.0 and leftClear < 1.6 and rightClear < 1.6 and
     forwardClear < 2.0 then
    targetSpeed = 0
  end

  p.liveObstacleDistance = center
  p.liveLeftClearance = leftClear
  p.liveRightClearance = rightClear
  p.liveForwardClearance = forwardClear
  p.liveObstacleRays = distances
  return steer, targetSpeed
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

  p.cruiseSpeed = state.speedMax / 3.6 * (p.paceMultiplier or 1.0)

  local speed, fx, fz = readVehicleDynamics(
    car,
    p,
    dt
  )

  calculateStability(p)

  local steerTarget = calculateSteering(
    car,
    p,
    speed,
    fx,
    fz
  )

  local steer = filterSteering(
    p,
    steerTarget,
    dt
  )

  -- Calculate speed from upcoming route curvature.
  local targetSpeed = planTargetSpeed(p, speed)

  -- Check the car ahead and predict possible collisions.
  local gap, leaderSpeed, collisionTarget =
    carAhead(carIndex, car, p)

  if gap then
    local safeSpeed = math.sqrt(math.max(
      0,
      leaderSpeed * leaderSpeed +
      2 * CFG.BRAKE_ACCEL * math.max(0, gap - state.followGap)
    ))

    targetSpeed = math.min(
      targetSpeed,
      safeSpeed
    )

    if gap < state.followGap then
      targetSpeed = math.min(
        targetSpeed,
        math.max(
          0,
          leaderSpeed - (state.followGap - gap) * 0.8
        )
      )
    end

    if gap < 2.5 then
      targetSpeed = 0
    end
  end

  if collisionTarget and collisionTarget < math.huge then
    targetSpeed = math.min(
      targetSpeed,
      collisionTarget
    )
  end

  -- Track collision sensing is disabled; traffic coordination remains active.

  -- At a predicted crossing, one car owns the gap and keeps pace; the
  -- designated yielding car sheds speed early instead of both cars panic-braking.
  if p.passRole == "cross_yield" then
    targetSpeed = math.min(targetSpeed, math.max(12.0, targetSpeed * 0.58))
  end

  p.collisionRisk = collisionTarget and collisionTarget < math.huge
    and math.max(0, speed - collisionTarget) or 0

  local gas, brake = calculateThrottle(
    p,
    speed,
    targetSpeed,
    dt
  )

  -- Emergency braking for a predicted immediate collision.
  if collisionTarget and collisionTarget <= 0.1 then
    gas = 0
    brake = math.max(brake, 0.65)
  end

  local collisionHold =
    (gap and gap < state.followGap + 1.0)
    or (collisionTarget and collisionTarget <= 0.1)

  -- Recovery is performed before sending the next control command.
  if updateRecoveryState(
    carIndex,
    car,
    p,
    dt,
    collisionHold
  ) then
    return
  end

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

  p.lastSteerTarget = steerTarget
  p.targetSpeed = targetSpeed
  p.gas = gas
  p.brake = brake

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
        "ROUTE_OFFSET=%.2f " ..
        "PASS_TARGET=%d " ..
        "PASS_ROLE=%s " ..
        "PASS_OFFSET=%.2f " ..
        "AIR_TIMER=%.1f " ..
        "STUCK_TIMER=%.1f " ..
        "COLLISION_RISK=%.2f " ..
        "FOLLOW_GAP=%.2f " ..
        "LIVE_OBSTACLE=%.2f " ..
        "LIVE_LEFT=%.2f " ..
        "LIVE_RIGHT=%.2f " ..
        "PASS_CLEAR=%d " ..
        "RISK_CAR=%d " ..
        "RISK_TTC=%.2f " ..
        "RISK_LONG=%.2f " ..
        "RISK_LAT=%.2f " ..
        "RECOVERIES=%d " ..
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
        p.routeOffset or 0,
        p.passTargetIndex or -1,
        p.passRole or "none",
        p.routeOffset or 0,

        p.airTimer or 0,
        p.moveTimer or 0,
        p.collisionRisk or 0,
        p.followGapActual or -1,
        p.liveObstacleDistance or -1,
        p.liveLeftClearance or -1,
        p.liveRightClearance or -1,
        p.passCorridorClear and 1 or 0,
        p.riskCar or -1,
        p.riskTTC or -1,
        p.riskLong or 0,
        p.riskLat or 0,
        p.recoveryCount or 0,

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

  dt = math.min(dt, 0.05)

  local sim = ac.getSim()

  if not sim then
    return
  end

  local player = ac.getCar(0)

  -- Track how long the player has been stationary so AI can yield to a faster player.
  if player and safeNumber(player.speedMs) < 2.0 then
    state.playerSlowTimer = (state.playerSlowTimer or 0) + dt
  else
    state.playerSlowTimer = 0
  end

  -- Record player route.
  if state.recording and player then
    addPoint(player.position)
  end

  -- Static collision scanning is disabled until the map geometry is validated.

  -- Teleport the AI grid once, before route-following starts.
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
    updateTrafficCoordinator(sim, dt)

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

function script.windowMain(dt)
  local player = ac.getCar(0)

  ui.text(string.format(
    "Точек: %d | %s",
    #state.nodes,
    state.status
  ))

  ui.text(string.format(
    "Автопилот: %s | Обнаружение препятствий: ВЫКЛ",
    state.enabled and "ВКЛ" or "ВЫКЛ"
  ))

  if ui.button(
    state.recording and "Стоп запись" or "Запись линии",
    vec2(150, 24)
  ) then
    state.recording = not state.recording

    if state.recording then
      state.nodes = {}
      state.p = {}
      state.loaded = false
      state.obstacleMap = {}
      state.obstacleReady = false
      state.obstacleScanStarted = false
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

  if ui.button("Сохранить", vec2(96, 24)) then
    saveLine()
  end

  ui.sameLine()

  if ui.button("Загрузить", vec2(96, 24)) then
    loadLine()
  end

  if ui.button("Очистить", vec2(-1, 24)) then
    state.nodes = {}
    state.p = {}
    state.status = "Очищено"
    state.loaded = false
  end

  -- The route map is drawn inline in this single main window, below the controls.
  local canvas = vec2(ui.availableSpaceX(), math.min(360, ui.availableSpaceY()))

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