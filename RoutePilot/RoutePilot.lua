-- RoutePilot.lua
-- RoutePilot 0.4.2 — Arcade Racing Traffic (main script)
--
-- v0.4.2:
--  * CORRIDOR DE-NOISED: raw scan values jump (40 -> 6 -> 2 -> 34) from
--    pillars and clutter; a median filter smooths the corridor and the
--    save file, so one stray ray no longer makes cars crawl or hug walls.
--    A NEW corridor file is computed from the old scan on load, no
--    re-scan needed.
--  * LIVE ROOM RAYS: each car casts left/right rays every frame to pick
--    the overtake side with the most room (NFS-style: it knows it fits).
--  * MAP FIXED: only UI calls verified on this CSP build are used
--    (availableSpaceX/getItemRectMin/getItemRectMax, as in the working
--    v0.3 app); the whole window body is pcall-protected so the app can
--    never fail to draw again. The map is a normal layout item pinned
--    under the buttons — it cannot float.
--  * UI: no sliders, no AI toggle — buttons on top, map fills the rest.

local okLogger, Logger = pcall(require, "logger")
if not okLogger or type(Logger) ~= "table" then Logger = nil end
local okDriver, Driver = pcall(require, "driver")
if not okDriver or type(Driver) ~= "table" or not Driver.update then Driver = nil end

local CFG = {
  LOOKAHEAD_BASE = 12.0,
  LOOKAHEAD_SPEED = 0.7,
  LOOKAHEAD_MIN = 12.0,
  LOOKAHEAD_MAX = 45.0,
  STEER_GAIN = 0.55,
  STEER_LIMIT = 1.0,
  STEER_RATE = 2.5,
  STEER_FILTER = 0.15,

  MAX_LATERAL_ACCEL = 9.0,
  BRAKE_ACCEL = 8.5,
  PLAN_HORIZON = 170.0,
  MIN_APEX_SPEED = 11.0,
  CURVE_MIN = 0.002,
  GAS_GAIN = 0.45,
  BRAKE_GAIN = 0.60,
  MAX_GAS = 1.0,
  MAX_BRAKE = 1.0,

  SLIP_START = 0.09,
  SLIP_FULL = 0.45,
  SLIP_DAMP_MIN_SPEED = 8.0,
  SLIP_DAMP_MIN_ANGLE = 4.0 * math.pi / 180,

  AVOID_LATERAL = 5.0,        -- cone half-width for obstacles (cars + player)
  PASS_MIN_ROOM = 6.0,        -- need this much live room to commit to a pass
  OVERTAKE_LATERAL = 5.5,
  OVERTAKE_CLEAR_AHEAD = 5.0,

  SPAWN_GAP = 25.0,
  SPAWN_MIN_GAP = 14.0,
  SPAWN_STABLE_TIME = 0.35,
  SPAWN_WAIT_TIMEOUT = 8.0,

  SMOOTH_PASSES = 3,
  SMOOTH_WINDOW = 5,

  CORRIDOR_RAY = 40.0,
  CORRIDOR_RAY_HEIGHT = 1.0,
  CORRIDOR_MEDIAN_WINDOW = 7,
  CORRIDOR_MIN_SAVED = 6.0,   -- saved values below this are noise

  WALL_RAY_AHEAD = 60.0,
  ROOM_RAY = 20.0,

  LOG_INTERVAL = 1.0,
  MAX_ROUTE_POINTS = 50000,
  RECORD_DISTANCE = 3.0,
}

local state = {
  enabled = true,
  recording = false,
  loopRoute = true,

  speedMax = 170,
  followGap = 8.0,

  corridor = {},

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
  if ok then logger = result end
end

local function writeLog(fmt, ...)
  if not logger or not logger.log then return end
  pcall(logger.log, logger, fmt, ...)
end

local function flushLog()
  if logger and logger.flush then pcall(logger.flush, logger) end
end

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
  if l < 0.00001 then return 0, 1 end
  return x / l, z / l
end

local function safeNumber(v, fallback)
  if type(v) ~= "number" then return fallback or 0 end
  if v ~= v or v == math.huge or v == -math.huge then return fallback or 0 end
  return v
end

local function getTrackTag()
  local tid = (ac.getTrackID and ac.getTrackID()) or "track"
  local layout = (ac.getTrackLayout and ac.getTrackLayout()) or ""
  if layout ~= "" then return tostring(tid) .. "_" .. tostring(layout) end
  return tostring(tid)
end

local function getLineFilePath()
  local appDir = ""
  if ac.getFolder and ac.FolderID then
    appDir = ac.getFolder(ac.FolderID.ACApps) or ""
  end
  local tag = string.gsub(getTrackTag(), "[^%w_%-]", "_")
  if appDir ~= "" then return appDir .. "/lua/RoutePilot/line_" .. tag .. ".txt" end
  return "line_" .. tag .. ".txt"
end

local function getCorridorFilePath()
  local p = getLineFilePath()
  return string.gsub(p, "%.txt$", "_corridor.txt")
end

------------------------------------------------------------
-- LINE LOAD / SAVE
------------------------------------------------------------

local function serializeNodes(nodes)
  local lines = {}
  for i = 1, #nodes do
    local n = nodes[i]
    lines[#lines + 1] = string.format("%.4f %.4f %.4f", n.x, n.y, n.z)
  end
  return table.concat(lines, "\n")
end

local function parseNodes(raw)
  if type(raw) ~= "string" or raw == "" then return nil end
  local loaded = {}
  for line in string.gmatch(raw, "[^\r\n]+") do
    local xs, ys, zs = string.match(line, "^%s*([%-0-9%.]+)%s+([%-0-9%.]+)%s+([%-0-9%.]+)%s*$")
    local x, y, z = tonumber(xs), tonumber(ys), tonumber(zs)
    if x and y and z and x == x and y == y and z == z then
      loaded[#loaded + 1] = vec3(x, y, z)
    end
    if #loaded >= CFG.MAX_ROUTE_POINTS then break end
  end
  if #loaded >= 2 then return loaded end
  return nil
end

local function saveLine()
  if #state.nodes < 2 then
    state.status = "Недостаточно точек"
    return
  end
  local data = serializeNodes(state.nodes)
  local path = getLineFilePath()
  local wroteFile = false
  if io and io.open then
    local ok, f = pcall(io.open, path, "w")
    if ok and f then
      wroteFile = pcall(function() f:write(data) f:close() end)
    end
  end
  if ac.storage then
    pcall(ac.storage, "RP_LINE_DATA_" .. getTrackTag(), data)
  end
  if wroteFile then
    state.status = string.format("Сохранено: %d точек", #state.nodes)
    writeLog("SAVE_ROUTE points=%d", #state.nodes)
    flushLog()
  else
    state.status = "Ошибка сохранения"
  end
end

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
      if success then loaded = parseNodes(raw) end
    end
  end
  if not loaded and ac.storage then
    local ok, raw = pcall(ac.storage, "RP_LINE_DATA_" .. getTrackTag())
    if ok and raw then loaded = parseNodes(raw) end
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
    state.status = string.format("Загружено: %d точек", #loaded)
    writeLog("LOAD_ROUTE points=%d track=%s", #loaded, getTrackTag())
    flushLog()
    return true
  end
  state.status = "Линия не найдена"
  state.loaded = false
  return false
end

------------------------------------------------------------
-- LINE SMOOTHING
------------------------------------------------------------

local function smoothLine()
  local n = #state.nodes
  if n < 5 then return end
  for _ = 1, CFG.SMOOTH_PASSES do
    local newX, newZ = {}, {}
    for i = 1, n do
      local sx, sz, cnt = 0, 0, 0
      for j = -CFG.SMOOTH_WINDOW, CFG.SMOOTH_WINDOW do
        local k = i + j
        if state.loopRoute then
          k = ((k - 1) % n) + 1
        elseif k >= 1 and k <= n then
        else
          k = nil
        end
        if k then
          sx = sx + state.nodes[k].x
          sz = sz + state.nodes[k].z
          cnt = cnt + 1
        end
      end
      newX[i] = sx / cnt
      newZ[i] = sz / cnt
    end
    for i = 1, n do
      state.nodes[i] = vec3(newX[i], state.nodes[i].y, newZ[i])
    end
  end
end

------------------------------------------------------------
-- LINE GEOMETRY
------------------------------------------------------------

local function segmentCount()
  if state.loopRoute then return #state.nodes end
  return math.max(0, #state.nodes - 1)
end

local function segEnd(index)
  local count = #state.nodes
  if state.loopRoute then return state.nodes[(index % count) + 1] end
  return state.nodes[index + 1]
end

local function walkLine(startSeg, startT, distance)
  local segs = segmentCount()
  if segs < 1 then return nil end
  local seg = startSeg
  local t = startT
  local remaining = distance
  for _ = 1, segs + 2 do
    local a = state.nodes[seg]
    local b = segEnd(seg)
    local length = flatDist(a, b)
    if length >= 0.0001 then
      local room = length * (1 - t)
      if room >= remaining then
        local nt = (length * t + remaining) / length
        return vec3(lerp(a.x, b.x, nt), lerp(a.y, b.y, nt), lerp(a.z, b.z, nt))
      end
      remaining = remaining - room
      t = 0
    end
    if state.loopRoute then
      seg = (seg % segs) + 1
    elseif seg < segs then
      seg = seg + 1
    else
      return nil
    end
  end
  return nil
end

local function getVertexCurvature(index)
  local nodes = state.nodes
  local count = #nodes
  if count < 5 then return 0 end
  local W = 4
  local i0 = index - W
  local i1 = index
  local i2 = index + W
  if state.loopRoute then
    i0 = ((i0 - 1) % count) + 1
    i1 = ((i1 - 1) % count) + 1
    i2 = ((i2 - 1) % count) + 1
  else
    if i0 < 1 or i2 > count then return 0 end
  end
  local a, b, c = nodes[i0], nodes[i1], nodes[i2]
  local v1x, v1z = b.x - a.x, b.z - a.z
  local v2x, v2z = c.x - b.x, c.z - b.z
  local l1 = math.sqrt(v1x * v1x + v1z * v1z)
  local l2 = math.sqrt(v2x * v2x + v2z * v2z)
  if l1 < 0.5 or l2 < 0.5 then return 0 end
  local dot = clamp((v1x * v2x + v1z * v2z) / (l1 * l2), -1, 1)
  local angle = math.acos(dot)
  if angle < 0.02 then return 0 end
  return angle / math.max(0.1, (l1 + l2) * 0.5)
end

local function findNearestSegment(car, p)
  local nodes = state.nodes
  local segs = segmentCount()
  if segs < 1 then return 1, 0, 0, segs end
  local px = car.position.x
  local pz = car.position.z
  local bestSeg = clamp(p.segIdx, 1, segs)
  local bestT = 0
  local bestDist = math.huge
  local function checkSegment(index)
    local a = nodes[index]
    local b = segEnd(index)
    local vx = b.x - a.x
    local vz = b.z - a.z
    local lenSq = vx * vx + vz * vz
    if lenSq < 0.00001 then return end
    local t = clamp(((px - a.x) * vx + (pz - a.z) * vz) / lenSq, 0, 1)
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
  local back = math.min(3, segs - 1)
  local ahead = math.min(segs - 1, math.max(15, math.floor(segs * 0.20)))
  for offset = -back, ahead do
    local idx
    if state.loopRoute then
      idx = ((p.segIdx - 1 + offset) % segs) + 1
    else
      idx = p.segIdx + offset
      if idx >= 1 and idx <= segs then checkSegment(idx) end
      idx = nil
    end
    if idx then checkSegment(idx) end
  end
  if bestDist > 20 * 20 then
    for i = 1, segs do checkSegment(i) end
  end
  p.segIdx = bestSeg
  return bestSeg, bestT, math.sqrt(bestDist), segs
end

-- corridor half-width at a node (default wide)
local function corridorAt(index)
  local v = state.corridor[index]
  if type(v) == "number" then return v end
  return 25.0
end

------------------------------------------------------------
-- RAYCAST HELPERS
------------------------------------------------------------

local function castRay(pos, dirx, dirz, length)
  if type(physics) ~= "table" or type(physics.raycastTrack) ~= "function" then
    return -1
  end
  local ok, d = pcall(physics.raycastTrack, pos, vec3(dirx, 0, dirz), length)
  if ok and type(d) == "number" then return d end
  return -1
end

-- live forward wall distance (nil if nothing within range)
local function wallAhead(car)
  local look = car.look
  local fx, fz = normalize2(look.x, look.z)
  local pos = car.position
  local p0 = vec3(pos.x + fx * 3, pos.y + 1.0, pos.z + fz * 3)
  local d = castRay(p0, fx, fz, CFG.WALL_RAY_AHEAD)
  if d >= 0 then
    return d + 3
  end
  return nil
end

-- live room to the left and right of THIS car, right now
local function liveRoomLeftRight(car)
  local look = car.look
  local fx, fz = normalize2(look.x, look.z)
  local pos = car.position
  local p0 = vec3(pos.x, pos.y + 1.0, pos.z)
  local dl = castRay(p0, -fz, fx, CFG.ROOM_RAY)
  local dr = castRay(p0, fz, -fx, CFG.ROOM_RAY)
  if dl < 0 then dl = CFG.ROOM_RAY end
  if dr < 0 then dr = CFG.ROOM_RAY end
  return dl, dr
end

------------------------------------------------------------
-- CARS AHEAD (includes the player: index 0)
------------------------------------------------------------

local function carAheadFull(carIndex, car, lateralWidth)
  local fx, fz = normalize2(car.look.x, car.look.z)
  local sim = ac.getSim()
  if not sim then return nil, nil, nil end
  local bestGap, bestSpeed, bestIndex = nil, nil, nil
  for i = 0, sim.carsCount - 1 do
    if i ~= carIndex then
      local other = ac.getCar(i)
      if other and other.isActive then
        local dx = other.position.x - car.position.x
        local dz = other.position.z - car.position.z
        local forward = dx * fx + dz * fz
        if forward > 0 and forward < 90 then
          local lateral = math.abs(dx * fz - dz * fx)
          if lateral <= (lateralWidth or CFG.AVOID_LATERAL) then
            if not bestGap or forward < bestGap then
              bestGap = forward
              bestSpeed = math.max(0, safeNumber(other.speedMs))
              bestIndex = i
            end
          end
        end
      end
    end
  end
  return bestGap, bestSpeed, bestIndex
end

local function carAhead(carIndex, car, lateralWidth)
  local gap, speed, index = carAheadFull(carIndex, car, lateralWidth)
  return gap, speed, index
end

------------------------------------------------------------
-- CORRIDOR (median-filtered, saved once)
------------------------------------------------------------

local corridorScan = { active = false, index = 0, saved = {} }

local function medianFilterCorridor(values, window)
  local n = #values
  local out = {}
  local halfW = math.floor(window / 2)
  for i = 1, n do
    local sample = {}
    for j = -halfW, halfW do
      local k = ((i - 1 + j) % n) + 1
      sample[#sample + 1] = values[k]
    end
    table.sort(sample)
    out[i] = sample[math.ceil(#sample / 2)]
  end
  return out
end

local function scanCorridorStep()
  local n = #state.nodes
  if n < 2 then
    corridorScan.active = false
    return
  end
  local i = corridorScan.index
  if i >= n then
    -- finish: median filter, save, activate
    local filtered = medianFilterCorridor(corridorScan.saved, CFG.CORRIDOR_MEDIAN_WINDOW)
    local parts = {}
    for k = 1, n do
      parts[#parts + 1] = string.format("%.1f", math.max(CFG.CORRIDOR_MIN_SAVED, filtered[k]))
    end
    local data = table.concat(parts, " ")
    local path = getCorridorFilePath()
    if io and io.open then
      local ok, f = pcall(io.open, path, "w")
      if ok and f then
        pcall(function() f:write(data) f:close() end)
      end
    end
    for k = 1, n do
      state.corridor[k] = math.max(CFG.CORRIDOR_MIN_SAVED, filtered[k])
    end
    corridorScan.active = false
    writeLog("CORRIDOR_SAVED points=%d median=%d", n, CFG.CORRIDOR_MEDIAN_WINDOW)
    flushLog()
    return
  end
  i = i + 1
  corridorScan.index = i
  local node = state.nodes[i]
  local prevIdx = (i == 1) and 1 or i - 1
  local a = state.nodes[prevIdx]
  local b = state.nodes[(i % n) + 1]
  local lx, lz = normalize2(b.x - a.x, b.z - a.z)
  local pos = vec3(node.x, node.y + CFG.CORRIDOR_RAY_HEIGHT, node.z)
  local dLeft = castRay(pos, -lz, lx, CFG.CORRIDOR_RAY)
  local dRight = castRay(pos, lz, -lx, CFG.CORRIDOR_RAY)
  local half
  if dLeft < 0 and dRight < 0 then
    half = CFG.CORRIDOR_RAY
  elseif dLeft < 0 then
    half = dRight
  elseif dRight < 0 then
    half = dLeft
  else
    half = math.min(dLeft, dRight)
  end
  half = clamp(half, 1.0, CFG.CORRIDOR_RAY)
  corridorScan.saved[i] = half
end

local function loadCorridor()
  local path = getCorridorFilePath()
  if io and io.open then
    local ok, f = pcall(io.open, path, "r")
    if ok and f then
      local success, raw = pcall(function()
        local content = f:read("*all")
        f:close()
        return content
      end)
      if success and type(raw) == "string" and raw ~= "" then
        local vals = {}
        for token in string.gmatch(raw, "%S+") do
          local v = tonumber(token)
          if v then vals[#vals + 1] = v end
        end
        if #vals == #state.nodes then
          -- de-noise the saved values too (median filter)
          local filtered = medianFilterCorridor(vals, CFG.CORRIDOR_MEDIAN_WINDOW)
          for k = 1, #vals do
            state.corridor[k] = math.max(CFG.CORRIDOR_MIN_SAVED, filtered[k])
          end
          writeLog("CORRIDOR_LOADED points=%d median=%d", #vals, CFG.CORRIDOR_MEDIAN_WINDOW)
          return true
        end
      end
    end
  end
  return false
end

local function updateCorridorScan()
  if corridorScan.active then
    local budget = 6
    for _ = 1, budget do
      if not corridorScan.active then break end
      scanCorridorStep()
    end
  end
end

------------------------------------------------------------
-- TELEMETRY
------------------------------------------------------------

local function recordSecondTelemetry(sim)
  local anyWritten = false
  for i = 1, sim.carsCount - 1 do
    local car = ac.getCar(i)
    local p = state.p[i]
    if car and car.isActive and car.isAIControlled and p then
      if Driver and Driver.telemetryLine then
        writeLog(Driver.telemetryLine(i, car, p))
      else
        writeLog("CAR=%d SPEED=%.2f SEGMENT=%d", i, safeNumber(car.speedMs), p.segIdx or 0)
      end
      anyWritten = true
    end
  end
  if not anyWritten and logger and logger.heartbeat then
    pcall(logger.heartbeat, logger, "nodes=" .. tostring(#state.nodes))
  end
  flushLog()
end

------------------------------------------------------------
-- SPAWN
------------------------------------------------------------

local function buildSpawnRouteMetrics()
  local count = #state.nodes
  if count < 2 then return nil end
  local segs = segmentCount()
  local segments = {}
  local totalLength = 0
  for i = 1, segs do
    local a = state.nodes[i]
    local b = segEnd(i)
    local length = flatDist(a, b)
    if length > 0.05 then
      segments[#segments + 1] = { a = a, b = b, start = totalLength, length = length }
      totalLength = totalLength + length
    end
  end
  if #segments < 2 or totalLength < CFG.SPAWN_MIN_GAP * 2 then return nil end
  return { segments = segments, length = totalLength, loop = state.loopRoute }
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
      local t = clamp(((position.x - a.x) * vx + (position.z - a.z) * vz) / lenSq, 0, 1)
      local px = a.x + vx * t
      local pz = a.z + vz * t
      local dx = position.x - px
      local dz = position.z - pz
      local dSq = dx * dx + dz * dz
      if dSq < bestDistanceSq then
        bestDistanceSq = dSq
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
      local position = vec3(lerp(a.x, b.x, t), lerp(a.y, b.y, t), lerp(a.z, b.z, t))
      local direction = vec3(b.x - a.x, b.y - a.y, b.z - a.z)
      if direction:length() < 0.0001 then return nil, nil end
      direction:normalize()
      return position, direction
    end
  end
  return nil, nil
end

local function collectActiveAICars(sim)
  local cars = {}
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
  local gap = math.min(CFG.SPAWN_GAP, metrics.length / (count + 1))
  if gap < CFG.SPAWN_MIN_GAP then
    state.status = "Недостаточно места для расстановки"
    writeLog("SPAWN_ABORT reason=route_too_short cars=%d route=%.1f gap=%.1f",
      count, metrics.length, gap)
    state.spawnDone = true
    flushLog()
    return
  end
  local startDistance = 0
  if metrics.loop then
    local player = ac.getCar(0)
    if player then
      local nearestDistance, playerLineError = findRouteDistanceAtPosition(player.position, metrics)
      startDistance = nearestDistance + gap
      writeLog("SPAWN_ANCHOR player_line_error=%.2f route_distance=%.2f",
        playerLineError, nearestDistance)
    else
      startDistance = gap
    end
  end
  local moved, failed = 0, 0
  writeLog("SPAWN_BEGIN cars=%d route=%.1f gap=%.1f start=%.1f loop=%s",
    count, metrics.length, gap, startDistance, tostring(metrics.loop))
  for order = 1, count do
    local carIndex = aiCars[order]
    local routeDistance = startDistance + (order - 1) * gap
    local position, direction = sampleSpawnRoute(metrics, routeDistance)
    if position and direction then
      -- this CSP build points the car OPPOSITE to the passed dir (verified)
      local ok, result = pcall(physics.setAICarPosition, carIndex,
        position, vec3(-direction.x, -direction.y, -direction.z))
      if ok and result ~= false then
        moved = moved + 1
        state.p[carIndex] = nil
        writeLog("SPAWN_CAR id=%d order=%d distance=%.2f pos=(%.2f,%.2f,%.2f) dir=(%.3f,%.3f,%.3f) inverted=1",
          carIndex, order, routeDistance,
          position.x, position.y, position.z,
          -direction.x, -direction.y, -direction.z)
      else
        failed = failed + 1
        writeLog("SPAWN_CAR_FAILED id=%d order=%d error=%s", carIndex, order, tostring(result))
      end
    else
      failed = failed + 1
      writeLog("SPAWN_CAR_FAILED id=%d order=%d reason=invalid_sample", carIndex, order)
    end
  end
  state.spawnDone = true
  if failed == 0 then
    state.status = string.format("ИИ расставлен: %d машин, интервал %.0f м", moved, gap)
  else
    state.status = string.format("Телепортировано %d из %d", moved, count)
  end
  writeLog("SPAWN_FINISH moved=%d failed=%d total=%d gap=%.2f", moved, failed, count, gap)
  flushLog()
end

local function updateStartupTeleport(sim, dt)
  if state.spawnDone then return true end
  if not state.loaded or state.recording or #state.nodes < 2 then return true end
  state.spawnWait = state.spawnWait + dt
  local aiCars = collectActiveAICars(sim)
  local count = #aiCars
  local roster = table.concat(aiCars, ",")
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
-- RECORDING / RELEASE
------------------------------------------------------------

local function addPoint(pos)
  if #state.nodes >= CFG.MAX_ROUTE_POINTS then
    state.recording = false
    state.status = "Достигнут лимит точек"
    return
  end
  if #state.nodes == 0 or flatDist(state.nodes[#state.nodes], pos) >= CFG.RECORD_DISTANCE then
    state.nodes[#state.nodes + 1] = vec3(pos.x, pos.y + 0.1, pos.z)
  end
end

local function releaseCar(carIndex)
  local ok, controls = pcall(ac.overrideCarControls, carIndex)
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
-- DRIVER WIRING
------------------------------------------------------------

local driverReady = false

local function ensureDriver()
  if driverReady or not Driver then return Driver ~= nil end
  if type(Driver.setup) ~= "function" then return false end
  Driver.setup(CFG, state, {
    segEnd = segEnd,
    segmentCount = segmentCount,
    walkLine = walkLine,
    flatDist = flatDist,
    getVertexCurvature = getVertexCurvature,
    findNearestSegment = findNearestSegment,
    carAhead = carAhead,
    corridorAt = corridorAt,
    wallAhead = wallAhead,
    liveRoomLeftRight = liveRoomLeftRight,
    log = writeLog,
  })
  driverReady = true
  return true
end

------------------------------------------------------------
-- UPDATE
------------------------------------------------------------

function script.update(dt)
  if not dt or dt <= 0 then return end
  dt = math.min(dt, 0.05)
  local sim = ac.getSim()
  if not sim then return end
  local player = ac.getCar(0)

  if state.recording and player then
    addPoint(player.position)
  end

  updateCorridorScan()

  local startupReady = true
  if state.enabled and state.loaded and #state.nodes >= 2 then
    startupReady = updateStartupTeleport(sim, dt)
  end

  if not state.enabled or #state.nodes < 2 or not Driver or not ensureDriver() then
    for idx in pairs(state.p) do
      releaseCar(idx)
    end
  elseif startupReady then
    for i = 1, sim.carsCount - 1 do
      local car = ac.getCar(i)
      if car and car.isActive and car.isAIControlled then
        local ok, err = pcall(Driver.update, i, car, dt)
        if not ok then
          writeLog("DRIVER_ERROR id=%d err=%s", i, tostring(err))
          releaseCar(i)
        end
      elseif state.p[i] then
        releaseCar(i)
      end
    end
  else
    local playerCar = ac.getCar(0)
    if playerCar and playerCar.isActive and playerCar.isAIControlled then
      local ok, err = pcall(Driver.update, 0, playerCar, dt)
      if not ok then
        writeLog("DRIVER_ERROR id=0 err=%s", tostring(err))
      end
    end
  end

  state.logAccum = state.logAccum + dt
  if state.logAccum >= CFG.LOG_INTERVAL then
    state.logAccum = state.logAccum - CFG.LOG_INTERVAL
    recordSecondTelemetry(sim)
  end
end

------------------------------------------------------------
-- UI (verified calls only; buttons on top, map fills the rest)
------------------------------------------------------------

local mapBounds = nil

local function computeMapBounds()
  if mapBounds then return mapBounds end
  local minX, maxX, minZ, maxZ = math.huge, -math.huge, math.huge, -math.huge
  for i = 1, #state.nodes do
    local n = state.nodes[i]
    if n.x < minX then minX = n.x end
    if n.x > maxX then maxX = n.x end
    if n.z < minZ then minZ = n.z end
    if n.z > maxZ then maxZ = n.z end
  end
  if minX == math.huge then return nil end
  mapBounds = { minX = minX, maxX = maxX, minZ = minZ, maxZ = maxZ }
  return mapBounds
end

local function invalidateMap()
  mapBounds = nil
end

-- draw the map inside rect (p0, p1); only verified ui.* calls
local function drawMapInRect(p0, p1)
  if #state.nodes < 2 then return end
  local b = computeMapBounds()
  if not b then return end

  ui.drawRectFilled(p0, p1, rgbm(0.08, 0.08, 0.10, 1), 4)
  ui.drawRect(p0, p1, rgbm(0.3, 0.3, 0.35, 1), 4)

  local spanX = math.max(1, b.maxX - b.minX) + 24
  local spanZ = math.max(1, b.maxZ - b.minZ) + 24
  local scale = math.min((p1.x - p0.x) / spanX, (p1.y - p0.y) / spanZ)
  local cx = (b.minX + b.maxX) * 0.5
  local cz = (b.minZ + b.maxZ) * 0.5
  local mx = (p0.x + p1.x) * 0.5
  local my = (p0.y + p1.y) * 0.5

  local function toScreen(n)
    return vec2(mx + (n.x - cx) * scale, my + (n.z - cz) * scale)
  end

  -- corridor edges (dim green)
  if #state.corridor == #state.nodes then
    for i = 1, #state.nodes, 2 do
      local a = state.nodes[i]
      local prev = state.nodes[((i - 1) % #state.nodes) + 1]
      local lx, lz = normalize2(a.x - prev.x, a.z - prev.z)
      local half = state.corridor[i]
      local sL = toScreen({ x = a.x - lz * half, z = a.z + lx * half })
      local sR = toScreen({ x = a.x + lz * half, z = a.z - lx * half })
      ui.drawCircleFilled(sL, 1.0, rgbm(0.1, 0.5, 0.2, 0.5), 6)
      ui.drawCircleFilled(sR, 1.0, rgbm(0.1, 0.5, 0.2, 0.5), 6)
    end
  end

  local prev = toScreen(state.nodes[1])
  for i = 2, #state.nodes do
    local cur = toScreen(state.nodes[i])
    ui.drawLine(prev, cur, rgbm(1.0, 0.85, 0.0, 0.9), 2)
    prev = cur
  end
  if state.loopRoute then
    ui.drawLine(prev, toScreen(state.nodes[1]), rgbm(1.0, 0.85, 0.0, 0.9), 2)
  end

  local sim = ac.getSim()
  if sim then
    for i = 0, sim.carsCount - 1 do
      local car = ac.getCar(i)
      if car and car.isActive then
        local s = toScreen(car.position)
        if s.x > p0.x and s.x < p1.x and s.y > p0.y and s.y < p1.y then
          if i == 0 then
            ui.drawCircleFilled(s, 4.5, rgbm(0.0, 0.8, 1.0, 1.0), 12)
          else
            local p = state.p[i]
            if p and p.overtakeSide ~= 0 then
              ui.drawCircleFilled(s, 3.5, rgbm(0.2, 1.0, 0.4, 1.0), 10)
            else
              ui.drawCircleFilled(s, 3.5, rgbm(1, 1, 1, 0.9), 10)
            end
          end
        end
      end
    end
  end
end

local function drawMapFixed()
  -- fixed-size canvas as a normal layout item (verified calls only):
  -- full window width, height from the remaining space
  local width = ui.availableSpaceX()
  if width < 80 then width = 80 end
  local height = ui.availableSpaceY and ui.availableSpaceY() or 0
  if type(height) ~= "number" or height < 120 then
    height = math.max(140, math.min(300, width * 0.6))
  end
  ui.invisibleButton("line_map", vec2(width, height))
  local p0 = ui.getItemRectMin()
  local p1 = ui.getItemRectMax()
  drawMapInRect(p0, p1)
end

function windowMain(dt)
  local okDraw = pcall(function()
    -- controls pinned to the TOP
    ui.text(string.format("Точек: %d | %s", #state.nodes, state.status))

    if ui.button(state.recording and "Стоп запись" or "Запись линии", vec2(-1, 26)) then
      state.recording = not state.recording
      if state.recording then
        state.nodes = {}
        state.p = {}
        state.corridor = {}
        state.loaded = false
        invalidateMap()
        state.status = "Запись линии..."
      else
        state.status = "Записано точек: " .. #state.nodes
        smoothLine()
      end
    end

    if ui.button(state.loopRoute and "Круг: ВКЛ" or "Круг: ВЫКЛ", vec2(-1, 24)) then
      state.loopRoute = not state.loopRoute
      state.p = {}
    end

    if ui.button("Сохранить", vec2(-1, 24)) then saveLine() end
    if ui.button("Загрузить", vec2(-1, 24)) then
      if loadLine() then
        invalidateMap()
        smoothLine()
        state.corridor = {}
        if not loadCorridor() and #state.nodes >= 2 then
          corridorScan.active = true
          corridorScan.index = 0
          corridorScan.saved = {}
          writeLog("CORRIDOR_SCAN_START points=%d", #state.nodes)
        end
      end
    end
    if ui.button("Очистить", vec2(-1, 24)) then
      state.nodes = {}
      state.p = {}
      state.corridor = {}
      state.status = "Очищено"
      state.loaded = false
      invalidateMap()
    end

    -- map fills the rest of the window
    ui.separator()
    drawMapFixed()
  end)
  if not okDraw then
    pcall(ui.text, "UI error")
  end
end

------------------------------------------------------------
-- RELEASE + INIT
------------------------------------------------------------

if ac.onRelease then
  ac.onRelease(function()
    for idx in pairs(state.p) do releaseCar(idx) end
    if logger and logger.shutdown then
      pcall(logger.shutdown, logger, "RoutePilot released")
    end
  end)
end

loadLine()
smoothLine()
if not loadCorridor() and #state.nodes >= 2 then
  corridorScan.active = true
  corridorScan.index = 0
  corridorScan.saved = {}
  writeLog("CORRIDOR_SCAN_START points=%d", #state.nodes)
end
writeLog("ROUTEPILOT INITIALIZED track=%s points=%d", getTrackTag(), #state.nodes)
flushLog()
