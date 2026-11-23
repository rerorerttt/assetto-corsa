-- RoutePilot.lua
-- RoutePilot 0.4 — Arcade Racing Traffic
-- CSP Lua App. Controls AI cars along a recorded line (closed loop supported).
--
-- Fix history of 0.4:
--  * SPAWN HEADING: physics.setAICarPosition(carIndex, pos, dir) points the
--    car OPPOSITE to dir on this CSP build (verified in the trace log:
--    requested dir=(0,0,1) resulted in LOOK=(0,-1) for every car). Now the
--    direction is inverted before the call, and after teleport each car
--    gets a heading auto-correction: if the car's LOOK is backwards
--    relative to the line direction, the controller turns it around
--    within a couple of seconds.
--  * LOW-SPEED SLIP DAMPING removed: at walking speeds the lateral
--    velocity noise created a fake "slip" that cut the throttle to 0.08
--    and cars crawled. Slip-based damping now engages only above 8 m/s
--    and above 4 degrees of real slip angle.
--  * RACING TEMPO: cars drive as fast as the line allows — the planner
--    looks 170 m ahead, computes corner apex speeds from curvature and
--    brakes exactly the distance needed. Per-car pace multiplier
--    (0.85..1.15) makes fast cars overtake slow ones naturally.
--  * OVERTAKING: a faster car does not brake behind a slower one — it
--    shifts to the overtake side (left/right of the line) and passes,
--    then returns to the line. Slower cars hold their line and pace
--    (they do not yield and do not brake; the passing car does the work).
--  * COLLISION SAFETY: forward-cone check against ALL cars including
--    the player; hard brake only if the space ahead is actually taken.
--  * MAP: the track map is embedded in the MAIN window as a static
--    canvas that fills the whole window width. No separate floating map
--    window anymore (the broken manifest is replaced).
--  * UI: only working elements remain — record/stop, save/load/clear,
--    loop toggle, enable toggle, speed sliders, follow distance.

local okLogger, Logger = pcall(require, "logger")
if not okLogger or type(Logger) ~= "table" then
  Logger = nil
end

------------------------------------------------------------
-- CONFIGURATION
------------------------------------------------------------

local CFG = {
  -- steering
  LOOKAHEAD_BASE = 8.0,
  LOOKAHEAD_SPEED = 0.55,
  LOOKAHEAD_MIN = 8.0,
  LOOKAHEAD_MAX = 30.0,
  STEER_GAIN = 0.78,
  STEER_LIMIT = 1.0,
  STEER_RATE = 3.5,
  STEER_FILTER = 0.10,

  -- grip and speed planning
  MAX_LATERAL_ACCEL = 9.0,     -- arcade grip for corner speeds
  BRAKE_ACCEL = 8.5,           -- arcade braking power
  PLAN_HORIZON = 170.0,        -- look this far ahead for corners
  CORNER_HORIZON = 45.0,       -- where corner speed matters without braking
  MIN_APEX_SPEED = 11.0,       -- m/s, never plan below this (~40 km/h)
  GAS_GAIN = 0.45,
  BRAKE_GAIN = 0.60,
  MAX_GAS = 1.0,
  MAX_BRAKE = 1.0,

  -- slip / stability
  SLIP_START = 0.09,
  SLIP_FULL = 0.45,
  SLIP_DAMP_MIN_SPEED = 8.0,   -- below this speed no slip damping at all
  SLIP_DAMP_MIN_ANGLE = 4.0 * math.pi / 180,

  -- following and overtaking
  FOLLOW_LATERAL = 4.0,        -- cone half-width for cars ahead
  FOLLOW_DISTANCE = 8.0,
  OVERTAKE_TRIGGER_TIME = 1.2, -- follow this long before committing
  OVERTAKE_LATERAL = 5.5,      -- shift this far from the line to pass
  OVERTAKE_CLEAR_AHEAD = 4.0,  -- end overtake this far past the leader
  OVERTAKE_SIDE_PREF = 1,      -- 1 = prefer right side of the line

  -- line keeping
  LINE_HARD_PULL = 25.0,       -- further than this: strong pull to the line
  LINE_SOFT_PULL = 10.0,       -- further than this: moderate pull

  -- spawning
  SPAWN_GAP = 25.0,
  SPAWN_MIN_GAP = 14.0,
  SPAWN_STABLE_TIME = 0.35,
  SPAWN_WAIT_TIMEOUT = 8.0,
  SPAWN_CORRECT_TIME = 6.0,    -- heading auto-correction window

  -- telemetry
  LOG_INTERVAL = 1.0,
  MAX_ROUTE_POINTS = 50000,
  RECORD_DISTANCE = 3.0,

  -- map
  MAP_MARGIN = 10.0,           -- padding around the drawn line, meters
}

------------------------------------------------------------
-- STATE
------------------------------------------------------------

local state = {
  enabled = true,
  recording = false,
  loopRoute = true,

  speedMin = 70,
  speedMax = 170,

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

-- deterministic per-car random from the index (stable across frames)
local function carRand(carIndex, salt)
  local n = math.sin(carIndex * 127.1 + salt * 311.7) * 43758.5453
  return n - math.floor(n)
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
  local tag = string.gsub(getTrackTag(), "[^%w_%-]", "_")
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
    lines[#lines + 1] = string.format("%.4f %.4f %.4f", n.x, n.y, n.z)
  end
  return table.concat(lines, "\n")
end

local function parseNodes(raw)
  if type(raw) ~= "string" or raw == "" then
    return nil
  end
  local loaded = {}
  for line in string.gmatch(raw, "[^\r\n]+") do
    local xs, ys, zs = string.match(line, "^%s*([%-0-9%.]+)%s+([%-0-9%.]+)%s+([%-0-9%.]+)%s*$")
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
-- SAVE / LOAD LINE
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
    local ok = pcall(ac.storage, "RP_LINE_DATA_" .. getTrackTag(), data)
    wroteStorage = ok
  end
  if wroteFile or wroteStorage then
    state.status = string.format("Сохранено: %d точек", #state.nodes)
    writeLog("SAVE_ROUTE points=%d file=%s storage=%s",
      #state.nodes, tostring(wroteFile), tostring(wroteStorage))
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
      if success then
        loaded = parseNodes(raw)
      end
    end
  end
  if not loaded and ac.storage then
    local ok, raw = pcall(ac.storage, "RP_LINE_DATA_" .. getTrackTag())
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
-- LINE GEOMETRY (nodes are stored in route order)
------------------------------------------------------------

local function nodeCount()
  return #state.nodes
end

local function segmentCount()
  if state.loopRoute then
    return #state.nodes
  end
  return math.max(0, #state.nodes - 1)
end

local function segEnd(index)
  local count = #state.nodes
  if state.loopRoute then
    return state.nodes[(index % count) + 1]
  end
  return state.nodes[index + 1]
end

-- walk `distance` meters along the line from segment index / t; returns a point
local function walkLine(startSeg, startT, distance)
  local count = #state.nodes
  local segs = segmentCount()
  if segs < 1 then return nil end
  local seg = startSeg
  local t = startT
  local remaining = distance
  for _ = 1, segs + 2 do
    local a = state.nodes[seg]
    local b = segEnd(seg)
    local length = flatDist(a, b)
    local room = length * (1 - t)
    if room >= remaining then
      local nt = (length * t + remaining) / math.max(0.0001, length)
      return vec3(lerp(a.x, b.x, nt), lerp(a.y, b.y, nt), lerp(a.z, b.z, nt)), seg, nt
    end
    remaining = remaining - room
    t = 0
    if state.loopRoute then
      seg = (seg % segs) + 1
    elseif seg < segs then
      seg = seg + 1
    else
      return vec3(b.x, b.y, b.z), seg, 1
    end
  end
  return nil
end

------------------------------------------------------------
-- STARTUP AI TELEPORT
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
  if #segments < 2 or totalLength < CFG.SPAWN_MIN_GAP * 2 then
    return nil
  end
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
      local position = vec3(lerp(a.x, b.x, t), lerp(a.y, b.y, t), lerp(a.z, b.z, t))
      local direction = vec3(b.x - a.x, b.y - a.y, b.z - a.z)
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
      writeLog("SPAWN_ANCHOR player_line_error=%.2f route_distance=%.2f", playerLineError, nearestDistance)
    else
      startDistance = gap
    end
  end
  local moved = 0
  local failed = 0
  writeLog("SPAWN_BEGIN cars=%d route=%.1f gap=%.1f start=%.1f loop=%s",
    count, metrics.length, gap, startDistance, tostring(metrics.loop))
  for order = 1, count do
    local carIndex = aiCars[order]
    local routeDistance = startDistance + (order - 1) * gap
    local position, direction = sampleSpawnRoute(metrics, routeDistance)
    if position and direction then
      -- THE FIX: on this CSP build the car ends up facing OPPOSITE to the
      -- given dir (verified via trace log). Pass the inverted direction.
      local ok, result = pcall(physics.setAICarPosition, carIndex,
        position, vec3(-direction.x, -direction.y, -direction.z))
      if ok and result ~= false then
        moved = moved + 1
        state.p[carIndex] = nil
        writeLog("SPAWN_CAR id=%d order=%d distance=%.2f pos=(%.2f,%.2f,%.2f) dir=(%.3f,%.3f,%.3f)",
          carIndex, order, routeDistance, position.x, position.y, position.z,
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
-- RECORD PLAYER LINE
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

------------------------------------------------------------
-- CAR PERSONALITY
------------------------------------------------------------

local function newPersonality(carIndex)
  local span = math.max(0, state.speedMax - state.speedMin)
  -- deterministic per-car pace so fast cars overtake slow ones
  local pace = 0.85 + carRand(carIndex, 1) * 0.30   -- 0.85..1.15
  local kmh = clamp(state.speedMin + span * 0.5, 40, 260) * pace
  local p = {
    segIdx = 1,
    pace = pace,
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
    stuck = 0,
    lineDist = 0,
    targetX = 0,
    targetZ = 0,
    targetSpeed = 0,
    slipSeverity = 0,
    stability = 1,
    lastCurve = 0,
    -- heading auto-correction after teleport
    correctTimer = CFG.SPAWN_CORRECT_TIME,
    -- overtaking state
    overtakeSide = 0,            -- -1 left, 1 right, 0 none
    overtakeTarget = nil,       -- carIndex being passed
    followTimer = 0,
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
  writeLog("CAR_INIT id=%d cruise=%.1fkmh pace=%.2f segment=%d", carIndex, kmh, pace, p.segIdx)
end

------------------------------------------------------------
-- SEGMENT SEARCH
------------------------------------------------------------

local function findNearestSegment(car, p)
  local nodes = state.nodes
  local count = #nodes
  local segs = segmentCount()
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
      if idx >= 1 and idx <= segs then
        checkSegment(idx)
      end
      idx = nil
    end
    if idx then
      checkSegment(idx)
    end
  end
  if bestDist > 20 * 20 then
    for i = 1, segs do
      checkSegment(i)
    end
  end
  p.segIdx = bestSeg
  return bestSeg, bestT, math.sqrt(bestDist), segs
end

------------------------------------------------------------
-- LOOKAHEAD TARGET
------------------------------------------------------------

local function getLookaheadTarget(car, p, speed)
  local nodes = state.nodes
  local count = #nodes
  local seg, t = findNearestSegment(car, p)
  local a = nodes[seg]
  local b = segEnd(seg)
  local startX = lerp(a.x, b.x, t)
  local startZ = lerp(a.z, b.z, t)
  local lookAhead = clamp(
    CFG.LOOKAHEAD_BASE + speed * CFG.LOOKAHEAD_SPEED,
    CFG.LOOKAHEAD_MIN,
    CFG.LOOKAHEAD_MAX
  )
  local point = walkLine(seg, t, lookAhead)
  if point then
    return point.x, point.z, seg, t
  end
  return b.x, b.z, seg, t
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
  local dot = clamp((v1x * v2x + v1z * v2z) / (l1 * l2), -1, 1)
  local angle = math.acos(dot)
  return angle / math.max(0.1, (l1 + l2) * 0.5)
end

-- racing planner: scan PLAN_HORIZON ahead, compute the allowed speed at each
-- corner and brake exactly as late as physics allows
local function planTargetSpeed(p, speed)
  local nodes = state.nodes
  local count = #nodes
  local segs = segmentCount()
  local targetSpeed = p.cruiseSpeed
  local distance = 0
  local current = p.segIdx
  local maxCurve = 0
  for _ = 1, 90 do
    if distance > CFG.PLAN_HORIZON then break end
    local curvature = getVertexCurvature(current)
    if curvature > 0.0008 then
      maxCurve = math.max(maxCurve, curvature)
      local cornerSpeed = math.max(CFG.MIN_APEX_SPEED, math.sqrt(CFG.MAX_LATERAL_ACCEL / curvature))
      local safeSpeed = math.sqrt(cornerSpeed * cornerSpeed + 2 * CFG.BRAKE_ACCEL * distance)
      if safeSpeed < targetSpeed then
        targetSpeed = safeSpeed
      end
    end
    local a = nodes[current]
    local b = segEnd(current)
    local segLen = flatDist(a, b)
    distance = distance + segLen
    if state.loopRoute then
      current = (current % segs) + 1
    elseif current < segs then
      current = current + 1
    else
      break
    end
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
  local rx = fz
  local rz = -fx
  local vx = safeNumber(velocity.x)
  local vz = safeNumber(velocity.z)
  local longitudinalVelocity = vx * fx + vz * fz
  local lateralVelocity = vx * rx + vz * rz
  local slipAngle = math.atan2(lateralVelocity, math.max(0.5, longitudinalVelocity))
  local yawRate = 0
  if p.previousFx and dt > 0.0001 then
    local headingChange = signedAngle(p.previousFx, p.previousFz, fx, fz)
    yawRate = headingChange / dt
  end
  local speed = math.max(0, safeNumber(car.speedMs, longitudinalVelocity))
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
-- STABILITY
------------------------------------------------------------

local function calculateStability(p, speed)
  local slip = math.abs(p.slipAngle)
  local severity = 0
  -- FIX: only apply slip damping at real speeds with real slip angles;
  -- at low speed the lateral noise used to create fake slip and kill gas
  if speed > CFG.SLIP_DAMP_MIN_SPEED and slip > CFG.SLIP_DAMP_MIN_ANGLE then
    local slipSeverity = clamp((slip - CFG.SLIP_START) / (CFG.SLIP_FULL - CFG.SLIP_START), 0, 1)
    local accelerationSeverity = clamp(
      (p.lateralAccel - CFG.MAX_LATERAL_ACCEL) / (CFG.MAX_LATERAL_ACCEL * 1.6 - CFG.MAX_LATERAL_ACCEL),
      0, 1)
    severity = math.max(slipSeverity, accelerationSeverity)
  end
  p.slipSeverity = severity
  p.stability = 1 - severity
  return severity
end

------------------------------------------------------------
-- PURE PURSUIT STEERING (with heading auto-correction)
------------------------------------------------------------

local function headingMatchesLine(fx, fz, seg)
  local a = state.nodes[seg]
  local b = segEnd(seg)
  local dx, dz = normalize2(b.x - a.x, b.z - a.z)
  return (fx * dx + fz * dz) > -0.1
end

local function calculateSteering(car, p, speed, fx, fz)
  local targetX, targetZ, seg, t = getLookaheadTarget(car, p, speed)
  p.targetX = targetX
  p.targetZ = targetZ
  p.segIdx = seg

  -- heading auto-correction right after teleport: if the car faces
  -- backwards along the line, turn it around instead of steering crazy
  if p.correctTimer > 0 then
    p.correctTimer = p.correctTimer - dtLastFrame
    if not headingMatchesLine(fx, fz, seg) then
      return (fz > 0) and 1 or -1, seg, t  -- steady full-lock turn
    end
  end

  local dx = targetX - car.position.x
  local dz = targetZ - car.position.z
  local distance = math.sqrt(dx * dx + dz * dz)
  if distance < 0.001 then
    return 0, seg, t
  end
  local dirX = dx / distance
  local dirZ = dz / distance
  local angle = signedAngle(fx, fz, dirX, dirZ)
  local routeSteer = angle * CFG.STEER_GAIN
  local slipCompensation = p.slipAngle * 0.62
  local steer = routeSteer + slipCompensation

  -- overtaking: shift the steering target laterally off the line
  if p.overtakeSide ~= 0 then
    local lateral = p.overtakeSide * CFG.OVERTAKE_LATERAL
    local lineDirX, lineDirZ = normalize2(targetX - car.position.x, targetZ - car.position.z)
    local nx, nz = -lineDirZ, lineDirX
    local shiftedX = targetX + nx * lateral
    local shiftedZ = targetZ + nz * lateral
    dx = shiftedX - car.position.x
    dz = shiftedZ - car.position.z
    distance = math.sqrt(dx * dx + dz * dz)
    if distance > 0.001 then
      angle = signedAngle(fx, fz, dx / distance, dz / distance)
      steer = angle * CFG.STEER_GAIN + slipCompensation
    end
  end

  -- keep the car from straying too far from the line
  local segA = state.nodes[seg]
  local segB = segEnd(seg)
  local lineDx, lineDz = normalize2(segB.x - segA.x, segB.z - segA.z)
  local toCarX = car.position.x - segA.x
  local toCarZ = car.position.z - segA.z
  local along = toCarX * lineDx + toCarZ * lineDz
  local lateralOffset = -(toCarX * lineDirZ) + (toCarZ * lineDx) * 0 + (toCarX * -lineDirZ + toCarZ * lineDx)
  local perpDist = math.abs(toCarX * -lineDirZ + toCarZ * lineDirX)
  if perpDist > CFG.LINE_HARD_PULL and p.overtakeSide == 0 then
    -- strong pull back to the line
    local side = math.sign((toCarX * -lineDirZ + toCarZ * lineDx)) or 1
    steer = steer - side * 0.4
    steer = clamp(steer, -CFG.STEER_LIMIT, CFG.STEER_LIMIT)
  end

  return clamp(steer, -CFG.STEER_LIMIT, CFG.STEER_LIMIT), seg, t
end

------------------------------------------------------------
-- STEERING FILTER
------------------------------------------------------------

local function filterSteering(p, target, dt)
  local alpha = clamp(dt / (CFG.STEER_FILTER + dt), 0, 1)
  local filtered = lerp(p.steerFiltered, target, alpha)
  local maxChange = CFG.STEER_RATE * dt
  local delta = clamp(filtered - p.steer, -maxChange, maxChange)
  p.steer = clamp(p.steer + delta, -CFG.STEER_LIMIT, CFG.STEER_LIMIT)
  p.steerFiltered = filtered
  return p.steer
end

------------------------------------------------------------
-- CARS AHEAD
------------------------------------------------------------

local function carAhead(carIndex, car, lateralWidth)
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
        if forward > 0 and forward < 90 then
          local lateral = math.abs(dx * fz - dz * fx)
          if lateral <= (lateralWidth or CFG.FOLLOW_LATERAL) then
            if not bestGap or forward < bestGap then
              bestGap = forward
              bestSpeed = math.max(0, safeNumber(other.speedMs))
            end
          end
        end
 