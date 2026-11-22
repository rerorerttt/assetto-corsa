-- RoutePilot: следование по записанным точкам круга + ac.overrideCarControls + логи каждую секунду

local okLogger, Logger = pcall(require, "logger")
if not okLogger or type(Logger) ~= "table" then
  Logger = nil
end

local BRAKE_ACCEL = 7.0
local FOLLOW_LATERAL = 3.0
local WALL_MARGIN = 6.0

-- Настройки стабилизации руления
local LOOKAHEAD_BASE = 18.0
local LOOKAHEAD_SPEED = 0.55
local LOOKAHEAD_MIN = 18.0
local LOOKAHEAD_MAX = 30.0

local STEER_GAIN = 0.82
local STEER_LIMIT = 0.68
local STEER_RATE = 1.15
local STEER_DEADBAND = 0.012

local state = {
  enabled = true,
  recording = false,
  loopRoute = true,
  speedMin = 60,
  speedMax = 140,
  followGap = 8.0,
  p = {},
  nodes = {},
  status = "Готов",
  logAccum = 0.0,
}

local logger = nil
if Logger and type(Logger.new) == "function" then
  logger = Logger.new("routepilot_trace.txt", 4000)
  if logger.init then
    logger:init()
  end
end

local function writeLog(fmt, ...)
  if not logger or not logger.log then return end
  logger:log(fmt, ...)
end

local function flushLog()
  if not logger or not logger.flush then return end
  logger:flush()
end

local function clamp(v, lo, hi)
  if v < lo then return lo end
  if v > hi then return hi end
  return v
end

local function flatDist(a, b)
  local dx = b.x - a.x
  local dz = b.z - a.z
  return math.sqrt(dx * dx + dz * dz)
end

local function getTrackTag()
  local tid = (ac.getTrackID and ac.getTrackID()) or "track"
  local lay = (ac.getTrackLayout and ac.getTrackLayout()) or ""
  if lay ~= "" then
    return tostring(tid) .. "_" .. tostring(lay)
  end
  return tostring(tid)
end

local function getLineFilePath()
  local appDir = (ac.getFolder and ac.getFolder(ac.FolderID.ACApps)) or ""
  local safeTag = string.gsub(getTrackTag(), "[^%w_%-]", "_")
  if appDir ~= "" then
    return appDir .. "/lua/RoutePilot/line_" .. safeTag .. ".txt"
  end
  return "line_" .. safeTag .. ".txt"
end

local function serializeNodes(nodes)
  local lines = {}
  for i = 1, #nodes do
    local n = nodes[i]
    table.insert(lines, string.format("%.4f %.4f %.4f", n.x, n.y, n.z))
  end
  return table.concat(lines, "\n")
end

local function parseNodes(raw)
  if type(raw) ~= "string" or raw == "" then return nil end
  local loaded = {}
  for line in string.gmatch(raw, "[^\r\n]+") do
    local xs, ys, zs = string.match(line, "^%s*([%-0-9.]+)%s+([%-0-9.]+)%s+([%-0-9.]+)%s*$")
    local x, y, z = tonumber(xs), tonumber(ys), tonumber(zs)
    if x and y and z then
      table.insert(loaded, vec3(x, y, z))
    end
  end
  if #loaded >= 2 then
    return loaded
  end
  return nil
end

local function saveLine()
  if #state.nodes < 2 then
    state.status = "Нужно минимум 2 точки"
    return
  end

  local data = serializeNodes(state.nodes)
  local wroteFile = false
  local path = getLineFilePath()

  if io and io.open then
    local okOpen, f = pcall(io.open, path, "w")
    if okOpen and f then
      local okW = pcall(function()
        f:write(data)
        f:close()
      end)
      if okW then wroteFile = true end
    end
  end

  local wroteStorage = false
  if ac.storage then
    local okS = pcall(ac.storage, "RP_LINE_DATA_" .. getTrackTag(), data)
    if okS then wroteStorage = true end
  end

  if wroteFile or wroteStorage then
    state.status = string.format("Сохранено: %d т. (%s)", #state.nodes, wroteFile and "файл" or "storage")
    writeLog("SAVE_LINE count=%d file=%s storage=%s", #state.nodes, tostring(wroteFile), tostring(wroteStorage))
    flushLog()
  else
    state.status = "Ошибка сохранения"
  end
end

local function loadLine()
  local loaded = nil
  local path = getLineFilePath()

  if io and io.open then
    local okOpen, f = pcall(io.open, path, "r")
    if okOpen and f then
      local okR, raw = pcall(function()
        local txt = f:read("*all")
        f:close()
        return txt
      end)
      if okR and raw and raw ~= "" then
        loaded = parseNodes(raw)
      end
    end
  end

  if not loaded and ac.storage then
    local okS, rawSt = pcall(ac.storage, "RP_LINE_DATA_" .. getTrackTag())
    if okS and rawSt and rawSt ~= "" then
      loaded = parseNodes(rawSt)
    end
  end

  if loaded and #loaded >= 2 then
    state.nodes = loaded
    state.p = {}
    state.status = string.format("Загружено: %d т.", #state.nodes)
    writeLog("LOAD_LINE count=%d track=%s", #state.nodes, getTrackTag())
    flushLog()
  else
    state.status = "Линия не найдена"
  end
end

local function addPoint(pos)
  if #state.nodes == 0 or flatDist(state.nodes[#state.nodes], pos) >= 3.0 then
    table.insert(state.nodes, vec3(pos.x, pos.y + 0.1, pos.z))
  end
end

local function newPersonality(carIndex)
  local span = math.max(5, state.speedMax - state.speedMin)
  local kmh = state.speedMin + math.random() * span
  local p = {
    t = math.random() * 100,
    segIdx = 1,
    speed = kmh / 3.6,
    steer = 0,
    gas = 0,
    brake = 0,
    hb = 0,
    stuck = 0,
    lastTargetX = 0,
    lastTargetZ = 0,
    lastLineDist = 0,
    lastSteerTarget = 0,
  }

  local car = ac.getCar(carIndex)
  if car and #state.nodes >= 2 then
    local bestI, bestD = 1, math.huge
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
  writeLog("car #%d personality: %.0f km/h, startSeg=%d", carIndex, kmh, p.segIdx)
end

local function carAhead(carIndex, car)
  local look = car.look
  local lx, lz = look.x, look.z
  local len = math.sqrt(lx * lx + lz * lz)
  if len < 0.001 then return nil end
  lx, lz = lx / len, lz / len

  local sim = ac.getSim()
  if not sim then return nil end

  local bestGap, bestSpeed = nil, nil
  for i = 0, sim.carsCount - 1 do
    if i ~= carIndex then
      local other = ac.getCar(i)
      if other then
        local dx = other.position.x - car.position.x
        local dz = other.position.z - car.position.z
        local forward = dx * lx + dz * lz
        if forward > 0 and forward < 60 then
          local lateral = math.abs(dx * (-lz) + dz * lx)
          if lateral <= FOLLOW_LATERAL and (bestGap == nil or forward < bestGap) then
            bestGap = forward
            bestSpeed = math.max(0, other.speedMs or 0)
          end
        end
      end
    end
  end

  return bestGap, bestSpeed
end

local function liveRays(car)
  local dLeft, dRight, dFwd = 15.0, 15.0, 50.0
  local pos = car.position
  local look = car.look
  local len = math.sqrt(look.x * look.x + look.z * look.z)

  if len > 1e-4 and physics and physics.raycastTrack then
    local fx, fz = look.x / len, look.z / len
    local rx, rz = -fz, fx

    local okL, rL = pcall(physics.raycastTrack, pos + vec3(0, 1.0, 0), vec3(-rx, 0, -rz), 25.0)
    if okL and type(rL) == "number" and rL > 0 then dLeft = rL end

    local okR, rR = pcall(physics.raycastTrack, pos + vec3(0, 1.0, 0), vec3(rx, 0, rz), 25.0)
    if okR and type(rR) == "number" and rR > 0 then dRight = rR end

    local okF, rF = pcall(physics.raycastTrack, pos + vec3(0, 1.0, 0), vec3(fx, 0, fz), 60.0)
    if okF and type(rF) == "number" and rF > 0 then dFwd = rF end
  end

  return dLeft, dRight, dFwd
end

local function followRouteSteer(car, p, speed, nodes)
  local n = #nodes
  if n < 2 then return 0, 0, 0, 0, 0 end

  local segCount = state.loopRoute and n or (n - 1)
  local px, pz = car.position.x, car.position.z

  local function segEnd(sIdx)
    return state.loopRoute and nodes[(sIdx % n) + 1] or nodes[sIdx + 1]
  end

  local bestSeg = clamp(p.segIdx, 1, segCount)
  local bestT = 0
  local bestDistSq = math.huge

  local function checkSeg(sIdx)
    local a = nodes[sIdx]
    local b = segEnd(sIdx)
    local vx = b.x - a.x
    local vz = b.z - a.z
    local lenSq = vx * vx + vz * vz

    local t = 0
    if lenSq > 1e-6 then
      t = clamp(((px - a.x) * vx + (pz - a.z) * vz) / lenSq, 0, 1)
    end

    local cx = a.x + vx * t
    local cz = a.z + vz * t
    local dx = px - cx
    local dz = pz - cz
    local dSq = dx * dx + dz * dz

    if dSq < bestDistSq then
      bestDistSq = dSq
      bestSeg = sIdx
      bestT = t
    end
  end

  -- Ограничиваем поиск небольшим откатом назад и расширенным поиском вперёд.
  -- Это предотвращает перескакивание на уже пройденные участки при заносе.
  local backwardWindow = math.min(2, segCount - 1)
  local forwardWindow = math.min(
    segCount - 1,
    math.max(12, math.floor(segCount * 0.30))
  )

  for offset = -backwardWindow, forwardWindow do
    local idx = ((bestSeg - 1 + offset) % segCount) + 1
    checkSeg(idx)
  end

  if bestDistSq > 25.0 * 25.0 then
    for i = 1, segCount do
      checkSeg(i)
    end
  end

  p.segIdx = bestSeg
  local lineDist = math.sqrt(bestDistSq)

  local segA = nodes[bestSeg]
  local segB = segEnd(bestSeg)

  local startX = segA.x + (segB.x - segA.x) * bestT
  local startZ = segA.z + (segB.z - segA.z) * bestT

  -- Увеличенное упреждение уменьшает частые перекладки руля.
  local lookAhead = clamp(
    LOOKAHEAD_BASE + speed * LOOKAHEAD_SPEED,
    LOOKAHEAD_MIN,
    LOOKAHEAD_MAX
  )

  local curSeg = bestSeg
  local rem = lookAhead
  local targetX, targetZ = segB.x, segB.z
  local steps = 0

  while rem > 0 and steps < segCount do
    steps = steps + 1

    local a = nodes[curSeg]
    local b = segEnd(curSeg)

    local curX = (steps == 1) and startX or a.x
    local curZ = (steps == 1) and startZ or a.z

    local segLen = math.sqrt(
      (b.x - curX) * (b.x - curX) +
      (b.z - curZ) * (b.z - curZ)
    )

    if segLen >= rem and segLen > 1e-4 then
      local ratio = rem / segLen
      targetX = curX + (b.x - curX) * ratio
      targetZ = curZ + (b.z - curZ) * ratio
      rem = 0
    else
      rem = math.max(0, rem - segLen)
      targetX, targetZ = b.x, b.z

      if state.loopRoute then
        curSeg = (curSeg % segCount) + 1
      else
        if curSeg < segCount then
          curSeg = curSeg + 1
        else
          break
        end
      end
    end
  end

  -- Оценка кривизны маршрута для расчёта целевой скорости.
  local prevSeg = ((bestSeg - 2) % segCount) + 1
  local nextSeg = (bestSeg % segCount) + 1

  local pA, pB = nodes[prevSeg], segEnd(prevSeg)
  local nA, nB = nodes[nextSeg], segEnd(nextSeg)

  local v1x, v1z = pB.x - pA.x, pB.z - pA.z
  local v2x, v2z = nB.x - nA.x, nB.z - nA.z

  local l1 = math.sqrt(v1x * v1x + v1z * v1z)
  local l2 = math.sqrt(v2x * v2x + v2z * v2z)

  local curve = 0
  if l1 > 0.1 and l2 > 0.1 then
    curve = math.abs(
      (v1x / l1) * (v2z / l2) -
      (v1z / l1) * (v2x / l2)
    )
  end

  local look = car.look
  local fx, fz = look.x, look.z
  local fLen = math.sqrt(fx * fx + fz * fz)

  if fLen < 1e-4 then
    return 0, curve, lineDist, targetX, targetZ
  end

  fx, fz = fx / fLen, fz / fLen

  local dx = targetX - px
  local dz = targetZ - pz
  local dLen = math.sqrt(dx * dx + dz * dz)

  if dLen < 1e-4 then
    return 0, curve, lineDist, targetX, targetZ
  end

  local dirX, dirZ = dx / dLen, dz / dLen

  local dot = clamp(fx * dirX + fz * dirZ, -1, 1)
  local cross = fx * dirZ - fz * dirX
  local angle = math.atan2(cross, dot)

  -- Меньшее усиление и более узкий диапазон не дают контроллеру
  -- постоянно перекладывать руль из одного крайнего положения в другое.
  local steer = clamp(angle * STEER_GAIN, -STEER_LIMIT, STEER_LIMIT)

  if math.abs(steer) < STEER_DEADBAND then
    steer = 0
  end

  return steer, curve, lineDist, targetX, targetZ
end

local function limitSteerRate(previous, target, dt)
  local maxChange = STEER_RATE * math.max(0, dt)
  local delta = target - previous

  if delta > maxChange then
    delta = maxChange
  elseif delta < -maxChange then
    delta = -maxChange
  end

  return clamp(previous + delta, -STEER_LIMIT, STEER_LIMIT)
end

local function releaseCar(carIndex)
  local ok, c = pcall(ac.overrideCarControls, carIndex)
  if ok and c then
    c.combineAxis = false
    c.steer = 0
    c.gas = 0
    c.brake = 0
    c.handbrake = 0
  end

  state.p[carIndex] = nil
end

local function updateLineCar(carIndex, car, dt)
  local p = state.p[carIndex]

  if not p then
    newPersonality(carIndex)
    p = state.p[carIndex]
  end

  p.t = p.t + dt

  local dLeft, dRight, dFwd = liveRays(car)

  local room = dLeft + dRight
  local nearWall = math.min(dLeft, dRight)
  local offCenter = clamp(
    (dRight - dLeft) / math.max(1, room),
    -1,
    1
  )

  local speed = math.max(0, car.speedMs or 0)

  local steer, curve, lineDist, targetX, targetZ =
    followRouteSteer(car, p, speed, state.nodes)

  p.lastTargetX = targetX
  p.lastTargetZ = targetZ
  p.lastLineDist = lineDist

  -- Коррекция у стены остаётся мягкой и не должна перебивать основное руление.
  if nearWall < 4.0 then
    local wallAvoid = -offCenter * 0.25
    local wBlend = clamp((4.0 - nearWall) / 4.0, 0, 0.25)

    steer = clamp(
      steer * (1 - wBlend) + wallAvoid * wBlend,
      -STEER_LIMIT,
      STEER_LIMIT
    )
  end

  -- Ограничение скорости по кривизне маршрута.
  local targetSpeed = p.speed * clamp(1.0 - curve * 2.0, 0.45, 1.0)

  local wallSpeedCap = math.sqrt(
    math.max(0, 2 * BRAKE_ACCEL * math.max(0, dFwd - WALL_MARGIN))
  )

  if wallSpeedCap > 0 then
    targetSpeed = math.min(targetSpeed, wallSpeedCap)
  end

  local gap, leaderSpeed = carAhead(carIndex, car)

  if gap then
    local safeSpeed = math.sqrt(
      math.max(
        0,
        leaderSpeed * leaderSpeed +
        2 * BRAKE_ACCEL * math.max(0, gap - state.followGap)
      )
    )

    if safeSpeed < targetSpeed then
      targetSpeed = safeSpeed
    end

    if gap < state.followGap then
      targetSpeed = 0
    end
  end

  local err = targetSpeed - speed
  local gas = clamp(err * 0.35, 0, 1)
  local brake = clamp(-err * 0.45, 0, 1)
  local hb = 0

  if speed < 1.5 then
    p.stuck = p.stuck + dt

    if p.stuck > 3.0 then
      p.stuck = 0
      newPersonality(carIndex)
      p = state.p[carIndex]
      writeLog("car #%d: stuck, new personality", carIndex)
    end
  else
    p.stuck = 0
  end

  -- Ограничиваем скорость изменения рулевого угла.
  -- В отличие от прежнего экспоненциального сглаживания, это не позволяет
  -- мгновенно перебрасывать руль в противоположную сторону.
  p.steer = limitSteerRate(p.steer, steer, dt)

  p.gas = gas
  p.brake = brake
  p.hb = hb
  p.lastSteerTarget = steer

  local controls = ac.overrideCarControls(carIndex)

  if controls then
    controls.combineAxis = false
    controls.steer = p.steer
    controls.gas = p.gas
    controls.brake = p.brake
    controls.handbrake = p.hb
  end

  if physics and physics.setAIThrottleLimit then
    pcall(physics.setAIThrottleLimit, carIndex, 1)
  end
end

local function formatNearbyNodes(car, segIdx)
  local n = #state.nodes
  if n == 0 then return "none" end

  local center = clamp(segIdx or 1, 1, n)
  local parts = {}

  for offset = -2, 3 do
    local idx = ((center - 1 + offset) % n) + 1
    local pt = state.nodes[idx]
    local d = flatDist(car.position, pt)

    table.insert(
      parts,
      string.format(
        "#%d(%.1f,%.1f,%.1f d=%.1fm)",
        idx,
        pt.x, pt.y, pt.z,
        d
      )
    )
  end

  return table.concat(parts, " ")
end

local function recordSecondTelemetry(sim)
  local anyWritten = false

  for i = 0, sim.carsCount - 1 do
    local car = ac.getCar(i)
    local p = state.p[i]

    if car and car.isActive and car.isAIControlled and p then
      local pos = car.position
      local look = car.look
      local vel = car.velocity
      local spdMs = car.speedMs or 0
      local spdKmh = spdMs * 3.6
      local nearby = formatNearbyNodes(car, p.segIdx)

      writeLog(
        "car=%d pos=(%.2f,%.2f,%.2f) spd=%.1fkm/h(%.1fm/s) | ctrl: steer=%.3f target=%.3f gas=%.2f brk=%.2f hb=%.2f | path: seg=%d err=%.2fm tgt=(%.1f,%.1f) look=(%.2f,%.2f) vel=(%.2f,%.2f) | nodes: %s",
        i,
        pos.x, pos.y, pos.z,
        spdKmh, spdMs,
        p.steer, p.lastSteerTarget or 0,
        p.gas, p.brake, p.hb,
        p.segIdx, p.lastLineDist,
        p.lastTargetX, p.lastTargetZ,
        look and look.x or 0,
        look and look.z or 0,
        vel and vel.x or 0,
        vel and vel.z or 0,
        nearby
      )

      anyWritten = true
    end
  end

  if not anyWritten and logger and logger.heartbeat then
    logger:heartbeat("nodes=" .. tostring(#state.nodes))
  end

  flushLog()
end

function script.update(dt)
  if not dt or dt <= 0 then return end

  local player = ac.getCar(0)

  if state.recording and player then
    addPoint(player.position)
  end

  local sim = ac.getSim()
  if not sim then return end

  if not state.enabled or #state.nodes < 2 then
    for idx in pairs(state.p) do
      releaseCar(idx)
    end
  else
    for i = 0, sim.carsCount - 1 do
      local car = ac.getCar(i)

      if car and car.isActive and car.isAIControlled then
        updateLineCar(i, car, dt)
      elseif state.p[i] then
        releaseCar(i)
      end
    end
  end

  state.logAccum = state.logAccum + dt

  if state.logAccum >= 1.0 then
    state.logAccum = state.logAccum - 1.0
    recordSecondTelemetry(sim)
  end
end

function windowMain(dt)
  local player = ac.getCar(0)

  ui.text(string.format("Точек: %d | %s", #state.nodes, state.status))

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
  end

  local canvas = vec2(300, 125)

  ui.invisibleButton("line_radar", canvas)

  local p0 = ui.getItemRectMin()
  local p1 = ui.getItemRectMax()

  ui.drawRectFilled(
    p0,
    p1,
    rgbm(0.1, 0.1, 0.12, 1),
    4.0
  )

  ui.drawRect(
    p0,
    p1,
    rgbm(0.3, 0.3, 0.35, 1),
    4.0
  )

  if #state.nodes >= 2 then
    local cx = player and player.position.x or state.nodes[1].x
    local cz = player and player.position.z or state.nodes[1].z

    local scale = 1.2
    local mx = (p0.x + p1.x) * 0.5
    local my = (p0.y + p1.y) * 0.5

    local segs = state.loopRoute and #state.nodes or (#state.nodes - 1)

    for i = 1, segs do
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
          2.0
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
end

if ac.onRelease then
  ac.onRelease(function()
    for idx in pairs(state.p) do
      releaseCar(idx)
    end

    if logger and logger.shutdown then
      logger:shutdown("RoutePilot released")
    end
  end)
end

loadLine()