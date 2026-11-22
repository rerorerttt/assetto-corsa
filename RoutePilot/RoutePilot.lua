-- RoutePilot: route recording and AI control for Assetto Corsa CSP.
-- No stock AI fast lane is used. AI cars follow a route recorded by the player.
-- Controls written: steering, throttle and brake only. Gearbox remains automatic.

local APP_VERSION = "0.1.0"
local STORAGE_PREFIX = "routepilot_route_"
local SAMPLE_DISTANCE = 2.0
local MAX_ROUTE_POINTS = 12000
local LOOKAHEAD_MIN = 7.0
local LOOKAHEAD_MAX = 24.0
local LOOKAHEAD_TIME = 0.55
local LATERAL_ACCEL = 5.0
local BRAKE_ACCEL = 7.0
local WHEELBASE = 2.65
local MAX_ROAD_WHEEL_ANGLE = 0.55

local state = {
  recording = false,
  autopilot = false,
  route = {},
  closed = true,
  speedMultiplier = 1.0,
  status = "Маршрут не загружен",
  lastRecordPosition = nil,
  recordTimer = 0,
  ai = {},
  updateTimer = 0,
  routeKey = nil,
  trackID = "",
  controlledCount = 0,
  lastError = ""
}

local function clamp(v, lo, hi)
  if v < lo then return lo end
  if v > hi then return hi end
  return v
end

local function distance2(a, b)
  local dx = a.x - b.x
  local dz = a.z - b.z
  return dx * dx + dz * dz
end

local function distance(a, b)
  return math.sqrt(distance2(a, b))
end

local function safeTrackID(value)
  value = string.lower(value or "unknown_track")
  value = string.gsub(value, "[^%w_%-]", "_")
  return value
end

local function getTrackKey()
  local id = safeTrackID(ac.getTrackFullID("_"))
  state.trackID = id
  state.routeKey = STORAGE_PREFIX .. id
  return state.routeKey
end

local function routeLength()
  if #state.route < 2 then return 0 end
  local total = 0
  for i = 1, #state.route - 1 do
    total = total + state.route[i].length
  end
  if state.closed then
    total = total + state.route[#state.route].length
  end
  return total
end

local function nextIndex(i)
  if i < #state.route then return i + 1 end
  if state.closed then return 1 end
  return #state.route
end

local function previousIndex(i)
  if i > 1 then return i - 1 end
  if state.closed then return #state.route end
  return 1
end

local function rebuildRoute()
  local n = #state.route
  if n < 3 then
    for i = 1, n do
      state.route[i].length = 0
      state.route[i].curveSpeed = 12
    end
    return
  end

  for i = 1, n do
    local p = state.route[i]
    local ni = nextIndex(i)
    local pi = previousIndex(i)
    local nextPoint = state.route[ni]
    local prevPoint = state.route[pi]
    local segLength = distance(p, nextPoint)
    p.length = segLength

    local ax = p.x - prevPoint.x
    local az = p.z - prevPoint.z
    local bx = nextPoint.x - p.x
    local bz = nextPoint.z - p.z
    local la = math.sqrt(ax * ax + az * az)
    local lb = math.sqrt(bx * bx + bz * bz)
    local curvature = 0

    if la > 0.05 and lb > 0.05 then
      local cross = ax * bz - az * bx
      local dot = ax * bx + az * bz
      local angle = math.abs(math.atan2(cross, dot))
      curvature = angle / math.max(0.1, (la + lb) * 0.5)
    end

    local curveSpeed = 60
    if curvature > 0.0005 then
      curveSpeed = math.sqrt(LATERAL_ACCEL / curvature)
    end
    p.curveSpeed = clamp(curveSpeed, 5.0, 60.0)
    p.recordedSpeed = clamp(tonumber(p.recordedSpeed) or 12, 3.0, 65.0)
  end
end

local function serializeRoute()
  local out = {}
  out[#out + 1] = "ROUTEPILOT|1|" .. (state.closed and "1" or "0")
  for i = 1, #state.route do
    local p = state.route[i]
    out[#out + 1] = string.format("%.3f,%.3f,%.3f", p.x, p.z, p.recordedSpeed)
  end
  return table.concat(out, "\n")
end

local function parseRoute(data)
  if type(data) ~= "string" or data == "" then
    return false, "Сохранённый маршрут не найден"
  end

  local parsed = {}
  local first = true
  local closed = true

  for line in string.gmatch(data, "[^\r\n]+") do
    if first then
      first = false
      local flag = string.match(line, "^ROUTEPILOT|1|([01])$")
      if not flag then return false, "Неподдерживаемый формат маршрута" end
      closed = flag == "1"
    else
      local sx, sz, sv = string.match(line, "^([^,]+),([^,]+),([^,]+)$")
      local x, z, speed = tonumber(sx), tonumber(sz), tonumber(sv)
      if x and z and speed then
        parsed[#parsed + 1] = { x = x, y = 0, z = z, recordedSpeed = speed }
        if #parsed > MAX_ROUTE_POINTS then
          return false, "В маршруте слишком много точек"
        end
      end
    end
  end

  if #parsed < 3 then return false, "В маршруте меньше трёх точек" end
  state.route = parsed
  state.closed = closed
  rebuildRoute()
  state.ai = {}
  return true, "Загружено точек: " .. tostring(#state.route)
end

local function saveRoute()
  if #state.route < 3 then
    state.status = "Сначала запиши маршрут (минимум 3 точки)"
    return
  end
  rebuildRoute()
  local ok, err = pcall(function()
    ac.storage[getTrackKey()] = serializeRoute()
  end)
  if ok then
    state.status = "Маршрут сохранён: " .. tostring(#state.route) .. " точек"
  else
    state.status = "Ошибка сохранения: " .. tostring(err)
    ac.error(state.status)
  end
end

local function loadRoute()
  local ok, data = pcall(function()
    return ac.storage[getTrackKey()]
  end)
  if not ok then
    state.status = "Ошибка чтения хранилища: " .. tostring(data)
    return
  end
  local success, message = parseRoute(data)
  state.status = message
  if success then
    state.autopilot = false
  end
end

local function addRecordPoint(car)
  local pos = car.position
  local speed = math.max(3.0, car.speedMs or 0)
  state.route[#state.route + 1] = {
    x = pos.x,
    y = pos.y,
    z = pos.z,
    recordedSpeed = speed
  }
  state.lastRecordPosition = { x = pos.x, z = pos.z }
  if #state.route >= MAX_ROUTE_POINTS then
    state.recording = false
    state.status = "Достигнут лимит точек записи"
  end
end

local function startRecording()
  state.autopilot = false
  state.route = {}
  state.ai = {}
  state.lastRecordPosition = nil
  state.recordTimer = 0
  state.recording = true
  state.status = "Запись идёт — проедь нужную траекторию"
  local car = ac.getCar(0)
  if car then addRecordPoint(car) end
end

local function stopRecording()
  state.recording = false
  state.lastRecordPosition = nil
  rebuildRoute()
  state.status = "Запись остановлена: " .. tostring(#state.route) .. " точек"
end

local function findNearestRoutePoint(pos)
  local bestIndex = 1
  local bestDistance = math.huge
  for i = 1, #state.route do
    local d = distance2(pos, state.route[i])
    if d < bestDistance then
      bestDistance = d
      bestIndex = i
    end
  end
  return bestIndex, math.sqrt(bestDistance)
end

local function findNearestLocal(pos, currentIndex)
  local n = #state.route
  local bestIndex = currentIndex
  local bestDistance = math.huge
  for offset = -3, 14 do
    local idx = currentIndex + offset
    if state.closed then
      idx = ((idx - 1) % n) + 1
    elseif idx < 1 then
      idx = 1
    elseif idx > n then
      idx = n
    end
    local d = distance2(pos, state.route[idx])
    if d < bestDistance then
      bestDistance = d
      bestIndex = idx
    end
  end
  return bestIndex, math.sqrt(bestDistance)
end

local function pointAtLookahead(index, lookahead)
  local n = #state.route
  local travelled = 0
  local current = index
  for _ = 1, n do
    local p = state.route[current]
    local nextI = nextIndex(current)
    local len = p.length or 0
    if len > 0.001 then
      if travelled + len >= lookahead then
        local t = clamp((lookahead - travelled) / len, 0, 1)
        local q = state.route[nextI]
        return {
          x = p.x + (q.x - p.x) * t,
          z = p.z + (q.z - p.z) * t
        }, current
      end
      travelled = travelled + len
    end
    if nextI == current then break end
    current = nextI
    if not state.closed and current == n then
      return state.route[n], n
    end
  end
  return state.route[current], current
end

local function speedTargetAt(index, speedMultiplier)
  local n = #state.route
  local base = state.route[index].recordedSpeed * speedMultiplier
  local allowed = base
  local travelled = 0
  local current = index

  for _ = 1, math.min(n, 90) do
    local p = state.route[current]
    local curve = p.curveSpeed * speedMultiplier
    local brakingLimit = math.sqrt(curve * curve + 2 * BRAKE_ACCEL * travelled)
    if brakingLimit < allowed then allowed = brakingLimit end
    travelled = travelled + (p.length or 0)
    if travelled > 75 then break end
    local ni = nextIndex(current)
    if ni == current then break end
    current = ni
    if not state.closed and current == n then
      allowed = math.min(allowed, 0)
      break
    end
  end

  if not state.closed then
    local remaining = 0
    for i = index, n - 1 do
      remaining = remaining + (state.route[i].length or 0)
    end
    if remaining < 18 then
      allowed = math.min(allowed, math.sqrt(math.max(0, 2 * BRAKE_ACCEL * remaining)))
    end
  end

  return clamp(allowed, 0, 65)
end

local function updateOneAI(carIndex, car, dt)
  local runtime = state.ai[carIndex]
  if not runtime then
    local nearest = findNearestRoutePoint(car.position)
    runtime = { index = nearest, globalSearch = 0, steer = 0 }
    state.ai[carIndex] = runtime
  end

  runtime.globalSearch = runtime.globalSearch + dt
  local nearestIndex, distanceFromRoute
  if runtime.globalSearch >= 1.5 or distance2(car.position, state.route[runtime.index]) > 225 then
    nearestIndex, distanceFromRoute = findNearestRoutePoint(car.position)
    runtime.globalSearch = 0
  else
    nearestIndex, distanceFromRoute = findNearestLocal(car.position, runtime.index)
  end
  runtime.index = nearestIndex

  if distanceFromRoute > 35 then
    local controls = ac.overrideCarControls(carIndex)
    if controls then
      controls.combineAxis = false
      controls.steer = 0
      controls.gas = 0
      controls.brake = 0.6
    end
    return
  end

  local speed = math.max(0, car.speedMs or 0)
  local lookahead = clamp(LOOKAHEAD_MIN + speed * LOOKAHEAD_TIME, LOOKAHEAD_MIN, LOOKAHEAD_MAX)
  local target = pointAtLookahead(runtime.index, lookahead)
  local dx = target.x - car.position.x
  local dz = target.z - car.position.z
  local look = car.look
  local forwardDot = look.x * dx + look.z * dz
  local cross = look.z * dx - look.x * dz
  local alpha = math.atan2(cross, forwardDot)
  local steeringAngle = math.atan2(2 * WHEELBASE * math.sin(alpha), math.max(1, math.sqrt(dx * dx + dz * dz)))
  local steer = clamp(steeringAngle / MAX_ROAD_WHEEL_ANGLE, -1, 1)

  local targetSpeed = speedTargetAt(runtime.index, state.speedMultiplier)
  if not state.closed and runtime.index == #state.route then
    targetSpeed = 0
  end
  local speedError = targetSpeed - speed
  local gas = clamp(speedError * 0.32, 0, 1)
  local brake = clamp(-speedError * 0.48, 0, 1)

  if distanceFromRoute > 12 then
    gas = math.min(gas, 0.25)
    brake = math.max(brake, 0.12)
  end

  runtime.steer = runtime.steer + (steer - runtime.steer) * clamp(dt * 7, 0, 1)

  local controls = ac.overrideCarControls(carIndex)
  if controls then
    controls.combineAxis = false
    controls.steer = runtime.steer
    controls.gas = gas
    controls.brake = brake
  end
end

local function updateAutopilot(dt)
  local sim = ac.getSim()
  if not sim or not sim.isLive or #state.route < 3 then
    state.controlledCount = 0
    return
  end

  local active = {}
  local count = 0
  for carIndex = 1, sim.carsCount - 1 do
    local car = ac.getCar(carIndex)
    if car and car.isActive and car.isAIControlled then
      active[carIndex] = true
      updateOneAI(carIndex, car, dt)
      count = count + 1
    end
  end

  for carIndex in pairs(state.ai) do
    if not active[carIndex] then state.ai[carIndex] = nil end
  end
  state.controlledCount = count
end

function script.update(dt)
  local sim = ac.getSim()
  if not sim or not sim.isLive then return end

  if state.recording then
    local car = ac.getCar(0)
    if car then
      state.recordTimer = state.recordTimer + dt
      local pos = car.position
      if not state.lastRecordPosition or distance(pos, state.lastRecordPosition) >= SAMPLE_DISTANCE then
        addRecordPoint(car)
      end
    end
  end

  if state.autopilot then
    updateAutopilot(dt)
  else
    state.controlledCount = 0
  end
end

function script.windowMain(dt)
  ui.text("RoutePilot  " .. APP_VERSION)
  ui.text("Трасса: " .. (state.trackID ~= "" and state.trackID or safeTrackID(ac.getTrackFullID("_"))))
  ui.text("Точек маршрута: " .. tostring(#state.route) .. "   AI под управлением: " .. tostring(state.controlledCount))
  ui.separator()

  if state.recording then
    if ui.button("Остановить запись", vec2(ui.availableSpaceX(), 32)) then
      stopRecording()
    end
  else
    if ui.button("Начать запись маршрута", vec2(ui.availableSpaceX(), 32)) then
      startRecording()
    end
  end

  if ui.checkbox("Замкнутый маршрут (круг)", state.closed) then
    state.closed = not state.closed
    rebuildRoute()
  end

  if ui.button("Сохранить маршрут этой трассы", vec2(ui.availableSpaceX(), 28)) then
    saveRoute()
  end
  if ui.button("Загрузить маршрут этой трассы", vec2(ui.availableSpaceX(), 28)) then
    loadRoute()
  end

  ui.separator()
  local multiplier, changed = ui.slider("Скорость AI", state.speedMultiplier, 0.5, 1.8, "%.2fx")
  if changed then state.speedMultiplier = multiplier end

  if state.autopilot then
    if ui.button("Остановить автопилот", vec2(ui.availableSpaceX(), 34)) then
      state.autopilot = false
      state.ai = {}
      state.controlledCount = 0
      state.status = "Автопилот выключен"
    end
  else
    if ui.button("Запустить автопилот", vec2(ui.availableSpaceX(), 34)) then
      if #state.route >= 3 then
        rebuildRoute()
        state.ai = {}
        state.autopilot = true
        state.status = "Автопилот активен"
      else
        state.status = "Сначала запиши или загрузи маршрут"
      end
    end
  end

  ui.separator()
  ui.textWrapped("Статус: " .. state.status)
  if state.recording then
    ui.text("Записано точек: " .. tostring(#state.route))
  end
  ui.text("Коробка передач остаётся под управлением автомобиля.")
end

function script.onSessionStart()
  getTrackKey()
  loadRoute()
end
