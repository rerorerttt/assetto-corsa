-- RoutePilot: route recording and AI control for Assetto Corsa CSP.
-- v0.2.3: text log in the app folder (routepilot_log.txt) for remote diagnosis:
--         script load, init, every frame errors (protected), autopilot decisions,
--         controls-override status, self-test verdicts, per-car state snapshots.
--         If script.update throws, CSP normally kills the app silently for the
--         rest of the session — now the error is caught, logged and the app lives.

local Logger = require('logger')
local log = Logger.new('routepilot_log.txt', 1200)
log:log('=== RoutePilot log start ===')

local APP_VERSION = "0.2.3"
log:log('script loaded, version %s', APP_VERSION)

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
local FOLLOW_LATERAL = 2.4
local SELFTEST_SECONDS = 4.0
local SNAPSHOT_INTERVAL = 2.0 -- seconds between per-car state snapshots in the log

local state = {
  initialized = false,
  recording = false,
  autopilot = false,
  route = {},
  closed = true,
  speedMultiplier = 1.0,
  followGap = 6.0,
  controlAllCars = false,
  status = "Маршрут не загружен",
  lastRecordPosition = nil,
  recordTimer = 0,
  ai = {},
  routeKey = nil,
  trackID = "",
  controlledCount = 0,
  activeCount = 0,
  slowingCount = 0,
  updateErrors = 0,
  lastError = "",
  snapshotTimer = 0,
  selfTest = { active = false, carIndex = nil, timer = 0,
    steerSeen = 0, brakeSeen = 0, appliedFrames = 0, verdict = nil },
}

-- ─────────────────── small helpers ───────────────────

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
  return string.gsub(value, "[^%w_%-]", "_")
end

local function getTrackKey()
  local id = safeTrackID(ac.getTrackFullID("_"))
  state.trackID = id
  state.routeKey = STORAGE_PREFIX .. id
  return state.routeKey
end

-- safe read of a car field (userdata throws on unknown fields)
local function cget(car, key, default)
  local ok, v = pcall(function() return car[key] end)
  if ok and type(v) == 'number' then return v end
  return default
end

-- ─────────────────── route model ───────────────────

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

local function routeLengthSafe()
  local n = #state.route
  if n < 2 then return 0 end
  local total = 0
  for i = 1, n - 1 do total = total + (state.route[i].length or 0) end
  if state.closed then total = total + (state.route[n].length or 0) end
  return total
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
    local ni, pi = nextIndex(i), previousIndex(i)
    local np, pp = state.route[ni], state.route[pi]
    p.length = distance(p, np)
    local ax, az = p.x - pp.x, p.z - pp.z
    local bx, bz = np.x - p.x, np.z - p.z
    local la, lb = math.sqrt(ax*ax+az*az), math.sqrt(bx*bx+bz*bz)
    local curvature = 0
    if la > 0.05 and lb > 0.05 then
      local angle = math.abs(math.atan2(ax*bz-az*bx, ax*bx+az*bz))
      curvature = angle / math.max(0.1, (la+lb)*0.5)
    end
    local curveSpeed = 60
    if curvature > 0.0005 then curveSpeed = math.sqrt(LATERAL_ACCEL / curvature) end
    p.curveSpeed = clamp(curveSpeed, 5.0, 60.0)
    p.recordedSpeed = clamp(tonumber(p.recordedSpeed) or 12, 3.0, 65.0)
  end
end

local function serializeRoute()
  local out = {}
  out[#out+1] = "ROUTEPILOT|1|" .. (state.closed and "1" or "0")
  for i = 1, #state.route do
    local p = state.route[i]
    out[#out+1] = string.format("%.3f,%.3f,%.3f", p.x, p.z, p.recordedSpeed)
  end
  return table.concat(out, "\n")
end

local function parseRoute(data)
  if type(data) ~= "string" or data == "" then
    return false, "Сохранённый маршрут не найден"
  end
  local parsed, first, closed = {}, true, true
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
        parsed[#parsed+1] = { x = x, y = 0, z = z, recordedSpeed = speed }
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
  return true, "Загружено точек: " .. tostring(#parsed)
end

local function saveRoute()
  if #state.route < 3 then
    state.status = "Сначала запиши маршрут (минимум 3 точки)"
    return
  end
  rebuildRoute()
  local ok, err = pcall(function() ac.storage[getTrackKey()] = serializeRoute() end)
  if ok then
    state.status = "Маршрут сохранён: " .. tostring(#state.route) .. " точек"
  else
    state.status = "Ошибка сохранения: " .. tostring(err)
    ac.error(state.status)
  end
  log:log('saveRoute: %s', state.status)
  log:flush()
end

local function loadRoute()
  local ok, data = pcall(function() return ac.storage[getTrackKey()] end)
  if not ok then
    state.status = "Ошибка чтения хранилища: " .. tostring(data)
    log:log('loadRoute: storage read failed: %s', tostring(data))
    return
  end
  local success, message = parseRoute(data)
  state.status = message
  log:log('loadRoute: %s', tostring(message))
  if success then state.autopilot = false end
end

-- ─────────────────── recording ───────────────────

local function addRecordPoint(car)
  local pos = car.position
  local speed = math.max(3.0, car.speedMs or 0)
  state.route[#state.route+1] = { x = pos.x, y = pos.y, z = pos.z, recordedSpeed = speed }
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
  state.recording = true
  state.status = "Запись идёт — проедь нужную траекторию"
  log:log('startRecording')
  local car = ac.getCar(0)
  if car then addRecordPoint(car) end
end

local function stopRecording()
  state.recording = false
  state.lastRecordPosition = nil
  rebuildRoute()
  state.status = "Запись остановлена: " .. tostring(#state.route) .. " точек"
  log:log('stopRecording: %d points, length %.0f m', #state.route, routeLengthSafe())
  log:flush()
end

-- ─────────────────── navigation ───────────────────

local function findNearestRoutePoint(pos)
  local bestIndex, bestDistance = 1, math.huge
  for i = 1, #state.route do
    local d = distance2(pos, state.route[i])
    if d < bestDistance then bestDistance, bestIndex = d, i end
  end
  return bestIndex, math.sqrt(bestDistance)
end

local function findNearestLocal(pos, currentIndex)
  local n = #state.route
  local bestIndex, bestDistance = currentIndex, math.huge
  for offset = -3, 14 do
    local idx = currentIndex + offset
    if state.closed then
      idx = ((idx - 1) % n) + 1
    elseif idx < 1 then idx = 1
    elseif idx > n then idx = n end
    local d = distance2(pos, state.route[idx])
    if d < bestDistance then bestDistance, bestIndex = d, idx end
  end
  return bestIndex, math.sqrt(bestDistance)
end

local function pointAtLookahead(index, lookahead)
  local n = #state.route
  local travelled, current = 0, index
  for _ = 1, n do
    local p = state.route[current]
    local nextI = nextIndex(current)
    local len = p.length or 0
    if len > 0.001 then
      if travelled + len >= lookahead then
        local t = clamp((lookahead - travelled) / len, 0, 1)
        local q = state.route[nextI]
        return { x = p.x + (q.x - p.x) * t, z = p.z + (q.z - p.z) * t }, current
      end
      travelled = travelled + len
    end
    if nextI == current then break end
    current = nextI
    if not state.closed and current == n then return state.route[n], n end
  end
  return state.route[current], current
end

local function speedTargetAt(index, speedMultiplier)
  local n = #state.route
  local allowed = state.route[index].recordedSpeed * speedMultiplier
  local travelled, current = 0, index
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
    if not state.closed and current == n then allowed = math.min(allowed, 0); break end
  end
  if not state.closed then
    local remaining = 0
    for i = index, n - 1 do remaining = remaining + (state.route[i].length or 0) end
    if remaining < 18 then
      allowed = math.min(allowed, math.sqrt(math.max(0, 2 * BRAKE_ACCEL * remaining)))
    end
  end
  return clamp(allowed, 0, 65)
end

local function carAhead(carIndex, car)
  local look = car.look
  local lx, lz = look.x, look.z
  local lookLen = math.sqrt(lx*lx + lz*lz)
  if lookLen < 0.001 then return nil end
  lx, lz = lx / lookLen, lz / lookLen
  local sim = ac.getSim()
  if not sim then return nil end
  local bestGap, bestSpeed = nil, nil
  for otherIndex = 0, sim.carsCount - 1 do
    if otherIndex ~= carIndex then
      local other = ac.getCar(otherIndex)
      if other then
        local dx = other.position.x - car.position.x
        local dz = other.position.z - car.position.z
        local forward = dx * lx + dz * lz
        if forward > 0 then
          local lateral = dx * (-lz) + dz * lx
          if math.abs(lateral) <= FOLLOW_LATERAL and (bestGap == nil or forward < bestGap) then
            bestGap = forward
            bestSpeed = math.max(0, other.speedMs or 0)
          end
        end
      end
    end
  end
  if bestGap == nil then return nil end
  return bestGap, bestSpeed
end

-- ─────────────────── AI control ───────────────────

local function applyControls(carIndex, steer, gas, brake)
  local ok, controls = pcall(ac.overrideCarControls, carIndex)
  if not ok then
    log:log('overrideCarControls(%d) THREW: %s', carIndex, tostring(controls))
    return false
  end
  if not controls then
    log:log('overrideCarControls(%d) returned nil', carIndex)
    return false
  end
  local okSet, errSet = pcall(function()
    controls.combineAxis = false
    controls.steer = steer
    controls.gas = gas
    controls.brake = brake
  end)
  if not okSet then
    log:log('controls(%d) set failed: %s', carIndex, tostring(errSet))
    return false
  end
  return true
end

local function updateOneAI(carIndex, car, dt)
  local runtime = state.ai[carIndex]
  if not runtime then
    local nearest, dist = findNearestRoutePoint(car.position)
    runtime = { index = nearest, globalSearch = 0, steer = 0, slowing = false }
    state.ai[carIndex] = runtime
    log:log('car #%d adopted, nearest point %d, dist %.1f m', carIndex, nearest, dist)
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
    applyControls(carIndex, 0, 0, 0.6)
    runtime.lastCommand = { steer = 0, gas = 0, brake = 0.6, targetSpeed = 0 }
    return
  end

  local speed = math.max(0, car.speedMs or 0)
  local lookahead = clamp(LOOKAHEAD_MIN + speed * LOOKAHEAD_TIME, LOOKAHEAD_MIN, LOOKAHEAD_MAX)
  local target = pointAtLookahead(runtime.index, lookahead)
  local dx = target.x - car.position.x
  local dz = target.z - car.position.z
  local look = car.look
  local alpha = math.atan2(look.z * dx - look.x * dz, look.x * dx + look.z * dz)
  local steeringAngle = math.atan2(2 * WHEELBASE * math.sin(alpha), math.max(1, math.sqrt(dx*dx + dz*dz)))
  local steer = clamp(steeringAngle / MAX_ROAD_WHEEL_ANGLE, -1, 1)

  local targetSpeed = speedTargetAt(runtime.index, state.speedMultiplier)
  if not state.closed and runtime.index == #state.route then targetSpeed = 0 end

  local isSlowing = false
  local gap, leaderSpeed = carAhead(carIndex, car)
  if gap then
    local safeSpeed = math.sqrt(math.max(0, leaderSpeed * leaderSpeed
      + 2 * BRAKE_ACCEL * math.max(0, gap - state.followGap)))
    if safeSpeed < targetSpeed then targetSpeed = safeSpeed; isSlowing = true end
    if gap < state.followGap then targetSpeed = 0; isSlowing = true end
  end
  runtime.slowing = isSlowing

  local speedError = targetSpeed - speed
  local gas = clamp(speedError * 0.32, 0, 1)
  local brake = clamp(-speedError * 0.48, 0, 1)
  if distanceFromRoute > 12 then
    gas = math.min(gas, 0.25)
    brake = math.max(brake, 0.12)
  end
  runtime.steer = runtime.steer + (steer - runtime.steer) * clamp(dt * 7, 0, 1)
  applyControls(carIndex, runtime.steer, gas, brake)
  runtime.lastCommand = { steer = runtime.steer, gas = gas, brake = brake, targetSpeed = targetSpeed }
end

local function updateAutopilot(dt)
  local sim = ac.getSim()
  if not sim or not sim.isLive or #state.route < 3 then
    state.controlledCount = 0
    state.slowingCount = 0
    state.activeCount = 0
    return
  end
  local active, count, slowing = {}, 0, 0
  for carIndex = 1, sim.carsCount - 1 do
    local car = ac.getCar(carIndex)
    local eligible = car and car.isActive and (car.isAIControlled or state.controlAllCars)
    if eligible then
      active[carIndex] = true
      updateOneAI(carIndex, car, dt)
      count = count + 1
      if state.ai[carIndex] and state.ai[carIndex].slowing then slowing = slowing + 1 end
    end
  end
  for carIndex in pairs(state.ai) do
    if not active[carIndex] then state.ai[carIndex] = nil end
  end
  state.controlledCount = count
  state.slowingCount = slowing

  -- how many overrides the physics thread actually consumes
  state.activeCount = 0
  for carIndex in pairs(active) do
    local ok, controls = pcall(ac.overrideCarControls, carIndex)
    if ok and controls and controls:active() then
      state.activeCount = state.activeCount + 1
    end
  end
end

-- per-car state snapshot for the log: what we command vs what the car reports
local function logCarSnapshot()
  local sim = ac.getSim()
  if not sim then return end
  for carIndex = 1, sim.carsCount - 1 do
    local car = ac.getCar(carIndex)
    local runtime = state.ai[carIndex]
    if car and car.isActive and runtime and runtime.lastCommand then
      local brake = cget(car, 'brake', -1)
      local gas = cget(car, 'gas', -1)
      local steer = cget(car, 'steer', 100)
      log:log('car#%d state: cmd[steer %.2f gas %.2f brake %.2f v %.0f] real[steer %.2f gas %.2f brake %.2f] kmh %.1f',
        carIndex,
        runtime.lastCommand.steer, runtime.lastCommand.gas, runtime.lastCommand.brake,
        runtime.lastCommand.targetSpeed,
        steer, gas, brake, cget(car, 'speedKmh', -1))
    end
  end
end

-- ─────────────────── self-test ───────────────────

local function startSelfTest()
  local sim = ac.getSim()
  if not sim then
    state.selfTest.verdict = "Сессия не активна"
    log:log('selfTest: no session')
    return
  end
  local carIndex = nil
  for i = 1, sim.carsCount - 1 do
    local car = ac.getCar(i)
    if car and car.isActive then carIndex = i; break end
  end
  if not carIndex then
    state.selfTest.verdict = "Нет машин с индексом >= 1"
    log:log('selfTest: no cars with index >= 1, carsCount=%d', sim.carsCount)
    log:flush()
    return
  end
  state.selfTest = { active = true, carIndex = carIndex, timer = SELFTEST_SECONDS,
    steerSeen = 0, brakeSeen = 0, appliedFrames = 0, verdict = nil }
  state.status = "Тест перехвата: машина #" .. carIndex
  log:log('selfTest: START on car #%d (carsCount=%d)', carIndex, sim.carsCount)
  log:flush()
end

local function updateSelfTest(dt)
  local st = state.selfTest
  if not st.active then return end
  st.timer = st.timer - dt
  local car = st.carIndex and ac.getCar(st.carIndex) or nil
  if not car then
    st.active = false
    st.verdict = "Машина #" .. tostring(st.carIndex) .. " исчезла"
    log:log('selfTest: car disappeared')
    return
  end
  local ok, controls = pcall(ac.overrideCarControls, st.carIndex)
  if not ok or not controls then
    st.active = false
    st.verdict = "overrideCarControls вернул nil — перехват недоступен"
    log:log('selfTest: overrideCarControls FAILED: ok=%s controls=%s', tostring(ok), tostring(controls))
    log:flush()
    return
  end
  local okSet, errSet = pcall(function()
    controls.combineAxis = false
    controls.gas = 0
    controls.brake = 1
    controls.steer = 0.8
  end)
  if not okSet then
    st.active = false
    st.verdict = "Не удалось записать команды в структуру перехвата"
    log:log('selfTest: controls set failed: %s', tostring(errSet))
    log:flush()
    return
  end
  st.appliedFrames = st.appliedFrames + 1
  local brake = cget(car, 'brake', -1)
  local steer = cget(car, 'steer', 100)
  if brake > 0.5 then st.brakeSeen = st.brakeSeen + 1 end
  if math.abs(steer) > 0.3 then st.steerSeen = st.steerSeen + 1 end

  if st.timer <= 0 then
    st.active = false
    local accepted = st.brakeSeen >= 10 or st.steerSeen >= 10
    if accepted then
      st.verdict = "ПЕРЕХВАТ РАБОТАЕТ: физика применила команды ("
        .. st.brakeSeen .. "/" .. st.appliedFrames .. ")"
    else
      st.verdict = "ПЕРЕХВАТ НЕ РАБОТАЕТ: физика игнорирует команды (тормоз виден "
        .. st.brakeSeen .. " кадров из " .. st.appliedFrames .. ")"
    end
    log:log('selfTest: END verdict=%s appliedFrames=%d brakeSeen=%d steerSeen=%d',
      tostring(st.verdict), st.appliedFrames, st.brakeSeen, st.steerSeen)
    local okA, cA = pcall(ac.overrideCarControls, st.carIndex)
    if okA and cA then
      log:log('selfTest: controls:active()=%s', tostring(cA:active()))
    end
    local ok2, controls2 = pcall(ac.overrideCarControls, st.carIndex)
    if ok2 and controls2 then
      controls2.brake = 0
      controls2.steer = 0
    end
    state.status = "Тест перехвата завершён"
    log:flush()
  end
end

-- ─────────────────── main loop ───────────────────

local function updateInner(dt)
  if not state.initialized then
    state.initialized = true
    local ok, err = pcall(function() getTrackKey(); loadRoute() end)
    if not ok then
      state.status = "Ошибка инициализации: " .. tostring(err)
      log:log('init FAILED: %s', tostring(err))
    end
    local sim = ac.getSim()
    log:log('init: track=%s cars=%d isLive=%s',
      tostring(state.trackID),
      sim and sim.carsCount or -1,
      sim and tostring(sim.isLive) or 'nil')
    log:flush()
  end

  local sim = ac.getSim()
  if not sim or not sim.isLive then return end

  updateSelfTest(dt)

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
    state.slowingCount = 0
    state.activeCount = 0
  end

  -- periodic snapshot + log flush
  state.snapshotTimer = state.snapshotTimer + dt
  if state.snapshotTimer >= SNAPSHOT_INTERVAL then
    state.snapshotTimer = 0
    if state.autopilot then logCarSnapshot() end
    log:flush()
  end
end

function script.update(dt)
  -- protect the whole frame: an exception would otherwise silently kill the app
  local ok, err = pcall(updateInner, dt)
  if not ok then
    state.updateErrors = state.updateErrors + 1
    state.lastError = tostring(err)
    log:log('update ERROR #%d: %s', state.updateErrors, state.lastError)
    log:flush()
  end
end

function script.windowMain(dt)
  ui.text("RoutePilot  " .. APP_VERSION)
  ui.text("Трасса: " .. (state.trackID ~= "" and state.trackID or safeTrackID(ac.getTrackFullID("_"))))
  ui.text("Точек маршрута: " .. tostring(#state.route) .. "   AI под управлением: " .. tostring(state.controlledCount))
  ui.text("Притормаживают: " .. tostring(state.slowingCount)
    .. "   Перехват активен (физика): " .. tostring(state.activeCount))
  if state.updateErrors > 0 then
    ui.textColored("Ошибок update: " .. tostring(state.updateErrors) .. " (см. лог)", rgbm(1, 0.4, 0.3, 1))
  end
  if log.writeError then
    ui.textColored("Лог не пишется: " .. tostring(log.writeError), rgbm(1, 0.4, 0.3, 1))
  end
  ui.separator()

  if state.recording then
    if ui.button("Остановить запись", vec2(ui.availableSpaceX(), 32)) then stopRecording() end
  else
    if ui.button("Начать запись маршрута", vec2(ui.availableSpaceX(), 32)) then startRecording() end
  end

  if ui.checkbox("Замкнутый маршрут (круг)", state.closed) then
    state.closed = not state.closed
    rebuildRoute()
  end
  if ui.button("Сохранить маршрут этой трассы", vec2(ui.availableSpaceX(), 28)) then saveRoute() end
  if ui.button("Загрузить маршрут этой трассы", vec2(ui.availableSpaceX(), 28)) then loadRoute() end

  ui.separator()
  local multiplier, changed = ui.slider("Скорость AI", state.speedMultiplier, 0.5, 1.8, "%.2fx")
  if changed then state.speedMultiplier = multiplier end
  local gapValue, gapChanged = ui.slider("Мин. дистанция до машины", state.followGap, 3, 15, "%.1f м")
  if gapChanged then state.followGap = gapValue end
  if ui.checkbox("Управлять всеми машинами (не только AI)", state.controlAllCars) then
    state.controlAllCars = not state.controlAllCars
    log:log('controlAllCars -> %s', tostring(state.controlAllCars))
  end

  ui.separator()
  if ui.button("Тест перехвата (тормоза + руль, 4 с)", vec2(ui.availableSpaceX(), 32)) then
    startSelfTest()
  end
  if state.selfTest.active then
    ui.textColored("Тест идёт: " .. string.format("%.1f", state.selfTest.timer) .. " с", rgbm(1, 0.8, 0.2, 1))
  elseif state.selfTest.verdict then
    local good = string.find(state.selfTest.verdict, "РАБОТАЕТ") ~= nil
    ui.textColored(state.selfTest.verdict,
      good and rgbm(0.3, 1, 0.5, 1) or rgbm(1, 0.3, 0.3, 1))
  end

  if state.autopilot then
    if ui.button("Остановить автопилот", vec2(ui.availableSpaceX(), 34)) then
      state.autopilot = false
      state.ai = {}
      state.controlledCount = 0
      state.slowingCount = 0
      state.activeCount = 0
      state.status = "Автопилот выключен"
      log:log('autopilot OFF')
      log:flush()
    end
  else
    if ui.button("Запустить автопилот", vec2(ui.availableSpaceX(), 34)) then
      if #state.route >= 3 then
        rebuildRoute()
        state.ai = {}
        state.autopilot = true
        state.status = "Автопилот активен"
        log:log('autopilot ON, %d route points', #state.route)
        log:flush()
      else
        state.status = "Сначала запиши или загрузи маршрут"
      end
    end
  end

  ui.separator()
  ui.textWrapped("Статус: " .. state.status)
  ui.text("Лог: " .. tostring(log:count()) .. " строк · файл routepilot_log.txt в папке приложения")
  if state.recording then ui.text("Записано точек: " .. tostring(#state.route)) end
  ui.text("Коробка передач остаётся под управлением автомобиля.")
end
