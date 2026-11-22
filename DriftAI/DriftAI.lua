-- DriftAI main script (v1.1.0).
-- Makes every AI car in an offline session drift around the track continuously:
-- auto-detects AI cars, neutralizes the stock AI controller, fully overrides
-- controls (ac.overrideCarControls with combineAxis = false) and steers along
-- the track geometry. No AI fast lane, no per-car setup, no track preparation.
--
-- v1.1.0: at drift start the app scans the whole track once with physics
-- raycasts (Bounds module) to fix the drivable area and wall distances;
-- drivers use this map to decide how fast they can go and how long a drift
-- arc fits before a wall, and manage throttle to keep the rear wheels
-- spinning (wheelspin-targeted throttle in Driver).

local Driver = require('driver')
local Bounds = require('bounds')

local APP_VERSION = '1.1.0'
local ACTIVE_CHECK_INTERVAL = 1.0 -- seconds between controls:active() checks
local SPAWN_SCAN_INTERVAL = 2.0   -- seconds between AI-car rescans

local state = {
  running = false,
  drivers = {},          -- [carIndex] = Driver
  activeCount = 0,
  aiCount = 0,
  appliedCount = 0,
  lastActiveCheck = 0,
  lastScan = 0,
  trackLengthM = 0,
  status = 'Готово. Нажми «Запустить дрифт».',
  splineAvailable = false,
  mapReady = false,
  mapProgress = 0,
  avgSlip = 0,
  showDebug = false,
}

local config = {}

-- ─────────────────── helpers ───────────────────

local function fmt(n)
  return string.format('%.1f', n)
end

local function collectAICars()
  local list = {}
  local sim = ac.getSim()
  for i = 0, sim.carsCount - 1 do
    local car = ac.getCar(i)
    if car and car.isActive and car.isAIControlled then
      list[#list + 1] = i
    end
  end
  return list
end

local function statusText(d)
  if d.state == 'drift' then return 'ДРИФТ' end
  if d.state == 'accelerate' then return 'разгон' end
  if d.state == 'recover' then return 'восстановление' end
  return 'поиск трассы'
end

-- ─────────────────── start/stop ───────────────────

local function startDrift()
  local sim = ac.getSim()
  if not sim then
    state.status = 'Сессия не активна'
    return
  end
  if sim.isOnlineRace then
    state.status = 'Онлайн-сессии не поддерживаются'
    return
  end
  if sim.isReplayActive then
    state.status = 'Реплей: управление недоступно'
    return
  end

  config.trackLengthM = sim.trackLengthM or 0
  state.splineAvailable = ac.hasTrackSpline and ac.hasTrackSpline() or false

  -- scan the whole track once with physics raycasts: drivable area + walls
  config.bounds = Bounds.new(config.trackLengthM)
  state.mapReady = config.bounds.ready
  state.mapProgress = config.bounds:progressFraction()

  state.drivers = {}
  state.running = true
  state.lastScan = 0
  state.status = state.mapReady
    and 'Дрифт запущен (карта границ не нужна: трасса без сплайна)'
    or 'Дрифт запущен: сканирую границы трассы…'
  ac.log('[DriftAI] started, track length: ' .. tostring(config.trackLengthM) .. ' m')
end

local function stopDrift()
  state.running = false
  state.drivers = {}
  state.activeCount = 0
  state.appliedCount = 0
  state.status = 'Остановлено. AI вернулись к штатному поведению.'
  ac.log('[DriftAI] stopped')
end

local function adoptCar(carIndex)
  local d = Driver.new(carIndex, config)
  state.drivers[carIndex] = d
  d:disableStockAI()
  ac.log('[DriftAI] adopted AI car #' .. carIndex)
end

-- ─────────────────── main update ───────────────────

function script.update(dt)
  local sim = ac.getSim()
  if not sim or sim.isReplayActive or sim.isOnlineRace then return end
  if sim.dt == 0 then return end -- paused

  if not state.running then
    state.lastScan = state.lastScan + dt
    if state.lastScan >= SPAWN_SCAN_INTERVAL then
      state.lastScan = 0
      local list = collectAICars()
      state.aiCount = #list
      if #list == 0 then
        state.status = 'AI-машин нет. Добавь соперников в сетапе сессии.'
      end
    end
    return
  end

  -- build the boundary map over a few frames right after the start
  if config.bounds and not config.bounds.ready then
    config.bounds:buildStep()
    state.mapProgress = config.bounds:progressFraction()
    if config.bounds.ready then
      state.mapReady = true
      state.status = 'Карта границ готова: ' .. tostring(config.bounds.good)
        .. ' точек. AI в деле.'
      ac.log('[DriftAI] boundary map ready: ' .. tostring(config.bounds.good)
        .. '/' .. tostring(config.bounds.total) .. ' samples')
    end
  end

  -- rescan for new AI cars periodically
  state.lastScan = state.lastScan + dt
  if state.lastScan >= SPAWN_SCAN_INTERVAL then
    state.lastScan = 0
    local list = collectAICars()
    state.aiCount = #list
    for _, carIndex in ipairs(list) do
      if not state.drivers[carIndex] then adoptCar(carIndex) end
    end
    for carIndex in pairs(state.drivers) do
      local car = ac.getCar(carIndex)
      if not car or not car.isAIControlled or not car.isActive then
        state.drivers[carIndex] = nil
      end
    end
  end

  -- per-frame control of every AI car
  local applied, slipSum, slipCount = 0, 0, 0
  for carIndex, driver in pairs(state.drivers) do
    local car = ac.getCar(carIndex)
    if car and car.isActive then
      driver:update(car, dt)
      if driver:applyControls(dt) then applied = applied + 1 end
      slipSum = slipSum + math.abs(driver.slipAngle)
      slipCount = slipCount + 1
    end
  end

  -- verify the physics actually consumes the overrides (watchdog)
  state.lastActiveCheck = state.lastActiveCheck + dt
  if state.lastActiveCheck >= ACTIVE_CHECK_INTERVAL then
    state.lastActiveCheck = 0
    local active = 0
    for _, driver in pairs(state.drivers) do
      if driver:isActive() then active = active + 1 end
    end
    state.activeCount = active
  end

  state.appliedCount = applied
  if slipCount > 0 then state.avgSlip = math.deg(slipSum / slipCount) end

  if state.appliedCount == 0 and state.aiCount == 0 then
    state.status = 'AI-машин нет. Добавь соперников в сетапе сессии.'
  elseif state.appliedCount > 0 and state.mapReady then
    state.status = 'Дрифт: ' .. tostring(state.appliedCount) .. ' из '
      .. tostring(state.aiCount) .. ' AI-машин'
  end
end

-- ─────────────────── UI ───────────────────

local function textColored(text, color)
  ui.pushStyleColor(ui.StyleColor.Text, color)
  ui.text(text)
  ui.popStyleColor()
end

function script.windowMain(dt)
  ui.text('DriftAI  ' .. APP_VERSION)
  ui.pushFont(ui.Font.Small)
  ui.text('Автономный дрифт AI-машин · CSP 0.3.2-preview2+')
  ui.popFont()
  ui.separator()

  ui.textWrapped('Статус: ' .. state.status)
  ui.text('AI-машин в сессии: ' .. tostring(state.aiCount)
    .. '  ·  под контролем: ' .. tostring(state.appliedCount)
    .. '  ·  перехват активен: ' .. tostring(state.activeCount))
  ui.text('Сплайн трассы: ' .. (state.splineAvailable and 'есть' or 'нет'))
  if state.running and not state.mapReady then
    ui.text('Карта границ: ' .. string.format('%.0f%%', state.mapProgress * 100)
      .. ' (вертикальные коллизии + стены)')
  else
    ui.text('Карта границ: ' .. (state.mapReady and 'готова' or 'не запущена'))
  end
  ui.separator()

  if state.running then
    if ui.button('Остановить', vec2(ui.availableSpaceX(), 36)) then
      stopDrift()
    end
  else
    if ui.button('Запустить дрифт', vec2(ui.availableSpaceX(), 36)) then
      startDrift()
    end
  end

  ui.textWrapped('После запуска приложение само найдёт все AI-машины, отсканирует '
    .. 'границы трассы лучами (проезжаемая поверхность, стены слева/справа/впереди), '
    .. 'отключит штатный AI-контроллер и поведёт каждую машину в управляемом заносе: '
    .. 'газ держит пробуксовку задних колёс, скорость планирует по расстоянию до стен. '
    .. 'Ничего настраивать не нужно.')

  ui.separator()
  ui.text('Средний угол скольжения: ' .. fmt(state.avgSlip) .. '°')

  ui.separator()
  ui.text('Машины:')
  local any = false
  for carIndex, driver in pairs(state.drivers) do
    any = true
    local car = ac.getCar(carIndex)
    local name = car and ac.getCarName(carIndex) or ('#' .. carIndex)
    local color = rgbm(0.4, 1, 0.6, 1)
    if driver.state == 'recover' then color = rgbm(1, 0.8, 0.2, 1) end
    if driver.state == 'lost' then color = rgbm(1, 0.4, 0.3, 1) end
    local extra = ''
    if driver.rearSlip then extra = string.format('  букс %.2f', driver.rearSlip) end
    if driver.wallAhead then extra = extra .. string.format('  стена %d м', driver.wallAhead) end
    textColored(string.format('#%d %s — %s  %s  slip %.0f°%s',
      carIndex, name, statusText(driver),
      car and (fmt(car.speedKmh) .. ' км/ч') or '--',
      math.deg(driver.slipAngle), extra), color)
  end
  if not any then
    ui.pushFont(ui.Font.Small)
    ui.textWrapped('Пока пусто: AI-машины подхватываются автоматически после запуска.')
    ui.popFont()
  end

  ui.separator()
  if ui.checkbox('Показывать отладочную информацию', state.showDebug) then
    state.showDebug = not state.showDebug
  end
  if state.showDebug then
    ui.pushFont(ui.Font.Small)
    ui.textWrapped('Длина трассы: ' .. fmt(config.trackLengthM) .. ' м')
    ui.textWrapped('Перехват: ac.overrideCarControls(i), combineAxis=false, '
      .. 'steer/gas/brake/handbrake/requestedGearIndex; штатный AI отключён '
      .. 'через physics.setAIAggression/setAICaution.')
    ui.textWrapped('Карта: physics.raycastTrack, шаг 8 м; целевая пробуксовка 0.30; '
      .. 'скорость = sqrt(2*a*(d_стены − 8 м)).')
    ui.popFont()
  end
end

function script.onSessionStart()
  state.drivers = {}
  state.running = false
  state.mapReady = false
  state.mapProgress = 0
  state.status = 'Новая сессия. Нажми «Запустить дрифт».'
  local sim = ac.getSim()
  if sim then config.trackLengthM = sim.trackLengthM or 0 end
  ac.log('[DriftAI] session start')
end
