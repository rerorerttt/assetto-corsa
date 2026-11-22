-- RoutePilot: takeover of any AI car with wall-aware drift — CSP Lua app.
-- v0.3.3: WALL AWARENESS. New module bounds.lua builds a track boundary map
--   ONCE at session start (spline sides + horizontal raycasts for vertical
--   collisions). The drift AI now:
--     * steers toward the widest open area (best drift line, away from walls);
--     * plans speed by the distance to the nearest wall ahead:
--       v = sqrt(2 * a * (d - margin));
--     * only launches a drift when there is enough run-off room;
--     * priorities: HIGH SPEED and WHEELSPIN (long slides, wall-to-wall).
-- v0.3.2 (WORKING RELEASE): activation copied 1:1 from the proven
--   FullThrottleAI — no buttons, every frame every isAIControlled car is
--   taken over; setAIThrottleLimit; full gas held while rear wheels spin.
-- Author: Ashna-X1 (AshnaAI).

local Logger = require('logger')
local Bounds = require('bounds')

local APP_VERSION = "0.3.3"
local HEARTBEAT_INTERVAL = 1.0
local BRAKE_ACCEL = 7.0
local FOLLOW_LATERAL = 3.0
local WALL_MARGIN = 6.0        -- keep this far from walls, meters
local MIN_DRIFT_ROOM = 12.0    -- don't drift without this much run-off, m

local state = {
  initialized = false,
  sessionScanned = false,
  enabled = true,
  speedMin = 60,   -- km/h
  speedMax = 140,  -- km/h
  followGap = 8.0,
  controlledCount = 0,
  playerAI = false,
  bounds = nil,
  p = {},
  selfTest = { active = false, carIndex = nil, timer = 0,
    steerSeen = 0, brakeSeen = 0, appliedFrames = 0, verdict = nil },
  log = nil,
  logClock = 0,
  heartbeatTimer = 0,
  updateErrors = 0,
  lastError = "",
  status = "Готово: включи Ctrl+8 — машина поедет и задрифтит вдоль стен",
}

-- ─────────────────── helpers ───────────────────

local function clamp(v, lo, hi)
  if v < lo then return lo end
  if v > hi then return hi end
  return v
end

local function log(fmt, ...)
  local l = state.log
  if l then l:log(fmt, ...) end
end

local function logFlush()
  if state.log then state.log:flush() end
end

-- ─────────────────── personalities ───────────────────

local function newPersonality(carIndex)
  local span = math.max(5, state.speedMax - state.speedMin)
  local kmh = state.speedMin + math.random() * span
  local p = {
    t = math.random() * 100,
    amp = 0.3 + math.random() * 0.5,
    period = 4 + math.random() * 6,
    phase = math.random() * math.pi * 2,
    speed = kmh / 3.6,
    hbEvery = 4 + math.random() * 7,
    hbTime = 2 + math.random() * 5,
    pulseLeft = 0,
    steer = 0, gas = 0, brake = 0, hb = 0,
    stuck = 0,
    slowing = false,
  }
  state.p[carIndex] = p
  log("car #%d personality: %.0f km/h, amp %.2f, period %.1f s",
    carIndex, kmh, p.amp, p.period)
end

-- rear-axle wheelspin magnitude
local function rearSlip(car)
  local slip = 0
  pcall(function()
    local w = car.wheels
    if w then
      if w[2] and type(w[2].slipRatio) == 'number' then slip = slip + math.abs(w[2].slipRatio) end
      if w[3] and type(w[3].slipRatio) == 'number' then slip = slip + math.abs(w[3].slipRatio) end
    end
  end)
  return slip
end

-- nearest car ahead in a forward cone
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

-- ─────────────────── wall-aware drift driving ───────────────────

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

local function updateDriftCar(carIndex, car, dt)
  local p = state.p[carIndex]
  if not p then
    newPersonality(carIndex)
    p = state.p[carIndex]
    log("car #%d: AI control detected, RoutePilot takes over", carIndex)
  end
  p.t = p.t + dt

  -- base wander steering
  local wander = p.amp * math.sin(math.pi * 2 * p.t / p.period + p.phase)
    + 0.35 * p.amp * math.sin(math.pi * 2 * p.t / (p.period * 0.37) + p.phase * 2.3)
  local steer = clamp(wander, -1, 1)

  -- WALL AWARENESS: distances left/right/forward
  local dLeft, dRight, dFwd = state.bounds:liveRays(car)
  local room = dLeft + dRight
  local nearWall = math.min(dLeft, dRight)
  local offCenter = clamp((dRight - dLeft) / math.max(1, room), -1, 1)

  -- steer toward the widest part of the area (away from the closer wall)
  local wallAvoid = -offCenter * 0.8
  -- urgency grows as the car closes on a wall
  local urgency = clamp(1 - nearWall / 20, 0, 1)
  steer = clamp(steer * (1 - 0.6 * urgency) + wallAvoid * urgency, -1, 1)

  -- speed planning: high speed prioritized, but capped by the wall ahead
  local speed = math.max(0, car.speedMs or 0)
  local targetSpeed = p.speed
  local wallSpeedCap = math.sqrt(math.max(0, 2 * BRAKE_ACCEL * math.max(0, dFwd - WALL_MARGIN)))
  if wallSpeedCap > 0 then targetSpeed = math.min(targetSpeed, wallSpeedCap) end

  local gap, leaderSpeed = carAhead(carIndex, car)
  local slowing = false
  if gap then
    local safeSpeed = math.sqrt(math.max(0, leaderSpeed * leaderSpeed
      + 2 * BRAKE_ACCEL * math.max(0, gap - state.followGap)))
    if safeSpeed < targetSpeed then targetSpeed = safeSpeed; slowing = true end
    if gap < state.followGap then targetSpeed = 0; slowing = true end
  end
  p.slowing = slowing

  local err = targetSpeed - speed
  local gas = clamp(err * 0.4, 0, 1)
  local brake = clamp(-err * 0.5, 0, 1)

  -- handbrake pulses kick the car sideways, but ONLY with run-off room
  p.hbTime = p.hbTime - dt
  if p.pulseLeft <= 0 and p.hbTime <= 0 then
    if nearWall >= MIN_DRIFT_ROOM then
      p.pulseLeft = 0.3 + math.random() * 0.5
      p.hbTime = p.hbEvery
    else
      p.hbTime = 1.0   -- too close to a wall: retry shortly
    end
  end
  p.hb = 0
  if p.pulseLeft > 0 then
    p.pulseLeft = p.pulseLeft - dt
    p.hb = 1
    gas = 1
    brake = 0
    steer = clamp(steer * 1.6, -1, 1)
  end

  -- while the rear wheels spin (drift), hold full gas so the slide lives
  local slip = rearSlip(car)
  if slip > 0.25 then
    gas = 1
    brake = 0
  end

  -- stuck detection
  if speed < 1.5 and p.pulseLeft <= 0 then
    p.stuck = p.stuck + dt
    if p.stuck > 3 then
      p.stuck = 0
      newPersonality(carIndex)
      p = state.p[carIndex]
      log("car #%d: stuck, new personality", carIndex)
    end
  else
    p.stuck = 0
  end

  p.steer = p.steer + (steer - p.steer) * clamp(dt * 8, 0, 1)
  p.gas, p.brake = gas, brake
  p.dLeft, p.dRight, p.dFwd = dLeft, dRight, dFwd

  -- the proven FullThrottleAI mechanism
  local controls = ac.overrideCarControls(carIndex)
  if controls then
    controls.combineAxis = false
    controls.steer = p.steer
    controls.gas = gas
    controls.brake = brake
    controls.handbrake = p.hb
  else
    log("car #%d: overrideCarControls returned nil", carIndex)
  end
  if physics and physics.setAIThrottleLimit then
    pcall(physics.setAIThrottleLimit, carIndex, 1)
  end
end

-- ─────────────────── self-test ───────────────────

local function startSelfTest()
  local car = ac.getCar(0)
  if not car then
    state.selfTest.verdict = "Нет машины игрока"
    return
  end
  state.selfTest = { active = true, carIndex = 0, timer = 4.0,
    steerSeen = 0, brakeSeen = 0, appliedFrames = 0, verdict = nil }
  state.status = "Тест перехвата: машина игрока"
  log("SELFTEST start: car#0, cmd brake=1.00 steer=0.80 for 4.0 s (включи Ctrl+8!)")
  logFlush()
end

local function updateSelfTest(dt)
  local st = state.selfTest
  if not st.active then return end
  st.timer = st.timer - dt
  local car = ac.getCar(st.carIndex)
  if not car then
    st.active = false
    st.verdict = "Машина исчезла"
    return
  end
  local ok, c = pcall(ac.overrideCarControls, st.carIndex)
  if not ok or not c then
    st.active = false
    st.verdict = "overrideCarControls вернул nil — перехват недоступен"
    log("SELFTEST: overrideCarControls(0) -> nil/ERROR")
    logFlush()
    return
  end
  c.combineAxis = false
  c.gas = 0
  c.brake = 1
  c.steer = 0.8
  st.appliedFrames = st.appliedFrames + 1
  local brakeSeen = car.brake or 0
  local steerSeen = car.steer or 0
  if brakeSeen > 0.5 then st.brakeSeen = st.brakeSeen + 1 end
  if math.abs(steerSeen) > 0.3 then st.steerSeen = st.steerSeen + 1 end
  if st.timer <= 0 then
    st.active = false
    if st.brakeSeen >= 10 or st.steerSeen >= 10 then
      st.verdict = "ПЕРЕХВАТ РАБОТАЕТ (тормоз виден " .. st.brakeSeen
        .. " кадров из " .. st.appliedFrames .. ")"
    else
      st.verdict = "ПЕРЕХВАТ НЕ РАБОТАЕТ (тормоз виден " .. st.brakeSeen
        .. " кадров из " .. st.appliedFrames .. ")"
    end
    log("SELFTEST verdict: %s", st.verdict)
    local ok2, c2 = pcall(ac.overrideCarControls, st.carIndex)
    if ok2 and c2 then
      c2.brake = 0
      c2.steer = 0
    end
    state.status = "Тест перехвата завершён"
    logFlush()
  end
end

-- ─────────────────── session scan ───────────────────

local function scanSession(sim)
  if state.sessionScanned or not sim or not sim.isLive then return end
  state.sessionScanned = true
  log("SESSION: cars=%d", sim.carsCount)
  for i = 0, sim.carsCount - 1 do
    local car = ac.getCar(i)
    if car then
      local name = "?"
      pcall(function() name = ac.getCarName(i) end)
      log("CAR#%d: active=%s aiControlled=%s name=%s",
        i, tostring(car.isActive), tostring(car.isAIControlled), name)
    end
  end
  logFlush()
end

-- ─────────────────── main loop ───────────────────

local function updateInner(dt)
  state.logClock = state.logClock + dt

  if not state.initialized then
    state.initialized = true
    state.log = Logger.new('routepilot_log.txt', 2000)
    local path = state.log:init()
    log("=== RoutePilot %s log start ===", APP_VERSION)
    if path then log("log file: %s", path) end
    logFlush()
  end

  if not state.bounds then
    state.bounds = Bounds.new(state.log)
  end
  if not state.bounds:ready() then
    state.bounds:startBuild()
    state.bounds:update(dt)
  end

  local sim = ac.getSim()
  if sim then scanSession(sim) end
  if not sim or not sim.isLive then return end

  updateSelfTest(dt)

  local controlled = 0
  if state.enabled then
    for i = 0, sim.carsCount - 1 do
      local car = ac.getCar(i)
      if car then
        local isAI = false
        pcall(function() isAI = car.isAIControlled and true or false end)
        if i == 0 then state.playerAI = isAI end
        if isAI then
          updateDriftCar(i, car, dt)
          controlled = controlled + 1
        elseif state.p[i] then
          releaseCar(i)
        end
      end
    end
  else
    for i in pairs(state.p) do releaseCar(i) end
  end
  state.controlledCount = controlled

  state.heartbeatTimer = state.heartbeatTimer + dt
  if state.heartbeatTimer >= HEARTBEAT_INTERVAL then
    state.heartbeatTimer = 0
    state.log:heartbeat(string.format(
      "controlled=%d ctrl8=%s errors=%d",
      state.controlledCount, tostring(state.playerAI), state.updateErrors))
    for carIndex, p in pairs(state.p) do
      local car = ac.getCar(carIndex)
      if car then
        log("CAR#%d: cmd steer %.2f gas %.2f brake %.2f hb %.1f | %.1f km/h | walls L%.0f R%.0f F%.0f | slip %.2f",
          carIndex, p.steer, p.gas, p.brake, p.hb,
          car.speedKmh or 0,
          p.dLeft or -1, p.dRight or -1, p.dFwd or -1,
          rearSlip(car))
      end
    end
    logFlush()
  end
end

function script.update(dt)
  local ok, err = pcall(updateInner, dt)
  if not ok then
    state.updateErrors = state.updateErrors + 1
    state.lastError = tostring(err)
    if state.log then
      log("update ERROR #%d: %s", state.updateErrors, state.lastError)
      logFlush()
    else
      ac.error("[RoutePilot] update error: " .. tostring(err))
    end
  end
end

script.onRelease = function()
  local sim = ac.getSim()
  if sim then
    for i = 0, sim.carsCount - 1 do releaseCar(i) end
  end
  if state.log then
    state.log:shutdown('script released (session end or game exit)')
  end
end

function script.windowMain(dt)
  ui.text("RoutePilot  " .. APP_VERSION)
  if state.playerAI then
    ui.textColored("Ctrl+8: ВКЛ — твоя машина под RoutePilot", rgbm(0.3, 1, 0.5, 1))
  else
    ui.textColored("Ctrl+8: выкл — нажми Ctrl+8", rgbm(1, 0.8, 0.2, 1))
  end
  ui.text("Машин под управлением: " .. tostring(state.controlledCount))
  if state.bounds then
    ui.text("Карта границ: " .. (state.bounds:ready() and ("готова, " .. tostring(state.bounds.count) .. " точек") or ("строится... " .. tostring(state.bounds.count))))
  end
  if state.updateErrors > 0 then
    ui.textColored("Ошибок update: " .. tostring(state.updateErrors) .. " (см. лог)", rgbm(1, 0.4, 0.3, 1))
  end
  ui.separator()

  if ui.checkbox("RoutePilot включён", state.enabled) then
    state.enabled = not state.enabled
    log("OPTION enabled=%s", tostring(state.enabled))
  end
  local sMin, chMin = ui.slider("Мин. скорость, км/ч", state.speedMin, 10, 100, "%.0f")
  if chMin then state.speedMin = sMin end
  local sMax, chMax = ui.slider("Макс. скорость, км/ч", state.speedMax, 30, 200, "%.0f")
  if chMax then state.speedMax = math.max(sMax, state.speedMin + 5) end
  local gVal, gCh = ui.slider("Мин. дистанция до машин", state.followGap, 3, 15, "%.1f м")
  if gCh then state.followGap = gVal end
  ui.separator()

  if ui.button("Тест перехвата (машина игрока, 4 с)", vec2(ui.availableSpaceX(), 28)) then
    startSelfTest()
  end
  if state.selfTest.active then
    ui.textColored("Тест идёт: " .. string.format("%.1f", state.selfTest.timer) .. " с", rgbm(1, 0.8, 0.2, 1))
  elseif state.selfTest.verdict then
    local good = string.find(state.selfTest.verdict, "РАБОТАЕТ") ~= nil
    ui.textColored(state.selfTest.verdict,
      good and rgbm(0.3, 1, 0.5, 1) or rgbm(1, 0.3, 0.3, 1))
  end

  ui.separator()
  ui.textWrapped("Статус: " .. state.status)
  ui.textWrapped("v0.3.3: ИИ видит стены (карта границ по рейкастам). Дрифт запускается только при запасе до стены, скорость ограничена расстоянием до стены впереди. Приоритет — высокая скорость и пробуксовка.")
  ui.text("Лог: " .. (state.log and state.log.path or "создаётся…"))
end
