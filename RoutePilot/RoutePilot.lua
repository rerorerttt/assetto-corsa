-- RoutePilot: free-drive AI control for Assetto Corsa CSP.
-- v0.3.1: REMOVED the isAIControlled gate. Reference projects (nndrive) prove
--         that ac.overrideCarControls(0) drives the player car DIRECTLY,
--         every frame, WITHOUT Ctrl+8 and without any flag checks. The old
--         gate made the app silently do nothing whenever the flag wasn't set
--         — the exact "it stopped working" symptom. Now:
--         * the button "Запустить автопилот" is the master switch;
--         * while ON, the player car (#0) is driven every frame;
--         * Ctrl+8 is optional (works the same with or without it);
--         * when OFF (or on release), all overrides are zeroed and freed.
-- v0.3.0: free-drive style: wander steering, speed target, handbrake drift
--         pulses, collision avoidance, stuck detection; crash-safe logging.

local Logger = require('logger')

local APP_VERSION = "0.3.1"
local HEARTBEAT_INTERVAL = 1.0
local SELFTEST_SECONDS = 4.0
local BRAKE_ACCEL = 7.0
local FOLLOW_LATERAL = 2.6

local state = {
  initialized = false,
  sessionScanned = false,
  aiCount = 0,
  active = false,          -- master switch (the UI button)
  includeOpponents = false,
  speedMin = 50,   -- km/h
  speedMax = 110,  -- km/h
  followGap = 8.0,
  controlledCount = 0,
  activeCount = 0,
  slowingCount = 0,
  playerCaught = false,
  p = {},          -- per-car personalities
  selfTest = { active = false, carIndex = nil, timer = 0,
    steerSeen = 0, brakeSeen = 0, appliedFrames = 0, verdict = nil },
  log = nil,
  logClock = 0,
  heartbeatTimer = 0,
  updateErrors = 0,
  lastError = "",
  status = "Автопилот выключен",
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
    amp = 0.25 + math.random() * 0.5,
    period = 4 + math.random() * 6,
    phase = math.random() * math.pi * 2,
    speed = kmh / 3.6,
    hbEvery = 4 + math.random() * 7,
    hbTime = 2 + math.random() * 5,
    pulseLeft = 0,
    steer = 0, gas = 0, brake = 0, hb = 0,
    stuck = 0,
    slowing = false, active = false,
  }
  state.p[carIndex] = p
  log("car #%d personality: %.0f km/h, amp %.2f, period %.1f s",
    carIndex, kmh, p.amp, p.period)
end

-- ─────────────────── collision avoidance ───────────────────

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

-- ─────────────────── free-drive control ───────────────────

local function updateFreeCar(carIndex, car, dt)
  local p = state.p[carIndex]
  if not p then
    newPersonality(carIndex)
    p = state.p[carIndex]
  end
  p.t = p.t + dt

  -- wander steering: two sinusoids for a lively, unpredictable line
  local wander = p.amp * math.sin(math.pi * 2 * p.t / p.period + p.phase)
    + 0.35 * p.amp * math.sin(math.pi * 2 * p.t / (p.period * 0.37) + p.phase * 2.3)
  local steer = clamp(wander, -1, 1)

  -- speed planning with collision avoidance
  local speed = math.max(0, car.speedMs or 0)
  local targetSpeed = p.speed
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

  -- handbrake pulses initiate drifts
  p.hbTime = p.hbTime - dt
  if p.pulseLeft <= 0 and p.hbTime <= 0 then
    p.pulseLeft = 0.3 + math.random() * 0.5
    p.hbTime = p.hbEvery
  end
  p.hb = 0
  if p.pulseLeft > 0 then
    p.pulseLeft = p.pulseLeft - dt
    p.hb = 1
    gas = 1
    brake = 0
    steer = clamp(steer * 1.6, -1, 1)
  end

  -- stuck detection: re-randomize the personality to escape
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

  -- THE core mechanism, same as nndrive/FullThrottleAI: apply EVERY frame
  local ok, c = pcall(ac.overrideCarControls, carIndex)
  if ok and c then
    c.combineAxis = false
    c.steer = p.steer
    c.gas = gas
    c.brake = brake
    c.handbrake = p.hb
  else
    log("car #%d: overrideCarControls FAILED: %s", carIndex, tostring(c))
  end
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

local function releaseAll()
  local sim = ac.getSim()
  if sim then
    for i = 0, sim.carsCount - 1 do releaseCar(i) end
  end
end

-- ─────────────────── main control loop ───────────────────

local function updateControl(dt)
  local sim = ac.getSim()
  if not sim or not sim.isLive then return end
  state.controlledCount = 0
  state.slowingCount = 0
  state.playerCaught = false
  local active = {}

  -- v0.3.1: NO isAIControlled gate. The button is the switch. The player
  -- car is intercepted directly, exactly like the proven nndrive pattern.
  local player = ac.getCar(0)
  if player then
    updateFreeCar(0, player, dt)
    active[0] = true
    state.playerCaught = true
    state.controlledCount = state.controlledCount + 1
  end

  if state.includeOpponents then
    for i = 1, sim.carsCount - 1 do
      local car = ac.getCar(i)
      if car and car.isActive then
        updateFreeCar(i, car, dt)
        active[i] = true
        state.controlledCount = state.controlledCount + 1
      end
    end
  end

  for i in pairs(state.p) do
    if not active[i] then releaseCar(i) end
  end
  for i in pairs(active) do
    if state.p[i] and state.p[i].slowing then state.slowingCount = state.slowingCount + 1 end
  end

  state.activeCount = 0
  for i in pairs(active) do
    local ok, c = pcall(ac.overrideCarControls, i)
    local a = ok and c and c:active()
    if state.p[i] then state.p[i].active = a and true or false end
    if a then state.activeCount = state.activeCount + 1 end
  end
end

-- ─────────────────── self-test ───────────────────

local function startSelfTest()
  local sim = ac.getSim()
  if not sim then
    state.selfTest.verdict = "Сессия не активна"
    return
  end
  -- test the player car directly — that's the car we drive
  local carIndex = 0
  local car = ac.getCar(0)
  if not car then
    state.selfTest.verdict = "Нет машины игрока"
    return
  end
  state.selfTest = { active = true, carIndex = carIndex, timer = SELFTEST_SECONDS,
    steerSeen = 0, brakeSeen = 0, appliedFrames = 0, verdict = nil }
  state.status = "Тест перехвата: машина игрока"
  log("SELFTEST start: car#0 (player), cmd brake=1.00 steer=0.80 for %.1f s", SELFTEST_SECONDS)
  logFlush()
end

local function updateSelfTest(dt)
  local st = state.selfTest
  if not st.active then return end
  st.timer = st.timer - dt
  local car = st.carIndex and ac.getCar(st.carIndex) or nil
  if not car then
    st.active = false
    st.verdict = "Машина #" .. tostring(st.carIndex) .. " исчезла"
    return
  end
  local ok, c = pcall(ac.overrideCarControls, st.carIndex)
  if not ok or not c then
    st.active = false
    st.verdict = "overrideCarControls вернул nil — перехват недоступен"
    log("SELFTEST: overrideCarControls(%d) -> nil/ERROR", st.carIndex)
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
  if st.appliedFrames % 10 == 1 then
    log("SELFTEST car#%d: cmd(steer 0.80 brake 1.00) | car reports steer %.2f brake %.2f | %.1f km/h | active=%s",
      st.carIndex, steerSeen, brakeSeen, car.speedKmh, tostring(c:active()))
  end
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
  local aiCount = 0
  for i = 0, sim.carsCount - 1 do
    local car = ac.getCar(i)
    if car then
      local name = "?"
      pcall(function() name = ac.getCarName(i) end)
      log("CAR#%d: active=%s aiControlled=%s speed=%.1f name=%s",
        i, tostring(car.isActive), tostring(car.isAIControlled), car.speedKmh, name)
      if car.isAIControlled and i > 0 then aiCount = aiCount + 1 end
    end
  end
  state.aiCount = aiCount
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
    if path then
      log("log file: %s", path)
    else
      log("LOG ERROR: file could not be created: %s", tostring(state.log.writeError))
      ac.error("[RoutePilot] log file could not be created: " .. tostring(state.log.writeError))
    end
    logFlush()
  end

  local sim = ac.getSim()
  if sim then scanSession(sim) end
  if not sim or not sim.isLive then return end

  updateSelfTest(dt)

  if state.active then
    updateControl(dt)
  else
    state.controlledCount = 0
    state.slowingCount = 0
    state.activeCount = 0
    state.playerCaught = false
  end

  state.heartbeatTimer = state.heartbeatTimer + dt
  if state.heartbeatTimer >= HEARTBEAT_INTERVAL then
    state.heartbeatTimer = 0
    if state.active then
      state.log:heartbeat(string.format(
        "auto=ON playerCaught=%s controlled=%d active=%d errors=%d",
        tostring(state.playerCaught), state.controlledCount,
        state.activeCount, state.updateErrors))
      for carIndex, p in pairs(state.p) do
        local car = ac.getCar(carIndex)
        if car then
          local okGas, gas = pcall(function() return car.gas end)
          local okBrake, brake = pcall(function() return car.brake end)
          local okSteer, steer = pcall(function() return car.steer end)
          log("CAR#%d: cmd steer %.2f gas %.2f brake %.2f hb %.1f | car reports steer %.2f gas %.2f brake %.2f | %.1f km/h | active=%s",
            carIndex, p.steer, p.gas, p.brake, p.hb,
            (okSteer and type(steer) == 'number') and steer or -1,
            (okGas and type(gas) == 'number') and gas or -1,
            (okBrake and type(brake) == 'number') and brake or -1,
            car.speedKmh, tostring(p.active))
        end
      end
    else
      state.log:heartbeat(string.format("auto=OFF aiCars=%d errors=%d",
        state.aiCount, state.updateErrors))
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
  releaseAll()
  if state.log then
    state.log:shutdown('script released (session end or game exit)')
  end
end

function script.windowMain(dt)
  ui.text("RoutePilot  " .. APP_VERSION)
  ui.text("Под управлением: " .. tostring(state.controlledCount)
    .. "   Перехват активен (физика): " .. tostring(state.activeCount))
  if state.playerCaught then
    ui.textColored("Твоя машина ведётся RoutePilot", rgbm(0.3, 1, 0.5, 1))
  end
  if state.updateErrors > 0 then
    ui.textColored("Ошибок update: " .. tostring(state.updateErrors) .. " (см. лог)", rgbm(1, 0.4, 0.3, 1))
  end
  ui.separator()

  local sMin, chMin = ui.slider("Мин. скорость, км/ч", state.speedMin, 10, 80, "%.0f")
  if chMin then state.speedMin = sMin end
  local sMax, chMax = ui.slider("Макс. скорость, км/ч", state.speedMax, 20, 160, "%.0f")
  if chMax then state.speedMax = math.max(sMax, state.speedMin + 5) end
  local gVal, gCh = ui.slider("Мин. дистанция до машины", state.followGap, 3, 15, "%.1f м")
  if gCh then state.followGap = gVal end
  if ui.checkbox("Также вести AI-соперников", state.includeOpponents) then
    state.includeOpponents = not state.includeOpponents
    log("OPTION includeOpponents=%s", tostring(state.includeOpponents))
  end
  ui.separator()

  if state.active then
    if ui.button("Остановить автопилот", vec2(ui.availableSpaceX(), 34)) then
      releaseAll()
      state.active = false
      state.controlledCount = 0
      state.slowingCount = 0
      state.activeCount = 0
      state.status = "Автопилот выключен"
      log("AUTOPILOT stop (all released)")
      logFlush()
    end
  else
    if ui.button("Запустить автопилот (моя машина)", vec2(ui.availableSpaceX(), 34)) then
      state.p = {}
      state.active = true
      state.status = "Автопилот активен"
      log("AUTOPILOT start: free-drive, includeOpponents=%s",
        tostring(state.includeOpponents))
      logFlush()
    end
  end

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
  ui.textWrapped("Кнопка = единственный выключатель. Ctrl+8 не обязателен: включи автопилот — машина сама поедет, будет вилять и дрифтить. Не забудь нажать кнопку (в логе должно появиться auto=ON).")
  ui.text("Лог: " .. (state.log and state.log.path or "создаётся…"))
end
