-- RoutePilot: free-drive AI interception for Assetto Corsa CSP.
-- v0.3.2: EXACT copy of the proven FullThrottleAI mechanism:
--   * NO buttons, NO master switch — script.update runs every frame and
--     takes over ANY car with isAIControlled == true:
--       - the player car the moment Ctrl+8 is pressed (FullThrottleAI proof),
--       - every AI opponent in the session;
--   * while a car is caught, RoutePilot drives it with drift style:
--     wander steering, target speed, handbrake pulses, full gas in slides;
--   * AI throttle limit lifted via physics.setAIThrottleLimit (as in the
--     working app);
--   * when a car stops being AI-controlled, the override is zeroed at once.
-- No fast lane, no route, no files needed. Crash-safe logging kept.

local Logger = require('logger')

local APP_VERSION = "0.3.2"
local HEARTBEAT_INTERVAL = 1.0
local SELFTEST_SECONDS = 4.0
local BRAKE_ACCEL = 7.0
local FOLLOW_LATERAL = 2.6

local state = {
  initialized = false,
  sessionScanned = false,
  speedMin = 50,   -- km/h
  speedMax = 110,  -- km/h
  followGap = 8.0,
  caught = 0,          -- cars currently intercepted
  activePhys = 0,      -- overrides the physics actually consumes
  slowingCount = 0,
  playerCaught = false,
  ctrl8Was = false,
  p = {},               -- per-car personalities
  selfTest = { active = false, carIndex = nil, timer = 0,
    steerSeen = 0, brakeSeen = 0, appliedFrames = 0, verdict = nil },
  log = nil,
  logClock = 0,
  heartbeatTimer = 0,
  updateErrors = 0,
  lastError = "",
  status = "Ожидание: включи Ctrl+8 (или сессию с AI-соперниками)",
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
  log("car #%d caught: personality %.0f km/h, amp %.2f, period %.1f s",
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

-- ─────────────────── free-drive drift control ───────────────────

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
    end
  else
    p.stuck = 0
  end

  p.steer = p.steer + (steer - p.steer) * clamp(dt * 8, 0, 1)
  p.gas, p.brake = gas, brake

  -- THE core mechanism (exactly like the working FullThrottleAI):
  -- apply every frame while the car is AI-controlled
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

  -- lift the AI throttle limit, same guard as in the working app
  if physics and physics.setAIThrottleLimit then
    physics.setAIThrottleLimit(carIndex, 1)
  end
end

local function releaseCar(carIndex)
  local controls = ac.overrideCarControls(carIndex)
  if controls then
    controls.combineAxis = false
    controls.steer = 0
    controls.gas = 0
    controls.brake = 0
    controls.handbrake = 0
  end
  if state.p[carIndex] then
    log("car #%d released (no longer AI-controlled)", carIndex)
  end
  state.p[carIndex] = nil
end

-- ─────────────────── main control loop (always on) ───────────────────

local function updateControl(dt)
  local sim = ac.getSim()
  if not sim or not sim.isLive then return end
  state.caught = 0
  state.slowingCount = 0
  state.playerCaught = false
  local active = {}

  -- EXACT FullThrottleAI condition, applied to EVERY car:
  -- any car with isAIControlled == true is ours to drive
  for i = 0, sim.carsCount - 1 do
    local car = ac.getCar(i)
    if car and car.isAIControlled then
      if i == 0 then state.playerCaught = true end
      updateFreeCar(i, car, dt)
      active[i] = true
      state.caught = state.caught + 1
    end
  end

  -- release cars that stopped being AI-controlled (Ctrl+8 off, AI left)
  for i in pairs(state.p) do
    if not active[i] then releaseCar(i) end
  end
  for i in pairs(active) do
    if state.p[i] and state.p[i].slowing then state.slowingCount = state.slowingCount + 1 end
  end

  state.activePhys = 0
  for i in pairs(active) do
    local controls = ac.overrideCarControls(i)
    local a = controls and controls:active()
    if state.p[i] then state.p[i].active = a and true or false end
    if a then state.activePhys = state.activePhys + 1 end
  end

  -- log the Ctrl+8 transition for the player car
  if state.playerCaught ~= state.ctrl8Was then
    log("PLAYER: Ctrl+8 %s", state.playerCaught and "ON — car #0 intercepted" or "OFF — car #0 released")
    state.ctrl8Was = state.playerCaught
    logFlush()
  end

  if state.caught > 0 then
    state.status = "Перехвачено машин: " .. tostring(state.caught)
      .. (state.playerCaught and " (в т.ч. твоя — Ctrl+8)" or " (AI-соперники)")
  else
    state.status = "Ожидание: включи Ctrl+8 (или сессию с AI-соперниками)"
  end
end

-- ─────────────────── self-test ───────────────────

local function startSelfTest()
  local sim = ac.getSim()
  if not sim then
    state.selfTest.verdict = "Сессия не активна"
    return
  end
  local carIndex = 0
  local car = ac.getCar(0)
  if not car then
    state.selfTest.verdict = "Нет машины игрока"
    return
  end
  state.selfTest = { active = true, carIndex = carIndex, timer = SELFTEST_SECONDS,
    steerSeen = 0, brakeSeen = 0, appliedFrames = 0, verdict = nil }
  state.status = "Тест перехвата: машина игрока"
  log("SELFTEST start: car#0 (player), cmd brake=1.00 steer=0.80 for %.1f s; Ctrl+8=%s",
    SELFTEST_SECONDS, tostring(car.isAIControlled))
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
  local controls = ac.overrideCarControls(st.carIndex)
  if not controls then
    st.active = false
    st.verdict = "overrideCarControls вернул nil — перехват недоступен"
    log("SELFTEST: overrideCarControls(%d) -> nil", st.carIndex)
    logFlush()
    return
  end
  controls.combineAxis = false
  controls.gas = 0
  controls.brake = 1
  controls.steer = 0.8
  st.appliedFrames = st.appliedFrames + 1
  local brakeSeen = car.brake or 0
  local steerSeen = car.steer or 0
  if brakeSeen > 0.5 then st.brakeSeen = st.brakeSeen + 1 end
  if math.abs(steerSeen) > 0.3 then st.steerSeen = st.steerSeen + 1 end
  if st.appliedFrames % 10 == 1 then
    log("SELFTEST car#%d: cmd(steer 0.80 brake 1.00) | car reports steer %.2f brake %.2f | %.1f km/h | active=%s | ctrl8=%s",
      st.carIndex, steerSeen, brakeSeen, car.speedKmh, tostring(controls:active()),
      tostring(car.isAIControlled))
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
    local c2 = ac.overrideCarControls(st.carIndex)
    if c2 then
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
      log("CAR#%d: active=%s aiControlled=%s speed=%.1f name=%s",
        i, tostring(car.isActive), tostring(car.isAIControlled), car.speedKmh, name)
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
    if path then
      log("log file: %s", path)
    else
      log("LOG ERROR: file could not be created: %s", tostring(state.log.writeError))
      ac.error("[RoutePilot] log file could not be created: " .. tostring(state.log.writeError))
    end
    log("Always-on mode: включи Ctrl+8 — машина перехватится автоматически, кнопки не нужны")
    logFlush()
  end

  local sim = ac.getSim()
  if sim then scanSession(sim) end
  if not sim or not sim.isLive then return end

  updateSelfTest(dt)
  updateControl(dt)

  state.heartbeatTimer = state.heartbeatTimer + dt
  if state.heartbeatTimer >= HEARTBEAT_INTERVAL then
    state.heartbeatTimer = 0
    state.log:heartbeat(string.format(
      "caught=%d playerCtrl8=%s physActive=%d slowing=%d errors=%d",
      state.caught, tostring(state.playerCaught), state.activePhys,
      state.slowingCount, state.updateErrors))
    for carIndex, p in pairs(state.p) do
      local car = ac.getCar(carIndex)
      if car then
        log("CAR#%d: cmd steer %.2f gas %.2f brake %.2f hb %.1f | car reports steer %.2f gas %.2f brake %.2f | %.1f km/h | active=%s",
          carIndex, p.steer, p.gas, p.brake, p.hb,
          car.steer or -1, car.gas or -1, car.brake or -1,
          car.speedKmh, tostring(p.active))
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
  for i in pairs(state.p) do
    local controls = ac.overrideCarControls(i)
    if controls then
      controls.combineAxis = false
      controls.steer = 0
      controls.gas = 0
      controls.brake = 0
      controls.handbrake = 0
    end
  end
  state.p = {}
  if state.log then
    state.log:shutdown('script released (session end or game exit)')
  end
end

function script.windowMain(dt)
  ui.text("RoutePilot  " .. APP_VERSION .. "  (always-on)")
  ui.text("Перехвачено машин: " .. tostring(state.caught)
    .. "   Физика приняла: " .. tostring(state.activePhys))
  if state.playerCaught then
    ui.textColored("Ctrl+8 ВКЛ — твоя машина ведётся RoutePilot", rgbm(0.3, 1, 0.5, 1))
  else
    ui.textColored("Ctrl+8 выкл — включи, и машина перехватится сама", rgbm(1, 0.8, 0.2, 1))
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
  ui.textWrapped("Кнопок включения нет: как FullThrottleAI, приложение всегда активно и берёт под контроль любую машину ИИ — свою (после Ctrl+8) и AI-соперников.")
  ui.text("Лог: " .. (state.log and state.log.path or "создаётся…"))
end
