-- AC DriftLab 0.1.0
-- Experimental AI input and telemetry recorder for CSP Lua apps.
-- Controls only steering, throttle and brake. Gearbox remains under AI control.

local APP = "AC DriftLab"
local VERSION = "0.1.0"
local selectedCar = 0
local aiWasActive = false
local running, status, lastError = false, "Ожидание", ""
local elapsed, sampleAccumulator, flushAccumulator = 0, 0, 0
local phaseTime, phaseDuration, phase, phaseNumber = 0, 0, "IDLE", 0
local commandSteer, commandGas, commandBrake = 0, 0, 0
local targetSteer, targetGas, targetBrake = 0, 0, 0
local experimentID, outputDir, csvPath, eventPath = "", "", "", ""
local csvRows, eventRows, sampleCount = {}, {}, 0
local lastEvent, override = "", nil
local emergencyUntil, noSteerResponseTime = 0, 0
local previousSteer, previousYaw = 0, 0
local SAMPLE_INTERVAL, FLUSH_INTERVAL = 0.10, 1.0
local MAX_SPEED_KMH, MAX_YAW_RATE, MAX_STEER = 70, 2.5, 0.55

local function clamp(x, lo, hi)
  if x < lo then return lo end
  if x > hi then return hi end
  return x
end

local function approach(current, target, maxDelta)
  if target > current then return math.min(current + maxDelta, target) end
  return math.max(current - maxDelta, target)
end

local function safeNumber(v)
  if type(v) ~= "number" or v ~= v or v == math.huge or v == -math.huge then return 0 end
  return v
end

local function timestamp()
  local ok, value = pcall(function() return os.date("!%Y-%m-%dT%H:%M:%SZ") end)
  if ok and value then return value end
  local ok2, value2 = pcall(function() return tostring(os.time()) end)
  if ok2 and value2 then return value2 end
  return "unknown"
end

local function csvValue(v)
  if v == nil then return "" end
  if type(v) == "string" then return '"' .. v:gsub('"', '""') .. '"' end
  return tostring(v)
end

local function joinCSV(values)
  local out = {}
  for i = 1, #values do out[i] = csvValue(values[i]) end
  return table.concat(out, ",")
end

local function saveEvents()
  if eventPath == "" or not io.available() then return end
  local header = "utc,experiment_id,elapsed_s,type,phase,message\n"
  local ok, err = pcall(function() return io.save(eventPath, header .. table.concat(eventRows, "\n") .. "\n") end)
  if not ok then lastError = tostring(err) end
end

local function logEvent(kind, message)
  eventRows[#eventRows + 1] = joinCSV({timestamp(), experimentID, string.format("%.3f", elapsed), kind, phase, message})
  lastEvent = kind .. ": " .. message
  saveEvents()
end

local function flushCSV()
  if csvPath == "" or not io.available() then return end
  local header = "utc,experiment_id,elapsed_s,car_index,car_id,phase,phase_number,phase_time_s,command_steer,command_gas,command_brake,actual_steer_deg,actual_gas,actual_brake,speed_kmh,local_vx,local_vy,local_vz,yaw_rate_x,yaw_rate_y,yaw_rate_z,slip_angle_est_deg,rpm,gear,fl_slip_angle,fr_slip_angle,rl_slip_angle,rr_slip_angle,fl_slip_ratio,fr_slip_ratio,rl_slip_ratio,rr_slip_ratio\n"
  local ok, err = pcall(function() return io.save(csvPath, header .. table.concat(csvRows, "\n") .. "\n") end)
  if not ok then lastError, status = tostring(err), "Ошибка записи лога" end
end

local function newPhase()
  phaseNumber, phaseTime = phaseNumber + 1, 0
  local pick = math.random(1, 4)
  if pick == 1 then
    phase, phaseDuration = "STEER_GAS", 1.3 + math.random() * 1.8
    targetSteer, targetGas, targetBrake = (math.random() * 2 - 1) * MAX_STEER, 0.30 + math.random() * 0.38, 0
  elseif pick == 2 then
    phase, phaseDuration = "COAST_STEER", 0.8 + math.random() * 1.4
    targetSteer, targetGas, targetBrake = (math.random() * 2 - 1) * MAX_STEER, 0.03 + math.random() * 0.12, 0
  elseif pick == 3 then
    phase, phaseDuration = "BRAKE_PULSE", 0.18 + math.random() * 0.22
    targetSteer, targetGas, targetBrake = (math.random() * 2 - 1) * 0.25, 0, 0.18 + math.random() * 0.20
  else
    phase, phaseDuration = "STRAIGHT_RECOVERY", 0.8 + math.random() * 1.2
    targetSteer, targetGas, targetBrake = (math.random() * 2 - 1) * 0.12, 0.22 + math.random() * 0.18, 0
  end
  logEvent("PHASE", string.format("phase=%s;duration=%.2f;target_steer=%.3f;target_gas=%.3f;target_brake=%.3f",
    phase, phaseDuration, targetSteer, targetGas, targetBrake))
end

local function stopExperiment(reason)
  if not running then return end
  running, status = false, "Остановлено"
  commandSteer, commandGas, commandBrake = 0, 0, 0
  targetSteer, targetGas, targetBrake = 0, 0, 0
  if override then
    pcall(function()
      override.combineAxis = false
      override.combineAxis = true
      override.steer = math.huge
      override.gas, override.brake, override.handbrake = 0, 0, 0
    end)
  end
  logEvent("STOP", reason or "user")
  flushCSV()
  logEvent("SAVED", string.format("samples=%d;telemetry=%s;events=%s", sampleCount, csvPath, eventPath))
end

local function startExperiment()
  if not io.available() then
    status, lastError = "Нет доступа к файловой системе CSP", "Включи I/O access для приложения AC DriftLab"
    return
  end
  local car = ac.getCar(selectedCar)
  if not car then status = "Автомобиль не найден"; return end
  if not car.isAIControlled then status = "Выбранная машина не управляется AI"; return end
  local ok, result = pcall(function() return ac.overrideCarControls(selectedCar) end)
  if not ok or not result then status, lastError = "Не удалось получить управление", tostring(result); return end
  override = result
  override.combineAxis = false
  experimentID = timestamp():gsub("[^%w]", "") .. "_" .. tostring(math.random(1000, 9999))
  outputDir = ac.getFolder(ac.FolderID.ScriptConfig) .. "\\AC_DriftLab"
  io.createDir(outputDir)
  csvPath, eventPath = outputDir .. "\\" .. experimentID .. "_telemetry.csv", outputDir .. "\\" .. experimentID .. "_events.csv"
  csvRows, eventRows = {}, {}
  elapsed, phaseTime, sampleAccumulator, flushAccumulator = 0, 0, 0, 0
  sampleCount, phaseNumber = 0, 0
  commandSteer, commandGas, commandBrake = 0, 0, 0
  targetSteer, targetGas, targetBrake = 0, 0, 0
  emergencyUntil, noSteerResponseTime = 0, 0
  previousSteer, previousYaw = safeNumber(car.steer), safeNumber(car.localAngularVelocity.y)
  running, status = true, "Эксперимент идёт"
  logEvent("START", string.format("car=%d;id=%s;track=%s;layout=%s", selectedCar, tostring(ac.getCarID(selectedCar)), ac.getTrackID(), ac.getTrackLayout()))
  newPhase()
end

local function wheelField(car, index, key)
  local w = car.wheels and car.wheels[index]
  return w and safeNumber(w[key]) or 0
end

local function sampleCar(car)
  local lv, av = car.localVelocity, car.localAngularVelocity
  local vx, vy, vz = safeNumber(lv.x), safeNumber(lv.y), safeNumber(lv.z)
  local yawX, yawY, yawZ = safeNumber(av.x), safeNumber(av.y), safeNumber(av.z)
  local slipEstimate = math.deg(math.atan2(vx, math.max(math.abs(vz), 0.01)))
  csvRows[#csvRows + 1] = joinCSV({
    timestamp(), experimentID, string.format("%.3f", elapsed), selectedCar, ac.getCarID(selectedCar) or "",
    phase, phaseNumber, string.format("%.3f", phaseTime),
    string.format("%.4f", commandSteer), string.format("%.4f", commandGas), string.format("%.4f", commandBrake),
    string.format("%.3f", safeNumber(car.steer)), string.format("%.4f", safeNumber(car.gas)), string.format("%.4f", safeNumber(car.brake)),
    string.format("%.3f", safeNumber(car.speedKmh)), string.format("%.4f", vx), string.format("%.4f", vy), string.format("%.4f", vz),
    string.format("%.4f", yawX), string.format("%.4f", yawY), string.format("%.4f", yawZ), string.format("%.3f", slipEstimate),
    string.format("%.0f", safeNumber(car.rpm)), safeNumber(car.gear),
    wheelField(car, 0, "slipAngle"), wheelField(car, 1, "slipAngle"), wheelField(car, 2, "slipAngle"), wheelField(car, 3, "slipAngle"),
    wheelField(car, 0, "slipRatio"), wheelField(car, 1, "slipRatio"), wheelField(car, 2, "slipRatio"), wheelField(car, 3, "slipRatio")
  })
  sampleCount = sampleCount + 1
end

local function updateExperiment(dt)
  if not running then return end
  local ok, err = pcall(function()
    local car = ac.getCar(selectedCar)
    if not car or not car.isActive then stopExperiment("car_missing_or_inactive"); return end
    if not car.isAIControlled then stopExperiment("selected_car_is_no_longer_AI"); return end
    if not override then stopExperiment("control_override_missing"); return end

    elapsed, phaseTime = elapsed + dt, phaseTime + dt
    local speed, yawRate = safeNumber(car.speedKmh), math.abs(safeNumber(car.localAngularVelocity.y))
    if speed > MAX_SPEED_KMH or yawRate > MAX_YAW_RATE then
      if elapsed >= emergencyUntil then logEvent("SAFETY", "emergency_limit") end
      emergencyUntil = math.max(emergencyUntil, elapsed + 1.0)
    end

    if elapsed < emergencyUntil then
      targetSteer, targetGas, targetBrake, phase = 0, 0, 0.28, "SAFETY_RECOVERY"
    elseif phase == "SAFETY_RECOVERY" or phaseTime >= phaseDuration then
      newPhase()
    end

    commandSteer = approach(commandSteer, targetSteer, 0.75 * dt)
    commandGas = approach(commandGas, targetGas, 0.65 * dt)
    commandBrake = approach(commandBrake, targetBrake, 1.5 * dt)
    override.combineAxis = false
    override.steer = clamp(commandSteer, -MAX_STEER, MAX_STEER)
    override.gas = clamp(commandGas, 0, 0.72)
    override.brake = clamp(commandBrake, 0, 0.40)
    override.handbrake = 0

    if math.abs(safeNumber(car.steer) - previousSteer) < 0.05 and math.abs(commandSteer) > 0.25 and speed > 8 then
      noSteerResponseTime = noSteerResponseTime + dt
    else
      noSteerResponseTime = math.max(0, noSteerResponseTime - dt * 2)
    end
    if noSteerResponseTime > 4 then
      logEvent("WARNING", "steering_response_not_confirmed")
      noSteerResponseTime = 0
    end
    previousSteer, previousYaw = safeNumber(car.steer), safeNumber(car.localAngularVelocity.y)

    sampleAccumulator = sampleAccumulator + dt
    if sampleAccumulator >= SAMPLE_INTERVAL then sampleAccumulator = sampleAccumulator - SAMPLE_INTERVAL; sampleCar(car) end
    flushAccumulator = flushAccumulator + dt
    if flushAccumulator >= FLUSH_INTERVAL then flushAccumulator = 0; flushCSV() end
  end)
  if not ok then
    lastError = tostring(err)
    logEvent("LUA_ERROR", lastError)
    stopExperiment("lua_error")
  end
end

function script.update(dt)
  local car = ac.getCar(selectedCar)
  local aiNow = car ~= nil and car.isAIControlled == true
  if aiNow and not aiWasActive and not running then
    logEvent("AI_DETECTED", "AI autopilot detected; starting experiment")
    startExperiment()
  elseif not aiNow and aiWasActive and running then
    stopExperiment("AI_autopilot_disabled")
  end
  aiWasActive = aiNow
  updateExperiment(math.min(safeNumber(dt), 0.1))
end

function script.windowMain(dt)
    ui.text("AC DriftLab " .. VERSION .. " — экспериментальное управление AI")
    ui.text("Выбери машину. Для машины 0 запуск происходит автоматически при включении AI-автопилота.")
    ui.separator()
    selectedCar = ui.slider("Индекс AI-машины", selectedCar, 0, 31, "%.0f", true)
    local car = ac.getCar(selectedCar)
    if car then
      ui.text("Машина: " .. tostring(ac.getCarName(selectedCar) or ac.getCarID(selectedCar) or "unknown"))
      ui.text(string.format("AI: %s | Скорость: %.1f км/ч | Передача: %d", tostring(car.isAIControlled), safeNumber(car.speedKmh), safeNumber(car.gear)))
    else
      ui.text("Машина с таким индексом не найдена")
    end
    ui.separator()
    if running then
      if ui.button("ОСТАНОВИТЬ", vec2(-1, 34)) then stopExperiment("user_stop") end
    else
      if ui.button("НАЧАТЬ ЭКСПЕРИМЕНТ", vec2(-1, 34)) then startExperiment() end
    end
    ui.text("Статус: " .. status)
    ui.text(string.format("Фаза: %s | Время: %.1f с | Сэмплов: %d", phase, elapsed, sampleCount))
    ui.text(string.format("Команды: steer %.2f | gas %.2f | brake %.2f", commandSteer, commandGas, commandBrake))
    ui.text(string.format("Ограничения: скорость %d км/ч, yaw %.1f рад/с", MAX_SPEED_KMH, MAX_YAW_RATE))
    if lastEvent ~= "" then ui.textWrapped("Последнее событие: " .. lastEvent) end
    if lastError ~= "" then ui.textWrapped("Ошибка: " .. lastError) end
    if csvPath ~= "" then
      ui.separator()
      ui.textWrapped("Логи: " .. outputDir)
      ui.textWrapped(experimentID .. "_telemetry.csv")
      ui.textWrapped(experimentID .. "_events.csv")
    end
end

ac.onRelease(function()
  if running then stopExperiment("script_unload") end
end)
