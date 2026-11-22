-- AC DriftLab 0.2.0
-- CSP Custom AI controller + experiment recorder.
-- Uses CSP's native named shared-memory API. No AI fastline is read or used.
-- Gear changes remain automatic: no gear_up/gear_dn commands are ever sent.

local APP_VERSION = "0.2.0"
local MAX_CARS = 31
local CTRL_BYTES = 256
local DATA_BYTES = 128
local SAMPLE_INTERVAL = 0.10
local LOG_FLUSH_INTERVAL = 5.0
local EVENT_CHUNK_SIZE = 20

local selectedCar = 0
local autoStart = true
local running = false
local status = "Ожидание AI Autopilot или ручного запуска"
local lastError = ""
local lastEvent = ""
local experimentID = ""
local outputDir = ""
local elapsed = 0
local sampleAccumulator = 0
local flushAccumulator = 0
local sampleCount = 0
local phase = "IDLE"
local phaseNumber = 0
local phaseTime = 0
local phaseDuration = 0
local emergencyTime = 0
local noSteerResponseTime = 0
local lastPacket = -1
local dataRetry = 0
local telemetryChunk = 1
local eventChunk = 1
local telemetryRows = {}
local eventRows = {}
local controlMap = nil
local controlBytes = nil
local dataMap = nil
local dataBytes = nil
local commandSteer, commandGas, commandBrake = 0, 0, 0
local targetSteer, targetGas, targetBrake = 0, 0, 0
local maxSteer, maxGas, maxBrake = 0.58, 0.62, 0.38
local maxSpeedKmh, maxYawRate = 75, 3.0
local rngSeeded = false

local CTRL_NAME_PREFIX = "AcTools.CSP.NewBehaviour.CustomAI.CarControls"
local DATA_NAME_PREFIX = "AcTools.CSP.NewBehaviour.CustomAI.Car"

local function clamp(v, lo, hi)
  if v < lo then return lo end
  if v > hi then return hi end
  return v
end

local function finite(v)
  return type(v) == "number" and v == v and v ~= math.huge and v ~= -math.huge
end

local function number(v)
  if finite(v) then return v end
  return 0
end

local function approach(current, target, delta)
  if target > current then return math.min(current + delta, target) end
  return math.max(current - delta, target)
end

local function atan2(y, x)
  if math.atan2 then return math.atan2(y, x) end
  if x > 0 then return math.atan(y / x) end
  if x < 0 and y >= 0 then return math.atan(y / x) + math.pi end
  if x < 0 and y < 0 then return math.atan(y / x) - math.pi end
  if y > 0 then return math.pi * 0.5 end
  if y < 0 then return -math.pi * 0.5 end
  return 0
end

local function timestamp()
  local ok, value = pcall(function() return os.date("%Y-%m-%dT%H:%M:%S") end)
  if ok and value then return value end
  return "time-unavailable"
end

local function csv(v)
  if v == nil then return "" end
  if type(v) == "string" then return '"' .. v:gsub('"', '""') .. '"' end
  return tostring(v)
end

local function csvRow(values)
  local out = {}
  for i = 1, #values do out[i] = csv(values[i]) end
  return table.concat(out, ",")
end

local function safeSave(path, content)
  local ok, err = pcall(function() io.save(path, content) end)
  if not ok then
    lastError = tostring(err)
    status = "Ошибка записи логов"
    return false
  end
  return true
end

local function flushTelemetry()
  if #telemetryRows == 0 or outputDir == "" then return end
  local header = "utc,experiment_id,elapsed_s,car_index,packet_id,phase,phase_number,phase_time_s,command_steer,command_gas,command_brake,actual_steer_deg,actual_gas,actual_brake,speed_kmh,gear,rpm,velocity_x,velocity_y,velocity_z,local_vx,local_vy,local_vz,yaw_rate_x,yaw_rate_y,yaw_rate_z,slip_angle_est_deg,acc_g_x,acc_g_y,acc_g_z,look_x,look_y,look_z\n"
  local path = string.format("%s\\telemetry_%04d.csv", outputDir, telemetryChunk)
  if safeSave(path, header .. table.concat(telemetryRows, "\n") .. "\n") then
    telemetryRows = {}
    telemetryChunk = telemetryChunk + 1
  end
end

local function flushEvents()
  if #eventRows == 0 or outputDir == "" then return end
  local header = "utc,experiment_id,elapsed_s,event,phase,message\n"
  local path = string.format("%s\\events_%04d.csv", outputDir, eventChunk)
  if safeSave(path, header .. table.concat(eventRows, "\n") .. "\n") then
    eventRows = {}
    eventChunk = eventChunk + 1
  end
end

local function event(kind, message)
  local row = csvRow({timestamp(), experimentID, string.format("%.3f", elapsed), kind, phase, message})
  eventRows[#eventRows + 1] = row
  lastEvent = kind .. ": " .. message
  if #eventRows >= EVENT_CHUNK_SIZE then flushEvents() end
end

local function writeFloat(offset, value)
  ffi.cast("float*", controlBytes + offset)[0] = value
end

local function writeByte(offset, value)
  controlBytes[offset] = value and 1 or 0
end

local function writeControls(steer, gas, brake)
  if not controlBytes then return false end
  writeFloat(0, clamp(gas, 0, 1))
  writeFloat(4, clamp(brake, 0, 1))
  writeFloat(8, 0) -- Autoclutch owns clutch operation.
  writeFloat(12, clamp(steer, -1, 1))
  writeFloat(16, 0) -- Handbrake is never used in the first experiment.
  -- No gear_up/gear_dn writes: automatic gearbox remains responsible for shifting.
  writeByte(20, false)
  writeByte(21, false)
  writeByte(40, false) -- teleport_to = 0
  writeByte(41, true)  -- autoclutch_on_start
  writeByte(42, true)  -- autoclutch_on_change
  writeByte(43, true)  -- autoblip_active
  writeByte(68, true)  -- autoshift_active
  return true
end

local function releaseMaps()
  if dataMap then
    pcall(ac.disposeMemoryMappedFile, dataMap)
    dataMap, dataBytes = nil, nil
  end
  if controlMap then
    pcall(ac.disposeMemoryMappedFile, controlMap)
    controlMap, controlBytes = nil, nil
  end
end

local function createOutputFolder()
  local ok, base = pcall(function() return ac.getFolder(ac.FolderID.ScriptConfig) end)
  if not ok or type(base) ~= "string" or base == "" then
    error("Не удалось получить ScriptConfig через CSP")
  end
  outputDir = base .. "\\AC_DriftLab\\" .. experimentID
  local made, err = pcall(function() io.createDir(outputDir) end)
  if not made then error("Не удалось создать папку логов: " .. tostring(err)) end
end

local function connectController()
  local controlName = CTRL_NAME_PREFIX .. tostring(selectedCar) .. ".v0"
  local dataName = DATA_NAME_PREFIX .. tostring(selectedCar) .. ".v0"

  local ok, result = pcall(function()
    return ac.writeMemoryMappedFile(controlName, CTRL_BYTES)
  end)
  if not ok or not result then
    error("CSP не разрешил создать Custom AI memory map: " .. tostring(result))
  end

  controlMap = result
  controlBytes = ffi.cast("uint8_t*", controlMap)
  for i = 0, CTRL_BYTES - 1 do controlBytes[i] = 0 end
  writeControls(0, 0, 0)

  local opened, readResult = pcall(function()
    return ac.readMemoryMappedFile(dataName, DATA_BYTES)
  end)
  if opened and readResult then
    dataMap = readResult
    dataBytes = ffi.cast("uint8_t*", dataMap)
    status = "Custom AI подключён"
  else
    status = "Контроллер создан; ожидаю подтверждение CSP"
  end
end

local function beginExperiment()
  if running then return end
  local car = ac.getCar(selectedCar)
  if not car or not car.isActive then
    status = "Выбранная машина неактивна"
    return
  end

  if not rngSeeded then
    math.randomseed(os.time and os.time() or 1731)
    rngSeeded = true
  end

  experimentID = timestamp():gsub("[^%w]", "") .. "_" .. tostring(math.random(1000, 9999))
  elapsed, sampleAccumulator, flushAccumulator = 0, 0, 0
  sampleCount, phaseNumber, phaseTime, emergencyTime = 0, 0, 0, 0
  telemetryChunk, eventChunk = 1, 1
  telemetryRows, eventRows = {}, {}
  commandSteer, commandGas, commandBrake = 0, 0, 0
  targetSteer, targetGas, targetBrake = 0, 0, 0
  lastPacket, dataRetry, noSteerResponseTime = -1, 0, 0

  local ok, err = pcall(function()
    createOutputFolder()
    connectController()
  end)
  if not ok then
    lastError = tostring(err)
    status = "Не удалось подключить Custom AI"
    releaseMaps()
    return
  end

  running = true
  event("START", string.format("car=%d;car_id=%s;track=%s;layout=%s;ai=%s",
    selectedCar, tostring(ac.getCarID(selectedCar)), tostring(ac.getTrackID()),
    tostring(ac.getTrackLayout()), tostring(car.isAIControlled)))
  chooseNextPhase()
end

function chooseNextPhase()
  phaseNumber = phaseNumber + 1
  phaseTime = 0
  local pick = math.random(1, 5)
  if pick == 1 then
    phase, phaseDuration = "POWER_STEER_LEFT", 1.4 + math.random() * 1.4
    targetSteer, targetGas, targetBrake = -(0.22 + math.random() * 0.30), 0.35 + math.random() * 0.27, 0
  elseif pick == 2 then
    phase, phaseDuration = "POWER_STEER_RIGHT", 1.4 + math.random() * 1.4
    targetSteer, targetGas, targetBrake = 0.22 + math.random() * 0.30, 0.35 + math.random() * 0.27, 0
  elseif pick == 3 then
    phase, phaseDuration = "COAST_STEER", 0.7 + math.random() * 1.2
    targetSteer, targetGas, targetBrake = (math.random() * 2 - 1) * 0.48, 0.03 + math.random() * 0.10, 0
  elseif pick == 4 then
    phase, phaseDuration = "BRAKE_PULSE", 0.16 + math.random() * 0.22
    targetSteer, targetGas, targetBrake = (math.random() * 2 - 1) * 0.22, 0, 0.18 + math.random() * 0.18
  else
    phase, phaseDuration = "RECOVER_AND_ACCELERATE", 0.8 + math.random() * 1.0
    targetSteer, targetGas, targetBrake = (math.random() * 2 - 1) * 0.15, 0.24 + math.random() * 0.22, 0
  end
  event("PHASE", string.format("%s;duration=%.2f;steer=%.3f;gas=%.3f;brake=%.3f",
    phase, phaseDuration, targetSteer, targetGas, targetBrake))
end

local function stopExperiment(reason)
  if not running and not controlMap then return end
  running = false
  if controlBytes then
    writeControls(0, 0, 0)
  end
  event("STOP", reason or "user")
  flushTelemetry()
  flushEvents()
  releaseMaps()
  status = "Остановлено; управление освобождено"
end

local function readFloat(offset)
  return number(ffi.cast("float*", dataBytes + offset)[0])
end

local function readInt(offset)
  return tonumber(ffi.cast("int32_t*", dataBytes + offset)[0]) or 0
end

local function readVec3(offset)
  local p = ffi.cast("float*", dataBytes + offset)
  return number(p[0]), number(p[1]), number(p[2])
end

local function sampleTelemetry()
  if not dataBytes then return end
  local packet = readInt(0)
  if packet == lastPacket then return end
  lastPacket = packet

  local vx, vy, vz = readVec3(40)
  local ax, ay, az = readVec3(52)
  local lx, ly, lz = readVec3(64)
  local px, py, pz = readVec3(88)
  local lvx, lvy, lvz = readVec3(100)
  local avx, avy, avz = readVec3(112)
  local slipAngle = math.deg(atan2(lvx, math.max(math.abs(lvz), 0.01)))

  telemetryRows[#telemetryRows + 1] = csvRow({
    timestamp(), experimentID, string.format("%.3f", elapsed), selectedCar, packet,
    phase, phaseNumber, string.format("%.3f", phaseTime),
    string.format("%.4f", commandSteer), string.format("%.4f", commandGas), string.format("%.4f", commandBrake),
    string.format("%.3f", readFloat(16)), string.format("%.4f", readFloat(4)), string.format("%.4f", readFloat(8)),
    string.format("%.3f", readFloat(36)), readInt(28), string.format("%.0f", readFloat(32)),
    string.format("%.4f", vx), string.format("%.4f", vy), string.format("%.4f", vz),
    string.format("%.4f", lvx), string.format("%.4f", lvy), string.format("%.4f", lvz),
    string.format("%.4f", avx), string.format("%.4f", avy), string.format("%.4f", avz),
    string.format("%.3f", slipAngle),
    string.format("%.4f", ax), string.format("%.4f", ay), string.format("%.4f", az),
    string.format("%.4f", lx), string.format("%.4f", ly), string.format("%.4f", lz)
  })
  sampleCount = sampleCount + 1
end

local function updateDataConnection(dt)
  if dataBytes then return true end
  dataRetry = dataRetry + dt
  if dataRetry < 0.5 then return false end
  dataRetry = 0
  local ok, result = pcall(function()
    return ac.readMemoryMappedFile(DATA_NAME_PREFIX .. tostring(selectedCar) .. ".v0", DATA_BYTES)
  end)
  if ok and result then
    dataMap = result
    dataBytes = ffi.cast("uint8_t*", dataMap)
    status = "CSP Custom AI подтверждён; сбор данных"
    event("CONNECTED", "CSP created readback memory map")
    return true
  end
  status = "Ожидаю CSP Custom AI; проверь разрешение трассы и new_behaviour.ini"
  return false
end

local function updateExperiment(dt)
  if not running or not controlBytes then return end
  elapsed = elapsed + dt
  if not updateDataConnection(dt) then
    -- Keep a neutral command until CSP confirms the controller takeover.
    writeControls(0, 0, 0)
    return
  end

  phaseTime = phaseTime + dt
  local speed = readFloat(36)
  local yawRate = math.abs(readFloat(116))

  if speed > maxSpeedKmh or yawRate > maxYawRate then
    if emergencyTime <= elapsed then
      event("SAFETY_LIMIT", string.format("speed=%.1f;yaw_rate=%.3f", speed, yawRate))
    end
    emergencyTime = math.max(emergencyTime, elapsed + 0.9)
  end

  if elapsed < emergencyTime then
    phase = "SAFETY_RECOVERY"
    targetSteer, targetGas, targetBrake = 0, 0, 0.24
  elseif phase == "SAFETY_RECOVERY" or phaseTime >= phaseDuration then
    chooseNextPhase()
  end

  commandSteer = approach(commandSteer, targetSteer, 0.72 * dt)
  commandGas = approach(commandGas, targetGas, 0.58 * dt)
  commandBrake = approach(commandBrake, targetBrake, 1.35 * dt)
  commandSteer = clamp(commandSteer, -maxSteer, maxSteer)
  commandGas = clamp(commandGas, 0, maxGas)
  commandBrake = clamp(commandBrake, 0, maxBrake)
  writeControls(commandSteer, commandGas, commandBrake)

  local localVx, _, localVz = readVec3(100)
  local response = math.abs(readFloat(116))
  if math.abs(commandSteer) > 0.30 and speed > 8 and math.abs(localVx) < 0.12 and response < 0.025 then
    noSteerResponseTime = noSteerResponseTime + dt
  else
    noSteerResponseTime = math.max(0, noSteerResponseTime - dt * 2)
  end
  if noSteerResponseTime > 4 then
    event("WARNING", "steering_response_not_detected")
    noSteerResponseTime = 0
  end

  sampleAccumulator = sampleAccumulator + dt
  if sampleAccumulator >= SAMPLE_INTERVAL then
    sampleAccumulator = sampleAccumulator - SAMPLE_INTERVAL
    sampleTelemetry()
  end

  flushAccumulator = flushAccumulator + dt
  if flushAccumulator >= LOG_FLUSH_INTERVAL then
    flushAccumulator = 0
    flushTelemetry()
    flushEvents()
  end
end

function script.update(dt)
  local delta = clamp(number(dt), 0, 0.1)
  if autoStart and not running and not controlMap then
    local car = ac.getCar(selectedCar)
    if car and car.isActive and car.isAIControlled then
      beginExperiment()
    end
  end

  local ok, err = pcall(updateExperiment, delta)
  if not ok then
    lastError = tostring(err)
    event("LUA_ERROR", lastError)
    stopExperiment("lua_error")
  end
end

function script.windowMain(dt)
  ui.text("AC DriftLab " .. APP_VERSION .. " — CSP Custom AI")
  ui.text("Управление рулём, газом и тормозом. КПП остаётся автоматической.")
  ui.separator()

  selectedCar = ui.slider("Индекс машины", selectedCar, 0, MAX_CARS, "%.0f", true)
  autoStart = ui.checkbox("Автозапуск при включении AI Autopilot", autoStart)

  local car = ac.getCar(selectedCar)
  if car then
    ui.text("Автомобиль: " .. tostring(ac.getCarName(selectedCar) or ac.getCarID(selectedCar) or "unknown"))
    ui.text(string.format("AI Autopilot: %s | Скорость AC: %.1f км/ч | Передача: %d",
      tostring(car.isAIControlled), number(car.speedKmh), number(car.gear)))
  else
    ui.text("Машина с таким индексом сейчас недоступна")
  end

  ui.separator()
  if running then
    if ui.button("ОСТАНОВИТЬ И ОСВОБОДИТЬ УПРАВЛЕНИЕ", vec2(-1, 34)) then
      stopExperiment("user_stop")
    end
  else
    if ui.button("НАЧАТЬ CUSTOM AI ЭКСПЕРИМЕНТ", vec2(-1, 34)) then
      beginExperiment()
    end
  end

  ui.text("Статус: " .. status)
  ui.text(string.format("Фаза: %s | Время: %.1f с | Сэмплов: %d", phase, elapsed, sampleCount))
  ui.text(string.format("Команды: steer %.2f | gas %.2f | brake %.2f",
    commandSteer, commandGas, commandBrake))
  ui.text(string.format("Пределы: %.0f км/ч | yaw %.1f рад/с", maxSpeedKmh, maxYawRate))
  if lastEvent ~= "" then ui.textWrapped("Событие: " .. lastEvent) end
  if lastError ~= "" then ui.textWrapped("Ошибка: " .. lastError) end
  if outputDir ~= "" then ui.textWrapped("Логи: " .. outputDir) end
  ui.separator()
  ui.textWrapped("Для Custom AI необходимы ENABLED=1 в [CUSTOM_AI] файла new_behaviour.ini и ALLOW_CUSTOM_AI_MANIPULATION=1 в surfaces.ini трассы. Fastline не используется.")
end

ac.onRelease(function()
  if running or controlMap then
    stopExperiment("app_unload")
  end
end)
