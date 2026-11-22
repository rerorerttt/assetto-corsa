-- RoutePilot: route following + anti-donut traction and spin recovery

local okLogger, Logger = pcall(require, "logger")
if not okLogger or type(Logger) ~= "table" then
  Logger = nil
end

local BRAKE_ACCEL = 7.0
local FOLLOW_LATERAL = 3.0
local WALL_MARGIN = 6.0

-- Anti-donut tuning
local SLIP_ANGLE_START = 16.0
local SLIP_ANGLE_FULL = 34.0
local SLIP_SPEED_MIN = 3.0
local LATERAL_SLIP_START = 2.2
local LATERAL_SLIP_FULL = 6.0
local YAW_RECOVERY_START = 0.75
local YAW_RECOVERY_FULL = 1.8
local REAR_SLIP_START = 1.05
local REAR_SLIP_FULL = 1.65
local RECOVERY_RELEASE_TIME = 0.75
local GAS_RISE_RATE = 0.42
local GAS_FALL_RATE = 4.5

local state = {
  enabled = true,
  recording = false,
  loopRoute = true,
  speedMin = 60,
  speedMax = 140,
  followGap = 8.0,
  p = {},
  nodes = {},
  telemetry = {},
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

local function smoothstep(a, b, x)
  if b <= a then return 0 end
  local t = clamp((x - a) / (b - a), 0, 1)
  return t * t * (3 - 2 * t)
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

local TELEMETRY_HEADER = "ROUTEPILOT_LINE_V2"

local function serializeNodes(nodes, telemetry)
  local lines = { TELEMETRY_HEADER }
  for i = 1, #nodes do
    local n, t = nodes[i], telemetry and telemetry[i] or nil
    if t then
      table.insert(lines, string.format(
        "%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.0f %.1f %.3f %.3f %.4f %.4f",
        n.x, n.y, n.z, t.speed or 0, t.steer or 0, t.gas or 0, t.brake or 0,
        t.clutch or 0, t.handbrake or 0, t.gear or 0, t.rpm or 0,
        t.slipAngle or 0, t.lateralSpeed or 0, t.yawRate or 0, t.rearSlip or 0
      ))
    else
      table.insert(lines, string.format("%.4f %.4f %.4f", n.x, n.y, n.z))
    end
  end
  return table.concat(lines, "\n")
end

local function parseNodes(raw)
  if type(raw) ~= "string" or raw == "" then return nil end
  local loaded, telemetry, isV2 = {}, {}, false
  for line in string.gmatch(raw, "[^\r\n]+") do
    if line == TELEMETRY_HEADER then
      isV2 = true
    else
      local v = {}
      for token in string.gmatch(line, "%S+") do v[#v + 1] = tonumber(token) end
      if #v >= 3 and v[1] and v[2] and v[3] then
        loaded[#loaded + 1] = vec3(v[1], v[2], v[3])
        if isV2 and #v >= 15 then
          telemetry[#loaded] = {
            speed=v[4], steer=v[5], gas=v[6], brake=v[7], clutch=v[8],
            handbrake=v[9], gear=v[10], rpm=v[11], slipAngle=v[12],
            lateralSpeed=v[13], yawRate=v[14], rearSlip=v[15]
          }
        end
      end
    end
  end
  if #loaded >= 2 then return loaded, telemetry end
  return nil
end

local function saveLine()
  if #state.nodes < 2 then
    state.status = "Нужно минимум 2 точки"
    return
  end

  local data = serializeNodes(state.nodes, state.telemetry)
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
  local loaded, loadedTelemetry = nil, {}
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
        loaded, loadedTelemetry = parseNodes(raw)
      end
    end
  end

  if not loaded and ac.storage then
    local okS, rawSt = pcall(ac.storage, "RP_LINE_DATA_" .. getTrackTag())
    if okS and rawSt and rawSt ~= "" then
      loaded, loadedTelemetry = parseNodes(rawSt)
    end
  end

  if loaded and #loaded >= 2 then
    state.nodes = loaded
    state.telemetry = loadedTelemetry or {}
    state.p = {}
    state.status = string.format("Загружено: %d т.", #state.nodes)
    writeLog("LOAD_LINE count=%d track=%s", #state.nodes, getTrackTag())
    flushLog()
  else
    state.status = "Линия не найдена"
  end
end

local function addPoint(pos, car)
  if #state.nodes == 0 or flatDist(state.nodes[#state.nodes], pos) >= 0.75 then
    table.insert(state.nodes, vec3(pos.x, pos.y + 0.1, pos.z))
    local t = {
      speed=math.max(0, car.speedMs or 0),
      steer=(car.steer or 0)/math.max(1, car.steerLock or 450),
      gas=car.gas or 0, brake=car.brake or 0, clutch=car.clutch or 0,
      handbrake=car.handbrake or 0, gear=car.gear or 0, rpm=car.rpm or 0,
      slipAngle=0, lateralSpeed=0, yawRate=0, rearSlip=0
    }
    if car.velocity and car.side and car.look then
      local okS, sv = pcall(math.dot, car.velocity, car.side)
      local okF, fv = pcall(math.dot, car.velocity, car.look)
      if okS and type(sv)=="number" then t.lateralSpeed=sv end
      if okF and type(fv)=="number" and t.speed>SLIP_SPEED_MIN then
        t.slipAngle=math.deg(math.atan2(math.abs(t.lateralSpeed), math.max(0.1,math.abs(fv))))
      end
    end
    if car.localAngularVelocity then
      local okY, y = pcall(function() return car.localAngularVelocity.y end)
      if okY and type(y)=="number" then t.yawRate=y end
    end
    if car.wheels then
      local total, count = 0, 0
      for wi=2,3 do
        local w=car.wheels[wi]
        if w then
          local okN, nd=pcall(function() return w.ndSlip end)
          if okN and type(nd)=="number" then total=total+math.abs(nd); count=count+1 end
        end
      end
      if count>0 then t.rearSlip=total/count end
    end
    state.telemetry[#state.nodes]=t
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
    gasOut = 0,
    brake = 0,
    hb = 0,
    stuck = 0,
    lastTargetX = 0,
    lastTargetZ = 0,
    lastLineDist = 0,

    recovering = false,
    recoveryStable = 0,
    recoveryTime = 0,
    slipAngle = 0,
    lateralSpeed = 0,
    yawRate = 0,
    rearSlip = 0,
    tractionLimit = 1,
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

-- Reads normalized slip from the rear tyres.
-- Missing wheel data is ignored safely.
local function readRearSlip(car)
  if not car or not car.wheels then return nil end

  local total = 0
  local count = 0

  for i = 2, 3 do
    local wheel = car.wheels[i]
    if wheel then
      local ok, value = pcall(function()
        return wheel.ndSlip
      end)
      if ok and type(value) == "number" then
        total = total + math.abs(value)
        count = count + 1
      end
    end
  end

  if count == 0 then return nil end
  return total / count
end

-- Measures how far the car's actual movement direction differs
-- from its body direction. This also works when the car is sliding.
local function readMotionState(car)
  local speed = math.max(0, car.speedMs or 0)
  local lateralSpeed = 0
  local slipAngle = 0

  if car.velocity and car.side and car.look then
    local okSide, sideValue = pcall(math.dot, car.velocity, car.side)
    local okLook, forwardValue = pcall(math.dot, car.velocity, car.look)

    if okSide and type(sideValue) == "number" then
      lateralSpeed = sideValue
    end

    if okLook and type(forwardValue) == "number" then
      if speed > SLIP_SPEED_MIN then
        slipAngle = math.deg(math.atan2(
          math.abs(lateralSpeed),
          math.max(0.1, math.abs(forwardValue))
        ))
      end
    end
  end

  local yawRate = 0
  if car.localAngularVelocity then
    local ok, value = pcall(function()
      return car.localAngularVelocity.y
    end)
    if ok and type(value) == "number" then
      yawRate = value
    end
  end

  return slipAngle, lateralSpeed, yawRate, readRearSlip(car)
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

  local win = math.min(segCount - 1, math.max(8, math.floor(segCount * 0.25)))
  for offset = -win, win do
    local idx = ((bestSeg - 1 + offset) % segCount) + 1
    checkSeg(idx)
  end
  if bestDistSq > 16.0 * 16.0 then
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

  local lookAhead = clamp(9.0 + speed * 0.16, 9.0, 16.0)
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
    local segLen = math.sqrt((b.x - curX) * (b.x - curX) + (b.z - curZ) * (b.z - curZ))

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
    curve = math.abs((v1x / l1) * (v2z / l2) - (v1z / l1) * (v2x / l2))
  end

  local look = car.look
  local fx, fz = look.x, look.z
  local fLen = math.sqrt(fx * fx + fz * fz)
  if fLen < 1e-4 then return 0, 0, lineDist, targetX, targetZ end
  fx, fz = fx / fLen, fz / fLen

  local dx = targetX - px
  local dz = targetZ - pz
  local dLen = math.sqrt(dx * dx + dz * dz)
  if dLen < 1e-4 then return 0, 0, lineDist, targetX, targetZ end
  local dirX, dirZ = dx / dLen, dz / dLen

  local dot = clamp(fx * dirX + fz * dirZ, -1, 1)
  local cross = fx * dirZ - fz * dirX
  local angle = math.atan2(cross, dot)
  local steer = clamp(angle * 1.4, -0.8, 0.8)

  local reference = state.telemetry and state.telemetry[bestSeg] or nil
  return steer, curve, lineDist, targetX, targetZ, reference
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

-- Anti-donut controller:
-- detects excessive lateral motion, rear wheelspin and uncontrolled yaw.
-- Recovery stays active until the car has settled for a short period.
local function updateAntiDonut(car, p, dt, routeSteer, lineDist, reference)
  local slipAngle, lateralSpeed, yawRate, rearSlip = readMotionState(car)

  p.slipAngle = slipAngle
  p.lateralSpeed = lateralSpeed
  p.yawRate = yawRate
  p.rearSlip = rearSlip or 0

  local absLateral = math.abs(lateralSpeed)
  local absYaw = math.abs(yawRate)
  local absSlipAngle = math.abs(slipAngle)

  local targetAngle = reference and math.abs(reference.slipAngle or 0) or 0
  local targetLateral = reference and math.abs(reference.lateralSpeed or 0) or 0
  local targetYaw = reference and math.abs(reference.yawRate or 0) or 0
  local angleExcess = math.max(0, absSlipAngle-targetAngle-8.0)
  local lateralExcess = math.max(0, absLateral-targetLateral-1.5)
  local yawExcess = math.max(0, absYaw-targetYaw-0.35)
  local angleRisk = smoothstep(SLIP_ANGLE_START, SLIP_ANGLE_FULL, angleExcess)
  local lateralRisk = smoothstep(LATERAL_SLIP_START, LATERAL_SLIP_FULL, lateralExcess)
  local yawRisk = smoothstep(YAW_RECOVERY_START, YAW_RECOVERY_FULL, yawExcess)

  local wheelRisk = 0
  if rearSlip then
    wheelRisk = smoothstep(REAR_SLIP_START, REAR_SLIP_FULL, rearSlip)
  end

  local risk = math.max(angleRisk, lateralRisk, yawRisk, wheelRisk)

  -- At very low speed the slip angle is unreliable.
  -- Yaw rate remains useful for detecting a car rotating on the spot.
  local spinAtLowSpeed = car.speedMs < 3.0 and absYaw > YAW_RECOVERY_START
  local severeSlide = angleExcess > 24.0 or lateralExcess > 4.5
  local severeWheelspin = rearSlip and rearSlip > math.max(1.55, (reference and (reference.rearSlip or 0) or 0) + 0.65)

  if spinAtLowSpeed or severeSlide or severeWheelspin then
    p.recovering = true
    p.recoveryStable = 0
  end

  if p.recovering then
    local settled =
      absYaw < 0.42 and
      absSlipAngle < 11.0 and
      absLateral < 1.5 and
      (not rearSlip or rearSlip < 1.05)

    if settled then
      p.recoveryStable = p.recoveryStable + dt
    else
      p.recoveryStable = 0
    end

    if p.recoveryStable >= RECOVERY_RELEASE_TIME then
      p.recovering = false
      p.recoveryStable = 0
    end
  end

  local tractionLimit = 1.0 - risk * 0.88

  -- A large route deviation combined with strong steering means
  -- the car is no longer following the route cleanly.
  local routeRisk = smoothstep(2.5, 9.0, lineDist)
  local steeringRisk = smoothstep(0.35, 0.72, math.abs(routeSteer))
  local routeLimit = 1.0 - routeRisk * steeringRisk * 0.85

  tractionLimit = math.min(tractionLimit, routeLimit)

  if p.recovering then
    tractionLimit = math.min(tractionLimit, 0.04)
  end

  -- Countersteer against the lateral direction of travel.
  -- When nearly stopped, use yaw rate to oppose rotation.
  local recoverySteer = routeSteer

  if p.recovering then
    if absLateral > 0.8 then
      local lateralDirection = lateralSpeed >= 0 and 1 or -1
      recoverySteer = clamp(
        routeSteer + lateralDirection * 0.42,
        -0.8, 0.8
      )
    elseif absYaw > 0.25 then
      recoverySteer = clamp(
        routeSteer - yawRate * 0.42,
        -0.8, 0.8
      )
    end
  elseif risk > 0.15 and absLateral > 1.0 then
    local lateralDirection = lateralSpeed >= 0 and 1 or -1
    recoverySteer = clamp(
      routeSteer + lateralDirection * 0.28 * risk,
      -0.8, 0.8
    )
  end

  p.tractionLimit = tractionLimit
  p.recoveryTime = p.recovering and (p.recoveryTime + dt) or 0

  return tractionLimit, recoverySteer, risk
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
  local offCenter = clamp((dRight - dLeft) / math.max(1, room), -1, 1)

  local speed = math.max(0, car.speedMs or 0)
  local steer, curve, lineDist, targetX, targetZ, reference =
    followRouteSteer(car, p, speed, state.nodes)

  p.lastTargetX = targetX
  p.lastTargetZ = targetZ
  p.lastLineDist = lineDist

  if nearWall < 4.0 then
    local wallAvoid = -offCenter * 0.35
    local wBlend = clamp((4.0 - nearWall) / 4.0, 0, 0.35)
    steer = clamp(steer * (1 - wBlend) + wallAvoid * wBlend, -0.8, 0.8)
  end

  local targetSpeed
  if reference and type(reference.speed)=="number" then
    targetSpeed = math.max(0, reference.speed)
  else
    targetSpeed = p.speed * clamp(1.0 - curve * 2.0, 0.45, 1.0)
  end
  local wallSpeedCap = math.sqrt(math.max(0, 2 * BRAKE_ACCEL * math.max(0, dFwd - WALL_MARGIN)))
  if wallSpeedCap > 0 then
    targetSpeed = math.min(targetSpeed, wallSpeedCap)
  end

  local gap, leaderSpeed = carAhead(carIndex, car)
  if gap then
    local safeSpeed = math.sqrt(math.max(0, leaderSpeed * leaderSpeed + 2 * BRAKE_ACCEL * math.max(0, gap - state.followGap)))
    if safeSpeed < targetSpeed then targetSpeed = safeSpeed end
    if gap < state.followGap then targetSpeed = 0 end
  end

  local speedError = targetSpeed - speed
  local rawGas = clamp(speedError * 0.35, 0, 1)
  local brake = clamp(-speedError * 0.45, 0, 1)
  if reference then
    if speedError >= 0 and type(reference.gas)=="number" then
      rawGas = clamp(rawGas * 0.55 + reference.gas * 0.45, 0, 1)
    elseif speedError < 0 and type(reference.brake)=="number" then
      brake = math.max(brake, clamp(reference.brake * 0.35, 0, 1))
    end
  end

  -- Evaluate grip and rotation before applying throttle.
  if reference then
    if type(reference.steer)=="number" then
      steer = clamp(steer * 0.45 + reference.steer * 0.55, -0.8, 0.8)
    end
    if type(reference.yawRate)=="number" then
      steer = clamp(steer + clamp((reference.yawRate-(p.yawRate or 0))*0.12,-0.22,0.22),-0.8,0.8)
    end
  end
  local tractionLimit, correctedSteer, risk =
    updateAntiDonut(car, p, dt, steer, lineDist, reference)

  local desiredGas = math.min(rawGas, tractionLimit)

  -- During recovery, do not add throttle to a car that is still rotating.
  if p.recovering then
    desiredGas = 0
    brake = 0
  end

  -- Smooth throttle: reduce quickly when grip is lost, restore gradually.
  local gasRate = desiredGas < p.gasOut and GAS_FALL_RATE or GAS_RISE_RATE
  local maxGasChange = gasRate * math.max(dt, 0)
  p.gasOut = p.gasOut + clamp(desiredGas - p.gasOut, -maxGasChange, maxGasChange)
  p.gasOut = clamp(p.gasOut, 0, 1)

  if speed < 1.5 then
    p.stuck = p.stuck + dt
    if p.stuck > 3.0 and not p.recovering then
      p.stuck = 0
      newPersonality(carIndex)
      p = state.p[carIndex]
      writeLog("car #%d: stuck, new personality", carIndex)
    end
  else
    p.stuck = 0
  end

  -- Steering is smoothed, but countersteering remains responsive.
  local steerRate = p.recovering and 12.0 or 8.0
  p.steer = p.steer + (correctedSteer - p.steer) * clamp(dt * steerRate, 0, 1)

  p.gas = p.gasOut
  p.brake = brake
  p.hb = 0

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
    table.insert(parts, string.format("#%d(%.1f,%.1f,%.1f d=%.1fm)", idx, pt.x, pt.y, pt.z, d))
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
        "car=%d pos=(%.2f,%.2f,%.2f) spd=%.1fkm/h(%.1fm/s) | ctrl: steer=%.3f gas=%.2f brk=%.2f hb=%.2f | grip: recovering=%s risk=%.2f slip=%.1fdeg lat=%.2f yaw=%.2f rearSlip=%.2f limit=%.2f | path: seg=%d err=%.2fm tgt=(%.1f,%.1f) look=(%.2f,%.2f) vel=(%.2f,%.2f) | nodes: %s",
        i,
        pos.x, pos.y, pos.z,
        spdKmh, spdMs,
        p.steer, p.gas, p.brake, p.hb,
        tostring(p.recovering), risk or 0,
        p.slipAngle or 0, p.lateralSpeed or 0,
        p.yawRate or 0, p.rearSlip or 0,
        p.tractionLimit or 1,
        p.segIdx, p.lastLineDist, p.lastTargetX, p.lastTargetZ,
        look and look.x or 0, look and look.z or 0,
        vel and vel.x or 0, vel and vel.z or 0,
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
  if state.recording and player and not player.isAIControlled then
    addPoint(player.position, player)
  end

  local sim = ac.getSim()
  if not sim then return end

  if not state.enabled or #state.nodes < 2 then
    for idx in pairs(state.p) do releaseCar(idx) end
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

  local telemetryCount = 0
  for _ in pairs(state.telemetry) do telemetryCount = telemetryCount + 1 end
  ui.text(string.format("Точек: %d | телеметрия: %d | %s", #state.nodes, telemetryCount, state.status))
  if logger and logger.path then
    ui.text("Лог: " .. tostring(logger.path))
  end

  if ui.button(state.recording and "Стоп запись" or "Запись линии", vec2(150, 24)) then
    state.recording = not state.recording
    if state.recording then
      state.nodes = {}
      state.telemetry = {}
      state.p = {}
      state.status = "Запись линии + телеметрия..."
    else
      state.status = "Записано точек: " .. #state.nodes
    end
  end
  ui.sameLine()
  if ui.button(state.loopRoute and "Круг: ВКЛ" or "Круг: ВЫКЛ", vec2(-1, 24)) then
    state.loopRoute = not state.loopRoute
  end

  local canvas = vec2(300, 125)
  ui.invisibleButton("line_radar", canvas)
  local p0 = ui.getItemRectMin()
  local p1 = ui.getItemRectMax()
  ui.drawRectFilled(p0, p1, rgbm(0.1, 0.1, 0.12, 1), 4.0)
  ui.drawRect(p0, p1, rgbm(0.3, 0.3, 0.35, 1), 4.0)

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
      if x1 >= p0.x and x1 <= p1.x and y1 >= p0.y and y1 <= p1.y and
         x2 >= p0.x and x2 <= p1.x and y2 >= p0.y and y2 <= p1.y then
        ui.drawLine(vec2(x1, y1), vec2(x2, y2), rgbm(1.0, 0.85, 0.0, 0.9), 2.0)
      end
    end
    ui.drawCircleFilled(vec2(mx, my), 4.5, rgbm(0.0, 0.8, 1.0, 1.0), 12)
  end

  if ui.button("Сохранить", vec2(96, 24)) then saveLine() end
  ui.sameLine()
  if ui.button("Загрузить", vec2(96, 24)) then loadLine() end
  ui.sameLine()
  if ui.button("Очистить", vec2(-1, 24)) then
    state.nodes = {}
    state.telemetry = {}
    state.p = {}
    state.status = "Очищено"
  end

  state.speedMin = ui.slider("Мин. скорость", state.speedMin, 40, 180, "%.0f км/ч")
  state.speedMax = ui.slider("Макс. скорость", state.speedMax, 60, 260, "%.0f км/ч")
end

if ac.onRelease then
  ac.onRelease(function()
    for idx in pairs(state.p) do releaseCar(idx) end
    if logger and logger.shutdown then
      logger:shutdown("RoutePilot released")
    end
  end)
end

loadLine()
