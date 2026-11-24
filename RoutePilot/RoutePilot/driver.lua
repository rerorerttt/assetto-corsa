-- RoutePilot/driver.lua — single-car kinematic path test (CSP 0.3.2-preview2)
-- One active AI car is positioned on the recorded lane every update.
-- Engine, gearbox, throttle and audio remain under AC physics control.
-- No tyre-force steering controller, braking planner or multi-car logic.

local Driver = {}
local state, H

local function normalize2(x, z)
  local length = math.sqrt(x * x + z * z)
  if length < 0.00001 then return 0, 1 end
  return x / length, z / length
end

local function safeNumber(v, fallback)
  if type(v) ~= "number" or v ~= v or v == math.huge or v == -math.huge then
    return fallback or 0
  end
  return v
end

function Driver.setup(_, st, helpers)
  state = st
  H = helpers
end

local function applyEngineControls(carIndex)
  local controls = ac.overrideCarControls(carIndex)
  if not controls then
    error("ac.overrideCarControls returned nil")
  end
  controls.combineAxis = false
  controls.steer = 0
  controls.gas = 1
  controls.brake = 0
  controls.handbrake = 0
end

function Driver.update(carIndex, car, dt)
  if type(physics) ~= "table" or
      type(physics.setAICarPosition) ~= "function" then
    error("physics.setAICarPosition is unavailable")
  end

  local p = state.p[carIndex]
  if not p then
    local seg = select(1, H.findNearestSeg(car.position, 1))
    local laneOffset, twoLane = H.laneOffset(carIndex, seg)
    p = {
      segIdx = seg,
      laneOffset = laneOffset,
      laneRole = not twoLane and "SINGLE" or
        (laneOffset >= 0 and "LEFT" or "RIGHT"),
      lineDist = 0,
      retireTimer = 0
    }
    state.p[carIndex] = p
    H.log("KINEMATIC_INIT id=%d lane=%s offset=%.2f", carIndex,
      p.laneRole, laneOffset)
  end

  local seg, t, lineDistance = H.findNearestSeg(car.position, p.segIdx)
  p.segIdx = seg
  p.lineDist = lineDistance
  local laneOffset, twoLane = H.laneOffset(carIndex, seg)
  p.laneOffset = laneOffset
  p.laneRole = not twoLane and "SINGLE" or
    (laneOffset >= 0 and "LEFT" or "RIGHT")

  -- Full throttle and no braking: native AC engine and automatic gearbox
  -- determine acceleration and top speed. No artificial speed ceiling.
  applyEngineControls(carIndex)

  p.retireTimer = p.retireTimer + dt
  if p.retireTimer >= 1.0 then
    p.retireTimer = 0
    if physics.preventAIFromRetiring then
      pcall(physics.preventAIFromRetiring, carIndex)
    end
  end

  local speed = math.max(0, safeNumber(car.speedMs, 0))
  local advance = speed * math.min(math.max(dt, 0), 0.05)
  local point = H.walkLine(seg, t, advance)
  local ahead = H.walkLine(seg, t, advance + 1.0)
  if not point then
    error("recorded route cannot be sampled")
  end
  if not ahead then ahead = point end

  local tx, tz = normalize2(ahead.x - point.x, ahead.z - point.z)
  local target = vec3(
    point.x - tz * laneOffset,
    point.y,
    point.z + tx * laneOffset
  )
  -- Match the direction convention already used by RoutePilot spawn code.
  local direction = vec3(-tx, 0, -tz)

  physics.setAICarPosition(carIndex, target, direction)
  p.speed = speed
end

function Driver.telemetryLine(i, car, p)
  return string.format(
    "CAR=%d MODE=KINEMATIC LANE=%s POS=(%.2f,%.2f,%.2f) SPEED=%.2f GAS=1.000 BRAKE=0.000 LINE_ERROR=%.2f SEGMENT=%d LANE_OFFSET=%.2f",
    i, p.laneRole, car.position.x, car.position.y, car.position.z,
    safeNumber(car.speedMs), p.lineDist, p.segIdx, p.laneOffset)
end

return Driver
