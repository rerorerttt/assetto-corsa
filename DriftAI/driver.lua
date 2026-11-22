-- DriftAI/driver.lua
-- Per-car drift controller (v1.1.0).
--
-- Control interception (confirmed in CSP 0.3.2-preview2):
--   ac.overrideCarControls(carIndex) -> ac.CarControlsInput (struct `carcontrolsoverride`,
--   SDK: common/ac_game.lua). Available to Lua apps since 0.2.2-preview1, AI overrides included.
--   With `.combineAxis = false` (added in 0.3.0-preview125) pedals and handbrake fully
--   replace the stock AI values instead of being max-combined; `.steer` always replaces.
--   `.requestedGearIndex` allows direct gear selection. `:active()` verifies that the physics
--   thread has consumed the inputs.
--
-- v1.1.0 changes:
--   * Wheelspin-targeted throttle: the target is the actual rear-wheel slip ratio
--     (car.wheels[2]/[3].slipRatio, 0-based ac.Wheel order FL,FR,RL,RR). Throttle
--     integrates toward TARGET_WHEEL_SLIP; if the car is starved at wide-open
--     throttle, a lower gear is requested automatically.
--   * Boundary-aware speed planning: speed limit from the braking distance to the
--     nearest wall ahead (v = sqrt(2*a*(d - margin))); with enough free road the
--     driver commits to a long, fast drift.
--   * Higher entry speed (55 km/h), higher hard cap (150 km/h), target slip 32 deg.

local Track = require('track')

local clamp = math.clamp
local abs, max, min, atan2, rad = math.abs, math.max, math.min, math.atan2, math.rad

local function sign(v)
  if v > 0 then return 1 elseif v < 0 then return -1 end
  return 0
end

-- ─────────────────── tunables ───────────────────

local TARGET_SLIP = rad(32)       -- target body slip angle
local SLIP_DEADBAND = rad(4)       -- no correction inside this band
local SLIP_KP = 2.4                -- steer correction gain (per radian of error)
local SLIP_KD = 0.35               -- steer damping (per rad/s of slip rate)
local STEER_RATE = 12.0            -- steering slew limit (per second)
local MAX_STEER = 1.0

-- wheelspin target (rear slip ratio, ac.StateWheel.slipRatio)
local TARGET_WHEEL_SLIP = 0.30     -- sustained rear wheelspin for a long drift
local WHEEL_SLIP_DEADBAND = 0.03
local WHEEL_SLIP_GAIN = 60.0       -- throttle correction per second per unit of error
local STARVED_TIME = 0.5           -- s of WOT without wheelspin -> downshift
local GAS_MIN_DRIFT = 0.45
local HANDBRAKE_KICK = 1.0

local INIT_SPEED_KMH = 55          -- entry speed
local SPEED_LIMIT_KMH = 150        -- hard cap
local BRAKE_DECEL = 6.0            -- planning deceleration, m/s^2
local WALL_MARGIN = 8              -- meters kept away from any wall
local OPEN_TRACK_DIST = 120        -- more free road than this -> full commitment

local GEAR_HOLDS = { [1] = 60, [2] = 95, [3] = 130 } -- upshift thresholds, km/h

local WHEEL_RL, WHEEL_RR = 2, 3    -- 0-based ac.Wheel: FL,FR,RL,RR

-- ─────────────────── driver class ───────────────────

local Driver = {}
Driver.__index = Driver

function Driver.new(carIndex, config)
  local self = setmetatable({}, Driver)
  self.index = carIndex
  self.config = config
  self.bounds = config.bounds
  self.state = 'accelerate'     -- accelerate | drift | recover | lost
  self.stateTime = 0
  self.steer = 0
  self.gas = 0
  self.brake = 0
  self.handbrake = 0
  self.requestedGearIndex = 0
  self.slipAngle = 0
  self.prevSlipAngle = 0
  self.slipRate = 0
  self.rearSlip = nil
  self.starvedTimer = 0
  self.progress = nil
  self.lateral = 0
  self.wallAhead = nil
  self.driftDir = 0             -- -1 right drift, 1 left drift, 0 unset
  self.driftDirLock = 0
  self.stuckTimer = 0
  self.driftTime = 0
  return self
end

local function bodySlipAngle(car)
  -- Angle between heading and velocity direction, signed.
  local v = car.velocity
  local speed = v:length()
  if speed < 1.5 then return 0 end
  local look = car.look
  local dx, dz = v.x / speed, v.z / speed
  local dot = clamp(look.x * dx + look.z * dz, -1, 1)
  local cross = look.z * dx - look.x * dz
  return atan2(cross, dot)
end

-- Average rear-wheel slip ratio (actual wheelspin feedback), or nil.
local function rearWheelspin(car)
  local ok, wl = pcall(function() return car.wheels[WHEEL_RL] end)
  local ok2, wr = pcall(function() return car.wheels[WHEEL_RR] end)
  if not ok or not ok2 or not wl or not wr then return nil end
  local ok3, sl = pcall(function() return wl.slipRatio end)
  local ok4, sr = pcall(function() return wr.slipRatio end)
  if not ok3 or not ok4 or type(sl) ~= 'number' or type(sr) ~= 'number' then
    return nil
  end
  return (abs(sl) + abs(sr)) * 0.5
end

-- Speed the car may carry so the braking distance fits the free road ahead:
--   v = sqrt(2 * a * (d_wall - margin))
local function wallLimitedSpeed(wallAheadM)
  if not wallAheadM then return SPEED_LIMIT_KMH end
  local d = max(0, wallAheadM - WALL_MARGIN)
  return min(SPEED_LIMIT_KMH, math.sqrt(2 * BRAKE_DECEL * d) * 3.6)
end

-- Compute steering command from drift stabilisation + navigation.
function Driver:_steering(car, dt, targetX, targetZ)
  local pos = car.position
  local look = car.look
  local dx = targetX - pos.x
  local dz = targetZ - pos.z
  local dist = math.sqrt(dx * dx + dz * dz)
  local aim = 0
  if dist > 0.5 then
    local forwardDot = look.x * dx + look.z * dz
    local cross = look.z * dx - look.x * dz
    aim = atan2(cross, forwardDot)
    aim = clamp(atan2(2 * 2.65 * math.sin(aim), max(1, dist)) / 0.55, -1, 1)
  end

  -- counter-steer: hold the target slip angle, damp the slip rate
  local slipErr = self.slipAngle - self.driftDir * TARGET_SLIP
  if abs(self.slipAngle) < SLIP_DEADBAND then slipErr = 0 end
  local correction = -(slipErr * SLIP_KP + self.slipRate * SLIP_KD)
  correction = clamp(correction, -0.85, 0.85)

  local steer
  if self.state == 'drift' then
    steer = clamp(self.driftDir * 0.45 + correction, -MAX_STEER, MAX_STEER)
  else
    steer = clamp(aim, -MAX_STEER, MAX_STEER)
  end

  local rate = STEER_RATE * dt
  return clamp(steer, self.steer - rate, self.steer + rate)
end

function Driver:_pickDriftDirection(curvature)
  if self.driftDir ~= 0 and self.driftDirLock > 0 then
    return self.driftDir
  end
  if curvature and abs(curvature) > 1e-4 then
    return curvature > 0 and 1 or -1
  end
  return self.driftDir ~= 0 and self.driftDir or (self.index % 2 == 0 and 1 or -1)
end

-- ─────────────────── main update ───────────────────

---@param car ac.StateCar
---@param dt number
function Driver:update(car, dt)
  self.stateTime = self.stateTime + dt

  local slip = bodySlipAngle(car)
  self.prevSlipAngle = self.slipAngle
  self.slipAngle = slip
  self.slipRate = (slip - self.prevSlipAngle) / max(dt, 1e-3)
  self.rearSlip = rearWheelspin(car)
  local kmh = car.speedKmh

  -- navigation
  local progress, lateral = Track.getTrackPosition(car.position)
  self.progress = progress
  self.lateral = lateral or 0

  -- stuck detection
  if kmh < 8 and self.state ~= 'accelerate' then
    self.stuckTimer = self.stuckTimer + dt
  else
    self.stuckTimer = 0
  end

  if progress == nil then
    self.state = 'lost'
  end

  -- ── state machine ──
  if self.state == 'accelerate' then
    if kmh > INIT_SPEED_KMH then
      self.state = 'drift'
      self.stateTime = 0
      self.driftDirLock = 2.0
    end

  elseif self.state == 'drift' then
    self.driftTime = self.driftTime + dt
    if abs(slip) < rad(6) and kmh > INIT_SPEED_KMH and self.stateTime > 3 then
      self.state = 'accelerate' -- re-initiate
      self.stateTime = 0
    end
    if abs(slip) > rad(75) then
      self.state = 'recover'
      self.stateTime = 0
    end

  elseif self.state == 'recover' then
    if abs(slip) < rad(18) and kmh > 15 then
      self.state = 'drift'
      self.stateTime = 0
      self.driftDir = 0
      self.driftDirLock = 0
    elseif self.stuckTimer > 2.5 then
      self.state = 'accelerate'
      self.stateTime = 0
      self.stuckTimer = 0
    end
  end

  local curvature = Track.curvatureAhead(self.progress, self.config.trackLengthM, 20)
  if self.state == 'drift' then
    self.driftDir = self:_pickDriftDirection(curvature)
    self.driftDirLock = max(0, self.driftDirLock - dt)
  end

  -- ── navigation target ──
  local lookahead = clamp(8 + kmh * 0.06, 8, 22)
  local targetX, targetZ, halfWidth
  if self.progress then
    targetX, targetZ, halfWidth = Track.lookaheadTarget(car.position, self.progress,
      lookahead, self.config.trackLengthM)
  else
    local look = car.look
    targetX = car.position.x + look.x * lookahead
    targetZ = car.position.z + look.z * lookahead
  end

  -- ── boundary-aware speed planning ──
  local wallAhead = self.bounds and self.bounds:wallAheadOver(progress, 160) or nil
  self.wallAhead = wallAhead
  local limit = wallLimitedSpeed(wallAhead)
  if halfWidth and halfWidth < 6 then limit = min(limit, 70) end
  local openTrack = wallAhead == nil or wallAhead >= OPEN_TRACK_DIST

  -- ── controls ──
  self.steer = self:_steering(car, dt, targetX, targetZ)
  local gas, brake, handbrake = 0, 0, 0

  if self.state == 'accelerate' then
    -- accelerate hard: speed is needed to spin the rear tires
    gas = 1.0
    if kmh > limit then gas = 0.3; brake = 0.25 end
    -- initiate a drift: steer into the corner and kick the handbrake
    if curvature and abs(curvature) > 0.004 and kmh > 45 and self.stateTime > 1.0 then
      handbrake = HANDBRAKE_KICK
    end

  elseif self.state == 'drift' then
    -- wheelspin-targeted throttle: keep the rear wheels spinning at TARGET_WHEEL_SLIP
    gas = self.gas
    if self.rearSlip then
      local err = TARGET_WHEEL_SLIP - self.rearSlip
      if abs(err) > WHEEL_SLIP_DEADBAND then
        gas = gas + err * WHEEL_SLIP_GAIN * dt -- continuous correction
      end
      -- not enough spin even at wide-open throttle -> ask for a lower gear
      if self.rearSlip < 0.08 and gas > 0.92 then
        self.starvedTimer = self.starvedTimer + dt
        if self.starvedTimer > STARVED_TIME then
          self.requestedGearIndex = max(1, car.gear - 1)
          self.starvedTimer = 0
        end
      else
        self.starvedTimer = 0
      end
      -- overspin: bleed off
      if self.rearSlip > TARGET_WHEEL_SLIP + 0.15 then
        gas = gas * 0.7
      end
    else
      -- no wheel telemetry: hold a strong throttle
      gas = max(GAS_MIN_DRIFT, gas)
    end
    gas = clamp(gas, GAS_MIN_DRIFT, 1)
    -- wall proximity: brake to the plan, pull back to the centerline
    if kmh > limit then
      gas = gas * 0.5
      brake = max(brake, clamp((kmh - limit) * 0.04, 0, 0.6))
    end
    -- open road and slip established: commit to a long fast drift
    if openTrack and abs(slip) > rad(15) then
      gas = max(gas, 0.9)
    end
    -- light handbrake tap if the drift dies out
    if abs(slip) < rad(10) and kmh > 50 then
      handbrake = 0.6
    end
    if halfWidth then
      local margin = halfWidth - 3.5
      if abs(self.lateral) > margin then
        self.steer = clamp(self.steer - sign(self.lateral) * 0.25, -1, 1)
        brake = max(brake, 0.25)
      end
    end

  elseif self.state == 'recover' then
    self.steer = clamp(-sign(slip) * abs(slip) * 1.2, -1, 1)
    brake = clamp(0.3 + abs(slip) * 0.3, 0.2, 0.8)
    gas = 0

  elseif self.state == 'lost' then
    self.steer = 0
    gas = clamp((40 - kmh) * 0.05, 0, 0.5)
  end

  -- spin-out prevention even in drift: cut gas at extreme slip
  if abs(slip) > rad(60) then gas = min(gas, 0.2) end

  self.gas = clamp(gas, 0, 1)
  self.brake = clamp(brake, 0, 1)
  self.handbrake = clamp(handbrake, 0, 1)

  -- ── gearbox: hold low gears for torque ──
  if self.state == 'drift' or self.state == 'accelerate' then
    local gear = car.gear
    if self.requestedGearIndex == 0 and gear >= 1 and gear <= 3 then
      if kmh > GEAR_HOLDS[gear] and gear < 3 then
        self.requestedGearIndex = gear + 1
      end
    end
  else
    self.requestedGearIndex = 0
  end
end

---Apply the computed inputs to the car through the controls override.
---@return boolean applied true when the override struct was written
function Driver:applyControls(dt)
  local ok, controls = pcall(ac.overrideCarControls, self.index)
  if not ok or not controls then return false end
  controls.combineAxis = false
  controls.steer = self.steer
  controls.gas = self.gas
  controls.brake = self.brake
  controls.handbrake = self.handbrake
  controls.requestedGearIndex = self.requestedGearIndex
  return true
end

---True if the physics thread is consuming this override (checked periodically).
function Driver:isActive()
  local ok, controls = pcall(ac.overrideCarControls, self.index)
  if not ok or not controls then return false end
  return controls:active()
end

---Neutralize the stock AI controller for this car.
function Driver:disableStockAI()
  local idx = self.index
  pcall(physics.setAIAggression, idx, 0)
  pcall(physics.setAICaution, idx, 0)
  pcall(physics.setAILevel, idx, 1)
  pcall(physics.preventAIFromRetiring, idx, true)
end

return Driver
