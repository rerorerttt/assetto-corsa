-- DriftAI/driver.lua
-- Per-car drift controller.
--
-- Control interception (confirmed in CSP 0.3.2-preview2):
--   ac.overrideCarControls(carIndex) -> ac.CarControlsInput (struct `carcontrolsoverride`,
--   SDK: common/ac_game.lua). Available to Lua apps since 0.2.2-preview1, AI overrides included.
--   With `.combineAxis = false` (added in 0.3.0-preview125) pedals and handbrake fully
--   replace the stock AI values instead of being max-combined; `.steer` always replaces.
--   `.requestedGearIndex` allows direct gear selection. `:active()` verifies that the physics
--   thread has consumed the inputs.
--
-- Stock AI is additionally neutralized with physics.setAIAggression/setAICaution/setAILevel
-- (verified signatures from real-world apps: FullSendAI, Realistic Trackday).

local Track = require('track')

local clamp = math.clamp
local abs, max, min, atan2, rad = math.abs, math.max, math.min, math.atan2, math.rad

local function sign(v)
  if v > 0 then return 1 elseif v < 0 then return -1 end
  return 0
end

-- ─────────────────── tunables ───────────────────

local TARGET_SLIP = rad(28)      -- target body slip angle
local SLIP_DEADBAND = rad(4)      -- no correction inside this band
local SLIP_KP = 2.4               -- steer correction gain (per radian of error)
local SLIP_KD = 0.35              -- steer damping (per rad/s of slip rate)
local STEER_RATE = 10.0           -- steering slew limit (per second)
local MAX_STEER = 1.0

local GAS_BASE = 0.85             -- steady-state throttle while drifting
local GAS_MIN_DRIFT = 0.55        -- never below this while a drift is established
local GAS_SLIP_GAIN = 0.8         -- throttle cut per radian of over-rotation
local BRAKE_LIMITER = 0.35        -- speed limiter brake blend (keeps the drift)
local HANDBRAKE_KICK = 1.0        -- handbrake pulse to initiate a drift

local INIT_SPEED_KMH = 40         -- build speed before the first drift entry
local SPEED_LIMIT_KMH = 120       -- hard cap
local CORNER_SPEED_FACTOR = 0.62  -- target drift speed vs grip cornering speed

local GEAR_HOLDS = { [1] = 55, [2] = 85, [3] = 118 } -- upshift thresholds, km/h

-- ─────────────────── driver class ───────────────────

local Driver = {}
Driver.__index = Driver

function Driver.new(carIndex, config)
  local self = setmetatable({}, Driver)
  self.index = carIndex
  self.config = config
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
  self.progress = nil
  self.lateral = 0
  self.driftDir = 0             -- -1 right drift, 1 left drift, 0 unset
  self.driftDirLock = 0          -- time the drift direction stays forced
  self.stuckTimer = 0
  self.driftTime = 0
  return self
end

local function bodySlipAngle(car)
  -- Angle between heading and velocity direction, signed.
  -- car.look is the forward vector, car.velocity is world velocity in m/s.
  local v = car.velocity
  local speed = v:length()
  if speed < 1.5 then return 0 end
  local look = car.look
  local dx, dz = v.x / speed, v.z / speed
  local dot = clamp(look.x * dx + look.z * dz, -1, 1)
  local cross = look.z * dx - look.x * dz
  return atan2(cross, dot)
end

-- Compute steering command from drift stabilisation + navigation.
-- Navigation: pure-pursuit aim at a lookahead point on the track centerline,
-- blended with counter-steer correction of the body slip angle.
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
    -- pure-pursuit steering angle for a ~2.65 m wheelbase, clamped
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

  -- slew limit for smoothness
  local rate = STEER_RATE * dt
  return clamp(steer, self.steer - rate, self.steer + rate)
end

-- Speed management: keep the car fast enough to hold a drift, slow enough
-- not to fly off at corner entries.
function Driver:_speed(car, halfWidth)
  local kmh = car.speedKmh
  local curvature = Track.curvatureAhead(self.progress, self.config.trackLengthM, 25)
  local limit = SPEED_LIMIT_KMH
  if curvature and abs(curvature) > 1e-4 then
    -- grip-limited corner speed (a_lat ~= 8 m/s^2), scaled down for drift style
    local vGrip = math.sqrt(8 / abs(curvature)) * 3.6
    limit = min(limit, vGrip * CORNER_SPEED_FACTOR + 25)
  end
  if halfWidth and halfWidth < 6 then limit = min(limit, 70) end
  return limit
end

function Driver:_pickDriftDirection(curvature)
  if self.driftDir ~= 0 and self.driftDirLock > 0 then
    return self.driftDir
  end
  -- enter the turn: left turn (positive curvature) -> left-side drift
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

  -- off-track / lost detection
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
    -- spin-out guard: too much slip
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

  -- drift direction follows the upcoming corner
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

  -- ── controls ──
  self.steer = self:_steering(car, dt, targetX, targetZ)
  local speedLimit = self:_speed(car, halfWidth)
  local gas, brake, handbrake = 0, 0, 0

  if self.state == 'accelerate' then
    local err = speedLimit - kmh
    gas = clamp(0.35 + err * 0.05, 0.2, 1)
    if kmh > speedLimit then brake = 0.2 end
    -- initiate a drift: steer into the corner and kick the handbrake
    if curvature and abs(curvature) > 0.004 and kmh > 45 and self.stateTime > 1.2 then
      handbrake = HANDBRAKE_KICK
      gas = max(GAS_MIN_DRIFT, gas * 0.7)
    end

  elseif self.state == 'drift' then
    gas = GAS_BASE - clamp((abs(slip) - TARGET_SLIP) * GAS_SLIP_GAIN, 0, 0.4)
    gas = clamp(gas, GAS_MIN_DRIFT, 1)
    if kmh > speedLimit then
      gas = gas * 0.6
      brake = BRAKE_LIMITER
    end
    -- light handbrake tap if the drift dies out
    if abs(slip) < rad(10) and kmh > 50 then
      handbrake = 0.6
    end
    -- wall proximity: pull the lateral offset back toward the centerline
    if halfWidth then
      local margin = halfWidth - 3.5
      if abs(self.lateral) > margin then
        self.steer = clamp(self.steer - sign(self.lateral) * 0.25, -1, 1)
        brake = max(brake, 0.25)
      end
    end

  elseif self.state == 'recover' then
    -- counter-steer against the spin, brake moderately, then resume
    self.steer = clamp(-sign(slip) * abs(slip) * 1.2, -1, 1)
    brake = clamp(0.3 + abs(slip) * 0.3, 0.2, 0.8)
    gas = 0

  elseif self.state == 'lost' then
    -- no track spline: drive straight ahead, gently
    self.steer = 0
    gas = clamp((40 - kmh) * 0.05, 0, 0.5)
  end

  -- spin-out prevention even in drift: cut gas at extreme slip
  if abs(slip) > rad(60) then gas = min(gas, 0.2) end

  self.gas = clamp(gas, 0, 1)
  self.brake = clamp(brake, 0, 1)
  self.handbrake = clamp(handbrake, 0, 1)

  -- ── gearbox: hold low gears for torque, straight from requestedGearIndex ──
  self.requestedGearIndex = 0
  if self.state == 'drift' or self.state == 'accelerate' then
    local gear = car.gear
    if gear >= 1 and gear <= 3 then
      if kmh < GEAR_HOLDS[gear] * 0.75 then
        self.requestedGearIndex = max(1, gear - 1) -- hold
      elseif kmh > GEAR_HOLDS[gear] and gear < 3 then
        self.requestedGearIndex = gear + 1
      end
    end
  end
end

---Apply the computed inputs to the car through the controls override.
---@param dt number
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
---@return boolean
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
