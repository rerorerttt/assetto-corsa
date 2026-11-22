-- DriftAI/driver.lua
-- Per-car drift controller (v1.2.1).
--
-- Control interception (confirmed in CSP 0.3.2-preview2):
--   ac.overrideCarControls(carIndex) -> ac.CarControlsInput (struct `carcontrolsoverride`,
--   SDK: common/ac_game.lua). Available to Lua apps since 0.2.2-preview1, AI overrides included.
--   With `.combineAxis = false` (added in 0.3.0-preview125) pedals and handbrake fully
--   replace the stock AI values; `.steer` always replaces. `.requestedGearIndex` selects
--   gears directly. `:active()` verifies that the physics thread consumed the inputs.
--
-- Stock AI is additionally neutralized with physics.setAIAggression/setAICaution/setAILevel.
--
-- v1.2.1 (on top of v1.2.0, user feedback: "too slow, never reaches a drift"):
--   * `initiate` state: deterministic drift entry on ANY section — big turn-in
--     (0.9), full throttle and handbrake pulses (0.35 s every 0.8 s) until the
--     slip angle exceeds 12 deg; retries with the opposite direction after 2.5 s.
--     No more waiting for curvature to happen to be high.
--   * Wheelspin-first throttle: full throttle is the default; it is trimmed only
--     when the rear slip ratio overshoots the target (0.35). Wide-open throttle
--     with no wheelspin for 0.35 s -> immediate lower gear request (3 s cooldown
--     prevents upshift oscillation). No telemetry -> full throttle.
--   * Corridor speed planning: the limit comes from the length of the free track
--     corridor ahead (Bounds:freeCorridor), not from the worst single ray within
--     160 m — flowing sections are no longer choked by a far barrier.
--     v_limit = min(170, sqrt(2*8*(corridor - 12 m))); corner speed from curvature
--     is the entry target. Corridor >= 150 m -> commit to a long, fast drift.
--   * Higher speeds: entry 70 km/h, hard cap 170 km/h, target slip 35 deg,
--     upshift thresholds extended to 4 gears.

local Track = require('track')

local clamp = math.clamp
local abs, max, min, atan2, rad = math.abs, math.max, math.min, math.atan2, math.rad

local function sign(v)
  if v > 0 then return 1 elseif v < 0 then return -1 end
  return 0
end

-- ─────────────────── tunables ───────────────────

-- slip-angle stabilisation
local TARGET_SLIP = rad(35)        -- target body slip angle
local SLIP_DEADBAND = rad(4)        -- no correction inside this band
local SLIP_KP = 2.6                -- steer correction gain (per radian of error)
local SLIP_KD = 0.4                -- steer damping (per rad/s of slip rate)
local STEER_RATE = 14.0             -- steering slew limit (per second)
local MAX_STEER = 1.0

-- wheelspin control (rear slip ratio, ac.StateWheel.slipRatio)
local TARGET_WHEEL_SLIP = 0.35      -- sustained rear wheelspin for a long drift
local WHEEL_SLIP_DEADBAND = 0.02
local WHEEL_SLIP_GAIN = 80.0        -- throttle correction per second per unit of error
local GAS_MIN_DRIFT = 0.6           -- floor while regulating wheelspin
local STARVED_SLIP = 0.06           -- wheelspin considered "starved" below this
local STARVED_TIME = 0.35           -- s at WOT without wheelspin -> downshift
local GEAR_COOLDOWN = 3.0           -- s without upshifts after a downshift
local OVERSPIN_MARGIN = 0.20        -- trim throttle above target + margin
local OVERSPIN_BLEED = 0.75

-- drift initiation
local INIT_TURN = 0.9               -- turn-in steering during initiation
local INIT_TIMEOUT = 2.5             -- s without slip -> retry other direction
local HANDBRAKE_PULSE = 0.9
local HANDBRAKE_PERIOD = 0.8        -- pulse every N seconds
local HANDBRAKE_DURATION = 0.35
local INIT_CONFIRM_SLIP = rad(12)   -- slip needed to confirm the drift

local REENTRY_KICK = 0.6            -- handbrake tap when the drift dies out

-- speed planning
local INIT_SPEED_KMH = 70           -- desired entry speed (capped by planning)
local SPEED_LIMIT_KMH = 170         -- hard cap
local BRAKE_DECEL = 8.0             -- planning deceleration, m/s^2
local WALL_MARGIN = 12              -- meters kept away from tight spots
local OPEN_CORRIDOR_M = 150         -- corridor longer than this -> full commitment
local CORNER_LATG = 9.0             -- lateral g used for the corner speed estimate
local NARROW_ROAD_M = 10            -- left+right clearance below this = narrow
local NARROW_LIMIT_KMH = 90
local LATERAL_MARGIN = 3.5          -- keep this away from the track edges

local GEAR_HOLDS = { [1] = 60, [2] = 100, [3] = 140, [4] = 180 } -- upshifts, km/h

local WHEEL_RL, WHEEL_RR = 2, 3     -- 0-based ac.StateCar.wheels order: FL,FR,RL,RR

-- ─────────────────── driver class ───────────────────

local Driver = {}
Driver.__index = Driver

function Driver.new(carIndex, config)
  local self = setmetatable({}, Driver)
  self.index = carIndex
  self.config = config
  self.bounds = config.bounds
  self.state = 'accelerate'     -- accelerate | initiate | drift | recover | lost
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
  self.gearCooldown = 0
  self.handbrakeTimer = 0
  self.initTries = 0
  self.slipLowTimer = 0
  self.progress = nil
  self.lateral = 0
  self.corridor = nil
  self.limitKmh = SPEED_LIMIT_KMH
  self.driftDir = 0             -- 1 left drift, -1 right drift, 0 unset
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
  local sl, sr
  local ok1 = pcall(function() sl = car.wheels[WHEEL_RL].slipRatio end)
  local ok2 = pcall(function() sr = car.wheels[WHEEL_RR].slipRatio end)
  if not ok1 or not ok2 then return nil end
  if type(sl) ~= 'number' or type(sr) ~= 'number' then return nil end
  return (abs(sl) + abs(sr)) * 0.5
end

-- Speed the car may carry so the braking distance fits the free corridor:
--   v = min(cap, sqrt(2 * BRAKE_DECEL * (corridor - WALL_MARGIN)))
local function corridorLimitedSpeed(corridorM)
  if not corridorM then return SPEED_LIMIT_KMH end
  local d = max(0, corridorM - WALL_MARGIN)
  return min(SPEED_LIMIT_KMH, math.sqrt(2 * BRAKE_DECEL * d) * 3.6)
end

-- Corner speed estimate from signed curvature (km/h), or nil on straights.
local function cornerSpeedKmh(curvature)
  if not curvature or abs(curvature) < 1e-4 then return nil end
  local v = math.sqrt(CORNER_LATG / max(abs(curvature), 2e-4))
  return clamp(v * 3.6, 40, SPEED_LIMIT_KMH)
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
  correction = clamp(correction, -0.95, 0.95)

  local steer
  if self.state == 'drift' then
    steer = clamp(self.driftDir * 0.5 + correction, -MAX_STEER, MAX_STEER)
  elseif self.state == 'initiate' then
    steer = clamp(self.driftDir * INIT_TURN, -MAX_STEER, MAX_STEER)
  elseif self.state == 'recover' then
    steer = clamp(-sign(self.slipAngle) * abs(self.slipAngle) * 1.2, -1, 1)
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

-- Speed plan: free corridor + grip corner speed + road width.
function Driver:_speedPlan(curvature)
  local limit = SPEED_LIMIT_KMH

  -- the braking distance must always fit the free corridor ahead
  local corridor = self.bounds and self.bounds:freeCorridor(self.progress, 300) or nil
  self.corridor = corridor
  limit = min(limit, corridorLimitedSpeed(corridor))

  -- grip-limited corner speed, kept drift-capable
  local cornerV = cornerSpeedKmh(curvature)
  if cornerV then
    limit = min(limit, max(cornerV, 50))
  end

  -- narrow sections: cap the speed so a mistake is survivable
  local sample = self.bounds and self.bounds:sampleAt(self.progress) or nil
  if sample and (sample.left + sample.right) < NARROW_ROAD_M then
    limit = min(limit, NARROW_LIMIT_KMH)
  end
  self.limitKmh = limit
  return limit
end

-- ─────────────────── main update ───────────────────

---@param car ac.StateCar
---@param dt number
function Driver:update(car, dt)
  self.stateTime = self.stateTime + dt
  self.gearCooldown = max(0, self.gearCooldown - dt)

  local slip = bodySlipAngle(car)
  self.prevSlipAngle = self.slipAngle
  self.slipAngle = slip
  self.slipRate = (slip - self.prevSlipAngle) / max(dt, 1e-3)
  self.rearSlip = rearWheelspin(car) or self.rearSlip
  local kmh = car.speedKmh

  -- keep the boundary reference fresh (built by the main script)
  self.bounds = self.config.bounds

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

  -- ── speed plan (used by several states) ──
  local curvature = Track.curvatureAhead(self.progress, self.config.trackLengthM, 30)
  local speedLimit = self:_speedPlan(curvature)
  local openTrack = self.corridor == nil or self.corridor >= OPEN_CORRIDOR_M

  -- ── state machine ──
  if self.state == 'accelerate' then
    local targetEntry = clamp(min(INIT_SPEED_KMH, max(cornerSpeedKmh(curvature) or INIT_SPEED_KMH, 55)), 40, speedLimit)
    if kmh >= targetEntry then
      self.state = 'initiate'
      self.stateTime = 0
      self.handbrakeTimer = 0
      self.initTries = 0
      self.driftDir = self:_pickDriftDirection(curvature)
      if self.driftDir == 0 then self.driftDir = (self.index % 2 == 0 and 1 or -1) end
    end

  elseif self.state == 'initiate' then
    self.driftDir = self:_pickDriftDirection(curvature)
    if self.driftDir == 0 then self.driftDir = (self.index % 2 == 0 and 1 or -1) end
    if abs(slip) > INIT_CONFIRM_SLIP then
      -- traction broken: the drift is on
      self.state = 'drift'
      self.stateTime = 0
      self.driftDir = sign(slip)
      self.driftDirLock = 1.5
      self.slipLowTimer = 0
    elseif self.stateTime > INIT_TIMEOUT then
      self.initTries = self.initTries + 1
      if self.initTries >= 2 then
        self.driftDir = -self.driftDir -- retry the other direction
        self.initTries = 0
      end
      self.state = 'accelerate'
      self.stateTime = 0
    elseif kmh < 30 then
      self.state = 'accelerate'
      self.stateTime = 0
    end

  elseif self.state == 'drift' then
    self.driftTime = self.driftTime + dt
    self.driftDirLock = max(0, self.driftDirLock - dt)
    if abs(slip) < rad(5) then
      self.slipLowTimer = self.slipLowTimer + dt
    else
      self.slipLowTimer = 0
    end
    if self.slipLowTimer > 4 and kmh > 30 then
      self.state = 'accelerate' -- build speed and re-enter
      self.stateTime = 0
    end
    if abs(slip) > rad(80) then
      self.state = 'recover'
      self.stateTime = 0
    end

  elseif self.state == 'recover' then
    if abs(slip) < rad(15) and kmh > 15 then
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

  -- ── navigation target ──
  local lookahead = clamp(8 + kmh * 0.06, 8, 24)
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
  local gas, brake, handbrake = 0, 0, 0
  self.requestedGearIndex = 0

  if self.state == 'accelerate' then
    -- full throttle: speed and wheelspin are both needed
    gas = 1.0
    if kmh > speedLimit + 10 then
      gas = 0.35
      brake = 0.3
    end

  elseif self.state == 'initiate' then
    -- break traction: full throttle, big turn-in, handbrake pulses
    gas = 1.0
    self.handbrakeTimer = self.handbrakeTimer + dt
    local phase = self.handbrakeTimer % HANDBRAKE_PERIOD
    if phase < HANDBRAKE_DURATION then
      handbrake = HANDBRAKE_PULSE
    end
    if kmh > speedLimit then
      brake = clamp((kmh - speedLimit) * 0.04, 0, 0.5)
      gas = 0.5
    end

  elseif self.state == 'drift' then
    -- wheelspin-first throttle: full throttle unless the rear wheels overspin
    gas = self.gas
    if self.rearSlip then
      local err = TARGET_WHEEL_SLIP - self.rearSlip
      if abs(err) > WHEEL_SLIP_DEADBAND then
        gas = gas + err * WHEEL_SLIP_GAIN * dt -- integrates UP on its own
      end
      -- wide-open throttle and still no wheelspin -> lower gear immediately
      if self.rearSlip < STARVED_SLIP and gas >= 0.95 then
        self.starvedTimer = self.starvedTimer + dt
        if self.starvedTimer > STARVED_TIME then
          self.requestedGearIndex = max(1, car.gear - 1)
          self.starvedTimer = 0
          self.gearCooldown = GEAR_COOLDOWN
        end
      else
        self.starvedTimer = 0
      end
      gas = clamp(gas, GAS_MIN_DRIFT, 1)
      -- overspin: trim so the car keeps pulling instead of burning the tyres
      if self.rearSlip > TARGET_WHEEL_SLIP + OVERSPIN_MARGIN then
        gas = gas * OVERSPIN_BLEED
      end
    else
      -- no wheel telemetry: keep it floored, the drift is steering-controlled
      gas = 1.0
    end
    -- corridor plan: brake to the limit, keep the arc inside the free road
    if kmh > speedLimit then
      gas = gas * 0.45
      brake = max(brake, clamp((kmh - speedLimit) * 0.05, 0, 0.7))
    end
    -- open road with slip established: commit to a long, fast drift
    if openTrack and abs(slip) > rad(15) then
      gas = max(gas, 0.95)
    end
    -- drift dying out: tap the handbrake to bring the slip angle back
    if abs(slip) < rad(8) and kmh > 45 then
      handbrake = REENTRY_KICK
    end
    -- stay away from the track edges
    if halfWidth then
      local margin = halfWidth - LATERAL_MARGIN
      if abs(self.lateral) > margin then
        self.steer = clamp(self.steer - sign(self.lateral) * 0.25, -1, 1)
        brake = max(brake, 0.25)
      end
    end

  elseif self.state == 'recover' then
    brake = clamp(0.3 + abs(slip) * 0.3, 0.2, 0.8)
    gas = 0

  elseif self.state == 'lost' then
    self.steer = 0
    gas = clamp((40 - kmh) * 0.05, 0, 0.5)
  end

  -- spin-out prevention even mid-drift: cut gas at extreme slip
  if abs(slip) > rad(65) then gas = min(gas, 0.25) end

  self.gas = clamp(gas, 0, 1)
  self.brake = clamp(brake, 0, 1)
  self.handbrake = clamp(handbrake, 0, 1)

  -- ── gearbox: short gears for torque, direct gear requests ──
  if self.requestedGearIndex > 0 and car.gear == self.requestedGearIndex then
    self.requestedGearIndex = 0 -- reached, let the box live again
  end
  if (self.state == 'accelerate' or self.state == 'initiate' or self.state == 'drift')
    and self.requestedGearIndex == 0 and self.gearCooldown == 0 then
    local gear = car.gear
    if gear >= 1 and gear <= 4 and GEAR_HOLDS[gear] and kmh > GEAR_HOLDS[gear] then
      self.requestedGearIndex = gear + 1
    end
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
