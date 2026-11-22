-- DriftAI/bounds.lua
-- Boundary map for the drift app.
--
-- At session start the app scans the whole track once and fixes the drivable
-- area with physics raycasts (physics.raycastTrack, SDK ac_physics.lua,
-- api_physics_raycast.cpp):
--   * a vertical ray confirms there is a drivable surface below the sample;
--   * horizontal rays to the left, right and forward measure the distance
--     to walls and other solid obstacles.
-- The map is sampled every 8 m along the track spline and is built over a
-- few frames (no hitch). Drivers use `freeCorridor()` for speed planning:
-- the braking distance always fits the length of the free track ahead, so
-- the car can commit to a long, fast drift on open sections and slows down
-- before tight spots. If the track has no spline, the map stays empty and
-- drivers fall back to conservative speed limits.

local Track = require('track')

local Bounds = {}
Bounds.__index = Bounds

local SIDE_RAY_LENGTH = 35    -- sideways wall search
local AHEAD_RAY_LENGTH = 140  -- forward wall search
local SAMPLE_STEP_M = 8       -- one sample per N meters of track
local BUILD_BUDGET = 24       -- samples built per frame

function Bounds.new(trackLengthM)
  local self = setmetatable({}, Bounds)
  self.trackLengthM = trackLengthM or 0
  self.total = self.trackLengthM > 0
    and math.max(3, math.floor(self.trackLengthM / SAMPLE_STEP_M)) or 0
  self.samples = {}
  self.built = 0
  self.good = 0
  self.ready = self.total == 0
  return self
end

local function castDistance(origin, dirX, dirZ, length)
  local ok, d = pcall(physics.raycastTrack, origin, vec3(dirX, 0, dirZ), length)
  if not ok or type(d) ~= 'number' or d < 0 or d >= length then
    return length -- nothing solid within reach
  end
  return d
end

function Bounds:_buildSample(i)
  local progress = (i + 0.5) / self.total
  local pos = Track.positionAt(progress)
  if not pos then return false end

  -- track direction at this sample
  local nextPos = Track.positionAt((progress + 4 / self.trackLengthM) % 1) or pos
  local dx, dz = nextPos.x - pos.x, nextPos.z - pos.z
  local l = math.sqrt(dx * dx + dz * dz)
  if l > 1e-4 then dx, dz = dx / l, dz / l else dx, dz = 1, 0 end

  -- vertical collision check: must be drivable surface below
  local okS, dS = pcall(physics.raycastTrack, pos + vec3(0, 3, 0), vec3(0, -1, 0), 8)
  if not okS or type(dS) ~= 'number' or dS < 0 then return false end

  local origin = pos + vec3(0, 0.4, 0)
  local left = castDistance(origin, dz, -dx, SIDE_RAY_LENGTH)
  local right = castDistance(origin, -dz, dx, SIDE_RAY_LENGTH)
  local ahead = castDistance(origin, dx, dz, AHEAD_RAY_LENGTH)

  self.samples[i + 1] = {
    progress = progress, x = pos.x, z = pos.z, dirX = dx, dirZ = dz,
    left = left, right = right, ahead = ahead,
  }
  return true
end

---Build the next slice of the map; call every frame until `ready`.
function Bounds:buildStep()
  if self.ready or self.total == 0 then return end
  local done = 0
  while done < BUILD_BUDGET and self.built < self.total do
    if self:_buildSample(self.built) then self.good = self.good + 1 end
    self.built = self.built + 1
    done = done + 1
  end
  if self.built >= self.total then self.ready = true end
end

---Sample closest to the given normalized progress, or nil.
function Bounds:sampleAt(progress)
  if not self.ready or not progress then return nil end
  local i = math.max(0, math.min(self.total - 1, math.floor(progress * self.total)))
  return self.samples[i + 1]
end

---Length in meters of the free track corridor starting at `progress`:
-- the distance to the first sample where the forward ray hits something
-- close (tight corner, wall closing in), capped at `maxMeters`.
-- Returns nil when the map is not usable.
function Bounds:freeCorridor(progress, maxMeters)
  if not self.ready or not progress or self.good == 0 then return nil end
  local maxSamples = math.max(1, math.floor((maxMeters or 200) / SAMPLE_STEP_M))
  local idx = math.floor(progress * self.total)
  -- distance covered by full samples ahead of the current one
  for k = 0, maxSamples - 1 do
    local s = self.samples[((idx + k) % self.total) + 1]
    if not s then return k * SAMPLE_STEP_M end
    -- a tight spot: not much free road straight ahead from this sample
    if s.ahead < 45 and k > 0 then
      return k * SAMPLE_STEP_M
    end
    if k == maxSamples - 1 then
      return maxSamples * SAMPLE_STEP_M
    end
  end
  return maxSamples * SAMPLE_STEP_M
end

---Minimum forward wall distance over the next `meters` of track, or nil.
function Bounds:wallAheadOver(progress, meters)
  if not self.ready or not progress or self.good == 0 then return nil end
  local count = math.max(1, math.ceil(meters / SAMPLE_STEP_M))
  local idx = math.floor(progress * self.total)
  local best = nil
  for k = 0, count - 1 do
    local s = self.samples[((idx + k) % self.total) + 1]
    if s and (not best or s.ahead < best) then best = s.ahead end
  end
  return best
end

---Build progress 0..1 (for the UI).
function Bounds:progressFraction()
  if self.total == 0 then return 1 end
  return self.built / self.total
end

return Bounds
