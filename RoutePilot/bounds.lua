-- RoutePilot/bounds.lua
-- One-time track boundary map for wall-aware drifting.
--
-- Built ONCE, a few frames after the session starts (before any interception):
--   * samples the track every 5 m of progress using the AI spline APIs;
--   * for every sample: left/right drivable width via ac.getTrackAISplineSides,
--     verified/strengthened by horizontal raycasts (physics.raycastTrack)
--     in left/right/forward directions — vertical collisions (walls) stop
--     the ray, giving true distances to obstacles;
--   * stores per-sample: position, forward direction, distances left/right
--     and the lateral "drift room".
--
-- The drift AI uses live raycasts every frame for immediate wall reaction
-- and this map for the overall geometry (best drift line planning).
--
-- All API calls are pcall-guarded: on tracks without a spline the map
-- degrades to pure raycasts from the live car position (works on drift
-- playgrounds which have no fast_lane).

local Bounds = {}
Bounds.__index = Bounds

local SAMPLE_STEP = 0.004      -- progress step (~5 m on a 1.2 km track)
local RAY_HEIGHT = 1.0         -- raycast origin height above surface
local RAY_LENGTH = 60         -- how far to look for walls

function Bounds.new(logger)
  local self = setmetatable({}, Bounds)
  self.log = logger
  self.samples = {}
  self.built = false
  self.inProgress = false
  self.cursor = 0
  self.count = 0
  return self
end

-- true when the map is ready for use
function Bounds:ready()
  return self.built
end

-- Kick off the build; call :update(dt) every frame afterwards.
function Bounds:startBuild()
  if self.built or self.inProgress then return end
  self.inProgress = true
  self.cursor = 0
  if self.log then self.log:log("BOUNDS: building track map...") end
end

local function ray(pos, dir, len)
  local ok, d = pcall(physics.raycastTrack, pos, dir, len)
  if ok and type(d) == 'number' then return d end
  return -1
end

-- Build one sample at progress t (0..1). Returns nothing.
function Bounds:_sampleAt(t)
  local okPos, pos = pcall(ac.trackProgressToWorldCoordinate, t)
  if not okPos or not pos then return end
  -- lift the origin above the surface for horizontal rays
  pos = vec3(pos.x, pos.y + RAY_HEIGHT, pos.z)

  -- forward direction from the next point on the spline
  local fwd = nil
  local okFwd, npos = pcall(ac.trackProgressToWorldCoordinate, (t + SAMPLE_STEP) % 1)
  if okFwd and npos then
    fwd = vec3(npos.x - pos.x, 0, npos.z - pos.z)
    local len = math.sqrt(fwd.x * fwd.x + fwd.z * fwd.z)
    if len > 0.001 then
      fwd = vec3(fwd.x / len, 0, fwd.z / len)
    else
      fwd = nil
    end
  end
  if not fwd then return end

  local lx, lz = -fwd.z, fwd.x     -- left normal
  local rx, rz = fwd.z, -fwd.x     -- right normal

  -- horizontal raycasts for walls: left, right, forward
  local dLeft = ray(pos, vec3(lx, 0, lz), RAY_LENGTH)
  local dRight = ray(pos, vec3(rx, 0, rz), RAY_LENGTH)
  local dFwd = ray(pos, vec3(fwd.x, 0, fwd.z), RAY_LENGTH)
  if dLeft < 0 then dLeft = RAY_LENGTH end
  if dRight < 0 then dRight = RAY_LENGTH end
  if dFwd < 0 then dFwd = RAY_LENGTH end

  -- spline sides as a secondary source (may be unavailable)
  local okS, sides = pcall(ac.getTrackAISplineSides, t)
  if okS and sides and sides.x and sides.y then
    local sLeft = math.max(0, sides.x)
    local sRight = math.max(0, sides.y)
    -- prefer the more conservative (closer) of the two sources
    dLeft = math.min(dLeft, sLeft)
    dRight = math.min(dRight, sRight)
  end

  local total = math.max(dLeft, 0) + math.max(dRight, 0)
  self.samples[#self.samples + 1] = {
    t = t,
    x = pos.x, y = pos.y, z = pos.z,
    fx = fwd.x, fz = fwd.z,
    left = dLeft, right = dRight,
    room = total,
  }
  self.count = self.count + 1
end

-- Call every frame; builds a few samples per frame so the build is quick
-- but never freezes the game.
function Bounds:update(dt)
  if self.built or not self.inProgress then return end
  local budget = 16   -- samples per frame: 250 samples over ~16 frames
  for _ = 1, budget do
    if self.cursor >= 1.0 then
      self.built = true
      self.inProgress = false
      if self.log then
        self.log:log("BOUNDS: map built, %d samples", self.count)
        self.log:flush()
      end
      return
    end
    self:_sampleAt(self.cursor)
    self.cursor = self.cursor + SAMPLE_STEP
  end
end

-- Live raycasts from the car (used EVERY FRAME for instant wall reaction;
-- also the fallback for tracks without a spline).
-- Returns: dLeft, dRight, dFwd (meters to nearest wall, RAY_LENGTH if none)
function Bounds:liveRays(car)
  local pos = car.position
  local p = vec3(pos.x, pos.y + RAY_HEIGHT, pos.z)
  local look = car.look
  local len = math.sqrt(look.x * look.x + look.z * look.z)
  if len < 0.001 then return RAY_LENGTH, RAY_LENGTH, RAY_LENGTH end
  local fx, fz = look.x / len, look.z / len
  local dFwd = ray(p, vec3(fx, 0, fz), RAY_LENGTH)
  local dLeft = ray(p, vec3(-fz, 0, fx), RAY_LENGTH)
  local dRight = ray(p, vec3(fz, 0, -fx), RAY_LENGTH)
  if dLeft < 0 then dLeft = RAY_LENGTH end
  if dRight < 0 then dRight = RAY_LENGTH end
  if dFwd < 0 then dFwd = RAY_LENGTH end
  return dLeft, dRight, dFwd
end

-- Nearest map sample to a world position. Returns the sample or nil.
function Bounds:nearest(pos)
  local best, bestD = nil, math.huge
  for i = 1, #self.samples do
    local s = self.samples[i]
    local dx = s.x - pos.x
    local dz = s.z - pos.z
    local d = dx * dx + dz * dz
    if d < bestD then bestD, best = d, s end
  end
  return best
end

return Bounds
