-- DriftAI/track.lua
-- Track navigation for the drift app: finds where the AI car is relative to the
-- track and where the track goes, without any prepared AI fast lane or route files.
--
-- Two providers (all available to Lua apps in CSP 0.3.2-preview2):
--   1. CSP track coordinates (the track's own fast_lane spline):
--      ac.hasTrackSpline(), ac.worldCoordinateToTrackProgress(pos),
--      ac.trackProgressToWorldCoordinate(progress), ac.trackCoordinateToWorld(vec3).
--   2. Raycast fallback for tracks without a spline: sample the ground with
--      render.createRay():physics() (SDK: common/ac_ray.lua).
--
-- The app never requires a custom fastline.ai and never writes track files.

local atan2, sqrt = math.atan2, math.sqrt

local Track = {}

local splineCache = { valid = false, checked = false }

-- ─────────────────────────── spline provider ───────────────────────────

local function checkSpline()
  if splineCache.checked then return splineCache.valid end
  splineCache.checked = true
  splineCache.valid = ac.hasTrackSpline and ac.hasTrackSpline() or false
  return splineCache.valid
end

---Normalized position on the track spline for a world position, or nil.
---@param pos vec3
---@return number? progress normalized 0..1
---@return number? lateralOffset meters, signed (positive = right of the spline direction)
function Track.getTrackPosition(pos)
  if not checkSpline() then return nil end
  local ok, progress = pcall(ac.worldCoordinateToTrackProgress, pos)
  if not ok or type(progress) ~= 'number' or progress < 0 then return nil end
  local okCenter, center = pcall(ac.trackProgressToWorldCoordinate, progress)
  if not okCenter or not center then return progress, 0 end
  local dx = pos.x - center.x
  local dz = pos.z - center.z
  local dirx, dirz = 1, 0
  local okPrev, prev = pcall(ac.trackProgressToWorldCoordinate, progress - 0.0005)
  if okPrev and prev then
    dirx, dirz = center.x - prev.x, center.z - prev.z
    local l = sqrt(dirx * dirx + dirz * dirz)
    if l > 1e-6 then dirx, dirz = dirx / l, dirz / l end
  end
  local lateral = dirx * dz - dirz * dx
  return progress, lateral
end

---World coordinate at a normalized progress.
---@param progress number
---@return vec3? position
function Track.positionAt(progress)
  if not checkSpline() then return nil end
  local ok, pos = pcall(ac.trackProgressToWorldCoordinate, math.clamp(progress, 0, 1))
  if ok then return pos end
  return nil
end

---Half-width of the drivable area at the current progress, or nil.
---Uses trackCoordinateToWorld with lateral extremes (-1 and 1) mapping to
---the left/right edges of the drivable surface around the spline.
---@param progress number
---@return number? halfWidth meters
function Track.halfWidthAt(progress)
  if not checkSpline() then return nil end
  local okL, left = pcall(ac.trackCoordinateToWorld, vec3(-1, 0, progress))
  local okR, right = pcall(ac.trackCoordinateToWorld, vec3(1, 0, progress))
  if not okL or not okR or not left or not right then return nil end
  local dx = right.x - left.x
  local dz = right.z - left.z
  return sqrt(dx * dx + dz * dz) / 2
end

-- ─────────────────────────── raycast fallback ───────────────────────────

local RAY_LENGTH = 40

---Sample local ground below the car via a physics raycast.
---Returns true if the car is over a drivable physical surface.
---@param pos vec3 @car position
---@return boolean
function Track.groundBelow(pos)
  local ok, ray = pcall(render.createRay, pos + vec3(0, 1, 0), vec3(0, -1, 0), RAY_LENGTH)
  if not ok or not ray then return false end
  local hitPos = vec3()
  local hitNormal = vec3()
  return ray:physics(hitPos, hitNormal, false, false) >= 0
end

---Target point ahead of the car for steering, plus half-width estimation.
---Spline mode when available; straight-ahead fallback otherwise.
---@param pos vec3
---@param progress number? normalized track progress
---@param lookaheadMeters number
---@param trackLengthM number
---@return number targetX, number targetZ, number? halfWidth
function Track.lookaheadTarget(pos, progress, lookaheadMeters, trackLengthM)
  if checkSpline() and progress and trackLengthM and trackLengthM > 0 then
    local ahead = (progress + lookaheadMeters / trackLengthM) % 1
    local target = Track.positionAt(ahead)
    if target then
      local hw = Track.halfWidthAt(progress)
      return target.x, target.z, hw
    end
  end
  return pos.x, pos.z, nil
end

---Signed curvature estimate over the next section (1/radius, signed).
---Positive = left turn, negative = right turn. Nil when the spline is missing.
---@param progress number
---@param trackLengthM number
---@param distance number meters ahead to sample
---@return number? signedCurvature
function Track.curvatureAhead(progress, trackLengthM, distance)
  if not checkSpline() then return nil end
  if not trackLengthM or trackLengthM <= 0 then return nil end
  local step = distance / trackLengthM
  local p0 = Track.positionAt(progress % 1)
  local p1 = Track.positionAt((progress + step) % 1)
  local p2 = Track.positionAt((progress + step * 2) % 1)
  if not p0 or not p1 or not p2 then return nil end

  local ax, az = p1.x - p0.x, p1.z - p0.z
  local bx, bz = p2.x - p1.x, p2.z - p1.z
  local la, lb = sqrt(ax * ax + az * az), sqrt(bx * bx + bz * bz)
  if la < 1e-3 or lb < 1e-3 then return nil end

  local cross = ax * bz - az * bx
  local dot = ax * bx + az * bz
  local angle = atan2(cross, dot)
  return angle / ((la + lb) * 0.5)
end

---Normalized track progress of a car (spline mode), or nil.
---@param pos vec3
---@return number? progress
function Track.progressOf(pos)
  local progress = Track.getTrackPosition(pos)
  if progress == nil then return nil end
  return progress
end

return Track
