-- RoutePilot/racing_line.lua
-- Short-horizon corridor optimizer. It optimizes the actual driven path,
-- not the recorded reference spline. All candidate points are constrained
-- by the recorded physical road bounds and the vehicle footprint.
local RacingLine = {}

local function clamp(v, a, b)
  return math.max(a, math.min(b, v))
end

local function safe(v, fallback)
  if type(v) ~= "number" or v ~= v or v == math.huge or v == -math.huge then
    return fallback or 0
  end
  return v
end

local function pointAt(sample, offset)
  return {
    x = sample.x - sample.tz * offset,
    z = sample.z + sample.tx * offset,
    offset = offset,
    s = sample.s,
    sample = sample
  }
end

local function curvature(a, b, c)
  local abx, abz = b.x - a.x, b.z - a.z
  local bcx, bcz = c.x - b.x, c.z - b.z
  local cax, caz = a.x - c.x, a.z - c.z
  local ab = math.sqrt(abx * abx + abz * abz)
  local bc = math.sqrt(bcx * bcx + bcz * bcz)
  local ca = math.sqrt(cax * cax + caz * caz)
  if ab < 0.05 or bc < 0.05 or ca < 0.05 then return 0 end
  local cross = abx * bcz - abz * bcx
  return 2 * cross / math.max(0.001, ab * bc * ca)
end

local function localCost(samples, bounds, offsets, index, baseOffset)
  local n = #samples
  if index <= 1 or index >= n then return 0 end
  local a = pointAt(samples[index - 1], offsets[index - 1])
  local b = pointAt(samples[index], offsets[index])
  local c = pointAt(samples[index + 1], offsets[index + 1])
  local k = curvature(a, b, c)
  local second = offsets[index - 1] - 2 * offsets[index] + offsets[index + 1]
  local clearance = math.min(offsets[index] - bounds[index][1],
    bounds[index][2] - offsets[index])
  local edgePenalty = 0.00008 / math.max(0.12, clearance + 0.12)
  local laneBias = (offsets[index] - baseOffset) ^ 2 * 0.0002
  return k * k * 180.0 + second * second * 0.055 +
    edgePenalty + laneBias
end

-- boundsAt(sample) returns left/right offsets in the spline's local frame.
-- fixedOffset is used while passing: it disables racing-line optimization and
-- follows the reserved passing corridor instead.
function RacingLine.plan(route, sampleFn, progress, car, boundsAt, startOffset, baseOffset,
    fixedOffset, horizon, spacing)
  if not route or not progress or not boundsAt then return nil end
  local size = car and car.aabbSize
  local halfWidth = math.max(0.75, safe(size and size.x, 2.0) * 0.5)
  local margin = halfWidth + 0.42
  local step = math.max(3.0, spacing or 5.0)
  local length = math.max(35.0, horizon or 85.0)
  local count = math.floor(length / step) + 1
  local samples, bounds, offsets = {}, {}, {}
  local blockedDistance=nil
  local initial = safe(startOffset, safe(baseOffset, 0))
  local preferred = safe(baseOffset, initial)

  for i = 0, count do
    local d = i * step
    local sample = sampleFn and sampleFn(route, progress + d) or nil
    if not sample then break end
    local left, right = boundsAt(sample)
    left, right = safe(left, -2.0), safe(right, 2.0)
    local lo, hi = left + margin, right - margin
    if lo > hi then
      -- A near-field corridor that cannot contain the car is an immediate
      -- hard stop. A distant narrowing becomes a braking constraint instead
      -- of stopping the car hundreds of metres early.
      if d <= 10.0 then return nil, "NO_FIT" end
      blockedDistance=blockedDistance or d
      local center=(left+right)*0.5
      lo,hi=center,center
    end
    samples[#samples + 1] = sample
    bounds[#bounds + 1] = { lo, hi }
    local t = clamp(d / 22.0, 0, 1)
    local desired
    if fixedOffset ~= nil then
      desired=initial+(safe(fixedOffset,initial)-initial)*t
    elseif i==0 then
      desired=initial
    else
      -- Seed the optimizer on the outside of the upcoming bend. Starting
      -- every point in the same lane creates a local minimum that cannot
      -- discover a smooth corner-cutting path through coordinate descent.
      local previous=sampleFn(route,progress+math.max(0,d-step))
      local following=sampleFn(route,progress+d+step)
      local kp=safe(previous and previous.k)
      local kc=safe(sample.k)
      local kn=safe(following and following.k,kc)
      local smoothCurvature=(kp+2.0*kc+kn)*0.25
      local cornerBias=clamp(-smoothCurvature*90.0,-2.8,2.8)
      desired=preferred+cornerBias
    end
    offsets[#offsets + 1] = clamp(desired, lo, hi)
  end

  local n = #samples
  if n < 3 then return nil end

  if fixedOffset == nil then
    -- Coordinate descent over the complete available corridor. The objective
    -- minimizes curvature of the actual offset path, penalizes abrupt lateral
    -- changes and keeps a weak preference for the normal cruising lane.
    for _ = 1, 7 do
      for i = 2, n - 1 do
        local lo, hi = bounds[i][1], bounds[i][2]
        local best, bestCost = offsets[i], math.huge
        local candidates = 13
        for c = 0, candidates - 1 do
          local candidate = lo + (hi - lo) * c / (candidates - 1)
          offsets[i] = candidate
          local cost = localCost(samples, bounds, offsets, i, preferred)
          if cost < bestCost then
            best, bestCost = candidate, cost
          end
        end
        offsets[i] = best
      end
    end
  end

  local points = {}
  for i = 1, n do
    points[i] = pointAt(samples[i], offsets[i])
  end
  local curvatures = {}
  for i = 2, n - 1 do
    curvatures[i] = curvature(points[i - 1], points[i], points[i + 1])
  end
  curvatures[1] = curvatures[2] or 0
  curvatures[n] = curvatures[n - 1] or 0

  return {
    points = points,
    curvature = curvatures,
    spacing = step,
    count = n,
    horizon = (n - 1) * step,
    fixed = fixedOffset ~= nil,
    blockedDistance = blockedDistance
  }
end

function RacingLine.sample(plan, distance)
  if not plan or not plan.points or plan.count < 2 then return nil end
  local usableDistance=plan.blockedDistance and
    math.max(0,plan.blockedDistance-10.0) or distance
  local position = clamp(usableDistance / plan.spacing, 0, plan.count - 1)
  local index = math.floor(position) + 1
  local nextIndex = math.min(plan.count, index + 1)
  local t = position - math.floor(position)
  local a, b = plan.points[index], plan.points[nextIndex]
  local ca = plan.curvature[index] or 0
  local cb = plan.curvature[nextIndex] or ca
  return {
    x = a.x + (b.x - a.x) * t,
    z = a.z + (b.z - a.z) * t,
    offset = a.offset + (b.offset - a.offset) * t,
    k = ca + (cb - ca) * t
  }
end

return RacingLine
