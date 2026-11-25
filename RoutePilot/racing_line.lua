-- RoutePilot/racing_line.lua
-- Whole-sequence racing corridor optimizer.
-- The planner treats consecutive bends as one path-planning problem and
-- searches the complete legal road width while preserving a hard body margin.
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

local function addCandidate(list, value, lo, hi)
  local v = clamp(value, lo, hi)
  for j = 1, #list do
    if math.abs(list[j] - v) < 0.035 then return end
  end
  list[#list + 1] = v
end

-- boundsAt(sample) returns legal left/right offsets in the route spline frame.
-- fixedOffset is reserved for an active overtake: the racing-line search is
-- disabled, but the transition to the reserved corridor remains smooth.
function RacingLine.plan(route, sampleFn, progress, car, boundsAt, startOffset, baseOffset,
    fixedOffset, horizon, spacing, obstacleAt, lateralAccelLimit)
  if not route or not progress or not boundsAt then return nil end

  local size = car and car.aabbSize
  local halfWidth = math.max(0.75, safe(size and size.x, 2.0) * 0.5)
  local halfLength = math.max(1.5, safe(size and size.z, 4.5) * 0.5)
  local marginBase = halfWidth + 0.52
  local step = math.max(3.5, spacing or 4.0)
  local length = math.min(135.0, math.max(45.0, horizon or 120.0))
  local count = math.floor(length / step) + 1
  local samples, bounds = {}, {}
  local blockedDistance = nil
  local initial = safe(startOffset, safe(baseOffset, 0))
  local preferred = safe(baseOffset, initial)
  local referenceSpeed = clamp(math.max(22.0, safe(car and car.speedMs, 0) * 1.15), 22.0, 65.0)
  local lateralLimit = clamp(safe(lateralAccelLimit, 8.0), 6.75, 18.0)

  for i = 0, count do
    local d = i * step
    local sample = sampleFn and sampleFn(route, progress + d) or nil
    if not sample then break end
    local left, right = boundsAt(sample)
    left, right = safe(left, -2.0), safe(right, 2.0)
    -- Add a conservative yaw envelope: a car following a curved path is not
    -- a point or a rectangle permanently aligned with the route reference.
    local yawEnvelope = math.min(0.45, halfLength * halfLength *
      math.abs(safe(sample.k)) * 0.35)
    local margin = marginBase + yawEnvelope
    local lo, hi = left + margin, right - margin
    if lo > hi then
      if d <= 10.0 then return nil, "NO_FIT" end
      blockedDistance = blockedDistance or d
      local center = (left + right) * 0.5
      lo, hi = center, center
    end
    samples[#samples + 1] = sample
    bounds[#bounds + 1] = { lo, hi }
  end

  local n = #samples
  if n < 3 then return nil, "SHORT_PLAN" end

  local offsets = {}
  if fixedOffset ~= nil then
    for i = 1, n do
      local d = (i - 1) * step
      local t = clamp(d / 22.0, 0, 1)
      local desired = initial + (safe(fixedOffset, initial) - initial) * t
      offsets[i] = clamp(desired, bounds[i][1], bounds[i][2])
    end
  else
    -- Nine lateral samples plus the current/preferred offsets give enough
    -- resolution to exploit the whole road without an expensive dense grid.
    local candidateSets = {}
    candidateSets[1] = { clamp(initial, bounds[1][1], bounds[1][2]) }

    for i = 2, n do
      local lo, hi = bounds[i][1], bounds[i][2]
      local list = {}
      local d = (i - 1) * step
      local forbidden = obstacleAt and obstacleAt(samples[i], d) or nil
      local function isBlocked(value)
        for _, interval in ipairs(forbidden or {}) do
          if value > interval[1] and value < interval[2] then return true end
        end
        return false
      end

      for c = 0, 8 do
        local value = lo + (hi - lo) * c / 8
        if not isBlocked(value) then addCandidate(list, value, lo, hi) end
      end
      if not isBlocked(preferred) then addCandidate(list, preferred, lo, hi) end

      local previous = sampleFn(route, progress + math.max(0, d - step))
      local following = sampleFn(route, progress + d + step)
      local kp = safe(previous and previous.k)
      local kc = safe(samples[i].k)
      local kn = safe(following and following.k, kc)
      local smoothCurvature = (kp + 2.0 * kc + kn) * 0.25
      local curvatureCandidate = clamp(
        preferred - clamp(smoothCurvature * 105.0, -4.0, 4.0), lo, hi)
      if not isBlocked(curvatureCandidate) then
        addCandidate(list, curvatureCandidate, lo, hi)
      end

      -- Keep a feasible candidate even if traffic blocks almost the entire
      -- cross-section. Longitudinal following control then handles the queue.
      if #list == 0 then
        local edge = lo
        for _, interval in ipairs(forbidden or {}) do
          if edge >= interval[1] and edge <= interval[2] then
            edge = clamp(interval[2] + 0.08, lo, hi)
          end
        end
        addCandidate(list, edge, lo, hi)
      end
      candidateSets[i] = list
    end

    -- Dynamic programming over consecutive offset pairs. Each transition
    -- evaluates the curvature of the actual path through three stations.
    local firstLayer = {}
    for j = 1, #candidateSets[2] do
      local off0 = candidateSets[1][1]
      local off1 = candidateSets[2][j]
      local clearance = math.min(off1 - bounds[2][1], bounds[2][2] - off1)
      local shift = off1 - off0
      local excessShift = math.max(0, math.abs(shift) - step * 0.48)
      local cost = shift * shift * 0.035 + excessShift * excessShift * 0.8 +
        0.018 / math.max(0.20, clearance + 0.20)
      firstLayer[#firstLayer + 1] = { a = 1, b = j, cost = cost, prev = nil }
    end
    if #firstLayer == 0 then
      for j = 1, #candidateSets[2] do
        local off0 = candidateSets[1][1]
        local off1 = candidateSets[2][j]
        local shift = off1 - off0
        firstLayer[#firstLayer + 1] = {
          a = 1, b = j, cost = shift * shift * 0.035, prev = nil
        }
      end
    end

    local layer = firstLayer
    local keyStride = 32
    for station = 2, n - 1 do
      local nextLayer, bestByPair = {}, {}
      for _, node in ipairs(layer) do
        local offA = candidateSets[station - 1][node.a]
        local offB = candidateSets[station][node.b]
        local pointA = pointAt(samples[station - 1], offA)
        local pointB = pointAt(samples[station], offB)
        for c = 1, #candidateSets[station + 1] do
          local offC = candidateSets[station + 1][c]
          local shift = offC - offB
          do
            local pointC = pointAt(samples[station + 1], offC)
            local k = curvature(pointA, pointB, pointC)
            local second = offA - 2.0 * offB + offC
            local slope = shift / step
            local clearance = math.min(offC - bounds[station + 1][1],
              bounds[station + 1][2] - offC)

            -- Approximate time loss at the car's current high-speed operating
            -- point. This makes the optimizer prefer a single broad chord
            -- through a linked left-right sequence over tracing both bends.
            local curveSpeed = math.sqrt(lateralLimit /
              math.max(0.00035, math.abs(k)))
            local timeLoss = math.max(0, 1.0 / curveSpeed - 1.0 / referenceSpeed) * step
            local curvatureCost = k * k * 150.0
            local excessShift = math.max(0, math.abs(shift) - step * 0.48)
            local smoothCost = second * second * 0.11 + slope * slope * 0.055 +
              excessShift * excessShift * 0.8
            local edgeCost = 0.018 / math.max(0.20, clearance + 0.20)
            local cost = node.cost + curvatureCost + timeLoss * 34.0 +
              smoothCost + edgeCost

            local key = node.b * keyStride + c
            local existing = bestByPair[key]
            if not existing or cost < existing.cost then
              bestByPair[key] = { a = node.b, b = c, cost = cost, prev = node }
            end
          end
        end
      end
      for _, node in pairs(bestByPair) do nextLayer[#nextLayer + 1] = node end
      if #nextLayer == 0 then return nil, "NO_PATH" end
      layer = nextLayer
    end

    local best
    for _, node in ipairs(layer) do
      if not best or node.cost < best.cost then best = node end
    end
    if not best then return nil, "NO_PATH" end

    local node, station = best, n
    while node and station >= 2 do
      offsets[station] = candidateSets[station][node.b]
      offsets[station - 1] = candidateSets[station - 1][node.a]
      node = node.prev
      station = station - 1
    end
    offsets[1] = candidateSets[1][1]
  end

  local points = {}
  for i = 1, n do points[i] = pointAt(samples[i], offsets[i]) end
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

local function hermite(a, b, m0, m1, t)
  local t2, t3 = t * t, t * t * t
  local h00 = 2 * t3 - 3 * t2 + 1
  local h10 = t3 - 2 * t2 + t
  local h01 = -2 * t3 + 3 * t2
  local h11 = t3 - t2
  local value = h00 * a + h10 * m0 + h01 * b + h11 * m1
  local dh00 = 6 * t2 - 6 * t
  local dh10 = 3 * t2 - 4 * t + 1
  local dh01 = -6 * t2 + 6 * t
  local dh11 = 3 * t2 - 2 * t
  local first = dh00 * a + dh10 * m0 + dh01 * b + dh11 * m1
  local ddh00 = 12 * t - 6
  local ddh10 = 6 * t - 4
  local ddh01 = -12 * t + 6
  local ddh11 = 6 * t - 2
  local second = ddh00 * a + ddh10 * m0 + ddh01 * b + ddh11 * m1
  return value, first, second
end

function RacingLine.sample(plan, distance)
  if not plan or not plan.points or plan.count < 2 then return nil end
  local usableDistance = plan.blockedDistance and
    math.max(0, plan.blockedDistance - 10.0) or distance
  local position = clamp(usableDistance / plan.spacing, 0, plan.count - 1)
  local index = math.min(plan.count - 1, math.floor(position) + 1)
  local t = clamp(position - (index - 1), 0, 1)
  local points = plan.points
  local p0, p1 = points[index], points[index + 1]
  local prev = points[math.max(1, index - 1)]
  local nextNext = points[math.min(plan.count, index + 2)]

  local m0x, m0z = (p1.x - prev.x) * 0.5, (p1.z - prev.z) * 0.5
  local m1x, m1z = (nextNext.x - p0.x) * 0.5, (nextNext.z - p0.z) * 0.5
  local x, dx, ddx = hermite(p0.x, p1.x, m0x, m1x, t)
  local z, dz, ddz = hermite(p0.z, p1.z, m0z, m1z, t)
  local offset = hermite(p0.offset, p1.offset,
    (p1.offset - prev.offset) * 0.5,
    (nextNext.offset - p0.offset) * 0.5, t)
  local speedSq = dx * dx + dz * dz
  local k = 0
  if speedSq > 1e-8 then
    k = (dx * ddz - dz * ddx) / (speedSq * math.sqrt(speedSq))
  end
  return { x = x, z = z, offset = offset, k = safe(k) }
end

return RacingLine
