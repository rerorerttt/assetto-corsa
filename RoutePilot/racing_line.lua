-- RoutePilot/racing_line.lua
-- Geometry-first racing line optimizer. It minimizes the curvature of the
-- complete swept vehicle path over several bends, using the actual vehicle
-- footprint and recorded road boundaries. Traffic is a soft path cost: a
-- car ahead must not make an otherwise drivable road return NO_PATH.
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

local function normalize(x, z)
  local length = math.sqrt(x * x + z * z)
  if length < 0.00001 then return 0, 1 end
  return x / length, z / length
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

local function footprintClearance(sampleA, sampleB, offsetA, offsetB,
    boundsA, boundsB, halfWidth, halfLength, edgeMargin)
  local dx, dz = sampleB.x - sampleA.x, sampleB.z - sampleA.z
  local baseLength = math.max(0.05, math.sqrt(dx * dx + dz * dz))
  local offsetSlope = (offsetB - offsetA) / baseLength
  local function sweptAt(t)
    local tx,tz=normalize(
      sampleA.tx+(sampleB.tx-sampleA.tx)*t,
      sampleA.tz+(sampleB.tz-sampleA.tz)*t)
    local k=safe(sampleA.k+(sampleB.k-sampleA.k)*t)
    local offset=offsetA+(offsetB-offsetA)*t
    -- The car follows the tangent of the optimized offset curve, not the
    -- straight chord between two planning stations. In Frenet coordinates:
    -- dP/ds = (1 - k*offset)*T + offsetSlope*N.
    local pathX,pathZ=normalize(
      tx*(1.0-k*offset)-tz*offsetSlope,
      tz*(1.0-k*offset)+tx*offsetSlope)
    local dot=math.abs(pathX*tx+pathZ*tz)
    local cross=math.abs(pathX*tz-pathZ*tx)
    local envelope=halfWidth*dot+halfLength*cross+edgeMargin
    local left=boundsA[1]+(boundsB[1]-boundsA[1])*t
    local right=boundsA[2]+(boundsB[2]-boundsA[2])*t
    return math.min(offset-(left+envelope),(right-envelope)-offset)
  end
  return math.min(sweptAt(0),sweptAt(0.25),sweptAt(0.5),
    sweptAt(0.75),sweptAt(1))
end

local function trafficCost(offset, forbidden)
  local cost = 0
  for _, interval in ipairs(forbidden or {}) do
    local lo, hi = safe(interval[1]), safe(interval[2])
    if offset >= lo and offset <= hi then
      local penetration = math.min(offset - lo, hi - offset)
      cost = cost + 260.0 + penetration * 90.0
    else
      local clearance = offset < lo and lo - offset or offset - hi
      if clearance < 0.75 then
        local near = 0.75 - clearance
        cost = cost + near * near * 55.0
      end
    end
  end
  return cost
end

local function edgeCost(clearance)
  return 0.002 / math.max(0.06, clearance + 0.06)
end

-- boundsAt(sample) returns left/right road-edge offsets in the spline frame.
-- fixedOffset is used only for a previously validated passing corridor.
function RacingLine.plan(route, sampleFn, progress, car, boundsAt, startOffset, baseOffset,
    fixedOffset, horizon, spacing, obstacleAt, previousPlan, previousAdvance)
  if not route or not progress or not boundsAt then return nil end

  local size = car and car.aabbSize
  local halfWidth = math.max(0.75, safe(size and size.x, 2.0) * 0.5)
  local halfLength = math.max(1.5, safe(size and size.z, 4.5) * 0.5)
  local edgeMargin = 0.38
  local baseMargin = halfWidth + edgeMargin
  local step = math.max(3.5, spacing or 4.5)
  -- A path may use the full road, but its lateral position cannot jump
  -- across the whole width in one sample. The speed envelope evaluates the
  -- resulting curvature and lifts the throttle early when a slower path is
  -- required; the geometry solver itself must not fail just because the car
  -- arrived too fast for a sharp transition.
  local maxOffsetStep = step * 0.42
  local length = math.min(180.0, math.max(45.0, horizon or 150.0))
  local count = math.floor(length / step) + 1
  local samples, bounds, offsets = {}, {}, {}
  local blockedDistance = nil
  local initial = safe(startOffset, safe(baseOffset, 0))
  local preferred = safe(baseOffset, initial)

  -- First build the physical corridor. A distant narrowing terminates the
  -- usable plan; it is not converted into a fake center-line path.
  for i = 0, count do
    local d = i * step
    local sample = sampleFn and sampleFn(route, progress + d) or nil
    if not sample then break end
    local left, right = boundsAt(sample)
    left, right = safe(left, -2.0), safe(right, 2.0)
    local lo, hi = left + baseMargin, right - baseMargin
    if lo > hi then
      if d <= 8.0 then return nil, "NO_FIT" end
      blockedDistance = d
      break
    end
    samples[#samples + 1] = sample
    bounds[#bounds + 1] = {left, right, lo, hi}
  end

  local n = #samples
  if n < 3 then return nil, "NO_FIT" end

  if fixedOffset ~= nil then
    for i = 1, n do
      local d = (i - 1) * step
      local t = clamp(d / 22.0, 0, 1)
      local desired = initial + (safe(fixedOffset, initial) - initial) * t
      offsets[i] = clamp(desired, bounds[i][3], bounds[i][4])
      if i > 1 then
        local clearance = footprintClearance(samples[i - 1], samples[i],
          offsets[i - 1], offsets[i], bounds[i - 1], bounds[i],
          halfWidth, halfLength, edgeMargin)
        if clearance < -0.015 then
          blockedDistance = d
          n = i - 1
          break
        end
      end
    end
    if n < 3 then return nil, "NO_FIT" end
  else
    -- Candidate offsets span the complete physically usable road. Traffic
    -- intervals add a strong cost but never erase every candidate and freeze
    -- a car merely because a leader occupies the same lane.
    local candidateSets, priorOffsets, forbiddenAt = {}, {}, {}
    local priorDistance = math.max(0, safe(previousAdvance, 0))
    candidateSets[1] = {clamp(initial, bounds[1][3], bounds[1][4])}
    priorOffsets[1] = initial

    local function addCandidate(list, value, lo, hi)
      local v = clamp(value, lo, hi)
      for j = 1, #list do
        if math.abs(list[j] - v) < 0.035 then return end
      end
      list[#list + 1] = v
    end

    for i = 2, n do
      local lo, hi = bounds[i][3], bounds[i][4]
      local list = {}
      local d = (i - 1) * step
      forbiddenAt[i] = obstacleAt and obstacleAt(samples[i], d) or nil
      local prior = previousPlan and RacingLine.sample(previousPlan,
        priorDistance + d) or nil
      priorOffsets[i] = prior and prior.offset or nil

      for c = 0, 6 do
        addCandidate(list, lo + (hi - lo) * c / 10.0, lo, hi)
      end
      addCandidate(list, preferred, lo, hi)
      if prior then addCandidate(list, prior.offset, lo, hi) end

      local previous = sampleFn(route, progress + math.max(0, d - step))
      local following = sampleFn(route, progress + d + step)
      local kp = safe(previous and previous.k)
      local kc = safe(samples[i].k)
      local kn = safe(following and following.k, kc)
      local smoothCurvature = (kp + 2.0 * kc + kn) * 0.25
      local curvatureCandidate = clamp(
        preferred + clamp(-smoothCurvature * 110.0, -3.5, 3.5), lo, hi)
      addCandidate(list, curvatureCandidate, lo, hi)
      if #list == 0 then return nil, "NO_FIT" end
      candidateSets[i] = list
    end

    local firstLayer = {}
    for j = 1, #candidateSets[2] do
      local off0 = candidateSets[1][1]
      local off1 = candidateSets[2][j]
      local clearance = footprintClearance(samples[1], samples[2],
        off0, off1, bounds[1], bounds[2],
        halfWidth, halfLength, edgeMargin)
      if clearance >= -0.015 and math.abs(off1 - off0) <= maxOffsetStep then
        local continuity = priorOffsets[2] and
          (off1 - priorOffsets[2]) ^ 2 * 0.12 or 0
        local cost = (off1 - off0) ^ 2 * 0.16 + continuity +
          edgeCost(clearance) + trafficCost(off1, forbiddenAt[2])
        firstLayer[#firstLayer + 1] = {
          a = 1, b = j, cost = cost, prev = nil
        }
      end
    end
    if #firstLayer == 0 then return nil, "NO_FIT" end

    local layer = firstLayer
    local reachable = 2
    for station = 2, n - 1 do
      local nextLayer = {}
      local bestByPair = {}
      for _, node in ipairs(layer) do
        local offA = candidateSets[station - 1][node.a]
        local offB = candidateSets[station][node.b]
        local pointA = pointAt(samples[station - 1], offA)
        local pointB = pointAt(samples[station], offB)
        for c = 1, #candidateSets[station + 1] do
          local offC = candidateSets[station + 1][c]
          local clearance = footprintClearance(samples[station], samples[station + 1],
            offB, offC, bounds[station], bounds[station + 1],
            halfWidth, halfLength, edgeMargin)
          local second = offA - 2.0 * offB + offC
          if clearance >= -0.015 and
             math.abs(offC - offB) <= maxOffsetStep and
             math.abs(second) <= maxOffsetStep * 0.90 then
            local pointC = pointAt(samples[station + 1], offC)
            local k = curvature(pointA, pointB, pointC)
            local slope = offC - offB
            local continuity = priorOffsets[station + 1] and
              (offC - priorOffsets[station + 1]) ^ 2 * 0.12 or 0
            -- Integrated curvature plus a peak-curvature term approximates
            -- lap-time minimization: it prefers long straight chords through
            -- small boundary wiggles and opens consecutive bends as one line.
            local k2 = k * k
            -- With lateral acceleration limited to 12 m/s², ds*sqrt(|k|/a)
            -- is proportional to the time spent on this segment. This makes
            -- the optimizer prefer the fastest multi-bend chord, not merely
            -- the path with the smallest pointwise offset error.
            local segmentLength=math.sqrt(
              (pointC.x-pointB.x)^2+(pointC.z-pointB.z)^2)
            local timeCost=segmentLength*math.sqrt(math.abs(k)/12.0)*30.0
            local cost = node.cost + timeCost +
              k2 * 300.0 + k2 * k2 * 110000.0 +
              second * second * 0.65 + slope * slope * 0.12 +
              continuity + edgeCost(clearance) +
              trafficCost(offC, forbiddenAt[station + 1])
            local key = node.b * 100 + c
            local existing = bestByPair[key]
            if not existing or cost < existing.cost then
              bestByPair[key] = {
                a = node.b, b = c, cost = cost, prev = node
              }
            end
          end
        end
      end
      for _, node in pairs(bestByPair) do
        nextLayer[#nextLayer + 1] = node
      end
      if #nextLayer == 0 then
        -- Keep the last physically valid part of the path and let the speed
        -- envelope slow before the first point that cannot contain the body.
        blockedDistance = station * step
        n = reachable
        break
      end
      layer = nextLayer
      reachable = station + 1
    end

    if n < 3 then return nil, "NO_FIT" end
    local best = nil
    for _, node in ipairs(layer) do
      if not best or node.cost < best.cost then best = node end
    end
    if not best then return nil, "NO_FIT" end
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
  local usableDistance = plan.blockedDistance and
    math.min(distance, math.max(0, plan.blockedDistance - 10.0)) or distance
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
