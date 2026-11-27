-- RoutePilot traffic network foundation.
-- Preserves every authored CSP lane as an independent directed lane.
-- No raycasts, no synthetic lane stitching, and no inferred loop closure.

local Network = {}

local DEFAULT_LANE_WIDTH = 3.0
local DEFAULT_CELL_SIZE = 24.0
local EPSILON = 1e-6

local function finite(v)
  return type(v) == "number" and v == v and v ~= math.huge and v ~= -math.huge
end

local function clamp(v, lo, hi)
  return math.max(lo, math.min(hi, v))
end

local function flatDistance(a, b)
  local dx, dz = b.x - a.x, b.z - a.z
  return math.sqrt(dx * dx + dz * dz)
end

local function normalize(x, z)
  local length = math.sqrt(x * x + z * z)
  if length < EPSILON then return 0, 1 end
  return x / length, z / length
end

local function cellKey(x, z)
  return tostring(x) .. ":" .. tostring(z)
end

local function median(values)
  if #values == 0 then return nil end
  table.sort(values)
  local middle = math.floor((#values + 1) * 0.5)
  if #values % 2 == 1 then return values[middle] end
  return (values[middle] + values[middle + 1]) * 0.5
end

local function insertSegment(index, cellSize, segment)
  local minX = math.floor(segment.minX / cellSize)
  local maxX = math.floor(segment.maxX / cellSize)
  local minZ = math.floor(segment.minZ / cellSize)
  local maxZ = math.floor(segment.maxZ / cellSize)
  for x = minX, maxX do
    for z = minZ, maxZ do
      local key = cellKey(x, z)
      local bucket = index[key]
      if not bucket then bucket = {}; index[key] = bucket end
      bucket[#bucket + 1] = segment
    end
  end
end

local function collectSegments(index, cellSize, x, z, radius)
  local result, seen = {}, {}
  local minX = math.floor((x - radius) / cellSize)
  local maxX = math.floor((x + radius) / cellSize)
  local minZ = math.floor((z - radius) / cellSize)
  local maxZ = math.floor((z + radius) / cellSize)
  for cx = minX, maxX do
    for cz = minZ, maxZ do
      local bucket = index[cellKey(cx, cz)]
      if bucket then
        for i = 1, #bucket do
          local segment = bucket[i]
          if not seen[segment] then
            seen[segment] = true
            result[#result + 1] = segment
          end
        end
      end
    end
  end
  return result
end

local function projectSegment(segment, position)
  local a, b = segment.a, segment.b
  local vx, vz = b.x - a.x, b.z - a.z
  local t = clamp(((position.x - a.x) * vx + (position.z - a.z) * vz) /
    math.max(EPSILON, segment.lengthSq), 0, 1)
  local px, pz = a.x + vx * t, a.z + vz * t
  local dx, dz = position.x - px, position.z - pz
  local distanceSq = dx * dx + dz * dz
  local lateral = (position.x - px) * -segment.tz + (position.z - pz) * segment.tx
  return {
    distanceSq = distanceSq,
    distance = math.sqrt(distanceSq),
    lateral = lateral,
    t = t,
    progress = segment.s0 + segment.length * t,
    tx = segment.tx,
    tz = segment.tz,
    segment = segment.index,
    lane = segment.lane
  }
end

local function buildLane(source, width)
  if type(source) ~= "table" or type(source.points) ~= "table" or #source.points < 2 then
    return nil
  end
  local lane = {
    id = source.id,
    name = source.name or ("Lane " .. tostring(source.id)),
    loop = source.loop == true,
    role = source.role,
    priority = source.priority or 0,
    speedLimit = source.speedLimit or 90,
    params = source.params or {},
    source = source,
    points = source.points,
    width = width,
    halfWidth = width * 0.5,
    segments = {},
    cumulative = { 0 },
    neighbours = { left = {}, right = {}, opposing = {} },
    minX = math.huge, maxX = -math.huge,
    minZ = math.huge, maxZ = -math.huge
  }
  local count = lane.loop and #lane.points or (#lane.points - 1)
  local total = 0
  for i = 1, #lane.points do
    local p = lane.points[i]
    if not p or not finite(p.x) or not finite(p.y) or not finite(p.z) then return nil end
    lane.minX, lane.maxX = math.min(lane.minX, p.x), math.max(lane.maxX, p.x)
    lane.minZ, lane.maxZ = math.min(lane.minZ, p.z), math.max(lane.maxZ, p.z)
  end
  for i = 1, count do
    local a = lane.points[i]
    local b = lane.points[(i % #lane.points) + 1]
    local dx, dz = b.x - a.x, b.z - a.z
    local lengthSq = dx * dx + dz * dz
    local length = math.sqrt(lengthSq)
    if length > 0.05 then
      local tx, tz = normalize(dx, dz)
      local segment = {
        lane = lane, index = i, a = a, b = b,
        length = length, lengthSq = lengthSq,
        tx = tx, tz = tz, s0 = total,
        minX = math.min(a.x, b.x), maxX = math.max(a.x, b.x),
        minZ = math.min(a.z, b.z), maxZ = math.max(a.z, b.z)
      }
      lane.segments[#lane.segments + 1] = segment
      total = total + length
      lane.cumulative[#lane.segments + 1] = total
    end
  end
  if #lane.segments == 0 then return nil end
  lane.length = total
  return lane
end

local function nearestOnLane(lane, position, headingX, headingZ)
  local best
  for i = 1, #lane.segments do
    local candidate = projectSegment(lane.segments[i], position)
    local score = candidate.distanceSq
    if headingX and headingZ then
      local alignment = headingX * candidate.tx + headingZ * candidate.tz
      score = score + (1 - alignment) * 12
    end
    if not best or score < best.score then
      candidate.score = score
      best = candidate
    end
  end
  return best
end

local function buildNeighbours(lanes, cellIndex, cellSize, log)
  local evidence = {}
  for i = 1, #lanes do
    local lane = lanes[i]
    local stride = math.max(1, math.floor(#lane.segments / 30))
    for j = 1, #lane.segments, stride do
      local segment = lane.segments[j]
      local sample = {
        x = (segment.a.x + segment.b.x) * 0.5,
        y = (segment.a.y + segment.b.y) * 0.5,
        z = (segment.a.z + segment.b.z) * 0.5
      }
      local nearby = collectSegments(cellIndex, cellSize, sample.x, sample.z, 5.0)
      local tested = {}
      for k = 1, #nearby do
        local otherSegment = nearby[k]
        local other = otherSegment.lane
        if other ~= lane and not tested[other] then
          tested[other] = true
          local dot = segment.tx * otherSegment.tx + segment.tz * otherSegment.tz
          if math.abs(dot) >= 0.86 then
            local projected = projectSegment(otherSegment, sample)
            if projected.distance >= 1.8 and projected.distance <= 4.3 then
              local signed = (sample.x - (otherSegment.a.x +
                (otherSegment.b.x - otherSegment.a.x) * projected.t)) * -segment.tz +
                (sample.z - (otherSegment.a.z +
                (otherSegment.b.z - otherSegment.a.z) * projected.t)) * segment.tx
              local key = tostring(lane.id) .. ">" .. tostring(other.id)
              local item = evidence[key]
              if not item then
                item = { from = lane, to = other, distances = {}, alignments = {} }
                evidence[key] = item
              end
              item.distances[#item.distances + 1] = projected.distance
              item.alignments[#item.alignments + 1] = dot
              item.side = signed >= 0 and "left" or "right"
            end
          end
        end
      end
    end
  end
  local count = 0
  for _, item in pairs(evidence) do
    if #item.distances >= 2 then
      local distance = median(item.distances)
      local alignment = median(item.alignments)
      if distance and distance >= 2.2 and distance <= 3.8 and alignment then
        local record = { lane = item.to, centerDistance = distance, alignment = alignment }
        if alignment <= -0.86 then
          item.from.neighbours.opposing[#item.from.neighbours.opposing + 1] = record
        else
          item.from.neighbours[item.side][#item.from.neighbours[item.side] + 1] = record
        end
        count = count + 1
      end
    end
  end
  if log then pcall(log, "NETWORK_NEIGHBOURS links=%d", count) end
end

function Network.build(source, options, log)
  options = options or {}
  if type(source) ~= "table" or type(source.lanes) ~= "table" then
    return nil, "network has no lanes"
  end
  local width = finite(options.laneWidth) and clamp(options.laneWidth, 2.0, 4.5) or DEFAULT_LANE_WIDTH
  local cellSize = finite(options.cellSize) and clamp(options.cellSize, 8.0, 64.0) or DEFAULT_CELL_SIZE
  local network = {
    lanes = {}, lanesById = {}, intersections = source.intersections or {},
    areas = source.areas or {}, rules = source.rules or {},
    laneWidth = width, cellSize = cellSize, segmentIndex = {},
    source = source.source
  }
  local pointCount, segmentCount = 0, 0
  for i = 1, #source.lanes do
    local lane = buildLane(source.lanes[i], width)
    if lane then
      if network.lanesById[lane.id] then
        return nil, "duplicate lane id: " .. tostring(lane.id)
      end
      network.lanes[#network.lanes + 1] = lane
      network.lanesById[lane.id] = lane
      pointCount = pointCount + #lane.points
      segmentCount = segmentCount + #lane.segments
      for j = 1, #lane.segments do
        insertSegment(network.segmentIndex, cellSize, lane.segments[j])
      end
    end
  end
  if #network.lanes == 0 then return nil, "network has no valid lanes" end
  buildNeighbours(network.lanes, network.segmentIndex, cellSize, log)
  network.pointCount = pointCount
  network.segmentCount = segmentCount
  network.laneCount = #network.lanes
  network.intersectionCount = #network.intersections
  network.areaCount = #network.areas
  if log then
    pcall(log, "NETWORK_READY lanes=%d points=%d segments=%d intersections=%d areas=%d lane_width=%.2f",
      network.laneCount, pointCount, segmentCount, network.intersectionCount, network.areaCount, width)
  end
  return network
end

function Network.projectLane(lane, position, heading, maxDistance)
  if not lane or not position then return nil end
  local hx, hz
  if heading then hx, hz = normalize(heading.x, heading.z) end
  local best = nearestOnLane(lane, position, hx, hz)
  if not best then return nil end
  if maxDistance and best.distance > maxDistance then return nil end
  return best
end

function Network.findNearestLane(network, position, heading, maxDistance)
  if not network or not position then return nil end
  local radius = maxDistance or 15.0
  local candidates = collectSegments(network.segmentIndex, network.cellSize,
    position.x, position.z, radius)
  local hx, hz
  if heading then hx, hz = normalize(heading.x, heading.z) end
  local best, bestScore
  local seen = {}
  for i = 1, #candidates do
    local segment = candidates[i]
    local lane = segment.lane
    if not seen[lane] then
      seen[lane] = true
      local candidate = nearestOnLane(lane, position, hx, hz)
      if candidate and candidate.distance <= radius then
        local score = candidate.distance * candidate.distance
        if hx and hz then
          local alignment = hx * candidate.tx + hz * candidate.tz
          score = score + (1 - alignment) * 18
        end
        if not bestScore or score < bestScore then
          best, bestScore = candidate, score
        end
      end
    end
  end
  return best
end

function Network.getLaneCorridor(lane, safetyMargin)
  if not lane then return nil end
  local margin = finite(safetyMargin) and math.max(0, safetyMargin) or 0.45
  local usable = math.max(0, lane.halfWidth - margin)
  return -usable, usable
end

function Network.isInsideLane(lane, projection, safetyMargin)
  if not lane or not projection then return false end
  local left, right = Network.getLaneCorridor(lane, safetyMargin)
  return projection.lateral >= left and projection.lateral <= right
end

return Network
