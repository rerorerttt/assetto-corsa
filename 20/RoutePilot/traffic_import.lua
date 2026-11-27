local json = require('json')

local M = {}
local MAX_POINTS = 50000
local CONNECT_DISTANCE = 8.0

local function finite(v)
  return type(v) == 'number' and v == v and v ~= math.huge and v ~= -math.huge
end

local function pointFromTable(p)
  if type(p) ~= 'table' then return nil end
  local x = p.x or p[1]
  local y = p.y or p[2]
  local z = p.z or p[3]
  if not finite(x) or not finite(y) or not finite(z) then return nil end
  return vec3(x, y, z)
end

local function flatDistance(a, b)
  local dx, dz = b.x - a.x, b.z - a.z
  return math.sqrt(dx * dx + dz * dz)
end

local function normalized(x, z)
  local length = math.sqrt(x * x + z * z)
  if length < 0.00001 then return 0, 1 end
  return x / length, z / length
end

local function decodeLanes(data)
  if type(data) ~= 'table' or type(data.lanes) ~= 'table' then
    return nil, 'traffic.json has no lanes array'
  end
  local lanes = {}
  for i = 1, #data.lanes do
    local source = data.lanes[i]
    if type(source) == 'table' and type(source.points) == 'table' then
      local points = {}
      for j = 1, #source.points do
        local p = pointFromTable(source.points[j])
        if p then points[#points + 1] = p end
        if #points >= MAX_POINTS then break end
      end
      if #points >= (source.loop == true and 3 or 2) then
        lanes[#lanes + 1] = {
          id = source.id or i,
          name = source.name or ('Lane ' .. tostring(source.id or i)),
          points = points,
          loop = source.loop == true
        }
      end
    end
  end
  if #lanes == 0 then return nil, 'traffic.json contains no usable directed lanes' end
  return lanes
end

local function projectToLane(lane, position, hx, hz)
  local points = lane.points
  local segmentCount = lane.loop and #points or (#points - 1)
  local best = nil
  for i = 1, segmentCount do
    local a = points[i]
    local b = points[i % #points + 1]
    local vx, vz = b.x - a.x, b.z - a.z
    local lenSq = vx * vx + vz * vz
    if lenSq > 0.0001 then
      local t = math.max(0, math.min(1,
        ((position.x - a.x) * vx + (position.z - a.z) * vz) / lenSq))
      local px, pz = a.x + vx * t, a.z + vz * t
      local dx, dz = position.x - px, position.z - pz
      local distanceSq = dx * dx + dz * dz
      local tx, tz = normalized(vx, vz)
      local alignment = hx * tx + hz * tz
      -- Direction is never reversed: a lane facing away from the player
      -- cannot win over a lane whose authored direction matches the player.
      local score = distanceSq + (1 - alignment) * 100
      if alignment >= -0.05 and (not best or score < best.score) then
        best = { score = score, distance = math.sqrt(distanceSq),
          segment = i, t = t, tx = tx, tz = tz }
      end
    end
  end
  return best
end

local function appendPoint(out, p)
  local last = out[#out]
  if not last or flatDistance(last, p) > 0.05 then
    out[#out + 1] = vec3(p.x, p.y, p.z)
  end
end

local function buildDirectedChain(lanes, player)
  local hx, hz = normalized(player.look.x, player.look.z)
  local startIndex, startProjection, startScore = nil, nil, math.huge
  for i = 1, #lanes do
    local projection = projectToLane(lanes[i], player.position, hx, hz)
    if projection and projection.score < startScore then
      startIndex, startProjection, startScore = i, projection, projection.score
    end
  end
  if not startIndex then return nil, nil, 'no lane matches player heading' end

  local nodes = {}
  local visited = {}
  local currentIndex = startIndex
  local firstPoint = nil
  local loopRoute = false
  local chainCount = 0

  while currentIndex and not visited[currentIndex] and #nodes < MAX_POINTS do
    local lane = lanes[currentIndex]
    visited[currentIndex] = true
    chainCount = chainCount + 1
    local points = lane.points
    local firstSegment = currentIndex == startIndex and startProjection.segment or 1
    local firstT = currentIndex == startIndex and startProjection.t or 0
    if currentIndex ~= startIndex then appendPoint(nodes, points[1]) end
    local a = points[firstSegment]
    local b = points[firstSegment % #points + 1]
    if currentIndex == startIndex then
      local start = vec3(a.x + (b.x-a.x)*firstT,
        a.y + (b.y-a.y)*firstT, a.z + (b.z-a.z)*firstT)
      appendPoint(nodes, start)
      firstPoint = start
    end

    local segmentCount = lane.loop and #points or (#points - 1)
    local segment = firstSegment
    local traversed = 0
    while traversed < segmentCount and #nodes < MAX_POINTS do
      local nextIndex = segment % #points + 1
      appendPoint(nodes, points[nextIndex])
      segment = nextIndex
      traversed = traversed + 1
      if lane.loop and segment == firstSegment then
        loopRoute = true
        currentIndex = nil
        break
      end
    end
    if currentIndex == nil then break end
    if lane.loop then
      loopRoute = true
      break
    end

    local endpoint = nodes[#nodes]
    local prev = nodes[#nodes - 1]
    if not endpoint or not prev then break end
    local dx, dz = normalized(endpoint.x - prev.x, endpoint.z - prev.z)
    local bestIndex, bestScore = nil, math.huge
    for candidateIndex = 1, #lanes do
      if not visited[candidateIndex] then
        local candidate = lanes[candidateIndex]
        local start = candidate.points[1]
        local distance = flatDistance(endpoint, start)
        if distance <= CONNECT_DISTANCE then
          local nextPoint = candidate.points[2]
          local cx, cz = normalized(nextPoint.x - start.x, nextPoint.z - start.z)
          local alignment = dx * cx + dz * cz
          if alignment > 0.35 then
            local score = distance * 4 + (1 - alignment) * 10
            if score < bestScore then
              bestScore, bestIndex = score, candidateIndex
            end
          end
        end
      end
    end
    if not bestIndex then
      -- A directed lane can close only by reaching the original start point
      -- in the same authored direction. No lane is ever reversed to force a loop.
      if firstPoint and flatDistance(endpoint, firstPoint) <= CONNECT_DISTANCE then
        loopRoute = true
      end
      break
    end
    currentIndex = bestIndex
  end

  if #nodes < 2 then return nil, nil, 'directed lane chain is too short' end
  return nodes, loopRoute, string.format(
    'lane=%s chain=%d points=%d loop=%s start_error=%.2f',
    tostring(lanes[startIndex].id), chainCount, #nodes,
    tostring(loopRoute), startProjection.distance)
end

function M.load()
  if not ac or type(ac.getTrackDataFilename) ~= 'function' then
    return nil, nil, 'ac.getTrackDataFilename unavailable'
  end
  if not io or type(io.load) ~= 'function' then
    return nil, nil, 'CSP io.load unavailable'
  end
  local okPath, path = pcall(ac.getTrackDataFilename, 'traffic.json')
  if not okPath or type(path) ~= 'string' or path == '' then
    return nil, nil, 'traffic.json path unavailable'
  end
  local okRead, raw = pcall(io.load, path)
  if not okRead or type(raw) ~= 'string' or raw == '' then
    return nil, nil, 'traffic.json not found'
  end
  local okDecode, data = pcall(json.decode, raw)
  if not okDecode then return nil, nil, 'traffic.json parse error: ' .. tostring(data) end
  local lanes, laneError = decodeLanes(data)
  if not lanes then return nil, nil, laneError end
  local player = ac.getCar(0)
  if not player or not player.position or not player.look then
    return nil, nil, 'player car unavailable during import'
  end
  return buildDirectedChain(lanes, player)
end

return M
