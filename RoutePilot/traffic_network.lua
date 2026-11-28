-- RoutePilot/traffic_network.lua
-- Automatic CSP Traffic Tool traffic.json reader and directed road-network model.
-- This module deliberately does not alter driver.lua: it converts source data into
-- validated lane geometry + explicit directed lane transitions for RoutePilot.
local Network = {}

local function finite(v)
  return type(v) == "number" and v == v and v ~= math.huge and v ~= -math.huge
end

local function point3(raw)
  if type(raw) ~= "table" then return nil end
  local x, y, z = raw[1], raw[2], raw[3]
  if not finite(x) or not finite(y) or not finite(z) then return nil end
  return { x = x, y = y, z = z }
end

local function flatDistance(a, b)
  local dx, dz = b.x - a.x, b.z - a.z
  return math.sqrt(dx * dx + dz * dz)
end

local function cleanPoints(raw)
  if type(raw) ~= "table" then return nil, "POINTS_NOT_ARRAY" end
  local out, length = {}, 0
  for i = 1, #raw do
    local p = point3(raw[i])
    if not p then return nil, "INVALID_POINT_" .. tostring(i) end
    local prev = out[#out]
    if not prev or flatDistance(prev, p) > 0.015 then
      if prev then length = length + flatDistance(prev, p) end
      out[#out + 1] = p
    end
  end
  if #out < 2 or length < 0.5 then return nil, "LANE_TOO_SHORT" end
  return out, nil, length
end

local function roleRules(data)
  local result = {}
  local roles = data.rules and data.rules.laneRoles
  if type(roles) == "table" then
    for i = 1, #roles do
      local r = roles[i]
      if type(r) == "table" and type(r.name) == "string" then
        result[r.name] = {
          speedLimit = finite(r.speedLimit) and r.speedLimit or nil,
          priority = finite(r.priority) and r.priority or 0
        }
      end
    end
  end
  return result
end

-- Returns a graph keyed by the original numeric lane IDs. An edge exists only
-- when CSP explicitly supplies a trajectoryAttributes transition; proximity
-- between two polylines is never treated as proof of connectivity.
function Network.build(data)
  if type(data) ~= "table" or type(data.lanes) ~= "table" then
    return nil, "TRAFFIC_JSON_MISSING_LANES"
  end
  local graph = {
    lanes = {}, laneOrder = {}, outgoing = {}, incoming = {},
    intersections = {}, rules = roleRules(data),
    stats = { sourceLanes = #data.lanes, validLanes = 0,
      invalidLanes = 0, sourceIntersections = 0,
      transitions = 0, invalidTransitions = 0 }
  }

  for i = 1, #data.lanes do
    local source = data.lanes[i]
    if type(source) == "table" and finite(source.id) then
      local id = source.id
      if graph.lanes[id] then
        graph.stats.invalidLanes = graph.stats.invalidLanes + 1
      else
        local points, err, length = cleanPoints(source.points)
        if points then
          -- CSP loop lanes are cyclic even when the serialized point array
          -- omits the duplicate closing vertex. Close the runtime polyline.
          if source.loop == true then
            local first, last = points[1], points[#points]
            local gap = flatDistance(first, last)
            if gap > 0.015 then
              points[#points + 1] = { x=first.x, y=first.y, z=first.z }
              length = length + gap
            end
          end
          local role = graph.rules[source.role or source.laneRole or ""]
          graph.lanes[id] = {
            id = id, name = tostring(source.name or ("Lane #" .. id)),
            loop = source.loop == true, points = points, length = length,
            role = source.role or source.laneRole,
            speedLimit = role and role.speedLimit or nil,
            priority = role and role.priority or 0,
            source = source
          }
          graph.laneOrder[#graph.laneOrder + 1] = id
          graph.outgoing[id], graph.incoming[id] = {}, {}
          graph.stats.validLanes = graph.stats.validLanes + 1
        else
          graph.stats.invalidLanes = graph.stats.invalidLanes + 1
        end
      end
    else
      graph.stats.invalidLanes = graph.stats.invalidLanes + 1
    end
  end

  local intersections = type(data.intersections) == "table" and data.intersections or {}
  graph.stats.sourceIntersections = #intersections
  for i = 1, #intersections do
    local intersection = intersections[i]
    if type(intersection) == "table" then
      local record = { id = intersection.id, name = intersection.name,
        points = {}, transitions = 0, source = intersection,
        trajectoryAttributes = type(intersection.trajectoryAttributes) == "table" and intersection.trajectoryAttributes or {} }
      if type(intersection.points) == "table" then
        for j = 1, #intersection.points do
          local p = point3(intersection.points[j])
          if p then record.points[#record.points + 1] = p end
        end
      end
      local attrs = intersection.trajectoryAttributes
      if type(attrs) == "table" then
        for j = 1, #attrs do
          local item = attrs[j]
          local fromId = type(item) == "table" and item[1] or nil
          local toId = type(item) == "table" and item[2] or nil
          local params = type(item) == "table" and item[3] or nil
          local fromLane, toLane = graph.lanes[fromId], graph.lanes[toId]
          if fromLane and toLane and type(params) == "table" then
            local edge = {
              from = fromId, to = toId, intersectionId = intersection.id,
              cb = finite(params.cb) and math.max(0, math.min(1, params.cb)) or nil,
              ce = finite(params.ce) and math.max(0, math.min(1, params.ce)) or nil
            }
            graph.outgoing[fromId][#graph.outgoing[fromId] + 1] = edge
            graph.incoming[toId][#graph.incoming[toId] + 1] = edge
            record.transitions = record.transitions + 1
            graph.stats.transitions = graph.stats.transitions + 1
          else
            graph.stats.invalidTransitions = graph.stats.invalidTransitions + 1
          end
        end
      end
      graph.intersections[#graph.intersections + 1] = record
    end
  end

  table.sort(graph.laneOrder)
  for _, id in ipairs(graph.laneOrder) do
    table.sort(graph.outgoing[id], function(a, b)
      if a.to == b.to then return (a.intersectionId or 0) < (b.intersectionId or 0) end
      return a.to < b.to
    end)
  end
  return graph
end

function Network.loadTrackTraffic()
  if not ac or type(ac.getTrackDataFilename) ~= "function" then
    return nil, "TRACK_DATA_API_UNAVAILABLE"
  end
  if not io or type(io.load) ~= "function" then
    return nil, "IO_LOAD_UNAVAILABLE"
  end
  local okJson, json = pcall(require, "lib/json")
  if not okJson or type(json) ~= "table" or type(json.decode) ~= "function" then
    return nil, "JSON_LIBRARY_UNAVAILABLE"
  end
  local okPath, path = pcall(ac.getTrackDataFilename, "traffic.json")
  if not okPath or type(path) ~= "string" or path == "" then
    return nil, "TRAFFIC_JSON_PATH_UNAVAILABLE"
  end
  local okRaw, raw = pcall(io.load, path, "")
  if not okRaw or type(raw) ~= "string" or raw == "" then
    return nil, "TRAFFIC_JSON_NOT_FOUND"
  end
  local okDecode, decoded = pcall(json.decode, raw)
  if not okDecode or type(decoded) ~= "table" then
    return nil, "TRAFFIC_JSON_INVALID"
  end
  return Network.build(decoded)
end

function Network.nearestLane(graph, position, heading)
  if not graph or not position then return nil end
  local best, bestScore
  local hx, hz = 0, 0
  if heading then
    local hlen = math.sqrt((heading.x or 0)^2 + (heading.z or 0)^2)
    if hlen > 1e-6 then hx, hz = heading.x / hlen, heading.z / hlen end
  end
  for _, id in ipairs(graph.laneOrder) do
    local lane = graph.lanes[id]
    for i = 1, #lane.points - 1 do
      local a, b = lane.points[i], lane.points[i + 1]
      local vx, vz = b.x - a.x, b.z - a.z
      local len2 = vx * vx + vz * vz
      if len2 > 1e-8 then
        local t = math.max(0, math.min(1,
          ((position.x-a.x)*vx+(position.z-a.z)*vz)/len2))
        local px, pz = a.x+vx*t, a.z+vz*t
        local dx, dz = position.x-px, position.z-pz
        local distance2 = dx*dx + dz*dz
        local inv = 1 / math.sqrt(len2)
        local alignment = hx == 0 and hz == 0 and 1 or
          hx*vx*inv + hz*vz*inv
        local score = distance2 + (1-alignment)*(1-alignment)*9
        if not bestScore or score < bestScore then
          bestScore = score
          best = { laneId = id, segment = i, t = t,
            distance = math.sqrt(distance2), alignment = alignment,
            x = px, z = pz }
        end
      end
    end
  end
  return best
end

return Network
