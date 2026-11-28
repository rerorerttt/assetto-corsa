-- RoutePilot directed graph builder for CSP Traffic Tool lane geometry.
-- This module only builds topology; it does not move or control cars.
local Graph = {}

local function finite(v)
  return type(v) == 'number' and v == v and v ~= math.huge and v ~= -math.huge
end

local function pointInside(x, z, polygon)
  local inside = false
  local j = #polygon
  for i = 1, #polygon do
    local a, b = polygon[i], polygon[j]
    local az, bz = a[3], b[3]
    if ((az > z) ~= (bz > z)) then
      local crossX = (b[1] - a[1]) * (z - az) / (bz - az) + a[1]
      if x < crossX then inside = not inside end
    end
    j = i
  end
  return inside
end

local function segmentCrossing(ax, az, bx, bz, cx, cz, dx, dz)
  local rx, rz, sx, sz = bx - ax, bz - az, dx - cx, dz - cz
  local den = rx * sz - rz * sx
  if math.abs(den) < 1e-8 then return nil end
  local qx, qz = cx - ax, cz - az
  local t = (qx * sz - qz * sx) / den
  local u = (qx * rz - qz * rx) / den
  if t > 1e-6 and t < 1 - 1e-6 and u >= -1e-6 and u <= 1 + 1e-6 then
    return t, u
  end
  return nil
end

local function polygonInfo(def)
  if type(def) ~= 'table' or type(def.points) ~= 'table' or #def.points < 3 then return nil end
  local points, minY, maxY = {}, math.huge, -math.huge
  for i = 1, #def.points do
    local p = def.points[i]
    if type(p) ~= 'table' or not finite(tonumber(p[1])) or not finite(tonumber(p[2])) or not finite(tonumber(p[3])) then return nil end
    points[i] = { tonumber(p[1]), tonumber(p[2]), tonumber(p[3]) }
    minY, maxY = math.min(minY, points[i][2]), math.max(maxY, points[i][2])
  end
  local centerY = (minY + maxY) * 0.5
  return {
    id = def.id, name = def.name or ('Intersection ' .. tostring(def.id or '?')),
    points = points, centerY = centerY,
    yThreshold = 5 + maxY - minY,
    definition = def, links = {}, movements = {}
  }
end

local function laneLength(lane)
  local length = 0
  for i = 2, #lane.points do
    local a, b = lane.points[i - 1], lane.points[i]
    local dx, dy, dz = b.x - a.x, b.y - a.y, b.z - a.z
    length = length + math.sqrt(dx * dx + dy * dy + dz * dz)
  end
  if lane.loop and #lane.points > 2 then
    local a, b = lane.points[#lane.points], lane.points[1]
    local dx, dy, dz = b.x - a.x, b.y - a.y, b.z - a.z
    length = length + math.sqrt(dx * dx + dy * dy + dz * dz)
  end
  return length
end

local function laneIntersectionIntervals(lane, inter)
  local pts = lane.points
  if #pts < 2 then return {} end
  local result, active = {}, nil
  local total = 0
  local count = lane.loop and #pts or #pts - 1
  for i = 1, count do
    local a, b = pts[i], pts[i % #pts + 1]
    local dx, dz = b.x - a.x, b.z - a.z
    local segLen = math.sqrt(dx * dx + dz * dz + (b.y - a.y) * (b.y - a.y))
    if segLen > 1e-5 then
      local cuts = { 0, 1 }
      for k = 1, #inter.points do
        local p, q = inter.points[k], inter.points[k % #inter.points + 1]
        local t = segmentCrossing(a.x, a.z, b.x, b.z, p[1], p[3], q[1], q[3])
        if t then cuts[#cuts + 1] = t end
      end
      table.sort(cuts)
      local unique = {}
      for k = 1, #cuts do
        if #unique == 0 or math.abs(cuts[k] - unique[#unique]) > 1e-6 then unique[#unique + 1] = cuts[k] end
      end
      for k = 1, #unique - 1 do
        local t0, t1 = unique[k], unique[k + 1]
        local tm = (t0 + t1) * 0.5
        local y = a.y + (b.y - a.y) * tm
        local inside = math.abs(y - inter.centerY) <= inter.yThreshold and pointInside(a.x + dx * tm, a.z + dz * tm, inter.points)
        local d0, d1 = total + segLen * t0, total + segLen * t1
        if inside then
          if not active then active = { from = d0, to = d1 } else active.to = d1 end
        elseif active then
          result[#result + 1] = active
          active = nil
        end
      end
    end
    total = total + segLen
  end
  if active then result[#result + 1] = active end
  if lane.loop and #result > 1 then
    local first, last = result[1], result[#result]
    if first.from <= 0.1 and total - last.to <= 0.1 then
      first.from = last.from - total
      table.remove(result)
    end
  end
  return result
end

local function disallowed(def, fromID, toID)
  local list = def.disallowedTrajectories
  if type(list) ~= 'table' then return false end
  for i = 1, #list do
    local pair = list[i]
    if type(pair) == 'table' and tonumber(pair[1]) == fromID and tonumber(pair[2]) == toID then return true end
  end
  return false
end

function Graph.build(lanes, intersectionDefs, log)
  local graph = { lanes = lanes, intersections = {}, edges = {}, movements = {}, nodes = {}, stats = {
    validLanes = 0, invalidLanes = 0, intersectionLinks = 0, movements = 0,
    disconnectedLanes = 0, rejectedIntersections = 0
  } }
  local laneByID = {}
  for i = 1, #lanes do
    local lane = lanes[i]
    if lane.id ~= nil then laneByID[tostring(lane.id)] = lane end
    lane.graphLinks = {}
    if lane.valid then graph.stats.validLanes = graph.stats.validLanes + 1
    else graph.stats.invalidLanes = graph.stats.invalidLanes + 1 end
  end

  for i = 1, #intersectionDefs do
    local inter = polygonInfo(intersectionDefs[i])
    if inter then graph.intersections[#graph.intersections + 1] = inter
    else graph.stats.rejectedIntersections = graph.stats.rejectedIntersections + 1 end
  end

  for i = 1, #graph.intersections do
    local inter = graph.intersections[i]
    for j = 1, #lanes do
      local lane = lanes[j]
      if lane.valid then
        local intervals = laneIntersectionIntervals(lane, inter)
        for k = 1, #intervals do
          local span = intervals[k]
          if span.to - span.from >= 1.0 then
            local link = { intersection = inter, lane = lane, from = span.from, to = span.to }
            inter.links[#inter.links + 1] = link
            lane.graphLinks[#lane.graphLinks + 1] = link
            graph.stats.intersectionLinks = graph.stats.intersectionLinks + 1
          end
        end
      end
    end
  end

  -- A directed movement exists only from a lane entering the polygon to a
  -- different lane leaving it. User-authored forbidden movements are honored.
  for i = 1, #graph.intersections do
    local inter = graph.intersections[i]
    for a = 1, #inter.links do
      local incoming = inter.links[a]
      if incoming.from > 0.5 then
        for b = 1, #inter.links do
          local outgoing = inter.links[b]
          if incoming.lane ~= outgoing.lane and outgoing.to < outgoing.lane.totalLength - 0.5 then
            local fromID, toID = tonumber(incoming.lane.id), tonumber(outgoing.lane.id)
            if not disallowed(inter.definition, fromID, toID) then
              local movement = { intersection = inter, from = incoming, to = outgoing }
              inter.movements[#inter.movements + 1] = movement
              graph.movements[#graph.movements + 1] = movement
            end
          end
        end
      end
    end
  end
  graph.stats.movements = #graph.movements

  -- Build directed lane segments between consecutive intersection spans.
  -- These edges preserve lane direction; junction movements are separate.
  for i = 1, #lanes do
    local lane = lanes[i]
    if lane.valid then
      table.sort(lane.graphLinks, function(a, b) return a.from < b.from end)
      local cursor, length = 0, lane.totalLength
      local previous = nil
      for j = 1, #lane.graphLinks do
        local link = lane.graphLinks[j]
        if link.from >= cursor - 0.1 then
          graph.edges[#graph.edges + 1] = { lane = lane, from = cursor, to = link.from, enter = previous, exit = link }
          cursor = math.max(cursor, link.to)
          previous = link
        end
      end
      if cursor < length - 0.5 then
        graph.edges[#graph.edges + 1] = { lane = lane, from = cursor, to = length, enter = previous, exit = nil }
      end
      if #lane.graphLinks == 0 then graph.stats.disconnectedLanes = graph.stats.disconnectedLanes + 1 end
    end
  end

  if log then
    log('CSP_GRAPH_BUILT lanes=%d valid=%d invalid=%d intersections=%d links=%d movements=%d edges=%d disconnected=%d rejected_intersections=%d',
      #lanes, graph.stats.validLanes, graph.stats.invalidLanes, #graph.intersections,
      graph.stats.intersectionLinks, graph.stats.movements, #graph.edges,
      graph.stats.disconnectedLanes, graph.stats.rejectedIntersections)
  end
  return graph
end

return Graph
