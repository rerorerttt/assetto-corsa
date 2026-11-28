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
  local minX, maxX, minZ, maxZ = math.huge, -math.huge, math.huge, -math.huge
  for i = 1, #def.points do
    local p = def.points[i]
    if type(p) ~= 'table' or not finite(tonumber(p[1])) or not finite(tonumber(p[2])) or not finite(tonumber(p[3])) then return nil end
    points[i] = { tonumber(p[1]), tonumber(p[2]), tonumber(p[3]) }
    minY, maxY = math.min(minY, points[i][2]), math.max(maxY, points[i][2])
    minX, maxX = math.min(minX, points[i][1]), math.max(maxX, points[i][1])
    minZ, maxZ = math.min(minZ, points[i][3]), math.max(maxZ, points[i][3])
  end
  local centerY = (minY + maxY) * 0.5
  return {
    id = def.id, name = def.name or ('Intersection ' .. tostring(def.id or '?')),
    points = points, centerY = centerY,
    minX = minX, maxX = maxX, minZ = minZ, maxZ = maxZ,
    yThreshold = 5 + maxY - minY,
    definition = def, links = {}, movements = {}
  }
end

local function laneLength(lane)
  local length, cumulative = 0, { [1] = 0 }
  for i = 2, #lane.points do
    local a, b = lane.points[i - 1], lane.points[i]
    local dx, dy, dz = b.x - a.x, b.y - a.y, b.z - a.z
    length = length + math.sqrt(dx * dx + dy * dy + dz * dz)
    cumulative[i] = length
  end
  if lane.loop and #lane.points > 2 then
    local a, b = lane.points[#lane.points], lane.points[1]
    local dx, dy, dz = b.x - a.x, b.y - a.y, b.z - a.z
    length = length + math.sqrt(dx * dx + dy * dy + dz * dz)
    cumulative[#lane.points + 1] = length
  end
  lane.cumulative = cumulative
  return length
end

local function lanePositionAt(lane, distance)
  local points, cumulative = lane.points, lane.cumulative
  local n = #points
  distance = math.max(0, math.min(lane.totalLength, distance))
  for i = 1, n do
    local d0, d1 = cumulative[i] or 0, cumulative[i + 1] or lane.totalLength
    if distance <= d1 or i == n then
      local a = points[i]
      local b = points[i % n + 1]
      local span = d1 - d0
      local t = span > 1e-6 and math.max(0, math.min(1, (distance - d0) / span)) or 0
      return vec3(a.x + (b.x-a.x)*t, a.y + (b.y-a.y)*t, a.z + (b.z-a.z)*t)
    end
  end
  return points[n]
end

local function laneDirectionAt(lane, distance)
  local points, cumulative = lane.points, lane.cumulative
  local n = #points
  for i = 1, n do
    local d0, d1 = cumulative[i] or 0, cumulative[i + 1] or lane.totalLength
    if distance <= d1 or i == n then
      local a, b = points[i], points[i % n + 1]
      local dx, dy, dz = b.x-a.x, b.y-a.y, b.z-a.z
      local length = math.sqrt(dx*dx+dy*dy+dz*dz)
      if length > 1e-6 then return vec3(dx/length, dy/length, dz/length) end
    end
  end
  return vec3(0, 0, 1)
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
  -- Keep seam-crossing spans split at 0/total. The graph builder links
  -- the two ends explicitly; negative distances would corrupt lane ordering.
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
  for i = 1, #lanes do
    local lane = lanes[i]
    lane.graphLinks = {}
    lane.totalLength = lane.valid and laneLength(lane) or 0
    lane.graphBounds = nil
    if lane.valid and #lane.points > 0 then
      local minX, maxX, minZ, maxZ = math.huge, -math.huge, math.huge, -math.huge
      for j = 1, #lane.points do
        local p = lane.points[j]
        minX, maxX = math.min(minX, p.x), math.max(maxX, p.x)
        minZ, maxZ = math.min(minZ, p.z), math.max(maxZ, p.z)
      end
      lane.graphBounds = { minX = minX, maxX = maxX, minZ = minZ, maxZ = maxZ }
    end
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
      local bounds = lane.graphBounds
      local overlaps = bounds and bounds.maxX >= inter.minX and bounds.minX <= inter.maxX
        and bounds.maxZ >= inter.minZ and bounds.minZ <= inter.maxZ
      if lane.valid and overlaps then
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

  local nextNodeID, nextEdgeID = 0, 0
  local function addNode(kind, position, owner)
    nextNodeID = nextNodeID + 1
    local node = { id = nextNodeID, kind = kind, position = position, owner = owner, outgoing = {} }
    graph.nodes[#graph.nodes + 1] = node
    return node
  end
  local function addEdge(lane, fromDistance, toDistance, fromNode, toNode, enterLink, exitLink, wrap)
    if not fromNode or not toNode or toDistance - fromDistance <= 0.1 then return nil end
    nextEdgeID = nextEdgeID + 1
    local edge = {
      id = nextEdgeID, lane = lane, from = fromDistance, to = toDistance,
      length = toDistance - fromDistance, fromNode = fromNode, toNode = toNode,
      enterLink = enterLink, exitLink = exitLink, wrap = wrap == true,
      nextEdges = {}
    }
    graph.edges[#graph.edges + 1] = edge
    fromNode.outgoing[#fromNode.outgoing + 1] = edge
    return edge
  end

  for i = 1, #graph.intersections do
    local inter = graph.intersections[i]
    local x, z = 0, 0
    for j = 1, #inter.points do x, z = x + inter.points[j][1], z + inter.points[j][3] end
    inter.node = addNode('intersection', vec3(x / #inter.points, inter.centerY, z / #inter.points), inter)
  end

  -- Split each directed lane at its ordered intersection spans. A movement
  -- connects the incoming segment to an outgoing segment at the same node.
  for i = 1, #lanes do
    local lane = lanes[i]
    if lane.valid then
      table.sort(lane.graphLinks, function(a, b) return a.from < b.from end)
      local length = lane.totalLength
      local links = lane.graphLinks
      if #links == 0 then
        local startNode = addNode('lane_start', lane.points[1], lane)
        local endNode = lane.loop and startNode or addNode('lane_end', lane.points[#lane.points], lane)
        addEdge(lane, 0, length, startNode, endNode, nil, nil, lane.loop)
      else
        local first, last = links[1], links[#links]
        local startNode = first.from > 0.5 and addNode('lane_start', lane.points[1], lane) or first.intersection.node
        local endNode = last.to < length - 0.5 and addNode('lane_end', lane.points[#lane.points], lane) or last.intersection.node
        local cursor, currentNode, previousLink = 0, startNode, nil
        local firstSegmentEdge = nil
        for j = 1, #links do
          local link = links[j]
          if link.from >= cursor - 0.1 then
            local edge = addEdge(lane, cursor, link.from, currentNode, link.intersection.node, previousLink, link, false)
            link.inEdge = edge
            if j == 1 then firstSegmentEdge = edge end
            if previousLink then previousLink.outEdge = edge end
            cursor = math.max(cursor, link.to)
            currentNode, previousLink = link.intersection.node, link
          else
            link.topologyError = 'overlapping_intersection_spans'
            graph.stats.rejectedIntersections = graph.stats.rejectedIntersections + 1
          end
        end
        if lane.loop then
          local edge = addEdge(lane, cursor, length, currentNode, startNode, previousLink, first, true)
          if previousLink then previousLink.outEdge = edge end
          if first.from <= 0.5 then
            first.inEdge = first.inEdge or edge
          elseif edge and firstSegmentEdge then
            edge.nextEdges[#edge.nextEdges + 1] = firstSegmentEdge
          end
        else
          local edge = addEdge(lane, cursor, length, currentNode, endNode, previousLink, nil, false)
          if previousLink then previousLink.outEdge = edge end
        end
      end
      if #lane.graphLinks == 0 then graph.stats.disconnectedLanes = graph.stats.disconnectedLanes + 1 end
    end
  end

  -- A lane crossing an intersection remains continuous along its own
  -- directed spline. This is not a lane-change movement and must never be lost.
  for i = 1, #graph.intersections do
    local inter = graph.intersections[i]
    for j = 1, #inter.links do
      local link = inter.links[j]
      if link.inEdge and link.outEdge then
        link.inEdge.nextEdges[#link.inEdge.nextEdges + 1] = link.outEdge
        local continuation = {
          intersection = inter, from = link, to = link,
          fromEdge = link.inEdge, toEdge = link.outEdge,
          continuation = true, priority = 0
        }
        inter.movements[#inter.movements + 1] = continuation
        graph.movements[#graph.movements + 1] = continuation
      end
    end
  end

  -- Honor explicit forbidden movements. Each accepted movement is an edge-to-
  -- edge transition; no transition is inferred from proximity alone.
  for i = 1, #graph.intersections do
    local inter = graph.intersections[i]
    for a = 1, #inter.links do
      local incoming = inter.links[a]
      if incoming.inEdge and incoming.from > 0.5 then
        for b = 1, #inter.links do
          local outgoing = inter.links[b]
          if outgoing.outEdge and outgoing.to < outgoing.lane.totalLength - 0.5 then
            local fromID, toID = tonumber(incoming.lane.id), tonumber(outgoing.lane.id)
            if incoming.lane ~= outgoing.lane and not disallowed(inter.definition, fromID, toID) then
              local movement = {
                intersection = inter, from = incoming, to = outgoing,
                fromEdge = incoming.inEdge, toEdge = outgoing.outEdge,
                priority = 0,
                attributes = nil
              }
              local attributes = inter.definition.trajectoryAttributes
              if type(attributes) == 'table' then
                for k = 1, #attributes do
                  local item = attributes[k]
                  if type(item) == 'table' and tonumber(item[1]) == fromID and tonumber(item[2]) == toID then
                    movement.attributes = item[3]
                    movement.priority = tonumber(item[3] and item[3].po) or 0
                    break
                  end
                end
              end
              inter.movements[#inter.movements + 1] = movement
              graph.movements[#graph.movements + 1] = movement
              incoming.inEdge.nextEdges[#incoming.inEdge.nextEdges + 1] = outgoing.outEdge
            end
          end
        end
      end
    end
  end
  graph.stats.movements = #graph.movements

  if log then
    log('CSP_GRAPH_BUILT lanes=%d valid=%d invalid=%d intersections=%d links=%d movements=%d edges=%d disconnected=%d rejected_intersections=%d',
      #lanes, graph.stats.validLanes, graph.stats.invalidLanes, #graph.intersections,
      graph.stats.intersectionLinks, graph.stats.movements, #graph.edges,
      graph.stats.disconnectedLanes, graph.stats.rejectedIntersections)
  end
  return graph
end

return Graph
