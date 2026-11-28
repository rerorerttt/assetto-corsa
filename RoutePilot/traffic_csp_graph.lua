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
  if t >= -1e-6 and t <= 1 + 1e-6 and u >= -1e-6 and u <= 1 + 1e-6 then
    return math.max(0, math.min(1, t)), u
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
    if i == n and not lane.loop then return points[n] end
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
    if i == n and not lane.loop then
      local a, b = points[n-1], points[n]
      local dx, dy, dz = b.x-a.x, b.y-a.y, b.z-a.z
      local length = math.sqrt(dx*dx+dy*dy+dz*dz)
      if length > 1e-6 then return vec3(dx/length, dy/length, dz/length) end
    end
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
      local cuts = { { t = 0, side = nil }, { t = 1, side = nil } }
      for k = 1, #inter.points do
        local p, q = inter.points[k], inter.points[k % #inter.points + 1]
        local t = segmentCrossing(a.x, a.z, b.x, b.z, p[1], p[3], q[1], q[3])
        if t then cuts[#cuts + 1] = { t = t, side = k } end
      end
      table.sort(cuts, function(x, y) return x.t < y.t end)
      local unique = {}
      for k = 1, #cuts do
        local item = cuts[k]
        if #unique == 0 or math.abs(item.t - unique[#unique].t) > 1e-6 then
          unique[#unique + 1] = item
        elseif unique[#unique].side == nil then
          unique[#unique].side = item.side
        end
      end
      for k = 1, #unique - 1 do
        local t0, t1 = unique[k].t, unique[k + 1].t
        local tm = (t0 + t1) * 0.5
        local y = a.y + (b.y - a.y) * tm
        local inside = math.abs(y - inter.centerY) <= inter.yThreshold and pointInside(a.x + dx * tm, a.z + dz * tm, inter.points)
        local d0, d1 = total + segLen * t0, total + segLen * t1
        if inside then
          if not active then
            active = { from = d0, to = d1, fromSide = unique[k].side, toSide = unique[k + 1].side }
          else
            active.to, active.toSide = d1, unique[k + 1].side or active.toSide
          end
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
            local link = {
              intersection = inter, lane = lane, from = span.from, to = span.to,
              fromSide = span.fromSide, toSide = span.toSide,
              fromPos = lanePositionAt(lane, span.from),
              toPos = lanePositionAt(lane, span.to),
              fromDir = laneDirectionAt(lane, span.from),
              toDir = laneDirectionAt(lane, span.to)
            }
            inter.links[#inter.links + 1] = link
            lane.graphLinks[#lane.graphLinks + 1] = link
            graph.stats.intersectionLinks = graph.stats.intersectionLinks + 1
          end
        end
      end
    end
  end

  -- Recreate CSP Traffic Tool's polygon-side ordering. This provides stable
  -- lane ranks for turn restrictions without merging nearby road geometry.
  for i = 1, #graph.intersections do
    local inter = graph.intersections[i]
    inter.sides = {}
    for j = 1, #inter.links do
      local link = inter.links[j]
      if link.fromSide then
        local side = inter.sides[link.fromSide] or { entries = {}, exits = {} }
        inter.sides[link.fromSide] = side
        side.entries[#side.entries + 1] = link
      end
      if link.toSide then
        local side = inter.sides[link.toSide] or { entries = {}, exits = {} }
        inter.sides[link.toSide] = side
        side.exits[#side.exits + 1] = link
      end
    end
    for sideIndex, side in pairs(inter.sides) do
      local ex, ey, ez, xx, xy, xz = 0, 0, 0, 0, 0, 0
      local edx, edy, edz, xdx, xdy, xdz = 0, 0, 0, 0, 0, 0
      for j = 1, #side.entries do
        local link = side.entries[j]
        ex, ey, ez = ex + link.fromPos.x, ey + link.fromPos.y, ez + link.fromPos.z
        edx, edy, edz = edx + link.fromDir.x, edy + link.fromDir.y, edz + link.fromDir.z
      end
      for j = 1, #side.exits do
        local link = side.exits[j]
        xx, xy, xz = xx + link.toPos.x, xy + link.toPos.y, xz + link.toPos.z
        xdx, xdy, xdz = xdx + link.toDir.x, xdy + link.toDir.y, xdz + link.toDir.z
      end
      if #side.entries > 0 then
        side.centerEntries = vec3(ex/#side.entries, ey/#side.entries, ez/#side.entries)
        local l = math.sqrt(edx*edx+edy*edy+edz*edz)
        side.entryDir = l > 1e-6 and vec3(edx/l, edy/l, edz/l) or side.entries[1].fromDir
      end
      if #side.exits > 0 then
        side.centerExits = vec3(xx/#side.exits, xy/#side.exits, xz/#side.exits)
        local l = math.sqrt(xdx*xdx+xdy*xdy+xdz*xdz)
        side.exitDir = l > 1e-6 and vec3(xdx/l, xdy/l, xdz/l) or side.exits[1].toDir
      end
      if #side.entries > 0 and side.centerExits then
        table.sort(side.entries, function(a, b)
          return (a.fromPos.x-side.centerExits.x)^2 + (a.fromPos.z-side.centerExits.z)^2 <
            (b.fromPos.x-side.centerExits.x)^2 + (b.fromPos.z-side.centerExits.z)^2
        end)
        for j = 1, #side.entries do side.entries[j].entryRank = j end
      end
      if #side.exits > 0 and side.centerEntries then
        table.sort(side.exits, function(a, b)
          return (a.toPos.x-side.centerEntries.x)^2 + (a.toPos.z-side.centerEntries.z)^2 <
            (b.toPos.x-side.centerEntries.x)^2 + (b.toPos.z-side.centerEntries.z)^2
        end)
        for j = 1, #side.exits do side.exits[j].exitRank = j end
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
          fromPos = link.fromPos, toPos = link.toPos,
          fromDir = link.fromDir, toDir = link.toDir,
          continuation = true, priority = 0
        }
        inter.movements[#inter.movements + 1] = continuation
        graph.movements[#graph.movements + 1] = continuation
      end
    end
  end

  local function movementLaneRules(inter, incoming, outgoing)
    local entrySide = incoming.fromSide and inter.sides[incoming.fromSide] or nil
    local exitSide = outgoing.toSide and inter.sides[outgoing.toSide] or nil
    if entrySide and #entrySide.entries >= 2 and
        incoming.entryRank and incoming.entryRank > 1 and
        incoming.fromSide == outgoing.toSide then
      return false, false
    end
    if entrySide and exitSide and #entrySide.entries >= 2 and #exitSide.exits >= 2 and
        entrySide.entryDir and exitSide.exitDir then
      local dot = entrySide.entryDir.x*exitSide.exitDir.x +
        entrySide.entryDir.y*exitSide.exitDir.y + entrySide.entryDir.z*exitSide.exitDir.z
      if dot < 0.5 then
        local best, bestDistance = nil, math.huge
        for j = 1, #entrySide.entries do
          local candidate = entrySide.entries[j]
          local dx = candidate.fromPos.x - exitSide.centerExits.x
          local dz = candidate.fromPos.z - exitSide.centerExits.z
          local d = dx*dx + dz*dz
          if d < bestDistance then best, bestDistance = candidate, d end
        end
        local bestHasAllowedExit = false
        if best then
          for j = 1, #exitSide.exits do
            if not disallowed(inter.definition, tonumber(best.lane.id), tonumber(exitSide.exits[j].lane.id)) then
              bestHasAllowedExit = true
              break
            end
          end
        end
        if bestHasAllowedExit and best ~= incoming then return false, false end
      end
    end
    local weakLaneChange = false
    if entrySide and exitSide and #entrySide.entries >= 2 and
        #entrySide.entries == #exitSide.exits and entrySide.entryDir and exitSide.exitDir then
      local dot = entrySide.entryDir.x*exitSide.exitDir.x +
        entrySide.entryDir.y*exitSide.exitDir.y + entrySide.entryDir.z*exitSide.exitDir.z
      if dot > -0.5 and incoming.entryRank and outgoing.exitRank and
          incoming.entryRank ~= outgoing.exitRank then
        weakLaneChange = true
      end
    end
    return true, weakLaneChange
  end

  -- Honor explicit forbidden movements and CSP's key lane-order turn rules.
  -- Each accepted movement is an edge-to-edge transition, never a proximity link.
  for i = 1, #graph.intersections do
    local inter = graph.intersections[i]
    for a = 1, #inter.links do
      local incoming = inter.links[a]
      if incoming.inEdge and incoming.from > 0.5 then
        for b = 1, #inter.links do
          local outgoing = inter.links[b]
          if outgoing.outEdge and outgoing.to < outgoing.lane.totalLength - 0.5 then
            local fromID, toID = tonumber(incoming.lane.id), tonumber(outgoing.lane.id)
            local rulesAllow, weakLaneChange = movementLaneRules(inter, incoming, outgoing)
            if incoming.lane ~= outgoing.lane and rulesAllow and
                not disallowed(inter.definition, fromID, toID) then
              local movement = {
                intersection = inter, from = incoming, to = outgoing,
                fromEdge = incoming.inEdge, toEdge = outgoing.outEdge,
                fromPos = incoming.fromPos, toPos = outgoing.toPos,
                fromDir = incoming.fromDir, toDir = outgoing.toDir,
                priority = tonumber(incoming.lane.priority) or 0,
                weakLaneChange = weakLaneChange,
                trafficLight = inter.definition.trafficLight,
                attributes = nil
              }
              local entryOffsets = inter.definition.entryPriorityOffsets
              if type(entryOffsets) == 'table' then
                for k = 1, #entryOffsets do
                  local item = entryOffsets[k]
                  if type(item) == 'table' and tonumber(item.lane) == fromID then
                    movement.priority = movement.priority + (tonumber(item.offset) or 0)
                    break
                  end
                end
              end
              local attributes = inter.definition.trajectoryAttributes
              if type(attributes) == 'table' then
                for k = 1, #attributes do
                  local item = attributes[k]
                  if type(item) == 'table' and tonumber(item[1]) == fromID and tonumber(item[2]) == toID then
                    movement.attributes = item[3]
                    movement.priority = movement.priority + (tonumber(item[3] and item[3].po) or 0)
                    break
                  end
                end
              end
              inter.movements[#inter.movements + 1] = movement
              graph.movements[#graph.movements + 1] = movement
              incoming.inEdge.nextEdges[#incoming.inEdge.nextEdges + 1] = outgoing.outEdge
              incoming.inEdge.transitionPenalty = incoming.inEdge.transitionPenalty or {}
              local directionDot = incoming.fromDir.x*outgoing.toDir.x +
                incoming.fromDir.y*outgoing.toDir.y + incoming.fromDir.z*outgoing.toDir.z
              local penalty = math.max(0, -movement.priority) * 0.1
              if weakLaneChange then penalty = penalty + 8 end
              if directionDot < 0.5 then penalty = penalty + 4 end
              if directionDot < -0.5 then penalty = penalty + 12 end
              incoming.inEdge.transitionPenalty[outgoing.outEdge.id] = penalty
            end
          end
        end
      end
    end
  end
  -- Match CSP's movement ranking: straight trajectories have priority,
  -- U-turns are strongly deprioritized, and paths crossing a straight movement
  -- yield. This is a route cost, not a hard stop rule.
  local straightMovements = {}
  for i = 1, #graph.movements do
    local movement = graph.movements[i]
    if movement.fromDir and movement.toDir then
      local dot = movement.fromDir.x*movement.toDir.x +
        movement.fromDir.y*movement.toDir.y + movement.fromDir.z*movement.toDir.z
      movement.directionDot = dot
      if dot > 0.5 and movement.fromPos and movement.toPos then
        straightMovements[#straightMovements+1] = movement
      end
    end
  end
  for i = 1, #graph.movements do
    local movement = graph.movements[i]
    local dot = movement.directionDot
    if dot and dot < -0.5 then
      movement.priority = movement.priority - 200
    elseif dot and dot <= 0.5 and movement.fromPos and movement.toPos then
      for j = 1, #straightMovements do
        local straight = straightMovements[j]
        if straight ~= movement and straight.from ~= movement.from and
            straight.to ~= movement.to then
          local t, u = segmentCrossing(
            movement.fromPos.x, movement.fromPos.z, movement.toPos.x, movement.toPos.z,
            straight.fromPos.x, straight.fromPos.z, straight.toPos.x, straight.toPos.z)
          if t and t > 0.02 and t < 0.98 and u > 0.02 and u < 0.98 then
            local yA = movement.fromPos.y + (movement.toPos.y-movement.fromPos.y)*t
            local yB = straight.fromPos.y + (straight.toPos.y-straight.fromPos.y)*u
            if math.abs(yA-yB) <= 1.5 then
              movement.priority = movement.priority - 100
              break
            end
          end
        end
      end
    end
    if movement.fromEdge and movement.toEdge then
      movement.fromEdge.transitionPenalty = movement.fromEdge.transitionPenalty or {}
      local penalty = math.max(0, -movement.priority) * 0.1
      if movement.weakLaneChange then penalty = penalty + 8 end
      if dot and dot < 0.5 then penalty = penalty + 4 end
      if dot and dot < -0.5 then penalty = penalty + 12 end
      movement.fromEdge.transitionPenalty[movement.toEdge.id] = penalty
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


local function heapPush(heap, item)
  local i = #heap + 1
  heap[i] = item
  while i > 1 do
    local p = math.floor(i / 2)
    if heap[p].cost <= item.cost then break end
    heap[i], i = heap[p], p
  end
  heap[i] = item
end

local function heapPop(heap)
  local first = heap[1]
  local last = table.remove(heap)
  if #heap > 0 then
    local i, n = 1, #heap
    while true do
      local left, right = i*2, i*2+1
      if left > n then break end
      local child = right <= n and heap[right].cost < heap[left].cost and right or left
      if heap[child].cost >= last.cost then break end
      heap[i], i = heap[child], child
    end
    heap[i] = last
  end
  return first
end

-- Dijkstra over lane-segment edges. Returns an ordered edge path and total
-- cost; callers can choose a reachable terminal edge or a known destination.
function Graph.shortestPath(graph, startEdge, targetEdge, maxVisited)
  if not graph or not startEdge then return nil, 'missing_start_edge' end
  if targetEdge and startEdge == targetEdge then return { startEdge }, 0 end
  local distances, previous, heap = { [startEdge.id] = 0 }, {}, {}
  heapPush(heap, { edge = startEdge, cost = 0 })
  local visited, reached = 0, nil
  maxVisited = maxVisited or 20000
  while #heap > 0 and visited < maxVisited do
    local item = heapPop(heap)
    local edge, cost = item.edge, item.cost
    if cost <= (distances[edge.id] or math.huge) + 1e-7 then
      visited = visited + 1
      if targetEdge and edge == targetEdge then reached = edge; break end
      if not targetEdge and #edge.nextEdges == 0 then reached = edge; break end
      for i = 1, #edge.nextEdges do
        local nextEdge = edge.nextEdges[i]
        local transitionPenalty = edge.transitionPenalty and edge.transitionPenalty[nextEdge.id] or 0
        local nextCost = cost + math.max(0.1, nextEdge.length) + transitionPenalty
        if nextCost < (distances[nextEdge.id] or math.huge) then
          distances[nextEdge.id] = nextCost
          previous[nextEdge.id] = edge
          heapPush(heap, { edge = nextEdge, cost = nextCost })
        end
      end
    end
  end
  if not reached then return nil, visited >= maxVisited and 'search_limit' or 'no_path' end
  local path, cursor = {}, reached
  while cursor do
    table.insert(path, 1, cursor)
    if cursor == startEdge then break end
    cursor = previous[cursor.id]
  end
  if path[1] ~= startEdge then return nil, 'broken_predecessor_chain' end
  return path, distances[reached.id]
end

return Graph
