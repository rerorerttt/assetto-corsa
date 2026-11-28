-- Convert a directed CSP graph path into a smooth per-car RoutePilot spline.
local Graph = require('traffic_csp_graph')
local Geometry = require('route_geometry')
local Routes = {}

local function distance(a, b)
  local dx, dy, dz = b.x-a.x, b.y-a.y, b.z-a.z
  return math.sqrt(dx*dx+dy*dy+dz*dz)
end

local function appendPoint(route, p, left, right)
  if not p then return end
  local last = route.nodes[#route.nodes]
  if last and distance(last, p) < 0.20 then
    local i = #route.nodes
    route.boundsL[i] = math.min(route.boundsL[i], left)
    route.boundsR[i] = math.max(route.boundsR[i], right)
    return
  end
  route.nodes[#route.nodes+1] = vec3(p.x, p.y, p.z)
  route.boundsL[#route.nodes] = left
  route.boundsR[#route.nodes] = right
end

local function laneBounds(roads, lane)
  local group = roads and roads.laneGroups and roads.laneGroups[lane]
  if not group then return 3.0, -3.0 end
  local offset = group.laneOffsets[lane] or 0
  local left = group.leftOffset + group.boundaryOffset - offset
  local right = group.rightOffset - group.boundaryOffset - offset
  return math.max(1.0, left), math.min(-1.0, right)
end

local function appendLaneRange(route, roads, edge, fromDistance, toDistance)
  local length = math.max(0, toDistance-fromDistance)
  local steps = math.max(1, math.ceil(length/2.0))
  local left, right = laneBounds(roads, edge.lane)
  for i = 0, steps do
    local d = fromDistance + length*i/steps
    local p = Graph.sampleLane(edge.lane, d)
    appendPoint(route, p, left, right)
  end
end

local function appendBezier(route, movement)
  local p0, p3 = movement.fromPos, movement.toPos
  if not p0 or not p3 then return false end
  local chord = distance(p0, p3)
  local handle = math.max(1.5, math.min(12.0, chord*0.42))
  local p1 = vec3(
    p0.x + movement.fromDir.x*handle,
    p0.y + movement.fromDir.y*handle,
    p0.z + movement.fromDir.z*handle)
  local p2 = vec3(
    p3.x - movement.toDir.x*handle,
    p3.y - movement.toDir.y*handle,
    p3.z - movement.toDir.z*handle)
  local steps = math.max(3, math.min(80, math.ceil(chord/1.2)))
  for i = 1, steps do
    local t = i/steps
    local u = 1-t
    local p = vec3(
      u*u*u*p0.x + 3*u*u*t*p1.x + 3*u*t*t*p2.x + t*t*t*p3.x,
      u*u*u*p0.y + 3*u*u*t*p1.y + 3*u*t*t*p2.y + t*t*t*p3.y,
      u*u*u*p0.z + 3*u*u*t*p1.z + 3*u*t*t*p2.z + t*t*t*p3.z)
    appendPoint(route, p, 3.5, -3.5)
  end
  return true
end

local function appendMovement(route, roads, movement)
  if not movement then return true end
  if movement.continuation then
    local link = movement.from
    local edge = movement.fromEdge
    appendLaneRange(route, roads, edge, link.from, link.to)
    return true
  end
  return appendBezier(route, movement)
end

function Routes.sample(route, distanceAlong)
  if not route or not route.geometry then return nil end
  return Geometry.sample(route.geometry, distanceAlong)
end

function Routes.build(graph, roads, startEdge, startDistance, options)
  options = options or {}
  if not graph or not startEdge then return nil, 'missing_start_edge' end
  local path, pathCost = Graph.shortestPath(
    graph, startEdge, options.targetEdge, options.maxVisited)
  if not path then return nil, pathCost end
  local loopRoute = false
  if options.loop == true then
    local terminal = path[#path]
    local returnPath = Graph.shortestPath(graph, terminal, startEdge, options.maxVisited)
    if returnPath and #returnPath >= 2 then
      local combined = {}
      for i = 1, #path do combined[#combined+1] = path[i] end
      for i = 2, #returnPath-1 do combined[#combined+1] = returnPath[i] end
      path = combined
      loopRoute = true
    elseif options.requireLoop then
      return nil, 'no_directed_return_path'
    end
  end
  local route = { nodes = {}, boundsL = {}, boundsR = {}, edges = path, loop = loopRoute }
  for i = 1, #path do
    local edge = path[i]
    local from = i == 1 and math.max(edge.from, startDistance or edge.from) or edge.from
    appendLaneRange(route, roads, edge, from, edge.to)
    local nextEdge = path[i+1] or (loopRoute and path[1] or nil)
    if nextEdge then
      local movement = edge.nextMovement and edge.nextMovement[nextEdge.id]
      if movement then
        if not appendMovement(route, roads, movement) then
          return nil, 'missing_intersection_curve'
        end
      else
        local exitPoint = Graph.sampleLane(edge.lane, edge.to)
        local entryPoint = Graph.sampleLane(nextEdge.lane, nextEdge.from)
        if not exitPoint or not entryPoint or distance(exitPoint, entryPoint) > 1.5 then
          return nil, 'missing_transition_metadata'
        end
      end
    end
    if #route.nodes > (options.maxPoints or 50000) then
      return nil, 'route_point_limit'
    end
  end
  if loopRoute and startDistance and startDistance > startEdge.from + 0.1 then
    -- Close the cycle through the skipped prefix of the first edge so the
    -- loop can begin at the player's projected position without a gap.
    appendLaneRange(route, roads, startEdge, startEdge.from, startDistance)
  end
  if loopRoute and #route.nodes > 2 and distance(route.nodes[1], route.nodes[#route.nodes]) < 0.20 then
    route.nodes[#route.nodes] = nil
    route.boundsL[#route.boundsL] = nil
    route.boundsR[#route.boundsR] = nil
  end
  if #route.nodes < 2 then return nil, 'route_too_short' end
  route.geometry = Geometry.build(route.nodes, loopRoute, 8)
  if not route.geometry then return nil, 'route_geometry_failed' end
  route.length = route.geometry.total or 0
  route.pathCost = pathCost
  route.startEdge = startEdge
  route.endEdge = path[#path]
  route.loop = loopRoute
  if route.length < (options.minLength or 80) then
    return nil, 'route_too_short'
  end
  return route
end

return Routes
