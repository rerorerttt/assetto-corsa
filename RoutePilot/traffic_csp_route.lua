-- Convert a directed CSP graph path into a smooth per-car RoutePilot spline.
local Graph = require('traffic_csp_graph')
local Geometry = require('route_geometry')
local Routes = {}

local function distance(a, b)
  local dx, dy, dz = b.x-a.x, b.y-a.y, b.z-a.z
  return math.sqrt(dx*dx+dy*dy+dz*dz)
end

local function appendPoint(route, p, left, right, speedLimit)
  if not p then return end
  local last = route.nodes[#route.nodes]
  if last and distance(last, p) < 0.20 then
    local i = #route.nodes
    route.boundsL[i] = math.max(route.boundsL[i], right)
    route.boundsR[i] = math.min(route.boundsR[i], left)
    route.speedLimits[i] = math.min(route.speedLimits[i] or speedLimit or 200,
      speedLimit or 200)
    return
  end
  route.nodes[#route.nodes+1] = vec3(p.x, p.y, p.z)
  -- RoutePilot stores lateral lower/upper limits: right is negative,
  -- left is positive. Keep this convention for the driver's safe corridor.
  route.boundsL[#route.nodes] = right
  route.boundsR[#route.nodes] = left
  route.speedLimits[#route.nodes] = speedLimit or 200
end

local function nearestBoundaryPoint(position, points, loop)
  if not points or #points < 2 then return nil end
  local n = #points
  local count = loop and n or n-1
  local step = math.max(1, math.floor(count/300))
  local bestDistance, bestPoint, bestIndex = math.huge, nil, nil
  local function testSegment(i)
    local nextIndex = i % n + 1
    local a, b = points[i], points[nextIndex]
    local vx, vy, vz = b.x-a.x, b.y-a.y, b.z-a.z
    local den = vx*vx+vy*vy+vz*vz
    if den > 1e-8 then
      local t = ((position.x-a.x)*vx+(position.y-a.y)*vy+(position.z-a.z)*vz)/den
      t = math.max(0, math.min(1, t))
      local q = vec3(a.x+vx*t,a.y+vy*t,a.z+vz*t)
      local dx,dy,dz = position.x-q.x,position.y-q.y,position.z-q.z
      local d = dx*dx+dy*dy+dz*dz
      if d < bestDistance then bestDistance,bestPoint,bestIndex=d,q,i end
    end
  end
  for i = 1, count, step do testSegment(i) end
  if bestIndex then
    if loop then
      for offset = -step, step do
        local i = (bestIndex+offset-1)%count+1
        testSegment(i)
      end
    else
      for i = math.max(1,bestIndex-step), math.min(count,bestIndex+step) do
        testSegment(i)
      end
    end
  end
  return bestPoint, bestDistance < math.huge and math.sqrt(bestDistance) or nil
end

local function laneBoundsAt(roads, lane, position, direction)
  local group = roads and roads.laneGroups and roads.laneGroups[lane]
  if not group or not group.boundaries then return 3.0, -3.0 end
  local tx,tz=direction.x,direction.z
  local leftX,leftZ=-tz,tx
  local leftPoint,leftDistance=nearestBoundaryPoint(position,group.boundaries.left,group.boundaryLoops.left)
  local rightPoint,rightDistance=nearestBoundaryPoint(position,group.boundaries.right,group.boundaryLoops.right)
  local left,right=nil,nil
  if leftPoint and leftDistance and leftDistance<=40 then
    left=(leftPoint.x-position.x)*leftX+(leftPoint.z-position.z)*leftZ
  end
  if rightPoint and rightDistance and rightDistance<=40 then
    right=(rightPoint.x-position.x)*leftX+(rightPoint.z-position.z)*leftZ
  end
  local offset=group.laneOffsets[lane] or 0
  local idealLeft=group.leftOffset+(group.leftBoundaryOffset or group.boundaryOffset)-offset
  local idealRight=group.rightOffset-(group.rightBoundaryOffset or group.boundaryOffset)-offset
  if not left or left<=0 then left=idealLeft end
  if not right or right>=0 then right=idealRight end
  return math.max(0.05,left), math.min(-0.05,right)
end

local function appendLaneRange(route, roads, edge, fromDistance, toDistance)
  local length = math.max(0, toDistance-fromDistance)
  local steps = math.max(1, math.ceil(length/2.0))
  for i = 0, steps do
    local d = fromDistance + length*i/steps
    local p = Graph.sampleLane(edge.lane, d)
    local direction = Graph.directionAt(edge.lane, d)
    local left, right = laneBoundsAt(roads, edge.lane, p, direction)
    local speedLimit=edge.lane.speedLimitExplicit and
      math.max(40,math.min(200,edge.lane.speedLimit or 200)) or 200
    appendPoint(route, p, left, right, speedLimit)
  end
end

local function appendBezier(route, roads, movement)
  local p0, p3 = movement.fromPos, movement.toPos
  if not p0 or not p3 then return false end
  local chord = distance(p0, p3)
  local directionDot = movement.fromDir.x*movement.toDir.x +
    movement.fromDir.y*movement.toDir.y + movement.fromDir.z*movement.toDir.z
  local handleFactor = directionDot < -0.5 and 0.22 or (directionDot < 0.5 and 0.30 or 0.42)
  local handle = math.max(1.0, math.min(12.0, chord*handleFactor))
  local p1 = vec3(
    p0.x + movement.fromDir.x*handle,
    p0.y + movement.fromDir.y*handle,
    p0.z + movement.fromDir.z*handle)
  local p2 = vec3(
    p3.x - movement.toDir.x*handle,
    p3.y - movement.toDir.y*handle,
    p3.z - movement.toDir.z*handle)
  local leftA,rightA=laneBoundsAt(roads,movement.from.lane,p0,movement.fromDir)
  local leftB,rightB=laneBoundsAt(roads,movement.to.lane,p3,movement.toDir)
  local halfWidth=math.max(0.05,math.min(3.5,leftA,-rightA,leftB,-rightB))
  local speedA=movement.from.lane.speedLimitExplicit and
    math.max(40,math.min(200,movement.from.lane.speedLimit or 200)) or 200
  local speedB=movement.to.lane.speedLimitExplicit and
    math.max(40,math.min(200,movement.to.lane.speedLimit or 200)) or 200
  local speedLimit=math.min(speedA,speedB)
  local steps = math.max(3, math.min(80, math.ceil(chord/1.2)))
  for i = 1, steps do
    local t = i/steps
    local u = 1-t
    local p = vec3(
      u*u*u*p0.x + 3*u*u*t*p1.x + 3*u*t*t*p2.x + t*t*t*p3.x,
      u*u*u*p0.y + 3*u*u*t*p1.y + 3*u*t*t*p2.y + t*t*t*p3.y,
      u*u*u*p0.z + 3*u*u*t*p1.z + 3*u*t*t*p2.z + t*t*t*p3.z)
    appendPoint(route, p, halfWidth, -halfWidth, speedLimit)
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
  return appendBezier(route, roads, movement)
end

function Routes.sample(route, distanceAlong)
  if not route or not route.geometry then return nil end
  return Geometry.sample(route.geometry, distanceAlong)
end

function Routes.build(graph, roads, startEdge, startDistance, options)
  options = options or {}
  if not graph or not startEdge then return nil, 'missing_start_edge' end
  local path, pathCost, loopRoute
  if options.loop == true then
    path, pathCost = Graph.loopPath(graph, startEdge, options.maxVisited, options.maxRouteLength or 20000)
    loopRoute = path ~= nil
    if not path then
      if options.requireLoop then return nil, pathCost end
      path, pathCost = Graph.shortestPath(
        graph, startEdge, options.targetEdge, options.maxVisited)
      if not path then return nil, pathCost end
      loopRoute = false
    end
  else
    path, pathCost = Graph.shortestPath(
      graph, startEdge, options.targetEdge, options.maxVisited)
    if not path then return nil, pathCost end
    loopRoute = false
  end
  local route = { nodes = {}, boundsL = {}, boundsR = {}, speedLimits = {}, edges = path, loop = loopRoute }
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
    if #route.nodes > (options.maxPoints or 12000) then
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
