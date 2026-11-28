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
  if not group or not group.boundariesVerified or not group.boundaries then return nil, nil end
  local tx,tz=direction.x,direction.z
  local leftX,leftZ=-tz,tx
  local leftPoint,leftDistance=nearestBoundaryPoint(position,group.boundaries.left,group.boundaryLoops.left)
  local rightPoint,rightDistance=nearestBoundaryPoint(position,group.boundaries.right,group.boundaryLoops.right)
  if not leftPoint or not leftDistance or leftDistance>40 or
      not rightPoint or not rightDistance or rightDistance>40 then
    return nil, nil
  end
  local left=(leftPoint.x-position.x)*leftX+(leftPoint.z-position.z)*leftZ
  local right=(rightPoint.x-position.x)*leftX+(rightPoint.z-position.z)*leftZ
  -- Boundary lines can belong to a wider shared multi-lane corridor. Keep
  -- the local planning envelope bounded even if a distant edge is selected.
  left=math.min(left,8.0)
  right=math.max(right,-8.0)
  if left<1.25 or right>-1.25 then return nil, nil end
  return left, right
end

local function appendLaneRange(route, roads, edge, fromDistance, toDistance)
  local length = math.max(0, toDistance-fromDistance)
  local steps = math.max(1, math.ceil(length/2.0))
  for i = 0, steps do
    local d = fromDistance + length*i/steps
    local p = Graph.sampleLane(edge.lane, d)
    local direction = Graph.directionAt(edge.lane, d)
    local left, right = laneBoundsAt(roads, edge.lane, p, direction)
    if not left or not right then return false end
    local speedLimit=edge.lane.speedLimitExplicit and
      math.max(40,math.min(200,edge.lane.speedLimit or 200)) or 200
    appendPoint(route, p, left, right, speedLimit)
  end
  return true
end

local function polygonPoint(point)
  return point.x or point[1], point.z or point[3]
end

local function pointInsideIntersection(point, polygon)
  if type(polygon) ~= 'table' or #polygon < 3 then return false end
  local x,z=point.x,point.z
  local inside=false
  for i=1,#polygon do
    local j=i%#polygon+1
    local ax,az=polygonPoint(polygon[i])
    local bx,bz=polygonPoint(polygon[j])
    local ex,ez=bx-ax,bz-az
    local px,pz=x-ax,z-az
    local cross=ex*pz-ez*px
    local dot=px*ex+pz*ez
    local length2=ex*ex+ez*ez
    if math.abs(cross)<1e-4 and dot>=-1e-4 and dot<=length2+1e-4 then return true end
    if (az>z) ~= (bz>z) then
      local crossingX=ax+(z-az)*(bx-ax)/(bz-az)
      if x<crossingX then inside=not inside end
    end
  end
  return inside
end

local function rayPolygonDistance(point, dx, dz, polygon)
  local best=math.huge
  for i=1,#polygon do
    local j=i%#polygon+1
    local ax,az=polygonPoint(polygon[i])
    local bx,bz=polygonPoint(polygon[j])
    local ex,ez=bx-ax,bz-az
    local den=dx*ez-dz*ex
    if math.abs(den)>1e-8 then
      local rx,rz=ax-point.x,az-point.z
      local t=(rx*ez-rz*ex)/den
      local u=(rx*dz-rz*dx)/den
      if t>0 and u>=-1e-6 and u<=1+1e-6 and t<best then best=t end
    end
  end
  return best<math.huge and best or nil
end

local function intersectionBoundsAt(point, direction, intersection)
  local polygon=intersection and intersection.points
  if not pointInsideIntersection(point,polygon) then return nil,nil end
  local length=math.sqrt(direction.x*direction.x+direction.z*direction.z)
  if length<1e-6 then return nil,nil end
  local tx,tz=direction.x/length,direction.z/length
  local leftX,leftZ=-tz,tx
  local left=rayPolygonDistance(point,leftX,leftZ,polygon)
  local right=rayPolygonDistance(point,-leftX,-leftZ,polygon)
  if not left or not right then return nil,nil end
  left,right=left-0.5,right-0.5
  if left<1.25 or right<1.25 then return nil,nil end
  return left,right
end

local function projectIntersectionPoint(point)
  if not physics or type(physics.raycastTrack)~='function' then return nil end
  local probe=vec3(point.x,point.y+2,point.z)
  local normal=vec3()
  local ok,hit=pcall(function()
    return physics.raycastTrack(probe,vec3(0,-1,0),4,nil,normal)
  end)
  if not ok or type(hit)~='number' or hit<0 or normal.y<0.30 then return nil end
  local surfaceType=nil
  if __util and type(__util.native)=='function' then
    local okType,value=pcall(__util.native,'physics.raycastTrackSurface.type',probe,vec3(0,-1,0),4)
    if okType then surfaceType=value end
  end
  if surfaceType==1 or surfaceType==2 or surfaceType==6 then return nil end
  local projectedY=probe.y-hit
  if math.abs(projectedY-point.y)>1.5 then return nil end
  return vec3(point.x,projectedY,point.z)
end

local function appendBezier(route, roads, movement)
  local p0, p3 = movement.fromPos, movement.toPos
  if not p0 or not p3 then return false end
  local chord = distance(p0, p3)
  if chord < 0.1 then return false end
  local directionDot = movement.fromDir.x*movement.toDir.x +
    movement.fromDir.y*movement.toDir.y + movement.fromDir.z*movement.toDir.z
  local attributes = type(movement.attributes) == 'table' and movement.attributes or nil
  local cb = attributes and tonumber(attributes.cb) or 0.5
  local ce = attributes and tonumber(attributes.ce) or 0.5
  if cb ~= cb or cb == math.huge or cb == -math.huge then cb = 0.5 end
  if ce ~= ce or ce == math.huge or ce == -math.huge then ce = 0.5 end
  cb = math.max(0.02, math.min(2.0, cb))
  ce = math.max(0.02, math.min(2.0, ce))

  local curveSegments = {}
  local function addCurveSegment(a, dirA, b, dirB, handleA, handleB)
    local segmentChord = distance(a, b)
    local c1 = vec3(
      a.x + dirA.x*segmentChord*handleA,
      a.y + dirA.y*segmentChord*handleA,
      a.z + dirA.z*segmentChord*handleA)
    local c2 = vec3(
      b.x - dirB.x*segmentChord*handleB,
      b.y - dirB.y*segmentChord*handleB,
      b.z - dirB.z*segmentChord*handleB)
    local points, lengths = {}, { [1] = 0 }
    points[1] = a
    local total = 0
    local previous = a
    -- CSP's BezierCurveNormalized uses 30 base samples to build a
    -- distance-normalized trajectory. Mirror that arc-length parameterization
    -- before resampling the route at its own approximately 1.2 m spacing.
    for sample = 1, 30 do
      local t = sample/30
      local u = 1-t
      local point = vec3(
        u*u*u*a.x + 3*u*u*t*c1.x + 3*u*t*t*c2.x + t*t*t*b.x,
        u*u*u*a.y + 3*u*u*t*c1.y + 3*u*t*t*c2.y + t*t*t*b.y,
        u*u*u*a.z + 3*u*u*t*c1.z + 3*u*t*t*c2.z + t*t*t*b.z)
      total = total + distance(previous, point)
      points[sample+1] = point
      lengths[sample+1] = total
      previous = point
    end
    curveSegments[#curveSegments+1] = { points = points, lengths = lengths, length = total }
  end

  if directionDot < -0.8 then
    -- Match CSP's two-part U-turn trajectory rather than folding a 180-degree
    -- reversal into one cubic, which can cut through the island or curb.
    local uTurnSize = attributes and tonumber(attributes.ul) or nil
    if not uTurnSize or uTurnSize ~= uTurnSize or
        uTurnSize == math.huge or uTurnSize == -math.huge then
      uTurnSize = chord < 5 and 5 or math.min(4, 0.7*chord)
    else
      uTurnSize = uTurnSize*chord
    end
    local halfwayDir = vec3((p3.x-p0.x)/chord,(p3.y-p0.y)/chord,(p3.z-p0.z)/chord)
    local halfway = vec3(
      (p3.x+p0.x)*0.5 + (movement.fromDir.x-movement.toDir.x)*uTurnSize,
      (p3.y+p0.y)*0.5 + (movement.fromDir.y-movement.toDir.y)*uTurnSize,
      (p3.z+p0.z)*0.5 + (movement.fromDir.z-movement.toDir.z)*uTurnSize)
    addCurveSegment(p0, movement.fromDir, halfway, halfwayDir, cb, 0.5)
    addCurveSegment(halfway, halfwayDir, p3, movement.toDir, 0.5, ce)
  else
    addCurveSegment(p0, movement.fromDir, p3, movement.toDir, cb, ce)
  end

  local totalLength = 0
  for i = 1, #curveSegments do
    curveSegments[i].startDistance = totalLength
    totalLength = totalLength + curveSegments[i].length
  end
  if totalLength < 0.1 then return false end

  local leftA,rightA=laneBoundsAt(roads,movement.from.lane,p0,movement.fromDir)
  local leftB,rightB=laneBoundsAt(roads,movement.to.lane,p3,movement.toDir)
  if not leftA or not rightA or not leftB or not rightB then
    return false, 'unverified_road_boundaries'
  end
  local speedA=movement.from.lane.speedLimitExplicit and
    math.max(40,math.min(200,movement.from.lane.speedLimit or 200)) or 200
  local speedB=movement.to.lane.speedLimitExplicit and
    math.max(40,math.min(200,movement.to.lane.speedLimit or 200)) or 200
  local speedLimit=math.min(speedA,speedB)
  local steps = math.max(3, math.min(120, math.ceil(totalLength/1.2)))

  local function pointAtArcDistance(target)
    local segmentIndex = #curveSegments
    for i = 1, #curveSegments do
      local segment = curveSegments[i]
      if target <= segment.startDistance + segment.length or i == #curveSegments then
        segmentIndex = i
        break
      end
    end
    local segment = curveSegments[segmentIndex]
    local localDistance = math.max(0, math.min(segment.length, target-segment.startDistance))
    local lo, hi = 1, #segment.lengths-1
    while lo < hi do
      local mid = math.floor((lo+hi)*0.5)
      if segment.lengths[mid+1] < localDistance then lo = mid+1 else hi = mid end
    end
    local d0,d1=segment.lengths[lo],segment.lengths[lo+1]
    local t=d1-d0>1e-8 and (localDistance-d0)/(d1-d0) or 0
    local a,b=segment.points[lo],segment.points[lo+1]
    return vec3(a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,a.z+(b.z-a.z)*t)
  end

  for i = 1, steps do
    local arcDistance=totalLength*i/steps
    local p=pointAtArcDistance(arcDistance)
    local projected=projectIntersectionPoint(p)
    if not projected then return false, 'intersection_surface_projection_failed' end
    local before=pointAtArcDistance(math.max(0,arcDistance-0.25))
    local after=pointAtArcDistance(math.min(totalLength,arcDistance+0.25))
    local tangent=vec3(after.x-before.x,after.y-before.y,after.z-before.z)
    local left,right=intersectionBoundsAt(projected,tangent,movement.intersection)
    if not left or not right then
      if distance(projected,p0)<=4.0 then
        left,right=leftA,-rightA
      elseif distance(projected,p3)<=4.0 then
        left,right=leftB,-rightB
      else
        return false, 'unverified_intersection_boundaries'
      end
    end
    if left<1.25 or right<1.25 then
      return false, 'insufficient_intersection_clearance'
    end
    appendPoint(route, projected, left, -right, speedLimit)
  end
  return true
end

local function appendMovement(route, roads, movement)
  if not movement then return true end
  if movement.seamContinuation then
    if not appendLaneRange(route, roads, movement.fromEdge,
        movement.from.from, movement.from.to) then
      return false, 'unverified_road_boundaries'
    end
    if not appendLaneRange(route, roads, movement.toEdge,
        movement.to.from, movement.to.to) then
      return false, 'unverified_road_boundaries'
    end
    return true
  end
  if movement.continuation then
    local link = movement.from
    local edge = movement.fromEdge
    if not appendLaneRange(route, roads, edge, link.from, link.to) then
      return false, 'unverified_road_boundaries'
    end
    return true
  end
  return appendBezier(route, roads, movement)
end

function Routes.sample(route, distanceAlong)
  if not route or not route.geometry then return nil end
  return Geometry.sample(route.geometry, distanceAlong)
end

function Routes.project(route, position, heading)
  if not route or not route.geometry or not position then return nil end
  return Geometry.project(route.geometry,position,nil,heading,0,0,false)
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
  if not loopRoute and not options.targetEdge then
    local terminal = path[#path]
    if not terminal or not terminal.toNode or terminal.toNode.kind ~= 'lane_end' then
      return nil, 'open_route_not_at_lane_end'
    end
  end
  local route = { nodes = {}, boundsL = {}, boundsR = {}, speedLimits = {}, edges = path, loop = loopRoute }
  for i = 1, #path do
    local edge = path[i]
    local from = i == 1 and math.max(edge.from, startDistance or edge.from) or edge.from
    if not appendLaneRange(route, roads, edge, from, edge.to) then
      return nil, 'unverified_road_boundaries'
    end
    local nextEdge = path[i+1] or (loopRoute and path[1] or nil)
    if nextEdge then
      local movement = edge.nextMovement and edge.nextMovement[nextEdge.id]
      if movement then
        local movementOK, movementError = appendMovement(route, roads, movement)
        if not movementOK then
          return nil, movementError or 'missing_intersection_curve'
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
    route.speedLimits[#route.speedLimits] = nil
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
