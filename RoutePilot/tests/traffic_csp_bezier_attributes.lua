-- Verify that RoutePilot preserves CSP-authored entry/exit Bezier handle lengths.
package.path = "RoutePilot/?.lua;" .. package.path
_G.vec3 = function(x, y, z) return {x=x or 0,y=y or 0,z=z or 0} end
package.preload["route_geometry"] = function()
  return { build = function(nodes, loop) return {total=#nodes,nodes=nodes,loop=loop} end }
end
_G.physics = {
  raycastTrack = function(origin, direction, length, filter, normal)
    normal.y = 1
    return 2
  end
}
_G.__util = nil

package.preload["traffic_csp_graph"] = function()
  local Graph = {}
  function Graph.shortestPath(_, startEdge, targetEdge)
    return {startEdge,targetEdge}, 20
  end
  function Graph.sampleLane(lane, d)
    local a,b=lane.points[1],lane.points[2]
    local t=math.max(0,math.min(1,d/lane.totalLength))
    return vec3(a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,a.z+(b.z-a.z)*t)
  end
  function Graph.directionAt(lane)
    local a,b=lane.points[1],lane.points[2]
    local dx,dy,dz=b.x-a.x,b.y-a.y,b.z-a.z
    local n=math.sqrt(dx*dx+dy*dy+dz*dz)
    return vec3(dx/n,dy/n,dz/n)
  end
  return Graph
end

local Routes = require("traffic_csp_route")
local function lane(id, a, b)
  local dx,dy,dz=b.x-a.x,b.y-a.y,b.z-a.z
  local length=math.sqrt(dx*dx+dy*dy+dz*dz)
  return {id=id,points={a,b},totalLength=length,
    loop=false,speedLimitExplicit=false}
end

local a = lane(1, vec3(10,0,-10), vec3(10,0,0))
local b = lane(2, vec3(10,0,10), vec3(20,0,10))
local edgeA = {id=1,lane=a,from=0,to=10,length=10,nextEdges={}}
local edgeB = {id=2,lane=b,from=0,to=10,length=10,nextEdges={}}
local function roadGroupFor(lane)
  local left,right={},{}
  local a,b=lane.points[1],lane.points[2]
  local dx,dz=b.x-a.x,b.z-a.z
  local length=math.sqrt(dx*dx+dz*dz)
  local lx,lz=-dz/length,dx/length
  for i=1,2 do
    left[i]=vec3(lane.points[i].x+lx*3,lane.points[i].y,lane.points[i].z+lz*3)
    right[i]=vec3(lane.points[i].x-lx*3,lane.points[i].y,lane.points[i].z-lz*3)
  end
  return {boundariesVerified=true,boundaries={left=left,right=right},
    boundaryLoops={left=false,right=false},laneOffsets={[lane]=0},
    leftOffset=0,rightOffset=0,leftBoundaryOffset=3,rightBoundaryOffset=3}
end
local roads={laneGroups={[a]=roadGroupFor(a),[b]=roadGroupFor(b)}}
local intersection={points={{-50,0,-50},{50,0,-50},{50,0,50},{-50,0,50}}}
local movement = {
  from={lane=a}, to={lane=b},
  fromEdge=edgeA,toEdge=edgeB,
  fromPos=vec3(10,0,0),toPos=vec3(10,0,10),
  fromDir=vec3(0,0,1),toDir=vec3(1,0,0),
  intersection=intersection,
  attributes={cb=0.1,ce=0.9}
}
edgeA.nextMovement = {[edgeB.id]=movement}
local route, err = Routes.build({edges={edgeA,edgeB}}, roads, edgeA, 0,
  {targetEdge=edgeB,minLength=1})
assert(route, "route build failed: "..tostring(err))

-- Reproduce CSP's 30-point arc-length normalization for the authored
-- cubic. The route's first transition sample must lie at equal traveled
-- distance, not at an equal raw Bezier parameter.
local control1, control2 = vec3(10,0,1), vec3(1,0,10)
local samples, lengths, total = {vec3(10,0,0)}, {0}, 0
local previous = samples[1]
for i = 1, 30 do
  local t, u = i/30, 1-i/30
  local point = vec3(
    u*u*u*10 + 3*u*u*t*control1.x + 3*u*t*t*control2.x + t*t*t*10,
    0,
    3*u*u*t*control1.z + 3*u*t*t*control2.z + t*t*t*10)
  local dx,dz=point.x-previous.x,point.z-previous.z
  total=total+math.sqrt(dx*dx+dz*dz)
  samples[i+1],lengths[i+1]=point,total
  previous=point
end
local steps=math.max(3,math.min(120,math.ceil(total/1.2)))
local target=total/steps
local index=1
while index<30 and lengths[index+1]<target do index=index+1 end
local mix=(target-lengths[index])/(lengths[index+1]-lengths[index])
local expectedX=samples[index].x+(samples[index+1].x-samples[index].x)*mix
local firstTransitionPoint = route.nodes[7]
assert(firstTransitionPoint, "intersection transition was not sampled")
assert(math.abs(firstTransitionPoint.x-expectedX) < 1e-5,
  string.format("CSP Bezier attributes/arc-length sampling mismatch: x=%.6f expected=%.6f",
    firstTransitionPoint.x, expectedX))

-- Intersection projection must reject a raycast hit on another vertical
-- deck instead of silently snapping the trajectory by more than 1.5 m.
physics.raycastTrack=function(origin,direction,length,filter,normal)
  normal.y=1
  return 0.25
end
local wrongDeckRoute,wrongDeckError=Routes.build({edges={edgeA,edgeB}},roads,
  edgeA,0,{targetEdge=edgeB,minLength=1})
assert(not wrongDeckRoute and wrongDeckError=="intersection_surface_projection_failed",
  "intersection trajectory accepted a vertically mismatched surface")
physics.raycastTrack=function(origin,direction,length,filter,normal)
  normal.y=1
  return 2
end

local wideIntersection=movement.intersection
movement.intersection={points={{9,0,-1},{11,0,-1},{11,0,11},{9,0,11}}}
local outsideRoute,outsideError=Routes.build({edges={edgeA,edgeB}},roads,
  edgeA,0,{targetEdge=edgeB,minLength=1})
assert(not outsideRoute and outsideError=="unverified_intersection_boundaries",
  "trajectory leaving the exported intersection polygon was accepted")
movement.intersection=wideIntersection

-- An open route without an explicit target must never stop inside an
-- intersection merely because the directed graph has no outgoing movement.
edgeA.toNode={kind="intersection"}
local unsafeRoute,unsafeError=Routes.build({edges={edgeA}},roads,edgeA,0,{minLength=1})
assert(not unsafeRoute and unsafeError=="open_route_not_at_lane_end",
  "open route ending at an intersection was not rejected")

-- A CSP U-turn is two normalized Bezier segments joined at a midpoint
-- displaced by ul * chord along (fromDir - toDir).
local reverse = lane(3, vec3(10,0,10), vec3(10,0,0))
local reverseEdge = {id=3,lane=reverse,from=0,to=10,length=10,nextEdges={}}
roads.laneGroups[reverse]=roadGroupFor(reverse)
local uTurn = {
  from={lane=a}, to={lane=reverse},
  fromEdge=edgeA,toEdge=reverseEdge,
  fromPos=vec3(10,0,0),toPos=vec3(10,0,10),
  fromDir=vec3(0,0,1),toDir=vec3(0,0,-1),
  intersection=intersection,
  attributes={cb=0.1,ce=0.9,ul=0.3}
}
edgeA.nextMovement = {[reverseEdge.id]=uTurn}
local uTurnRoute, uTurnError = Routes.build({edges={edgeA,reverseEdge}}, roads,
  edgeA, 0, {targetEdge=reverseEdge,minLength=1})
assert(uTurnRoute, "CSP U-turn route build failed: "..tostring(uTurnError))
local maxTurnZ = -math.huge
for i=7,#uTurnRoute.nodes do maxTurnZ=math.max(maxTurnZ,uTurnRoute.nodes[i].z) end
assert(maxTurnZ > 10.7 and maxTurnZ < 12.2,
  string.format("CSP ul midpoint offset was not applied: maxZ=%.3f",maxTurnZ))

print("PASS: intersection route preserves CSP cb/ce arc-length curves and ul U-turn geometry")
