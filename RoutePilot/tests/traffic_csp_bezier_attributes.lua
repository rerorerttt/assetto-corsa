-- Verify that RoutePilot preserves CSP-authored entry/exit Bezier handle lengths.
package.path = "RoutePilot/?.lua;" .. package.path
_G.vec3 = function(x, y, z) return {x=x or 0,y=y or 0,z=z or 0} end
package.preload["route_geometry"] = function()
  return {
    build = function(nodes, loop)
      return { total = #nodes, nodes = nodes, loop = loop }
    end
  }
end
_G.physics = {
  raycastTrack = function(origin, direction, length, filter, normal)
    normal.y = 1
    return 2
  end
}
_G.__util = nil

local Graph = require("traffic_csp_graph")
local Routes = require("traffic_csp_route")
local function lane(id, points)
  local length = 0
  local cumulative = { [1] = 0 }
  for i = 2, #points do
    local a, b = points[i-1], points[i]
    local dx, dy, dz = b.x-a.x, b.y-a.y, b.z-a.z
    length = length + math.sqrt(dx*dx+dy*dy+dz*dz)
    cumulative[i] = length
  end
  return {id=id,points=points,cumulative=cumulative,totalLength=length,
    loop=false,speedLimitExplicit=false}
end

local a = lane(1, {vec3(0,0,0), vec3(10,0,0)})
local b = lane(2, {vec3(10,0,10), vec3(20,0,10)})
local edgeA = {id=1,lane=a,from=0,to=10,length=10,nextEdges={}}
local edgeB = {id=2,lane=b,from=0,to=10,length=10,nextEdges={}}
edgeA.nextEdges[1] = edgeB
local movement = {
  from={lane=a}, to={lane=b},
  fromEdge=edgeA,toEdge=edgeB,
  fromPos=vec3(10,0,0),toPos=vec3(10,0,10),
  fromDir=vec3(0,0,1),toDir=vec3(1,0,0),
  attributes={cb=0.1,ce=0.9}
}
edgeA.nextMovement = {[edgeB.id]=movement}
local debugPath, debugCost = Graph.shortestPath({edges={edgeA,edgeB}}, edgeA, edgeB)
assert(debugPath and #debugPath == 2, "synthetic graph did not produce two-edge path")
assert(Graph.sampleLane(a, 0) and Graph.sampleLane(a, 5) and Graph.sampleLane(a, 10),
  "synthetic lane samples are unavailable")
local route, err = Routes.build({edges={edgeA,edgeB}}, nil, edgeA, 0,
  {targetEdge=edgeB})
assert(route, "route build failed: "..tostring(err))

-- Reproduce CSP's 30-point arc-length normalization for the authored
-- cubic. The route's first transition sample must lie at equal traveled
-- distance, not at an equal raw Bezier parameter.
local control1, control2 = vec3(10,0,1), vec3(19,0,10)
local samples, lengths, total = {vec3(10,0,0)}, {0}, 0
local previous = samples[1]
for i = 1, 30 do
  local t, u = i/30, 1-i/30
  local point = vec3(
    u*u*u*10 + 3*u*u*t*control1.x + 3*u*t*t*control2.x + t*t*t*10,
    0,
    u*u*u*0 + 3*u*u*t*control1.z + 3*u*t*t*control2.z + t*t*t*10)
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

print("PASS: intersection route uses CSP cb/ce Bezier attributes")
