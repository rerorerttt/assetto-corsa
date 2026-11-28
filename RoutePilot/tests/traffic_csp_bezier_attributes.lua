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
local route, err = Routes.build({edges={edgeA,edgeB}}, nil, edgeA, 0,
  {targetEdge=edgeB})
assert(route, "route build failed: "..tostring(err))

-- The first transition sample is t=1/9 on the cubic. CSP cb=0.1 and
-- ce=0.9 produce control points (10,0,1) and (19,0,10), respectively.
local t = 1/9
local u = 1-t
local expectedX = 10*u*u*u + 10*3*u*u*t + 19*3*u*t*t + 10*t*t*t
local firstTransitionPoint = route.nodes[7]
assert(firstTransitionPoint, "intersection transition was not sampled")
assert(math.abs(firstTransitionPoint.x-expectedX) < 1e-5,
  string.format("CSP Bezier entry/exit handles were not preserved: x=%.6f expected=%.6f",
    firstTransitionPoint.x, expectedX))

print("PASS: intersection route uses CSP cb/ce Bezier attributes")
