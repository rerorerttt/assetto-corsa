-- Synthetic regression test for a closed CSP lane whose intersection spans
-- straddle the lane's distance-zero seam.
_G.vec3 = function(x, y, z)
  return { x = x or 0, y = y or 0, z = z or 0 }
end

package.path = "RoutePilot/?.lua;" .. package.path
package.preload["route_geometry"] = function()
  local Geometry = {}
  function Geometry.build(nodes, loop)
    local total = 0
    local function distance(a, b)
      local x, y, z = b.x-a.x, b.y-a.y, b.z-a.z
      return math.sqrt(x*x+y*y+z*z)
    end
    for i = 2, #nodes do total = total + distance(nodes[i-1], nodes[i]) end
    if loop and #nodes > 1 then total = total + distance(nodes[#nodes], nodes[1]) end
    return { total = total, nodes = nodes, loop = loop }
  end
  return Geometry
end

local Graph = require("traffic_csp_graph")
local Routes = require("traffic_csp_route")

local function point(x, z)
  return vec3(x, 0, z)
end

local lane = {
  id = 17,
  name = "seam-loop",
  valid = true,
  loop = true,
  params = {},
  speedLimit = 90,
  speedLimitExplicit = true,
  points = {
    point(0, 0), point(0, 10), point(10, 10), point(10, 0),
    point(10, -10), point(0, -10), point(-10, -10),
    point(-10, 0), point(-10, 10), point(-4, 10), point(-4, 0)
  }
}

local intersection = {
  id = 1,
  points = {
    { -5, 0, -5 }, { 5, 0, -5 }, { 5, 0, 5 }, { -5, 0, 5 }
  }
}

local graph = Graph.build({ lane }, { intersection })
assert(#lane.graphLinks >= 2, "expected intersection spans at both sides of the loop seam")
local first, last = lane.graphLinks[1], lane.graphLinks[#lane.graphLinks]
assert(first.from <= 0.5, "first span must begin at the lane seam")
assert(last.to >= lane.totalLength - 0.5, "last span must end at the lane seam")
assert(last.inEdge and first.outEdge and last.inEdge == first.outEdge,
  "seam spans must share their connecting self-edge")

local edge = last.inEdge
assert(edge.wrap and edge.fromNode == edge.toNode,
  "seam self-edge must be marked as a directed loop")
assert(edge.nextMovement and edge.nextMovement[edge.id] and
  edge.nextMovement[edge.id].seamContinuation,
  "seam self-transition metadata is missing")

local path, pathCost = Graph.loopPath(graph, edge)
assert(path and #path == 1 and path[1] == edge,
  "loop search must accept the seam self-cycle")
assert(pathCost and pathCost > 0, "loop path must have a positive cost")

local route, routeError = Routes.build(graph, nil, edge, edge.from, {
  loop = true, requireLoop = true, minLength = 1
})
assert(route, "route construction failed: " .. tostring(routeError))
assert(route.loop and #route.nodes > 20,
  "route must contain both the road span and the intersection seam spans")
local dx = route.nodes[1].x - route.nodes[#route.nodes].x
local dz = route.nodes[1].z - route.nodes[#route.nodes].z
assert(dx*dx + dz*dz < 2.5*2.5,
  "sampled loop must close near its starting point")

print("PASS: loop lane intersection seam remains connected and sampleable")
