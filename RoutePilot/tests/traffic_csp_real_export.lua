-- Import a real CSP Traffic Tool export committed with the RoutePilot branch.
-- Surface raycasts are stubbed at each source point's height; the actual
-- directed graph is built, while road-boundary raycasts remain disabled.
package.path = "RoutePilot/?.lua;" .. package.path

local vectorMethods = {}
function vectorMethods:clone()
  return vec3(self.x,self.y,self.z)
end
function vectorMethods:distance(other)
  local x,y,z=self.x-other.x,self.y-other.y,self.z-other.z
  return math.sqrt(x*x+y*y+z*z)
end
_G.vec3 = function(x,y,z)
  return setmetatable({x=x or 0,y=y or 0,z=z or 0},{__index=vectorMethods})
end

package.preload["shared/sim/surfaces"] = function() return false end

local file=assert(io.open("RoutePilot/tests/fixtures/traffic_20.json","rb"))
local raw=file:read("*a")
file:close()
_G.ac={getTrackDataFilename=function() return "fixture/traffic.json" end}
io.load=function() return raw end
_G.physics={
  raycastTrack=function(origin,direction,length,filter,normal)
    normal.y=1
    return 2
  end
}

local Importer=require("traffic_csp_import")
local Graph=require("traffic_csp_graph")
local Routes=require("traffic_csp_route")
local importer=Importer.new()
assert(importer:load(),"representative CSP export failed to import: "..tostring(importer.error))
assert(importer.stats.lanes==25,"unexpected lane count in fixture")
assert(#importer.intersections==14,"unexpected intersection count in fixture")
assert(importer.stats.usable>0,"no lane survived source sampling")
assert(importer.stats.invalid==0,
  "fixture lanes were unexpectedly rejected: "..tostring(importer.stats.invalid))
assert(importer.graph,"real CSP export did not produce a directed graph: "..tostring(importer.graphError))
assert(importer.graph.stats.intersectionLinks>0,
  "real CSP export produced no lane/intersection links")
assert(importer.graph.stats.movements>0,
  "real CSP export produced no directed intersection movements")
assert(importer.roads,"real CSP export did not produce road corridors: "..tostring(importer.roadsError))
assert(importer.roads.stats.validLanes==25 and importer.roads.stats.groups>0,
  "road corridor builder did not retain the imported lanes")
for i=1,#importer.lanes do
  local lane=importer.lanes[i]
  local group=importer.roads.laneGroups[lane]
  assert(group and group.boundaries.left and group.boundaries.right,
    "lane "..tostring(lane.id).." has no generated road boundaries")
  assert(#group.boundaries.left>=2 and #group.boundaries.right>=2,
    "lane "..tostring(lane.id).." has incomplete road boundaries")
end

local main,highway
for i=1,#importer.lanes do
  local lane=importer.lanes[i]
  if lane.role==3 then main=lane end
  if lane.role==4 then highway=lane end
end
assert(main and main.speedLimit==80 and main.priority==0,
  "default Main lane role was not resolved to 80 km/h / priority 0")
assert(highway and highway.speedLimit==90 and highway.priority==4,
  "Highway lane role was not resolved to 90 km/h / priority 4")

local cycleStarts=0
for i=1,#importer.graph.edges do
  local path=Graph.loopPath(importer.graph,importer.graph.edges[i],2000,20000)
  if path then cycleStarts=cycleStarts+1 end
end
local openRoutes,bestOpenLength=0,0
for i=1,#importer.graph.edges do
  local route=Routes.build(importer.graph,nil,importer.graph.edges[i],
    importer.graph.edges[i].from,{minLength=250})
  if route and not route.loop then
    openRoutes=openRoutes+1
    bestOpenLength=math.max(bestOpenLength,route.length or 0)
  end
end
assert(cycleStarts==0,"fixture unexpectedly gained a directed cycle; review route selection assumptions")
assert(openRoutes>0 and bestOpenLength>=250,
  "fixture has no usable open route for one-way traffic fallback")

print(string.format(
  "PASS: real CSP export imported %d/%d lanes, %d intersections, %d edges, %d links, %d movements, %d loop starts, %d valid open routes (max %.0f m)",
  importer.stats.usable,importer.stats.lanes,#importer.intersections,
  #importer.graph.edges,importer.graph.stats.intersectionLinks,
  importer.graph.stats.movements,cycleStarts,openRoutes,bestOpenLength))
