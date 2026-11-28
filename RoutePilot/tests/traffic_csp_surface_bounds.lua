-- Verify that RoutePilot measures physical road edges from CSP surface types
-- and never substitutes an assumed lane width when the surface has no edge.
package.path = "RoutePilot/?.lua;" .. package.path

local vectorMethods={}
function vectorMethods:clone() return vec3(self.x,self.y,self.z) end
function vectorMethods:distance(other)
  local x,y,z=self.x-other.x,self.y-other.y,self.z-other.z
  return math.sqrt(x*x+y*y+z*z)
end
_G.vec3=function(x,y,z)
  return setmetatable({x=x or 0,y=y or 0,z=z or 0},{__index=vectorMethods})
end
_G.ac={getTrackDataFilename=function() return "synthetic-road/traffic.json" end}
local raw='{"lanes":[{"id":1,"loop":false,"role":3,"points":[[-10,0,0],[0,0,0],[10,0,0]]}],"intersections":[]}'
io.load=function() return raw end
_G.physics={
  raycastTrack=function(origin,direction,length,filter,normal)
    normal.y=1
    return 2
  end
}
_G.__util={
  native=function(name,position)
    if math.abs(position.z)>4.1 then return 2 end -- grass beyond +/-4.0 m
    return 0 -- default drivable surface
  end
}

local Importer=require("traffic_csp_import")
local Routes=require("traffic_csp_route")
local importer=Importer.new()
assert(importer:load(),"synthetic road export failed to import: "..tostring(importer.error))
local lane=importer.lanes[1]
assert(lane.surfaceBoundsValid and lane.surfaceBounds.leftValid and lane.surfaceBounds.rightValid,
  "physical surface edges were not detected on both sides")
for i=1,#lane.points do
  assert(math.abs(lane.surfaceBounds.left[i]-4.0)<1e-6,
    "left edge width was not measured from surface transition")
  assert(math.abs(lane.surfaceBounds.right[i]-4.0)<1e-6,
    "right edge width was not measured from surface transition")
end
assert(importer.roads.stats.verifiedGroups==1 and importer.roads.stats.unverifiedGroups==0,
  "road corridor was not marked as physically verified")
local group=importer.roads.laneGroups[lane]
local mid=math.floor((#group.boundaries.left+1)/2)
assert(math.abs(group.boundaries.left[mid].z-4.0)<0.1,
  "left road boundary does not follow the measured surface edge")
assert(math.abs(group.boundaries.right[mid].z+4.0)<0.1,
  "right road boundary does not follow the measured surface edge")

local route,routeError=Routes.build(importer.graph,importer.roads,
  importer.graph.edges[1],0,{minLength=1})
assert(route and not route.loop,
  "route with verified physical boundaries failed: "..tostring(routeError))

-- A second export has no detectable surface transition within the 8 m probe
-- range. It must remain unverified instead of receiving a guessed 3 m width.
__util.native=function() return 0 end
local unbounded=Importer.new()
assert(unbounded:load(),"unbounded synthetic export failed to load")
assert(not unbounded.lanes[1].surfaceBoundsValid,
  "unbounded surface was incorrectly assigned an assumed road width")
assert(unbounded.roads.stats.verifiedGroups==0,
  "unbounded road corridor was incorrectly marked as verified")
local noRoute,noRouteError=Routes.build(unbounded.graph,unbounded.roads,
  unbounded.graph.edges[1],0,{minLength=1})
assert(not noRoute and noRouteError=="unverified_road_boundaries",
  "route was built despite missing physical road boundaries")

print("PASS: CSP surface probes establish road edges; unbounded corridors are rejected")
