-- Importer regression: a nearby but vertically mismatched track surface
-- must not silently pull a traffic lane onto another road deck.
package.path = "RoutePilot/?.lua;" .. package.path

local vectorMethods = {}
function vectorMethods:clone()
  return vec3(self.x, self.y, self.z)
end
function vectorMethods:distance(other)
  local x, y, z = self.x-other.x, self.y-other.y, self.z-other.z
  return math.sqrt(x*x+y*y+z*z)
end
_G.vec3 = function(x, y, z)
  return setmetatable({x=x or 0,y=y or 0,z=z or 0},{__index=vectorMethods})
end

local raw = '{"lanes":[{"id":1,"loop":false,"points":[[-10,0,0],[0,0,0],[10,0,0]]}],"intersections":[]}'
_G.ac = { getTrackDataFilename = function() return "synthetic-traffic.json" end }
io.load = function() return raw end
_G.__util = { native = function() return 0 end }
_G.physics = {
  raycastTrack = function(origin, direction, length, filter, normal)
    normal.y = 1
    return _G.syntheticHitDistance
  end
}

local Importer = require("traffic_csp_import")
syntheticHitDistance = 2.0
local validImport = Importer.new()
assert(validImport:load(), "valid synthetic traffic.json did not load")
assert(validImport.stats.usable == 1 and validImport.lanes[1].valid,
  "road-level projection should preserve a valid open lane")
assert(validImport.graph and validImport.roads,
  "valid import should build both graph and road geometry")

syntheticHitDistance = 0.25
local wrongDeckImport = Importer.new()
assert(wrongDeckImport:load(), "wrong-deck sample should not abort the import")
assert(wrongDeckImport.stats.invalid == 1 and not wrongDeckImport.lanes[1].valid,
  "surface projection more than 1.5 m from source height must be rejected")

print("PASS: importer rejects vertically mismatched surface projections")
