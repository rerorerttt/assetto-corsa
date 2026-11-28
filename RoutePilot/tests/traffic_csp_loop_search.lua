-- Regression: the first 64 longest cycle candidates exceed maxCost,
-- but a shorter 65th candidate is a valid directed loop.
package.path = "RoutePilot/?.lua;" .. package.path
local Graph = require("traffic_csp_graph")

local start = { id = 1, length = 10, nextEdges = {} }
local edges = { start }
local shortCandidate
for i = 1, 65 do
  local candidate = {
    id = i + 1,
    length = i == 65 and 10 or 99,
    nextEdges = { start }
  }
  if i == 65 then shortCandidate = candidate end
  start.nextEdges[#start.nextEdges + 1] = candidate
  edges[#edges + 1] = candidate
end

local path, cost = Graph.loopPath({ edges = edges }, start, 1000, 100)
assert(path, "loop search discarded the valid short cycle: " .. tostring(cost))
assert(#path == 2 and path[1] == start and path[2] == shortCandidate,
  "loop search returned the wrong cycle")
assert(math.abs(cost - 20) < 1e-6,
  "loop search returned an incorrect cycle cost: " .. tostring(cost))

print("PASS: loop search finds a feasible cycle beyond the former 64-candidate cutoff")
