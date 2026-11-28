-- Open-route racing-line planning must stop adding stations at the
-- terminal instead of repeating a clamped endpoint through the whole horizon.
package.path = "RoutePilot/?.lua;" .. package.path
local RacingLine = require("racing_line")

local route={loop=false,total=20}
local function sample(_,s)
  local d=math.max(0,math.min(route.total,s))
  return {x=d,y=0,z=0,tx=1,tz=0,k=0,s=d}
end
local function bounds()
  return -4,4
end
local plan,err=RacingLine.plan(route,sample,0,
  {aabbSize={x=2,z=4}},bounds,0,0,0,135,7.5,nil,0,22,10)
assert(plan,"open-route racing-line plan failed: "..tostring(err))
assert(plan.count==3,
  "planner extended past the 20 m open-route terminal: "..tostring(plan.count))
assert(math.abs(plan.horizon-15)<1e-6,
  "planner horizon must end at the last valid open-route station")
assert(plan.points[#plan.points].x==15,
  "planner must not duplicate the clamped terminal point")

print("PASS: racing-line planner stops at open-route terminal")
