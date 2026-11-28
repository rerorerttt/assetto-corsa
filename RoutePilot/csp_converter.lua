-- RoutePilot CSP Traffic Tool converter.
-- Converts the directed CSP lane network into one RoutePilot centerline and
-- one continuous lateral corridor. The source traffic.json is never modified.
local Converter = {}

local function finite(v)
  return type(v) == 'number' and v == v and v ~= math.huge and v ~= -math.huge
end

local function norm(x, z)
  local d = math.sqrt(x * x + z * z)
  if d < 1e-6 then return 0, 1 end
  return x / d, z / d
end

local function dist2(a, b)
  local x, z = a.x - b.x, a.z - b.z
  return x * x + z * z
end

local function project(points, p, hx, hz)
  local best
  for i = 1, #points - 1 do
    local a, b = points[i], points[i + 1]
    local vx, vz = b.x - a.x, b.z - a.z
    local l2 = vx * vx + vz * vz
    if l2 > 1e-5 then
      local t = math.max(0, math.min(1, ((p.x-a.x)*vx+(p.z-a.z)*vz)/l2))
      local q = {x=a.x+vx*t,y=a.y+(b.y-a.y)*t,z=a.z+vz*t}
      local tx, tz = norm(vx, vz)
      local align = hx * tx + hz * tz
      local d = dist2(q, p)
      local score = d + (1 - align) * (1 - align) * 25
      if align > 0.15 and (not best or score < best.score) then
        best = {point=q,tx=tx,tz=tz,distance=math.sqrt(d),score=score,segment=i,t=t}
      end
    end
  end
  return best
end

local function laneLength(lane)
  local n = 0
  for i = 1, #lane.points - 1 do
    n = n + math.sqrt(dist2(lane.points[i], lane.points[i + 1]))
  end
  return n
end

local function append(out, p)
  local last = out[#out]
  if not last or dist2(last, p) > 0.01 then
    out[#out + 1] = vec3(p.x, p.y, p.z)
  end
end

local function pointAt(lane, seg, t)
  local a, b = lane.points[seg], lane.points[seg + 1]
  return {x=a.x+(b.x-a.x)*t,y=a.y+(b.y-a.y)*t,z=a.z+(b.z-a.z)*t}
end

local function transitionScore(fromLane, toLane)
  local a, b = fromLane.points[#fromLane.points], toLane.points[1]
  local gap = math.sqrt(dist2(a, b))
  if gap > 35 then return nil end
  local ax, az = norm(a.x-fromLane.points[#fromLane.points-1].x,
    a.z-fromLane.points[#fromLane.points-1].z)
  local bx, bz = norm(toLane.points[2].x-b.x,toLane.points[2].z-b.z)
  local dot = ax*bx+az*bz
  if dot < -0.15 then return nil end
  return gap + (1-dot)*12
end

local function chooseNext(graph, lane, visited)
  local edges = graph.outgoing[lane.id] or {}
  local best, bestScore
  for i = 1, #edges do
    local edge = edges[i]
    local nextLane = graph.lanes[edge.to]
    if nextLane and not visited[nextLane.id] then
      local score = transitionScore(lane, nextLane)
      if score and (not bestScore or score < bestScore) then
        best, bestScore = nextLane, score
      end
    end
  end
  return best
end

local function collectRoute(graph, player)
  local hx, hz = norm(player.look.x, player.look.z)
  local start, startProjection, startScore
  for _, id in ipairs(graph.laneOrder) do
    local lane = graph.lanes[id]
    local p = project(lane.points, player.position, hx, hz)
    if p and p.distance < 80 and (not startScore or p.score < startScore) then
      start, startProjection, startScore = lane, p, p.score
    end
  end
  if not start then return nil, nil, nil, 'No CSP lane near the player in the driving direction' end

  local raw, visited = {}, {}
  local lane, first = start, true
  local loop = false
  local routeLaneIDs = {}
  for _ = 1, #graph.laneOrder + 2 do
    if not lane or visited[lane.id] then
      loop = lane ~= nil and lane.id == start.id
      break
    end
    visited[lane.id] = true
    routeLaneIDs[#routeLaneIDs + 1] = lane.id
    if first then
      append(raw, pointAt(lane, startProjection.segment, startProjection.t))
      for j = startProjection.segment + 1, #lane.points do append(raw, lane.points[j]) end
      if lane.loop then
        for j = 1, startProjection.segment do append(raw, lane.points[j]) end
      end
      first = false
    else
      -- A short connector bridges the gap between CSP lane endpoint and
      -- successor start; it is retained only for an explicitly directed edge.
      local prev = raw[#raw]
      local nextPoint = lane.points[1]
      local gap = math.sqrt(dist2(prev, nextPoint))
      if gap > 0.05 and gap < 35 then
        local steps = math.max(1, math.ceil(gap / 3))
        for k = 1, steps - 1 do
          local t = k / steps
          append(raw, {x=prev.x+(nextPoint.x-prev.x)*t,
            y=prev.y+(nextPoint.y-prev.y)*t,z=prev.z+(nextPoint.z-prev.z)*t})
        end
      end
      for j = 1, #lane.points do append(raw, lane.points[j]) end
    end
    local nextLane = chooseNext(graph, lane, visited)
    if not nextLane then
      if lane.id == start.id and lane.loop then loop = true end
      break
    end
    lane = nextLane
  end
  if #raw < 2 then return nil, nil, nil, 'CSP route contains fewer than two points' end
  return raw, loop, routeLaneIDs, nil
end

-- Find parallel, same-direction lanes around the chosen route. Their signed
-- offsets define the outer road envelope; inner lane boundaries disappear.
local function buildCorridor(graph, raw)
  local nodes, left, right = {}, {}, {}
  local sampleStep = 3.0
  local samples = {}
  for i = 1, #raw - 1 do
    local a, b = raw[i], raw[i+1]
    local dx, dz = b.x-a.x, b.z-a.z
    local length = math.sqrt(dx*dx+dz*dz)
    local count = math.max(1, math.ceil(length/sampleStep))
    for k = 0, count - 1 do
      local t = k/count
      samples[#samples+1] = {x=a.x+dx*t,y=a.y+(b.y-a.y)*t,z=a.z+dz*t}
    end
  end
  samples[#samples+1] = raw[#raw]

  for i = 1, #samples do
    local p = samples[i]
    local prev = samples[math.max(1,i-1)]
    local nxt = samples[math.min(#samples,i+1)]
    local tx,tz = norm(nxt.x-prev.x,nxt.z-prev.z)
    local nx,nz = -tz,tx
    local offsets = {0}
    for _, id in ipairs(graph.laneOrder) do
      local lane = graph.lanes[id]
      local pr = project(lane.points,p,tx,tz)
      if pr and pr.distance <= 11.0 then
        local ox,oz=pr.point.x-p.x,pr.point.z-p.z
        local lateral=ox*nx+oz*nz
        if math.abs(lateral) >= 1.0 and math.abs(lateral) <= 11.0 then
          offsets[#offsets+1]=lateral
        end
      end
    end
    table.sort(offsets)
    -- Keep only the contiguous lane group around the reference lane.
    local lo,hi=0,0
    for j=1,#offsets do
      if offsets[j] < 0 and offsets[j] >= lo-5.5 then lo=offsets[j] end
      if offsets[j] > 0 and offsets[j] <= hi+5.5 then hi=offsets[j] end
    end
    local laneCount=math.max(1,math.floor((hi-lo)/3.0+1.5))
    local width=laneCount==1 and 6.0 or laneCount*3.0
    local centerShift=(lo+hi)*0.5
    local center={x=p.x+nx*centerShift,y=p.y,z=p.z+nz*centerShift}
    if #nodes==0 or dist2(nodes[#nodes],center)>0.01 then
      nodes[#nodes+1]=vec3(center.x,center.y,center.z)
      left[#left+1]=-(width*0.5)
      right[#right+1]=width*0.5
    end
  end
  if #nodes < 2 then return nil,nil,nil end
  -- Align bounds with deduplicated node list.
  while #left > #nodes do table.remove(left) table.remove(right) end
  return nodes,left,right
end

function Converter.load(log)
  local function trace(fmt,...)
    if type(log)=='function' then pcall(log,fmt,...) end
  end
  local ok, Network = pcall(require, 'traffic_network')
  if not ok or type(Network)~='table' or type(Network.loadTrackTraffic)~='function' then
    return nil,nil,nil,nil,'traffic_network module unavailable: '..tostring(Network)
  end
  local graph, err = Network.loadTrackTraffic()
  if not graph then return nil,nil,nil,nil,err end
  local player = ac.getCar(0)
  if not player or not player.position or not player.look then
    return nil,nil,nil,nil,'player car unavailable'
  end
  local raw, loop, routeIDs, routeError = collectRoute(graph,player)
  if not raw then return nil,nil,nil,nil,routeError end
  local nodes,left,right = buildCorridor(graph,raw)
  if not nodes then return nil,nil,nil,nil,'failed to build RoutePilot corridor' end
  trace('CSP_CONVERT_OK source_lanes=%d route_lanes=%d points=%d loop=%s',
    graph.stats.validLanes,#routeIDs,#nodes,tostring(loop))
  return nodes,loop,left,right,{
    sourceLanes=graph.stats.validLanes,
    routeLanes=routeIDs,
    sourceTransitions=graph.stats.transitions,
    points=#nodes
  }
end

return Converter
