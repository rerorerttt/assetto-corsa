-- RoutePilot CSP Traffic Tool converter.
-- Converts the directed CSP lane network into one RoutePilot centerline and
-- one continuous lateral corridor. The source traffic.json is never modified.
local Converter = {}
local cachedGraph = nil

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

local function pointInside(poly, p)
  local inside=false
  local j=#poly
  for i=1,#poly do
    local a,b=poly[i],poly[j]
    if ((a.z>p.z) ~= (b.z>p.z)) then
      local x=(b.x-a.x)*(p.z-a.z)/(b.z-a.z)+a.x
      if p.x<x then inside=not inside end
    end
    j=i
  end
  return inside
end

local function segmentHit(a,b,c,d)
  local rx,rz=b.x-a.x,b.z-a.z
  local sx,sz=d.x-c.x,d.z-c.z
  local den=rx*sz-rz*sx
  if math.abs(den)<1e-8 then return nil end
  local qx,qz=c.x-a.x,c.z-a.z
  local t=(qx*sz-qz*sx)/den
  local u=(qx*rz-qz*rx)/den
  if t>=-1e-6 and t<=1+1e-6 and u>=-1e-6 and u<=1+1e-6 then
    return math.max(0,math.min(1,t))
  end
  return nil
end

local function buildIntersectionLinks(graph)
  local links={}
  for _,id in ipairs(graph.laneOrder) do links[id]={} end
  for ii=1,#graph.intersections do
    local inter=graph.intersections[ii]
    local poly=inter.points
    if #poly>=3 then
      for _,id in ipairs(graph.laneOrder) do
        local lane=graph.lanes[id]
        local cumulative=0
        local activeStart,activeEnd=nil,nil
        local function flushInterval()
          if activeStart and activeEnd and activeEnd-activeStart>0.5 then
            links[id][#links[id]+1]={
              intersection=inter,enter=activeStart,exit=activeEnd
            }
          end
          activeStart,activeEnd=nil,nil
        end
        for si=1,#lane.points-1 do
          local p1,p2=lane.points[si],lane.points[si+1]
          local segLen=math.sqrt(dist2(p1,p2))
          local cuts={0,1}
          for e=1,#poly do
            local t=segmentHit(p1,p2,poly[e],poly[e%#poly+1])
            if t then cuts[#cuts+1]=t end
          end
          table.sort(cuts)
          for ci=1,#cuts-1 do
            local t1,t2=cuts[ci],cuts[ci+1]
            local tm=(t1+t2)*0.5
            local mid={x=p1.x+(p2.x-p1.x)*tm,z=p1.z+(p2.z-p1.z)*tm}
            local d1=cumulative+segLen*t1
            local d2=cumulative+segLen*t2
            if pointInside(poly,mid) then
              if activeStart and d1<=activeEnd+0.15 then
                activeEnd=d2
              else
                flushInterval()
                activeStart,activeEnd=d1,d2
              end
            elseif d2-d1>0.02 then
              flushInterval()
            end
          end
          cumulative=cumulative+segLen
        end
        flushInterval()
      end
    end
  end
  for _,id in ipairs(graph.laneOrder) do
    table.sort(links[id],function(a,b) return a.enter<b.enter end)
  end
  return links
end

local function sampleLane(lane, distance)
  local remain=math.max(0,distance)
  for i=1,#lane.points-1 do
    local a,b=lane.points[i],lane.points[i+1]
    local length=math.sqrt(dist2(a,b))
    if remain<=length or i==#lane.points-1 then
      local t=length>1e-6 and math.max(0,math.min(1,remain/length)) or 0
      return {x=a.x+(b.x-a.x)*t,y=a.y+(b.y-a.y)*t,z=a.z+(b.z-a.z)*t}
    end
    remain=remain-length
  end
  return lane.points[#lane.points]
end

local function appendLaneRange(out,lane,fromDistance,toDistance)
  local cumulative=0
  local first=sampleLane(lane,fromDistance)
  append(out,first)
  for i=1,#lane.points-1 do
    local a,b=lane.points[i],lane.points[i+1]
    local length=math.sqrt(dist2(a,b))
    local d=cumulative+length
    if d>fromDistance+0.05 and d<toDistance-0.05 then append(out,b) end
    cumulative=d
    if cumulative>=toDistance then break end
  end
  append(out,sampleLane(lane,toDistance))
end

local function bezierConnector(out,fromLane,fromDistance,toLane,toDistance)
  local p0=sampleLane(fromLane,fromDistance)
  local p3=sampleLane(toLane,toDistance)
  local function tangent(lane,d)
    local p=sampleLane(lane,d)
    local q=sampleLane(lane,math.min(laneLength(lane),d+1.5))
    return norm(q.x-p.x,q.z-p.z)
  end
  local ax,az=tangent(fromLane,fromDistance)
  local bx,bz=tangent(toLane,toDistance)
  local gap=math.sqrt(dist2(p0,p3))
  local handle=math.max(2.0,math.min(14.0,gap*0.42+2.0))
  local p1={x=p0.x+ax*handle,y=p0.y,z=p0.z+az*handle}
  local p2={x=p3.x-bx*handle,y=p3.y,z=p3.z-bz*handle}
  local steps=math.max(5,math.min(24,math.ceil((gap+handle*2)/2.0)))
  for i=1,steps do
    local t=i/steps
    local u=1-t
    append(out,{
      x=u*u*u*p0.x+3*u*u*t*p1.x+3*u*t*t*p2.x+t*t*t*p3.x,
      y=u*u*u*p0.y+3*u*u*t*p1.y+3*u*t*t*p2.y+t*t*t*p3.y,
      z=u*u*u*p0.z+3*u*u*t*p1.z+3*u*t*t*p2.z+t*t*t*p3.z
    })
  end
end

local function selectTransition(graph, currentLane, link, laneLinks, visited)
  -- trajectoryAttributes in traffic.json describe curve/priority attributes;
  -- they are NOT an allow-list. CSP compatibility is controlled separately
  -- by disallowedTrajectories and its geometric intersection rules.
  local disallowed={}
  local source=link.intersection.source
  local forbidden=source and source.disallowedTrajectories
  if type(forbidden)=='table' then
    for i=1,#forbidden do
      local item=forbidden[i]
      if type(item)=='table' and item[1]==currentLane.id then
        disallowed[item[2]]=true
      end
    end
  end

  local candidates={}
  for _,id in ipairs(graph.laneOrder) do
    if not disallowed[id] and
        (id==currentLane.id or not visited[tostring(link.intersection.id)..':'..tostring(id)]) then
      local target=graph.lanes[id]
      for j=1,#(laneLinks[id] or {}) do
        local targetLink=laneLinks[id][j]
        if targetLink.intersection.id==link.intersection.id and
            targetLink.exit<laneLength(target)-0.5 then
          local p0=sampleLane(currentLane,link.enter)
          local p1=sampleLane(target,targetLink.exit)
          local gap=math.sqrt(dist2(p0,p1))
          local before=sampleLane(currentLane,math.max(0,link.enter-2))
          local after=sampleLane(target,math.min(laneLength(target),targetLink.exit+2))
          local ax,az=norm(p0.x-before.x,p0.z-before.z)
          local bx,bz=norm(after.x-p1.x,after.z-p1.z)
          local dot=ax*bx+az*bz
          if gap<65 and dot>-0.5 then
            local sameLaneBias=id==currentLane.id and -12.0 or 0
            local score=gap*0.45+(1-dot)*10+sameLaneBias
            candidates[#candidates+1]={lane=target,link=targetLink,score=score}
          end
        end
      end
    end
  end
  table.sort(candidates,function(a,b) return a.score<b.score end)
  return candidates[1]
end

local function collectRoute(graph, player)
  local hx,hz=norm(player.look.x,player.look.z)
  local start,startProjection,startScore
  for _,id in ipairs(graph.laneOrder) do
    local lane=graph.lanes[id]
    local p=project(lane.points,player.position,hx,hz)
    if p and p.distance<80 and (not startScore or p.score<startScore) then
      start,startProjection,startScore=lane,p,p.score
    end
  end
  if not start then return nil,nil,nil,'No CSP lane near the player in the driving direction' end

  -- If several same-direction CSP lanes overlap at the player, seed the
  -- longest usable lane in that parallel group. This avoids selecting a
  -- short merge stub when a continuous main lane is directly beside it.
  local nearestDistance=startProjection.distance
  local seedLength=laneLength(start)
  for _,id in ipairs(graph.laneOrder) do
    local candidate=graph.lanes[id]
    if candidate.id~=start.id then
      local p=project(candidate.points,player.position,hx,hz)
      if p and p.distance<=math.max(6.0,nearestDistance+1.5) then
        local alignment=p.tx*startProjection.tx+p.tz*startProjection.tz
        local length=laneLength(candidate)
        if alignment>=0.85 and length>seedLength*1.05 then
          start,startProjection,seedLength=candidate,p,length
        end
      end
    end
  end

  local laneLinks=buildIntersectionLinks(graph)
  local raw,visited={},{}
  local lane=start
  local progress=0
  for i=1,startProjection.segment-1 do
    local a,b=lane.points[i],lane.points[i+1]
    progress=progress+math.sqrt(dist2(a,b))
  end
  local a,b=lane.points[startProjection.segment],lane.points[startProjection.segment+1]
  progress=progress+math.sqrt(dist2(a,b))*startProjection.t
  local startProgress=progress
  local loop=false
  local routeLaneIDs={}
  local maxSteps=math.max(8,#graph.laneOrder*4)
  for _=1,maxSteps do
    routeLaneIDs[#routeLaneIDs+1]=lane.id
    local nextLink
    for _,candidate in ipairs(laneLinks[lane.id] or {}) do
      if candidate.enter>progress+0.5 then
        nextLink=candidate
        break
      elseif candidate.exit>progress+0.5 then
        -- The player may start inside a junction. Continue through the
        -- authored lane to its exit instead of attempting a backwards turn.
        appendLaneRange(raw,lane,progress,candidate.exit)
        progress=candidate.exit
      end
    end
    if not nextLink then
      local total=laneLength(lane)
      appendLaneRange(raw,lane,progress,total)
      if lane.id==start.id and lane.loop then
        appendLaneRange(raw,lane,0,startProgress)
        loop=true
      end
      break
    end
    appendLaneRange(raw,lane,progress,nextLink.enter)
    local visitKey=tostring(nextLink.intersection.id)..':'..tostring(lane.id)
    if visited[visitKey] then break end
    visited[visitKey]=true
    local transition=selectTransition(graph,lane,nextLink,laneLinks,visited)
    if not transition then break end
    local target=transition.lane
    local targetLink=transition.link
    local targetKey=tostring(nextLink.intersection.id)..':'..tostring(target.id)
    if target.id==lane.id then
      -- Same-lane continuation is valid. A different link on that same lane
      -- means the lane re-enters this intersection (e.g. a closed circuit).
      if targetLink~=nextLink then
        bezierConnector(raw,lane,nextLink.enter,target,targetLink.exit)
      end
      progress=targetLink.exit
    else
      if visited[targetKey] then
        if target.id==start.id then
          bezierConnector(raw,lane,nextLink.enter,target,targetLink.exit)
          if targetLink.exit<=startProgress+0.5 then
            if targetLink.exit<startProgress-0.5 then
              appendLaneRange(raw,target,targetLink.exit,startProgress)
            end
            loop=true
          elseif target.loop then
            appendLaneRange(raw,target,targetLink.exit,laneLength(target))
            appendLaneRange(raw,target,0,startProgress)
            loop=true
          end
        end
        break
      end
      visited[targetKey]=true
      bezierConnector(raw,lane,nextLink.enter,target,targetLink.exit)
      lane=target
      progress=targetLink.exit
    end
    if lane.id==start.id and progress<=startProgress+1.0 then
      if progress<startProgress-0.5 then
        appendLaneRange(raw,lane,progress,startProgress)
      end
      loop=true
      break
    end
  end
  if #raw<2 then return nil,nil,nil,'CSP route contains fewer than two points' end
  return raw,loop,routeLaneIDs,nil
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
    local laneCount=math.max(1,math.floor((hi-lo)/3.0+0.25)+1)
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
  if not cachedGraph then
    local graph, err = Network.loadTrackTraffic()
    if not graph then return nil,nil,nil,nil,err end
    cachedGraph=graph
  end
  local graph=cachedGraph
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
