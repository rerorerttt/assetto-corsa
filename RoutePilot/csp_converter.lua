-- RoutePilot CSP Traffic Tool converter.
-- Converts EVERY authored CSP lane into an immutable directed network.
-- No road boundaries, synthetic centreline, lane merging or route truncation.
local Converter = {}

local function finite(v)
  return type(v) == "number" and v == v and v ~= math.huge and v ~= -math.huge
end
local function d2(a,b)
  local x,z=a.x-b.x,a.z-b.z
  return x*x+z*z
end
local function dist(a,b) return math.sqrt(d2(a,b)) end
local function norm(x,z)
  local l=math.sqrt(x*x+z*z)
  if l<1e-7 then return 0,1 end
  return x/l,z/l
end
local function clamp(v,a,b) return math.max(a,math.min(b,v)) end
local function pointAt(lane,s)
  s=clamp(s,0,lane.length)
  local lo,hi=1,#lane.cumulative-1
  while lo<hi do
    local mid=math.floor((lo+hi)/2)
    if lane.cumulative[mid+1]<s then lo=mid+1 else hi=mid end
  end
  local i=lo
  local a,b=lane.points[i],lane.points[i+1]
  local span=lane.cumulative[i+1]-lane.cumulative[i]
  local t=span>1e-8 and (s-lane.cumulative[i])/span or 0
  return {x=a.x+(b.x-a.x)*t,y=a.y+(b.y-a.y)*t,z=a.z+(b.z-a.z)*t}
end
local function tangentAt(lane,s)
  local p=pointAt(lane,s)
  local best,bestD
  for i=1,#lane.points-1 do
    local a,b=lane.points[i],lane.points[i+1]
    local vx,vz=b.x-a.x,b.z-a.z
    local l2=vx*vx+vz*vz
    if l2>1e-8 then
      local t=clamp(((p.x-a.x)*vx+(p.z-a.z)*vz)/l2,0,1)
      local qx,qz=a.x+vx*t,a.z+vz*t
      local dx,dz=p.x-qx,p.z-qz
      local dd=dx*dx+dz*dz
      if not bestD or dd<bestD then
        local tx,tz=norm(vx,vz)
        best={x=tx,z=tz}; bestD=dd
      end
    end
  end
  return best or {x=0,z=1}
end
local function makeLane(source)
  if type(source)~="table" or not finite(source.id) or type(source.points)~="table" then return nil end
  local points,cumulative={},{}
  local length=0
  for i=1,#source.points do
    local p=source.points[i]
    if type(p)~="table" or not finite(p[1]) or not finite(p[2]) or not finite(p[3]) then
      return nil,"INVALID_POINT"
    end
    local q={x=p[1],y=p[2],z=p[3]}
    if #points==0 or dist(points[#points],q)>0.015 then
      if #points>0 then length=length+dist(points[#points],q) end
      points[#points+1]=q
      cumulative[#points]=length
    end
  end
  if #points<2 or length<0.5 then return nil,"LANE_TOO_SHORT" end
  local loop=source.loop==true
  if loop and dist(points[1],points[#points])>0.015 then
    length=length+dist(points[#points],points[1])
    points[#points+1]={x=points[1].x,y=points[1].y,z=points[1].z}
    cumulative[#points]=length
  end
  return {
    id=source.id,name=tostring(source.name or ("Lane #"..source.id)),
    loop=loop,points=points,cumulative=cumulative,length=length,
    role=source.role or source.laneRole,source=source,
    outgoing={},incoming={},adjacent={},bounds=nil
  }
end

-- Extract the exact intervals in which a directed lane passes through a CSP
-- intersection polygon. Distances follow the source point order, so direction
-- is never inferred from proximity or the player's heading.
local function inside(poly,p)
  local hit,j=false,#poly
  for i=1,#poly do
    local a,b=poly[i],poly[j]
    if ((a.z>p.z)~=(b.z>p.z)) then
      local x=(b.x-a.x)*(p.z-a.z)/(b.z-a.z)+a.x
      if p.x<x then hit=not hit end
    end
    j=i
  end
  return hit
end
local function edgeCut(a,b,c,d)
  local rx,rz=b.x-a.x,b.z-a.z
  local sx,sz=d.x-c.x,d.z-c.z
  local den=rx*sz-rz*sx
  if math.abs(den)<1e-9 then return nil end
  local qx,qz=c.x-a.x,c.z-a.z
  local t=(qx*sz-qz*sx)/den
  local u=(qx*rz-qz*rx)/den
  if t>=-1e-7 and t<=1+1e-7 and u>=-1e-7 and u<=1+1e-7 then
    return clamp(t,0,1)
  end
end
local function intersectionIntervals(lane,poly)
  local intervals={}
  if #poly<3 then return intervals end
  local minY,maxY=math.huge,-math.huge
  for i=1,#poly do
    local y=poly[i].y or 0
    minY=math.min(minY,y); maxY=math.max(maxY,y)
  end
  local centerY=(minY+maxY)*0.5
  local yGate=5+(maxY-minY)
  local cumulative=0
  local activeStart,activeEnd=nil,nil
  local function flush()
    if activeStart and activeEnd and activeEnd-activeStart>0.35 then
      intervals[#intervals+1]={enter=activeStart,exit=activeEnd}
    end
    activeStart,activeEnd=nil,nil
  end
  for i=1,#lane.points-1 do
    local a,b=lane.points[i],lane.points[i+1]
    local len=dist(a,b)
    if math.abs(a.y-centerY)<=yGate and math.abs(b.y-centerY)<=yGate then
      local cuts={0,1}
      for e=1,#poly do
        local t=edgeCut(a,b,poly[e],poly[e%#poly+1])
        if t then cuts[#cuts+1]=t end
      end
      table.sort(cuts)
      for k=1,#cuts-1 do
        local t0,t1=cuts[k],cuts[k+1]
        if t1-t0>1e-7 then
          local tm=(t0+t1)*0.5
          local p={x=a.x+(b.x-a.x)*tm,z=a.z+(b.z-a.z)*tm}
          local s0,s1=cumulative+len*t0,cumulative+len*t1
          if inside(poly,p) then
            if activeStart and s0<=activeEnd+0.2 then activeEnd=s1
            else flush(); activeStart,activeEnd=s0,s1 end
          else flush() end
        end
      end
    else flush() end
    cumulative=cumulative+len
  end
  flush()
  return intervals
end
local function forbiddenPair(intersection,fromId,toId)
  local rows=intersection.disallowedTrajectories
  if type(rows)~="table" then return false end
  for i=1,#rows do
    local r=rows[i]
    if type(r)=="table" and r[1]==fromId and r[2]==toId then return true end
  end
  return false
end

local function buildJunctionTransitions(graph)
  local transitions={}
  for _,id in ipairs(graph.laneOrder) do transitions[id]={} end
  for _,intersection in ipairs(graph.intersections) do
    local passages={}
    for _,id in ipairs(graph.laneOrder) do
      local lane=graph.lanes[id]
      local intervals=intersectionIntervals(lane,intersection.points)
      if #intervals>0 then passages[#passages+1]={lane=lane,intervals=intervals} end
    end
    for i=1,#passages do
      local from=passages[i]
      for j=1,#passages do
        if i~=j then
          local to=passages[j]
          if not forbiddenPair(intersection.source,from.lane.id,to.lane.id) then
            for _,a in ipairs(from.intervals) do
              for _,b in ipairs(to.intervals) do
                -- Both polylines are already directed. Join the end of the
                -- incoming traversal to the beginning of the outgoing one.
                local pa=pointAt(from.lane,a.exit)
                local pb=pointAt(to.lane,b.enter)
                local ta=tangentAt(from.lane,a.exit)
                local tb=tangentAt(to.lane,b.enter)
                local alignment=ta.x*tb.x+ta.z*tb.z
                local gap=dist(pa,pb)
                if gap<=18 and alignment>=0.20 then
                  local e={from=from.lane.id,to=to.lane.id,
                    intersectionId=intersection.id,
                    fromS=a.exit,toS=b.enter,gap=gap,
                    alignment=alignment,kind="junction"}
                  transitions[from.lane.id][#transitions[from.lane.id]+1]=e
                end
              end
            end
          end
        end
      end
    end
  end
  for _,id in ipairs(graph.laneOrder) do
    table.sort(transitions[id],function(a,b)
      if a.fromS==b.fromS then
        if a.to==b.to then return a.toS<b.toS end
        return a.to<b.to
      end
      return a.fromS<b.fromS
    end)
    for _,e in ipairs(transitions[id]) do
      graph.lanes[id].outgoing[#graph.lanes[id].outgoing+1]=e
      graph.lanes[e.to].incoming[#graph.lanes[e.to].incoming+1]=e
    end
  end
  return transitions
end

-- Detect parallel, same-direction neighbouring lanes for legal lane changes.
-- A connection is emitted only where the two authored polylines overlap for
-- a meaningful distance and remain side-by-side. Opposing carriageways,
-- crossing ramps and merely nearby roads are rejected by tangent + distance.
local function buildLaneChanges(graph)
  local changes={}
  local cellSize=12.0
  local spatial={}
  local samplesByLane={}
  local function cell(x,z)
    local cx,cz=math.floor(x/cellSize),math.floor(z/cellSize)
    local col=spatial[cx]
    if not col then col={} spatial[cx]=col end
    local bucket=col[cz]
    if not bucket then bucket={} col[cz]=bucket end
    return bucket
  end
  -- Sample every authored lane once and build a spatial index. This avoids
  -- an O(points^2) comparison of long Shuto lane polylines.
  for _,id in ipairs(graph.laneOrder) do
    local lane=graph.lanes[id]
    local samples={}
    for distance=0,lane.length,8.0 do
      local p=pointAt(lane,distance)
      local t=tangentAt(lane,distance)
      local sample={laneId=id,s=distance,p=p,tx=t.x,tz=t.z}
      samples[#samples+1]=sample
      cell(p.x,p.z)[#cell(p.x,p.z)+1]=sample
    end
    samplesByLane[id]=samples
  end
  for ai=1,#graph.laneOrder do
    local aId=graph.laneOrder[ai]
    local a=graph.lanes[aId]
    for bi=ai+1,#graph.laneOrder do
      local bId=graph.laneOrder[bi]
      local b=graph.lanes[bId]
      local function scan(src,dst,out)
        local runStart,runLast,runLastTarget,runCount=nil,nil,nil,0
        local function flush()
          if runCount>=3 and runLast-runStart>=16 then
            out[#out+1]={fromS=runStart,toS=runLast,
              targetStart=runStartTarget,targetEnd=runLastTarget}
          end
          runStart,runLast,runLastTarget,runStartTarget,runCount=nil,nil,nil,nil,0
        end
        local srcSamples=samplesByLane[src.id]
        for si=1,#srcSamples do
          local sample=srcSamples[si]
          local p=sample.p
          local best,bestScore=nil,nil
          local cx,cz=math.floor(p.x/cellSize),math.floor(p.z/cellSize)
          for ox=-1,1 do
            local col=spatial[cx+ox]
            if col then
              for oz=-1,1 do
                local bucket=col[cz+oz]
                if bucket then
                  for k=1,#bucket do
                    local candidate=bucket[k]
                    if candidate.laneId==dst.id then
                      local dx,dz=candidate.p.x-p.x,candidate.p.z-p.z
                      local gap=math.sqrt(dx*dx+dz*dz)
                      if gap>=2.0 and gap<=7.5 and
                          math.abs(candidate.p.y-p.y)<=1.5 then
                        local alignment=sample.tx*candidate.tx+sample.tz*candidate.tz
                        if alignment>=0.94 then
                          local score=gap+(1-alignment)*8
                          if not bestScore or score<bestScore then
                            bestScore=score
                            best={s=sample.s,ds=candidate.s,gap=gap,alignment=alignment}
                          end
                        end
                      end
                    end
                  end
                end
              end
            end
          end
          if best then
            if runLast and (sample.s-runLast>12.1 or
                math.abs(best.ds-runLastTarget)>20.0) then flush() end
            if not runStart then
              runStart=sample.s
              runStartTarget=best.ds
            end
            runLast=sample.s
            runLastTarget=best.ds
            runCount=runCount+1
          else
            flush()
          end
        end
        flush()
      end
      local ab,ba={},{}
      scan(a,b,ab)
      scan(b,a,ba)
      for _,r in ipairs(ab) do
        local e={from=a.id,to=b.id,fromStart=r.fromS,fromEnd=r.toS,
          toStart=math.max(0,math.min(r.targetStart,r.targetEnd)-8.0),
          toEnd=math.min(b.length,math.max(r.targetStart,r.targetEnd)+8.0),
          kind="lane_change"}
        a.adjacent[#a.adjacent+1]=e
        changes[#changes+1]=e
      end
      for _,r in ipairs(ba) do
        local e={from=b.id,to=a.id,fromStart=r.fromS,fromEnd=r.toS,
          toStart=math.max(0,math.min(r.targetStart,r.targetEnd)-8.0),
          toEnd=math.min(a.length,math.max(r.targetStart,r.targetEnd)+8.0),
          kind="lane_change"}
        b.adjacent[#b.adjacent+1]=e
        changes[#changes+1]=e
      end
    end
  end
  return changes
end
function Converter.build(data,log)
  local function trace(fmt,...) if type(log)=="function" then pcall(log,fmt,...) end end
  if type(data)~="table" or type(data.lanes)~="table" then return nil,"TRAFFIC_JSON_MISSING_LANES" end
  local graph={lanes={},laneOrder={},intersections={},stats={
    sourceLanes=#data.lanes,validLanes=0,invalidLanes=0,
    sourceIntersections=0,junctionTransitions=0,laneChanges=0
  }}
  for i=1,#data.lanes do
    local lane,err=makeLane(data.lanes[i])
    if lane and not graph.lanes[lane.id] then
      graph.lanes[lane.id]=lane
      graph.laneOrder[#graph.laneOrder+1]=lane.id
      graph.stats.validLanes=graph.stats.validLanes+1
    else
      graph.stats.invalidLanes=graph.stats.invalidLanes+1
      trace("CSP_LANE_REJECT source_index=%d reason=%s",i,tostring(err or "DUPLICATE_OR_INVALID_ID"))
    end
  end
  table.sort(graph.laneOrder)
  local srcIntersections=type(data.intersections)=="table" and data.intersections or {}
  graph.stats.sourceIntersections=#srcIntersections
  for i=1,#srcIntersections do
    local src=srcIntersections[i]
    if type(src)=="table" and type(src.points)=="table" then
      local points={}
      for j=1,#src.points do
        local p=src.points[j]
        if type(p)=="table" and finite(p[1]) and finite(p[2]) and finite(p[3]) then
          points[#points+1]={x=p[1],y=p[2],z=p[3]}
        end
      end
      if #points>=3 then
        graph.intersections[#graph.intersections+1]={id=src.id,name=src.name,
          points=points,source=src}
      end
    end
  end
  buildJunctionTransitions(graph)
  local laneChanges=buildLaneChanges(graph)
  local junctionCount=0
  for _,id in ipairs(graph.laneOrder) do
    junctionCount=junctionCount+#graph.lanes[id].outgoing
  end
  graph.stats.junctionTransitions=junctionCount
  graph.stats.laneChanges=#laneChanges
  graph.stats.totalDirectedConnections=junctionCount+#laneChanges
  trace("CSP_NETWORK_CONVERTED lanes=%d rejected=%d junction_edges=%d lane_changes=%d intersections=%d",
    graph.stats.validLanes,graph.stats.invalidLanes,junctionCount,#laneChanges,#graph.intersections)
  return graph
end

function Converter.loadNetwork(log)
  local ok,Network=pcall(require,"traffic_network")
  if not ok or type(Network)~="table" or type(Network.loadTrackTraffic)~="function" then
    return nil,"TRAFFIC_NETWORK_MODULE_UNAVAILABLE: "..tostring(Network)
  end
  local source,err=Network.loadTrackTraffic()
  if not source then return nil,err end
  -- Rebuild from the original source records, retaining every lane and
  -- intersection. Network.loadTrackTraffic() returns the validated source graph.
  local data={lanes={},intersections={}}
  for _,id in ipairs(source.laneOrder or {}) do
    local lane=source.lanes[id]
    if lane and lane.source then data.lanes[#data.lanes+1]=lane.source end
  end
  for _,inter in ipairs(source.intersections or {}) do
    if inter.source then data.intersections[#data.intersections+1]=inter.source end
  end
  return Converter.build(data,log)
end

-- Compatibility entry point for older RoutePilot builds. It returns only the
-- nearest authored lane, never a synthetic route/corridor. New graph-based
-- driving code must consume loadNetwork() and traverse lane connections.
function Converter.load(log)
  local graph,err=Converter.loadNetwork(log)
  if not graph then return nil,nil,nil,nil,err end
  local player=ac.getCar(0)
  if not player or not player.position or not player.look then
    return nil,nil,nil,nil,"PLAYER_CAR_UNAVAILABLE"
  end
  local hx,hz=norm(player.look.x,player.look.z)
  local best,bestScore=nil,nil
  for _,id in ipairs(graph.laneOrder) do
    local lane=graph.lanes[id]
    for i=1,#lane.points-1 do
      local a,b=lane.points[i],lane.points[i+1]
      local vx,vz=b.x-a.x,b.z-a.z
      local l2=vx*vx+vz*vz
      if l2>1e-8 then
        local t=clamp(((player.position.x-a.x)*vx+(player.position.z-a.z)*vz)/l2,0,1)
        local q={x=a.x+vx*t,y=a.y+(b.y-a.y)*t,z=a.z+vz*t}
        local dx,dz=player.position.x-q.x,player.position.z-q.z
        local tx,tz=norm(vx,vz)
        local alignment=hx*tx+hz*tz
        local score=dx*dx+dz*dz+(1-alignment)*(1-alignment)*25
        if alignment>0.15 and math.sqrt(dx*dx+dz*dz)<80 and
          (not bestScore or score<bestScore) then
          bestScore=score; best={lane=lane,segment=i,t=t}
        end
      end
    end
  end
  if not best then return nil,nil,nil,nil,"NO_DIRECTED_CSP_LANE_NEAR_PLAYER" end
  local lane=best.lane
  local nodes={}
  for i=best.segment,#lane.points do
    local p=lane.points[i]
    nodes[#nodes+1]=vec3(p.x,p.y,p.z)
  end
  if #nodes<2 then return nil,nil,nil,nil,"SELECTED_CSP_LANE_TOO_SHORT" end
  return nodes,false,{}, {},{
    sourceLanes=graph.stats.validLanes,routeLanes={lane.id},
    sourceTransitions=graph.stats.junctionTransitions,
    laneChanges=graph.stats.laneChanges,network=graph
  }
end

return Converter
