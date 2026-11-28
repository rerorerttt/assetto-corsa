-- RoutePilot CSP Traffic Tool converter.
-- Converts EVERY authored CSP lane into an immutable directed network.
-- No road boundaries, synthetic centreline, lane merging or route truncation.
local Converter = {}
local makeLane
local okJSON, JSON = pcall(require, "routepilot_json")
if not okJSON or type(JSON) ~= "table" then JSON = nil end

local CACHE_VERSION = 1
local runtimeGraph = nil
local discoveryAttempted = false

local function trackCachePath()
  if not ac or not ac.getFolder or not ac.FolderID then return nil end
  local ok, root = pcall(ac.getFolder, ac.FolderID.ACApps)
  if not ok or type(root) ~= "string" or root == "" then return nil end
  local track = (ac.getTrackID and ac.getTrackID()) or "track"
  local layout = (ac.getTrackLayout and ac.getTrackLayout()) or ""
  local tag = tostring(track) .. (layout ~= "" and ("_" .. tostring(layout)) or "")
  tag = string.gsub(tag, "[^%w_%-]", "_")
  return root .. "/lua/RoutePilot/CspTracks/" .. tag .. ".json",
    root .. "/lua/RoutePilot/CspTracks"
end

local function cacheEdge(e)
  return {from=e.from,to=e.to,intersectionId=e.intersectionId,
    fromS=e.fromS,toS=e.toS,gap=e.gap,alignment=e.alignment,
    fromStart=e.fromStart,fromEnd=e.fromEnd,toStart=e.toStart,toEnd=e.toEnd,
    kind=e.kind}
end

local function saveNetworkCache(graph, log)
  if not JSON or not io or type(io.save) ~= "function" then return false end
  local path, folder = trackCachePath()
  if not path then return false end
  local alreadyThere=false
  if type(io.fileExists) == "function" then
    local ok, exists = pcall(io.fileExists, path)
    alreadyThere=ok and exists==true
  end
  if not alreadyThere and type(io.load)=="function" then
    local ok, existing=pcall(io.load,path,"")
    alreadyThere=ok and type(existing)=="string" and existing~=""
  end
  if alreadyThere then
    if type(log)=="function" then pcall(log,"CSP_CACHE_KEEP path=%s",path) end
    return true
  end
  if type(io.createDir) == "function" then pcall(io.createDir, folder) end
  local payload = {version=CACHE_VERSION, lanes={}, stats=graph.stats}
  for _, id in ipairs(graph.laneOrder) do
    local lane = graph.lanes[id]
    local row = {id=lane.id,name=lane.name,loop=lane.loop,points={},
      speedLimit=lane.speedLimit,priority=lane.priority,outgoing={},adjacent={}}
    for i=1,#lane.points do
      local p=lane.points[i]
      row.points[#row.points+1]={p.x,p.y,p.z}
    end
    for i=1,#lane.outgoing do row.outgoing[#row.outgoing+1]=cacheEdge(lane.outgoing[i]) end
    for i=1,#lane.adjacent do row.adjacent[#row.adjacent+1]=cacheEdge(lane.adjacent[i]) end
    payload.lanes[#payload.lanes+1]=row
  end
  local okEncode, raw = pcall(JSON.encode, payload)
  if not okEncode or type(raw)~="string" or raw=="" then return false end
  -- Never replace an existing track cache, even if it is stale or damaged.
  local existsNow=false
  if type(io.fileExists)=="function" then
    local ok,exists=pcall(io.fileExists,path)
    existsNow=ok and exists==true
  end
  if not existsNow and type(io.load)=="function" then
    local ok,existing=pcall(io.load,path,"")
    existsNow=ok and type(existing)=="string" and existing~=""
  end
  if existsNow then return true end
  local okSave = pcall(io.save, path, raw, true)
  if okSave and type(log)=="function" then
    pcall(log,"CSP_CACHE_SAVED path=%s lanes=%d bytes=%d",path,#payload.lanes,#raw)
  end
  return okSave
end

local function loadNetworkCache(log)
  if not JSON or not io or type(io.load) ~= "function" then return nil end
  local path = trackCachePath()
  if not path then return nil end
  local okLoad, raw = pcall(io.load, path, "")
  if not okLoad or type(raw)~="string" or raw=="" then return nil end
  local okDecode, payload = pcall(JSON.decode, raw)
  if not okDecode or type(payload)~="table" or payload.version~=CACHE_VERSION or
      type(payload.lanes)~="table" or #payload.lanes<1 then
    if type(log)=="function" then pcall(log,"CSP_CACHE_INVALID path=%s; source cache is preserved",path) end
    return nil
  end
  local graph={lanes={},laneOrder={},intersections={},stats=payload.stats or {}}
  for i=1,#payload.lanes do
    local row=payload.lanes[i]
    local lane=makeLane({id=row.id,name=row.name,loop=row.loop,points=row.points})
    if not lane or graph.lanes[lane.id] then
      if type(log)=="function" then pcall(log,"CSP_CACHE_INVALID_LANE path=%s index=%d",path,i) end
      return nil
    end
    lane.speedLimit=row.speedLimit
    lane.priority=row.priority or 0
    graph.lanes[lane.id]=lane
    graph.laneOrder[#graph.laneOrder+1]=lane.id
  end
  table.sort(graph.laneOrder)
  for i=1,#payload.lanes do
    local row=payload.lanes[i]
    local lane=graph.lanes[row.id]
    for j=1,#(row.outgoing or {}) do
      local e=row.outgoing[j]
      if graph.lanes[e.to] then
        lane.outgoing[#lane.outgoing+1]=e
        graph.lanes[e.to].incoming[#graph.lanes[e.to].incoming+1]=e
      end
    end
    for j=1,#(row.adjacent or {}) do
      local e=row.adjacent[j]
      if graph.lanes[e.to] then lane.adjacent[#lane.adjacent+1]=e end
    end
  end
  if type(log)=="function" then
    pcall(log,"CSP_CACHE_LOADED path=%s lanes=%d bytes=%d",path,#graph.laneOrder,#raw)
  end
  return graph
end

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
  s=clamp(s,0,lane.length)
  local c=lane.cumulative
  local lo,hi=1,#c-1
  while lo<hi do
    local mid=math.floor((lo+hi)/2)
    if c[mid+1]<s then lo=mid+1 else hi=mid end
  end
  local a,b=lane.points[lo],lane.points[lo+1]
  local tx,tz=norm(b.x-a.x,b.z-a.z)
  return {x=tx,z=tz}
end
makeLane = function(source)
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
  local minX,maxX,minY,maxY,minZ,maxZ=math.huge,-math.huge,math.huge,-math.huge,math.huge,-math.huge
  for i=1,#points do
    local p=points[i]
    minX=math.min(minX,p.x);maxX=math.max(maxX,p.x)
    minY=math.min(minY,p.y);maxY=math.max(maxY,p.y)
    minZ=math.min(minZ,p.z);maxZ=math.max(maxZ,p.z)
  end
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
    aabb={minX=minX,maxX=maxX,minY=minY,maxY=maxY,minZ=minZ,maxZ=maxZ},
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

local function trajectoryAttribute(intersection,fromId,toId)
  local rows=intersection.trajectoryAttributes
  if type(rows)~="table" then return nil end
  for i=1,#rows do
    local row=rows[i]
    if type(row)=="table" and row[1]==fromId and row[2]==toId then
      return type(row[3])=="table" and row[3] or nil
    end
  end
  return nil
end

local function buildJunctionTransitions(graph)
  local transitions={}
  for _,id in ipairs(graph.laneOrder) do transitions[id]={} end
  for _,intersection in ipairs(graph.intersections) do
    local passages={}
    local minX,maxX,minZ,maxZ=math.huge,-math.huge,math.huge,-math.huge
    for _,p in ipairs(intersection.points) do
      minX=math.min(minX,p.x);maxX=math.max(maxX,p.x)
      minZ=math.min(minZ,p.z);maxZ=math.max(maxZ,p.z)
    end
    -- Skip lane polylines whose bounding boxes cannot reach this junction.
    -- The generous margin includes segments whose endpoints lie outside it.
    local margin=22.0
    for _,id in ipairs(graph.laneOrder) do
      local lane=graph.lanes[id]
      local box=lane.aabb
      if box.maxX>=minX-margin and box.minX<=maxX+margin and
          box.maxZ>=minZ-margin and box.minZ<=maxZ+margin then
        local intervals=intersectionIntervals(lane,intersection.points)
        if #intervals>0 then passages[#passages+1]={lane=lane,intervals=intervals} end
      end
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
                local attributes=trajectoryAttribute(intersection.source,
                  from.lane.id,to.lane.id)
                -- Explicit CSP trajectory metadata can authorize sharp turns.
                -- Without it, require a close, forward-continuous geometric join
                -- so perpendicular crossing traffic is never treated as a route.
                local maxGap=attributes and 18.0 or 10.0
                local minAlignment=attributes and -0.25 or 0.50
                if gap<=maxGap and alignment>=minAlignment then
                  local e={from=from.lane.id,to=to.lane.id,
                    intersectionId=intersection.id,
                    fromS=a.exit,toS=b.enter,gap=gap,
                    alignment=alignment,trajectoryAttributes=attributes,
                    kind="junction"}
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
      local bucket=cell(p.x,p.z)
      bucket[#bucket+1]=sample
    end
    samplesByLane[id]=samples
  end
  -- Build only lane pairs whose samples occupy neighbouring spatial cells.
  -- This avoids comparing every lane against every other lane on large maps.
  local pairSet,pairsFound={},{}
  for _,id in ipairs(graph.laneOrder) do
    for _,sample in ipairs(samplesByLane[id]) do
      local cx,cz=math.floor(sample.p.x/cellSize),math.floor(sample.p.z/cellSize)
      for ox=-1,1 do
        local col=spatial[cx+ox]
        if col then
          for oz=-1,1 do
            local bucket=col[cz+oz]
            if bucket then
              for k=1,#bucket do
                local other=bucket[k].laneId
                if other~=id then
                  local a,b=id,other
                  if a>b then a,b=b,a end
                  local key=tostring(a)..":"..tostring(b)
                  if not pairSet[key] then
                    pairSet[key]=true
                    pairsFound[#pairsFound+1]={a=a,b=b}
                  end
                end
              end
            end
          end
        end
      end
    end
  end
  table.sort(pairsFound,function(a,b)
    if a.a==b.a then return a.b<b.b end
    return a.a<b.a
  end)
  local function scan(src,dst,out)
    local runStart,runLast,runLastTarget,runStartTarget,runCount=nil,nil,nil,nil,0
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
  for _,pair in ipairs(pairsFound) do
    local a,b=graph.lanes[pair.a],graph.lanes[pair.b]
    if a and b then
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
  if runtimeGraph then return runtimeGraph end
  if discoveryAttempted then return nil,"CSP_DISCOVERY_ALREADY_ATTEMPTED" end
  discoveryAttempted=true
  local cached=loadNetworkCache(log)
  if cached then runtimeGraph=cached;return runtimeGraph end
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
  local graph,buildError=Converter.build(data,log)
  if not graph then return nil,buildError end
  -- Carry resolved CSP lane-role limits into the runtime driving graph.
  for _,id in ipairs(source.laneOrder or {}) do
    local srcLane=source.lanes[id]
    local dstLane=graph.lanes[id]
    if srcLane and dstLane then
      dstLane.speedLimit=srcLane.speedLimit
      dstLane.priority=srcLane.priority
    end
  end
  -- Preserve source validation counts: the intermediate reader intentionally
  -- omits malformed records from its runtime lane table.
  if source.stats then
    graph.stats.sourceLanes=source.stats.sourceLanes or graph.stats.sourceLanes
    graph.stats.validLanes=source.stats.validLanes or graph.stats.validLanes
    graph.stats.invalidLanes=source.stats.invalidLanes or graph.stats.invalidLanes
    graph.stats.sourceIntersections=source.stats.sourceIntersections or graph.stats.sourceIntersections
  end
  saveNetworkCache(graph,log)
  runtimeGraph=graph
  return runtimeGraph
end

-- Compatibility entry point for older RoutePilot builds. It returns only the
-- nearest authored lane, never a synthetic route/corridor. New graph-based
-- driving code must consume loadNetwork() and traverse lane connections.
function Converter.load(log)
  local function trace(fmt,...)
    if type(log)=="function" then pcall(log,fmt,...) end
  end
  local graph,err=Converter.loadNetwork(log)
  if not graph then return nil,nil,nil,nil,err end
  local player=ac.getCar(0)
  if not player or not player.position or not player.look then
    return nil,nil,nil,nil,"PLAYER_CAR_UNAVAILABLE"
  end
  local best,bestScore=nil,nil
  if player and player.position and player.look then
    local hx,hz=norm(player.look.x,player.look.z)
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
            bestScore=score
            best={lane=lane,segment=i,t=t}
          end
        end
      end
    end
  end
  if not best then
    return nil,nil,nil,nil,"NO_CSP_LANE_NEAR_PLAYER"
  end

  local nodes,boundsL,boundsR={},{},{}
  local routeLanes,visited={},{}
  local anyPassing=false
  local firstPairedLaneId=nil
  local startLane=best.lane
  local startS=startLane.cumulative[best.segment]+
    (startLane.cumulative[best.segment+1]-startLane.cumulative[best.segment])*best.t
  local function appendPoint(p,left,right)
    local previous=nodes[#nodes]
    if previous then
      local dx,dz=p.x-previous.x,p.z-previous.z
      if dx*dx+dz*dz<0.01 then return end
    end
    nodes[#nodes+1]=vec3(p.x,p.y,p.z)
    boundsL[#boundsL+1]=left
    boundsR[#boundsR+1]=right
  end
  local function projectPartner(candidate,pos,hintS,sourceLane,sourceS)
    local first,last=1,#candidate.points-1
    if hintS then
      local c=candidate.cumulative
      local lo,hi=1,#c-1
      local target=math.max(0,hintS-28)
      while lo<hi do
        local mid=math.floor((lo+hi)*0.5)
        if c[mid+1]<target then lo=mid+1 else hi=mid end
      end
      first=lo
      lo,hi=first,#c-1
      target=math.min(candidate.length,hintS+28)
      while lo<hi do
        local mid=math.floor((lo+hi)*0.5)
        if c[mid]<target then lo=mid+1 else hi=mid end
      end
      last=math.max(first,lo)
    end
    local st=tangentAt(sourceLane,sourceS)
    local bestQ,bestScore=nil,nil
    for i=first,last do
      local a,b=candidate.points[i],candidate.points[i+1]
      local vx,vz=b.x-a.x,b.z-a.z
      local ll=vx*vx+vz*vz
      if ll>1e-8 then
        local t=clamp(((pos.x-a.x)*vx+(pos.z-a.z)*vz)/ll,0,1)
        local x,z=a.x+vx*t,a.z+vz*t
        local y=a.y+(b.y-a.y)*t
        local dx,dz=pos.x-x,pos.z-z
        local dy=pos.y-y
        local tx,tz=norm(vx,vz)
        local alignment=st.x*tx+st.z*tz
        local distance=math.sqrt(dx*dx+dz*dz)
        local score=dx*dx+dz*dz+(1-alignment)*(1-alignment)*16+dy*dy*4
        if alignment>=0.94 and math.abs(dy)<=1.5 and
          distance>=2.0 and distance<=7.5 and
          (not bestScore or score<bestScore) then
          bestScore=score
          bestQ={x=x,y=y,z=z,s=candidate.cumulative[i]+math.sqrt(ll)*t,distance=distance}
        end
      end
    end
    return bestQ
  end
  local function appendLaneSegment(lane,fromS,toS)
    fromS=clamp(fromS,0,lane.length)
    toS=clamp(toS,fromS,lane.length)
    if toS-fromS<0.25 then return end
    local samples={{s=fromS,p=pointAt(lane,fromS)}}
    for i=1,#lane.points do
      local ss=lane.cumulative[i]
      if ss>fromS+0.20 and ss<toS-0.20 then
        samples[#samples+1]={s=ss,p=lane.points[i]}
      end
    end
    samples[#samples+1]={s=toS,p=pointAt(lane,toS)}

    -- Keep every generated RoutePilot point on the exact authored
    -- Traffic Tool polyline. Never average adjacent yellow lines into a
    -- synthetic centreline: spawn and recovery use this same geometry.
    for i=1,#samples do
      appendPoint(samples[i].p,-2.0,2.0)
    end
  end

  local currentLane,currentS=startLane,startS
  local routeLoop=false
  local maxHops=#graph.laneOrder+1

  -- A junction edge is not automatically a continuation. CSP exports many
  -- short connector lanes which immediately point back to the lane they came
  -- from. Taking the first such edge truncates a multi-kilometre route after
  -- a few hundred metres. Require a meaningful forward exit on the target.
  local function targetHasForwardContinuation(target,entryS,sourceId,sourceFromS)
    if target.loop then return true end
    for _,edge in ipairs(target.outgoing or {}) do
      if edge.fromS>=entryS+20.0 and
        (edge.to~=sourceId or edge.toS>=sourceFromS+20.0) then
        return true
      end
    end
    for _,edge in ipairs(target.adjacent or {}) do
      if edge.fromEnd>=entryS+20.0 and
        (edge.to~=sourceId or (edge.toEnd or -math.huge)>=sourceFromS+20.0) then
        return true
      end
    end
    return false
  end

  -- Search for a complete directed circuit before committing to a route.
  -- This keeps the imported path continuous and prevents every car from
  -- reaching an arbitrary short connector/branch terminus.
  local searchNodes=0
  local searchLimit=50000
  local searchMaxHops=80
  local function routeCandidates(lane,fromS,visited)
    local candidates={}
    for _,edge in ipairs(lane.outgoing or {}) do
      local target=graph.lanes[edge.to]
      if target and finite(edge.fromS) and finite(edge.toS) and
        edge.fromS>=fromS-0.5 then
        local closesToStart=edge.to==startLane.id and
          (target.loop or edge.toS<startS-20.0 or math.abs(edge.toS-startS)<=20.0)
        if (not visited[edge.to] or closesToStart) and
          (closesToStart or targetHasForwardContinuation(
            target,edge.toS,lane.id,edge.fromS)) then
          candidates[#candidates+1]=edge
        end
      end
    end
    if #candidates>0 then
      table.sort(candidates,function(a,b)
        if math.abs(a.fromS-b.fromS)>2.0 then return a.fromS<b.fromS end
        local sa=(finite(a.alignment) and a.alignment or 0)*100-
          (finite(a.gap) and a.gap or 0)
        local sb=(finite(b.alignment) and b.alignment or 0)*100-
          (finite(b.gap) and b.gap or 0)
        return sa>sb
      end)
      -- Keep later exits available to DFS: an earlier exit can be a
      -- short connector/dead-end, while a later authored junction continues
      -- the actual road circuit.
      return candidates
    end

    local adjacent={}
    for _,edge in ipairs(lane.adjacent or {}) do
      local target=graph.lanes[edge.to]
      local fromEnd=finite(edge.fromEnd) and edge.fromEnd or nil
      local toEnd=finite(edge.toEnd) and edge.toEnd or nil
      if target and fromEnd and toEnd and fromEnd>=fromS+8.0 and
        not visited[target.id] and
        targetHasForwardContinuation(target,toEnd,lane.id,fromEnd) then
        local extent=fromEnd-math.max(fromS,edge.fromStart or fromS)
        adjacent[#adjacent+1]={from=lane.id,to=target.id,fromS=fromEnd,
          toS=clamp(toEnd,0,target.length),gap=dist(
            pointAt(lane,fromEnd),pointAt(target,clamp(toEnd,0,target.length))),
          alignment=1,kind="lane_change",_extent=extent}
      end
    end
    table.sort(adjacent,function(a,b) return a._extent>b._extent end)
    return adjacent
  end

  local function findCircuit(lane,sPos,visited,path,depth)
    searchNodes=searchNodes+1
    if searchNodes>searchLimit or depth>searchMaxHops then return nil end
    local options=routeCandidates(lane,sPos,visited)
    for _,edge in ipairs(options) do
      local target=graph.lanes[edge.to]
      if target then
        path[#path+1]=edge
        local closesToStart=edge.to==startLane.id and
          (target.loop or edge.toS<startS-20.0 or
            math.abs(edge.toS-startS)<=20.0)
        if closesToStart then return path end
        if not visited[target.id] then
          visited[target.id]=true
          local result=findCircuit(target,clamp(edge.toS,0,target.length),
            visited,path,depth+1)
          if result then return result end
          visited[target.id]=nil
        end
        path[#path]=nil
      end
    end
    return nil
  end

  local plannedRoute=findCircuit(startLane,startS,{[startLane.id]=true},{},0)
  if plannedRoute then
    trace("CSP_ROUTE_SEARCH cycle=true search_nodes=%d transitions=%d",searchNodes,#plannedRoute)
    for _,edge in ipairs(plannedRoute) do
      local sourceLane=graph.lanes[edge.from]
      local targetLane=graph.lanes[edge.to]
      if not sourceLane or not targetLane then break end
      routeLanes[#routeLanes+1]=sourceLane.id
      local segmentEnd=clamp(edge.fromS,currentS,sourceLane.length)
      appendLaneSegment(sourceLane,currentS,segmentEnd)
      local sourceEnd=pointAt(sourceLane,segmentEnd)
      local targetStart=pointAt(targetLane,edge.toS)
      local gap=dist(sourceEnd,targetStart)
      if gap>0.15 and gap<=18.0 then
        local steps=math.max(1,math.ceil(gap/3.0))
        for step=1,steps-1 do
          local t=step/steps
          appendPoint({x=sourceEnd.x+(targetStart.x-sourceEnd.x)*t,
            y=sourceEnd.y+(targetStart.y-sourceEnd.y)*t,
            z=sourceEnd.z+(targetStart.z-sourceEnd.z)*t},-2.0,2.0)
        end
      end
      if edge.to==startLane.id then
        if edge.toS<startS-20.0 then
          appendLaneSegment(startLane,edge.toS,startS)
        elseif startLane.loop and edge.toS>startS+20.0 then
          appendLaneSegment(startLane,edge.toS,startLane.length)
          appendLaneSegment(startLane,0,startS)
        end
        routeLoop=true
        trace("CSP_ROUTE_CLOSED via_lane=%d return_s=%.1f start_s=%.1f join=%.2f alignment=%.2f search_nodes=%d",
          startLane.id,edge.toS,startS,gap,edge.alignment or 0,searchNodes)
        break
      end
      currentLane=targetLane
      currentS=clamp(edge.toS,0,targetLane.length)
    end
  else
    trace("CSP_ROUTE_CYCLE_NOT_FOUND search_nodes=%d limit=%d; using bounded forward route",
      searchNodes,searchLimit)
    for hop=1,maxHops do
      if visited[currentLane.id] then break end
      visited[currentLane.id]=true
      routeLanes[#routeLanes+1]=currentLane.id
      local options=routeCandidates(currentLane,currentS,visited)
      local nextEdge=options[1]
      if not nextEdge then
        trace("CSP_ROUTE_TERMINUS lane=%d s=%.1f length=%.1f loop=%s junctions=%d adjacent=%d",
          currentLane.id,currentS,currentLane.length,tostring(currentLane.loop),
          #(currentLane.outgoing or {}),#(currentLane.adjacent or {}))
        appendLaneSegment(currentLane,currentS,currentLane.length)
        break
      end
      local segmentEnd=clamp(nextEdge.fromS,currentS,currentLane.length)
      appendLaneSegment(currentLane,currentS,segmentEnd)
      local sourceEnd=pointAt(currentLane,segmentEnd)
      local targetLane=graph.lanes[nextEdge.to]
      if not targetLane then break end
      local targetStart=pointAt(targetLane,nextEdge.toS)
      local gap=dist(sourceEnd,targetStart)
      if gap>0.15 and gap<=18.0 then
        local steps=math.max(1,math.ceil(gap/3.0))
        for step=1,steps-1 do
          local t=step/steps
          appendPoint({x=sourceEnd.x+(targetStart.x-sourceEnd.x)*t,
            y=sourceEnd.y+(targetStart.y-sourceEnd.y)*t,
            z=sourceEnd.z+(targetStart.z-sourceEnd.z)*t},-2.0,2.0)
        end
      end
      if nextEdge.to==startLane.id and
        (startLane.loop or nextEdge.toS<startS-20.0 or math.abs(nextEdge.toS-startS)<=20.0) then
        if nextEdge.toS<startS-20.0 then
          appendLaneSegment(startLane,nextEdge.toS,startS)
        elseif startLane.loop and nextEdge.toS>startS+20.0 then
          appendLaneSegment(startLane,nextEdge.toS,startLane.length)
          appendLaneSegment(startLane,0,startS)
        end
        routeLoop=true
        break
      end
      currentLane=targetLane
      currentS=clamp(nextEdge.toS,0,targetLane.length)
    end
    appendLaneSegment(currentLane,currentS,currentLane.length)
  end

  if #nodes<2 then return nil,nil,nil,nil,"SELECTED_CSP_ROUTE_TOO_SHORT" end
  local routeLength=0
  for i=2,#nodes do
    local dx,dz=nodes[i].x-nodes[i-1].x,nodes[i].z-nodes[i-1].z
    routeLength=routeLength+math.sqrt(dx*dx+dz*dz)
  end
  if routeLoop then
    local first,last=nodes[1],nodes[#nodes]
    local dx,dz=first.x-last.x,first.z-last.z
    routeLength=routeLength+math.sqrt(dx*dx+dz*dz)
  end
  if type(log)=="function" then
    pcall(log,"CSP_RACING_CORRIDOR lane=%d neighbour=%d paired=%s points=%d passing=%s",
      startLane.id,firstPairedLaneId or -1,tostring(anyPassing),#nodes,tostring(anyPassing))
    pcall(log,"CSP_ROUTE_CHAIN lanes=%d points=%d length=%.1f loop=%s transitions=%d",
      #routeLanes,#nodes,routeLength,tostring(routeLoop),math.max(0,#routeLanes-1))
  end
  return nodes,routeLoop,boundsL,boundsR,{
    routeLength=routeLength,
    sourceLanes=graph.stats.validLanes,routeLanes=routeLanes,
    racingCorridor=true,pairedLane=firstPairedLaneId,passingEnabled=anyPassing,
    sourceTransitions=graph.stats.junctionTransitions,
    laneChanges=graph.stats.laneChanges,network=graph
  }
end

return Converter