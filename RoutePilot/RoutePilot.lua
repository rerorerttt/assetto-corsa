-- RoutePilot.lua — v0.9.1 (CSP 0.3.2-preview2)
-- Existing route recording, boundaries, save/load, map and world-line UI are retained.
-- AI traffic uses two race lanes: initial assignments alternate left/right;
-- both lanes are unrestricted on straights and support safe lane changes.

local okLogger, Logger = pcall(require, "logger")
if not okLogger or type(Logger) ~= "table" then Logger = nil end
local okDriver, Driver = pcall(require, "driver")
if not okDriver or type(Driver) ~= "table" or not Driver.update then Driver = nil end

local CFG = {
  -- Aggressive race pace: late braking and high sustained lateral grip.
  MAX_LATERAL_ACCEL=17.5, BRAKE_ACCEL=14.0, PLAN_HORIZON=280.0,
  CURVE_MIN=0.0005,
  SPAWN_GAP=25.0, SPAWN_MIN_GAP=14.0, SPAWN_STABLE_TIME=0.35,
  LOG_INTERVAL=1.0, MAX_ROUTE_POINTS=50000, RECORD_DISTANCE=3.0,
  PREVIEW_AHEAD=30.0, PREVIEW_BACK=8.0,
  BOUNDS_MIN_PASS=3.0, SAFE_RESPAWN_CLEAR=8.0,
  MAP_TOP=175.0, MAP_PAD=6.0, MAP_MIN_OFFSET=10.0,
  VEHICLE_HALF_WIDTH=1.05, BOUNDARY_CLEARANCE=0.10,
  MIN_LANE_CENTER_GAP=2.35,
}
local WIDTH_PRESETS = {3, 7, 11, 15}

local state = {
  enabled=true, recLine=false, recBounds=false, loopRoute=true,
  widthPreset=1,
  boundsL={}, boundsR={}, recBoundsSeg=0, showWorldLines=false,
  spawnDone=false, spawnStable=0, spawnLastRoster="", routeDirectionChecked=false,
  nodes={}, p={}, laneAssignments={}, trackMap=nil, status="Готов", logAccum=0, loaded=false,
}
local ensureTrackMap
local logger=nil
if Logger and type(Logger.new)=="function" then
  local ok, result = pcall(function()
    local l=Logger.new("routepilot_trace.txt",6000); l:init(); return l
  end)
  if ok then logger=result end
end
local function writeLog(fmt,...)
  if not logger or not logger.log then return end
  pcall(logger.log, logger, fmt, ...)
end
local function flushLog()
  if logger and logger.flush then pcall(logger.flush, logger) end
end
local function clamp(v,lo,hi) return math.max(lo,math.min(hi,v)) end
local function lerp(a,b,t) return a+(b-a)*t end
local function flatDist(a,b)
  local dx,dz=b.x-a.x,b.z-a.z; return math.sqrt(dx*dx+dz*dz)
end
local function normalize2(x,z)
  local l=math.sqrt(x*x+z*z); if l<0.00001 then return 0,1 end
  return x/l, z/l
end
local function safeNumber(v,fb)
  if type(v)~="number" then return fb or 0 end
  if v~=v or v==math.huge or v==-math.huge then return fb or 0 end
  return v
end
local function getTrackTag()
  local tid=(ac.getTrackID and ac.getTrackID()) or "track"
  local layout=(ac.getTrackLayout and ac.getTrackLayout()) or ""
  if layout~="" then return tostring(tid).."_"..tostring(layout) end
  return tostring(tid)
end
local function getLineFilePath()
  local appDir=""
  if ac.getFolder and ac.FolderID then
    appDir=ac.getFolder(ac.FolderID.ACApps) or ""
  end
  local tag=string.gsub(getTrackTag(),"[^%w_%-]","_")
  if appDir~="" then return appDir.."/lua/RoutePilot/line_"..tag..".txt" end
  return "line_"..tag..".txt"
end
local function getBoundsFilePath()
  return string.gsub(getLineFilePath(),"%.txt$","_bounds.txt")
end
local function serializeNodes(nodes)
  local out={}
  for i=1,#nodes do
    local n=nodes[i]
    out[#out+1]=string.format("%.4f %.4f %.4f",n.x,n.y,n.z)
  end
  return table.concat(out,"\n")
end
local function parseNodes(raw)
  if type(raw)~="string" or raw=="" then return nil end
  local loaded={}
  for line in string.gmatch(raw,"[^\r\n]+") do
    local xs,ys,zs=string.match(line,"^%s*([%-0-9%.]+)%s+([%-0-9%.]+)%s+([%-0-9%.]+)%s*$")
    local x,y,z=tonumber(xs),tonumber(ys),tonumber(zs)
    if x and y and z and x==x and y==y and z==z then
      loaded[#loaded+1]=vec3(x,y,z)
    end
    if #loaded>=CFG.MAX_ROUTE_POINTS then break end
  end
  if #loaded>=2 then return loaded end
  return nil
end
local function saveBounds()
  local n=#state.nodes
  if n<2 then return end
  local parts={}
  for i=1,n do
    parts[#parts+1]=string.format("%.2f %.2f",
      safeNumber(state.boundsL[i],-1.5),safeNumber(state.boundsR[i],1.5))
  end
  if io and io.open then
    local ok,f=pcall(io.open,getBoundsFilePath(),"w")
    if ok and f then
      pcall(function() f:write(table.concat(parts,"\n")) f:close() end)
      writeLog("SAVE_BOUNDS points=%d",n)
    end
  end
end
local function loadBounds()
  local n=#state.nodes
  if n<2 then return false end
  if io and io.open then
    local ok,f=pcall(io.open,getBoundsFilePath(),"r")
    if ok and f then
      local success,raw=pcall(function()
        local c=f:read("*all"); f:close(); return c
      end)
      if success and type(raw)=="string" and raw~="" then
        local L,R={},{}
        for line in string.gmatch(raw,"[^\r\n]+") do
          local ls,rs=string.match(line,"^%s*([%-0-9%.]+)%s+([%-0-9%.]+)%s*$")
          L[#L+1]=tonumber(ls) or -1.5
          R[#R+1]=tonumber(rs) or 1.5
        end
        if #L==n then
          state.boundsL,state.boundsR=L,R
          writeLog("BOUNDS_LOADED points=%d",n)
          return true
        end
      end
    end
  end
  state.boundsL,state.boundsR={},{}
  local half=WIDTH_PRESETS[state.widthPreset]/2
  for i=1,n do state.boundsL[i]=-half state.boundsR[i]=half end
  writeLog("BOUNDS_DEFAULT points=%d",n)
  return false
end
local function saveLine()
  if #state.nodes<2 then state.status="Недостаточно точек" return end
  local data=serializeNodes(state.nodes)
  local wroteFile=false
  if io and io.open then
    local ok,f=pcall(io.open,getLineFilePath(),"w")
    if ok and f then wroteFile=pcall(function() f:write(data) f:close() end) end
  end
  if ac.storage then pcall(ac.storage,"RP_LINE_DATA_"..getTrackTag(),data) end
  saveBounds()
  if wroteFile then
    state.trackMap=nil
    ensureTrackMap()
    state.loaded=true
    state.spawnDone=false
    state.spawnStable=0
    state.spawnLastRoster=""
    state.status=string.format("Сохранено: %d точек",#state.nodes)
    writeLog("SAVE_ROUTE points=%d",#state.nodes)
    flushLog()
  else state.status="Ошибка сохранения" end
end
local function loadLine()
  local loaded=nil
  if io and io.open then
    local ok,f=pcall(io.open,getLineFilePath(),"r")
    if ok and f then
      local success,raw=pcall(function()
        local c=f:read("*all"); f:close(); return c
      end)
      if success then loaded=parseNodes(raw) end
    end
  end
  if not loaded and ac.storage then
    local ok,raw=pcall(ac.storage,"RP_LINE_DATA_"..getTrackTag())
    if ok and raw then loaded=parseNodes(raw) end
  end
  if loaded then
    state.nodes=loaded; state.p={}; state.trackMap=nil; state.loaded=true
    state.spawnDone=false; state.spawnStable=0; state.spawnLastRoster=""
    state.routeDirectionChecked=false
    state.status=string.format("Загружено: %d точек",#loaded)
    writeLog("LOAD_ROUTE points=%d track=%s",#loaded,getTrackTag())
    loadBounds()
    ensureTrackMap()
    flushLog()
    return true
  end
  state.status="Линия не найдена"; state.loaded=false
  return false
end
local function segmentCount()
  if state.loopRoute then return #state.nodes end
  return math.max(0,#state.nodes-1)
end
local function segEnd(index)
  local count=#state.nodes
  if state.loopRoute then return state.nodes[(index%count)+1] end
  return state.nodes[index+1]
end
local function walkLine(startSeg,startT,distance)
  local segs=segmentCount()
  if segs<1 then return nil end
  local seg=startSeg; local t=startT; local remaining=distance
  for _=1,segs+2 do
    local a=state.nodes[seg]; local b=segEnd(seg)
    local length=flatDist(a,b)
    if length>=0.0001 then
      local room=length*(1-t)
      if room>=remaining then
        local nt=(length*t+remaining)/length
        return vec3(lerp(a.x,b.x,nt),lerp(a.y,b.y,nt),lerp(a.z,b.z,nt)),seg,nt
      end
      remaining=remaining-room; t=0
    end
    if state.loopRoute then seg=(seg%segs)+1
    elseif seg<segs then seg=seg+1
    else return nil end
  end
  return nil
end
local function getVertexCurvature(index)
  if ensureTrackMap then ensureTrackMap() end
  local entry=state.trackMap and state.trackMap[index]
  return entry and entry.curvature or 0
end
local function findNearestSeg(pos,hintSeg,allowGlobalScan)
  local nodes=state.nodes; local segs=segmentCount()
  if segs<1 then return 1,0,0 end
  local px,pz=pos.x,pos.z
  local base=clamp(hintSeg or 1,1,segs)
  local bestSeg,bestT=base,0
  local bestDist=math.huge
  local function check(index)
    local a=nodes[index]; local b=segEnd(index)
    local vx,vz=b.x-a.x,b.z-a.z
    local lenSq=vx*vx+vz*vz
    if lenSq<0.00001 then return end
    local t=clamp(((px-a.x)*vx+(pz-a.z)*vz)/lenSq,0,1)
    local cx,cz=a.x+vx*t,a.z+vz*t
    local dx,dz=px-cx,pz-cz
    local dSq=dx*dx+dz*dz
    if dSq<bestDist then bestDist=dSq; bestSeg=index; bestT=t end
  end
  -- Keep the projection local. A 25% route window can jump to a nearby
  -- parallel section on hairpins and make the AI cut across the circuit.
  local back=math.min(4,segs-1)
  local ahead=math.min(segs-1,math.max(16,math.min(28,math.floor(segs*0.08))))
  for offset=-back,ahead do
    local idx
    if state.loopRoute then
      idx=((base-1+offset)%segs)+1
    else
      idx=base+offset
      if idx>=1 and idx<=segs then check(idx) end
      idx=nil
    end
    if idx then check(idx) end
  end
  if allowGlobalScan ~= false and bestDist>15*15 then
    for i=1,segs do check(i) end
  end
  return bestSeg,bestT,math.sqrt(bestDist)
end
local function lateralOf(pos,seg)
  local a=state.nodes[seg]; local b=segEnd(seg)
  local lx,lz=normalize2(b.x-a.x,b.z-a.z)
  return (pos.x-a.x)*-lz+(pos.z-a.z)*lx
end
local function boundsAt(index)
  local n=#state.nodes
  if n<1 then return -1.5,1.5 end
  local i=index
  if state.loopRoute then i=((i-1)%n)+1
  elseif i<1 or i>n then i=clamp(i,1,n) end
  return safeNumber(state.boundsL[i],-1.5),safeNumber(state.boundsR[i],1.5)
end
ensureTrackMap=function()
  local n=#state.nodes
  local expected=segmentCount()
  if expected<1 then state.trackMap=nil; return end
  if state.trackMap and state.trackMap.count==expected and
      state.trackMap.loop==state.loopRoute and state.trackMap.nodeCount==n then return end
  local map={count=expected,loop=state.loopRoute,nodeCount=n}
  local half=CFG.VEHICLE_HALF_WIDTH+CFG.BOUNDARY_CLEARANCE
  local function pointAt(index)
    if state.loopRoute then index=((index-1)%n)+1
    else index=clamp(index,1,n) end
    return state.nodes[index]
  end
  local function turnMetric(index)
    local before=pointAt(index-4)
    local pivot=pointAt(index)
    local after=pointAt(index+4)
    local ax,az=pivot.x-before.x,pivot.z-before.z
    local bx,bz=after.x-pivot.x,after.z-pivot.z
    local al=math.sqrt(ax*ax+az*az)
    local bl=math.sqrt(bx*bx+bz*bz)
    if al<0.5 or bl<0.5 then return 0 end
    ax,az=ax/al,az/al; bx,bz=bx/bl,bz/bl
    return math.abs(math.atan2(ax*bz-az*bx,clamp(ax*bx+az*bz,-1,1)))
  end
  for i=1,expected do
    local nextIndex=(i%expected)+1
    local prevIndex=i-1
    if state.loopRoute then prevIndex=((prevIndex-1)%n)+1
    else prevIndex=math.max(1,prevIndex) end
    local left1,right1=boundsAt(i)
    local left2,right2=boundsAt(nextIndex)
    local safeL=math.max(left1,left2)+half
    local safeR=math.min(right1,right2)-half
    if safeL>safeR then local mid=(safeL+safeR)*0.5; safeL,safeR=mid,mid end
    local width=safeR-safeL
    local twoLane=width>=CFG.MIN_LANE_CENTER_GAP
    local leftOffset,rightOffset=0,0
    if twoLane then
      leftOffset=safeL+width*0.25
      rightOffset=safeR-width*0.25
      if rightOffset-leftOffset<CFG.MIN_LANE_CENTER_GAP then twoLane=false end
    end
    if not twoLane then leftOffset=clamp(0,safeL,safeR); rightOffset=leftOffset end
    local prev,cur,nxt=state.nodes[prevIndex],state.nodes[i],segEnd(i)
    local turn=turnMetric(i)
    local curved=turn>=0.12
    local hairpin=turn>=0.85
    local ab,bc,ca=flatDist(prev,cur),flatDist(cur,nxt),flatDist(nxt,prev)
    local cross=math.abs((cur.x-prev.x)*(nxt.z-prev.z)-(cur.z-prev.z)*(nxt.x-prev.x))
    local curvature=(ab>0.05 and bc>0.05 and ca>0.05 and cross>0.0001)
      and (2*cross/(ab*bc*ca)) or 0
    map[i]={safeL=safeL,safeR=safeR,twoLane=twoLane,
      leftOffset=leftOffset,rightOffset=rightOffset,curvature=curvature,
      length=flatDist(cur,nxt),turnAngle=turn,curved=curved,hairpin=hairpin,
      speedLimit=curved and (70/3.6) or (160/3.6),
      section=hairpin and "HAIRPIN" or (curved and "CORNER" or "STRAIGHT")}
  end
  local runs={}
  local i=1
  while i<=expected do
    if not map[i].curved then
      local first=i; local length=0
      while i<=expected and not map[i].curved do
        length=length+map[i].length; i=i+1
      end
      runs[#runs+1]={first=first,last=i-1,length=length}
    else i=i+1 end
  end
  if state.loopRoute and #runs>=2 and runs[1].first==1 and runs[#runs].last==expected then
    local fr,lr=runs[1],runs[#runs]
    local limit=(fr.length+lr.length>=250.0) and (320/3.6) or (160/3.6)
    for k=fr.first,fr.last do map[k].speedLimit=limit end
    for k=lr.first,lr.last do map[k].speedLimit=limit end
    fr.merged=true; lr.merged=true
  end
  for r=1,#runs do
    local run=runs[r]
    if not run.merged then
      local limit=(run.length>=250.0) and (320/3.6) or (160/3.6)
      for k=run.first,run.last do map[k].speedLimit=limit end
    end
  end
  state.trackMap=map
  local two,single,corners,hairpins,shorts,longs=0,0,0,0,0,0
  for j=1,expected do
    local e=map[j]
    if e.twoLane then two=two+1 else single=single+1 end
    if e.curved then corners=corners+1; if e.hairpin then hairpins=hairpins+1 end
    elseif e.speedLimit>(200/3.6) then longs=longs+1 else shorts=shorts+1 end
  end
  writeLog("TRACK_MAP_BUILT segments=%d two_lane=%d single_lane=%d corners=%d hairpins=%d short_straight=%d long_straight=%d speed_limits_kmh=70/160/320",
    expected,two,single,corners,hairpins,shorts,longs)
end
local function safeBoundsAt(index)
  ensureTrackMap()
  local e=state.trackMap and state.trackMap[index]
  if e then return e.safeL,e.safeR end
  return -1.5,1.5
end
local function speedLimitAt(index)
  ensureTrackMap()
  local e=state.trackMap and state.trackMap[index]
  return e and e.speedLimit or (160/3.6)
end
local function rebuildTrackMap()
  state.trackMap=nil
  if #state.nodes>=2 then
    ensureTrackMap()
    state.loaded=true
    local count=state.trackMap and state.trackMap.count or 0
    state.status=string.format("Карта рассчитана: %d участков",count)
    writeLog("TRACK_MAP_RECALCULATED segments=%d",count)
    flushLog()
    return count>0
  end
  state.status="Сначала запишите или загрузите линию"
  return false
end
local function offsetSafeAhead(startSeg,startT,offset,distance)
  local remaining=math.max(0,distance or 0)
  local seg=startSeg
  local t=startT
  local checked=0
  while remaining>=0 and checked<600 do
    checked=checked+1
    local l,r=safeBoundsAt(seg)
    if offset<l-0.001 or offset>r+0.001 then return false,seg,l,r end
    local a=state.nodes[seg]
    local b=segEnd(seg)
    local len=flatDist(a,b)
    if len<0.001 then return false,seg,l,r end
    local available=len*(1-t)
    if remaining<=available then break end
    remaining=remaining-available
    if state.loopRoute then seg=(seg%segmentCount())+1
    elseif seg<segmentCount() then seg=seg+1
    else break end
    t=0
  end
  return true
end
local function laneOffsetForCar(carIndex,seg)
  ensureTrackMap()
  local entry=state.trackMap and state.trackMap[seg]
  if not entry then return 0,false end
  if not entry.twoLane then return entry.leftOffset,false end
  local side=state.laneAssignments and state.laneAssignments[carIndex]
  if side==nil then side=(carIndex%2==1) and 1 or -1 end
  return (side>0) and entry.leftOffset or entry.rightOffset,true
end
local function assignLaneRoster(sim)
  local leftCount,rightCount=0,0
  for i=1,sim.carsCount-1 do
    local car=ac.getCar(i)
    if car and car.isActive and car.isAIControlled then
      local side=state.laneAssignments[i]
      if side==1 then leftCount=leftCount+1
      elseif side==-1 then rightCount=rightCount+1 end
    end
  end
  for i=1,sim.carsCount-1 do
    local car=ac.getCar(i)
    if car and car.isActive and car.isAIControlled and not state.laneAssignments[i] then
      local side=(leftCount<=rightCount) and 1 or -1
      state.laneAssignments[i]=side
      if side==1 then leftCount=leftCount+1 else rightCount=rightCount+1 end
      writeLog("LANE_ASSIGN id=%d side=%s",i,side==1 and "LEFT" or "RIGHT")
    end
  end
end
local function recordSecondTelemetry(sim)
  local any=false
  for i=0,sim.carsCount-1 do
    local car=ac.getCar(i)
    local p=state.p[i]
    if car and car.isActive and car.isAIControlled and p then
      if Driver and Driver.telemetryLine then writeLog(Driver.telemetryLine(i,car,p)) end
      any=true
    end
  end
  if not any and logger and logger.heartbeat then
    pcall(logger.heartbeat,logger,"nodes="..tostring(#state.nodes).." ai=0")
  end
  flushLog()
end
local function buildSpawnRouteMetrics()
  local count=#state.nodes
  if count<2 then return nil end
  local segs=segmentCount()
  local segments={}
  local totalLength=0
  for i=1,segs do
    local a=state.nodes[i]; local b=segEnd(i)
    local length=flatDist(a,b)
    if length>0.05 then
      segments[#segments+1]={a=a,b=b,start=totalLength,length=length}
      totalLength=totalLength+length
    end
  end
  if #segments<2 or totalLength<CFG.SPAWN_MIN_GAP*2 then return nil end
  return {segments=segments,length=totalLength,loop=state.loopRoute}
end
local function findRouteDistanceAtPosition(position,metrics)
  local bestDistSq,bestRoute=math.huge,0
  for i=1,#metrics.segments do
    local seg=metrics.segments[i]
    local a,b=seg.a,seg.b
    local vx,vz=b.x-a.x,b.z-a.z
    local lenSq=vx*vx+vz*vz
    if lenSq>0.0001 then
      local t=clamp(((position.x-a.x)*vx+(position.z-a.z)*vz)/lenSq,0,1)
      local px,pz=a.x+vx*t,a.z+vz*t
      local dx,dz=position.x-px,position.z-pz
      local dSq=dx*dx+dz*dz
      if dSq<bestDistSq then bestDistSq=dSq; bestRoute=seg.start+seg.length*t end
    end
  end
  return bestRoute,math.sqrt(bestDistSq)
end
local function sampleSpawnRoute(metrics,distance)
  if metrics.loop then distance=distance%metrics.length
  else distance=clamp(distance,0,metrics.length) end
  for i=1,#metrics.segments do
    local seg=metrics.segments[i]
    if distance<=seg.start+seg.length or i==#metrics.segments then
      local t=clamp((distance-seg.start)/seg.length,0,1)
      local a,b=seg.a,seg.b
      local position=vec3(lerp(a.x,b.x,t),lerp(a.y,b.y,t),lerp(a.z,b.z,t))
      local direction=vec3(b.x-a.x,b.y-a.y,b.z-a.z)
      if direction:length()<0.0001 then return nil,nil end
      direction:normalize()
      return position,direction
    end
  end
  return nil,nil
end
local function spotIsClear(pos, exceptIndex)
  local sim=ac.getSim()
  if not sim then return true end
  for i=0,sim.carsCount-1 do
    if i~=exceptIndex then
      local other=ac.getCar(i)
      if other and other.isActive then
        local dx=other.position.x-pos.x
        local dz=other.position.z-pos.z
        if dx*dx+dz*dz < CFG.SAFE_RESPAWN_CLEAR*CFG.SAFE_RESPAWN_CLEAR then
          return false
        end
      end
    end
  end
  return true
end
-- Align the recorded point order with the player's actual forward direction.
-- This also keeps the signed left/right lane axis consistent with travel.
local function alignRouteDirectionToPlayer(metrics,player)
  if state.routeDirectionChecked or not player or not player.look then
    return metrics
  end
  local routeDistance,routeError=findRouteDistanceAtPosition(player.position,metrics)
  if routeError>15.0 then
    writeLog("ROUTE_DIRECTION_AMBIGUOUS route_error=%.2f",routeError)
    state.routeDirectionChecked=true
    return metrics
  end
  local _,routeDirection=sampleSpawnRoute(metrics,routeDistance)
  if not routeDirection then
    writeLog("ROUTE_DIRECTION_CHECK_FAILED")
    state.routeDirectionChecked=true
    return metrics
  end
  local dot=player.look.x*routeDirection.x+player.look.z*routeDirection.z
  if dot < -0.15 then
    local oldL,oldR=state.boundsL,state.boundsR
    local n=#state.nodes
    local reversedNodes,reversedL,reversedR={},{},{}
    for i=1,n do
      local source=n-i+1
      reversedNodes[i]=state.nodes[source]
      -- Reversing travel direction also reverses the signed left/right axis.
      reversedL[i]=-safeNumber(oldR[source],1.5)
      reversedR[i]=-safeNumber(oldL[source],-1.5)
    end
    state.nodes=reversedNodes
    state.boundsL,state.boundsR=reversedL,reversedR
    state.p={}
    metrics=buildSpawnRouteMetrics()
    writeLog("ROUTE_DIRECTION_REVERSED dot=%.3f player_route_error=%.2f",dot,routeError)
  else
    writeLog("ROUTE_DIRECTION_OK dot=%.3f player_route_error=%.2f",dot,routeError)
  end
  state.routeDirectionChecked=true
  return metrics
end

-- Spawn teleport runs when the roster is stable; it NEVER blocks driving.
local function trySpawnTeleport(sim,dt)
  if state.spawnDone then return end
  if not state.loaded or state.recLine or #state.nodes<2 then return end
  local aiCars={}
  for i=1,sim.carsCount-1 do
    local car=ac.getCar(i)
    if car and car.isActive and car.isAIControlled then aiCars[#aiCars+1]=i end
  end
  local roster=table.concat(aiCars,",")
  if #aiCars==0 then
    -- Keep waiting: AI cars can be added after the app has already started.
    return
  end
  if roster~=state.spawnLastRoster then
    state.spawnLastRoster=roster
    state.spawnStable=0
    return
  end
  state.spawnStable=state.spawnStable+dt
  if state.spawnStable<CFG.SPAWN_STABLE_TIME then return end
  -- teleport (in parallel with driving; adopted cars keep their personality)
  if type(physics)~="table" or type(physics.setAICarPosition)~="function" then
    state.spawnDone=true; return
  end
  local metrics=buildSpawnRouteMetrics()
  if not metrics then state.spawnDone=true; return end
  local player=ac.getCar(0)
  if player then metrics=alignRouteDirectionToPlayer(metrics,player) end
  if not metrics then state.spawnDone=true; return end
  local count=#aiCars
  local gap=math.min(CFG.SPAWN_GAP,metrics.length/(count+1))
  if gap<CFG.SPAWN_MIN_GAP then state.spawnDone=true; return end
  local startDistance=0
  if metrics.loop then
    if player then
      startDistance=findRouteDistanceAtPosition(player.position,metrics)+gap
    else startDistance=gap end
  end
  local moved,failed=0,0
  for order=1,count do
    local carIndex=aiCars[order]
    local position,direction=sampleSpawnRoute(metrics,startDistance+(order-1)*gap)
    if position and direction then
      local laneSeg=select(1,findNearestSeg(position,1))
      local laneOffset,twoLane=laneOffsetForCar(carIndex,laneSeg)
      local lanePosition=vec3(position.x-direction.z*laneOffset,position.y,
        position.z+direction.x*laneOffset)
      -- In the live CSP session, passing the route tangent directly made
      -- cars face against travel and perform a 180-degree correction.
      -- The spawn API's observed orientation is opposite to this route basis.
      local spawnDirection=vec3(-direction.x,-direction.y,-direction.z)
      local ok,result=pcall(physics.setAICarPosition,carIndex,
        lanePosition,spawnDirection)
      if ok and result~=false then
        moved=moved+1
        writeLog("SPAWN_LANE id=%d lane=%s offset=%.2f route_dir=(%.3f,%.3f) spawn_dir=(%.3f,%.3f)",carIndex,
          twoLane and (laneOffset<0 and "LEFT" or "RIGHT") or "SINGLE",
          laneOffset,direction.x,direction.z,spawnDirection.x,spawnDirection.z)
        state.p[carIndex]=nil
      else failed=failed+1 end
    else failed=failed+1 end
  end
  state.spawnDone=true
  writeLog("SPAWN_FINISH moved=%d failed=%d gap=%.2f",moved,failed,gap)
  flushLog()
end
local function teleportHome(carIndex,car)
  if type(physics)~="table" or type(physics.setAICarPosition)~="function" then
    return false
  end
  local n=#state.nodes
  if n<2 then return false end
  local bestI,bestD=1,math.huge
  for i=1,n do
    local d=flatDist(car.position,state.nodes[i])
    if d<bestD then bestD,bestI=d,i end
  end
  for k=0,n-1 do
    local i=((bestI-1+k)%n)+1
    local node=state.nodes[i]
    local nxt=state.nodes[(i%n)+1]
    local dx,dz=normalize2(nxt.x-node.x,nxt.z-node.z)
    local laneOffset=laneOffsetForCar(carIndex,i)
    local pos=vec3(node.x-dz*laneOffset-dx*4,node.y+0.1,
      node.z+dx*laneOffset-dz*4)
    if spotIsClear(pos,carIndex) then
      local ok=pcall(physics.setAICarPosition,carIndex,pos,vec3(-dx,0,-dz))
      if ok then
        state.p[carIndex]=nil
        writeLog("RESCUE id=%d node=%d offset=%d lane=%.2f",carIndex,i,k,laneOffset)
        flushLog()
      end
      return ok
    end
  end
  return false
end
local function addPoint(pos)
  if #state.nodes>=CFG.MAX_ROUTE_POINTS then
    state.recLine=false; state.status="Достигнут лимит точек"; return
  end
  if #state.nodes==0 or flatDist(state.nodes[#state.nodes],pos)>=CFG.RECORD_DISTANCE then
    state.nodes[#state.nodes+1]=vec3(pos.x,pos.y+0.1,pos.z)
    local half=WIDTH_PRESETS[state.widthPreset]/2
    state.boundsL[#state.nodes]=-half
    state.boundsR[#state.nodes]=half
  end
end
local function startBoundsRecording(player)
  if #state.nodes<2 then
    state.status="Сначала запишите линию ИИ"; return
  end
  local seg=select(1,findNearestSeg(player.position,1))
  local off=lateralOf(player.position,seg)
  local l,r=boundsAt(seg)
  if off<l or off>r then
    state.status="Машина вне границ — встаньте внутри границ трассы"
    writeLog("BOUNDS_REC_DENIED seg=%d off=%.1f bounds=[%.1f,%.1f]",seg,off,l,r)
    flushLog(); return
  end
  state.recBounds=true; state.recBoundsSeg=seg
  state.status="Запись границ... ведите машину вдоль участка"
  writeLog("BOUNDS_REC_START seg=%d off=%.1f",seg,off)
  flushLog()
end
local function updateBoundsRecording(player)
  local n=#state.nodes
  local seg=select(1,findNearestSeg(player.position,state.recBoundsSeg or 1))
  local off=lateralOf(player.position,seg)
  local l,r=boundsAt(seg)
  local newL,newR=l,r
  if off>=0 then
    newR=math.max(off+0.5,1.5)
    newL=math.min(l,newR-CFG.BOUNDS_MIN_PASS)
    if newL>-1.5 then newL=-1.5 end
  else
    newL=math.min(off-0.5,-1.5)
    newR=math.max(r,newL+CFG.BOUNDS_MIN_PASS)
    if newR<1.5 then newR=1.5 end
  end
  local idx=seg
  if state.loopRoute then idx=((seg-1)%n)+1 end
  if idx>=1 and idx<=n then
    state.boundsL[idx]=newL; state.boundsR[idx]=newR
  end
  if state.recBoundsSeg and seg~=state.recBoundsSeg then
    state.boundsL[state.recBoundsSeg]=newL
    state.boundsR[state.recBoundsSeg]=newR
    state.recBoundsSeg=seg
  end
end
local function applyWidthPreset(preset)
  state.widthPreset=preset
  state.trackMap=nil
  local n=#state.nodes
  if n<2 then return end
  local half=WIDTH_PRESETS[preset]/2
  for i=1,n do
    state.boundsL[i]=-half
    state.boundsR[i]=half
  end
  if n>=2 then ensureTrackMap() end
  writeLog("WIDTH_PRESET preset=%d width=%dm",preset,WIDTH_PRESETS[preset])
end
local function releaseCar(carIndex)
  local ok,controls=pcall(ac.overrideCarControls,carIndex)
  if ok and controls then
    controls.combineAxis=false
    controls.steer=0; controls.gas=0; controls.brake=0; controls.handbrake=0
  end
  state.p[carIndex]=nil
end
local driverReady=false
local function ensureDriver()
  if driverReady or not Driver then return Driver~=nil end
  if type(Driver.setup)~="function" then return false end
  Driver.setup(CFG,state,{
    segmentCount=segmentCount, segEnd=segEnd, walkLine=walkLine,
    findNearestSeg=findNearestSeg, lateralOf=lateralOf, boundsAt=boundsAt,
    safeBoundsAt=safeBoundsAt, offsetSafeAhead=offsetSafeAhead,
    laneOffset=laneOffsetForCar,
    getVertexCurvature=getVertexCurvature,
    getSpeedLimit=speedLimitAt,
    teleportHome=teleportHome, log=writeLog,
  })
  driverReady=true
  return true
end
function script.update(dt)
  if not dt or dt<=0 then return end
  dt=math.min(dt,0.05)
  local ok,err=pcall(function()
    local sim=ac.getSim()
    if not sim then return end
    assignLaneRoster(sim)
    local player=ac.getCar(0)
    if state.recLine and player then addPoint(player.position) end
    if state.recBounds and player then updateBoundsRecording(player) end
    if state.enabled and state.loaded and #state.nodes>=2 then
      trySpawnTeleport(sim,dt)
    end
    -- INSTANT ADOPTION + DRIVING: every active AI car is driven from the
    -- very first frame. Nothing blocks this. (This was the bug: the spawn
    -- gate used to block driving forever — cars stayed on the stock AI.)
    if state.enabled and #state.nodes>=2 and Driver and ensureDriver() then
      for i=0,sim.carsCount-1 do
        local car=ac.getCar(i)
        if car and car.isActive and car.isAIControlled then
          local okc,erc=pcall(Driver.update,i,car,dt)
          if not okc then
            writeLog("DRIVER_ERROR id=%d err=%s",i,tostring(erc))
            releaseCar(i)
          end
        elseif state.p[i] then
          releaseCar(i)
        end
      end
    else
      for idx in pairs(state.p) do releaseCar(idx) end
    end
    state.logAccum=state.logAccum+dt
    if state.logAccum>=CFG.LOG_INTERVAL then
      state.logAccum=state.logAccum-CFG.LOG_INTERVAL
      recordSecondTelemetry(sim)
    end
  end)
  if not ok then
    if logger then writeLog("UPDATE_ERROR err=%s",tostring(err)); flushLog() end
  end
end

-- 3D WORLD LINES.
-- Recording: only existing nodes within 8 m back .. 30 m ahead of the player
-- (data is written only under the car; the loop segment is never shown far).
-- Finished track ("Показать линии"): the whole route at long distance.
render.on('main.track.transparent', function()
  local n=#state.nodes
  if n<2 then return end
  local isRec=state.recLine or state.recBounds
  if not (state.showWorldLines or isRec) then return end
  local ok=pcall(function()
    local player=ac.getCar(0)
    if not player then return end
    render.setDepthMode(render.DepthMode.Off)
    local YELLOW=rgbm(1.0,0.85,0.0,1)
    local RED=rgbm(0.6,0.05,0.05,1)
    local seg=select(1,findNearestSeg(player.position,1))
    local fromI,toI=1,n
    if isRec then
      -- forward window up to PREVIEW_AHEAD meters
      fromI=seg; toI=seg
      local dist=0
      local i=seg
      while dist<CFG.PREVIEW_AHEAD do
        local a=state.nodes[i]
        local b=state.nodes[(i%n)+1]
        dist=dist+flatDist(a,b)
        i=(i%n)+1
        toI=i
        if i==seg then break end
      end
      -- backward window up to PREVIEW_BACK meters
      local j=seg
      local back=0
      while back<CFG.PREVIEW_BACK do
        local prev=(j-2)%n+1
        back=back+flatDist(state.nodes[prev],state.nodes[j])
        j=prev
        fromI=prev
      end
    end
    -- draw all segments in [fromI..toI] (loop-aware); finished track: all
    local count=0
    local maxSteps=n
    local i2=fromI
    while count<=maxSteps do
      count=count+1
      local node=state.nodes[i2]
      local nx=state.nodes[(i2%n)+1]
      local lx,lz=normalize2(nx.x-node.x,nx.z-node.z)
      local al,ar=boundsAt(i2)
      -- distance cull only for the finished track (long range view)
      if isRec or true then
        render.debugLine(vec3(node.x,node.y+0.05,node.z),vec3(nx.x,nx.y+0.05,nx.z),YELLOW)
        render.debugLine(vec3(node.x-lz*al,node.y+0.05,node.z+lx*al),
          vec3(nx.x-lz*al,nx.y+0.05,nx.z+lx*al),RED)
        render.debugLine(vec3(node.x-lz*ar,node.y+0.05,node.z+lx*ar),
          vec3(nx.x-lz*ar,nx.y+0.05,nx.z+lx*ar),RED)
      end
      if i2==toI then break end
      i2=(i2%n)+1
    end
  end)
  if not ok then
    writeLog("DRAW3D_ERROR"); flushLog()
  end
end)

-- MAP: window-local coords (comfy_map pattern). Bigger scale, thick lines,
-- bounds with a minimum offset so they never merge with the AI line.
local function computeMapBounds()
  local minX,maxX,minZ,maxZ=math.huge,-math.huge,math.huge,-math.huge
  for i=1,#state.nodes do
    local nd=state.nodes[i]
    local l,r=boundsAt(i)
    local b=segEnd(i)
    local lx,lz=normalize2(b.x-nd.x,b.z-nd.z)
    local pts={
      {x=nd.x-lz*l,z=nd.z+lx*l},
      {x=nd.x-lz*r,z=nd.z+lx*r},
    }
    for _,q in ipairs(pts) do
      if q.x<minX then minX=q.x end
      if q.x>maxX then maxX=q.x end
      if q.z<minZ then minZ=q.z end
      if q.z>maxZ then maxZ=q.z end
    end
  end
  if minX==math.huge then return nil end
  return {minX=minX,maxX=maxX,minZ=minZ,maxZ=maxZ}
end
local function drawMapInRect(q0,q1)
  local b=computeMapBounds()
  if not b then return end
  ui.drawRectFilled(q0,q1,rgbm(0.08,0.08,0.10,1),4)
  ui.drawRect(q0,q1,rgbm(0.3,0.3,0.35,1),4)
  local spanX=math.max(1,b.maxX-b.minX)+CFG.MAP_PAD*2
  local spanZ=math.max(1,b.maxZ-b.minZ)+CFG.MAP_PAD*2
  local scale=math.min((q1.x-q0.x)/spanX,(q1.y-q0.y)/spanZ)
  local cx=(b.minX+b.maxX)*0.5
  local cz=(b.minZ+b.maxZ)*0.5
  local mx=(q0.x+q1.x)*0.5
  local my=(q0.y+q1.y)*0.5
  local function segVisible(x1,y1,x2,y2)
    return not (x1<q0.x and x2<q0.x) and not (x1>q1.x and x2>q1.x)
      and not (y1<q0.y and y2<q0.y) and not (y1>q1.y and y2>q1.y)
  end
  local n=#state.nodes
  local minOff=CFG.MAP_MIN_OFFSET
  -- bounds: dark red, thick, min offset from the line
  for pass=1,2 do
    local prev
    for i=1,n do
      local node=state.nodes[i]
      local nxt=state.nodes[(i%n)+1]
      local lx,lz=normalize2(nxt.x-node.x,nxt.z-node.z)
      local l,r=boundsAt(i)
      local off=(pass==1) and l or r
      if math.abs(off)*scale<minOff then
        off=(off>=0 and 1 or -1)*(minOff/math.max(scale,0.0001))
      end
      local px,pz=node.x-lz*off,node.z+lx*off
      local sx=mx+(px-cx)*scale
      local sy=my+(pz-cz)*scale
      if prev and segVisible(prev.x,prev.y,sx,sy) then
        ui.drawLine(prev,vec2(sx,sy),rgbm(0.55,0.05,0.05,1),3)
      end
      prev=vec2(sx,sy)
    end
  end
  -- AI line: yellow, thick
  do
    local prev=vec2(mx+(state.nodes[1].x-cx)*scale,my+(state.nodes[1].z-cz)*scale)
    for i=2,n do
      local sx=mx+(state.nodes[i].x-cx)*scale
      local sy=my+(state.nodes[i].z-cz)*scale
      if segVisible(prev.x,prev.y,sx,sy) then
        ui.drawLine(prev,vec2(sx,sy),rgbm(1.0,0.85,0.0,0.95),3)
      end
      prev=vec2(sx,sy)
    end
    if state.loopRoute then
      local first=vec2(mx+(state.nodes[1].x-cx)*scale,my+(state.nodes[1].z-cz)*scale)
      if segVisible(prev.x,prev.y,first.x,first.y) then
        ui.drawLine(prev,first,rgbm(1.0,0.85,0.0,0.95),3)
      end
    end
  end
  -- cars: player = cyan, AI = white/green when overtaking
  local sim=ac.getSim()
  if sim then
    for i=0,sim.carsCount-1 do
      local car=ac.getCar(i)
      if car and car.isActive then
        local s=vec2(mx+(car.position.x-cx)*scale,my+(car.position.z-cz)*scale)
        if s.x>q0.x and s.x<q1.x and s.y>q0.y and s.y<q1.y then
          if i==0 then
            ui.drawCircleFilled(s,5,rgbm(0.0,0.8,1.0,1.0),12)
          else
            local p=state.p[i]
            if p and (p.laneRole=="LEFT" or p.laneRole=="LEFT_FAST") then
              ui.drawCircleFilled(s,4,rgbm(0.2,1.0,0.4,1.0),10)
            else
              ui.drawCircleFilled(s,4,rgbm(1,1,1,0.95),10)
            end
          end
        end
      end
    end
  end
end
function windowMain(dt)
  local okDraw,errDraw=pcall(function()
    ui.text(string.format("Точек: %d | %s",#state.nodes,state.status))
    for idx=1,4 do
      local label=string.format("%d %s",idx,idx==1 and "машина" or "машины")
      if idx>1 then ui.sameLine(0,4) end
      if ui.button(label,vec2(92,22)) then
        applyWidthPreset(idx)
      end
    end
    if ui.button(state.recLine and "СТОП запись линии" or "Запись линии",vec2(-0.5,24)) then
      state.recLine=not state.recLine
      if state.recLine then
        state.enabled=false
        for idx in pairs(state.p) do releaseCar(idx) end
        state.p={}
        state.nodes={}; state.p={}; state.trackMap=nil; state.boundsL={}; state.boundsR={}
        state.loaded=false; state.recBounds=false
        state.routeDirectionChecked=false
        state.status="Запись линии и границ..."
      else
        state.loaded=#state.nodes>=2
        state.trackMap=nil
        if state.loaded then ensureTrackMap() end
        state.status="Записано точек: "..#state.nodes
      end
    end
    ui.sameLine(0,4)
    if ui.button(state.recBounds and "СТОП запись границ" or "Запись границ",vec2(-0.5,24)) then
      if state.recBounds then
        state.recBounds=false
        saveBounds()
        state.trackMap=nil
        ensureTrackMap()
        state.status="Границы записаны"
      else
        local player=ac.getCar(0)
        if player then
          state.recLine=false
          startBoundsRecording(player)
        end
      end
    end
    if ui.button(state.showWorldLines and "Скрыть линии на трассе" or "Показать линии",vec2(-0.5,22)) then
      state.showWorldLines=not state.showWorldLines
      writeLog("WORLD_LINES %s",state.showWorldLines and "ON" or "OFF")
    end
    ui.sameLine(0,4)
    if ui.button(state.loopRoute and "Круг: ВКЛ" or "Круг: ВЫКЛ",vec2(-0.5,22)) then
      state.loopRoute=not state.loopRoute
      state.p={}
      state.trackMap=nil
      if #state.nodes>=2 then ensureTrackMap() end
    end
    if ui.button("Рассчитать участки",vec2(150,22)) then rebuildTrackMap() end
    ui.sameLine(0,4)
    if ui.button("Сохранить",vec2(82,22)) then saveLine() end
    ui.sameLine(0,4)
    if ui.button("Загрузить",vec2(82,22)) then loadLine() end
    if ui.button(state.enabled and "Остановить ИИ" or "Запустить ИИ",vec2(150,22)) then
      if state.enabled then
        state.enabled=false
        for idx in pairs(state.p) do releaseCar(idx) end
        state.p={}
        state.status="Управление ИИ остановлено"
        writeLog("AI_CONTROL_STOP")
      elseif #state.nodes>=2 then
        rebuildTrackMap()
        state.enabled=true
        state.spawnDone=false
        state.spawnStable=0
        state.spawnLastRoster=""
        state.p={}
        state.status="ИИ запущен"
        writeLog("AI_CONTROL_START")
      else
        state.status="Нет записанной линии"
      end
      flushLog()
    end
    ui.sameLine(0,4)
    if ui.button("Очистить",vec2(82,22)) then
      state.nodes={}; state.p={}; state.trackMap=nil; state.boundsL={}; state.boundsR={}
      state.status="Очищено"; state.loaded=false
    end
    ui.separator()
    if #state.nodes>=2 then
      local ws=ui.windowSize()
      if ws.y>CFG.MAP_TOP+60 then
        if ui.pushClipRect and ui.popClipRect then
          ui.pushClipRect(vec2(0,0),ws)
        end
        drawMapInRect(vec2(0,CFG.MAP_TOP),vec2(ws.x,ws.y))
        if ui.pushClipRect and ui.popClipRect then
          ui.popClipRect()
        end
      end
    else
      ui.text("Нет линии — нажмите «Запись линии»")
    end
  end)
  if not okDraw then
    if logger then
      writeLog("UI_ERROR err=%s",tostring(errDraw))
      flushLog()
      pcall(ui.text,"UI error (в логе)")
    else
      pcall(ui.text,"UI error")
    end
  end
end
if ac.onRelease then
  ac.onRelease(function()
    for idx in pairs(state.p) do releaseCar(idx) end
    if logger and logger.shutdown then
      pcall(logger.shutdown,logger,"RoutePilot released")
    end
  end)
end
loadLine()
loadBounds()
writeLog("ROUTEPILOT 0.10.0 INITIALIZED track=%s points=%d",getTrackTag(),#state.nodes)
flushLog()