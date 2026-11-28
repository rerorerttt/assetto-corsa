-- RoutePilot.lua — v0.9.1 (CSP 0.3.2-preview2)
-- THE TRACE SAID IT ALL: 184 s of session, ZERO CAR_INIT/CAR= lines — the
-- cars were never adopted (the spawn gate blocked driving forever) and drove
-- on the STOCK game AI: that is where the zigzags, burnouts, wall-hunting
-- and ignoring the player came from. Fixes:
--  * INSTANT ADOPTION: every active AI car is adopted from the very first
--    update, unconditionally. Spawn teleport runs in parallel and never
--    blocks driving. Telemetry (CAR= lines) returns to the trace.
--  * UI errors are LOGGED now (were silently swallowed) — any windowMain
--    failure lands in routepilot_trace.txt as UI_ERROR.
--  * Recording render: line/bounds shown 30 m AHEAD of the player only
--    (data still written only under the car); loop closing segment is
--    never drawn far away. Finished track ("Показать линии"): the whole
--    route renders at long distance (600 m from camera).
--  * Map: bigger scale (padding 16 -> 6), thicker lines (3 px), bounds
--    drawn with a minimum 10 px offset from the AI line so they are
--    clearly separate. Every AI car is also drawn as a labeled dot.
--  * Bounds re-record button "Запись границ" works from anywhere (denied
--    only if the player starts outside the current bounds), smooth joints.

local okLogger, Logger = pcall(require, "logger")
if not okLogger or type(Logger) ~= "table" then Logger = nil end
local okDriver, Driver = pcall(require, "driver")
if not okDriver or type(Driver) ~= "table" or not Driver.update then Driver = nil end
local okTrafficCSP, TrafficCSP = pcall(require, "traffic_csp_import")
if not okTrafficCSP or type(TrafficCSP) ~= "table" or type(TrafficCSP.new) ~= "function" then TrafficCSP = nil end
local okCSPGraph, CSPGraph = pcall(require, "traffic_csp_graph")
if not okCSPGraph or type(CSPGraph) ~= "table" or type(CSPGraph.nearestEdge) ~= "function" or
    type(CSPGraph.nearestEdges) ~= "function" or type(CSPGraph.loopPath) ~= "function" or
    type(CSPGraph.sampleLane) ~= "function" or type(CSPGraph.directionAt) ~= "function" then CSPGraph = nil end
local okCSPRoutes, CSPRoutes = pcall(require, "traffic_csp_route")
if not okCSPRoutes or type(CSPRoutes) ~= "table" or type(CSPRoutes.build) ~= "function" or
    type(CSPRoutes.sample) ~= "function" or type(CSPRoutes.project) ~= "function" then CSPRoutes = nil end

local CFG = {
  LOOKAHEAD_BASE=10.0, LOOKAHEAD_SPEED=0.55, LOOKAHEAD_MIN=10.0, LOOKAHEAD_MAX=85.0,
  CURVATURE_STEER_GAIN=4.5,
  STEER_LIMIT=1.0, STEER_RATE=4.5, STEER_FILTER=0.08,
  MAX_LATERAL_ACCEL=12.0, BRAKE_ACCEL=8.5, PLAN_HORIZON=95.0,
  CURVE_MIN=0.002, GAS_GAIN=0.45, BRAKE_GAIN=0.60, MAX_GAS=1.0, MAX_BRAKE=1.0,
  SLIP_START=0.09, SLIP_FULL=0.45, SLIP_DAMP_MIN_SPEED=8.0,
  SLIP_DAMP_MIN_ANGLE=4.0*math.pi/180,
  TC_SLIP_START=0.10, TC_SLIP_FULL=0.28, TC_MAX_CUT=0.88,
  TC_RECOVERY_RATE=1.25, TC_CORNER_CUT=0.22,
  SPAWN_GAP=25.0, SPAWN_MIN_GAP=14.0, SPAWN_STABLE_TIME=0.35, SPAWN_WAIT_TIMEOUT=8.0,
  LOG_INTERVAL=1.0, MAX_ROUTE_POINTS=50000, RECORD_DISTANCE=3.0,
  PREVIEW_AHEAD=30.0, PREVIEW_BACK=8.0, DRAW_RANGE=600.0,
  BOUNDS_MIN_PASS=3.0, SAFE_RESPAWN_CLEAR=8.0,
  MAP_TOP=175.0, MAP_PAD=6.0, MAP_MIN_OFFSET=10.0,
}
local WIDTH_PRESETS = {3, 7, 11, 15}

local state = {
  enabled=true, recLine=false, recBounds=false, loopRoute=true,
  followGap=8.0, widthPreset=1,
  boundsL={}, boundsR={}, recBoundsSeg=0, showWorldLines=false,
  spawnDone=false, spawnPerformed=false, spawnWait=0, spawnStable=0, spawnLastRoster="",
  spawnHints={}, nodes={}, p={}, status="Готов", logAccum=0, loaded=false,
  playerAutopilotLast=false, steeringDiagnostics=false, showCSPRoads=false,
}
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
local trafficCSP = TrafficCSP and TrafficCSP.new(writeLog) or nil
local trafficCSPAttempted = false
local trafficCSPRouteAttempted = false
local trafficCSPRouteRetry = 0
local trafficCSPRouteFailures = 0
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
    state.nodes=loaded; state.p={}; state.spawnHints={}; state.routeGeometry=nil; state.loaded=true
    if not state.spawnPerformed then
      state.spawnDone=false; state.spawnWait=0; state.spawnStable=0; state.spawnLastRoster=""
    end
    state.status=string.format("Загружено: %d точек",#loaded)
    writeLog("LOAD_ROUTE points=%d track=%s",#loaded,getTrackTag())
    loadBounds(); flushLog()
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
  local nodes=state.nodes; local count=#nodes
  if count<5 then return 0 end
  local W=4
  local i0,i1,i2=index-W,index,index+W
  if state.loopRoute then
    i0=((i0-1)%count)+1; i1=((i1-1)%count)+1; i2=((i2-1)%count)+1
  else
    if i0<1 or i2>count then return 0 end
  end
  local a,b,c=nodes[i0],nodes[i1],nodes[i2]
  local v1x,v1z=b.x-a.x,b.z-a.z
  local v2x,v2z=c.x-b.x,c.z-b.z
  local l1=math.sqrt(v1x*v1x+v1z*v1z)
  local l2=math.sqrt(v2x*v2x+v2z*v2z)
  if l1<0.5 or l2<0.5 then return 0 end
  local dot=clamp((v1x*v2x+v1z*v2z)/(l1*l2),-1,1)
  local angle=math.acos(dot)
  if angle<0.02 then return 0 end
  return angle/math.max(0.1,(l1+l2)*0.5)
end
local function findNearestSeg(pos,hintSeg,allowGlobal,heading)
  local nodes=state.nodes; local segs=segmentCount()
  if segs<1 then return 1,0,0 end
  local px,pz=pos.x,pos.z
  local base=clamp(hintSeg or 1,1,segs)
  local bestSeg,bestT=base,0
  local bestDist,bestScore=math.huge,math.huge
  local hx,hz=nil,nil
  if heading and type(heading.x)=="number" and type(heading.z)=="number" then
    hx,hz=normalize2(heading.x,heading.z)
  end
  local function check(index)
    local a=nodes[index]; local b=segEnd(index)
    local vx,vz=b.x-a.x,b.z-a.z
    local lenSq=vx*vx+vz*vz
    if lenSq<0.00001 then return end
    local t=clamp(((px-a.x)*vx+(pz-a.z)*vz)/lenSq,0,1)
    local cx,cz=a.x+vx*t,a.z+vz*t
    local dx,dz=px-cx,pz-cz
    local dSq=dx*dx+dz*dz
    local score=dSq
    if hx then
      local tx,tz=normalize2(vx,vz)
      local alignment=math.abs(hx*tx+hz*tz)
      score=score+12*(1-alignment)
    end
    if score<bestScore then
      bestScore=score; bestDist=dSq; bestSeg=index; bestT=t
    end
  end
  local back=math.min(5,segs-1)
  local ahead=math.min(segs-1,math.max(20,math.floor(segs*0.25)))
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
  -- A new car gets one global acquisition, biased toward the segment whose
  -- direction best matches its current heading. Locked cars never jump to a
  -- distant, topologically unrelated branch.
  if allowGlobal == true or (allowGlobal ~= false and bestDist>15*15) then
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
local function carAheadFull(carIndex,car,lateralWidth)
  local fx,fz=normalize2(car.look.x,car.look.z)
  local sim=ac.getSim()
  if not sim then return nil,nil,nil end
  local bestGap,bestSpeed,bestIndex=nil,nil,nil
  for i=0,sim.carsCount-1 do
    if i~=carIndex then
      local other=ac.getCar(i)
      if other and other.isActive then
        local dx=other.position.x-car.position.x
        local dz=other.position.z-car.position.z
        local forward=dx*fx+dz*fz
        if forward>0 and forward<90 then
          local lateral=math.abs(dx*fz-dz*fx)
          if lateral<=(lateralWidth or 5.0) then
            if not bestGap or forward<bestGap then
              bestGap=forward
              bestSpeed=math.max(0,safeNumber(other.speedMs))
              bestIndex=i
            end
          end
        end
      end
    end
  end
  return bestGap,bestSpeed,bestIndex
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

  local player=ac.getCar(0)
  local playerTelemetry=state.playerTelemetry
  if not state.p[0] and #state.nodes>=2 and not state.recLine and
     player and player.isActive and playerTelemetry and Driver and Driver.telemetryLine then
    writeLog(Driver.telemetryLine(0,player,playerTelemetry))
    any=true
  end

  if not any and logger and logger.heartbeat then
    pcall(logger.heartbeat,logger,"nodes="..tostring(#state.nodes).." ai=0 player=0")
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
      segments[#segments+1]={a=a,b=b,start=totalLength,length=length,index=i}
      totalLength=totalLength+length
    end
  end
  if #segments<2 or totalLength<CFG.SPAWN_MIN_GAP*2 then return nil end
  return {segments=segments,length=totalLength,loop=state.loopRoute}
end
local function findRouteDistanceAtPosition(position,metrics,heading)
  local bestScore,bestDistSq,bestRoute=math.huge,math.huge,0
  local hx,hz=nil,nil
  if heading then hx,hz=normalize2(heading.x,heading.z) end
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
      local score=dSq
      if hx then
        local inv=1/math.sqrt(lenSq)
        local alignment=hx*vx*inv+hz*vz*inv
        score=score+100*(1-alignment)*(1-alignment)
      end
      if score<bestScore then
        bestScore=score; bestDistSq=dSq; bestRoute=seg.start+seg.length*t
      end
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
      return position,direction,seg.index,t
    end
  end
  return nil,nil,nil,nil
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
-- Spawn teleport runs when the roster is stable; it NEVER blocks driving.
local function trySpawnTeleport(sim,dt)
  if state.spawnDone then return end
  if not state.loaded or state.recLine or #state.nodes<2 then return end
  state.spawnWait=state.spawnWait+dt
  local aiCars={}
  for i=1,sim.carsCount-1 do
    local car=ac.getCar(i)
    if car and car.isActive and car.isAIControlled then aiCars[#aiCars+1]=i end
  end
  local roster=table.concat(aiCars,",")
  if #aiCars==0 then
    if state.spawnWait>=CFG.SPAWN_WAIT_TIMEOUT then
      state.spawnDone=true
    end
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
  if not physics or type(physics.setAICarPosition)~="function" then
    state.spawnDone=true; return
  end
  local metrics=buildSpawnRouteMetrics()
  if not metrics then state.spawnDone=true; return end
  local count=#aiCars
  local gap=math.min(CFG.SPAWN_GAP,metrics.length/(count+1))
  if gap<CFG.SPAWN_MIN_GAP then state.spawnDone=true; return end
  local startDistance=0
  if metrics.loop then
    local player=ac.getCar(0)
    if player then
      local routeDistance,routeError=findRouteDistanceAtPosition(
        player.position,metrics,player.look)
      startDistance=routeDistance+gap
      writeLog("SPAWN_REFERENCE route=%.1f error=%.2f heading=(%.3f,%.3f)",
        routeDistance,routeError,player.look.x,player.look.z)
    else startDistance=gap end
  end
  local moved,failed=0,0
  for order=1,count do
    local carIndex=aiCars[order]
    local position,direction,routeSegment,routeParameter=
      sampleSpawnRoute(metrics,startDistance+(order-1)*gap)
    if position and direction then
      -- Keep the previously user-verified orientation. The SDK documents
      -- an explicit direction vector but does not define its sign convention.
      local ok,result=pcall(physics.setAICarPosition,carIndex,
        position,vec3(-direction.x,-direction.y,-direction.z))
      if ok and result~=false then
        moved=moved+1
        state.p[carIndex]=nil
        state.spawnHints[carIndex]={segment=routeSegment,parameter=routeParameter}
        writeLog("SPAWN_CAR id=%d pos=(%.2f,%.2f,%.2f) dir=(%.3f,%.3f) route_seg=%d route_t=%.3f",
          carIndex,position.x,position.y,position.z,direction.x,direction.z,
          routeSegment,routeParameter)
      else failed=failed+1 end
    else failed=failed+1 end
  end
  state.spawnDone=true
  writeLog("SPAWN_FINISH moved=%d failed=%d gap=%.2f",moved,failed,gap)
  flushLog()
end
local function trySpawnCSPTraffic(sim,dt,route)
  if not trafficCSP or trafficCSP.graphSpawnDone then return end
  local routes=trafficCSP.previewRoutes or (route and {route}) or {}
  if #routes==0 then return end
  table.sort(routes,function(a,b)
    if a.playerAligned~=b.playerAligned then return a.playerAligned==true end
    return (a.length or 0)>(b.length or 0)
  end)
  trafficCSP.playerRouteIndex=nil
  for ri=1,#routes do
    if routes[ri].playerAligned then trafficCSP.playerRouteIndex=ri; break end
  end
  trafficCSP.previewRoute=routes[1]
  trafficCSP.graphSpawnWait=(trafficCSP.graphSpawnWait or 0)+dt
  local aiCars={}
  for i=1,sim.carsCount-1 do
    local car=ac.getCar(i)
    if car and car.isActive and car.isAIControlled then aiCars[#aiCars+1]=i end
  end
  if #aiCars==0 then
    if trafficCSP.graphSpawnWait>=CFG.SPAWN_WAIT_TIMEOUT and not trafficCSP.graphSpawnNoCarsLogged then
      trafficCSP.graphSpawnNoCarsLogged=true
      writeLog("CSP_SPAWN_WAITING reason=no_active_ai_cars")
      flushLog()
    end
    return
  end
  local roster=table.concat(aiCars,",")
  if roster~=trafficCSP.graphSpawnLastRoster then
    trafficCSP.graphSpawnLastRoster=roster
    trafficCSP.graphSpawnStable=0
    return
  end
  trafficCSP.graphSpawnStable=(trafficCSP.graphSpawnStable or 0)+dt
  if trafficCSP.graphSpawnStable<CFG.SPAWN_STABLE_TIME then return end
  if not physics or type(physics.setAICarPosition)~="function" then
    trafficCSP.graphSpawnFailed='no_ai_position_api'
    writeLog("CSP_SPAWN_FAILED reason=no_ai_position_api")
    return
  end

  local routeCount=math.min(#routes,#aiCars)
  local plans=nil
  while routeCount>0 and not plans do
    local carsByRoute={}
    for ri=1,routeCount do carsByRoute[ri]={} end
    for order=1,#aiCars do
      local ri=(order-1)%routeCount+1
      carsByRoute[ri][#carsByRoute[ri]+1]=aiCars[order]
    end
    local candidatePlans={}
    local minGap=math.huge
    local valid=true
    for ri=1,routeCount do
      local currentRoute=routes[ri]
      local groupCars=carsByRoute[ri]
      local startDistance=math.min(40.0,currentRoute.length*0.08)
      local available=math.max(0,currentRoute.length-startDistance-10.0)
      local gap=math.min(CFG.SPAWN_GAP,available/math.max(1,#groupCars))
      if gap<CFG.SPAWN_MIN_GAP then
        valid=false
        break
      end
      minGap=math.min(minGap,gap)
      for order=1,#groupCars do
        local index=groupCars[order]
        local sample=CSPRoutes.sample(currentRoute,startDistance+(order-1)*gap)
        if not sample then
          valid=false
          break
        end
        candidatePlans[#candidatePlans+1]={
          index=index,routeIndex=ri,sample=sample,route=currentRoute
        }
      end
      if not valid then break end
    end
    if valid and #candidatePlans==#aiCars then
      -- Validate the complete teleport transaction before moving any car.
      -- Ignore current positions of cars that are part of this transaction,
      -- but keep the player and all non-moving active cars as hard obstacles.
      local movingCars={}
      for ci=1,#aiCars do movingCars[aiCars[ci]]=true end
      local clear=true
      local conflictA,conflictB=nil,nil
      local clearSq=CFG.SAFE_RESPAWN_CLEAR*CFG.SAFE_RESPAWN_CLEAR
      for pi=1,#candidatePlans do
        local sample=candidatePlans[pi].sample
        for ci=0,sim.carsCount-1 do
          if not movingCars[ci] then
            local other=ac.getCar(ci)
            if other and other.isActive then
              local dx=other.position.x-sample.x
              local dz=other.position.z-sample.z
              if dx*dx+dz*dz<clearSq then
                clear=false
                conflictA,conflictB=candidatePlans[pi].index,ci
                break
              end
            end
          end
        end
        if not clear then break end
        for pj=1,pi-1 do
          local previous=candidatePlans[pj].sample
          local dx=previous.x-sample.x
          local dz=previous.z-sample.z
          if dx*dx+dz*dz<clearSq then
            clear=false
            conflictA,conflictB=candidatePlans[pi].index,candidatePlans[pj].index
            break
          end
        end
        if not clear then break end
      end
      if clear then
        plans=candidatePlans
        trafficCSP.graphSpawnGap=minGap
      else
        writeLog("CSP_SPAWN_PLAN_REJECT reason=occupied_destination car=%d other=%d routes=%d",
          conflictA or -1,conflictB or -1,routeCount)
        routeCount=routeCount-1
      end
    else
      routeCount=routeCount-1
    end
  end
  if not plans then
    trafficCSP.graphSpawnFailed='insufficient_route_capacity'
    writeLog("CSP_SPAWN_FAILED reason=insufficient_route_capacity cars=%d routes=%d",
      #aiCars,#routes)
    flushLog()
    return
  end

  local backups,moved={},{}
  local failed=0
  trafficCSP.graphCarRoute={}
  for pi=1,#plans do
    local plan=plans[pi]
    local index,car,sample=plan.index,ac.getCar(plan.index),plan.sample
    if car then
      backups[index]={
        position=vec3(car.position.x,car.position.y,car.position.z),
        look=vec3(car.look.x,car.look.y,car.look.z)
      }
      local position=vec3(sample.x,sample.y,sample.z)
      local direction=vec3(-sample.tx,0,-sample.tz)
      local ok,result=pcall(physics.setAICarPosition,index,position,direction)
      if ok and result~=false then
        moved[#moved+1]=index
        state.p[index]=nil
        state.spawnHints[index]={segment=sample.seg,parameter=sample.t}
        state.rescueWatch=state.rescueWatch or {}
        state.rescueWatch[index]={cooldown=3.0}
        trafficCSP.graphCarRoute[index]=plan.routeIndex
      else
        failed=failed+1
        writeLog("CSP_SPAWN_CAR_FAILED id=%d err=%s",index,tostring(result))
      end
    else
      failed=failed+1
    end
  end

  if failed>0 then
    for i=1,#moved do
      local index=moved[i]
      local backup=backups[index]
      if backup then
        pcall(physics.setAICarPosition,index,backup.position,
          vec3(-backup.look.x,-backup.look.y,-backup.look.z))
      end
      state.spawnHints[index]=nil
      state.p[index]=nil
      if state.rescueWatch then state.rescueWatch[index]=nil end
      trafficCSP.graphCarRoute[index]=nil
    end
    trafficCSP.graphSpawnFailed='spawn_transaction_rolled_back'
    writeLog("CSP_SPAWN_ROLLBACK moved=%d failed=%d",#moved,failed)
    flushLog()
    return
  end

  trafficCSP.graphSpawnDone=true
  trafficCSP.graphSpawnCount=#plans
  trafficCSP.graphSpawnRouteCount=routeCount
  writeLog("CSP_SPAWN_FINISH moved=%d routes=%d min_gap=%.2f",
    #plans,routeCount,trafficCSP.graphSpawnGap or 0)
  flushLog()
end

local function tryRecycleGraphTraffic(route,order,player)
  if not route or not ac.closerToRenderThan or
      not physics or type(physics.setAICarPosition)~="function" then return 0 end
  if not route.loop then
    -- Open one-way networks have no directed cycle to wrap around. Recycle a
    -- car only after it has reached the terminal portion and is outside render
    -- range, placing it into a clear, spaced slot near the route entrance.
    -- This keeps every visible vehicle on the authored direction of travel.
    local total=route.length or 0
    if total<300 then return 0 end
    local terminalThreshold=math.min(500,math.max(150,total*0.08))
    local recycled=0
    for _,index in ipairs(order or {}) do
      local car=ac.getCar(index)
      local person=state.p[index]
      if car and person and person.routeProgress and
          person.routeProgress>=total-terminalThreshold and
          not ac.closerToRenderThan(car.position,1000) then
        local slots=math.max(1,math.floor((total-60)/CFG.SPAWN_GAP))
        for slot=0,math.min(slots-1,31) do
          local startDistance=40+slot*CFG.SPAWN_GAP
          local sample=CSPRoutes.sample(route,startDistance)
          if sample then
            local position=vec3(sample.x,sample.y,sample.z)
            if spotIsClear(position,index) then
              local direction=vec3(-sample.tx,0,-sample.tz)
              local ok,result=pcall(physics.setAICarPosition,index,position,direction)
              if ok and result~=false then
                state.p[index]=nil
                state.spawnHints[index]={segment=sample.seg,parameter=sample.t}
                state.rescueWatch=state.rescueWatch or {}
                state.rescueWatch[index]={cooldown=3.0}
                recycled=recycled+1
                writeLog("CSP_TRAFFIC_RECYCLE_OPEN id=%d progress=%.1f slot=%d start=%.1f",
                  index,person.routeProgress,slot,startDistance)
                break
              end
            end
          end
        end
      end
    end
    return recycled
  end
  if not player then return 0 end
  local projection=CSPRoutes.project(route,player.position,player.look)
  if not projection or not projection.distance or not projection.s or projection.distance>60 then return 0 end
  local total=route.length or 0
  if total<200 then return 0 end
  local behindThreshold=math.min(800,total*0.25)
  local recycled=0
  for _,index in ipairs(order) do
    local car=ac.getCar(index)
    local person=state.p[index]
    if car and person and person.routeProgress and
        not ac.closerToRenderThan(car.position,1000) then
      local aheadDelta=(person.routeProgress-projection.s)%total
      local behindDistance=aheadDelta>total*0.5 and total-aheadDelta or 0
      if behindDistance>behindThreshold then
        local offsets={120,180,240,300,360,420}
        for oi=1,#offsets do
          local sample=CSPRoutes.sample(route,projection.s+offsets[oi])
          if sample then
            local position=vec3(sample.x,sample.y,sample.z)
            if spotIsClear(position,index) then
              local direction=vec3(-sample.tx,0,-sample.tz)
              local ok,result=pcall(physics.setAICarPosition,index,position,direction)
              if ok and result~=false then
                state.p[index]=nil
                state.spawnHints[index]={segment=sample.seg,parameter=sample.t}
                state.rescueWatch=state.rescueWatch or {}
                state.rescueWatch[index]={cooldown=3.0}
                recycled=recycled+1
                writeLog("CSP_TRAFFIC_RECYCLE id=%d behind=%.1f ahead=%.1f",
                  index,behindDistance,offsets[oi])
                break
              end
            end
          end
        end
      end
    end
  end
  return recycled
end

local function walkLineSigned(startSeg,startT,distance,sign)
  local segs=segmentCount()
  if segs<1 then return nil end
  local seg=startSeg
  local t=startT
  local remaining=distance
  for _=1,segs+2 do
    local a=state.nodes[seg]
    local b=segEnd(seg)
    local length=flatDist(a,b)
    if length>=0.0001 then
      local room=sign>=0 and length*(1-t) or length*t
      if room>=remaining then
        local nt=(length*t+sign*remaining)/length
        return vec3(lerp(a.x,b.x,nt),lerp(a.y,b.y,nt),lerp(a.z,b.z,nt)),seg,nt
      end
      remaining=remaining-room
      if sign>=0 then
        if state.loopRoute then seg=(seg%segs)+1
        elseif seg<segs then seg=seg+1 else return nil end
        t=0
      else
        if state.loopRoute then seg=((seg-2)%segs)+1
        elseif seg>1 then seg=seg-1 else return nil end
        t=1
      end
    else
      if sign>=0 then
        if state.loopRoute then seg=(seg%segs)+1
        elseif seg<segs then seg=seg+1 else return nil end
        t=0
      else
        if state.loopRoute then seg=((seg-2)%segs)+1
        elseif seg>1 then seg=seg-1 else return nil end
        t=1
      end
    end
  end
  return nil
end
local function teleportHome(carIndex,car)
  if type(physics)~="table" or type(physics.setAICarPosition)~="function" then
    writeLog("RESCUE_FAILED id=%d reason=no_teleport_api",carIndex)
    return false
  end
  if #state.nodes<2 or not car or not car.position then return false end

  -- Recovery direction is fixed by the recorded line order, never by
  -- the vehicle's current heading or velocity after a spin/collision.
  -- Reuse the car's last route segment to disambiguate nearby track branches.
  local driverState=state.p and state.p[carIndex]
  local hintSeg=driverState and driverState.segIdx or 1
  local allowGlobal=not (driverState and driverState.routeLocked and driverState.segIdx)
  local seg,t,routeError=findNearestSeg(car.position,hintSeg,allowGlobal,nil)
  if not allowGlobal and routeError>15.0 then
    seg,t,routeError=findNearestSeg(car.position,hintSeg,true,nil)
  end
  local sign=1
  if routeError>250 then
    writeLog("RESCUE_FAILED id=%d reason=route_too_far error=%.1f",carIndex,routeError)
    return false
  end

  -- Never search backward along the recorded line. Candidate points are
  -- sampled strictly in increasing route-node order, beyond the obstruction.
  for distance=20,320,10 do
    local pos,sampleSeg,sampleT=walkLineSigned(seg,t,distance,sign)
    if pos then
      local nextPos=walkLineSigned(sampleSeg,sampleT,3.0,sign)
      if nextPos then
        local dx,dz=normalize2(nextPos.x-pos.x,nextPos.z-pos.z)
        local safePos=vec3(pos.x,pos.y+0.15,pos.z)
        if spotIsClear(safePos,carIndex) then
          local ok,result=pcall(physics.setAICarPosition,carIndex,
            safePos,vec3(-dx,0,-dz))
          if ok and result~=false then
            state.p[carIndex]=nil
            state.rescueWatch=state.rescueWatch or {}
            state.rescueWatch[carIndex]={cooldown=3.0}
            writeLog("RESCUE_FORWARD id=%d distance=%.1f route_error=%.1f direction=RECORDED_LINE_FORWARD sign=%d pos=(%.2f,%.2f,%.2f)",
              carIndex,distance,routeError,sign,safePos.x,safePos.y,safePos.z)
            flushLog()
            return true
          end
          writeLog("RESCUE_FORWARD_FAILED id=%d distance=%.1f error=%s",
            carIndex,distance,tostring(result))
        end
      end
    end
  end
  writeLog("RESCUE_FAILED id=%d reason=no_clear_forward_point route_error=%.1f",
    carIndex,routeError)
  flushLog()
  return false
end

local function updateRescueWatchdog(carIndex,car,dt)
  if not car or not car.isActive or not car.isAIControlled then
    if state.rescueWatch then state.rescueWatch[carIndex]=nil end
    return false
  end
  state.rescueWatch=state.rescueWatch or {}
  local w=state.rescueWatch[carIndex]
  if not w then
    w={rolloverT=0,gasStallT=0,trafficWaitT=0,lastPosition=vec3(car.position.x,car.position.y,car.position.z),progressPositionX=car.position.x,progressPositionZ=car.position.z,progressCheckT=0,cooldown=0}
    state.rescueWatch[carIndex]=w
  end
  w.cooldown=math.max(0,(w.cooldown or 0)-dt)
  if w.cooldown>0 then
    w.lastPosition=vec3(car.position.x,car.position.y,car.position.z)
    w.gasStallT=0
    w.rolloverT=0
    return false
  end

  local upY=car.up and safeNumber(car.up.y,1) or 1
  if upY<0.55 then w.rolloverT=w.rolloverT+dt else w.rolloverT=0 end

  local gas=safeNumber(car.gas)
  local speed=math.abs(safeNumber(car.speedMs))
  local p=state.p and state.p[carIndex]
  local trafficWaiting=p and
    (safeNumber(p.emergencyBlockedT)>0 or p.trafficAction=="EMERGENCY_WAIT")
  w.progressCheckT=safeNumber(w.progressCheckT)+dt
  if w.progressCheckT>=1.0 then
    local px=car.position.x-safeNumber(w.progressPositionX,car.position.x)
    local pz=car.position.z-safeNumber(w.progressPositionZ,car.position.z)
    local progress=math.sqrt(px*px+pz*pz)
    if not trafficWaiting and gas>=0.55 and progress<0.5 then
      w.gasStallT=w.gasStallT+w.progressCheckT
    else
      w.gasStallT=0
    end
    if trafficWaiting and speed<0.75 and progress<0.5 and
        trafficCSP and trafficCSP.graphSpawnDone and state.loopRoute then
      w.trafficWaitT=safeNumber(w.trafficWaitT)+w.progressCheckT
    else
      w.trafficWaitT=0
    end
    w.progressPositionX=car.position.x
    w.progressPositionZ=car.position.z
    w.progressCheckT=0
  end
  w.lastPosition=vec3(car.position.x,car.position.y,car.position.z)

  local offTrackT=safeNumber(p and p.offTrackT)
  local reason=nil
  if w.rolloverT>=10.0 then reason="rollover_10s"
  elseif offTrackT>=10.0 then reason="offtrack_10s"
  elseif w.trafficWaitT>=5.0 then reason="traffic_wait_deadlock_5s"
  elseif w.gasStallT>=10.0 then reason="gas_no_progress_10s" end
  w.retryT=math.max(0,safeNumber(w.retryT)-dt)
  if reason and w.retryT<=0 then
    local rescued=teleportHome(carIndex,car)
    writeLog("RESCUE_WATCHDOG id=%d reason=%s success=%s rollover=%.1f offtrack=%.1f gas_stall=%.1f speed=%.2f gas=%.2f",
      carIndex,reason,tostring(rescued),w.rolloverT,offTrackT,w.gasStallT,speed,gas)
    if rescued then
      state.rescueWatch[carIndex]={cooldown=3.0}
      return true
    end
    -- Keep the fault timers, but retry the forward-point scan only every two seconds.
    w.retryT=2.0
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
  local n=#state.nodes
  if n<2 then return end
  local half=WIDTH_PRESETS[preset]/2
  for i=1,n do
    state.boundsL[i]=-half
    state.boundsR[i]=half
  end
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
    getVertexCurvature=getVertexCurvature, carAheadFull=carAheadFull,
    teleportHome=teleportHome, log=writeLog,
  })
  driverReady=true
  return true
end
function script.update(dt)
  if not dt or dt<=0 then return end
  dt=math.min(dt,0.05)
  local ok,err=pcall(function()
    if not trafficCSPAttempted then
      trafficCSPAttempted = true
      if trafficCSP then
        local okImport, imported = pcall(trafficCSP.load, trafficCSP)
        if not okImport then
          writeLog("CSP_IMPORT_ERROR err=%s", tostring(imported))
        elseif not imported then
          writeLog("CSP_IMPORT_UNAVAILABLE reason=%s", tostring(trafficCSP.error))
        end
      else
        writeLog("CSP_IMPORT_MODULE_UNAVAILABLE")
      end
      flushLog()
    end
    local sim=ac.getSim()
    if not sim then return end
    local player=ac.getCar(0)
    if not trafficCSPRouteAttempted and trafficCSP and trafficCSP.loaded and
        trafficCSP.graph and trafficCSP.roads and CSPGraph and CSPRoutes and player then
      trafficCSPRouteRetry=trafficCSPRouteRetry+dt
      if trafficCSPRouteRetry>=1.0 then
        trafficCSPRouteRetry=0
        local sameCandidates=CSPGraph.nearestEdges(
          trafficCSP.graph,player.position,player.look,120,24)
        local allCandidates=CSPGraph.nearestEdges(
          trafficCSP.graph,player.position,nil,120,48)
        local candidates,seenEdges={},{}
        local function addCandidate(candidate)
          if candidate and not seenEdges[candidate.edge.id] then
            seenEdges[candidate.edge.id]=true
            candidates[#candidates+1]=candidate
          end
        end
        local hx,hz=normalize2(player.look.x,player.look.z)
        local firstSame=sameCandidates[1]
        if firstSame then
          local dir=CSPGraph.directionAt(firstSame.edge.lane,firstSame.along)
          if hx*dir.x+hz*dir.z>=0.5 then addCandidate(firstSame) end
        end
        -- Keep additional player-aligned lanes as fallbacks. Previously only
        -- the single nearest same-direction edge was attempted, so one bad
        -- route candidate could hide every other valid lane in that direction.
        for ci=1,#sameCandidates do
          local candidate=sameCandidates[ci]
          if candidate~=firstSame then
            local dir=CSPGraph.directionAt(candidate.edge.lane,candidate.along)
            if hx*dir.x+hz*dir.z>=0.5 then addCandidate(candidate) end
          end
        end
        local oppositeCandidates,otherCandidates={},{}
        for ci=1,#allCandidates do
          local candidate=allCandidates[ci]
          local dir=CSPGraph.directionAt(candidate.edge.lane,candidate.along)
          local dot=hx*dir.x+hz*dir.z
          if dot < -0.5 then oppositeCandidates[#oppositeCandidates+1]=candidate
          elseif dot > 0.5 then otherCandidates[#otherCandidates+1]=candidate end
        end
        for ci=1,#oppositeCandidates do addCandidate(oppositeCandidates[ci]) end
        for ci=1,#otherCandidates do addCandidate(otherCandidates[ci]) end
        local selectedRoutes,selectedGroups={},{}
        local lastError='no_lane_near_player'
        local attemptedCandidates=0
        trafficCSP.failedRouteEdges=trafficCSP.failedRouteEdges or {}
        local function collectCandidates(requireLoop)
          for ci=1,#candidates do
            if #selectedRoutes>=4 then break end
            local candidate=candidates[ci]
            local group=trafficCSP.roads.laneGroups[candidate.edge.lane] or candidate.edge.lane
            local failureKey=(requireLoop and 'loop:' or 'open:')..tostring(candidate.edge.id)
            if not selectedGroups[group] and not trafficCSP.failedRouteEdges[failureKey] then
              attemptedCandidates=attemptedCandidates+1
              local route,routeError=CSPRoutes.build(
                trafficCSP.graph,trafficCSP.roads,candidate.edge,candidate.along,
                requireLoop and
                  {minLength=250,loop=true,requireLoop=true} or
                  {minLength=250,loop=false})
              if route then
                local candidateDirection=CSPGraph.directionAt(candidate.edge.lane,candidate.along)
                route.playerAlignment=hx*candidateDirection.x+hz*candidateDirection.z
                route.playerAligned=route.playerAlignment>=0.5
                route.group=group
                route.startCandidate=candidate
                selectedRoutes[#selectedRoutes+1]=route
                selectedGroups[group]=true
              else
                trafficCSP.failedRouteEdges[failureKey]=true
                lastError=tostring(routeError)
              end
            end
          end
        end
        -- Prefer genuine directed cycles. Only if no loop can be built do
        -- we fall back to long one-way routes; never invent a reverse link
        -- or close an open road with a straight-line shortcut.
        collectCandidates(true)
        if #selectedRoutes==0 then collectCandidates(false) end
        if #selectedRoutes>0 then
          trafficCSPRouteAttempted=true
          trafficCSP.previewRoutes=selectedRoutes
          trafficCSP.previewRoute=selectedRoutes[1]
          for ri=1,#selectedRoutes do
            local route=selectedRoutes[ri]
            writeLog("CSP_ROUTE_READY index=%d length=%.1f points=%d edges=%d group_lanes=%d",
              ri,route.length or 0,#route.nodes,#route.edges,#(route.group and route.group.lanes or {}))
          end
          writeLog("CSP_ROUTES_SELECTED count=%d candidates=%d",
            #selectedRoutes,#candidates)
        else
          if #candidates>0 and attemptedCandidates==0 then lastError='all_nearby_edges_rejected' end
          trafficCSP.previewRouteError=lastError
          trafficCSPRouteFailures=trafficCSPRouteFailures+1
          if trafficCSPRouteFailures==1 or trafficCSPRouteFailures%5==0 then
            writeLog("CSP_ROUTE_PREVIEW_RETRY candidates=%d reason=%s attempt=%d",
              #candidates,lastError,trafficCSPRouteFailures)
          end
        end
        flushLog()
      end
    end
    local graphRoute=trafficCSP and trafficCSP.previewRoute or nil
    if state.enabled and not state.recLine and not state.recBounds and graphRoute then
      trySpawnCSPTraffic(sim,dt,graphRoute)
    end
    local useCSPRoute=state.enabled and not state.recLine and not state.recBounds and
      graphRoute and trafficCSP.graphSpawnDone==true
    if trafficCSP then
      if useCSPRoute then
        if not trafficCSP.graphDriverModeActive then
          for _,person in pairs(state.p) do
            person.routeProgress=nil
            person.routeLocked=false
            person.segIdx=1
          end
        end
        trafficCSP.graphDriverModeActive=true
      else
        if trafficCSP.graphDriverModeActive then
          for _,person in pairs(state.p) do
            person.routeProgress=nil
            person.routeLocked=false
            person.segIdx=1
          end
        end
        trafficCSP.graphDriverModeActive=false
      end
    end
    if state.recLine and player then addPoint(player.position) end
    if state.recBounds and player then updateBoundsRecording(player) end
    if state.enabled and state.loaded and #state.nodes>=2 and
        not (graphRoute and not trafficCSP.graphSpawnFailed and
          not state.recLine and not state.recBounds) then
      trySpawnTeleport(sim,dt)
    end
    -- Ctrl+8 switches the player car into Assetto Corsa's native AI state.
    -- RoutePilot adopts car 0 only in that state and runs the same route,
    -- lane, traffic and control loop as every other AI car. Manual driving
    -- remains passive and is never overridden.
    local playerAI = player and player.isActive and player.isAIControlled
    if playerAI then
      state.playerTelemetry=nil
    elseif #state.nodes>=2 and not state.recLine and Driver and ensureDriver() then
      if player and player.isActive and Driver.observePlayer then
        local steerInput=0
        if ac.getControllerSteerValue then
          local okSteer,value=pcall(ac.getControllerSteerValue)
          if okSteer then steerInput=value end
        end
        local okObserve,observeError=pcall(Driver.observePlayer,player,dt,steerInput)
        if not okObserve then
          state.playerTelemetry=nil
          writeLog("PLAYER_TELEMETRY_ERROR err=%s",tostring(observeError))
        end
      end
    elseif state.recLine then
      state.playerTelemetry=nil
    end

    local playerRoutePilotActive = state.enabled and (#state.nodes>=2 or useCSPRoute) and
      Driver and playerAI and (not useCSPRoute or trafficCSP.playerRouteIndex~=nil) or false
    if playerRoutePilotActive and not state.playerAutopilotLast then
      writeLog("PLAYER_AUTOPILOT_ADOPT source=AC_NATIVE_CTRL8")
    elseif not playerRoutePilotActive and state.playerAutopilotLast then
      writeLog("PLAYER_AUTOPILOT_RELEASE")
    end
    state.playerAutopilotLast = playerRoutePilotActive == true

    -- Drive every active car which Assetto Corsa currently marks as AI,
    -- including car 0 only while the native Ctrl+8 autopilot is active.
    local routeSnapshot=nil
    if state.enabled and (#state.nodes>=2 or useCSPRoute) and Driver and ensureDriver() then
      local driveOrder={}
      for i=0,sim.carsCount-1 do
        local car=ac.getCar(i)
        local shouldDrive=car and car.isActive and car.isAIControlled
        if i==0 then shouldDrive=shouldDrive and playerAI end
        if shouldDrive then
          driveOrder[#driveOrder+1]=i
        elseif state.p[i] then
          releaseCar(i)
        end
      end

      if useCSPRoute then
        routeSnapshot={
          nodes=state.nodes,boundsL=state.boundsL,boundsR=state.boundsR,
          loopRoute=state.loopRoute,routeGeometry=state.routeGeometry,
          routeLength=state.routeLength,routeMetricCount=state.routeMetricCount,
          routeMetricLoop=state.routeMetricLoop,graphRouteSwitch=state.graphRouteSwitch
        }
        state.graphRouteSwitch=true
        if playerAI then state.p[0]=nil end
        local routes=trafficCSP.previewRoutes or {graphRoute}
        local routeOrders={}
        for ri=1,#routes do routeOrders[ri]={} end
        for _,i in ipairs(driveOrder) do
          local ri=trafficCSP.graphCarRoute and trafficCSP.graphCarRoute[i] or nil
          if i==0 and playerAI then ri=trafficCSP.playerRouteIndex end
          if ri and routeOrders[ri] then routeOrders[ri][#routeOrders[ri]+1]=i end
        end

        for ri=1,#routes do
          local order=routeOrders[ri]
          if #order>0 then
            local route=routes[ri]
            state.nodes=route.nodes
            state.boundsL=route.boundsL
            state.boundsR=route.boundsR
            state.loopRoute=route.loop==true
            state.routeGeometry=route.geometry
            state.routeLength=route.length
            state.routeMetricCount=#route.nodes
            state.routeMetricLoop=route.loop==true
            tryRecycleGraphTraffic(route,order,player)

            local savedOther={}
            for idx,person in pairs(state.p) do
              local assigned=trafficCSP.graphCarRoute and trafficCSP.graphCarRoute[idx] or nil
              if idx==0 and playerAI then assigned=trafficCSP.playerRouteIndex end
              if assigned~=ri then
                savedOther[idx]={
                  routeProgress=person.routeProgress,
                  routeLocked=person.routeLocked,
                  segIdx=person.segIdx
                }
                person.routeProgress=nil
                person.routeLocked=false
                person.segIdx=1
              end
            end

            local okGroup,errGroup=pcall(function()
              local coordinatorOK=true
              if Driver.prepareTraffic then
                local okTraffic,errTraffic=pcall(Driver.prepareTraffic,sim,order,dt)
                if not okTraffic then
                  coordinatorOK=false
                  writeLog("TRAFFIC_COORDINATOR_ERROR route=%d err=%s",ri,tostring(errTraffic))
                end
              end
              if coordinatorOK then
                for _,i in ipairs(order) do
                  local person=state.p[i]
                  if person then
                    local speedLimit=200
                    if route.speedLimits and #route.speedLimits>0 and person.routeProgress then
                      local speedSample=CSPRoutes.sample(route,person.routeProgress)
                      if speedSample then
                        local seg=clamp(math.floor(speedSample.seg or 1),1,#route.speedLimits)
                        local nextSeg=seg%#route.speedLimits+1
                        local t=clamp(speedSample.t or 0,0,1)
                        speedLimit=route.speedLimits[seg]*(1-t)+route.speedLimits[nextSeg]*t
                      end
                    end
                    person.cruiseSpeed=clamp(speedLimit,40,200)/3.6
                  end
                end
                table.sort(order,function(a,b)
                  local pa,pb=state.p[a],state.p[b]
                  local priorityA=safeNumber(pa and pa.trafficPriority)
                  local priorityB=safeNumber(pb and pb.trafficPriority)
                  if priorityA~=priorityB then return priorityA>priorityB end
                  local progressA=pa and pa.routeProgress
                  local progressB=pb and pb.routeProgress
                  if progressA~=nil and progressB~=nil and progressA~=progressB then
                    return progressA<progressB
                  end
                  if progressA~=nil and progressB==nil then return true end
                  if progressA==nil and progressB~=nil then return false end
                  return a<b
                end)
                for _,i in ipairs(order) do
                  local car=ac.getCar(i)
                  if car and car.isActive and car.isAIControlled then
                    local rescued=updateRescueWatchdog(i,car,dt)
                    if not rescued then
                      local okCar,errCar=pcall(Driver.update,i,car,dt)
                      if not okCar then
                        writeLog("DRIVER_ERROR id=%d route=%d err=%s",i,ri,tostring(errCar))
                        releaseCar(i)
                      end
                    end
                  end
                end
              end
            end)
            for idx,saved in pairs(savedOther) do
              local person=state.p[idx]
              if person then
                person.routeProgress=saved.routeProgress
                person.routeLocked=saved.routeLocked
                person.segIdx=saved.segIdx
              end
            end
            if not okGroup then writeLog("DRIVER_ROUTE_BATCH_ERROR route=%d err=%s",ri,tostring(errGroup)) end
          end
        end
      else
        if Driver.prepareTraffic then
          local okTraffic,errTraffic=pcall(Driver.prepareTraffic,sim,driveOrder,dt)
          if not okTraffic then writeLog("TRAFFIC_COORDINATOR_ERROR err=%s",tostring(errTraffic)) end
        end
        local okLoop,errLoop=pcall(function()
          table.sort(driveOrder,function(a,b)
            local pa,pb=state.p[a],state.p[b]
            local priorityA=safeNumber(pa and pa.trafficPriority)
            local priorityB=safeNumber(pb and pb.trafficPriority)
            if priorityA~=priorityB then return priorityA>priorityB end
            local progressA=pa and pa.routeProgress
            local progressB=pb and pb.routeProgress
            if progressA~=nil and progressB~=nil and progressA~=progressB then
              return progressA<progressB
            end
            if progressA~=nil and progressB==nil then return true end
            if progressA==nil and progressB~=nil then return false end
            return a<b
          end)
          for _,i in ipairs(driveOrder) do
            local car=ac.getCar(i)
            if car and car.isActive and car.isAIControlled then
              local rescued=updateRescueWatchdog(i,car,dt)
              if not rescued then
                local okCar,errCar=pcall(Driver.update,i,car,dt)
                if not okCar then
                  writeLog("DRIVER_ERROR id=%d err=%s",i,tostring(errCar))
                  releaseCar(i)
                end
              end
            end
          end
        end)
        if not okLoop then writeLog("DRIVER_BATCH_ERROR err=%s",tostring(errLoop)) end
      end
    else
      for idx in pairs(state.p) do releaseCar(idx) end
    end
    if routeSnapshot then
      state.nodes=routeSnapshot.nodes
      state.boundsL=routeSnapshot.boundsL
      state.boundsR=routeSnapshot.boundsR
      state.loopRoute=routeSnapshot.loopRoute
      state.routeGeometry=routeSnapshot.routeGeometry
      state.routeLength=routeSnapshot.routeLength
      state.routeMetricCount=routeSnapshot.routeMetricCount
      state.routeMetricLoop=routeSnapshot.routeMetricLoop
      state.graphRouteSwitch=routeSnapshot.graphRouteSwitch
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
  local isRec=state.recLine or state.recBounds
  if n<2 and not state.showCSPRoads then return end
  if not (state.showWorldLines or isRec or state.showCSPRoads) then return end
  local ok=pcall(function()
    local player=ac.getCar(0)
    if not player then return end
    render.setDepthMode(render.DepthMode.Off)
    local YELLOW=rgbm(1.0,0.85,0.0,1)
    local RED=rgbm(0.6,0.05,0.05,1)
    if state.showCSPRoads and trafficCSP and trafficCSP.roads then
      local roads=trafficCSP.roads
      local green=rgbm(0.1,0.9,0.35,0.95)
      local cyan=rgbm(0.1,0.65,1.0,0.95)
      local preview=trafficCSP.previewRoute
      if preview and preview.nodes then
        local routeColor=rgbm(1.0,0.55,0.08,0.95)
        for ri=1,#preview.nodes-1 do
          local a,b=preview.nodes[ri],preview.nodes[ri+1]
          local dx,dz=a.x-player.position.x,a.z-player.position.z
          local ex,ez=b.x-player.position.x,b.z-player.position.z
          if dx*dx+dz*dz<360000 or ex*ex+ez*ez<360000 then
            render.debugLine(a,b,routeColor)
          end
        end
      end
      for gi=1,#roads.groups do
        local group=roads.groups[gi]
        local left,right=group.boundaries.left,group.boundaries.right
        for bi=1,2 do
          local line=bi==1 and left or right
          local color=bi==1 and green or cyan
          local count=#line
          local segments=group.boundaryLoops[bi==1 and "left" or "right"] and count or count-1
          for si=1,segments do
            local a,b=line[si],line[si%count+1]
            local dx,dz=a.x-player.position.x,a.z-player.position.z
            local ex,ez=b.x-player.position.x,b.z-player.position.z
            if dx*dx+dz*dz<360000 or ex*ex+ez*ez<360000 then
              render.debugLine(a,b,color)
            end
          end
        end
      end
    end
    if n<2 then return end
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
      if isRec or flatDist(player.position,node)<CFG.DRAW_RANGE then
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
            if p and p.overtakeSide~=0 then
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
    ui.tabBar("RoutePilotTabs", function()
      ui.tabItem("Маршрут", function()
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
        state.nodes={}; state.p={}; state.playerTelemetry=nil; state.routeGeometry=nil; state.boundsL={}; state.boundsR={}
        state.loaded=false; state.recBounds=false
        state.status="Запись линии и границ..."
      else
        state.status="Записано точек: "..#state.nodes
      end
    end
    ui.sameLine(0,4)
    if ui.button(state.recBounds and "СТОП запись границ" or "Запись границ",vec2(-0.5,24)) then
      if state.recBounds then
        state.recBounds=false; state.status="Границы записаны"
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
    end
    if ui.button("Сохранить",vec2(-0.25,22)) then saveLine() end
    ui.sameLine(0,4)
    if ui.button("Загрузить",vec2(-0.25,22)) then loadLine() end
    ui.sameLine(0,4)
    if ui.button("Очистить",vec2(-0.25,22)) then
      state.nodes={}; state.p={}; state.boundsL={}; state.boundsR={}
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
      ui.tabItem("Dev", function()
        ui.text("Диагностика контроллера RoutePilot")
        ui.text("Подробная телеметрия рулевого управления и динамики автомобиля.")
        if trafficCSP then
          local s = trafficCSP.stats
          ui.text(string.format("CSP Traffic: %s | полосы %d | пригодны %d | без поверхности %d",
            trafficCSP.loaded and "импортирован" or (trafficCSP.error or "ожидание"),
            s.lanes or 0, s.usable or 0, s.invalid or 0))
          if trafficCSP.graph and trafficCSP.graph.stats then
            local g = trafficCSP.graph.stats
            ui.text(string.format("CSP Graph: перекрёстки %d | связи %d | переходы %d | сегменты %d | без связей %d",
              #trafficCSP.graph.intersections, g.intersectionLinks or 0,
              g.movements or 0, #trafficCSP.graph.edges, g.disconnectedLanes or 0))
          elseif trafficCSP.graphError then
            ui.text("CSP Graph: ошибка — " .. tostring(trafficCSP.graphError))
          else
            ui.text("CSP Graph: не построен")
          end
          if trafficCSP.previewRoute then
            ui.text(string.format("CSP Routes: %d | основной %.0f м | точки %d | %s",
              #(trafficCSP.previewRoutes or {trafficCSP.previewRoute}),
              trafficCSP.previewRoute.length or 0,#trafficCSP.previewRoute.nodes,
              trafficCSP.previewRoute.loop and "замкнут" or "открыт"))
            if trafficCSP.graphSpawnDone then
              ui.text(string.format("CSP AI: поток активен | машин %d | направлений %d | интервал %.1f м",
                trafficCSP.graphSpawnCount or 0, trafficCSP.graphSpawnRouteCount or 0,
                trafficCSP.graphSpawnGap or 0))
            elseif trafficCSP.graphSpawnFailed then
              ui.text("CSP AI: " .. tostring(trafficCSP.graphSpawnFailed))
            elseif trafficCSP.previewRoute.loop then
              ui.text("CSP AI: ожидание активных AI-машин")
            end
          elseif trafficCSP.previewRouteError then
            ui.text("CSP Route: " .. tostring(trafficCSP.previewRouteError))
          end
          if trafficCSP.roads and trafficCSP.roads.stats then
            local r = trafficCSP.roads.stats
            ui.text(string.format("CSP Roads: групп %d | линий %d | стеновых коррекций %d | конфликтов %d | встречные %d | параллельные %d | узлы %d | ray %s",
              r.groups or 0, r.validLanes or 0, r.wallAdjusted or 0,
              r.wallConflicts or 0, r.opposingConflicts or 0, r.parallelConflicts or 0,
              r.boundaryJunctions or 0,r.wallRaycastAvailable and "OK" or "N/A"))
          elseif trafficCSP.roadsError then
            ui.text("CSP Roads: ошибка — " .. tostring(trafficCSP.roadsError))
          end
        else
          ui.text("CSP Traffic: модуль не загружен")
        end
        if ui.checkbox("Показать границы CSP Traffic", state.showCSPRoads) then
          state.showCSPRoads = not state.showCSPRoads
        end
        if ui.button(state.steeringDiagnostics and "ЛогиРуля: ВКЛ" or "ЛогиРуля: ВЫКЛ",vec2(-0.5,28)) then
          state.steeringDiagnostics=not state.steeringDiagnostics
          writeLog("STEERING_DIAGNOSTICS %s",state.steeringDiagnostics and "ON" or "OFF")
          flushLog()
        end
        ui.text(state.steeringDiagnostics and
          "Подробные STEER_DIAG и DYN_TRACE включены для ИИ; PLAYER_DYN_TRACE — для ручного управления игроком. Обычные CAR= строки остаются отдельным базовым логом." or
          "Подробная диагностика выключена. Для ИИ и игрока продолжается запись обычных CAR= логов.")
      end)
    end)
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
writeLog("ROUTEPILOT 0.9.1 INITIALIZED track=%s points=%d",getTrackTag(),#state.nodes)
flushLog()