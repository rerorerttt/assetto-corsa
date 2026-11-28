-- RoutePilot.lua — v0.10.0 (CSP 0.3.2-preview2)
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
local okConverter, CSPConverter = pcall(require, "csp_converter")
if not okConverter or type(CSPConverter) ~= "table" or type(CSPConverter.load) ~= "function" then CSPConverter = nil end
local okDriver, Driver = pcall(require, "driver")
if not okDriver or type(Driver) ~= "table" or not Driver.update then Driver = nil end

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
  spawnHints={}, nodes={}, trafficNetwork=nil, p={}, status="Готов", logAccum=0, loaded=false,
  playerAutopilotLast=false, steeringDiagnostics=false, autoImportAccum=0, cspImportPending=false,
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
  if ac.storage then
    pcall(ac.storage,"RP_LINE_DATA_"..getTrackTag(),data)
    pcall(ac.storage,"RP_CSP_LINE_DATA_"..getTrackTag(),state.loadedFromCSP==true)
  end
  saveBounds()
  if wroteFile then
    state.status=string.format("Сохранено: %d точек",#state.nodes)
    writeLog("SAVE_ROUTE points=%d",#state.nodes)
    flushLog()
  else state.status="Ошибка сохранения" end
end
local function loadLine(retryOnly)
  state.trafficNetwork=nil
  state.loadedFromCSP=false
  state.loaded=false
  state.nodes={};state.p={};state.boundsL={};state.boundsR={}
  state.routeGeometry=nil
  local loaded=nil
  -- Prefer the CSP conversion; a saved RoutePilot line is the fallback.
  if CSPConverter then
    local ok, nodes, loop, boundsL, boundsR, infoOrError = pcall(CSPConverter.load, writeLog)
    if ok and type(nodes)=="table" and #nodes>=2 then
      state.nodes=nodes
      state.trafficNetwork=type(infoOrError)=="table" and infoOrError.network or nil
      if state.trafficNetwork and state.trafficNetwork.stats then
        local ns=state.trafficNetwork.stats
        writeLog("CSP_DIRECTED_NETWORK lanes=%d junction_edges=%d lane_changes=%d",
          safeNumber(ns.validLanes),safeNumber(ns.junctionTransitions),
          safeNumber(ns.laneChanges))
      end
      state.loopRoute=loop==true
      state.boundsL=type(boundsL)=="table" and boundsL or {}
      state.boundsR=type(boundsR)=="table" and boundsR or {}
      state.p={}; state.spawnHints={}; state.routeGeometry=nil
      state.loaded=true; state.loadedFromCSP=true; state.cspImportPending=false
      state.status=string.format("CSP: конвертировано %d точек",#nodes)
      state.spawnPerformed=false
      state.spawnDone=false; state.spawnWait=0; state.spawnStable=0; state.spawnLastRoster=""
      writeLog("CSP_IMPORT_SUCCESS points=%d loop=%s route_lanes=%s",
        #nodes,tostring(state.loopRoute),
        type(infoOrError)=="table" and #infoOrError.routeLanes or -1)
      flushLog()
      return true
    end
    local reason=tostring(ok and infoOrError or nodes)
    state.cspImportPending=reason:find("player car unavailable",1,true)~=nil or
      reason:find("No CSP lane near",1,true)~=nil
    writeLog("CSP_IMPORT_FAILED ok=%s reason=%s pending=%s",tostring(ok),
      reason,tostring(state.cspImportPending))
  end
  if retryOnly then return false end
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
    state.loadedFromCSP=false
    if ac.storage then
      local okCSP,flag=pcall(ac.storage,"RP_CSP_LINE_DATA_"..getTrackTag())
      state.loadedFromCSP=okCSP and flag==true
    end
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
      if state.trafficNetwork then
        if p.networkLaneId~=nil then
          writeLog("NETWORK_CAR id=%d lane=%d s=%.2f pos=(%.2f,%.2f,%.2f) speed=%.2f distance=%.2f height_error=%.2f steer=%.3f gas=%.3f brake=%.3f action=%s",
            i,safeNumber(p.networkLaneId,-1),safeNumber(p.networkS),
            safeNumber(car.position.x),safeNumber(car.position.y),safeNumber(car.position.z),
            safeNumber(car.speedMs),safeNumber(p.networkDistance),safeNumber(p.networkHeightError),
            safeNumber(p.steer),safeNumber(p.gas),safeNumber(p.brake),
            tostring(p.trafficAction or "UNKNOWN"))
        else
          writeLog("NETWORK_CAR_NO_LANE id=%d pos=(%.2f,%.2f,%.2f) speed=%.2f",
            i,safeNumber(car.position.x),safeNumber(car.position.y),safeNumber(car.position.z),
            safeNumber(car.speedMs))
        end
      elseif Driver and Driver.telemetryLine then
        writeLog(Driver.telemetryLine(i,car,p))
      end
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
  if type(physics)~="table" or type(physics.setAICarPosition)~="function" then
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
        -- Preserve the original RoutePilot spawn point and heading. Store only
        -- the temporary launch pulse; the network driver attaches the car to
        -- the nearest authored CSP lane on its next coordinator pass.
        local sectionLength=0
        local prevX,prevZ=nil,nil
        local seg=routeSegment
        for step=0,math.min(segmentCount()-1,100) do
          local idx=seg+step
          if state.loopRoute then idx=((idx-1)%segmentCount())+1
          elseif idx>segmentCount() then break end
          local a=state.nodes[idx]
          local b=segEnd(idx)
          if a and b then
            local dx,dz=b.x-a.x,b.z-a.z
            local length=math.sqrt(dx*dx+dz*dz)
            local tx,tz=normalize2(dx,dz)
            if prevX then
              local alignment=clamp(prevX*tx+prevZ*tz,-1,1)
              if math.acos(alignment)>0.12 and sectionLength>0 then break end
            end
            sectionLength=sectionLength+length
            prevX,prevZ=tx,tz
            if sectionLength>=300 then break end
          end
        end
        local boostKmh=40+160*clamp(sectionLength/300,0,1)
        local variation=0.90+0.20*(math.abs(math.sin(carIndex*12.9898+routeSegment*78.233))%1)
        boostKmh=clamp(boostKmh*variation,40,200)
        state.p[carIndex]={networkSpawnBoostTarget=boostKmh/3.6,
          networkSpawnBoostTimer=4.0,networkAttachTimer=0}
        state.spawnHints[carIndex]={segment=routeSegment,parameter=routeParameter}
        writeLog("SPAWN_CAR id=%d pos=(%.2f,%.2f,%.2f) dir=(%.3f,%.3f) route_seg=%d route_t=%.3f section=%.1f boost=%.1fkmh",
          carIndex,position.x,position.y,position.z,direction.x,direction.z,
          routeSegment,routeParameter,sectionLength,boostKmh)
      else failed=failed+1 end
    else failed=failed+1 end
  end
  state.spawnDone=true
  writeLog("SPAWN_FINISH moved=%d failed=%d gap=%.2f",moved,failed,gap)
  flushLog()
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
    w={rolloverT=0,gasStallT=0,lastPosition=vec3(car.position.x,car.position.y,car.position.z),progressPositionX=car.position.x,progressPositionZ=car.position.z,progressCheckT=0,cooldown=0}
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
    w.progressPositionX=car.position.x
    w.progressPositionZ=car.position.z
    w.progressCheckT=0
  end
  w.lastPosition=vec3(car.position.x,car.position.y,car.position.z)

  local offTrackT=safeNumber(p and p.offTrackT)
  local reason=nil
  if w.rolloverT>=10.0 then reason="rollover_10s"
  elseif offTrackT>=10.0 then reason="offtrack_10s"
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
    local sim=ac.getSim()
    if not sim then return end
    local player=ac.getCar(0)
    if (state.cspImportPending or not state.loaded) and not state.recLine and CSPConverter then
      state.autoImportAccum=state.autoImportAccum+dt
      if state.autoImportAccum>=5.0 then
        state.autoImportAccum=0
        loadLine(state.cspImportPending)
      end
    end
    if state.recLine and player then addPoint(player.position) end
    if state.recBounds and player then updateBoundsRecording(player) end
    if state.enabled and state.loaded and #state.nodes>=2 then
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

    local playerRoutePilotActive = state.enabled and #state.nodes>=2 and
      Driver and playerAI or false
    if playerRoutePilotActive and not state.playerAutopilotLast then
      writeLog("PLAYER_AUTOPILOT_ADOPT source=AC_NATIVE_CTRL8")
    elseif not playerRoutePilotActive and state.playerAutopilotLast then
      writeLog("PLAYER_AUTOPILOT_RELEASE")
    end
    state.playerAutopilotLast = playerRoutePilotActive == true

    -- Drive every active car which Assetto Corsa currently marks as AI,
    -- including car 0 only while the native Ctrl+8 autopilot is active.
    if state.enabled and #state.nodes>=2 and Driver and ensureDriver() then
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
      -- Resolve every car's lane and near-future traffic conflicts as one batch.
      -- The coordinator runs before controls are written, so requests cannot
      -- race against per-car update order.
      if Driver.prepareTraffic then
        local okTraffic,errTraffic=pcall(Driver.prepareTraffic,sim,driveOrder,dt)
        if not okTraffic then writeLog("TRAFFIC_COORDINATOR_ERROR err=%s",tostring(errTraffic)) end
      end
      -- The coordinator has already resolved all lane decisions. Update cars
      -- in deterministic route order; no per-car yield queue or pass handshake.
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
          local rescued=false
          if not state.trafficNetwork then rescued=updateRescueWatchdog(i,car,dt) end
          if not rescued then
            local okc,erc=pcall(Driver.update,i,car,dt)
            if not okc then
              writeLog("DRIVER_ERROR id=%d err=%s",i,tostring(erc))
              releaseCar(i)
            end
          end
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
-- Draw only a bounded window around the player. The previous implementation
-- submitted three debug lines for every route sample on every render frame,
-- even when most of a long route was hundreds of metres away.
local function routeOffsetPoint(index, offset)
  local n=#state.nodes
  local prevIndex=index-1
  local nextIndex=index+1
  if state.loopRoute then
    if prevIndex<1 then prevIndex=n end
    if nextIndex>n then nextIndex=1 end
  else
    if prevIndex<1 then prevIndex=index end
    if nextIndex>n then nextIndex=index end
  end
  local p=state.nodes[index]
  local a=state.nodes[prevIndex]
  local b=state.nodes[nextIndex]
  local lx,lz=normalize2(b.x-a.x,b.z-a.z)
  return vec3(p.x-lz*offset,p.y+0.05,p.z+lx*offset)
end

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
    if state.trafficNetwork then
      -- Render the source CSP lane network as-is; boundary geometry is omitted.
      for _,laneId in ipairs(state.trafficNetwork.laneOrder or {}) do
        local lane=state.trafficNetwork.lanes[laneId]
        local box=lane and lane.aabb
        local near=true
        if box then
          local dx=math.max(box.minX-player.position.x,0,player.position.x-box.maxX)
          local dz=math.max(box.minZ-player.position.z,0,player.position.z-box.maxZ)
          near=dx*dx+dz*dz<=650.0*650.0
        end
        local pts=near and lane and lane.points or {}
        if #pts>=2 then
          local anchor=pts[1]
          local previous=anchor
          local accumulated=0
          for k=2,#pts do
            local current=pts[k]
            local dx,dz=current.x-previous.x,current.z-previous.z
            accumulated=accumulated+math.sqrt(dx*dx+dz*dz)
            if accumulated>=6.0 or k==#pts then
              local mx=(anchor.x+current.x)*0.5-player.position.x
              local mz=(anchor.z+current.z)*0.5-player.position.z
              if mx*mx+mz*mz<=650.0*650.0 then
                render.debugLine(vec3(anchor.x,anchor.y+0.05,anchor.z),
                  vec3(current.x,current.y+0.05,current.z),YELLOW)
              end
              anchor=current
              accumulated=0
            end
            previous=current
          end
        end
      end
      return
    end
    local seg=select(1,findNearestSeg(player.position,1))
    local first,last
    local maxBack=isRec and CFG.PREVIEW_BACK or 90.0
    local maxAhead=isRec and CFG.PREVIEW_AHEAD or 360.0
    first,last=seg,seg
    local distance=0
    local idx=seg
    while distance<maxBack do
      if not state.loopRoute and idx<=1 then break end
      local prev=idx-1
      if prev<1 then prev=n end
      distance=distance+flatDist(state.nodes[prev],state.nodes[idx])
      idx=prev
      first=idx
      if idx==seg then break end
    end
    distance=0
    idx=seg
    while distance<maxAhead do
      if not state.loopRoute and idx>=n then break end
      local nxt=idx+1
      if nxt>n then nxt=1 end
      distance=distance+flatDist(state.nodes[idx],state.nodes[nxt])
      idx=nxt
      last=idx
      if idx==seg then break end
    end

    -- Keep a maximum spacing of roughly 6 m between submitted segments.
    -- Each boundary endpoint uses its own centred tangent, so neighbouring
    -- segments share exactly the same endpoint instead of opening gaps.
    local previousIndex=first
    local accumulated=0
    local i=first
    local steps=0
    local maxSteps=state.loopRoute and n or (last-first)
    if state.loopRoute and last<first then maxSteps=(n-first)+last end
    while i~=last and steps<=maxSteps do
      steps=steps+1
      local nextIndex=i+1
      if nextIndex>n then nextIndex=1 end
      accumulated=accumulated+flatDist(state.nodes[i],state.nodes[nextIndex])
      local shouldDraw=accumulated>=6.0 or nextIndex==last
      if shouldDraw then
        local a=state.nodes[previousIndex]
        local b=state.nodes[nextIndex]
        render.debugLine(vec3(a.x,a.y+0.05,a.z),vec3(b.x,b.y+0.05,b.z),YELLOW)
        if not state.loadedFromCSP then
          local al,ar=boundsAt(previousIndex)
          local bl,br=boundsAt(nextIndex)
          render.debugLine(routeOffsetPoint(previousIndex,al),routeOffsetPoint(nextIndex,bl),RED)
          render.debugLine(routeOffsetPoint(previousIndex,ar),routeOffsetPoint(nextIndex,br),RED)
        end
        previousIndex=nextIndex
        accumulated=0
      end
      i=nextIndex
    end
  end)
  if not ok then
    writeLog("DRAW3D_ERROR"); flushLog()
  end
end)

-- MAP: window-local coords (comfy_map pattern). Bigger scale, thick lines,
-- bounds with a minimum offset so they never merge with the AI line.
local mapBoundsCache=nil
local mapBoundsCacheNodes=nil
local mapBoundsCacheLeft=nil
local mapBoundsCacheRight=nil
local mapBoundsCacheCount=-1
local mapBoundsCacheLoop=nil
local mapBoundsCacheLastX=nil
local mapBoundsCacheLastZ=nil
local mapBoundsCacheNetwork=nil
local function computeMapBounds()
  local n=#state.nodes
  local last=n>0 and state.nodes[n] or nil
  local lastX=last and last.x or nil
  local lastZ=last and last.z or nil
  -- The route is static between imports/recorded samples. Re-scanning every
  -- point on every UI frame was needless O(N) work and caused FPS loss on
  -- CSP networks containing thousands of samples.
  if mapBoundsCache and mapBoundsCacheNodes==state.nodes and
      mapBoundsCacheLeft==state.boundsL and mapBoundsCacheRight==state.boundsR and
      mapBoundsCacheCount==n and mapBoundsCacheLoop==state.loopRoute and
      mapBoundsCacheLastX==lastX and mapBoundsCacheLastZ==lastZ and
       mapBoundsCacheNetwork==state.trafficNetwork then
    return mapBoundsCache
  end
  local minX,maxX,minZ,maxZ=math.huge,-math.huge,math.huge,-math.huge
  for i=1,n do
    local nd=state.nodes[i]
    if nd then
      local l,r=boundsAt(i)
      local b=segEnd(i)
      -- Open routes have no outgoing segment at the final vertex. Use the
      -- incoming tangent; if even that is unavailable, skip this vertex.
      if not b and i>1 then b=state.nodes[i-1] end
      if b then
        local dx,dz=b.x-nd.x,b.z-nd.z
        if i==n and not state.loopRoute then dx,dz=-dx,-dz end
        local lx,lz=normalize2(dx,dz)
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
    end
  end
  -- Include every authored CSP lane in the map extent; no synthetic boundaries.
  if state.trafficNetwork then
    for _,laneId in ipairs(state.trafficNetwork.laneOrder or {}) do
      local lane=state.trafficNetwork.lanes[laneId]
      for _,pt in ipairs(lane and lane.points or {}) do
        minX=math.min(minX,pt.x-2.0); maxX=math.max(maxX,pt.x+2.0)
        minZ=math.min(minZ,pt.z-2.0); maxZ=math.max(maxZ,pt.z+2.0)
      end
    end
  end
  if minX==math.huge then
    mapBoundsCache=nil
  else
    mapBoundsCache={minX=minX,maxX=maxX,minZ=minZ,maxZ=maxZ}
  end
  mapBoundsCacheNodes=state.nodes
  mapBoundsCacheLeft=state.boundsL
  mapBoundsCacheRight=state.boundsR
  mapBoundsCacheCount=n
  mapBoundsCacheLoop=state.loopRoute
  mapBoundsCacheLastX=lastX
  mapBoundsCacheLastZ=lastZ
  mapBoundsCacheNetwork=state.trafficNetwork
  return mapBoundsCache
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
  -- Cap map drawing work: the source can contain thousands of 3 m samples.
  -- A screen map does not benefit from drawing every sample on every UI frame.
  local stride=math.max(1,math.ceil(n/120))
  local lastIndex=state.loopRoute and n or math.max(1,n-1)
  if not state.loadedFromCSP then
  for pass=1,2 do
    local prev
    local lastDrawn=0
    local function drawBoundaryAt(i)
      local node=state.nodes[i]
      local l,r=boundsAt(i)
      local off=(pass==1) and l or r
      if math.abs(off)*scale<minOff then
        off=(off>=0 and 1 or -1)*(minOff/math.max(scale,0.0001))
      end
      local prevIndex=i-1
      local nextIndex=i+1
      if state.loopRoute then
        if prevIndex<1 then prevIndex=n end
        if nextIndex>n then nextIndex=1 end
      else
        if prevIndex<1 then prevIndex=i end
        if nextIndex>n then nextIndex=i end
      end
      local a,b=state.nodes[prevIndex],state.nodes[nextIndex]
      local lx,lz=normalize2(b.x-a.x,b.z-a.z)
      local px,pz=node.x-lz*off,node.z+lx*off
      return vec2(mx+(px-cx)*scale,my+(pz-cz)*scale)
    end
    for i=1,lastIndex,stride do
      local point=drawBoundaryAt(i)
      if prev and segVisible(prev.x,prev.y,point.x,point.y) then
        ui.drawLine(prev,point,rgbm(0.55,0.05,0.05,1),3)
      end
      prev=point
      lastDrawn=i
    end
    if lastDrawn~=lastIndex then
      local point=drawBoundaryAt(lastIndex)
      if prev and segVisible(prev.x,prev.y,point.x,point.y) then
        ui.drawLine(prev,point,rgbm(0.55,0.05,0.05,1),3)
      end
    end
  end
  end
  -- Draw every authored CSP traffic line. No boundaries and no merged centreline.
  if state.trafficNetwork then
    local laneIds=state.trafficNetwork.laneOrder or {}
    local perLaneBudget=math.max(2,math.floor(1800/math.max(1,#laneIds)))
    for _,laneId in ipairs(laneIds) do
      local lane=state.trafficNetwork.lanes[laneId]
      local points=lane and lane.points or {}
      local count=#points
      local step=math.max(1,math.ceil(count/perLaneBudget))
      local prev
      for k=1,count,step do
        local node=points[k]
        local point=vec2(mx+(node.x-cx)*scale,my+(node.z-cz)*scale)
        if prev and segVisible(prev.x,prev.y,point.x,point.y) then
          ui.drawLine(prev,point,rgbm(1.0,0.78,0.12,0.90),2)
        end
        prev=point
      end
      if count>1 then
        local node=points[count]
        local point=vec2(mx+(node.x-cx)*scale,my+(node.z-cz)*scale)
        if prev and segVisible(prev.x,prev.y,point.x,point.y) then
          ui.drawLine(prev,point,rgbm(1.0,0.78,0.12,0.90),2)
        end
      end
    end
  else
    -- Manual RoutePilot route fallback.
    local prev=vec2(mx+(state.nodes[1].x-cx)*scale,my+(state.nodes[1].z-cz)*scale)
    for i=1+stride,lastIndex,stride do
      local node=state.nodes[i]
      local point=vec2(mx+(node.x-cx)*scale,my+(node.z-cz)*scale)
      if segVisible(prev.x,prev.y,point.x,point.y) then
        ui.drawLine(prev,point,rgbm(1.0,0.85,0.0,0.95),3)
      end
      prev=point
    end
    local finalNode=state.nodes[lastIndex]
    local finalPoint=vec2(mx+(finalNode.x-cx)*scale,my+(finalNode.z-cz)*scale)
    if lastIndex>1 and (prev.x~=finalPoint.x or prev.y~=finalPoint.y) and
        segVisible(prev.x,prev.y,finalPoint.x,finalPoint.y) then
      ui.drawLine(prev,finalPoint,rgbm(1.0,0.85,0.0,0.95),3)
    end
    if state.loopRoute then
      local first=vec2(mx+(state.nodes[1].x-cx)*scale,my+(state.nodes[1].z-cz)*scale)
      if segVisible(finalPoint.x,finalPoint.y,first.x,first.y) then
        ui.drawLine(finalPoint,first,rgbm(1.0,0.85,0.0,0.95),3)
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
    if state.trafficNetwork then
      local ns=state.trafficNetwork.stats or {}
      ui.text(string.format("CSP: %d линий | %d переходов | %d направленных перестроений | %s",
        safeNumber(ns.validLanes),safeNumber(ns.junctionTransitions),
        safeNumber(ns.laneChanges),state.status))
    else
      ui.text(string.format("Точек: %d | %s",#state.nodes,state.status))
    end
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
        state.nodes={}; state.p={}; state.trafficNetwork=nil; state.playerTelemetry=nil; state.routeGeometry=nil; state.boundsL={}; state.boundsR={}
        state.loaded=false; state.loadedFromCSP=false; state.recBounds=false
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
      state.nodes={}; state.p={}; state.trafficNetwork=nil; state.boundsL={}; state.boundsR={}
      state.status="Очищено"; state.loaded=false; state.loadedFromCSP=false
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
writeLog("ROUTEPILOT 0.10.0 INITIALIZED track=%s points=%d csp=%s",
  getTrackTag(),#state.nodes,tostring(state.loadedFromCSP==true))
flushLog()