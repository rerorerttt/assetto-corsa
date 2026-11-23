-- RoutePilot.lua — v0.9.0 (CSP 0.3.2-preview2)
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
local okBuilder, Builder = pcall(require, "spline_builder")
if not okBuilder or type(Builder) ~= "table" or not Builder.build then Builder = nil end

local CFG = {
  LOOKAHEAD_BASE=14.0, LOOKAHEAD_SPEED=0.8, LOOKAHEAD_MIN=14.0, LOOKAHEAD_MAX=50.0,
  STEER_GAIN=0.72, STEER_LIMIT=0.75, STEER_RATE=2.0, STEER_FILTER=0.18,
  MAX_LATERAL_ACCEL=9.0, BRAKE_ACCEL=8.5, PLAN_HORIZON=170.0, MIN_APEX_SPEED=11.0,
  CURVE_MIN=0.002, GAS_GAIN=0.45, BRAKE_GAIN=0.60, MAX_GAS=1.0, MAX_BRAKE=1.0,
  SLIP_START=0.09, SLIP_FULL=0.45, SLIP_DAMP_MIN_SPEED=8.0,
  SLIP_DAMP_MIN_ANGLE=4.0*math.pi/180,
  SPAWN_GAP=25.0, SPAWN_MIN_GAP=18.0, SPAWN_STABLE_TIME=0.35, SPAWN_WAIT_TIMEOUT=8.0,
  LOG_INTERVAL=1.0, MAX_ROUTE_POINTS=50000, RECORD_DISTANCE=3.0,
  PREVIEW_AHEAD=30.0, PREVIEW_BACK=8.0, DRAW_RANGE=600.0,
  BOUNDS_MIN_PASS=3.0, SAFE_RESPAWN_CLEAR=8.0,
  MAP_TOP=175.0, MAP_PAD=6.0,
}
local WIDTH_PRESETS = {3, 7, 11, 15}

local state = {
  enabled=true, recLine=false, recBounds=false, loopRoute=true,
  speedMax=300, followGap=8.0, widthPreset=2,
  boundsL={}, boundsR={}, recBoundsSeg=0, showWorldLines=false,
  spawnDone=false, spawnPerformed=false, spawnWait=0, spawnStable=0, spawnLastRoster="",
  nodes={}, p={}, status="Готов", logAccum=0, loaded=false,
  aiSplineBuilt=false, aiSplinePath=nil, aiRevision=0, sessionTime=0, trafficHints={},
}
local driverReady=false
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
local function buildAISpline()
  if not Builder then
    state.aiSplineBuilt=false
    state.status="Не загружен модуль native AI spline"
    writeLog("AI_SPLINE_BUILD_FAILED reason=builder_module_missing")
    return false
  end
  if #state.nodes<3 then
    state.aiSplineBuilt=false
    state.status="Для AI-линии нужны минимум 3 точки"
    return false
  end
  local stamp = (os and os.time and os.time()) or math.floor(os.clock()*1000)
  state.aiRevision=state.aiRevision+1
  local path=string.gsub(getLineFilePath(),"%.txt$",
    "_native_"..tostring(stamp).."_"..tostring(state.aiRevision)..".ai")
  local ok,info=Builder.build(state.nodes,state.boundsL,state.boundsR,path,state.speedMax,state.loopRoute)
  if not ok then
    state.aiSplineBuilt=false
    state.aiSplinePath=nil
    state.status="Ошибка построения native AI spline"
    writeLog("AI_SPLINE_BUILD_FAILED path=%s error=%s",path,tostring(info))
    flushLog()
    return false
  end
  if driverReady and Driver and Driver.invalidateAll then
    pcall(Driver.invalidateAll)
  else
    state.p={}
    state.trafficHints={}
  end
  state.aiSplinePath=path
  state.aiSplineBuilt=true
  state.lanesAssigned=false
  state.loaded=true
  state.spawnDone=false
  state.spawnPerformed=false
  state.spawnWait=0
  state.spawnStable=0
  state.spawnLastRoster=""
  state.status=string.format("Native AI готов: %d точек, %.0f м",#state.nodes,info or 0)
  writeLog("AI_SPLINE_BUILD_OK path=%s points=%d length=%.2f speedMax=%.1f widthPreset=%d",
    path,#state.nodes,info or 0,state.speedMax,state.widthPreset)
  flushLog()
  return true
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
          local avgWidth=0
          for i=1,n do avgWidth=avgWidth+(R[i]-L[i]) end
          avgWidth=avgWidth/n
          local best,bestError=1,math.huge
          for i=1,#WIDTH_PRESETS do
            local e=math.abs(WIDTH_PRESETS[i]-avgWidth)
            if e<bestError then best,bestError=i,e end
          end
          state.widthPreset=best
          writeLog("BOUNDS_LOADED points=%d avgWidth=%.2f preset=%d",n,avgWidth,best)
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
    state.loaded=true
    writeLog("SAVE_ROUTE points=%d",#state.nodes)
    buildAISpline()
    flushLog()
  else
    state.loaded=false
    state.aiSplineBuilt=false
    state.status="Ошибка сохранения"
  end
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
    state.nodes=loaded; state.p={}; state.loaded=true
    if not state.spawnPerformed then
      state.spawnDone=false; state.spawnWait=0; state.spawnStable=0; state.spawnLastRoster=""
    end
    state.status=string.format("Загружено: %d точек",#loaded)
    writeLog("LOAD_ROUTE points=%d track=%s",#loaded,getTrackTag())
    loadBounds()
    buildAISpline()
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
        return vec3(lerp(a.x,b.x,nt),lerp(a.y,b.y,nt),lerp(a.z,b.z,nt))
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
local function findNearestSeg(pos,hintSeg)
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
  if bestDist>15*15 then for i=1,segs do check(i) end end
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
  for i=1,sim.carsCount-1 do
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
local function spotIsClear(pos, exceptIndex, ignored)
  local sim=ac.getSim()
  if not sim then return true end
  local clearSq=CFG.SAFE_RESPAWN_CLEAR*CFG.SAFE_RESPAWN_CLEAR
  for i=0,sim.carsCount-1 do
    if i~=exceptIndex and not (ignored and ignored[i]) then
      local other=ac.getCar(i)
      if other and other.isActive then
        local dx=other.position.x-pos.x
        local dz=other.position.z-pos.z
        if dx*dx+dz*dz < clearSq then return false end
      end
    end
  end
  return true
end

-- Spawn only after every AI has accepted the generated native spline.
local function trySpawnTeleport(sim,dt)
  if state.spawnDone then return end
  if sim.customAISplinesAllowed==false then
    state.status="CSP запретил custom AI spline в этой сессии"
    return
  end
  if not state.loaded or not state.aiSplineBuilt or not state.aiSplinePath or
     state.recLine or #state.nodes<3 then return end

  state.spawnWait=state.spawnWait+dt
  local aiCars={}
  local ignored={}
  for i=1,sim.carsCount-1 do
    local car=ac.getCar(i)
    if car and car.isActive and car.isAIControlled then
      aiCars[#aiCars+1]=i
      ignored[i]=true
    end
  end
  local roster=table.concat(aiCars,",")
  if #aiCars==0 then
    if state.spawnWait>=CFG.SPAWN_WAIT_TIMEOUT then state.spawnDone=true end
    return
  end
  if roster~=state.spawnLastRoster then
    state.spawnLastRoster=roster
    state.spawnStable=0
    return
  end
  state.spawnStable=state.spawnStable+dt
  if state.spawnStable<CFG.SPAWN_STABLE_TIME then return end

  if not Driver or not Driver.isSplineReady then
    state.status="Native AI driver недоступен"
    return
  end
  for _,carIndex in ipairs(aiCars) do
    if not Driver.isSplineReady(carIndex) then
      if state.spawnWait>CFG.SPAWN_WAIT_TIMEOUT then
        state.status="CSP не подтвердил загрузку AI spline; смотри лог"
        if not state.splineTimeoutLogged then
          state.splineTimeoutLogged=true
          writeLog("SPAWN_WAIT_TIMEOUT car=%d path=%s",carIndex,tostring(state.aiSplinePath))
          flushLog()
        end
      else
        state.status="Загрузка native AI spline..."
      end
      return
    end
  end
  if type(physics)~="table" or type(physics.setAICarPosition)~="function" then
    state.status="CSP: physics.setAICarPosition недоступна"
    state.spawnDone=true
    writeLog("SPAWN_ABORT reason=setAICarPosition_unavailable")
    flushLog()
    return
  end

  local metrics=buildSpawnRouteMetrics()
  if not metrics then
    state.status="Ошибка: линия слишком короткая"
    state.spawnDone=true
    writeLog("SPAWN_ABORT reason=invalid_route")
    flushLog()
    return
  end
  local count=#aiCars
  local gap=math.min(CFG.SPAWN_GAP,metrics.length/(count+1))
  if gap<CFG.SPAWN_MIN_GAP then
    state.status="Недостаточно длины линии для безопасной расстановки"
    state.spawnDone=true
    writeLog("SPAWN_ABORT reason=route_too_short cars=%d route=%.1f gap=%.1f",count,metrics.length,gap)
    flushLog()
    return
  end

  local startDistance=gap
  local player=ac.getCar(0)
  if metrics.loop and player then
    local playerDistance,lineError=findRouteDistanceAtPosition(player.position,metrics)
    startDistance=playerDistance+gap
    writeLog("SPAWN_ANCHOR player_line_error=%.2f route_distance=%.2f",lineError,playerDistance)
  end

  local moved,failed=0,0
  local planned={}
  writeLog("SPAWN_BEGIN cars=%d route=%.1f gap=%.1f start=%.1f nativeSpline=%s",
    count,metrics.length,gap,startDistance,tostring(state.aiSplinePath))

  for order=1,count do
    local carIndex=aiCars[order]
    local routeDistance=startDistance+(order-1)*gap
    local position=sampleSpawnRoute(metrics,routeDistance)
    local plannedClear=true
    if position then
      for j=1,#planned do
        if flatDist(position,planned[j])<CFG.SAFE_RESPAWN_CLEAR*2 then
          plannedClear=false
          break
        end
      end
    end
    if position and plannedClear and spotIsClear(position,carIndex,ignored) then
      local ok,result=pcall(physics.setAICarPosition,carIndex,position,nil)
      if ok and result~=false then
        moved=moved+1
        planned[#planned+1]=position
        if Driver.resetAfterTeleport then Driver.resetAfterTeleport(carIndex) end
        writeLog("SPAWN_CAR id=%d order=%d routeDistance=%.1f pos=(%.2f,%.2f,%.2f) direction=AI_SPLINE",
          carIndex,order,routeDistance,position.x,position.y,position.z)
      else
        failed=failed+1
        writeLog("SPAWN_CAR_FAILED id=%d order=%d error=%s",carIndex,order,tostring(result))
      end
    else
      failed=failed+1
      writeLog("SPAWN_CAR_FAILED id=%d order=%d reason=%s",
        carIndex,order,position and (plannedClear and "occupied" or "planned_overlap") or "invalid_sample")
    end
  end

  state.spawnDone=true
  state.spawnPerformed=true
  state.status=string.format("Native AI: расставлено %d/%d, интервал %.0f м",moved,count,gap)
  writeLog("SPAWN_FINISH moved=%d failed=%d total=%d gap=%.2f",moved,failed,count,gap)
  flushLog()
end

local function teleportHome(carIndex,car)
  if type(physics)~="table" or type(physics.setAICarPosition)~="function" then return false end
  local metrics=buildSpawnRouteMetrics()
  if not metrics then return false end
  local startDistance=findRouteDistanceAtPosition(car.position,metrics)
  -- Never return a stuck car to its current node: search forward for a
  -- genuinely empty section of the recorded route, then align to the AI spline.
  for offset=18,metrics.length,12 do
    local pos=sampleSpawnRoute(metrics,startDistance+offset)
    if pos and spotIsClear(pos,carIndex) then
      local ok,result=pcall(physics.setAICarPosition,carIndex,pos,nil)
      if ok and result~=false then
        if Driver and Driver.resetAfterTeleport then Driver.resetAfterTeleport(carIndex) end
        writeLog("RESCUE id=%d offset=%.1f pos=(%.2f,%.2f,%.2f) direction=AI_SPLINE",
          carIndex,offset,pos.x,pos.y,pos.z)
        flushLog()
        return true
      end
    end
  end
  writeLog("RESCUE_FAILED id=%d reason=no_clear_route_position",carIndex)
  flushLog()
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
  -- Move only the boundary on the side where the player is driving. Never
  -- expand either edge beyond the previously recorded corridor.
  if off>=0 then
    local minimumRight=math.min(1.5,r)
    newR=math.min(r,math.max(off+0.5,minimumRight))
    newL=l
  else
    local maximumLeft=math.max(l,-1.5)
    newL=math.max(l,math.min(off-0.5,maximumLeft))
    newR=r
  end
  if newL>off or newR<off then
    state.status="Запись границ остановлена: машина вышла за текущий коридор"
    state.recBounds=false
    saveBounds()
    buildAISpline()
    writeLog("BOUNDS_REC_STOP seg=%d off=%.2f old=[%.2f,%.2f] new=[%.2f,%.2f]",
      seg,off,l,r,newL,newR)
    flushLog()
    return
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
  state.widthPreset=clamp(math.floor(preset),1,4)
  local n=#state.nodes
  if n<2 then return end
  local half=WIDTH_PRESETS[state.widthPreset]/2
  for i=1,n do
    state.boundsL[i]=-half
    state.boundsR[i]=half
  end
  saveBounds()
  writeLog("WIDTH_PRESET preset=%d width=%dm",state.widthPreset,WIDTH_PRESETS[state.widthPreset])
  buildAISpline()
end
local function releaseCar(carIndex)
  if Driver and Driver.release then
    pcall(Driver.release,carIndex)
  else
    state.p[carIndex]=nil
  end
end
local function ensureDriver()
  if driverReady or not Driver then return Driver~=nil end
  if type(Driver.setup)~="function" then return false end
  Driver.setup(CFG,state,{
    segmentCount=segmentCount, segEnd=segEnd, findNearestSeg=findNearestSeg,
    lateralOf=lateralOf, boundsAt=boundsAt, teleportHome=teleportHome, log=writeLog,
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
    state.sessionTime=state.sessionTime+dt
    local player=ac.getCar(0)
    if state.recLine and player then addPoint(player.position) end
    if state.recBounds and player then updateBoundsRecording(player) end

    if state.enabled and state.loaded and state.aiSplineBuilt and
      #state.nodes>=3 and Driver and ensureDriver() then
      -- Prepare the complete field first: all cars see the same positions,
      -- progress values and pass reservations before any offset is applied.
      if Driver.beginFrame then Driver.beginFrame(sim,dt) end
      trySpawnTeleport(sim,dt)
      for i=1,sim.carsCount-1 do
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
-- Boundary vertices use the averaged tangent at each route node. This keeps
-- the two edges connected when their width changes from point to point.
local function boundaryPoint(index, offset, partial)
  local n=#state.nodes
  local prevIndex,nextIndex
  if partial then
    prevIndex=math.max(1,index-1)
    nextIndex=math.min(n,index+1)
  else
    prevIndex=(index-2)%n+1
    nextIndex=index%n+1
  end
  local prev,cur,nxt=state.nodes[prevIndex],state.nodes[index],state.nodes[nextIndex]
  local tx,tz=normalize2(nxt.x-prev.x,nxt.z-prev.z)
  return vec3(cur.x-tz*offset,cur.y+0.06,cur.z+tx*offset)
end

-- NFS-style corner guidance: a short wall of chevrons spans the road and
-- follows the local route tangent. Only the upcoming section is rendered.
local function drawCornerArrows(startSeg,partial,maxDistance)
  local n=#state.nodes
  local count=segmentCount()
  if n<3 or count<3 then return end
  local seg=startSeg
  local travelled,nextArrow=0,0
  for _=1,count do
    if partial and seg>=n then break end
    local prevIndex=partial and math.max(1,seg-1) or (seg-2)%n+1
    local nextIndex=seg%n+1
    local prev,cur,nxt=state.nodes[prevIndex],state.nodes[seg],state.nodes[nextIndex]
    local ax,az=cur.x-prev.x,cur.z-prev.z
    local bx,bz=nxt.x-cur.x,nxt.z-cur.z
    local la,lb=math.sqrt(ax*ax+az*az),math.sqrt(bx*bx+bz*bz)
    local turn=0
    if la>0.05 and lb>0.05 then
      local dot=clamp((ax*bx+az*bz)/(la*lb),-1,1)
      turn=math.acos(dot)
    end
    if turn>0.06 and travelled>=nextArrow then
      local tx,tz=normalize2(bx,bz)
      local rx,rz=-tz,tx
      local left,right=boundsAt(seg)
      local offset=left+0.8
      while offset<=right-0.8 do
        local center=vec3(cur.x+rx*offset,cur.y+1.25,cur.z+rz*offset)
        local tip=vec3(center.x+tx*1.25,center.y,center.z+tz*1.25)
        local back=vec3(center.x-tx*0.75,center.y,center.z-tz*0.75)
        local wingL=vec3(tip.x-tx*0.75+rx*0.65,tip.y,tip.z-tz*0.75+rz*0.65)
        local wingR=vec3(tip.x-tx*0.75-rx*0.65,tip.y,tip.z-tz*0.75-rz*0.65)
        local color=rgbm(0.05,0.65,1.0,0.95)
        render.debugLine(back,tip,color)
        render.debugLine(tip,wingL,color)
        render.debugLine(tip,wingR,color)
        offset=offset+2.4
      end
      nextArrow=travelled+6.0
    end
    local d=flatDist(cur,nxt)
    travelled=travelled+d
    if travelled>=maxDistance then break end
    seg=nextIndex
  end
end

render.on('main.track.transparent', function()
  local n=#state.nodes
  if n<2 then return end
  local isRec=state.recLine or state.recBounds
  if not (state.showWorldLines or isRec) then return end
  local ok,err=pcall(function()
    local player=ac.getCar(0)
    if not player then return end
    render.setDepthMode(render.DepthMode.Off)
    local YELLOW=rgbm(1.0,0.85,0.0,1)
    local RED=rgbm(0.6,0.05,0.05,1)
    local partial=state.recLine
    local seg=select(1,findNearestSeg(player.position,1))
    local fromI,toI=1,n

    if isRec then
      fromI,toI=seg,seg
      local dist=0
      local i=seg
      while dist<CFG.PREVIEW_AHEAD do
        if partial and i>=n then break end
        local nextIndex=i%n+1
        dist=dist+flatDist(state.nodes[i],state.nodes[nextIndex])
        i=nextIndex
        toI=i
        if i==seg then break end
      end
      local j=seg
      local back=0
      while back<CFG.PREVIEW_BACK do
        if partial and j<=1 then break end
        local prevIndex=(j-2)%n+1
        back=back+flatDist(state.nodes[prevIndex],state.nodes[j])
        j=prevIndex
        fromI=j
        if j==seg then break end
      end
    end

    local count=0
    local i2=fromI
    while count<n do
      count=count+1
      local nextIndex
      if i2<n then
        nextIndex=i2+1
      elseif state.loopRoute and not partial then
        nextIndex=1
      else
        break
      end
      local node,nx=state.nodes[i2],state.nodes[nextIndex]
      render.debugLine(vec3(node.x,node.y+0.05,node.z),vec3(nx.x,nx.y+0.05,nx.z),YELLOW)
      local left,right=boundsAt(i2)
      local nextLeft,nextRight=boundsAt(nextIndex)
      local a1=boundaryPoint(i2,left,partial)
      local a2=boundaryPoint(nextIndex,nextLeft,partial)
      local b1=boundaryPoint(i2,right,partial)
      local b2=boundaryPoint(nextIndex,nextRight,partial)
      render.debugLine(a1,a2,RED)
      render.debugLine(b1,b2,RED)
      if i2==toI then break end
      i2=nextIndex
    end
    drawCornerArrows(seg,partial,isRec and 35.0 or 120.0)
  end)
  if not ok then
    writeLog("DRAW3D_ERROR err=%s",tostring(err))
    flushLog()
  end
end)

-- MAP: drawn directly in the main app window (not a detached map window).
-- The map uses the same real boundary vertices as the 3D overlay; no artificial
-- pixel offsets are applied, so displayed road width matches the saved limits.
local function computeMapBounds()
  local minX,maxX,minZ,maxZ=math.huge,-math.huge,math.huge,-math.huge
  local n=#state.nodes
  for i=1,n do
    local left,right=boundsAt(i)
    local lp=boundaryPoint(i,left,state.recLine)
    local rp=boundaryPoint(i,right,state.recLine)
    local pts={lp,rp,state.nodes[i]}
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
  local function toScreen(p)
    return vec2(mx+(p.x-cx)*scale,my+(p.z-cz)*scale)
  end
  local function segVisible(a,bp)
    return not (a.x<q0.x and bp.x<q0.x) and not (a.x>q1.x and bp.x>q1.x)
      and not (a.y<q0.y and bp.y<q0.y) and not (a.y>q1.y and bp.y>q1.y)
  end

  local n=#state.nodes
  for pass=1,2 do
    local color=rgbm(0.55,0.05,0.05,1)
    for i=1,n-1 do
      local left,right=boundsAt(i)
      local off=pass==1 and left or right
      local a=toScreen(boundaryPoint(i,off,state.recLine))
      local nextOff=pass==1 and select(1,boundsAt(i+1)) or select(2,boundsAt(i+1))
      local bp=toScreen(boundaryPoint(i+1,nextOff,state.recLine))
      if segVisible(a,bp) then ui.drawLine(a,bp,color,3) end
    end
    if state.loopRoute and not state.recLine and not state.recBounds then
      local left,right=boundsAt(n)
      local off=pass==1 and left or right
      local a=toScreen(boundaryPoint(n,off,false))
      local firstOff=pass==1 and select(1,boundsAt(1)) or select(2,boundsAt(1))
      local bp=toScreen(boundaryPoint(1,firstOff,false))
      if segVisible(a,bp) then ui.drawLine(a,bp,color,3) end
    end
  end

  for i=1,n-1 do
    local a=toScreen(state.nodes[i])
    local bp=toScreen(state.nodes[i+1])
    if segVisible(a,bp) then ui.drawLine(a,bp,rgbm(1.0,0.85,0.0,0.95),3) end
  end
  if state.loopRoute and not state.recLine and not state.recBounds then
    local a=toScreen(state.nodes[n])
    local bp=toScreen(state.nodes[1])
    if segVisible(a,bp) then ui.drawLine(a,bp,rgbm(1.0,0.85,0.0,0.95),3) end
  end

  local sim=ac.getSim()
  if sim then
    for i=0,sim.carsCount-1 do
      local car=ac.getCar(i)
      if car and car.isActive then
        local pnt=toScreen(car.position)
        if pnt.x>q0.x and pnt.x<q1.x and pnt.y>q0.y and pnt.y<q1.y then
          if i==0 then
            ui.drawCircleFilled(pnt,5,rgbm(0.0,0.8,1.0,1.0),12)
          else
            local p=state.p[i]
            if p and p.passState and p.passState~="RETURN" then
              ui.drawCircleFilled(pnt,4,rgbm(0.2,1.0,0.4,1.0),10)
            else
              ui.drawCircleFilled(pnt,4,rgbm(1,1,1,0.95),10)
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
      local label=string.format("%s%d %s",idx==state.widthPreset and "● " or "",idx,idx==1 and "машина" or "машины")
      if idx>1 then ui.sameLine(0,4) end
      if ui.button(label,vec2(92,22)) then
        applyWidthPreset(idx)
      end
    end
    if ui.button(state.recLine and "СТОП запись линии" or "Запись линии",vec2(-0.5,24)) then
      state.recLine=not state.recLine
      if state.recLine then
        if driverReady and Driver and Driver.invalidateAll then pcall(Driver.invalidateAll) end
        state.nodes={}; state.p={}; state.boundsL={}; state.boundsR={}
        state.loaded=false; state.aiSplineBuilt=false; state.aiSplinePath=nil
        state.spawnDone=true; state.recBounds=false
        state.status="Запись линии и границ..."
      else
        state.status="Записано точек: "..#state.nodes
      end
    end
    ui.sameLine(0,4)
    if ui.button(state.recBounds and "СТОП запись границ" or "Запись границ",vec2(-0.5,24)) then
      if state.recBounds then
        state.recBounds=false
        saveBounds()
        buildAISpline()
        state.status="Границы записаны; native AI обновлён"
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
      buildAISpline()
    end
    if ui.button("Сохранить",vec2(-0.25,22)) then saveLine() end
    ui.sameLine(0,4)
    if ui.button("Загрузить",vec2(-0.25,22)) then loadLine() end
    ui.sameLine(0,4)
    if ui.button("Очистить",vec2(-0.25,22)) then
      if driverReady and Driver and Driver.invalidateAll then pcall(Driver.invalidateAll) end
      state.nodes={}; state.p={}; state.boundsL={}; state.boundsR={}
      state.aiSplineBuilt=false; state.aiSplinePath=nil
      state.spawnDone=true; state.loaded=false
      state.status="Очищено"
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
    if Driver and Driver.invalidateAll and driverReady then pcall(Driver.invalidateAll) end
    if logger and logger.shutdown then
      pcall(logger.shutdown,logger,"RoutePilot released")
    end
  end)
end
loadLine()
writeLog("ROUTEPILOT 1.0.0 NATIVE AI INITIALIZED track=%s points=%d aiSpline=%s",
  getTrackTag(),#state.nodes,tostring(state.aiSplineBuilt))
flushLog()