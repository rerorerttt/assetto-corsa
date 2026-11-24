-- RoutePilot/driver.lua — v0.13.1 LANED RACING, HOTFIX (CSP 0.3.2-preview2)
-- THE ATTACHED TRACE: "driver.lua:138: attempt to call global 'lerp' (a nil
-- value)" repeated forever with CAR_INIT loops — cars were never driven.
-- v0.13.0 declared the helper lerp BELOW its use in railPoint; in Lua that
-- compiles to a call to a nil GLOBAL. Two order bugs fixed:
--   1) lerp moved to the top helpers block (before any use);
--   2) laneClear referenced p.halfW without p as a parameter (next latent
--      nil-error) — it now derives the own half-width via halfDims(car).
-- Everything else from v0.13.0 is unchanged: corner-cutting fix (rail
-- clamped into the bounds of the SEGMENT THE TARGET LANDS ON, lookahead
-- shrinks before corners, speed-scaled edge zone), checkers-style adjacent
-- lane changes with real car dimensions, no immediate return.

local Driver = {}
local CFG, S, H

local WHEELBASE, STEER_LOCK = 2.6, 0.60
local LOOKAHEAD_TIME, LOOK_MIN, LOOK_MAX = 0.55, 6, 35
local CORNER_LOOK_GAP = 60.0
local BOUND_MARGIN  = 0.35
local TRIGGER_TIME  = 0.3
local RESCUE_SPEED, RESCUE_TIME = 1.5, 5.0
local RESCUE_COOLDOWN = 10.0
local STEER_DEAD = 0.004
local DEFAULT_HALF_W, DEFAULT_HALF_L = 0.95, 2.2

local function clamp(x,a,b) return math.max(a, math.min(b,x)) end
local function lerp(a,b,t) return a+(b-a)*t end
local function norm(x,z)
  local n=math.sqrt(x*x+z*z)
  if n<1e-6 then return 0,1 end
  return x/n,z/n
end
local function angle(ax,az,bx,bz)
  return math.atan2(ax*bz-az*bx, clamp(ax*bx+az*bz,-1,1))
end
local function num(x,d)
  if type(x)~='number' or x~=x or x==math.huge or x==-math.huge then return d or 0 end
  return x
end
local function rnd(i,s)
  local x=math.sin(i*127.1+s*311.7)*43758.5453
  return x-math.floor(x)
end

function Driver.setup(cfg,state,helpers) CFG,S,H=cfg,state,helpers end

-- real car half-dimensions (aabbSize guarded)
local function halfDims(car)
  local ok, sz = pcall(function() return car.aabbSize end)
  if ok and sz then
    local okX,x = pcall(function() return sz.x end)
    local okZ,z = pcall(function() return sz.z end)
    if okX and okZ and type(x)=='number' and type(z)=='number' then
      return clamp(math.abs(x),0.5,2.5), clamp(math.abs(z),1.0,6.0)
    end
  end
  return DEFAULT_HALF_W, DEFAULT_HALF_L
end

-- rail centers for the preset lane count, inside the SAVED bounds at seg
local function lanes(seg)
  local count=clamp(math.floor(num(S.widthPreset,2)),1,4)
  local left,right=H.boundsAt(seg)
  local lo,hi=left+BOUND_MARGIN,right-BOUND_MARGIN
  if hi<=lo then return {0} end
  if count==1 then return {(lo+hi)*0.5} end
  local step=(hi-lo)/count
  local c={}
  for i=1,count do c[i]=lo+step*(i-0.5) end
  return c
end

-- dynamic safe gap: grows with speed (no rear-ending at 200 km/h)
local function minGap(speed)
  return 5.0 + 0.45*speed
end

local function newDriver(i)
  local pace=0.88+rnd(i,1)*0.22
  local p={segIdx=1,pace=pace,
    cruise=num(S.speedMax,210)/3.6*pace,
    homeLane=1,lane=1,passSide=0,passTime=0,blocked=0,
    steer=0,gas=0,brake=0,stuck=0,retire=0,
    lineDist=0,off=0,target=0,
    halfW=DEFAULT_HALF_W,halfL=DEFAULT_HALF_L}
  local c=lanes(1)
  if #c>1 then
    p.homeLane=clamp(1+math.floor((1.10-pace)/0.22*(#c-1)+0.5),1,#c)
    p.lane=p.homeLane
  end
  S.p[i]=p
  H.log('CAR_INIT id=%d pace=%.2f cruise=%.0fkmh lane=%d/%d',
    i,pace,p.cruise*3.6,p.homeLane,#c)
  return p
end

-- WALK-AHEAD RAIL POINT: the point 'distance' meters ahead, shifted onto
-- the rail, with the rail offset CLAMPED INTO THE BOUNDS OF THE SEGMENT
-- THE POINT LANDS ON (the corner-cutting fix).
local function railPoint(seg,t,distance,lat)
  local segs=H.segmentCount()
  local idx=seg
  local remaining=distance
  local tCur=t
  for _=1,seg+segs do
    local a=S.nodes[idx]
    local b=H.segEnd(idx)
    local len=math.sqrt((b.x-a.x)^2+(b.z-a.z)^2)
    if len<1e-4 then
      if S.loopRoute then idx=(idx%segs)+1 else break end
    else
      local room=len*(1-tCur)
      if remaining<=room then
        local bl,br=H.boundsAt(idx)
        local l=clamp(lat,bl+BOUND_MARGIN,br-BOUND_MARGIN)
        local nt=(len*tCur+remaining)/len
        local px=lerp(a.x,b.x,nt)
        local pz=lerp(a.z,b.z,nt)
        local tx,tz=norm(b.x-a.x,b.z-a.z)
        return px-tz*l, pz+tx*l
      end
      remaining=remaining-room
      tCur=0
      if S.loopRoute then idx=(idx%segs)+1
      elseif idx<segs then idx=idx+1
      else break end
    end
  end
  local a=S.nodes[seg]
  local b=H.segEnd(seg)
  local bl,br=H.boundsAt(seg)
  local l=clamp(lat,bl+BOUND_MARGIN,br-BOUND_MARGIN)
  local tx,tz=norm(b.x-a.x,b.z-a.z)
  return b.x-tz*l, b.z+tx*l
end

-- lookahead capped by an upcoming corner: shrinks BEFORE the corner,
-- opens back up after it
local function cornerCappedLookahead(seg,t,speed)
  local ld=clamp(LOOKAHEAD_TIME*math.max(speed,4),LOOK_MIN,LOOK_MAX)
  local segs=H.segmentCount()
  local idx=seg
  local distance=0
  local steps=0
  while distance<CORNER_LOOK_GAP and steps<200 do
    steps=steps+1
    local a=S.nodes[idx]
    local b=H.segEnd(idx)
    local len=math.sqrt((b.x-a.x)^2+(b.z-a.z)^2)
    if len<1e-4 then
      if S.loopRoute then idx=(idx%segs)+1 else break end
    else
      local curv=H.getVertexCurvature(idx)
      if curv>CFG.CURVE_MIN then
        local maxLd=math.max(LOOK_MIN,distance*0.8)
        if maxLd<ld then ld=maxLd end
      end
      distance=distance+len
      if S.loopRoute then idx=(idx%segs)+1
      elseif idx<segs then idx=idx+1
      else break end
      if S.loopRoute and idx==seg then break end
    end
  end
  return ld
end

local function purePursuit(car,p,speed)
  local seg,t,d=H.findNearestSeg(car.position,p.segIdx)
  p.segIdx,p.lineDist=seg,d
  p.off=H.lateralOf(car.position,seg)
  local c=lanes(seg)
  if p.lane>#c then p.lane=#c end
  local lat=c[p.lane] or 0
  local bl,br=H.boundsAt(seg)
  lat=clamp(lat,bl+BOUND_MARGIN+p.halfW*0.5,br-BOUND_MARGIN-p.halfW*0.5)
  local ld=cornerCappedLookahead(seg,t,speed)
  local tx,tz=railPoint(seg,t,ld,lat)
  local dx,dz=tx-car.position.x,tz-car.position.z
  local dist=math.sqrt(dx*dx+dz*dz)
  if dist<1e-5 then return p.steer end
  local fx,fz=norm(car.look.x,car.look.z)
  local alpha=angle(fx,fz,dx/dist,dz/dist)
  local steer=math.atan(2*WHEELBASE*math.sin(alpha)/ld)/STEER_LOCK
  if math.abs(alpha)<STEER_DEAD then steer=0 end
  local zone=1.0+0.06*speed
  if p.off<bl+zone then steer=math.max(steer,0.28)
  elseif p.off>br-zone then steer=math.min(steer,-0.28) end
  return clamp(steer,-CFG.STEER_LIMIT,CFG.STEER_LIMIT)
end

local function wheelSlip(car)
  local ok,w=pcall(function() return car.wheels end)
  if not ok or not w then return nil end
  local peak
  for _,idx in ipairs({2,3}) do
    local ok2,v=pcall(function() return w[idx].slipRatio end)
    if ok2 and type(v)=='number' and v==v then
      local a=math.abs(v)
      if not peak or a>peak then peak=a end
    end
  end
  return peak
end

-- CHECKERS clear check for the ADJACENT lane: real dimensions, ahead by
-- the dynamic gap and behind by the closing speed of the follower.
local function laneClear(i,car,seg,targetLat)
  local sim=ac.getSim()
  if not sim then return true end
  local fx,fz=norm(car.look.x,car.look.z)
  local myHalfW=halfDims(car)
  local aheadClear=minGap(num(car.speedMs,0))
  for j=0,sim.carsCount-1 do
    if j~=i then
      local o=ac.getCar(j)
      if o and o.isActive then
        local dx,dz=o.position.x-car.position.x,o.position.z-car.position.z
        local forward=dx*fx+dz*fz
        if forward>-12 and forward<aheadClear then
          local os=select(1,H.findNearestSeg(o.position,seg))
          local oo=H.lateralOf(o.position,os)
          local ohw,ohl=halfDims(o)
          if math.abs(oo-targetLat) < (ohw+myHalfW) then
            if forward>-12 and forward<-ohl then
              local rel=num(o.speedMs,0)-num(car.speedMs,0)
              if rel>1 then return false end
            else
              return false
            end
          end
        end
      end
    end
  end
  return true
end

local function planTargetSpeed(p)
  local nodes=S.nodes
  local segs=H.segmentCount()
  local target=p.cruise
  local distance=0
  local current=p.segIdx
  local steps=0
  while distance<=CFG.PLAN_HORIZON and steps<200 do
    steps=steps+1
    local curv=H.getVertexCurvature(current)
    if curv>CFG.CURVE_MIN then
      local cs=math.max(CFG.MIN_APEX_SPEED,math.sqrt(CFG.MAX_LATERAL_ACCEL/curv))
      local ss=math.sqrt(cs*cs+2*CFG.BRAKE_ACCEL*distance)
      if ss<target then target=ss end
    end
    local a=nodes[current]
    local b=H.segEnd(current)
    distance=distance+math.sqrt((b.x-a.x)^2+(b.z-a.z)^2)
    if S.loopRoute then current=(current%segs)+1
    elseif current<segs then current=current+1
    else break end
  end
  return math.max(0,target)
end

local function updateTraffic(i,car,p,speed,target,dt)
  local c=lanes(p.segIdx)
  if p.lane>#c then p.lane=#c end
  local myHalfW=p.halfW or DEFAULT_HALF_W
  local gap,leadSpeed=H.carAheadFull(i,car,myHalfW*2+0.4)

  if not gap then p.blocked=0
  elseif (leadSpeed or 0)<speed-1.5 then
    p.blocked=p.blocked+dt
    if p.blocked>TRIGGER_TIME and #c>1 then
      -- CHECKERS: strictly the ADJACENT lane (lane-1 first, then lane+1)
      local targetLane=nil
      if p.lane>1 and laneClear(i,car,p.segIdx,c[p.lane-1]) then
        targetLane=p.lane-1
      elseif p.lane<#c and laneClear(i,car,p.segIdx,c[p.lane+1]) then
        targetLane=p.lane+1
      end
      if targetLane then
        p.passSide=(targetLane>p.lane) and 1 or -1
        p.lane=targetLane
        p.homeLane=targetLane
        p.passTime=0; p.blocked=0
        H.log('CAR=%d LANE_CHANGE to=%d gap=%.1f own=%.0f lead=%.0f',
          i,targetLane,gap,target*3.6,(leadSpeed or 0)*3.6)
        return target
      end
    end
    local safe=math.sqrt(math.max(0,(leadSpeed or 0)^2+2*CFG.BRAKE_ACCEL*math.max(0,gap-minGap(speed))))
    return math.min(target,math.max(safe,3))
  else
    p.blocked=0
  end
  return target
end

local function throttle(p,car,speed,target)
  local err=target-speed
  local gas=clamp(err*CFG.GAS_GAIN,0,CFG.MAX_GAS)
  local slip=wheelSlip(car)
  if slip and slip>0.18 then
    gas=gas*clamp(1-(slip-0.18)*1.8,0.12,1)
  end
  if speed<2 then gas=math.min(gas,0.5) end
  local brake=clamp(-err*CFG.BRAKE_GAIN,0,CFG.MAX_BRAKE)
  if err>=0 then brake=0 end
  p.gas,p.brake=gas,brake
end

function Driver.update(i,car,dt)
  local p=S.p[i] or newDriver(i)
  p.dimT=(p.dimT or 0)+dt
  if p.dimT>=1 then
    p.dimT=0
    local hw,hl=halfDims(car)
    p.halfW,p.halfL=hw,hl
  end
  p.retire=p.retire+dt
  if p.retire>=2 then
    p.retire=0
    if physics and physics.preventAIFromRetiring then pcall(physics.preventAIFromRetiring,i) end
  end
  local speed=math.max(0,num(car.speedMs,0))
  if speed<RESCUE_speed then
    p.stuck=p.stuck+dt
    if p.stuck>RESCUE_TIME and num(S.sessionTime,999)>RESCUE_COOLDOWN then
      p.stuck=0
      if H.teleportHome then H.teleportHome(i,car) end
      return
    end
  else p.stuck=0 end
  local target=planTargetSpeed(p)
  target=updateTraffic(i,car,p,speed,target,dt)
  p.target=target
  p.steer=purePursuit(car,p,speed)
  throttle(p,car,speed,target)
  local ok,c=pcall(ac.overrideCarControls,i)
  if ok and c then
    c.combineAxis=false
    c.steer=p.steer; c.gas=p.gas; c.brake=p.brake; c.handbrake=0
  end
end

function Driver.telemetryLine(i,car,p)
  local bl,br=H.boundsAt(p.segIdx)
  return string.format('CAR=%d POS=(%.2f,%.2f,%.2f) SPEED=%.2f STEER=%.3f GAS=%.3f BRAKE=%.3f TARGET_SPEED=%.2f LINE_ERROR=%.2f SEGMENT=%d PACE=%.2f LANE=%d/%d BOUNDS=[%.1f,%.1f] OFF=%.1f DIM=%.1fx%.1f',
    i,car.position.x,car.position.y,car.position.z,num(car.speedMs),p.steer,p.gas,p.brake,p.target,p.lineDist,p.segIdx,p.pace,p.lane,p.homeLane,bl,br,p.off,p.halfW*2,p.halfL*2)
end
return Driver
