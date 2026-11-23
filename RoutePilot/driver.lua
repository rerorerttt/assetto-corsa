-- RoutePilot/driver.lua — v0.11.1 RAILS AI (CSP 0.3.2-preview2)
-- Textbook PURE PURSUIT (bicycle model) rail follower:
--    ld = clamp(LOOKAHEAD_TIME * v, LOOK_MIN, LOOK_MAX)
--    alpha = angle(heading -> target on the rail ld ahead)
--    steer = atan(2 * WHEELBASE * sin(alpha) / ld) / STEER_LOCK
-- One geometric command, no low-pass phase lag (lag = lateral oscillation),
-- no multi-point averaging (it cut corners outside the bounds in v0.10).
-- Lane count comes from state.widthPreset (set by the 1/2/3/4 buttons);
-- rail centers are derived from the SAVED per-segment bounds, so rails
-- always stay inside the recorded track even where it narrows.
-- Burnout control: wheel slip feedback (car.wheels[2]/[3].slipRatio,
-- guarded with pcall) + soft launch. Blockers only in the SAME lane;
-- overtakes are decisive changes into a CLEAR adjacent lane.

local Driver = {}
local CFG, S, H

local WHEELBASE, STEER_LOCK = 2.6, 0.60
local LOOKAHEAD_TIME, LOOK_MIN, LOOK_MAX = 0.55, 6, 35
local BOUND_MARGIN  = 0.35
local SAME_LANE_HALF = 1.15
local PASS_CLEAR, MIN_GAP = 16, 6
local TRIGGER_TIME, PASS_TIMEOUT = 0.55, 8
local RESCUE_SPEED, RESCUE_TIME = 1.5, 3.5
local STEER_DEAD = 0.004

local function clamp(x,a,b) return math.max(a, math.min(b,x)) end
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

-- rail centers for the preset lane count, inside the SAVED bounds at seg
local function lanes(seg)
  local count=clamp(math.floor(num(S.widthPreset,1)),1,4)
  local left,right=H.boundsAt(seg)
  local lo,hi=left+BOUND_MARGIN,right-BOUND_MARGIN
  if hi<=lo then return {0} end
  if count==1 then return {(lo+hi)*0.5} end
  local step=(hi-lo)/count
  local c={}
  for i=1,count do c[i]=lo+step*(i-0.5) end
  return c
end

local function newDriver(i)
  local pace=0.88+rnd(i,1)*0.22
  local p={segIdx=1,pace=pace,
    cruise=num(S.speedMax,170)*pace/3.6,
    homeLane=1,lane=1,passSide=0,passTime=0,blocked=0,
    steer=0,gas=0,brake=0,stuck=0,retire=0,
    lineDist=0,off=0,target=0}
  S.p[i]=p
  local c=lanes(1)
  if #c>1 then
    -- fast cars left (lane 1), slow right, by pace ranking
    p.homeLane=clamp(1+math.floor((1.10-pace)/0.22*(#c-1)+0.5),1,#c)
    p.lane=p.homeLane
  end
  H.log('CAR_INIT id=%d pace=%.2f lane=%d/%d',i,pace,p.homeLane,#c)
  return p
end

local function railPoint(seg,t,distance,lat)
  local pt=H.walkLine(seg,t,distance)
  if not pt then pt=H.segEnd(seg) end
  local a=S.nodes[seg]
  local b=H.segEnd(seg)
  local tx,tz=norm(b.x-a.x,b.z-a.z)
  return pt.x-tz*lat, pt.z+tx*lat
end

local function purePursuit(car,p,speed)
  local seg,t,d=H.findNearestSeg(car.position,p.segIdx)
  p.segIdx,p.lineDist=seg,d
  p.off=H.lateralOf(car.position,seg)
  local c=lanes(seg)
  if p.lane>#c then p.lane=#c end
  local bl,br=H.boundsAt(seg)
  local lat=c[p.lane] or 0
  lat=clamp(lat,bl+BOUND_MARGIN,br-BOUND_MARGIN)
  local ld=clamp(LOOKAHEAD_TIME*math.max(speed,4),LOOK_MIN,LOOK_MAX)
  local tx,tz=railPoint(seg,t,ld,lat)
  local dx,dz=tx-car.position.x,tz-car.position.z
  local dist=math.sqrt(dx*dx+dz*dz)
  if dist<1e-5 then return p.steer end
  local fx,fz=norm(car.look.x,car.look.z)
  local alpha=angle(fx,fz,dx/dist,dz/dist)
  local steer=math.atan(2*WHEELBASE*math.sin(alpha)/ld)/STEER_LOCK
  -- deadband: on a rail, zero angle error means literally zero steering
  if math.abs(alpha)<STEER_DEAD then steer=0 end
  -- recovery ONLY when the car is actually near/crossing a recorded edge
  if p.off<bl+0.25 then steer=math.max(steer,0.28)
  elseif p.off>br-0.25 then steer=math.min(steer,-0.28) end
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

local function laneClear(i,car,seg,targetLat)
  local sim=ac.getSim()
  if not sim then return true end
  local fx,fz=norm(car.look.x,car.look.z)
  for j=0,sim.carsCount-1 do
    if j~=i then
      local o=ac.getCar(j)
      if o and o.isActive then
        local dx,dz=o.position.x-car.position.x,o.position.z-car.position.z
        local forward=dx*fx+dz*fz
        if forward>-3 and forward<PASS_CLEAR then
          local os=select(1,H.findNearestSeg(o.position,seg))
          local oo=H.lateralOf(o.position,os)
          if math.abs(oo-targetLat)<SAME_LANE_HALF*2 then return false end
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
  local lat=c[p.lane] or 0
  local gap,leadSpeed=H.carAheadFull(i,car,SAME_LANE_HALF*2)
  if p.passSide~=0 then
    p.passTime=p.passTime+dt
    local stillBlocked=gap and (leadSpeed or 0)<speed-1
    if p.passTime>PASS_TIMEOUT or not stillBlocked then
      p.passSide=0; p.passTime=0
      if p.lane~=p.homeLane and c[p.homeLane] and laneClear(i,car,p.segIdx,c[p.homeLane]) then
        p.lane=p.homeLane
      end
      return target
    end
    if gap then
      return math.min(math.max(target,(leadSpeed or 0)+3),math.max(leadSpeed or 0,speed))
    end
    return target
  end
  if not gap then p.blocked=0; return target end
  if (leadSpeed or 0)<speed-1.5 then
    p.blocked=p.blocked+dt
    if speed>8 and p.blocked>TRIGGER_TIME and #c>1 then
      local best,bestLat=nil,nil
      for li=1,#c do
        if li~=p.lane and laneClear(i,car,p.segIdx,c[li]) then
          if not best or math.abs(li-p.homeLane)<math.abs(best-p.homeLane) then
            best,bestLat=li,c[li]
          end
        end
      end
      if best then
        p.passSide=(bestLat>lat) and 1 or -1
        p.lane=best; p.passTime=0; p.blocked=0
        H.log('CAR=%d LANE_CHANGE to=%d lat=%.1f gap=%.1f',i,best,bestLat,gap)
        return math.max(target,(leadSpeed or 0)+3)
      end
    end
    local safe=math.sqrt(math.max(0,(leadSpeed or 0)^2+2*CFG.BRAKE_ACCEL*math.max(0,gap-MIN_GAP)))
    return math.min(target,math.max(safe,5))
  end
  p.blocked=0
  return target
end

local function throttle(p,car,speed,target)
  local err=target-speed
  local gas=clamp(err*CFG.GAS_GAIN,0,CFG.MAX_GAS)
  local slip=wheelSlip(car)
  if slip and slip>0.18 then
    gas=gas*clamp(1-(slip-0.18)*1.8,0.12,1)
  end
  if speed<3 then gas=math.min(gas,0.4) end
  local brake=clamp(-err*CFG.BRAKE_GAIN,0,CFG.MAX_BRAKE)
  if err>=0 then brake=0 end
  p.gas,p.brake=gas,brake
end

function Driver.update(i,car,dt)
  local p=S.p[i] or newDriver(i)
  p.retire=p.retire+dt
  if p.retire>=2 then
    p.retire=0
    if physics and physics.preventAIFromRetiring then pcall(physics.preventAIFromRetiring,i) end
  end
  local speed=math.max(0,num(car.speedMs,0))
  if speed<RESCUE_SPEED then
    p.stuck=p.stuck+dt
    if p.stuck>RESCUE_TIME then
      p.stuck=0
      if H.teleportHome then H.teleportHome(i,car) end
      return
    end
  else p.stuck=0 end
  local target=planTargetSpeed(p)
  target=updateTraffic(i,car,p,speed,target,dt)
  p.target=target
  -- one geometric command per frame, applied directly (no lag -> no weave)
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
  return string.format('CAR=%d POS=(%.2f,%.2f,%.2f) SPEED=%.2f STEER=%.3f GAS=%.3f BRAKE=%.3f TARGET_SPEED=%.2f LINE_ERROR=%.2f SEGMENT=%d PACE=%.2f LANE=%d/%d BOUNDS=[%.1f,%.1f] OFF=%.1f OT=%d',
    i,car.position.x,car.position.y,car.position.z,num(car.speedMs),p.steer,p.gas,p.brake,p.target,p.lineDist,p.segIdx,p.pace,p.lane,p.homeLane,bl,br,p.off,p.passSide)
end
return Driver
