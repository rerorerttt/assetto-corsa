-- Directed CSP lane-network controller. No synthetic route or road boundaries.
local M={}
local CFG,S,H
local abs,min,max,sqrt,atan2=math.abs,math.min,math.max,math.sqrt,math.atan2
local function clamp(v,a,b)return max(a,min(b,v))end
local function n(v,d)if type(v)~="number" or v~=v or v==math.huge or v==-math.huge then return d or 0 end return v end
local function norm(x,z)local l=sqrt(x*x+z*z);if l<1e-8 then return 0,1 end return x/l,z/l end
local function dot(ax,az,bx,bz)return ax*bx+az*bz end
local function dist(a,b)local x,z=a.x-b.x,a.z-b.z;return sqrt(x*x+z*z)end
local function point(l,s)
 s=clamp(s,0,l.length);local c=l.cumulative;local lo,hi=1,#c-1
 while lo<hi do local m=math.floor((lo+hi)/2);if c[m+1]<s then lo=m+1 else hi=m end end
 local a,b=l.points[lo],l.points[lo+1];local d=c[lo+1]-c[lo];local t=d>1e-8 and (s-c[lo])/d or 0
 return {x=a.x+(b.x-a.x)*t,y=a.y+(b.y-a.y)*t,z=a.z+(b.z-a.z)*t}
end
local function tangent(l,s)
 local c=l.cumulative;local lo,hi=1,#c-1
 while lo<hi do local m=math.floor((lo+hi)/2);if c[m+1]<s then lo=m+1 else hi=m end end
 local a,b=l.points[lo],l.points[lo+1];return norm(b.x-a.x,b.z-a.z)
end
local function project(l,pos,hint,window)
 local first,last=1,#l.points-1
 if hint and window then
  -- Binary-search both ends of the local projection window. The previous
  -- linear walk from point 1 made every car projection O(route length), and
  -- occupancy repeats that work for every other car on every frame.
  local c=l.cumulative
  local lo,hi=1,#c-1
  local target=max(0,hint-window)
  while lo<hi do
   local mid=math.floor((lo+hi)*0.5)
   if c[mid+1]<target then lo=mid+1 else hi=mid end
  end
  first=lo
  lo,hi=first,#c-1
  target=min(l.length,hint+window)
  while lo<hi do
   local mid=math.floor((lo+hi)*0.5)
   if c[mid]<target then lo=mid+1 else hi=mid end
  end
  last=max(first,lo)
 end
 local best,bd
 for i=first,last do
  local a,b=l.points[i],l.points[i+1];local vx,vz=b.x-a.x,b.z-a.z;local ll=vx*vx+vz*vz
  if ll>1e-8 then
   local t=clamp(((pos.x-a.x)*vx+(pos.z-a.z)*vz)/ll,0,1);local x,z=a.x+vx*t,a.z+vz*t
   local dx,dz=pos.x-x,pos.z-z
   local y=a.y+(b.y-a.y)*t
   local dy=pos.y-y
   -- Use height in lane selection: on stacked highways, two lanes can
   -- overlap in X/Z while belonging to completely different road decks.
   local d2=dx*dx+dz*dz+dy*dy*4.0
   if not bd or d2<bd then local len=sqrt(ll);best={s=l.cumulative[i]+len*t,distance=sqrt(d2),heightError=dy,x=x,y=y,z=z,tx=vx/len,tz=vz/len,segment=i};bd=d2 end
  end
 end
 return best
end
local function projectNear(l,pos,hint,window)
 local q=project(l,pos,hint,window)
 if l.loop and hint~=nil then
  local altHint=nil
  if hint<window then altHint=l.length
  elseif hint>l.length-window then altHint=0 end
  if altHint~=nil then
   local alt=project(l,pos,altHint,window)
   if alt and(not q or alt.distance<q.distance)then q=alt end
  end
 end
 return q
end
local function nearest(g,car)
 local hx,hz=norm(car.look.x,car.look.z);local best,score
 local pos=car.position
 for _,id in ipairs(g.laneOrder)do
  local l=g.lanes[id];local box=l and l.aabb
  local possible=true
  if box then
   local dx=max(box.minX-pos.x,0,pos.x-box.maxX)
   local dz=max(box.minZ-pos.z,0,pos.z-box.maxZ)
   possible=dx*dx+dz*dz<35*35 and pos.y>=box.minY-8 and pos.y<=box.maxY+8
  end
  if possible then
   local q=project(l,pos)
   if q then
    local a=dot(hx,hz,q.tx,q.tz)
    if a>0.25 and q.distance<35 then
     local v=q.distance*q.distance+(1-a)*30
     if not score or v<score then best={id=id,s=q.s};score=v end
    end
   end
  end
 end
 return best
end
local function attachIfDue(g,c,p,dt,index)
 if p.networkLaneId then return end
 p.networkAttachTimer=max(0,n(p.networkAttachTimer)-max(0,dt))
 if p.networkAttachTimer>0 then return end
 local q=nearest(g,c)
 if q then
  p.networkLaneId=q.id;p.networkS=q.s;p.networkAttachTimer=0.5
  H.log("NETWORK_ATTACH id=%d lane=%d s=%.1f",index or -1,q.id,q.s)
 else
  p.networkAttachTimer=1.0
 end
end
local function occupancy(g,laneId,s,index,sim,range)
 local l=g.lanes[laneId];local ahead,behind
 if not sim then return nil,nil end
 for i=0,sim.carsCount-1 do if i~=index then local c=ac.getCar(i);if c and c.isActive then
  local q=projectNear(l,c.position,s,range+20)
  local otherState=S.p[i]
  local sameLane=otherState and otherState.networkLaneId==laneId
  local occupancyRadius=sameLane and 4.8 or 2.0
  if q and q.distance<occupancyRadius then local hx,hz=norm(c.look.x,c.look.z)
   local alignment=dot(hx,hz,q.tx,q.tz)
  local gap=q.s-s
  if l.loop then if gap>l.length*.5 then gap=gap-l.length elseif gap< -l.length*.5 then gap=gap+l.length end end
  if alignment>0.15 then
   if gap>=0 and gap<range and(not ahead or gap<ahead.gap)then ahead={gap=gap,speed=max(0,n(c.speedMs)),index=i}
   elseif gap<0 and -gap<18 and(not behind or -gap<behind.gap)then behind={gap=-gap,speed=max(0,n(c.speedMs)),index=i}end
  elseif (sameLane or not otherState) and abs(gap)<range and(not ahead or abs(gap)<ahead.gap)then
   ahead={gap=abs(gap),speed=-max(0,n(c.speedMs)),index=i}
  end
  end
 end end end
 return ahead,behind
end
local function clearLaneCrossing(src,dst,srcS,dstS)
 -- A geometric neighbour is not necessarily physically reachable: a fence
 -- can sit between two parallel CSP lines. Test the crossing against the
 -- track's physical mesh at several points before allowing a lane change.
 if not physics or type(physics.raycastTrack)~="function" or type(vec3)~="function" then return false end
 local offsets={0,5,10}
 for _,offset in ipairs(offsets)do
  local ss=src.loop and((srcS+offset)%src.length)or min(src.length,srcS+offset)
  local ts=dst.loop and((dstS+offset)%dst.length)or min(dst.length,dstS+offset)
  local a=point(src,ss);local b=point(dst,ts)
  local dx,dy,dz=b.x-a.x,b.y-a.y,b.z-a.z
  local length=sqrt(dx*dx+dy*dy+dz*dz)
  if length<1.8 or length>6.5 or abs(dy)>0.8 then return false end
  local heights={0.45,0.95}
  for _,height in ipairs(heights)do
   local origin=vec3(a.x,a.y+height,a.z)
   local direction=vec3(dx/length,dy/length,dz/length)
   local ok,hit=pcall(physics.raycastTrack,origin,direction,length)
   if not ok or type(hit)~="number" then return false end
   if hit>=0 and hit<length-0.35 then return false end
  end
 end
 return true
end
local function laneChange(g,l,p,index,sim,speed)
 local best,bestDistance
 for _,e in ipairs(l.adjacent or{})do
  local transitionDistance=clamp(speed*0.35,8,20)
  if p.s>=e.fromStart+2 and p.s<=e.fromEnd-transitionDistance then
   local target=g.lanes[e.to]
   if target then
    local src=point(l,p.s)
    local q=projectNear(target,src,(e.toStart+e.toEnd)*.5,(e.toEnd-e.toStart)*.5+10)
    if q and q.distance>=2.2 and q.distance<=5.5 and abs(q.heightError or 0)<=0.8 and
        dot(q.tx,q.tz,(select(1,tangent(l,p.s))),(select(2,tangent(l,p.s))))>=0.98 then
     local ahead,behind=occupancy(g,target.id,q.s,index,sim,max(45,speed*2.8))
     local safeAhead=not ahead or ahead.gap>max(24,speed*1.5)
     local safeBehind=not behind or behind.gap>max(16,speed*1.0)
     if safeAhead and safeBehind and clearLaneCrossing(l,target,p.s,q.s) then
      if not bestDistance or q.distance<bestDistance then
       best={id=target.id,s=q.s};bestDistance=q.distance
      end
     end
    end
   end
  end
 end
 return best
end
local function junction(g,l,p,car,index,sim,blockedTarget)
 local hx,hz=norm(car.look.x,car.look.z)
 local best,score
 -- Do not jump to a connected lane while the car is still approaching the
 -- intersection. Require physical proximity to the outgoing source polyline
 -- and a heading compatible with that lane's authored direction.
 for _,e in ipairs(l.outgoing or{})do
  if e.to~=blockedTarget and p.s>=e.fromS-1.5 and p.s<=e.fromS+5.0 then
   local target=g.lanes[e.to]
   if target then
    local q=projectNear(target,car.position,e.toS,24)
    if q and q.distance<=7.5 then
     local headingAlignment=dot(hx,hz,q.tx,q.tz)
     local routeAlignment=n(e.alignment,1)
     if headingAlignment>=-0.15 and routeAlignment>=-0.25 then
      local block=occupancy(g,target.id,q.s,index,sim,25)
      if not block or block.gap>8 then
       local value=headingAlignment*8+routeAlignment*4-q.distance
       if not score or value>score then best={id=target.id,s=q.s};score=value end
      end
     end
    end
   end
  end
 end
 return best
end
local function controls(i,steer,gas,brake)
 local ok,c=pcall(ac.overrideCarControls,i);if ok and c then c.combineAxis=false;c.steer=clamp(steer,-1,1);c.gas=clamp(gas,0,1);c.brake=clamp(brake,0,1);c.handbrake=0;return true end;return false
end
function M.setup(cfg,state,helpers)CFG,S,H=cfg,state,helpers end
function M.prepareTraffic(sim,order,dt)
 local g=S.trafficNetwork;if not g then return end
 for _,i in ipairs(order or{})do
  local c=ac.getCar(i)
  if c and c.isActive then
   local p=S.p[i]or{};S.p[i]=p
   attachIfDue(g,c,p,dt,i)
   local l=g.lanes[p.networkLaneId]
   if l then
    local q=projectNear(l,c.position,p.networkS,max(35,n(c.speedMs)*1.8+15))
    if q and q.distance<18 then
     p.networkS=q.s;p.networkDistance=q.distance;p.networkHeightError=q.heightError;p.networkProjectionLost=false
    else p.networkProjectionLost=true end
   end
  end
 end
end
function M.update(i,c,dt)
 local g=S.trafficNetwork;local p=S.p[i]or{};S.p[i]=p;local speed=max(0,n(c.speedMs))
 p.retireT=n(p.retireT)+max(0,dt)
 if p.retireT>=2 then p.retireT=0;if physics and physics.preventAIFromRetiring then pcall(physics.preventAIFromRetiring,i)end end
 local l=g.lanes[p.networkLaneId]
 if not l or p.networkS==nil then controls(i,0,0,1);p.trafficAction="NETWORK_NO_LANE";return end
 local q=projectNear(l,c.position,p.networkS,max(35,speed*1.8+15))
 if not q or q.distance>18 then
  -- Keep the assigned CSP lane when physical resistance (for example a
  -- roadside barrier) prevents lateral movement. Never brake to a stop or
  -- switch to a neighboring lane just because projection temporarily fails.
  local look=clamp(10+speed*.45,10,35)
  local aimS=l.loop and((p.networkS+look)%l.length)or min(l.length,p.networkS+look)
  local aim=point(l,aimS)
  local dx,dz=aim.x-c.position.x,aim.z-c.position.z
  local hx,hz=norm(c.look.x,c.look.z);local tx,tz=norm(dx,dz)
  local angle=atan2(hx*tz-hz*tx,clamp(dot(hx,hz,tx,tz),-1,1))
  local recoverSteer=clamp(atan2(5.4*math.sin(angle),max(1,sqrt(dx*dx+dz*dz)))/.55,-1,1)
  local previous=n(p.networkSteer,recoverSteer)
  local step=max(0.05,n(CFG.STEER_RATE,4.5)*max(0,dt))
  recoverSteer=previous+clamp(recoverSteer-previous,-step,step)
  p.networkSteer=recoverSteer
  local recoveryTarget=13.9
  local recoveryGas=clamp((recoveryTarget-speed)*.35,0,0.65)
  local recoveryBrake=clamp((speed-recoveryTarget)*.35,0,0.5)
  controls(i,recoverSteer,recoveryGas,recoveryBrake)
  p.projectionLost=true;p.trafficAction="NETWORK_HOLD_ASSIGNED_LANE"
  return
 end
 p.networkS=q.s;p.networkDistance=q.distance;p.networkHeightError=q.heightError
 local sim=ac.getSim()
 p.networkTransitionCooldown=max(0,n(p.networkTransitionCooldown)-max(0,dt))
 p.networkReverseLockTimer=max(0,n(p.networkReverseLockTimer)-max(0,dt))
 local blockedTarget=p.networkReverseLockTimer>0 and p.networkReverseLockFrom or nil
 local nextJ=p.networkTransitionCooldown<=0 and junction(g,l,{s=p.networkS},c,i,sim,blockedTarget) or nil
 if nextJ then
  local old,oldS=l.id,p.networkS
  l=g.lanes[nextJ.id];p.networkLaneId=l.id;p.networkS=nextJ.s
  local targetProjection=projectNear(l,c.position,p.networkS,35)
  if not targetProjection or targetProjection.distance>7.5 then
   p.networkLaneId=old;p.networkS=oldS;l=g.lanes[old]
   controls(i,0,0,0.65);p.projectionLost=true;p.trafficAction="NETWORK_JUNCTION_REACQUIRE";return
  end
  q=targetProjection;p.networkS=q.s;p.networkTransitionCooldown=1.0
  p.networkReverseLockFrom=old;p.networkReverseLockTimer=3.0
  H.log("NETWORK_JUNCTION id=%d from=%d to=%d distance=%.2f",i,old,l.id,q.distance)
 end
 local lead=occupancy(g,l.id,p.networkS,i,sim,max(100,speed*3+30))
 p.networkLaneChangeCooldown=max(0,n(p.networkLaneChangeCooldown)-max(0,dt))
 -- Change lanes only to the closest same-direction CSP neighbour, and only
 -- when a leader blocks the current lane. Physical raycasts reject fences
 -- and other track meshes between the two yellow lane lines.
 if lead and lead.gap<max(28,speed*1.5) and p.networkLaneChangeCooldown<=0 then
  local change=laneChange(g,l,{s=p.networkS},i,sim,speed)
  if change then
   local old=l.id
   l=g.lanes[change.id]
   p.networkLaneId=l.id;p.networkS=change.s
   q=projectNear(l,c.position,p.networkS,35)
   if q and q.distance<=5.5 then
    p.networkLaneChangeCooldown=4.0
    p.networkTransitionCooldown=max(p.networkTransitionCooldown or 0,1.0)
    lead=occupancy(g,l.id,p.networkS,i,sim,max(100,speed*3+30))
    H.log("NETWORK_LANE_CHANGE id=%d from=%d to=%d distance=%.2f",i,old,l.id,q.distance)
   else
    p.networkLaneId=old;l=g.lanes[old]
    p.networkS=projectNear(l,c.position,p.networkS,35) and projectNear(l,c.position,p.networkS,35).s or p.networkS
   end
  end
 end
 if p.networkS>=l.length-1.5 and not l.loop then
  local nextLane=p.networkTransitionCooldown<=0 and junction(g,l,{s=p.networkS},c,i,sim,blockedTarget) or nil
  if nextLane then
   local old,oldS=l.id,p.networkS
   l=g.lanes[nextLane.id];p.networkLaneId=l.id;p.networkS=nextLane.s
   local targetProjection=projectNear(l,c.position,p.networkS,35)
   if not targetProjection or targetProjection.distance>7.5 then
    p.networkLaneId=old;p.networkS=oldS;l=g.lanes[old]
    controls(i,0,0,0.65);p.projectionLost=true;return
   end
   q=targetProjection;p.networkS=q.s;p.networkTransitionCooldown=1.0
   p.networkReverseLockFrom=old;p.networkReverseLockTimer=3.0
  else controls(i,0,0,1);p.trafficAction="NETWORK_END_OF_LANE";return end
 end
 local look=clamp(6+speed*.32,7,20);local aimS=l.loop and((p.networkS+look)%l.length)or min(l.length,p.networkS+look)
 local aim=point(l,aimS);local dx,dz=aim.x-c.position.x,aim.z-c.position.z
 local hx,hz=norm(c.look.x,c.look.z);local tx,tz=norm(dx,dz)
 local angle=atan2(hx*tz-hz*tx,clamp(dot(hx,hz,tx,tz),-1,1))
 local steer=clamp(atan2(5.4*math.sin(angle),max(1,sqrt(dx*dx+dz*dz)))/.55,-1,1)
 local maxSteerStep=max(0.05,n(CFG.STEER_RATE,4.5)*max(0,dt))
 local previousSteer=n(p.networkSteer,steer)
 steer=previousSteer+clamp(steer-previousSteer,-maxSteerStep,maxSteerStep)
 if abs(steer)<0.002 then steer=0 end
 p.networkSteer=steer
 local target=75
 -- Preview curvature far enough ahead to brake before, not at, a bend.
 for aheadDistance=8,100,8 do
  local cs=p.networkS+aheadDistance
  if l.loop then cs=cs%l.length else cs=min(l.length,cs) end
  local s0,s1
  if l.loop then s0=(cs-3)%l.length;s1=(cs+3)%l.length
  else s0=max(0,cs-3);s1=min(l.length,cs+3) end
  local ax,az=tangent(l,s0);local bx,bz=tangent(l,s1)
  local curvature=abs(atan2(ax*bz-az*bx,clamp(dot(ax,az,bx,bz),-1,1)))/6.0
  if curvature>0.0001 then
   local curveSpeed=sqrt(8.0/curvature)
   local safeSpeed=sqrt(curveSpeed*curveSpeed+14.0*max(0,aheadDistance-5))
   target=min(target,safeSpeed)
  end
 end
 -- Account for the next directed junction turn and for terminal lane ends.
 local nextEdge,nextDistance=nil,math.huge
 for _,edge in ipairs(l.outgoing or {}) do
  local d=edge.fromS-p.networkS
  if d>=-6 and d<nextDistance then nextEdge=edge;nextDistance=max(0,d) end
 end
 if nextEdge then
  local alignment=clamp(n(nextEdge.alignment,1),-1,1)
  local turnAngle=math.acos(alignment)
  local turnSpeed=clamp(52-22*turnAngle,12,52)
  local safeTurnSpeed=sqrt(turnSpeed*turnSpeed+14*max(0,nextDistance-5))
  target=min(target,safeTurnSpeed)
 elseif not l.loop then
  local terminalDistance=max(0,l.length-p.networkS-3)
  target=min(target,sqrt(14*terminalDistance))
 end
 if l.speedLimit then target=min(target,l.speedLimit/3.6)end
 if lead then
  local available=max(0,lead.gap-7)
  if lead.speed<0 then
   local opposing=-lead.speed;local reaction=7.0*.45
   local safe=max(0,sqrt(max(0,14*available+2*opposing*reaction+reaction*reaction))-opposing-reaction)
   target=min(target,safe)
  else
   target=min(target,sqrt(max(0,lead.speed^2+14*available)))
  end
 end
 local err=target-speed;local gas=clamp(err*.35,0,1);local brake=clamp(-err*.55,0,1)
 -- A one-shot launch pulse gives freshly spawned traffic a natural initial
 -- acceleration. It ends as soon as the section-based target is reached or
 -- the short launch window expires; normal traffic control then takes over.
 local boostTimer=n(p.networkSpawnBoostTimer)
 local launchActive=false
 if boostTimer>0 then
  boostTimer=max(0,boostTimer-max(0,dt))
  local boostTarget=n(p.networkSpawnBoostTarget)
  if boostTarget>0 and speed<boostTarget-0.8 then
   gas=1;brake=0;launchActive=true
  else
   boostTimer=0
  end
  p.networkSpawnBoostTimer=boostTimer
 end
 if speed<5 and boostTimer<=0 then gas=min(gas,.65)end;if err>=0 and boostTimer<=0 then brake=0 end
 p.networkLaneId=l.id;p.networkS=p.networkS;p.routeProgress=p.networkS;p.routeLocked=true
 p.networkDistance=q.distance;p.networkHeightError=q.heightError;p.lineDist=q.distance;p.segIdx=q.segment or 1;p.latOff=0
 p.targetSpeed=target;p.steer=steer;p.gas=gas;p.brake=brake
 p.trafficPriority=n(l.priority)
 p.trafficAction=launchActive and"NETWORK_SPAWN_ACCEL"or(lead and"NETWORK_FOLLOW"or"NETWORK_OPEN");p.trafficLeaderIndex=lead and lead.index or -1;p.trafficGap=lead and lead.gap or -1
 p.projectionLost=false;controls(i,steer,gas,brake)
end
return M
