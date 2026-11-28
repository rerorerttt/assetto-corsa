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
  while first<last and l.cumulative[first+1]<hint-window do first=first+1 end
  last=first;while last<#l.points-1 and l.cumulative[last]<hint+window do last=last+1 end
 end
 local best,bd
 for i=first,last do
  local a,b=l.points[i],l.points[i+1];local vx,vz=b.x-a.x,b.z-a.z;local ll=vx*vx+vz*vz
  if ll>1e-8 then
   local t=clamp(((pos.x-a.x)*vx+(pos.z-a.z)*vz)/ll,0,1);local x,z=a.x+vx*t,a.z+vz*t
   local dx,dz=pos.x-x,pos.z-z;local d2=dx*dx+dz*dz
   if not bd or d2<bd then local len=sqrt(ll);best={s=l.cumulative[i]+len*t,distance=sqrt(d2),x=x,y=a.y+(b.y-a.y)*t,z=z,tx=vx/len,tz=vz/len,segment=i};bd=d2 end
  end
 end
 return best
end
local function nearest(g,car)
 local hx,hz=norm(car.look.x,car.look.z);local best,score
 for _,id in ipairs(g.laneOrder)do local l=g.lanes[id];local q=project(l,car.position)
  if q then local a=dot(hx,hz,q.tx,q.tz);if a>0.25 and q.distance<35 then local v=q.distance*q.distance+(1-a)*30;if not score or v<score then best={id=id,s=q.s};score=v end end end
 end
 return best
end
local function occupancy(g,laneId,s,index,sim,range)
 local l=g.lanes[laneId];local ahead,behind
 for i=0,sim.carsCount-1 do if i~=index then local c=ac.getCar(i);if c and c.isActive then
  local q=project(l,c.position,s,range+20)
  if q and q.distance<4.8 then local hx,hz=norm(c.look.x,c.look.z)
   if dot(hx,hz,q.tx,q.tz)>0.15 then local gap=q.s-s
    if l.loop then if gap>l.length*.5 then gap=gap-l.length elseif gap< -l.length*.5 then gap=gap+l.length end end
    if gap>=0 and gap<range and(not ahead or gap<ahead.gap)then ahead={gap=gap,speed=max(0,n(c.speedMs)),index=i}
    elseif gap<0 and -gap<18 and(not behind or -gap<behind.gap)then behind={gap=-gap,speed=max(0,n(c.speedMs)),index=i}end
   end
  end
 end end end
 return ahead,behind
end
local function laneChange(g,l,p,index,sim,speed)
 local best,score
 for _,e in ipairs(l.adjacent or{})do if p.s>=e.fromStart+2 and p.s<=e.fromEnd-2 then
  local target=g.lanes[e.to]
  if target then local src=point(l,p.s);local q=project(target,src,(e.toStart+e.toEnd)*.5,(e.toEnd-e.toStart)*.5+10)
   if q and q.distance>=2 and q.distance<=8 then
    local a,b=occupancy(g,target.id,q.s,index,sim,max(35,speed*2.5))
    if(not a or a.gap>max(18,speed*1.2))and(not b or b.gap>max(12,speed*.8))then
     local v=(a and a.gap or 1000)+(b and b.gap or 1000)*.4
     if not score or v>score then best={id=target.id,s=q.s};score=v end
    end
   end
  end
 end end
 return best
end
local function junction(g,l,p,car,index,sim)
 local hx,hz=norm(car.look.x,car.look.z);local best,score
 for _,e in ipairs(l.outgoing or{})do if p.s>=e.fromS-6 and p.s<=e.fromS+10 then
  local t=g.lanes[e.to]
  if t then local q=point(t,e.toS);local tx,tz=tangent(t,e.toS);local a=dot(hx,hz,tx,tz)
   if a>0.05 and dist(car.position,q)<32 then local block=occupancy(g,t.id,e.toS,index,sim,25)
    if not block or block.gap>8 then local v=a*10-dist(car.position,q)*.03;if not score or v>score then best={id=t.id,s=e.toS};score=v end end
   end
  end
 end end
 return best
end
local function controls(i,steer,gas,brake)
 local ok,c=pcall(ac.overrideCarControls,i);if ok and c then c.combineAxis=false;c.steer=clamp(steer,-1,1);c.gas=clamp(gas,0,1);c.brake=clamp(brake,0,1);c.handbrake=0;return true end;return false
end
function M.setup(cfg,state,helpers)CFG,S,H=cfg,state,helpers end
function M.prepareTraffic(sim,order,dt)
 local g=S.trafficNetwork;if not g then return end
 for _,i in ipairs(order or{})do local c=ac.getCar(i);if c and c.isActive then local p=S.p[i]or{};S.p[i]=p
  if not p.networkLaneId then local q=nearest(g,c);if q then p.networkLaneId=q.id;p.networkS=q.s;H.log("NETWORK_ATTACH id=%d lane=%d s=%.1f",i,q.id,q.s)end end
  local l=g.lanes[p.networkLaneId];if l then local q=project(l,c.position,p.networkS,max(35,n(c.speedMs)*1.8+15));if q and q.distance<18 then p.networkS=q.s;p.networkDistance=q.distance;p.networkProjectionLost=false else p.networkProjectionLost=true end end
 end end
end
function M.update(i,c,dt)
 local g=S.trafficNetwork;local p=S.p[i]or{};S.p[i]=p;local speed=max(0,n(c.speedMs))
 p.retireT=n(p.retireT)+max(0,dt)
 if p.retireT>=2 then p.retireT=0;if physics and physics.preventAIFromRetiring then pcall(physics.preventAIFromRetiring,i)end end
 if not p.networkLaneId then local q=nearest(g,c);if q then p.networkLaneId=q.id;p.networkS=q.s end end
 local l=g.lanes[p.networkLaneId]
 if not l or p.networkS==nil then controls(i,0,0,1);p.trafficAction="NETWORK_NO_LANE";return end
 local q=project(l,c.position,p.networkS,max(35,speed*1.8+15))
 if not q or q.distance>18 then controls(i,0,0,1);p.projectionLost=true;p.trafficAction="NETWORK_PROJECTION_LOST";return end
 p.networkS=q.s;p.networkDistance=q.distance
 local sim=ac.getSim();local nextJ=junction(g,l,{s=p.networkS},c,i,sim)
 if nextJ then
  local old,oldS=l.id,p.networkS
  l=g.lanes[nextJ.id];p.networkLaneId=l.id;p.networkS=nextJ.s
  local targetProjection=project(l,c.position,p.networkS,35)
  if not targetProjection or targetProjection.distance>28 then
   p.networkLaneId=old;p.networkS=oldS;l=g.lanes[old]
   controls(i,0,0,0.65);p.projectionLost=true;p.trafficAction="NETWORK_JUNCTION_REACQUIRE";return
  end
  q=targetProjection;p.networkS=q.s
  H.log("NETWORK_JUNCTION id=%d from=%d to=%d",i,old,l.id)
 end
 local lead=occupancy(g,l.id,p.networkS,i,sim,max(100,speed*3+30))
 if lead and lead.gap<max(32,speed*2)then local change=laneChange(g,l,{s=p.networkS},i,sim,speed)
  if change then
   local old,oldS=l.id,p.networkS
   l=g.lanes[change.id];p.networkLaneId=l.id;p.networkS=change.s
   local targetProjection=project(l,c.position,p.networkS,35)
   if not targetProjection or targetProjection.distance>12 then
    p.networkLaneId=old;p.networkS=oldS;l=g.lanes[old]
   else
    q=targetProjection;p.networkS=q.s
    lead=occupancy(g,l.id,p.networkS,i,sim,max(100,speed*3+30))
    H.log("NETWORK_LANE_CHANGE id=%d from=%d to=%d",i,old,l.id)
   end
  end
 end
 if p.networkS>=l.length-1.5 and not l.loop then
  local nextLane=junction(g,l,{s=p.networkS},c,i,sim)
  if nextLane then
   l=g.lanes[nextLane.id];p.networkLaneId=l.id;p.networkS=nextLane.s
   local targetProjection=project(l,c.position,p.networkS,35)
   if not targetProjection or targetProjection.distance>28 then controls(i,0,0,0.65);p.projectionLost=true;return end
   q=targetProjection;p.networkS=q.s
  else controls(i,0,0,1);p.trafficAction="NETWORK_END_OF_LANE";return end
 end
 local look=clamp(9+speed*.55,9,55);local aimS=l.loop and((p.networkS+look)%l.length)or min(l.length,p.networkS+look)
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
  local s0=max(0,cs-3);local s1=min(l.length,cs+3)
  local ax,az=tangent(l,s0);local bx,bz=tangent(l,s1)
  local curvature=abs(atan2(ax*bz-az*bx,clamp(dot(ax,az,bx,bz),-1,1)))/max(1,s1-s0)
  if curvature>0.0001 then
   local curveSpeed=sqrt(8.0/curvature)
   local safeSpeed=sqrt(curveSpeed*curveSpeed+14.0*max(0,aheadDistance-5))
   target=min(target,safeSpeed)
  end
 end
 if l.speedLimit then target=min(target,l.speedLimit/3.6)end
 if lead then target=min(target,sqrt(max(0,lead.speed^2+14*max(0,lead.gap-7))))end
 local err=target-speed;local gas=clamp(err*.35,0,1);local brake=clamp(-err*.55,0,1)
 if speed<5 then gas=min(gas,.65)end;if err>=0 then brake=0 end
 p.networkLaneId=l.id;p.networkS=p.networkS;p.routeProgress=p.networkS;p.routeLocked=true
 p.networkDistance=q.distance;p.lineDist=q.distance;p.segIdx=q.segment or 1;p.latOff=0
 p.targetSpeed=target;p.steer=steer;p.gas=gas;p.brake=brake
 p.trafficAction=lead and"NETWORK_FOLLOW"or"NETWORK_OPEN";p.trafficLeaderIndex=lead and lead.index or -1;p.trafficGap=lead and lead.gap or -1
 p.projectionLost=false;controls(i,steer,gas,brake)
end
return M
