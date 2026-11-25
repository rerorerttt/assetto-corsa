-- RoutePilot/route_geometry.lua
-- Shared arc-length geometry for the recorded Catmull-Rom route.
local Geometry = {}
local function clamp(v, a, b) return math.max(a, math.min(b, v)) end
local function normalize(x, z)
  local l = math.sqrt(x*x + z*z)
  if l < 1e-8 then return 0, 1 end
  return x/l, z/l
end
local function angle(ax, az, bx, bz)
  return math.atan2(ax*bz-az*bx, ax*bx+az*bz)
end
local function wrap(s, total)
  if total <= 0 then return 0 end
  return ((s % total) + total) % total
end
local function catmull(a,b,c,d,u)
  local u2,u3=u*u,u*u*u
  return 0.5*((2*b)+(-a+c)*u+(2*a-5*b+4*c-d)*u2+(-a+3*b-3*c+d)*u3)
end
local function derivative(a,b,c,d,u)
  local u2=u*u
  return 0.5*((-a+c)+2*(2*a-5*b+4*c-d)*u+3*(-a+3*b-3*c+d)*u2)
end
function Geometry.build(nodes, loopRoute, subdivisions)
  local count=#nodes
  if count < 2 then return nil end
  local steps=math.max(8, math.floor(subdivisions or 16))
  local function nodeAt(i)
    if loopRoute then return nodes[((i-1)%count)+1] end
    return nodes[clamp(i,1,count)]
  end
  local samples={}
  local segCount=loopRoute and count or count-1
  for seg=1,segCount do
    local p0,p1,p2,p3=nodeAt(seg-1),nodeAt(seg),nodeAt(seg+1),nodeAt(seg+2)
    for j=0,steps do
      if seg==1 and j==0 or j>0 then
        local u=j/steps
        local x=catmull(p0.x,p1.x,p2.x,p3.x,u)
        local z=catmull(p0.z,p1.z,p2.z,p3.z,u)
        local y=catmull(p0.y,p1.y,p2.y,p3.y,u)
        local tx,tz=normalize(derivative(p0.x,p1.x,p2.x,p3.x,u),
                              derivative(p0.z,p1.z,p2.z,p3.z,u))
        local prev=samples[#samples]
        if not prev or (x-prev.x)^2+(z-prev.z)^2>1e-10 then
          samples[#samples+1]={x=x,y=y,z=z,tx=tx,tz=tz,seg=seg,t=u,s=0,k=0}
        end
      end
    end
  end
  if #samples<2 then return nil end
  local total=0
  for i=1,#samples do
    if i>1 then
      local a,b=samples[i-1],samples[i]
      total=total+math.sqrt((b.x-a.x)^2+(b.z-a.z)^2)
    end
    samples[i].s=total
  end
  if loopRoute then
    local a,b=samples[#samples],samples[1]
    local closing=math.sqrt((b.x-a.x)^2+(b.z-a.z)^2)
    if closing>1e-5 then
      total=total+closing
      samples[#samples+1]={x=b.x,y=b.y,z=b.z,tx=b.tx,tz=b.tz,seg=1,t=0,s=total,k=0}
    end
  end
  if total<1 then return nil end
  if loopRoute then
    for i=1,#samples-1 do
      local prev=i==1 and samples[#samples-1] or samples[i-1]
      local nxt=samples[i+1]
      local prevS=i==1 and (prev.s-total) or prev.s
      local ds=nxt.s-prevS
      if ds>1e-5 then
        samples[i].k=angle(prev.tx,prev.tz,nxt.tx,nxt.tz)/ds
      end
    end
    samples[#samples].k=samples[1].k
  else
    for i=1,#samples do
      local prev=samples[math.max(1,i-1)]
      local nxt=samples[math.min(#samples,i+1)]
      local ds=nxt.s-prev.s
      if ds>1e-5 then
        samples[i].k=angle(prev.tx,prev.tz,nxt.tx,nxt.tz)/ds
      end
    end
  end
  return {samples=samples,total=total,loop=loopRoute==true,nodeCount=count,steps=steps}
end
function Geometry.sample(route, distance)
  if not route then return nil end
  local total=route.total
  local s=route.loop and wrap(distance,total) or clamp(distance,0,total)
  local a,b=route.samples[1],route.samples[#route.samples]
  local lo,hi=1,#route.samples
  while lo<=hi do
    local mid=math.floor((lo+hi)*0.5)
    if route.samples[mid].s<=s then lo=mid+1 else hi=mid-1 end
  end
  local i=clamp(hi,1,#route.samples-1)
  a,b=route.samples[i],route.samples[i+1]
  local span=math.max(1e-8,b.s-a.s)
  local t=clamp((s-a.s)/span,0,1)
  local tx,tz=normalize(a.tx+(b.tx-a.tx)*t,a.tz+(b.tz-a.tz)*t)
  local seg,segT
  if a.seg==b.seg then
    seg,segT=a.seg,a.t+(b.t-a.t)*t
  elseif t<0.5 then
    seg,segT=a.seg,a.t
  else
    seg,segT=b.seg,b.t
  end
  return {
    x=a.x+(b.x-a.x)*t,y=a.y+(b.y-a.y)*t,z=a.z+(b.z-a.z)*t,
    tx=tx,tz=tz,k=a.k+(b.k-a.k)*t,
    s=s,seg=seg,t=segT
  }
end
function Geometry.distanceAt(route, segment, parameter)
  if not route or not segment then return nil end
  local samples=route.samples
  local u=clamp(parameter or 0,0,1)
  local before,after
  for i=1,#samples do
    local sample=samples[i]
    if sample.seg==segment then
      if sample.t<=u then before=sample end
      if sample.t>=u then after=sample; break end
    end
  end
  if not before then return after and after.s or nil end
  if not after then return before.s end
  local span=after.t-before.t
  local t=span>1e-8 and clamp((u-before.t)/span,0,1) or 0
  return before.s+(after.s-before.s)*t
end
function Geometry.project(route, position, hintS, heading, speed, dt, locked)
  if not route then return nil end
  local samples,total=route.samples,route.total
  local best,bestScore=nil,math.huge
  local maxForward=math.max(12,(speed or 0)*(dt or 0)*4+6)
  local back=3.5
  local function scanEdge(i)
    if i<1 or i>=#samples then return end
    local a,b=samples[i],samples[i+1]
    local vx,vz=b.x-a.x,b.z-a.z
    local lenSq=vx*vx+vz*vz
    if lenSq<=1e-10 then return end
    local t=clamp(((position.x-a.x)*vx+(position.z-a.z)*vz)/lenSq,0,1)
    local px,pz=a.x+vx*t,a.z+vz*t
    local dx,dz=position.x-px,position.z-pz
    local dSq=dx*dx+dz*dz
    local s=a.s+(b.s-a.s)*t
    local delta=0
    if locked and hintS~=nil then
      delta=s-hintS
      if route.loop then
        if delta>total*0.5 then delta=delta-total end
        if delta< -total*0.5 then delta=delta+total end
      end
      if delta < -back or delta > maxForward then return end
    end
    local tx,tz=normalize(vx,vz)
    local score=dSq
    if heading then
      local hx,hz=normalize(heading.x,heading.z)
      local alignment=hx*tx+hz*tz
      score=score+2.0*(1-alignment)*(1-alignment)
    end
    if score<bestScore then
      bestScore=score
      local seg,segT
      if a.seg==b.seg then
        seg,segT=a.seg,a.t+(b.t-a.t)*t
      elseif t<0.5 then
        seg,segT=a.seg,a.t
      else
        seg,segT=b.seg,b.t
      end
      best={s=s,distance=math.sqrt(dSq),lateral=dx*(-tz)+dz*tx,
        seg=seg,t=segT,tx=tx,tz=tz}
    end
  end
  local function lowerBound(value)
    local lo,hi=1,#samples
    while lo<hi do
      local mid=math.floor((lo+hi)*0.5)
      if samples[mid].s<value then lo=mid+1 else hi=mid end
    end
    return lo
  end
  local function scanDistanceRange(low,high)
    local first=math.max(1,lowerBound(low)-1)
    local last=math.min(#samples-1,lowerBound(high))
    for i=first,last do scanEdge(i) end
  end
  if locked and hintS~=nil then
    if route.loop then
      local center=wrap(hintS,total)
      local low,high=center-back,center+maxForward
      if high-low>=total then
        for i=1,#samples-1 do scanEdge(i) end
      elseif low<0 then
        scanDistanceRange(0,high)
        scanDistanceRange(total+low,total)
      elseif high>total then
        scanDistanceRange(low,total)
        scanDistanceRange(0,high-total)
      else
        scanDistanceRange(low,high)
      end
    else
      scanDistanceRange(math.max(0,hintS-back),math.min(total,hintS+maxForward))
    end
  else
    for i=1,#samples-1 do scanEdge(i) end
  end
  if not best then return nil end
  if route.loop and hintS~=nil and locked then
    local delta=best.s-hintS
    if delta>total*0.5 then best.delta=delta-total
    elseif delta< -total*0.5 then best.delta=delta+total
    else best.delta=delta end
  elseif hintS~=nil and locked then best.delta=best.s-hintS
  else best.delta=0 end
  return best
end

-- Smooth racing-line offset. Positive curvature is a left turn, so the
-- preferred offset is to the outside (negative lateral offset), and vice versa.
-- Curvature is averaged over a multi-scale window to ignore recorded-point noise.
function Geometry.racingOffset(route, distance, maxOffset, gain)
  if not route then return 0 end
  local limit=math.max(0,maxOffset or 0)
  if limit<=1e-4 then return 0 end
  local scales={{-24,0.08},{-16,0.16},{-8,0.24},{0,0.28},{8,0.24},{16,0.16},{24,0.08}}
  local weighted,totalWeight=0,0
  for i=1,#scales do
    local sample=Geometry.sample(route,distance+scales[i][1])
    if sample then
      weighted=weighted+(sample.k or 0)*scales[i][2]
      totalWeight=totalWeight+scales[i][2]
    end
  end
  local curvature=totalWeight>0 and weighted/totalWeight or 0
  local desired=-curvature*(gain or 28.0)
  return clamp(desired,-limit,limit)
end

return Geometry
