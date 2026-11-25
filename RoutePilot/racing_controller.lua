-- RoutePilot/racing_controller.lua
-- Offline racing-line optimizer and curvature evaluator.
local Racing = {}
local Geometry = require("route_geometry")
local function clamp(v,a,b) return math.max(a,math.min(b,v)) end
local function normalize(x,z)
  local l=math.sqrt(x*x+z*z)
  if l<1e-8 then return 0,1 end
  return x/l,z/l
end
local function pointAt(route, waypoint, offset)
  local nx,nz=-waypoint.tz,waypoint.tx
  return waypoint.x+nx*offset,waypoint.z+nz*offset
end
local function routeBounds(route,nodes,boundsAt,sample)
  local count=#nodes
  local i=clamp(sample.seg or 1,1,count)
  local j=i<count and i+1 or (route.loop and 1 or i)
  local a,b=nodes[i],nodes[j]
  local l0,r0=boundsAt(i)
  if not a or not b or i==j then return l0,r0 end
  local l1,r1=boundsAt(j)
  local rawTx,rawTz=normalize(b.x-a.x,b.z-a.z)
  local rawNx,rawNz=-rawTz,rawTx
  local u=clamp(sample.t or 0,0,1)
  local cx=a.x+(b.x-a.x)*u
  local cz=a.z+(b.z-a.z)*u
  local left=l0+(l1-l0)*u
  local right=r0+(r1-r0)*u
  local snx,snz=-sample.tz,sample.tx
  local lo=(cx+rawNx*left-sample.x)*snx+(cz+rawNz*left-sample.z)*snz
  local hi=(cx+rawNx*right-sample.x)*snx+(cz+rawNz*right-sample.z)*snz
  return math.min(lo,hi),math.max(lo,hi)
end
local function getIndex(line,i)
  local n=#line.points
  if line.loop then return ((i-1)%n)+1 end
  return clamp(i,1,n)
end
local function localEnergy(line,i,candidate)
  local n=#line.points
  if not line.loop and (i<=1 or i>=n) then return math.huge end
  local function offset(k)
    if k==i then return candidate end
    return line.offsets[getIndex(line,k)]
  end
  local function second(k)
    local a,b,c=line.points[getIndex(line,k-1)],line.points[getIndex(line,k)],line.points[getIndex(line,k+1)]
    local ax,az=pointAt(line.route,a,offset(k-1))
    local bx,bz=pointAt(line.route,b,offset(k))
    local cx,cz=pointAt(line.route,c,offset(k+1))
    return ax-2*bx+cx,az-2*bz+cz
  end
  local energy=0
  for k=i-1,i+1 do
    if line.loop or (k>1 and k<n) then
      local x,z=second(k)
      energy=energy+x*x+z*z
    end
  end
  local prev,next=line.points[getIndex(line,i-1)],line.points[getIndex(line,i+1)]
  local oPrev,oNext=offset(i-1),offset(i+1)
  local currentS=line.points[getIndex(line,i)].s
  local dsPrev=currentS-prev.s
  local dsNext=next.s-currentS
  if line.loop and dsPrev<0 then dsPrev=dsPrev+line.route.total end
  if line.loop and dsNext<0 then dsNext=dsNext+line.route.total end
  dsPrev=math.max(0.1,dsPrev)
  dsNext=math.max(0.1,dsNext)
  energy=energy+0.10*((candidate-oPrev)^2+(oNext-candidate)^2)
  energy=energy+0.0005*candidate*candidate
  local x1,z1=pointAt(line.route,prev,oPrev)
  local x2,z2=pointAt(line.route,line.points[getIndex(line,i)],candidate)
  local x3,z3=pointAt(line.route,next,oNext)
  local l1=math.sqrt((x2-x1)^2+(z2-z1)^2)
  local l2=math.sqrt((x3-x2)^2+(z3-z2)^2)
  energy=energy+0.015*((l1-dsPrev)^2+(l2-dsNext)^2)
  return energy
end
function Racing.build(route,nodes,boundsAt,loopRoute)
  if not route or not nodes or #nodes<2 then return nil end
  local spacing=3.0
  local count=math.max(2,math.floor(route.total/spacing)+1)
  if not loopRoute then count=math.max(2,math.ceil(route.total/spacing)+1) end
  local line={route=route,loop=loopRoute==true,points={},offsets={},minimum={},maximum={},spacing=spacing}
  for i=1,count do
    local s=math.min(route.total,(i-1)*spacing)
    if loopRoute and i==count then break end
    local sample=Geometry.sample(route,s)
    if sample then
      local lo,hi=routeBounds(route,nodes,boundsAt,sample)
      -- Keep the complete vehicle body inside the recorded corridor.
      lo,hi=lo+1.30,hi-1.30
      if hi<lo then
        local mid=(lo+hi)*0.5
        lo,hi=mid,mid
      end
      line.points[#line.points+1]=sample
      line.minimum[#line.minimum+1]=lo
      line.maximum[#line.maximum+1]=hi
      line.offsets[#line.offsets+1]=clamp(0,lo,hi)
    end
  end
  if #line.points<3 then return nil end
  -- Seed a classical outside -> apex -> outside line. The curvature
  -- envelope identifies the corner's sustained apex region: entry/exit
  -- move outward, while the high-curvature middle moves toward the inside.
  -- This avoids the zero-offset local minimum of a purely local search.
  for i=1,#line.points do
    local sum,weight,peak=0,0,0
    for delta=-4,4 do
      local j=i+delta
      if line.loop then j=getIndex(line,j) end
      if j>=1 and j<=#line.points then
        local w=5-math.abs(delta)
        local k=line.points[j].k or 0
        sum=sum+k*w
        weight=weight+w
        peak=math.max(peak,math.abs(k))
      end
    end
    local meanCurvature=weight>0 and sum/weight or 0
    local magnitude=math.abs(meanCurvature)
    local seed=0
    if magnitude>0.0015 and peak>0.0015 then
      local phase=2.0*clamp(magnitude/peak,0,1)^2-1.0
      local amplitude=clamp(magnitude*18.0,0.35,1.6)
      seed=(meanCurvature>0 and 1 or -1)*amplitude*phase
    end
    if not line.loop and (i==1 or i==#line.points) then seed=0 end
    line.offsets[i]=clamp(seed,line.minimum[i],line.maximum[i])
  end
  -- Coordinate descent minimizes integrated path curvature, path-length
  -- growth and lateral oscillation, subject to the usable road corridor.
  local steps={1.0,0.5,0.25,0.125}
  for _,step in ipairs(steps) do
    for sweep=1,12 do
      local first,last,increment=2,#line.points-1,1
      if line.loop then first,last=1,#line.points end
      if sweep%2==0 then first,last,increment=last,first,-1 end
      local i=first
      while (increment>0 and i<=last) or (increment<0 and i>=last) do
        local lo,hi=line.minimum[i],line.maximum[i]
        local current=line.offsets[i]
        local best,bestCost=current,localEnergy(line,i,current)
        for delta=-step,step+0.0001,step do
          local candidate=clamp(current+delta,lo,hi)
          local cost=localEnergy(line,i,candidate)
          if cost<bestCost-1e-10 then best,bestCost=candidate,cost end
        end
        line.offsets[i]=best
        i=i+increment
      end
    end
  end
  line.maxAbsOffset=0
  for i=1,#line.offsets do
    line.maxAbsOffset=math.max(line.maxAbsOffset,math.abs(line.offsets[i]))
  end
  return line
end
function Racing.offsetAt(line,s)
  if not line or #line.points<2 then return 0 end
  local total=line.route.total
  if line.loop and total>0 then s=((s%total)+total)%total else s=clamp(s,0,total) end
  local points=line.points
  local lo,hi=1,#points
  while lo<=hi do
    local mid=math.floor((lo+hi)*0.5)
    if points[mid].s<=s then lo=mid+1 else hi=mid-1 end
  end
  local a=getIndex(line,clamp(hi,1,#points))
  local b=getIndex(line,a+1)
  local sa,sb=points[a].s,points[b].s
  if line.loop and b==1 then sb=total end
  local span=math.max(1e-6,sb-sa)
  local t=clamp((s-sa)/span,0,1)
  return line.offsets[a]+(line.offsets[b]-line.offsets[a])*t
end
local function pathPoint(line,s,overrideOffset)
  local sample=Geometry.sample(line.route,s)
  if not sample then return nil end
  local offset=overrideOffset
  if offset==nil then offset=Racing.offsetAt(line,s) end
  local nx,nz=-sample.tz,sample.tx
  return sample.x+nx*offset,sample.z+nz*offset
end
function Racing.curvatureAt(line,s,overrideOffset)
  if not line then return 0 end
  local ds=2.5
  local x0,z0=pathPoint(line,s-ds,overrideOffset)
  local x1,z1=pathPoint(line,s,overrideOffset)
  local x2,z2=pathPoint(line,s+ds,overrideOffset)
  if not x0 or not x1 or not x2 then return 0 end
  local ax,az=x1-x0,z1-z0
  local bx,bz=x2-x1,z2-z1
  local la=math.sqrt(ax*ax+az*az)
  local lb=math.sqrt(bx*bx+bz*bz)
  if la<0.05 or lb<0.05 then return 0 end
  local cross=ax*bz-az*bx
  local dot=ax*bx+az*bz
  return math.atan2(cross,dot)/math.max(0.1,(la+lb)*0.5)
end
return Racing
