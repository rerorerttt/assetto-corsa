-- RoutePilot/trajectory_optimizer.lua
-- Full-lap racing-line search and vehicle-specific speed profiling.
-- The optimizer minimizes estimated lap time, not route length.
local Optimizer = {}

local function clamp(v, lo, hi)
  return math.max(lo, math.min(hi, v))
end

local function safe(v, fallback)
  if type(v) ~= "number" or v ~= v or v == math.huge or v == -math.huge then
    return fallback or 0
  end
  return v
end

local function pointAt(sample, offset, distance)
  return {
    x = sample.x - sample.tz * offset,
    z = sample.z + sample.tx * offset,
    offset = offset,
    s = distance or sample.s
  }
end

local function curvature(a, b, c)
  local abx, abz = b.x-a.x, b.z-a.z
  local bcx, bcz = c.x-b.x, c.z-b.z
  local cax, caz = a.x-c.x, a.z-c.z
  local ab = math.sqrt(abx*abx+abz*abz)
  local bc = math.sqrt(bcx*bcx+bcz*bcz)
  local ca = math.sqrt(cax*cax+caz*caz)
  if ab < 0.05 or bc < 0.05 or ca < 0.05 then return 0 end
  return 2*(abx*bcz-abz*bcx)/math.max(0.001,ab*bc*ca)
end

local function fits(a, b, sample, offset, halfWidth, halfLength, lo, hi)
  local dx,dz=b.x-a.x,b.z-a.z
  local length=math.sqrt(dx*dx+dz*dz)
  if length<0.05 then return false end
  dx,dz=dx/length,dz/length
  local along=math.abs(dx*sample.tx+dz*sample.tz)
  local across=math.abs(dx*sample.tz-dz*sample.tx)
  local envelope=halfWidth*along+halfLength*across+0.38
  local extra=math.max(0,envelope-(halfWidth+0.38))
  return offset>=lo+extra and offset<=hi-extra
end

local function edgeDistance(plan,index)
  return plan.edgeDistances and plan.edgeDistances[index] or plan.spacing
end

local function makeSpeedProfile(plan, lateralAccel, brakeAccel, accel)
  local n=#plan.points
  if n<2 then return nil end
  lateralAccel=clamp(safe(lateralAccel,10.0),3.0,25.0)
  brakeAccel=clamp(safe(brakeAccel,8.0),2.5,25.0)
  -- A low-speed sample taken during cornering, wheelspin or partial throttle
  -- must not become the acceleration limit for the entire lap. The driver
  -- supplies a filtered straight-line estimate with a conservative floor.
  accel=clamp(safe(accel,5.0),2.5,20.0)
  local speeds={}
  local absCurvature={}
  for i=1,n do absCurvature[i]=math.abs(safe(plan.curvature[i])) end
  local function speedCurvature(i)
    local previous,nextIndex
    if plan.loop then
      previous=i==1 and n or i-1
      nextIndex=i==n and 1 or i+1
    else
      previous=math.max(1,i-1)
      nextIndex=math.min(n,i+1)
    end
    -- Smooth isolated spline/recording noise without cancelling an S-bend:
    -- average absolute curvature, not signed curvature.
    return absCurvature[i]*0.50+
      (absCurvature[previous]+absCurvature[nextIndex])*0.25
  end
  for i=1,n do
    local k=speedCurvature(i)
    speeds[i]=k>0.00035 and math.min(110.0,math.sqrt(lateralAccel/k)) or 110.0
  end
  if plan.loop then
    -- Sweep two complete laps in both directions so a slow corner constrains
    -- its true braking approach, including when it lies just after s=0.
    for _=1,2 do
      for k=2*n,1,-1 do
        local i=((k-1)%n)+1
        local nextIndex=i==n and 1 or i+1
        speeds[i]=math.min(speeds[i],math.sqrt(speeds[nextIndex]^2+
          2*brakeAccel*edgeDistance(plan,i)))
      end
      for k=1,2*n do
        local i=((k-1)%n)+1
        local previous=i==1 and n or i-1
        speeds[i]=math.min(speeds[i],math.sqrt(speeds[previous]^2+
          2*accel*edgeDistance(plan,previous)))
      end
    end
  else
    speeds[n]=0
    for i=n-1,1,-1 do
      speeds[i]=math.min(speeds[i],math.sqrt(speeds[i+1]^2+
        2*brakeAccel*edgeDistance(plan,i)))
    end
    for i=2,n do
      speeds[i]=math.min(speeds[i],math.sqrt(speeds[i-1]^2+
        2*accel*edgeDistance(plan,i-1)))
    end
  end
  return speeds
end

local function estimateTime(plan, speeds)
  local total=0
  local n=#plan.points
  local edges=plan.loop and n or n-1
  for i=1,edges do
    local j=i==n and 1 or i+1
    local mean=(speeds[i]+speeds[j])*0.5
    total=total+edgeDistance(plan,i)/math.max(1.0,mean)
  end
  return total
end

local function retime(plan, lateralAccel, brakeAccel, accel)
  local speeds=makeSpeedProfile(plan,lateralAccel,brakeAccel,accel)
  if not speeds then return nil end
  return speeds
end

local function boundsForSample(sample, nodes, loopRoute, boundsAt)
  local count=#nodes
  local i=clamp(sample.seg or 1,1,loopRoute and count or count-1)
  local j=i<count and i+1 or (loopRoute and 1 or i)
  local a,b=nodes[i],nodes[j]
  local bl0,br0=boundsAt(i)
  local bl1,br1=boundsAt(j)
  if not a or not b or i==j then return bl0,br0 end
  local dx,dz=b.x-a.x,b.z-a.z
  local length=math.sqrt(dx*dx+dz*dz)
  if length<0.001 then return bl0,br0 end
  local tx,tz=dx/length,dz/length
  local nx,nz=-tz,tx
  local u=clamp(safe(sample.t),0,1)
  local cx=a.x+dx*u
  local cz=a.z+dz*u
  local left=bl0+(bl1-bl0)*u
  local right=br0+(br1-br0)*u
  local snx,snz=-sample.tz,sample.tx
  local lo=(cx+nx*left-sample.x)*snx+(cz+nz*left-sample.z)*snz
  local hi=(cx+nx*right-sample.x)*snx+(cz+nz*right-sample.z)*snz
  return math.min(lo,hi),math.max(lo,hi)
end

local function solveVariant(samples, bounds, loopRoute, halfWidth, halfLength,
    lateralAccel, anchorFraction)
  local stationCount=#samples
  local uniqueCount=loopRoute and stationCount-1 or stationCount
  local candidates={}
  local anchor=nil
  for i=1,stationCount do
    local lo,hi=bounds[i][1],bounds[i][2]
    local margin=halfWidth+0.38
    lo,hi=lo+margin,hi-margin
    if lo>hi then return nil end
    local list={}
    if i==1 then
      anchor=lo+(hi-lo)*anchorFraction
      list[1]=anchor
    elseif loopRoute and i==stationCount then
      list[1]=clamp(anchor,lo,hi)
      if math.abs(list[1]-anchor)>0.25 then return nil end
    else
      for c=0,6 do list[#list+1]=lo+(hi-lo)*c/6 end
    end
    candidates[i]=list
  end

  local function makePoint(station,candidateIndex)
    return pointAt(samples[station],candidates[station][candidateIndex],
      samples[station].s)
  end

  local layer={}
  for j=1,#candidates[2] do
    local a=makePoint(1,1)
    local b=makePoint(2,j)
    if fits(a,b,samples[2],candidates[2][j],halfWidth,halfLength,
        bounds[2][1]+halfWidth+0.38,bounds[2][2]-halfWidth-0.38) then
      local delta=candidates[2][j]-candidates[1][1]
      layer[#layer+1]={a=1,b=j,cost=delta*delta*0.004,prev=nil}
    end
  end
  if #layer==0 then return nil end

  for station=2,stationCount-1 do
    local nextLayer={}
    local bestByPair={}
    for _,node in ipairs(layer) do
      local pa=makePoint(station-1,node.a)
      local pb=makePoint(station,node.b)
      for c=1,#candidates[station+1] do
        local pc=makePoint(station+1,c)
        local k=curvature(pa,pb,pc)
        local lo=bounds[station][1]+halfWidth+0.38
        local hi=bounds[station][2]-halfWidth-0.38
        local nextLo=bounds[station+1][1]+halfWidth+0.38
        local nextHi=bounds[station+1][2]-halfWidth-0.38
        if fits(pa,pb,samples[station],candidates[station][node.b],
            halfWidth,halfLength,lo,hi) and
           fits(pb,pc,samples[station+1],candidates[station+1][c],
            halfWidth,halfLength,nextLo,nextHi) then
          local second=candidates[station-1][node.a]-
            2*candidates[station][node.b]+candidates[station+1][c]
          local slope=candidates[station+1][c]-candidates[station][node.b]
          local clearance=math.min(
            candidates[station+1][c]-nextLo,nextHi-candidates[station+1][c])
          local pathDistance=math.sqrt((pc.x-pb.x)^2+(pc.z-pb.z)^2)
          local localSpeed=math.min(110.0,math.sqrt(lateralAccel/
            math.max(0.00035,math.abs(k))))
          local timeCost=pathDistance/math.max(2.0,localSpeed)
          local cost=node.cost+timeCost+second*second*0.018+
            slope*slope*0.0015+0.001/math.max(0.08,clearance+0.08)
          local key=node.b*100+c
          local old=bestByPair[key]
          if not old or cost<old.cost then
            bestByPair[key]={a=node.b,b=c,cost=cost,prev=node}
          end
        end
      end
    end
    for _,node in pairs(bestByPair) do nextLayer[#nextLayer+1]=node end
    layer=nextLayer
    if #layer==0 then return nil end
  end

  local best=nil
  for _,node in ipairs(layer) do
    local pa=makePoint(stationCount-1,node.a)
    local pb=makePoint(stationCount,node.b)
    local lo=bounds[stationCount][1]+halfWidth+0.38
    local hi=bounds[stationCount][2]-halfWidth-0.38
    if fits(pa,pb,samples[stationCount],candidates[stationCount][node.b],
        halfWidth,halfLength,lo,hi) then
      local seamPenalty=0
      if loopRoute then
        local startOffset=candidates[1][1]
        local endOffset=candidates[stationCount][node.b]
        seamPenalty=(endOffset-startOffset)^2*0.1
      end
      local score=node.cost+seamPenalty
      if not best or score<best.cost then best={node=node,cost=score} end
    end
  end
  if not best then return nil end

  local offsets={}
  local node=best.node
  for station=stationCount,2,-1 do
    offsets[station]=candidates[station][node.b]
    offsets[station-1]=candidates[station-1][node.a]
    node=node.prev
    if not node then break end
  end
  local points={}
  for i=1,uniqueCount do points[i]=pointAt(samples[i],offsets[i],samples[i].s) end
  local plan={
    points=points,curvature={},edgeDistances={},loop=loopRoute,total=samples[stationCount].s,
    spacing=samples[2].s-samples[1].s,anchor=anchor
  }
  local n=#points
  local edgeCount=loopRoute and n or n-1
  for i=1,edgeCount do
    local j=i==n and 1 or i+1
    local dx,dz=points[j].x-points[i].x,points[j].z-points[i].z
    plan.edgeDistances[i]=math.sqrt(dx*dx+dz*dz)
  end
  for i=1,n do
    local prev=i==1 and (loopRoute and n or 1) or i-1
    local nxt=i==n and (loopRoute and 1 or n) or i+1
    if prev==i or nxt==i then
      plan.curvature[i]=0
    else
      plan.curvature[i]=curvature(points[prev],points[i],points[nxt])
    end
  end
  if not loopRoute then
    plan.curvature[1]=plan.curvature[2] or 0
    plan.curvature[n]=plan.curvature[n-1] or 0
  end
  return plan
end

function Optimizer.build(route, sampleFn, nodes, loopRoute, boundsAt, car, options)
  if not route or not sampleFn or not nodes or #nodes<2 or not boundsAt then
    return nil,"INVALID_ROUTE"
  end
  options=options or {}
  local spacing=clamp(safe(options.spacing,4.5),3.0,20.0)
  local halfWidth=math.max(0.75,safe(car and car.aabbSize and car.aabbSize.x,2.0)*0.5)
  local halfLength=math.max(1.5,safe(car and car.aabbSize and car.aabbSize.z,4.5)*0.5)
  local uniqueCount=loopRoute and math.max(24,math.ceil(route.total/spacing)) or
    math.max(3,math.ceil(route.total/spacing))
  local step=loopRoute and route.total/uniqueCount or route.total/uniqueCount
  local stationCount=loopRoute and uniqueCount+1 or uniqueCount+1
  local samples,bounds={},{}
  for i=0,uniqueCount do
    local distance=step*i
    local sample=sampleFn(route,distance)
    if not sample then return nil,"SAMPLE_FAILED" end
    sample.s=distance
    samples[i+1]=sample
    local left,right=boundsForSample(sample,nodes,loopRoute,boundsAt)
    bounds[i+1]={left,right}
  end

  local roadWidth=math.max(0,bounds[1][2]-bounds[1][1])
  local laneCount=clamp(math.floor((roadWidth+0.5)/3.5),1,4)
  local anchors={}
  -- Exactly one optimizer anchor per physical lane. Lane centres are spaced
  -- by the fixed 3.5 m lane width; leftover road width is split at the edges.
  for lane=1,laneCount do
    local centerOffset=(lane-(laneCount+1)*0.5)*3.5
    anchors[lane]=clamp((roadWidth*0.5+centerOffset)/roadWidth,0,1)
  end
  local alternatives={}
  for i=1,#anchors do
    local plan=solveVariant(samples,bounds,loopRoute,halfWidth,halfLength,
      clamp(safe(options.lateralAccel,10.0),3.0,25.0),anchors[i])
    if plan then
      plan.genericSpeeds=makeSpeedProfile(plan,options.lateralAccel,
        options.brakeAccel,options.accel)
      plan.estimatedTime=estimateTime(plan,plan.genericSpeeds)
      plan.anchorFraction=anchors[i]
      alternatives[#alternatives+1]=plan
    end
  end
  if #alternatives==0 then return nil,"NO_VALID_TRAJECTORY" end
  local best=alternatives[1]
  for i=2,#alternatives do
    if alternatives[i].estimatedTime<best.estimatedTime then best=alternatives[i] end
  end
  best.alternatives=alternatives
  best.alternativeCount=#alternatives
  best.routeLength=route.total
  best.vehicleHalfWidth=halfWidth
  best.vehicleHalfLength=halfLength
  return best
end

function Optimizer.retime(plan,lateralAccel,brakeAccel,accel)
  if not plan then return nil end
  return makeSpeedProfile(plan,lateralAccel,brakeAccel,accel)
end

-- Re-evaluate all precomputed line variants using this car's measured grip,
-- braking and acceleration. The selected line minimizes its predicted lap time.
local function offsetAt(plan,distance)
  local n=#plan.points
  local s=safe(distance)
  if plan.loop then s=((s%plan.total)+plan.total)%plan.total
  else s=clamp(s,0,plan.total) end
  local position=s/plan.spacing
  local index,t
  if not plan.loop and s>=plan.total then index=n-1; t=1
  else
    index=math.floor(position)+1
    t=position-math.floor(position)
    if plan.loop then index=((index-1)%n)+1
    else index=clamp(index,1,n-1) end
  end
  local nextIndex=index+1
  if nextIndex>n then nextIndex=plan.loop and 1 or n end
  return plan.points[index].offset+
    (plan.points[nextIndex].offset-plan.points[index].offset)*t
end

function Optimizer.select(bundle,lateralAccel,brakeAccel,accel,preferredOffset,progress)
  if not bundle then return nil,nil,nil end
  local alternatives=bundle.alternatives or {bundle}
  local best,bestSpeeds,bestTime=nil,nil,math.huge
  for i=1,#alternatives do
    local candidate=alternatives[i]
    local speeds=makeSpeedProfile(candidate,lateralAccel,brakeAccel,accel)
    if speeds then
      local time=estimateTime(candidate,speeds)
      local score=time
      -- Nearby traffic makes the closest safe lane variant preferable;
      -- with clear road, the score is pure predicted lap time.
      if type(preferredOffset)=="number" then
        score=score+math.abs(offsetAt(candidate,progress or 0)-preferredOffset)*0.8
      end
      if score<bestTime then
        best,bestSpeeds,bestTime=candidate,speeds,score
      end
    end
  end
  if not best then return bundle,makeSpeedProfile(bundle,lateralAccel,brakeAccel,accel),nil end
  return best,bestSpeeds,bestTime
end

function Optimizer.sample(plan,distance,speeds)
  if not plan or not plan.points or #plan.points<2 then return nil end
  local n=#plan.points
  local s=safe(distance)
  if plan.loop then
    s=((s%plan.total)+plan.total)%plan.total
  else
    s=clamp(s,0,plan.total)
  end
  local position=s/plan.spacing
  local index,t
  if not plan.loop and s>=plan.total then
    index=n-1
    t=1
  else
    index=math.floor(position)+1
    t=position-math.floor(position)
    if plan.loop then index=((index-1)%n)+1
    else index=clamp(index,1,n-1) end
  end
  local nextIndex=index+1
  if nextIndex>n then nextIndex=plan.loop and 1 or n end
  local a,b=plan.points[index],plan.points[nextIndex]
  local ka,kb=plan.curvature[index] or 0,plan.curvature[nextIndex] or 0
  local va=speeds and speeds[index] or plan.genericSpeeds[index] or 0
  local vb=speeds and speeds[nextIndex] or plan.genericSpeeds[nextIndex] or va
  return {
    x=a.x+(b.x-a.x)*t,z=a.z+(b.z-a.z)*t,
    offset=a.offset+(b.offset-a.offset)*t,
    k=ka+(kb-ka)*t,
    targetSpeed=va+(vb-va)*t,
    s=s
  }
end

return Optimizer
