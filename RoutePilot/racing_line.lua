-- RoutePilot/racing_line.lua
-- Short-horizon corridor optimizer. It optimizes the actual driven path,
-- not the recorded reference spline. All candidate points are constrained
-- by the recorded physical road bounds and the vehicle footprint.
local RacingLine = {}

local function clamp(v, a, b)
  return math.max(a, math.min(b, v))
end

local function safe(v, fallback)
  if type(v) ~= "number" or v ~= v or v == math.huge or v == -math.huge then
    return fallback or 0
  end
  return v
end

local function pointAt(sample, offset)
  return {
    x = sample.x - sample.tz * offset,
    z = sample.z + sample.tx * offset,
    offset = offset,
    s = sample.s,
    sample = sample
  }
end

local function curvature(a, b, c)
  local abx, abz = b.x - a.x, b.z - a.z
  local bcx, bcz = c.x - b.x, c.z - b.z
  local cax, caz = a.x - c.x, a.z - c.z
  local ab = math.sqrt(abx * abx + abz * abz)
  local bc = math.sqrt(bcx * bcx + bcz * bcz)
  local ca = math.sqrt(cax * cax + caz * caz)
  if ab < 0.05 or bc < 0.05 or ca < 0.05 then return 0 end
  local cross = abx * bcz - abz * bcx
  return 2 * cross / math.max(0.001, ab * bc * ca)
end

-- boundsAt(sample) returns left/right offsets in the spline's local frame.
-- fixedOffset is used while passing: it disables racing-line optimization and
-- follows the reserved passing corridor instead.
function RacingLine.plan(route, sampleFn, progress, car, boundsAt, startOffset, baseOffset,
    fixedOffset, horizon, spacing, previousPlan, previousAdvance,
    fixedTransitionDistance,lateralAccelLimit)
  if not route or not progress or not boundsAt then return nil end
  local size = car and car.aabbSize
  local halfWidth = math.max(0.75, safe(size and size.x, 2.0) * 0.5)
  local halfLength = math.max(1.5, safe(size and size.z, 4.5) * 0.5)
  local margin = 0.22
  local lateralGrip=clamp(safe(lateralAccelLimit,10.0),6.0,20.0)
  local step = math.max(6.0, spacing or 7.5)
  local length = math.min(135.0, math.max(35.0, horizon or 95.0))
  local count = math.floor(length / step) + 1
  local samples, bounds, offsets = {}, {}, {}
  local blockedDistance=nil
  local initial = safe(startOffset, safe(baseOffset, 0))
  local preferred = safe(baseOffset, initial)

  for i = 0, count do
    local d = i * step
    local sample = sampleFn and sampleFn(route, progress + d) or nil
    if not sample then break end
    local left, right = boundsAt(sample)
    left, right = safe(left, -2.0), safe(right, 2.0)
    -- Approximate the lateral projection of the complete oriented body.
    -- The heading change across half a vehicle length captures cornering
    -- sweep; the body is not reduced to its centre point or width alone.
    local before = sampleFn(route, progress + d - halfLength)
    local after = sampleFn(route, progress + d + halfLength)
    local angle = 0
    if before and after then
      local hx, hz = after.x - before.x, after.z - before.z
      local hl = math.sqrt(hx * hx + hz * hz)
      if hl > 0.05 then
        hx, hz = hx / hl, hz / hl
        angle = math.abs(math.atan2(hx * sample.tz - hz * sample.tx,
          hx * sample.tx + hz * sample.tz))
      end
    end
    local bodyEnvelope = halfWidth * math.abs(math.cos(angle)) +
      halfLength * math.abs(math.sin(angle)) + margin
    local lo, hi = left + bodyEnvelope, right - bodyEnvelope
    if lo > hi then
      -- A near-field corridor that cannot contain the car is an immediate
      -- hard stop. A distant narrowing becomes a braking constraint instead.
      if d <= 10.0 then return nil, "NO_FIT" end
      blockedDistance=blockedDistance or d
      local center=(left+right)*0.5
      lo,hi=center,center
    end
    samples[#samples + 1] = sample
    bounds[#bounds + 1] = { lo, hi }
  end

  local n = #samples
  if n < 3 then return nil end

  -- Check the full oriented rectangle at multiple points along every swept
  -- segment. The centre path alone is never accepted as proof of clearance.
  local bodySampleCache={}
  local bodyBoundsCache={}
  local function bodyFitsAt(distance,point,hx,hz)
    local headingLength=math.sqrt(hx*hx+hz*hz)
    if headingLength<0.001 then return false end
    hx,hz=hx/headingLength,hz/headingLength
    local checked=0
    -- Five longitudinal slices cover the front, rear and intervening sweep
    -- of the complete rectangle while it travels between planner stations.
    for slice=-2,2 do
      local longitudinal=slice*halfLength*0.5
      local bodyDistance=distance+longitudinal
      local sample=bodySampleCache[bodyDistance]
      if sample==nil then
        sample=sampleFn(route,progress+bodyDistance) or false
        bodySampleCache[bodyDistance]=sample
      end
      if sample~=false then
        local bodyX=point.x+hx*longitudinal
        local bodyZ=point.z+hz*longitudinal
        local angle=math.abs(math.atan2(hx*sample.tz-hz*sample.tx,
          hx*sample.tx+hz*sample.tz))
        local envelope=halfWidth*math.abs(math.cos(angle))+margin
        local nx,nz=-sample.tz,sample.tx
        local offset=(bodyX-sample.x)*nx+(bodyZ-sample.z)*nz
        local roadBounds=bodyBoundsCache[bodyDistance]
        if not roadBounds then
          local left,right=boundsAt(sample)
          roadBounds={safe(left,-2.0),safe(right,2.0)}
          bodyBoundsCache[bodyDistance]=roadBounds
        end
        local left,right=roadBounds[1],roadBounds[2]
        if offset<left+envelope or offset>right-envelope then return false end
        checked=checked+1
      end
    end
    return checked>0
  end

  local previousOffsets={}
  if previousPlan then
    for i=1,n do
      local previous=RacingLine.sample(previousPlan,(i-1)*step+
        math.max(0,safe(previousAdvance,0)))
      previousOffsets[i]=previous and previous.offset or nil
    end
  end
  -- Preserve the previous path strongly on an open straight. The previous
  -- offset is also inserted into each candidate set below, so the optimizer
  -- can actually retain it instead of choosing the nearest coarse grid point.
  local straightOpen=previousPlan~=nil
  if straightOpen then
    for i=1,n do
      local sample=samples[i]
      if math.abs(safe(sample and sample.k))>=0.0012 then
        straightOpen=false
        break
      end
    end
  end
  -- A previous line is a warm start, not a straight-line leash. Keep only
  -- enough temporal weight to avoid visible lateral chatter while allowing
  -- the optimizer to cut small reference-spline kinks.
  local stabilityWeight=straightOpen and 0.08 or 0.22
  local function stabilityCost(index,value)
    local previous=previousOffsets[index]
    if previous==nil then return 0 end
    local delta=value-previous
    return delta*delta*stabilityWeight
  end

  if fixedOffset ~= nil then
    for i=1,n do
      local d=(i-1)*step
      local t=clamp(d/math.max(22.0,safe(fixedTransitionDistance,22.0)),0,1)
      t=t*t*(3.0-2.0*t)
      local desired=initial+(safe(fixedOffset,initial)-initial)*t
      offsets[i]=clamp(desired,bounds[i][1],bounds[i][2])
    end
  else
    -- Dynamic programming over pairs of lateral candidates. Each transition
    -- evaluates the curvature of the real offset path, so the search can
    -- compare outside-in-out lines globally instead of getting trapped by
    -- one-point coordinate-descent minima.
    local candidateSets={}
    candidateSets[1]={clamp(initial,bounds[1][1],bounds[1][2])}
    local function addCandidate(list,value,lo,hi)
      local v=clamp(value,lo,hi)
      for j=1,#list do
        if math.abs(list[j]-v)<0.04 then return end
      end
      list[#list+1]=v
    end
    for i=2,n do
      local lo,hi=bounds[i][1],bounds[i][2]
      local list={}
      local d=(i-1)*step
      for c=0,6 do
        local value=lo+(hi-lo)*c/6
        addCandidate(list,value,lo,hi)
      end
      addCandidate(list,preferred,lo,hi)
      -- Keep the exact previous lateral position available. Without this
      -- candidate, temporal cost alone cannot prevent hopping between grid slots.
      local previousOffset=previousOffsets[i]
      if previousOffset~=nil then addCandidate(list,previousOffset,lo,hi) end
      local previous=sampleFn(route,progress+math.max(0,d-step))
      local following=sampleFn(route,progress+d+step)
      local kp=safe(previous and previous.k)
      local kc=safe(samples[i].k)
      local kn=safe(following and following.k,kc)
      local smoothCurvature=(kp+2.0*kc+kn)*0.25
      local curvatureCandidate=clamp(
        preferred+clamp(-smoothCurvature*90.0,-2.8,2.8),lo,hi)
      addCandidate(list,curvatureCandidate,lo,hi)
      candidateSets[i]=list
    end

    local firstLayer={}
    for j=1,#candidateSets[2] do
      local off0=candidateSets[1][1]
      local off1=candidateSets[2][j]
      local pointA=pointAt(samples[1],off0)
      local pointB=pointAt(samples[2],off1)
      local hx,hz=pointB.x-pointA.x,pointB.z-pointA.z
      local mid={x=(pointA.x+pointB.x)*0.5,z=(pointA.z+pointB.z)*0.5}
      if bodyFitsAt(step*0.5,mid,hx,hz) and
         bodyFitsAt(step,pointB,hx,hz) then
        local clearance=math.min(off1-bounds[2][1],bounds[2][2]-off1)
        local cost=(off1-off0)^2*0.02+
          0.00008/math.max(0.12,clearance+0.12)+
          stabilityCost(2,off1)
        firstLayer[#firstLayer+1]={a=1,b=j,cost=cost,prev=nil}
      end
    end
    local layer=firstLayer
    for station=2,n-1 do
      local nextLayer={}
      local bestByPair={}
      for _,node in ipairs(layer) do
        local offA=candidateSets[station-1][node.a]
        local offB=candidateSets[station][node.b]
        local pointA=pointAt(samples[station-1],offA)
        local pointB=pointAt(samples[station],offB)
        for c=1,#candidateSets[station+1] do
          local offC=candidateSets[station+1][c]
          local pointC=pointAt(samples[station+1],offC)
          local abx,abz=pointB.x-pointA.x,pointB.z-pointA.z
          local bcx,bcz=pointC.x-pointB.x,pointC.z-pointB.z
          local midAB={x=(pointA.x+pointB.x)*0.5,z=(pointA.z+pointB.z)*0.5}
          local midBC={x=(pointB.x+pointC.x)*0.5,z=(pointB.z+pointC.z)*0.5}
          local bodyClear=bodyFitsAt((station-1.5)*step,midAB,abx,abz) and
            bodyFitsAt((station-1)*step,pointB,pointC.x-pointA.x,pointC.z-pointA.z) and
            bodyFitsAt((station-0.5)*step,midBC,bcx,bcz) and
            bodyFitsAt(station*step,pointC,bcx,bcz)
          if bodyClear then
          local k=curvature(pointA,pointB,pointC)
          local reference=math.abs(safe(samples[station].k))
          local excess=math.max(0,math.abs(k)-math.max(reference*1.05,0.003))
          local second=offA-2.0*offB+offC
          local slope=offC-offB
          local clearance=math.min(offC-bounds[station+1][1],
            bounds[station+1][2]-offC)
          local segmentLength=math.sqrt(
            (pointC.x-pointB.x)^2+(pointC.z-pointB.z)^2)
          local estimatedCornerSpeed=clamp(
            math.sqrt(lateralGrip/math.max(math.abs(k),0.0008)),8.0,90.0)
          local segmentTime=segmentLength/estimatedCornerSpeed
          local extraLength=math.max(0,segmentLength-step)
          local cost=node.cost+segmentTime*8.0+k*k*45.0+
            excess*excess*200000.0+second*second*0.18+
            slope*slope*0.075+extraLength*0.35+
            0.00008/math.max(0.12,clearance+0.12)+
            stabilityCost(station+1,offC)
          local key=node.b*100+c
          local existing=bestByPair[key]
          if not existing or cost<existing.cost then
            local nextNode={a=node.b,b=c,cost=cost,prev=node}
            bestByPair[key]=nextNode
          end
          end
        end
      end
      for _,node in pairs(bestByPair) do nextLayer[#nextLayer+1]=node end
      layer=nextLayer
      if #layer==0 then return nil, "NO_PATH",station*step end
    end

    local best=nil
    for _,node in ipairs(layer) do
      if not best or node.cost<best.cost then best=node end
    end
    if not best then return nil, "NO_PATH",length end
    local node,station=best,n
    while node and station>=2 do
      offsets[station]=candidateSets[station][node.b]
      offsets[station-1]=candidateSets[station-1][node.a]
      node=node.prev
      station=station-1
    end
    offsets[1]=candidateSets[1][1]
  end

  local points = {}
  for i = 1, n do
    points[i] = pointAt(samples[i], offsets[i])
  end
  -- Fixed pass/yield/return corridors bypass the dynamic-programming search,
  -- but they are held to the same full-body swept-path test.
  if fixedOffset ~= nil then
    for i=1,n-1 do
      local a,b=points[i],points[i+1]
      local hx,hz=b.x-a.x,b.z-a.z
      local mid={x=(a.x+b.x)*0.5,z=(a.z+b.z)*0.5}
      if not bodyFitsAt((i-0.5)*step,mid,hx,hz) or
         not bodyFitsAt(i*step,b,hx,hz) then
        return nil,"NO_PATH",i*step
      end
    end
  end
  local curvatures = {}
  for i = 2, n - 1 do
    curvatures[i] = curvature(points[i - 1], points[i], points[i + 1])
  end
  curvatures[1] = curvatures[2] or 0
  curvatures[n] = curvatures[n - 1] or 0

  -- Classify connected road features over the whole lookahead window. An
  -- S-section is detected as a sign reversal, not as two unrelated corners.
  local feature={}
  for i=1,n do
    local k=safe(curvatures[i])
    local kind=math.abs(k)>=0.12 and "HAIRPIN" or
      (math.abs(k)<0.0012 and "STRAIGHT" or
      (math.abs(k)<0.025 and "FAST_BEND" or "CORNER"))
    feature[i]=kind
  end
  for i=2,n-1 do
    local a,bk=curvatures[i-1] or 0,curvatures[i+1] or 0
    if a*bk< -0.00004 and math.abs(a)>0.001 and math.abs(bk)>0.001 then
      feature[i]="S_BEND"
    end
  end

  return {
    points = points,
    curvature = curvatures,
    feature = feature,
    spacing = step,
    count = n,
    horizon = (n - 1) * step,
    fixed = fixedOffset ~= nil,
    blockedDistance = blockedDistance
  }
end

function RacingLine.sample(plan, distance)
  if not plan or not plan.points or plan.count < 2 then return nil end
  distance=safe(distance)
  if distance<0 or distance>plan.horizon then return nil end
  local usableDistance=plan.blockedDistance and
    math.max(0,plan.blockedDistance-10.0) or distance
  local position = clamp(usableDistance / plan.spacing, 0, plan.count - 1)
  local index = math.floor(position) + 1
  local nextIndex = math.min(plan.count, index + 1)
  local t = position - math.floor(position)
  local a, b = plan.points[index], plan.points[nextIndex]
  local ca = plan.curvature[index] or 0
  local cb = plan.curvature[nextIndex] or ca
  return {
    x = a.x + (b.x - a.x) * t,
    z = a.z + (b.z - a.z) * t,
    offset = a.offset + (b.offset - a.offset) * t,
    k = ca + (cb - ca) * t
  }
end

return RacingLine