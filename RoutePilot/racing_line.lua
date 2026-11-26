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
    fixedOffset, horizon, spacing, obstacleAt, previousPlan, previousAdvance,
    fixedTransitionDistance)
  if not route or not progress or not boundsAt then return nil end
  local size = car and car.aabbSize
  local halfWidth = math.max(0.75, safe(size and size.x, 2.0) * 0.5)
  local margin = halfWidth + 0.42
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
    local lo, hi = left + margin, right - margin
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

  local previousOffsets={}
  if previousPlan then
    for i=1,n do
      local previous=RacingLine.sample(previousPlan,(i-1)*step+
        math.max(0,safe(previousAdvance,0)))
      previousOffsets[i]=previous and previous.offset or nil
    end
  end
  local function stabilityCost(index,value)
    local previous=previousOffsets[index]
    if previous==nil then return 0 end
    local delta=value-previous
    return delta*delta*0.08
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
      local forbidden=obstacleAt and obstacleAt(samples[i],d) or nil
      local function isBlocked(value)
        for _,interval in ipairs(forbidden or {}) do
          if value>interval[1] and value<interval[2] then return true end
        end
        return false
      end
      for c=0,6 do
        local value=lo+(hi-lo)*c/6
        if not isBlocked(value) then addCandidate(list,value,lo,hi) end
      end
      if not isBlocked(preferred) then addCandidate(list,preferred,lo,hi) end
      if #list==0 then
        -- Fail closed: never manufacture an edge candidate inside an
        -- occupied corridor. Longitudinal following handles the blocked road.
        return nil,"NO_PATH",d
      end
      local previous=sampleFn(route,progress+math.max(0,d-step))
      local following=sampleFn(route,progress+d+step)
      local kp=safe(previous and previous.k)
      local kc=safe(samples[i].k)
      local kn=safe(following and following.k,kc)
      local smoothCurvature=(kp+2.0*kc+kn)*0.25
      local curvatureCandidate=clamp(
        preferred+clamp(-smoothCurvature*90.0,-2.8,2.8),lo,hi)
      if not isBlocked(curvatureCandidate) then
        addCandidate(list,curvatureCandidate,lo,hi)
      end
      candidateSets[i]=list
    end

    local firstLayer={}
    for j=1,#candidateSets[2] do
      local off0=candidateSets[1][1]
      local off1=candidateSets[2][j]
      local clearance=math.min(off1-bounds[2][1],bounds[2][2]-off1)
      local cost=(off1-off0)^2*0.02+
        0.00008/math.max(0.12,clearance+0.12)+
        stabilityCost(2,off1)
      firstLayer[#firstLayer+1]={a=1,b=j,cost=cost,prev=nil}
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
          local k=curvature(pointA,pointB,pointC)
          local reference=math.abs(safe(samples[station].k))
          local excess=math.max(0,math.abs(k)-math.max(reference*1.05,0.003))
          local second=offA-2.0*offB+offC
          local slope=offC-offB
          local clearance=math.min(offC-bounds[station+1][1],
            bounds[station+1][2]-offC)
          local cost=node.cost+k*k*180.0+excess*excess*200000.0+
            second*second*0.18+slope*slope*0.075+
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
  local curvatures = {}
  for i = 2, n - 1 do
    curvatures[i] = curvature(points[i - 1], points[i], points[i + 1])
  end
  curvatures[1] = curvatures[2] or 0
  curvatures[n] = curvatures[n - 1] or 0

  return {
    points = points,
    curvature = curvatures,
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