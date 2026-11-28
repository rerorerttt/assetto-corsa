-- Topology-aware grouping of parallel directed CSP lanes and corridor boundaries.
-- A pair is grouped only when direction, geometric overlap and road topology agree.
local Roads = {}

local function dist2(a, b)
  local dx, dy, dz = a.x-b.x, a.y-b.y, a.z-b.z
  return dx*dx + dy*dy + dz*dz
end

local function norm(x, y, z)
  local l = math.sqrt(x*x+y*y+z*z)
  if l < 1e-7 then return 0, 0, 0 end
  return x/l, y/l, z/l
end

local function tangentAt(points, i, loop)
  local n = #points
  local a = points[math.max(1, i-1)]
  local b = points[math.min(n, i+1)]
  if loop then
    a = points[(i-2) % n + 1]
    b = points[i % n + 1]
  end
  return norm(b.x-a.x, b.y-a.y, b.z-a.z)
end

local function nearestOnLane(p, lane)
  local pts, bestD, bestP, bestDir = lane.points, math.huge, nil, nil
  local n = #pts
  local count = lane.loop and n or n - 1
  local step = math.max(1, math.floor(n / 1000))
  for i = 1, count, step do
    local nextIndex = i + step
    if lane.loop then nextIndex = (nextIndex - 1) % n + 1
    else nextIndex = math.min(n, nextIndex) end
    local a, b = pts[i], pts[nextIndex]
    local vx, vy, vz = b.x-a.x, b.y-a.y, b.z-a.z
    local den = vx*vx+vy*vy+vz*vz
    if den > 1e-8 then
      local t = ((p.x-a.x)*vx+(p.y-a.y)*vy+(p.z-a.z)*vz)/den
      t = math.max(0, math.min(1, t))
      local q = vec3(a.x+vx*t, a.y+vy*t, a.z+vz*t)
      local d = dist2(p, q)
      if d < bestD then
        bestD, bestP = d, q
        bestDir = { norm(vx, vy, vz) }
      end
    end
  end
  return bestD < math.huge and math.sqrt(bestD) or nil, bestP, bestDir
end

local function sharedTopology(a, b)
  local ids = {}
  for i = 1, #(a.graphLinks or {}) do
    local link = a.graphLinks[i]
    ids[link.intersection] = true
  end
  for i = 1, #(b.graphLinks or {}) do
    local link = b.graphLinks[i]
    if ids[link.intersection] then return true end
  end
  -- Intersection-free stretches can still be the same road if both directed
  -- endpoints align; this deliberately rejects parallel roads side-by-side.
  if a.loop or b.loop or #a.points < 2 or #b.points < 2 then return false end
  local a0, a1 = a.points[1], a.points[#a.points]
  local b0, b1 = b.points[1], b.points[#b.points]
  return dist2(a0, b0) <= 64 and dist2(a1, b1) <= 64
end

local function pairMatches(a, b)
  if not sharedTopology(a, b) then return false end
  local ba, bb = a.graphBounds, b.graphBounds
  if ba and bb then
    if ba.maxX + 12 < bb.minX or bb.maxX + 12 < ba.minX or
        ba.maxZ + 12 < bb.minZ or bb.maxZ + 12 < ba.minZ then
      return false
    end
  end
  local function coverage(src, dst)
    local n, good, sum = #src.points, 0, 0
    local step = math.max(1, math.floor(n / 60))
    for i = 1, n, step do
      local p = src.points[i]
      local d, q, dir = nearestOnLane(p, dst)
      if d and q and d >= 1.5 and d <= 12 and math.abs(p.y - q.y) <= 1.5 then
        local sx, sy, sz = tangentAt(src.points, i, src.loop)
        local dot = sx*dir[1] + sy*dir[2] + sz*dir[3]
        if dot >= 0.92 then good, sum = good + 1, sum + d end
      end
    end
    local total = math.ceil(n / step)
    return total > 0 and good / total or 0, good > 0 and sum/good or math.huge
  end
  local ca, da = coverage(a, b)
  local cb, db = coverage(b, a)
  return ca >= 0.65 and cb >= 0.65 and math.abs(da-db) <= 2.5
end

local function median(values)
  table.sort(values)
  local n = #values
  if n == 0 then return 0 end
  if n % 2 == 1 then return values[(n+1)/2] end
  return (values[n/2] + values[n/2+1]) * 0.5
end

local function lateralOffset(reference, candidate)
  local offsets = {}
  local pts = reference.points
  local step = math.max(1, math.floor(#pts / 40))
  for i = 1, #pts, step do
    local p = pts[i]
    local _, q = nearestOnLane(p, candidate)
    if q then
      local tx, _, tz = tangentAt(pts, i, reference.loop)
      local leftX, leftZ = -tz, tx
      offsets[#offsets+1] = (q.x-p.x)*leftX + (q.z-p.z)*leftZ
    end
  end
  return median(offsets)
end

local function wallDistance(position, direction, maxDistance)
  if not render or type(render.createRay) ~= 'function' then return nil end
  local best = nil
  for _, height in ipairs({ 0.35, 0.85, 1.35 }) do
    local origin = vec3(position.x, position.y + height, position.z)
    local hit, normal = vec3(), vec3()
    local ok, distance = pcall(function()
      local ray = render.createRay(origin, direction, maxDistance)
      return ray:physics(hit, normal, false, false)
    end)
    if ok and type(distance) == 'number' and distance > 0.15 and distance < maxDistance and
        math.abs(normal.y) <= 0.65 then
      if not best or distance < best then best = distance end
    end
  end
  return best
end

local function makeBoundary(lane, side, offset)
  local out, pts, directions, distances = {}, lane.points, {}, {}
  local adjusted, conflicts = 0, 0
  for i = 1, #pts do
    local tx, ty, tz = tangentAt(pts, i, lane.loop)
    local normal = lane.normals and lane.normals[i]
    local nx, ny, nz = 0, 1, 0
    if normal then nx, ny, nz = normal.x, normal.y, normal.z end
    -- Tangent × surface normal gives the lane's left vector.
    local lx = ty*nz - tz*ny
    local ly = tz*nx - tx*nz
    local lz = tx*ny - ty*nx
    lx, ly, lz = norm(lx, ly, lz)
    if lx == 0 and ly == 0 and lz == 0 then lx, ly, lz = -tz, 0, tx end
    local dir = vec3(lx*side, ly*side, lz*side)
    local wall = wallDistance(pts[i], dir, offset + 1.0)
    local distance = offset
    if wall and wall < offset then
      distance = math.max(0.20, wall - 0.40)
      adjusted = adjusted + 1
      if wall <= 0.60 then conflicts = conflicts + 1 end
    end
    directions[i], distances[i] = dir, distance
  end
  -- Smooth only inward: a neighboring wall constraint can pull the corridor
  -- inward, but smoothing can never push a boundary through a detected wall.
  for i = 1, #pts do
    local prevIndex = i > 1 and i - 1 or (lane.loop and #pts or i)
    local nextIndex = i < #pts and i + 1 or (lane.loop and 1 or i)
    local average = (distances[prevIndex] + 2*distances[i] + distances[nextIndex]) * 0.25
    local distance = math.min(distances[i], average)
    local dir = directions[i]
    out[i] = vec3(
      pts[i].x + dir.x*distance,
      pts[i].y + dir.y*distance,
      pts[i].z + dir.z*distance)
  end
  return out, adjusted, conflicts
end

local function segmentIntersection(a, b, c, d)
  local rx, rz = b.x-a.x, b.z-a.z
  local sx, sz = d.x-c.x, d.z-c.z
  local den = rx*sz-rz*sx
  if math.abs(den) < 1e-7 then return nil end
  local qx, qz = c.x-a.x, c.z-a.z
  local t = (qx*sz-qz*sx)/den
  local u = (qx*rz-qz*rx)/den
  if t < -1e-5 or t > 1+1e-5 or u < -1e-5 or u > 1+1e-5 then return nil end
  t, u = math.max(0, math.min(1, t)), math.max(0, math.min(1, u))
  local yA, yB = a.y+(b.y-a.y)*t, c.y+(d.y-c.y)*u
  if math.abs(yA-yB) > 1.25 then return nil end
  return t, u, vec3(
    (a.x+rx*t+c.x+sx*u)*0.5,
    (yA+yB)*0.5,
    (a.z+rz*t+c.z+sz*u)*0.5)
end

local function lineBounds(points)
  local b = { minX=math.huge, maxX=-math.huge, minZ=math.huge, maxZ=-math.huge }
  for i = 1, #points do
    local p = points[i]
    b.minX, b.maxX = math.min(b.minX,p.x), math.max(b.maxX,p.x)
    b.minZ, b.maxZ = math.min(b.minZ,p.z), math.max(b.maxZ,p.z)
  end
  return b
end

local function boundsOverlap(a, b)
  return a.maxX >= b.minX and b.maxX >= a.minX and
    a.maxZ >= b.minZ and b.maxZ >= a.minZ
end

local function mergeBoundaryCrossings(lines)
  local events, junctions = {}, 0
  for i = 1, #lines do events[lines[i].points] = {} end
  for i = 1, #lines do
    local a = lines[i]
    local boundsA = lineBounds(a.points)
    local countA = a.loop and #a.points or #a.points-1
    for j = i+1, #lines do
      local b = lines[j]
      if a.group ~= b.group then
        local boundsB = lineBounds(b.points)
        if boundsOverlap(boundsA, boundsB) then
          local countB = b.loop and #b.points or #b.points-1
          for ai = 1, countA do
            local a1, a2 = a.points[ai], a.points[ai % #a.points + 1]
            local aminX, amaxX = math.min(a1.x,a2.x), math.max(a1.x,a2.x)
            local aminZ, amaxZ = math.min(a1.z,a2.z), math.max(a1.z,a2.z)
            for bi = 1, countB do
              local b1, b2 = b.points[bi], b.points[bi % #b.points + 1]
              if math.max(aminX,math.min(b1.x,b2.x)) <= math.min(amaxX,math.max(b1.x,b2.x)) and
                  math.max(aminZ,math.min(b1.z,b2.z)) <= math.min(amaxZ,math.max(b1.z,b2.z)) then
                local ta, tb, point = segmentIntersection(a1,a2,b1,b2)
                if ta then
                  local function addEvent(list, index, t, p)
                    for k = 1, #list do
                      if list[k].index == index and math.abs(list[k].t-t) < 0.02 then return end
                    end
                    list[#list+1] = { index=index, t=t, point=p }
                  end
                  addEvent(events[a.points], ai, ta, point)
                  addEvent(events[b.points], bi, tb, point)
                  junctions = junctions + 1
                end
              end
            end
          end
        end
      end
    end
  end
  for i = 1, #lines do
    local line, list = lines[i], events[lines[i].points]
    if #list > 0 then
      local bySegment = {}
      for j = 1, #list do
        local e = list[j]
        bySegment[e.index] = bySegment[e.index] or {}
        bySegment[e.index][#bySegment[e.index]+1] = e
      end
      local rebuilt = {}
      local count = line.loop and #line.points or #line.points-1
      for j = 1, count do
        rebuilt[#rebuilt+1] = line.points[j]
        local additions = bySegment[j]
        if additions then
          table.sort(additions, function(a,b) return a.t < b.t end)
          for k = 1, #additions do rebuilt[#rebuilt+1] = additions[k].point end
        end
      end
      if not line.loop then rebuilt[#rebuilt+1] = line.points[#line.points] end
      line.group.boundaries[line.side] = rebuilt
    end
  end
  return junctions
end

function Roads.build(lanes, log)
  local valid = {}
  for i = 1, #lanes do if lanes[i].valid then valid[#valid+1] = lanes[i] end end
  local parent = {}
  for i = 1, #valid do parent[i] = i end
  local function root(i)
    while parent[i] ~= i do parent[i] = parent[parent[i]]; i = parent[i] end
    return i
  end
  local function unite(a, b)
    local ra, rb = root(a), root(b)
    if ra ~= rb then parent[rb] = ra end
  end
  for i = 1, #valid do
    for j = i+1, #valid do
      if pairMatches(valid[i], valid[j]) then unite(i, j) end
    end
  end
  local groups, byRoot = {}, {}
  for i = 1, #valid do
    local r = root(i)
    if not byRoot[r] then
      byRoot[r] = { id = #groups+1, lanes = {}, boundaries = {} }
      groups[#groups+1] = byRoot[r]
    end
    byRoot[r].lanes[#byRoot[r].lanes+1] = valid[i]
  end
  local wallAdjusted, wallConflicts = 0, 0
  for i = 1, #groups do
    local group = groups[i]
    local reference = group.lanes[1]
    for j = 2, #group.lanes do
      if #group.lanes[j].points > #reference.points then reference = group.lanes[j] end
    end
    local leftLane, rightLane = reference, reference
    local leftOffset, rightOffset = 0, 0
    for j = 1, #group.lanes do
      local lane = group.lanes[j]
      local offset = lane == reference and 0 or lateralOffset(reference, lane)
      if offset > leftOffset then leftOffset, leftLane = offset, lane end
      if offset < rightOffset then rightOffset, rightLane = offset, lane end
    end
    group.reference = reference
    group.leftLane, group.rightLane = leftLane, rightLane
    group.boundaryOffset = #group.lanes == 1 and 3.0 or 2.5
    group.leftOffset, group.rightOffset = leftOffset, rightOffset
    local leftBoundary, leftAdjusted, leftConflicts = makeBoundary(leftLane, 1, group.boundaryOffset)
    local rightBoundary, rightAdjusted, rightConflicts = makeBoundary(rightLane, -1, group.boundaryOffset)
    group.boundaries = { left = leftBoundary, right = rightBoundary }
    group.boundaryLoops = { left = leftLane.loop, right = rightLane.loop }
    group.wallAdjusted = leftAdjusted + rightAdjusted
    group.wallConflicts = leftConflicts + rightConflicts
    wallAdjusted = wallAdjusted + group.wallAdjusted
    wallConflicts = wallConflicts + group.wallConflicts
  end
  local boundaryLines = {}
  for i = 1, #groups do
    local group = groups[i]
    boundaryLines[#boundaryLines+1] = {
      group = group, side = 'left', points = group.boundaries.left,
      loop = group.boundaryLoops.left
    }
    boundaryLines[#boundaryLines+1] = {
      group = group, side = 'right', points = group.boundaries.right,
      loop = group.boundaryLoops.right
    }
  end
  local junctions = mergeBoundaryCrossings(boundaryLines)

  if log then
    local multi = 0
    for i = 1, #groups do if #groups[i].lanes > 1 then multi = multi + 1 end end
    log('CSP_ROADS_GROUPED valid_lanes=%d groups=%d multi_lane_groups=%d wall_adjusted=%d wall_conflicts=%d boundary_junctions=%d',
      #valid, #groups, multi, wallAdjusted, wallConflicts, junctions)
  end
  return { groups = groups, stats = {
    validLanes = #valid, groups = #groups,
    wallAdjusted = wallAdjusted, wallConflicts = wallConflicts, boundaryJunctions = junctions
  } }
end

return Roads
