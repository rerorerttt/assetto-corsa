-- RoutePilot/spline_builder.lua
-- Builds a native CSP/Assetto Corsa version-7 AI spline from the recorded route.
-- Binary layout and StructItem definitions match AISplineEditor/AISpline.lua.
-- The native AI owns steering, throttle, braking and gearbox; this module only
-- prepares the line, speed hints and left/right road limits.

local binaryUtils = require('shared/utils/binary')

local splinePoint = ac.StructItem.combine({
  ac.StructItem.explicit(),
  pos = ac.StructItem.vec3(),
  length = ac.StructItem.float(),
  payloadIndex = ac.StructItem.int32(),
})

local splinePayload = ac.StructItem.combine({
  ac.StructItem.explicit(),
  speed = ac.StructItem.float(),
  gas = ac.StructItem.float(),
  brake = ac.StructItem.float(),
  obsoleteLatG = ac.StructItem.float(),
  radius = ac.StructItem.float(),
  sideLeft = ac.StructItem.float(),
  sideRight = ac.StructItem.float(),
  camber = ac.StructItem.float(),
  direction = ac.StructItem.float(),
  normal = ac.StructItem.vec3(),
  length = ac.StructItem.float(),
  forwardVector = ac.StructItem.vec3(),
  tag = ac.StructItem.float(),
  grade = ac.StructItem.float(),
})

local Builder = {}

local MAX_LATERAL_ACCEL = 10.5
local BRAKE_ACCEL = 12.0
local MIN_SPEED_KMH = 35.0
local DEFAULT_SIDE = 3.5
local UP = vec3(0, 1, 0)
local DOWN = vec3(0, -1, 0)

local function flatDistance(a, b)
  local dx, dz = b.x - a.x, b.z - a.z
  return math.sqrt(dx * dx + dz * dz)
end

local function fullDistance(a, b)
  local dx, dy, dz = b.x - a.x, b.y - a.y, b.z - a.z
  return math.sqrt(dx * dx + dy * dy + dz * dz)
end

local function curvatureAt(nodes, index)
  local count = #nodes
  local prev = nodes[(index - 2) % count + 1]
  local cur = nodes[index]
  local next = nodes[index % count + 1]

  local ax, az = cur.x - prev.x, cur.z - prev.z
  local bx, bz = next.x - cur.x, next.z - cur.z
  local la = math.sqrt(ax * ax + az * az)
  local lb = math.sqrt(bx * bx + bz * bz)
  if la < 0.05 or lb < 0.05 then return 0, 100000.0 end

  local dot = math.max(-1, math.min(1, (ax * bx + az * bz) / (la * lb)))
  local angle = math.acos(dot)
  local curvature = angle / math.max(0.1, (la + lb) * 0.5)
  if curvature < 0.00005 then return 0, 100000.0 end
  return curvature, 1 / curvature
end

local function snapSurface(pos)
  local snapped=vec3(pos.x,pos.y,pos.z)
  local normal=vec3(0,1,0)
  if physics and physics.raycastTrack then
    pcall(
      physics.raycastTrack,
      vec3(pos.x,pos.y+10.0,pos.z),
      DOWN,20.0,snapped,normal
    )
  end
  -- Match AISplineEditor: snap to the track surface and lift 0.3 m.
  snapped.y=snapped.y+0.3
  return snapped,normal
end

local function sideWidths(boundsL, boundsR, index)
  local left = tonumber(boundsL and boundsL[index]) or -DEFAULT_SIDE
  local right = tonumber(boundsR and boundsR[index]) or DEFAULT_SIDE
  if left > 0 then left = -left end
  if right < 0 then right = -right end
  return math.max(1.0, -left), math.max(1.0, right)
end

function Builder.build(nodes, boundsL, boundsR, filename, maxSpeedKmh, loopRoute)
  if type(nodes) ~= 'table' or #nodes < 3 then
    return false, 'at least three route points are required'
  end
  if type(filename) ~= 'string' or filename == '' then
    return false, 'AI spline filename is empty'
  end
  if loopRoute == false then
    return false, 'native RoutePilot spline currently requires a closed lap'
  end

  local ok, err = pcall(function()
    local count = #nodes
    local closureGap=fullDistance(nodes[count],nodes[1])
    if closureGap>30.0 then
      error(string.format('route is not closed: start/end gap %.1f m (maximum 30 m)',closureGap))
    end
    local distances = {}
    local speeds = {}
    local radii = {}
    local aiNodes, normals = {}, {}
    for i=1,count do
      aiNodes[i],normals[i]=snapSurface(nodes[i])
      nodes[i].y=aiNodes[i].y
    end
    local maxSpeed = math.max(MIN_SPEED_KMH, tonumber(maxSpeedKmh) or 300.0) / 3.6

    for i = 1, count do
      local nextIndex = i % count + 1
      local d = fullDistance(aiNodes[i], aiNodes[nextIndex])
      if d < 0.05 then
        error(string.format('duplicate or collapsed route segment at point %d', i))
      end
      distances[i] = d

      local curvature, radius = curvatureAt(aiNodes, i)
      radii[i] = radius
      local v = curvature > 0 and math.sqrt(MAX_LATERAL_ACCEL / curvature) or maxSpeed
      speeds[i] = math.max(MIN_SPEED_KMH / 3.6, math.min(maxSpeed, v))
    end

    -- Backward braking envelope. Two cyclic passes propagate corner limits
    -- around the complete loop without a quadratic point-by-point search.
    for _ = 1, 2 do
      for i = count, 1, -1 do
        local nextIndex = i % count + 1
        local allowed = math.sqrt(
          speeds[nextIndex] * speeds[nextIndex] +
          2 * BRAKE_ACCEL * distances[i]
        )
        if speeds[i] > allowed then speeds[i] = allowed end
      end
    end

    local points, payloads = {}, {}
    local totalLength = 0

    for i = 1, count do
      local nextIndex = i % count + 1
      local cur, nxt = nodes[i], nodes[nextIndex]
      local distance = distances[i]

      local point = ffi.new(splinePoint)
      point.pos:set(aiNodes[i])
      point.length = totalLength
      point.payloadIndex = i - 1

      local payload = ffi.new(splinePayload)
      payload.speed = speeds[i] * 3.6
      payload.gas = 0.0
      payload.brake = 0.0
      payload.obsoleteLatG = 0.0
      payload.radius = radii[i]
      payload.sideLeft, payload.sideRight = sideWidths(boundsL, boundsR, i)
      payload.camber = 0.0
      payload.direction = 0.0
      payload.normal:set(normals[i])
      payload.length = distance
      payload.forwardVector:set(aiNodes[nextIndex]):sub(aiNodes[i]):scale(1 / distance)
      payload.tag = 0.0
      payload.grade = 0.0

      points[i] = point
      payloads[i] = payload
      totalLength = totalLength + distance
    end

    binaryUtils.writeData()
      :int32(7):int32(count):int64(0)
      :array(points)
      :array(payloads, true)
      :int32(0)
      :commit(filename)
      :dispose()

    return totalLength
  end)

  if not ok then return false, tostring(err) end
  return true, err
end

return Builder
