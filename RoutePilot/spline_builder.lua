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
local MIN_SPEED_KMH = 8.0
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

local SURFACE_RAY_START = 1.5
local SURFACE_RAY_LENGTH = 8.0
local SURFACE_LIFT = 0.3

-- Cast from just above the recorded car position, not 10 m above it.
-- A high ray can hit an upper bridge deck while the car is on a lower road.
-- A short local ray also prevents a failed cast from silently preserving a
-- bad 8–11 m height spike in the generated spline.
local function snapSurface(pos)
  local snapped = vec3(pos.x, pos.y, pos.z)
  local normal = vec3(0, 1, 0)
  if type(physics) == 'table' and type(physics.raycastTrack) == 'function' then
    local origin = vec3(pos.x, pos.y + SURFACE_RAY_START, pos.z)
    local ok, distance = pcall(
      physics.raycastTrack,
      origin, DOWN, SURFACE_RAY_LENGTH, snapped, normal
    )
    if ok and type(distance) == 'number' and distance >= 0 and
      distance <= SURFACE_RAY_LENGTH and normal.y > 0.25 and
      math.abs(snapped.y - pos.y) <= 2.5 then
      snapped.y = snapped.y + SURFACE_LIFT
      return snapped, normal, true
    end
  end
  return nil, normal, false
end

-- Missing ray hits are interpolated from the nearest valid road samples.
-- This is deliberately height-only: X/Z remain the exact recorded route.
local function resolveMissingSurfaceSamples(nodes, aiNodes, normals, valid)
  local count = #nodes
  local validCount = 0
  for i = 1, count do
    if valid[i] then validCount = validCount + 1 end
  end
  if validCount < 2 then
    error('track surface raycast failed: fewer than two valid samples')
  end

  local fallbackCount = count - validCount
  if fallbackCount > count * 0.40 then
    error(string.format(
      'track surface raycast failed for %.0f%% of route samples',
      fallbackCount / count * 100
    ))
  end

  for i = 1, count do
    if not valid[i] then
      local prev, nextPoint
      local prevDistance, nextDistance = 0, 0

      for step = 1, count - 1 do
        local from = ((i - step - 1) % count) + 1
        local to = from % count + 1
        prevDistance = prevDistance + flatDistance(nodes[from], nodes[to])
        if valid[from] then
          prev = from
          break
        end
      end

      for step = 1, count - 1 do
        local from = ((i + step - 2) % count) + 1
        local to = from % count + 1
        nextDistance = nextDistance + flatDistance(nodes[from], nodes[to])
        if valid[to] then
          nextPoint = to
          break
        end
      end

      if not prev or not nextPoint then
        error(string.format('cannot interpolate road height at route point %d', i))
      end

      local span = prevDistance + nextDistance
      local t = span > 0.001 and prevDistance / span or 0.5
      local y = aiNodes[prev].y + (aiNodes[nextPoint].y - aiNodes[prev].y) * t
      aiNodes[i] = vec3(nodes[i].x, y, nodes[i].z)
      normals[i] = vec3(0, 1, 0)
    end
  end

  return fallbackCount
end

local function sideWidths(boundsL, boundsR, index)
  local left = tonumber(boundsL and boundsL[index]) or -DEFAULT_SIDE
  local right = tonumber(boundsR and boundsR[index]) or DEFAULT_SIDE
  if left > 0 then left = -left end
  if right < 0 then right = -right end

  -- RoutePilot stores signed lateral bounds (left < 0, right > 0).
  -- AISplineEditor stores positive magnitudes on its forward × up frame,
  -- which matches RoutePilot's positive lateral direction.
  return math.max(0.1, right), math.max(0.1, -left)
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

  local ok, totalLength, diagnostics = pcall(function()
    local count = #nodes
    local closureGap=fullDistance(nodes[count],nodes[1])
    if closureGap>30.0 then
      error(string.format('route is not closed: start/end gap %.1f m (maximum 30 m)',closureGap))
    end
    local distances = {}
    local speeds = {}
    local radii = {}
    local aiNodes, normals, valid = {}, {}, {}
    for i = 1, count do
      local snapped, normal, hit = snapSurface(nodes[i])
      aiNodes[i], normals[i], valid[i] = snapped, normal, hit
    end
    local fallbackCount = resolveMissingSurfaceSamples(nodes, aiNodes, normals, valid)
    for i = 1, count do
      nodes[i].y = aiNodes[i].y
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
      -- fast_lane.ai stores speed in m/s, not km/h.
      payload.speed = speeds[i]

      -- Encode an aggressive throttle/braking profile from the same
      -- curvature-limited speed envelope. Flat and accelerating sections
      -- receive full throttle; deceleration sections receive a proportional
      -- brake hint. This avoids the previous all-zero pedal profile, which
      -- left every car stationary after being placed on the custom spline.
      local nextSpeed = speeds[nextIndex]
      local longitudinalAccel = (nextSpeed * nextSpeed - speeds[i] * speeds[i]) /
        math.max(0.1, 2 * distance)
      if longitudinalAccel < -0.25 then
        payload.gas = 0.0
        payload.brake = clamp(-longitudinalAccel / BRAKE_ACCEL, 0.0, 1.0)
      else
        payload.gas = 1.0
        payload.brake = 0.0
      end
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

    return totalLength, {
      surfaceHits = count - fallbackCount,
      surfaceFallbacks = fallbackCount,
      maxSpeedMs = maxSpeed
    }
  end)

  if not ok then return false, tostring(totalLength) end
  return true, totalLength, diagnostics
end

return Builder
