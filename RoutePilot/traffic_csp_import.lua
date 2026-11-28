-- RoutePilot CSP Traffic Tool importer.
-- Reads the active layout's traffic.json without modifying RoutePilot's recorded route.
-- Surface projection follows CSP Traffic Tool's own EditorLaneExporter convention.

local json = require('lib/json')
local okGraph, Graph = pcall(require, 'traffic_csp_graph')
if not okGraph or type(Graph) ~= 'table' or type(Graph.build) ~= 'function' then Graph = nil end
local okRoads, Roads = pcall(require, 'traffic_csp_roads')
if not okRoads or type(Roads) ~= 'table' or type(Roads.build) ~= 'function' then Roads = nil end
local okSurfaces, Surfaces = pcall(require, 'shared/sim/surfaces')
if not okSurfaces or type(Surfaces) ~= 'table' then Surfaces = nil end
local Importer = {}
Importer.__index = Importer

local DOWN = vec3(0, -1, 0)
local SAMPLE_STEP = 1.5
local RAY_START_OFFSET = 2.0
local RAY_LENGTH = 4.0
local MAX_SURFACE_SNAP = 1.5
local SURFACE_EDGE_STEP = 0.25
local SURFACE_EDGE_MAX = 8.0
local SURFACE_EDGE_GAP = 6.0
local MIN_LANE_POINTS = 2

local function finite(v)
  return type(v) == 'number' and v == v and v ~= math.huge and v ~= -math.huge
end

local function point3(p)
  if type(p) ~= 'table' then return nil end
  local x, y, z = tonumber(p[1]), tonumber(p[2]), tonumber(p[3])
  if not finite(x) or not finite(y) or not finite(z) then return nil end
  return vec3(x, y, z)
end

local function catmull(a, b, c, d, t)
  local t2, t3 = t * t, t * t * t
  local function v(p, q, r, s)
    return 0.5 * ((2 * q) + (-p + r) * t +
      (2 * p - 5 * q + 4 * r - s) * t2 +
      (-p + 3 * q - 3 * r + s) * t3)
  end
  return vec3(v(a.x, b.x, c.x, d.x),
    v(a.y, b.y, c.y, d.y), v(a.z, b.z, c.z, d.z))
end

local function sourcePoint(points, i, loop)
  local n = #points
  if loop then
    return points[((i - 1) % n) + 1]
  end
  return points[math.max(1, math.min(n, i))]
end

local function resample(points, loop)
  local count = #points
  if count < (loop and 3 or MIN_LANE_POINTS) then return nil end
  local out = {}
  local segmentCount = loop and count or count - 1
  for i = 1, segmentCount do
    local a = sourcePoint(points, i - 1, loop)
    local b = sourcePoint(points, i, loop)
    local c = sourcePoint(points, i + 1, loop)
    local d = sourcePoint(points, i + 2, loop)
    local chord = b:distance(c)
    local steps = math.max(1, math.ceil(chord / SAMPLE_STEP))
    for j = 0, steps - 1 do
      local t = j / steps
      -- Keep every segment start. Skipping these points shortens the spline
      -- and shifts every later sample away from its source lane geometry.
      out[#out + 1] = j == 0 and b:clone() or catmull(a, b, c, d, t)
    end
  end
  if not loop then out[#out + 1] = points[count]:clone() end
  return out
end

local function projectToTrack(p)
  if not physics or type(physics.raycastTrack) ~= 'function' then
    return nil, nil, 'raycastTrack_unavailable'
  end
  local origin = p:clone()
  origin.y = origin.y + RAY_START_OFFSET
  local normal = vec3()
  local ok, distance = pcall(physics.raycastTrack, origin, DOWN, RAY_LENGTH, nil, normal)
  if not ok then return nil, nil, tostring(distance) end
  if not finite(distance) or distance == -1 then return nil, nil, nil, 'no_surface' end
  local surfaceType = Surfaces and Surfaces.raycastType(origin, DOWN, RAY_LENGTH) or nil
  if surfaceType == 'grass' or surfaceType == 'sand' or surfaceType == 'extraturf' then
    return nil, nil, surfaceType, 'non_road_surface'
  end
  if not finite(normal.y) or normal.y < 0.30 then
    return nil, nil, surfaceType or 'steep_surface', 'invalid_surface_normal'
  end
  local projected = p:clone()
  projected.y = origin.y - distance
  if math.abs(projected.y - p.y) > MAX_SURFACE_SNAP then
    return nil, nil, surfaceType or 'unclassified', 'surface_height_mismatch'
  end
  return projected, normal, surfaceType or 'unclassified', nil
end

local function driveableSurface(surfaceType, centerType)
  if centerType == 'gravel' then
    return surfaceType == 'gravel' or surfaceType == 'kerb'
  end
  return surfaceType == 'default' or surfaceType == 'old' or surfaceType == 'kerb'
end

local function surfaceAtOffset(point, dx, dz, offset)
  if not Surfaces or not physics or type(physics.raycastTrack) ~= 'function' then return nil end
  local origin = vec3(point.x + dx*offset, point.y + RAY_START_OFFSET,
    point.z + dz*offset)
  local normal = vec3()
  local ok, hit = pcall(physics.raycastTrack, origin, DOWN, RAY_LENGTH, nil, normal)
  if not ok or not finite(hit) or hit == -1 or not finite(normal.y) or normal.y < 0.30 then
    return nil
  end
  if math.abs((origin.y-hit)-point.y) > MAX_SURFACE_SNAP then return nil end
  return Surfaces.raycastType(origin, DOWN, RAY_LENGTH)
end

local function measureSideWidth(point, dx, dz, centerType)
  local lastDriveable = 0
  local steps = math.floor(SURFACE_EDGE_MAX/SURFACE_EDGE_STEP)
  for step = 1, steps do
    local offset = step*SURFACE_EDGE_STEP
    local surfaceType = surfaceAtOffset(point, dx, dz, offset)
    if driveableSurface(surfaceType, centerType) then
      lastDriveable = offset
    else
      return lastDriveable >= 1.0 and lastDriveable or nil
    end
  end
  -- No physical surface transition was found within the probe range. Do not
  -- invent a road edge from a nominal lane width.
  return nil
end

local function fillBoundaryGaps(widths, loop)
  local n = #widths
  local original = {}
  for i = 1, n do original[i] = widths[i] end
  local maxSteps = math.max(1, math.ceil(SURFACE_EDGE_GAP/SAMPLE_STEP))
  for i = 1, n do
    if not widths[i] then
      local conservative = math.huge
      for radius = 1, maxSteps do
        local before, after = i-radius, i+radius
        if loop then
          before = (before-1)%n+1
          after = (after-1)%n+1
        end
        if before >= 1 and before <= n and original[before] then
          conservative = math.min(conservative, original[before])
        end
        if after >= 1 and after <= n and original[after] then
          conservative = math.min(conservative, original[after])
        end
      end
      if conservative < math.huge then widths[i] = conservative else return false end
    end
  end
  return true
end

local function measureLaneSurfaceBounds(lane)
  if not Surfaces or #lane.points < 2 then return false end
  local leftWidths, rightWidths = {}, {}
  local n = #lane.points
  -- Keep the arrays dense while probing alternate samples; Lua's length
  -- operator is undefined for sparse tables with holes.
  for i = 1, n do leftWidths[i], rightWidths[i] = false, false end
  for i = 1, n do
    -- Probe every 3 m along the lane; missing samples are filled only from
    -- nearby verified widths and always use the narrower neighbor.
    if i % 2 == 1 or i == n then
    local previousIndex = i > 1 and i-1 or (lane.loop and n or 1)
    local nextIndex = i < n and i+1 or (lane.loop and 1 or n)
    local a, b = lane.points[previousIndex], lane.points[nextIndex]
    local tx, ty, tz = b.x-a.x, b.y-a.y, b.z-a.z
    local tangentLength = math.sqrt(tx*tx+ty*ty+tz*tz)
    if tangentLength < 1e-6 then return false end
    tx, ty, tz = tx/tangentLength, ty/tangentLength, tz/tangentLength
    local normal = lane.normals[i]
    local nx, ny, nz = normal.x, normal.y, normal.z
    local lx = ty*nz - tz*ny
    local ly = tz*nx - tx*nz
    local lz = tx*ny - ty*nx
    local leftLength = math.sqrt(lx*lx+ly*ly+lz*lz)
    if leftLength < 1e-6 then return false end
    lx, ly, lz = lx/leftLength, ly/leftLength, lz/leftLength
    local centerType = lane.surfaceTypes[i]
    if not driveableSurface(centerType, centerType) then return false end
    leftWidths[i] = measureSideWidth(lane.points[i], lx, lz, centerType) or false
    rightWidths[i] = measureSideWidth(lane.points[i], -lx, -lz, centerType) or false
    end
  end
  local leftValid = fillBoundaryGaps(leftWidths, lane.loop)
  local rightValid = fillBoundaryGaps(rightWidths, lane.loop)
  lane.surfaceBounds = {
    left = leftWidths, right = rightWidths,
    leftValid = leftValid, rightValid = rightValid
  }
  lane.surfaceBoundsValid = leftValid and rightValid
  return lane.surfaceBoundsValid
end

function Importer.new(log)
  return setmetatable({
    log = log or function() end,
    loaded = false,
    error = nil,
    filename = nil,
    data = nil,
    lanes = {},
    intersections = {},
    areas = {},
    graph = nil,
    graphError = nil,
    roads = nil,
    roadsError = nil,
    stats = { lanes = 0, usable = 0, invalid = 0, bounded = 0, unbounded = 0, samples = 0, hits = 0, misses = 0, trimmed = 0 },
  }, Importer)
end

function Importer:load()
  if self.loaded then return true end
  if not ac or type(ac.getTrackDataFilename) ~= 'function' then
    self.error = 'ac.getTrackDataFilename_unavailable'
    return false
  end
  local okName, filename = pcall(ac.getTrackDataFilename, 'traffic.json')
  if not okName or type(filename) ~= 'string' or filename == '' then
    self.error = 'traffic_json_path_unavailable'
    return false
  end
  self.filename = filename
  local okRead, raw = pcall(io.load, filename, '')
  if not okRead or type(raw) ~= 'string' or raw == '' then
    self.error = 'traffic_json_missing_or_unreadable'
    return false
  end
  local okDecode, data = pcall(json.decode, raw)
  if not okDecode or type(data) ~= 'table' or type(data.lanes) ~= 'table' then
    self.error = 'traffic_json_invalid: ' .. tostring(data)
    return false
  end

  self.data = data
  self.intersections = type(data.intersections) == 'table' and data.intersections or {}
  self.areas = type(data.areas) == 'table' and data.areas or {}
  local laneRoles = type(data.rules) == 'table' and type(data.rules.laneRoles) == 'table' and data.rules.laneRoles or {}
  self.stats.lanes = #data.lanes

  for i = 1, #data.lanes do
    local src = data.lanes[i]
    local malformedLane = type(src) ~= 'table'
    if malformedLane then src = {} end
    local roleIndex = tonumber(src.role) or 3
    local roleDefinition = type(laneRoles[roleIndex]) == 'table' and laneRoles[roleIndex] or nil
    local priority = tonumber(src.priority)
    if not finite(priority) then
      priority = tonumber(roleDefinition and roleDefinition.priority) or 0
      local priorityOffset = tonumber(src.priorityOffset)
      if finite(priorityOffset) then priority = priority + priorityOffset end
    end
    local sourceSpeedLimit = tonumber(src.speedLimit)
    local roleSpeedLimit = tonumber(roleDefinition and roleDefinition.speedLimit)
    local speedLimit = finite(sourceSpeedLimit) and sourceSpeedLimit or
      (finite(roleSpeedLimit) and roleSpeedLimit or 90)
    local lane = {
      id = src.id,
      name = src.name or ('Lane ' .. tostring(src.id or i)),
      loop = src.loop == true,
      role = roleIndex,
      priority = finite(priority) and priority or 0,
      speedLimit = speedLimit,
      speedLimitExplicit = finite(sourceSpeedLimit) or finite(roleSpeedLimit),
      params = type(src.params) == 'table' and src.params or {},
      source = src,
      points = {},
      normals = {},
      surfaceValid = {},
      surfaceTypes = {},
      valid = false,
    }
    local sourcePoints = {}
    local malformedSourcePoint = malformedLane
    if type(src.points) == 'table' then
      for j = 1, #src.points do
        local p = point3(src.points[j])
        if not p then
          malformedSourcePoint = true
          break
        end
        -- Duplicate editor points create zero-length spline segments and can
        -- destabilize Catmull-Rom tangents. Collapse only adjacent duplicates.
        if #sourcePoints == 0 or sourcePoints[#sourcePoints]:distance(p) > 0.05 then
          sourcePoints[#sourcePoints + 1] = p
        end
      end
    else
      malformedSourcePoint = true
    end
    if lane.loop and #sourcePoints > 2 and
        sourcePoints[1]:distance(sourcePoints[#sourcePoints]) <= 0.05 then
      sourcePoints[#sourcePoints] = nil
    end
    local samples = not malformedSourcePoint and resample(sourcePoints, lane.loop) or nil
    if malformedSourcePoint then
      lane.invalidReason = 'malformed_source_point'
    elseif not samples then
      lane.invalidReason = 'insufficient_source_points'
    end
    if samples then
      local laneHits, laneMisses = 0, 0
      for j = 1, #samples do
        local projected, normal, surfaceType = projectToTrack(samples[j])
        self.stats.samples = self.stats.samples + 1
        if projected then
          lane.points[#lane.points + 1] = projected
          lane.normals[#lane.normals + 1] = normal
          lane.surfaceValid[#lane.surfaceValid + 1] = true
          lane.surfaceTypes[#lane.surfaceTypes + 1] = surfaceType
          laneHits = laneHits + 1
          self.stats.hits = self.stats.hits + 1
        else
          lane.points[#lane.points + 1] = samples[j]
          lane.normals[#lane.normals + 1] = vec3(0, 1, 0)
          lane.surfaceValid[#lane.surfaceValid + 1] = false
          lane.surfaceTypes[#lane.surfaceTypes + 1] = surfaceType or 'no_surface'
          laneMisses = laneMisses + 1
          self.stats.misses = self.stats.misses + 1
        end
      end
      -- A lane with any unconfirmed surface sample is retained as source data,
      -- but explicitly excluded from navigation until its topology can be rebuilt
      -- without silently breaking intersection connections.
      lane.valid = laneHits >= MIN_LANE_POINTS and laneMisses == 0
      lane.surfaceHits = laneHits
      lane.surfaceMisses = laneMisses

      -- Open lanes can be shortened at unsupported ends. An interior gap is
      -- not bridged: doing so would create a false connection across missing road.
      if not lane.loop then
        local first, last
        for j = 1, #lane.surfaceValid do
          if lane.surfaceValid[j] then
            first = first or j
            last = j
          end
        end
        local contiguous = first ~= nil and last ~= nil and (last - first + 1) >= MIN_LANE_POINTS
        if contiguous then
          for j = first, last do
            if not lane.surfaceValid[j] then contiguous = false break end
            if j > first and math.abs(lane.points[j].y - lane.points[j - 1].y) > 2.5 then
              contiguous = false
              break
            end
          end
        end
        if contiguous then
          local trimmed = first > 1 or last < #lane.points
          local points, normals, valid, types = {}, {}, {}, {}
          for j = first, last do
            points[#points + 1] = lane.points[j]
            normals[#normals + 1] = lane.normals[j]
            valid[#valid + 1] = lane.surfaceValid[j]
            types[#types + 1] = lane.surfaceTypes[j]
          end
          lane.points, lane.normals = points, normals
          lane.surfaceValid, lane.surfaceTypes = valid, types
          lane.trimmedStart, lane.trimmedEnd = first - 1, #samples - last
          lane.valid = true
          if trimmed then self.stats.trimmed = self.stats.trimmed + 1 end
        else
          lane.valid = false
        end
      else
        local continuous = laneMisses == 0
        if continuous then
          for j = 2, #lane.points do
            if math.abs(lane.points[j].y - lane.points[j - 1].y) > 2.5 then
              continuous = false
              break
            end
          end
        end
        lane.valid = continuous and laneHits >= MIN_LANE_POINTS
      end
    end
    if lane.valid then
      self.stats.usable = self.stats.usable + 1
      if measureLaneSurfaceBounds(lane) then
        self.stats.bounded = self.stats.bounded + 1
      else
        self.stats.unbounded = self.stats.unbounded + 1
      end
    else
      self.stats.invalid = self.stats.invalid + 1
    end
    self.lanes[#self.lanes + 1] = lane
  end

  if Graph then
    local okBuild, graph = pcall(Graph.build, self.lanes, self.intersections, self.log)
    if okBuild then
      self.graph = graph
    else
      self.graphError = tostring(graph)
      self.log('CSP_GRAPH_ERROR err=%s', self.graphError)
    end
  else
    self.graphError = 'traffic_csp_graph_module_unavailable'
    self.log('CSP_GRAPH_UNAVAILABLE')
  end

  if self.graph and Roads then
    local okRoadBuild, roads = pcall(Roads.build, self.lanes, self.log)
    if okRoadBuild then
      self.roads = roads
    else
      self.roadsError = tostring(roads)
      self.log('CSP_ROADS_ERROR err=%s', self.roadsError)
    end
  elseif not Roads then
    self.roadsError = 'traffic_csp_roads_module_unavailable'
    self.log('CSP_ROADS_UNAVAILABLE')
  end

  self.loaded = true
  self.log('CSP_IMPORT_LOADED file=%s lanes=%d usable=%d invalid=%d samples=%d hits=%d misses=%d trimmed=%d intersections=%d areas=%d surface_type_api=%s graph=%s roads=%s',
    tostring(self.filename), self.stats.lanes, self.stats.usable, self.stats.invalid,
    self.stats.samples, self.stats.hits, self.stats.misses, self.stats.trimmed,
    #self.intersections, #self.areas, tostring(Surfaces ~= nil), tostring(self.graph ~= nil), tostring(self.roads ~= nil))
  return true
end

return Importer
