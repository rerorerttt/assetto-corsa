-- RoutePilot CSP Traffic Tool importer.
-- Reads the active layout's traffic.json without modifying RoutePilot's recorded route.
-- Surface projection follows CSP Traffic Tool's own EditorLaneExporter convention.

local json = require('lib/json')
local okSurfaces, Surfaces = pcall(require, 'shared/sim/surfaces')
if not okSurfaces or type(Surfaces) ~= 'table' then Surfaces = nil end
local Importer = {}
Importer.__index = Importer

local DOWN = vec3(0, -1, 0)
local SAMPLE_STEP = 1.5
local RAY_START_OFFSET = 2.0
local RAY_LENGTH = 4.0
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
      if i == 1 and j == 0 then out[#out + 1] = b:clone() end
      local t = j / steps
      if j > 0 then out[#out + 1] = catmull(a, b, c, d, t) end
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
  local projected = p:clone()
  projected.y = origin.y - distance
  return projected, normal, surfaceType or 'unclassified', nil
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
    stats = { lanes = 0, usable = 0, invalid = 0, samples = 0, hits = 0, misses = 0 },
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
  self.stats.lanes = #data.lanes

  for i = 1, #data.lanes do
    local src = data.lanes[i]
    local lane = {
      id = src.id,
      name = src.name or ('Lane ' .. tostring(src.id or i)),
      loop = src.loop == true,
      role = src.role,
      priority = src.priority,
      speedLimit = src.speedLimit,
      params = src.params or {},
      source = src,
      points = {},
      normals = {},
      surfaceValid = {},
      surfaceTypes = {},
      valid = false,
    }
    local sourcePoints = {}
    if type(src.points) == 'table' then
      for j = 1, #src.points do
        local p = point3(src.points[j])
        if p then sourcePoints[#sourcePoints + 1] = p end
      end
    end
    local samples = resample(sourcePoints, lane.loop)
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
    end
    if lane.valid then
      self.stats.usable = self.stats.usable + 1
    else
      self.stats.invalid = self.stats.invalid + 1
    end
    self.lanes[#self.lanes + 1] = lane
  end

  self.loaded = true
  self.log('CSP_IMPORT_LOADED file=%s lanes=%d usable=%d invalid=%d samples=%d hits=%d misses=%d intersections=%d areas=%d surface_type_api=%s',
    tostring(self.filename), self.stats.lanes, self.stats.usable, self.stats.invalid,
    self.stats.samples, self.stats.hits, self.stats.misses, #self.intersections, #self.areas, tostring(Surfaces ~= nil))
  return true
end

return Importer
