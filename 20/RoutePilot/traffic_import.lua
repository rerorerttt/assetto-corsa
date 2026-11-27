-- Embedded JSON decoder: RoutePilot is self-contained and does not depend on a separately installed json.lua.
local json = (function()
--
-- json.lua
--
-- Copyright (c) 2020 rxi
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy of
-- this software and associated documentation files (the "Software"), to deal in
-- the Software without restriction, including without limitation the rights to
-- use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
-- of the Software, and to permit persons to whom the Software is furnished to do
-- so, subject to the following conditions:
--
-- The above copyright notice and this permission notice shall be included in all
-- copies or substantial portions of the Software.
--
-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
-- AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
-- OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
-- SOFTWARE.
--

local json = { _version = "0.1.2" }

-------------------------------------------------------------------------------
-- Encode
-------------------------------------------------------------------------------

local encode

local escape_char_map = {
  [ "\\" ] = "\\",
  [ "\"" ] = "\"",
  [ "\b" ] = "b",
  [ "\f" ] = "f",
  [ "\n" ] = "n",
  [ "\r" ] = "r",
  [ "\t" ] = "t",
}

local escape_char_map_inv = { [ "/" ] = "/" }
for k, v in pairs(escape_char_map) do
  escape_char_map_inv[v] = k
end


local function escape_char(c)
  return "\\" .. (escape_char_map[c] or string.format("u%04x", c:byte()))
end


local function encode_nil(val)
  return "null"
end


local function encode_table(val, stack)
  local res = {}
  stack = stack or {}

  -- Circular reference?
  if stack[val] then error("circular reference") end

  stack[val] = true

  if rawget(val, 1) ~= nil or next(val) == nil then
    -- Treat as array -- check keys are valid and it is not sparse
    local n = 0
    for k in pairs(val) do
      if type(k) ~= "number" then
        error("invalid table: mixed or invalid key types: key=" .. tostring(k))
      end
      n = n + 1
    end
    if n ~= #val then
      error("invalid table: sparse array")
    end
    -- Encode
    for i, v in ipairs(val) do
      table.insert(res, encode(v, stack))
    end
    stack[val] = nil
    return "[" .. table.concat(res, ",") .. "]"

  else
    -- Treat as an object
    for k, v in pairs(val) do
      if type(k) ~= "string" then
        error("invalid table: mixed or invalid key types")
      end
      table.insert(res, encode(k, stack) .. ":" .. encode(v, stack))
    end
    stack[val] = nil
    return "{" .. table.concat(res, ",") .. "}"
  end
end


local function encode_string(val)
  return '"' .. val:gsub('[%z\1-\31\\"]', escape_char) .. '"'
end


local function encode_number(val)
  -- Check for NaN, -inf and inf
  if val ~= val or val <= -math.huge or val >= math.huge then
    error("unexpected number value '" .. tostring(val) .. "'")
  end
  return string.format("%.14g", val)
end

local type_func_map = {
  [ "nil"     ] = encode_nil,
  [ "table"   ] = encode_table,
  [ "string"  ] = encode_string,
  [ "number"  ] = encode_number,
  [ "boolean" ] = tostring,
}


encode = function(val, stack)
  local t = type(val)
  local f = type_func_map[t]
  if f then
    return f(val, stack)
  end
  error("unexpected type '" .. t .. "'")
end


function json.encode(val)
  return ( encode(val) )
end


-------------------------------------------------------------------------------
-- Decode
-------------------------------------------------------------------------------

local parse

local function create_set(...)
  local res = {}
  for i = 1, select("#", ...) do
    res[ select(i, ...) ] = true
  end
  return res
end

local space_chars   = create_set(" ", "\t", "\r", "\n")
local delim_chars   = create_set(" ", "\t", "\r", "\n", "]", "}", ",")
local escape_chars  = create_set("\\", "/", '"', "b", "f", "n", "r", "t", "u")
local literals      = create_set("true", "false", "null")

local literal_map = {
  [ "true"  ] = true,
  [ "false" ] = false,
  [ "null"  ] = nil,
}


local function next_char(str, idx, set, negate)
  for i = idx, #str do
    if set[str:sub(i, i)] ~= negate then
      return i
    end
  end
  return #str + 1
end


local function decode_error(str, idx, msg)
  local line_count = 1
  local col_count = 1
  for i = 1, idx - 1 do
    col_count = col_count + 1
    if str:sub(i, i) == "\n" then
      line_count = line_count + 1
      col_count = 1
    end
  end
  error( string.format("%s at line %d col %d", msg, line_count, col_count) )
end


local function codepoint_to_utf8(n)
  -- http://scripts.sil.org/cms/scripts/page.php?site_id=nrsi&id=iws-appendixa
  local f = math.floor
  if n <= 0x7f then
    return string.char(n)
  elseif n <= 0x7ff then
    return string.char(f(n / 64) + 192, n % 64 + 128)
  elseif n <= 0xffff then
    return string.char(f(n / 4096) + 224, f(n % 4096 / 64) + 128, n % 64 + 128)
  elseif n <= 0x10ffff then
    return string.char(f(n / 262144) + 240, f(n % 262144 / 4096) + 128,
                       f(n % 4096 / 64) + 128, n % 64 + 128)
  end
  error( string.format("invalid unicode codepoint '%x'", n) )
end


local function parse_unicode_escape(s)
  local n1 = tonumber( s:sub(1, 4),  16 )
  local n2 = tonumber( s:sub(7, 10), 16 )
   -- Surrogate pair?
  if n2 then
    return codepoint_to_utf8((n1 - 0xd800) * 0x400 + (n2 - 0xdc00) + 0x10000)
  else
    return codepoint_to_utf8(n1)
  end
end


local function parse_string(str, i)
  local res = ""
  local j = i + 1
  local k = j

  while j <= #str do
    local x = str:byte(j)

    if x < 32 then
      decode_error(str, j, "control character in string")

    elseif x == 92 then -- `\`: Escape
      res = res .. str:sub(k, j - 1)
      j = j + 1
      local c = str:sub(j, j)
      if c == "u" then
        local hex = str:match("^[dD][89aAbB]%x%x\\u%x%x%x%x", j + 1)
                 or str:match("^%x%x%x%x", j + 1)
                 or decode_error(str, j - 1, "invalid unicode escape in string")
        res = res .. parse_unicode_escape(hex)
        j = j + #hex
      else
        if not escape_chars[c] then
          decode_error(str, j - 1, "invalid escape char '" .. c .. "' in string")
        end
        res = res .. escape_char_map_inv[c]
      end
      k = j + 1

    elseif x == 34 then -- `"`: End of string
      res = res .. str:sub(k, j - 1)
      return res, j + 1
    end

    j = j + 1
  end

  decode_error(str, i, "expected closing quote for string")
end


local function parse_number(str, i)
  local x = next_char(str, i, delim_chars)
  local s = str:sub(i, x - 1)
  local n = tonumber(s)
  if not n then
    decode_error(str, i, "invalid number '" .. s .. "'")
  end
  return n, x
end


local function parse_literal(str, i)
  local x = next_char(str, i, delim_chars)
  local word = str:sub(i, x - 1)
  if not literals[word] then
    decode_error(str, i, "invalid literal '" .. word .. "'")
  end
  return literal_map[word], x
end


local function parse_array(str, i)
  local res = {}
  local n = 1
  i = i + 1
  while 1 do
    local x
    i = next_char(str, i, space_chars, true)
    -- Empty / end of array?
    if str:sub(i, i) == "]" then
      i = i + 1
      break
    end
    -- Read token
    x, i = parse(str, i)
    res[n] = x
    n = n + 1
    -- Next token
    i = next_char(str, i, space_chars, true)
    local chr = str:sub(i, i)
    i = i + 1
    if chr == "]" then break end
    if chr ~= "," then decode_error(str, i, "expected ']' or ','") end
  end
  return res, i
end


local function parse_object(str, i)
  local res = {}
  i = i + 1
  while 1 do
    local key, val
    i = next_char(str, i, space_chars, true)
    -- Empty / end of object?
    if str:sub(i, i) == "}" then
      i = i + 1
      break
    end
    -- Read key
    if str:sub(i, i) ~= '"' then
      decode_error(str, i, "expected string for key")
    end
    key, i = parse(str, i)
    -- Read ':' delimiter
    i = next_char(str, i, space_chars, true)
    if str:sub(i, i) ~= ":" then
      decode_error(str, i, "expected ':' after key")
    end
    i = next_char(str, i + 1, space_chars, true)
    -- Read value
    val, i = parse(str, i)
    -- Set
    res[key] = val
    -- Next token
    i = next_char(str, i, space_chars, true)
    local chr = str:sub(i, i)
    i = i + 1
    if chr == "}" then break end
    if chr ~= "," then decode_error(str, i, "expected '}' or ','") end
  end
  return res, i
end


local char_func_map = {
  [ '"' ] = parse_string,
  [ "0" ] = parse_number,
  [ "1" ] = parse_number,
  [ "2" ] = parse_number,
  [ "3" ] = parse_number,
  [ "4" ] = parse_number,
  [ "5" ] = parse_number,
  [ "6" ] = parse_number,
  [ "7" ] = parse_number,
  [ "8" ] = parse_number,
  [ "9" ] = parse_number,
  [ "-" ] = parse_number,
  [ "t" ] = parse_literal,
  [ "f" ] = parse_literal,
  [ "n" ] = parse_literal,
  [ "[" ] = parse_array,
  [ "{" ] = parse_object,
}


parse = function(str, idx)
  local chr = str:sub(idx, idx)
  local f = char_func_map[chr]
  if f then
    return f(str, idx)
  end
  decode_error(str, idx, "unexpected character '" .. chr .. "'")
end


function json.decode(str)
  if type(str) ~= "string" then
    error("expected argument of type string, got " .. type(str))
  end
  local res, idx = parse(str, next_char(str, 1, space_chars, true))
  idx = next_char(str, idx, space_chars, true)
  if idx <= #str then
    decode_error(str, idx, "trailing garbage")
  end
  return res
end


return json
end)()

local M = {}
local MAX_POINTS = 50000
local CONNECT_DISTANCE = 8.0

local function finite(v)
  return type(v) == 'number' and v == v and v ~= math.huge and v ~= -math.huge
end

local function pointFromTable(p)
  if type(p) ~= 'table' then return nil end
  local x = p.x or p[1]
  local y = p.y or p[2]
  local z = p.z or p[3]
  if not finite(x) or not finite(y) or not finite(z) then return nil end
  return vec3(x, y, z)
end

local function flatDistance(a, b)
  local dx, dz = b.x - a.x, b.z - a.z
  return math.sqrt(dx * dx + dz * dz)
end

local function normalized(x, z)
  local length = math.sqrt(x * x + z * z)
  if length < 0.00001 then return 0, 1 end
  return x / length, z / length
end

local function decodeLanes(data)
  if type(data) ~= 'table' or type(data.lanes) ~= 'table' then
    return nil, 'traffic.json has no lanes array'
  end
  local lanes = {}
  for i = 1, #data.lanes do
    local source = data.lanes[i]
    if type(source) == 'table' and type(source.points) == 'table' then
      local points = {}
      for j = 1, #source.points do
        local p = pointFromTable(source.points[j])
        if p then points[#points + 1] = p end
        if #points >= MAX_POINTS then break end
      end
      if #points >= (source.loop == true and 3 or 2) then
        lanes[#lanes + 1] = {
          id = source.id or i,
          name = source.name or ('Lane ' .. tostring(source.id or i)),
          points = points,
          loop = source.loop == true
        }
      end
    end
  end
  if #lanes == 0 then return nil, 'traffic.json contains no usable directed lanes' end
  return lanes
end

local function projectToLane(lane, position, hx, hz)
  local points = lane.points
  local segmentCount = lane.loop and #points or (#points - 1)
  local best = nil
  for i = 1, segmentCount do
    local a = points[i]
    local b = points[i % #points + 1]
    local vx, vz = b.x - a.x, b.z - a.z
    local lenSq = vx * vx + vz * vz
    if lenSq > 0.0001 then
      local t = math.max(0, math.min(1,
        ((position.x - a.x) * vx + (position.z - a.z) * vz) / lenSq))
      local px, pz = a.x + vx * t, a.z + vz * t
      local dx, dz = position.x - px, position.z - pz
      local distanceSq = dx * dx + dz * dz
      local tx, tz = normalized(vx, vz)
      local alignment = hx * tx + hz * tz
      -- Direction is never reversed: a lane facing away from the player
      -- cannot win over a lane whose authored direction matches the player.
      local score = distanceSq + (1 - alignment) * 100
      if alignment >= -0.05 and (not best or score < best.score) then
        best = { score = score, distance = math.sqrt(distanceSq),
          segment = i, t = t, tx = tx, tz = tz }
      end
    end
  end
  return best
end

local function appendPoint(out, p)
  local last = out[#out]
  if not last or flatDistance(last, p) > 0.05 then
    out[#out + 1] = vec3(p.x, p.y, p.z)
  end
end

local function buildDirectedChain(lanes, player)
  local hx, hz = normalized(player.look.x, player.look.z)
  local startIndex, startProjection, startScore = nil, nil, math.huge
  for i = 1, #lanes do
    local projection = projectToLane(lanes[i], player.position, hx, hz)
    if projection and projection.score < startScore then
      startIndex, startProjection, startScore = i, projection, projection.score
    end
  end
  if not startIndex then return nil, nil, 'no lane matches player heading' end

  local nodes = {}
  local visited = {}
  local currentIndex = startIndex
  local firstPoint = nil
  local loopRoute = false
  local chainCount = 0

  while currentIndex and not visited[currentIndex] and #nodes < MAX_POINTS do
    local lane = lanes[currentIndex]
    visited[currentIndex] = true
    chainCount = chainCount + 1
    local points = lane.points
    local firstSegment = currentIndex == startIndex and startProjection.segment or 1
    local firstT = currentIndex == startIndex and startProjection.t or 0
    if currentIndex ~= startIndex then appendPoint(nodes, points[1]) end
    local a = points[firstSegment]
    local b = points[firstSegment % #points + 1]
    if currentIndex == startIndex then
      local start = vec3(a.x + (b.x-a.x)*firstT,
        a.y + (b.y-a.y)*firstT, a.z + (b.z-a.z)*firstT)
      appendPoint(nodes, start)
      firstPoint = start
    end

    local segmentCount = lane.loop and #points or (#points - firstSegment)
    local segment = firstSegment
    local traversed = 0
    while traversed < segmentCount and #nodes < MAX_POINTS do
      local nextIndex = lane.loop and (segment % #points + 1) or (segment + 1)
      if not (lane.loop and nextIndex == firstSegment) then
        appendPoint(nodes, points[nextIndex])
      end
      segment = nextIndex
      traversed = traversed + 1
      if lane.loop and segment == firstSegment then
        loopRoute = true
        currentIndex = nil
        break
      end
    end
    if currentIndex == nil then break end
    if lane.loop then
      loopRoute = true
      break
    end

    local endpoint = nodes[#nodes]
    local prev = nodes[#nodes - 1]
    if not endpoint or not prev then break end
    local dx, dz = normalized(endpoint.x - prev.x, endpoint.z - prev.z)
    local bestIndex, bestScore = nil, math.huge
    for candidateIndex = 1, #lanes do
      if not visited[candidateIndex] then
        local candidate = lanes[candidateIndex]
        local start = candidate.points[1]
        local distance = flatDistance(endpoint, start)
        if distance <= CONNECT_DISTANCE then
          local nextPoint = candidate.points[2]
          local cx, cz = normalized(nextPoint.x - start.x, nextPoint.z - start.z)
          local alignment = dx * cx + dz * cz
          if alignment > 0.35 then
            local score = distance * 4 + (1 - alignment) * 10
            if score < bestScore then
              bestScore, bestIndex = score, candidateIndex
            end
          end
        end
      end
    end
    if not bestIndex then
      -- A directed lane can close only by reaching the original start point
      -- in the same authored direction. No lane is ever reversed to force a loop.
      if firstPoint and flatDistance(endpoint, firstPoint) <= CONNECT_DISTANCE then
        loopRoute = true
      end
      break
    end
    currentIndex = bestIndex
  end

  if #nodes < 2 then return nil, nil, 'directed lane chain is too short' end
  return nodes, loopRoute, string.format(
    'lane=%s chain=%d points=%d loop=%s start_error=%.2f',
    tostring(lanes[startIndex].id), chainCount, #nodes,
    tostring(loopRoute), startProjection.distance)
end

function M.load(log)
  local function trace(message,...)
    if type(log)=="function" then pcall(log,message,...) end
  end
  trace("IMPORT_STAGE begin")
  if not ac or type(ac.getTrackDataFilename) ~= 'function' then
    trace("IMPORT_STAGE fail reason=getTrackDataFilename_unavailable")
    return nil, nil, 'ac.getTrackDataFilename unavailable'
  end
  if not io or type(io.load) ~= 'function' then
    trace("IMPORT_STAGE fail reason=io_load_unavailable")
    return nil, nil, 'CSP io.load unavailable'
  end
  local okPath, path = pcall(ac.getTrackDataFilename, 'traffic.json')
  trace("IMPORT_PATH ok=%s path=%s",tostring(okPath),tostring(path))
  if not okPath or type(path) ~= 'string' or path == '' then
    trace("IMPORT_STAGE fail reason=traffic_path_unavailable")
    return nil, nil, 'traffic.json path unavailable'
  end
  if not io or type(io.load) ~= 'function' then
    trace("IMPORT_STAGE fail reason=io_load_unavailable")
    return nil, nil, 'CSP io.load unavailable'
  end
  local okRead, raw = pcall(io.load, path)
  trace("IMPORT_READ ok=%s type=%s bytes=%s",tostring(okRead),type(raw),type(raw)=="string" and #raw or -1)
  if not okRead or type(raw) ~= 'string' or raw == '' then
    trace("IMPORT_STAGE fail reason=traffic_file_unreadable value=%s",tostring(raw))
    return nil, nil, 'traffic.json not found or unreadable: ' .. tostring(raw)
  end
  local okDecode, data = pcall(json.decode, raw)
  if not okDecode then
    trace("IMPORT_STAGE fail reason=json_decode error=%s",tostring(data))
    return nil, nil, 'traffic.json parse error: ' .. tostring(data)
  end
  trace("IMPORT_JSON_OK lanes_field_type=%s",type(data and data.lanes))
  local lanes, laneError = decodeLanes(data)
  if not lanes then
    trace("IMPORT_STAGE fail reason=lane_decode error=%s",tostring(laneError))
    return nil, nil, laneError
  end
  local pointTotal=0
  for i=1,#lanes do pointTotal=pointTotal+#lanes[i].points end
  trace("IMPORT_LANES_OK lanes=%d points=%d",#lanes,pointTotal)
  local player = ac.getCar(0)
  if not player or not player.position or not player.look then
    trace("IMPORT_STAGE defer reason=player_car_not_ready")
    return nil, nil, 'player car unavailable during import'
  end
  trace("IMPORT_PLAYER pos=(%.2f,%.2f,%.2f) heading=(%.3f,%.3f)",
    player.position.x,player.position.y,player.position.z,player.look.x,player.look.z)
  local nodes,loop,info=buildDirectedChain(lanes, player)
  if nodes then
    trace("IMPORT_CHAIN_OK points=%d loop=%s info=%s",#nodes,tostring(loop),tostring(info))
  else
    trace("IMPORT_CHAIN_FAIL reason=%s",tostring(info))
  end
  return nodes,loop,info
end

return M
