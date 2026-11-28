-- RoutePilot/ai_spline.lua
-- Builds a minimal, valid AC v7 AI spline from RoutePilot's saved world route.
-- The spline is assigned per AI car through CSP physics.setAISpline(), so the
-- native controller no longer has the track's unrelated fast_lane as its path.

local M = {}

local function finite(v)
  return type(v)=="number" and v==v and v~=math.huge and v~=-math.huge
end

local function distance(a,b)
  local x,y,z=b.x-a.x,b.y-a.y,b.z-a.z
  return math.sqrt(x*x+y*y+z*z)
end

local function normalized(x,z)
  local l=math.sqrt(x*x+z*z)
  if l<1e-6 then return 0,1 end
  return x/l,z/l
end

local function curveRadius(a,b,c)
  local function flat(a,b)
    local x,z=b.x-a.x,b.z-a.z
    return math.sqrt(x*x+z*z)
  end
  local ab=flat(a,b)
  local bc=flat(b,c)
  local ac=flat(a,c)
  local cross=math.abs((b.x-a.x)*(c.z-a.z)-(b.z-a.z)*(c.x-a.x))
  if cross<1e-5 then return 100000.0 end
  return math.max(1.0,math.min(100000.0,ab*bc*ac/(2.0*cross)))
end

function M.encode(nodes, loop)
  if type(string.pack)~="function" then
    return nil,"string.pack_unavailable"
  end
  if type(nodes)~="table" or #nodes<3 then
    return nil,"route_needs_at_least_3_points"
  end
  -- AC v7 AI splines are intrinsically cyclic: the file format has no
  -- open/closed flag. Assigning an open RoutePilot fragment makes native AI
  -- wrap from its last point to its first through unrelated world space.
  if loop~=true then
    return nil,"AC_AI_SPLINE_REQUIRES_CLOSED_LOOP"
  end

  local points={}
  for i=1,#nodes do
    local p=nodes[i]
    if not p or not finite(p.x) or not finite(p.y) or not finite(p.z) then
      return nil,"invalid_route_point_"..tostring(i)
    end
    if #points==0 or distance(points[#points],p)>=0.05 then
      points[#points+1]=p
    end
  end
  if loop and #points>3 and distance(points[#points],points[1])<0.05 then
    points[#points]=nil
  end
  local count=#points
  if count<3 then return nil,"route_needs_3_distinct_points" end

  local cumulative={}
  local total=0
  for i=1,count do
    if i>1 then total=total+distance(points[i-1],points[i]) end
    cumulative[i]=total
  end
  if loop then
    local closing=distance(points[count],points[1])
    if closing<0.05 then return nil,"degenerate_closing_segment" end
    total=total+closing
  end
  if total<10 then return nil,"route_too_short" end

  local chunks={}
  chunks[#chunks+1]=string.pack("<i4i4i4i4",7,count,0,0)
  for i=1,count do
    local p=points[i]
    chunks[#chunks+1]=string.pack("<ffffi4",
      p.x,p.y,p.z,cumulative[i],i-1)
  end

  -- AiPointExtra: 18 little-endian floats. A generous symmetric corridor is
  -- intentional: RoutePilot's own recorded boundaries remain authoritative.
  chunks[#chunks+1]=string.pack("<i4",count)
  for i=1,count do
    local prev=points[i==1 and (loop and count or 1) or i-1]
    local next=points[i==count and (loop and 1 or count) or i+1]
    local tx,tz=normalized(next.x-prev.x,next.z-prev.z)
    local radius=100000.0
    if loop or (i>1 and i<count) then
      local a=points[i==1 and count or i-1]
      local c=points[i==count and 1 or i+1]
      radius=curveRadius(a,points[i],c)
    end
    local length=cumulative[i]
    -- AiPointExtra.Direction is a camber-side sign (+/-1), not a yaw angle.
    -- The forward vector carries the actual world-space route direction.
    chunks[#chunks+1]=string.pack("<ffffffffffffffffff",
      60.0, 1.0, 0.0, 0.0, radius, 3.5, 3.5, 0.0, 1.0,
      0.0, 1.0, 0.0, length, tx, 0.0, tz, 0.0, 0.0)
  end

  -- Grid is optional in the AC v7 format (AiSpline.HasGrid=false).
  chunks[#chunks+1]=string.pack("<i4",0)
  return table.concat(chunks),nil,{count=count,length=total,version=7}
end

function M.write(path,nodes,loop)
  local data,err,meta=M.encode(nodes,loop)
  if not data then return false,err end
  if type(io)~="table" or type(io.save)~="function" then
    return false,"io_save_unavailable"
  end
  local ok,result=pcall(io.save,path,data,true)
  if not ok then return false,tostring(result) end
  if result==false then return false,"io_save_returned_false" end
  local verifyOk,verified=pcall(io.load,path)
  if not verifyOk or type(verified)~="string" or #verified~=#data then
    return false,"ai_file_readback_failed"
  end
  if verified~=data then return false,"ai_file_readback_mismatch" end
  return true,nil,meta,#data
end

return M
