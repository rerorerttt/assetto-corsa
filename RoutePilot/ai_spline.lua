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

function M.encode(nodes, loop)
  if type(string.pack)~="function" then
    return nil,"string.pack_unavailable"
  end
  if type(nodes)~="table" or #nodes<3 then
    return nil,"route_needs_at_least_3_points"
  end

  local count=#nodes
  local cumulative={}
  local total=0
  for i=1,count do
    local p=nodes[i]
    if not p or not finite(p.x) or not finite(p.y) or not finite(p.z) then
      return nil,"invalid_route_point_"..tostring(i)
    end
    if i>1 then
      local d=distance(nodes[i-1],p)
      if d<0.05 then return nil,"degenerate_segment_"..tostring(i-1) end
      total=total+d
    end
    cumulative[i]=total
  end
  if loop then
    local closing=distance(nodes[count],nodes[1])
    if closing<0.05 then return nil,"degenerate_closing_segment" end
    total=total+closing
  end
  if total<10 then return nil,"route_too_short" end

  local chunks={}
  chunks[#chunks+1]=string.pack("<i4i4i4i4",7,count,0,0)
  for i=1,count do
    local p=nodes[i]
    chunks[#chunks+1]=string.pack("<ffffi4",
      p.x,p.y,p.z,cumulative[i],i-1)
  end

  -- AiPointExtra: 18 little-endian floats. A generous symmetric corridor is
  -- intentional: RoutePilot's own recorded boundaries remain authoritative.
  chunks[#chunks+1]=string.pack("<i4",count)
  for i=1,count do
    local prev=nodes[i==1 and (loop and count or 1) or i-1]
    local next=nodes[i==count and (loop and 1 or count) or i+1]
    local tx,tz=normalized(next.x-prev.x,next.z-prev.z)
    local direction=math.atan2(tz,tx)
    local length=cumulative[i]
    chunks[#chunks+1]=string.pack("<18f",
      60.0, 1.0, 0.0, 0.0, 1.5, 3.5, 3.5, 0.0, direction,
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
  return true,nil,meta,#data
end

return M
