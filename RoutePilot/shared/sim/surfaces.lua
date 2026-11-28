-- CSP track-surface type helper, based on the public acc-lua-internal shared library.
-- The native query reads the physical track surface and does not inspect KN5 meshes.

local surfaces = {}

function surfaces.raycastType(pos, dir, length)
  if not __util or type(__util.native) ~= 'function' then return nil end
  local ok, value = pcall(__util.native, 'physics.raycastTrackSurface.type', pos, dir, length)
  if not ok or value == nil then return nil end
  if value == 1 then return 'extraturf' end
  if value == 2 then return 'grass' end
  if value == 3 then return 'gravel' end
  if value == 4 then return 'kerb' end
  if value == 5 then return 'old' end
  if value == 6 then return 'sand' end
  return 'default'
end

return surfaces
