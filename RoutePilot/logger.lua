-- RoutePilot/logger.lua
-- Text log for remote diagnosis. File: apps/lua/RoutePilot/routepilot_log.txt
-- Lines are kept in memory (capped) and flushed to disk with io.save().

local Logger = {}
Logger.__index = Logger

local function appDir()
  local ok, dir = pcall(function() return __dirname end)
  if ok and type(dir) == 'string' and dir ~= '' then return dir end
  local ok2, dir2 = pcall(ac.dirname)
  if ok2 and type(dir2) == 'string' and dir2 ~= '' then return dir2 end
  return '.'
end

function Logger.new(fileName, maxLines)
  local self = setmetatable({}, Logger)
  self.path = appDir() .. '/' .. (fileName or 'routepilot_log.txt')
  self.lines = {}
  self.maxLines = maxLines or 800
  self.enabled = true
  self.dirty = false
  self.writeError = nil
  return self
end

---Append one line ('printf'-style). Never throws.
function Logger:log(msg, ...)
  if not self.enabled then return end
  local line
  if select('#', ...) > 0 then
    local ok, formatted = pcall(string.format, msg, ...)
    line = ok and formatted or tostring(msg)
  else
    line = tostring(msg)
  end
  self.lines[#self.lines + 1] = line
  if #self.lines > self.maxLines then table.remove(self.lines, 1) end
  self.dirty = true
end

---Write the buffer to disk (rewrites the file; parent folder is ensured).
function Logger:flush()
  if not self.dirty then return true end
  local data = table.concat(self.lines, '\n')
  local ok, err = pcall(io.save, self.path, data, true)
  self.dirty = false
  if ok then
    self.writeError = nil
  else
    self.writeError = tostring(err)
  end
  return ok
end

function Logger:clear()
  self.lines = {}
  self.dirty = true
  return self:flush()
end

function Logger:count()
  return #self.lines
end

return Logger
