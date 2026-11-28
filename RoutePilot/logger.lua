-- RoutePilot/logger.lua
-- Crash-resilient text log, file strictly in the app folder.
--
-- Guarantees:
--   * the file is created IMMEDIATELY (init is called at script load):
--     each candidate path is verified by an actual io.save write, so a wrong
--     path can never fail silently;
--   * candidates: __dirname, ac.dirname(), AC root + /apps/lua/RoutePilot,
--     working directory — the first one that accepts a write wins;
--   * flush() rewrites the whole buffer, so after even a hard crash the file
--     holds everything up to the last flush (call it every second);
--   * shutdown(reason) writes the SHUTDOWN marker for normal exits — its
--     absence together with the last heartbeat line marks a crash moment.

local Logger = {}
Logger.__index = Logger

local function addCandidate(dirs, dir)
  if type(dir) == 'string' and #dir > 0 then
    dirs[#dirs + 1] = dir
  end
end

function Logger.new(fileName, maxLines)
  local self = setmetatable({}, Logger)
  self.fileName = fileName or 'routepilot_log.txt'
  self.maxLines = maxLines or 2000
  self.lines = {}
  self.path = nil
  self.writeError = nil
  self.dirty = false
  return self
end

---Find a working path by actually writing to it. Returns the path or nil.
function Logger:init()
  if self.path then return self.path end

  local dirs = {}
  local okDn, dn = pcall(function() return __dirname end)
  if okDn then addCandidate(dirs, dn) end

  local okAd, ad = pcall(ac.dirname)
  if okAd then addCandidate(dirs, ad) end

  local okRt, rt = pcall(ac.getFolder, ac.FolderID.Root)
  if okRt and type(rt) == 'string' and #rt > 0 then
    addCandidate(dirs, rt .. '/apps/lua/RoutePilot')
  end

  addCandidate(dirs, '.') -- last resort: the game working directory

  local firstError = nil
  for i = 1, #dirs do
    local path = dirs[i]:gsub('\\', '/')
    if path:sub(#path) == '/' then path = path:sub(1, #path - 1) end
    path = path .. '/' .. self.fileName
    local ok, err = pcall(io.save, path,
      '-- RoutePilot log file, opened by the game --\n', true)
    if ok then
      self.path = path
      self.lines = {}
      self.writeError = nil
      self.dirty = false
      return path
    end
    if not firstError then firstError = tostring(err) end
  end

  self.writeError = firstError or 'no writable folder found'
  return nil
end

---Append one line ('printf'-style). Never throws. Auto-inits the path.
function Logger:log(msg, ...)
  if not self.path and not self.writeError then self:init() end
  if not self.path then return end
  local line
  if select('#', ...) > 0 then
    local okF, formatted = pcall(string.format, msg, ...)
    line = okF and formatted or tostring(msg)
  else
    line = tostring(msg)
  end
  local ts = 0
  local okS, sim = pcall(ac.getSim)
  if okS and sim then ts = sim.gameTime or 0 end
  self.lines[#self.lines + 1] = string.format('[%8.2fs] %s', ts, line)
  if #self.lines > self.maxLines then table.remove(self.lines, 1) end
  self.dirty = true
end

---Write the whole buffer to disk. Call at least every second.
function Logger:flush()
  if not self.dirty then return true end
  if not self.path then return false end
  local data = table.concat(self.lines, '\n') .. '\n'
  local ok, err = pcall(io.save, self.path, data, true)
  self.dirty = false
  if ok then
    self.writeError = nil
  else
    self.writeError = tostring(err)
  end
  return ok
end

---Heartbeat line: written every second; the last one marks a crash moment.
function Logger:heartbeat(extra)
  if extra then
    self:log('heartbeat | %s', extra)
  else
    self:log('heartbeat')
  end
end

---Normal-exit marker. Its ABSENCE in the file means the game crashed.
function Logger:shutdown(reason)
  self:log('SHUTDOWN: %s', tostring(reason or 'script released normally'))
  self:flush()
end

function Logger:clear()
  self.lines = {}
  self.dirty = true
  self:log('=== log cleared ===')
  return self:flush()
end

function Logger:count()
  return #self.lines
end

return Logger
