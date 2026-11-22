-- Аркадный ИИ с правильной ориентацией машин и редактором трассы
-- Совместимость: CSP 0.3.2-preview2

local sim = ac.getSim()
local trackFolder = ac.getFolder(ac.FolderID.ContentTracks) .. '/' .. ac.getTrackID() .. '/'
local saveFilePath = trackFolder .. 'arcade_ai_path.lua'

local state = {
    isActive = false,
    isRecording = false,
    recordStep = 4.0,
    defaultHalfWidth = 5.0,
    maxSpeedKmh = 160.0,
    rubberBand = 0.3
}

local nodes = {}
local bots = {}

local function addNode(worldPos)
    table.insert(nodes, {
        pos = vec3(worldPos.x, worldPos.y + 0.1, worldPos.z),
        widthLeft = state.defaultHalfWidth,
        widthRight = state.defaultHalfWidth
    })
end

local function savePath()
    if #nodes == 0 then
        ac.setMessage("Аркадный ИИ", "Нет точек для сохранения!")
        return
    end
    local lines = { "return {" }
    for _, n in ipairs(nodes) do
        table.insert(lines, string.format(
            "  { pos = vec3(%.3f, %.3f, %.3f), widthLeft = %.2f, widthRight = %.2f },",
            n.pos.x, n.pos.y, n.pos.z, n.widthLeft, n.widthRight
        ))
    end
    table.insert(lines, "}")
    io.save(saveFilePath, table.concat(lines, "\n"))
    ac.setMessage("Аркадный ИИ", "Маршрут сохранен (" .. #nodes .. " точек)")
end

local function loadPath()
    if io.fileExists(saveFilePath) then
        local chunk = loadfile(saveFilePath)
        if chunk then
            local loaded = chunk()
            nodes = {}
            for _, item in ipairs(loaded) do
                table.insert(nodes, {
                    pos = item.pos,
                    widthLeft = item.widthLeft or state.defaultHalfWidth,
                    widthRight = item.widthRight or state.defaultHalfWidth
                })
            end
            ac.setMessage("Аркадный ИИ", "Загружено точек: " .. #nodes)
        end
    else
        ac.setMessage("Аркадный ИИ", "Файл траектории не найден. Запишите точки!")
    end
end

local function initBots()
    bots = {}
    for i = 1, sim.carsCount - 1 do
        local car = ac.getCar(i)
        if car then
            bots[i] = {
                pos = car.position:clone(),
                dir = car.look:clone(),
                speed = 0.0,
                targetNode = 1,
                laneOffset = (i % 3 - 1) * 2.0,
                skill = 0.9 + (i % 4) * 0.05,
                initialized = false
            }
        end
    end
end

local function releaseBots()
    if #nodes < 3 then
        ac.setMessage("Аркадный ИИ", "Нужно записать минимум 3 точки!")
        return
    end
    initBots()
    for i, bot in pairs(bots) do
        local startIdx = math.max(1, (#nodes - i * 2) % #nodes + 1)
        local nextIdx = startIdx % #nodes + 1
        local dir = (nodes[nextIdx].pos - nodes[startIdx].pos):normalize()
        local right = vec3(dir.z, 0, -dir.x)
        
        bot.pos = nodes[startIdx].pos + right * bot.laneOffset
        bot.dir = dir
        bot.speed = 15.0
        bot.targetNode = nextIdx
        bot.initialized = true
    end
    state.isActive = true
    ac.setMessage("Аркадный ИИ", "ИИ ботов запущен (вперед носом)!")
end

function script.update(dt)
    local player = ac.getCar(0)

    if state.isRecording and player then
        if #nodes == 0 or player.position:distance(nodes[#nodes].pos) >= state.recordStep then
            addNode(player.position)
        end
    end

    if not state.isActive or #nodes < 3 then return end

    if next(bots) == nil then
        initBots()
    end

    for i, bot in pairs(bots) do
        local car = ac.getCar(i)
        if car then
            if not bot.initialized then
                bot.pos = car.position:clone()
                bot.dir = car.look:clone()
                bot.initialized = true
            end

            local targetNode = nodes[bot.targetNode]
            local nextNode = nodes[bot.targetNode % #nodes + 1]
            local aimPoint = targetNode.pos

            local flatDist = vec2(bot.pos.x, bot.pos.z):distance(vec2(aimPoint.x, aimPoint.z))
            if flatDist < 8.0 then
                bot.targetNode = bot.targetNode % #nodes + 1
            end

            local targetSpeedKmh = state.maxSpeedKmh * bot.skill
            if player and state.rubberBand > 0 then
                local distToPlayer = bot.pos:distance(player.position)
                local toPlayer = (player.position - bot.pos):normalize()
                local isAhead = bot.dir:dot(toPlayer) < 0
                if isAhead and distToPlayer > 35.0 then
                    targetSpeedKmh = targetSpeedKmh * (1.0 - state.rubberBand * 0.4)
                elseif not isAhead and distToPlayer > 45.0 then
                    targetSpeedKmh = targetSpeedKmh * (1.0 + state.rubberBand * 0.5)
                end
            end

            local targetSpeedMs = targetSpeedKmh / 3.6
            if bot.speed < targetSpeedMs then
                bot.speed = math.min(bot.speed + 20.0 * dt, targetSpeedMs)
            else
                bot.speed = math.max(bot.speed - 30.0 * dt, targetSpeedMs)
            end

            -- Точный расчет направления вектора движения вперед носом
            local desiredDir = aimPoint - bot.pos
            desiredDir.y = 0
            if desiredDir:length() > 0.01 then
                desiredDir:normalize()
                bot.dir = math.lerp(bot.dir, desiredDir, math.clamp(6.0 * dt, 0, 1)):normalize()
            end

            bot.pos = bot.pos + bot.dir * bot.speed * dt

            if physics and physics.setCarPosition then
                physics.setCarPosition(i, bot.pos, bot.dir)
                physics.setCarVelocity(i, bot.dir * bot.speed)
            end
        end
    end
end

function windowMain(dt)
    ui.text("Точек траектории: " .. tostring(#nodes))
    ui.separator()

    -- Радар для визуального контроля
    local canvasSize = vec2(300, 160)
    if ui.invisibleButton("radar", canvasSize) then
        local player = ac.getCar(0)
        if player then
            addNode(player.position)
        end
    end
    
    local p0 = ui.getItemRectMin()
    local p1 = ui.getItemRectMax()
    ui.drawRectFilled(p0, p1, rgbm(0.1, 0.1, 0.12, 1), 4.0)
    ui.drawRect(p0, p1, rgbm(0.3, 0.3, 0.35, 1), 4.0)

    local player = ac.getCar(0)
    if player and #nodes > 0 then
        local centerPos = player.position
        local scale = 1.2
        local midX = (p0.x + p1.x) * 0.5
        local midY = (p0.y + p1.y) * 0.5

        for i = 1, #nodes do
            local curr = nodes[i]
            local nextN = nodes[i % #nodes + 1]
            
            local x1 = midX + (curr.pos.x - centerPos.x) * scale
            local y1 = midY + (curr.pos.z - centerPos.z) * scale
            local x2 = midX + (nextN.pos.x - centerPos.x) * scale
            local y2 = midY + (nextN.pos.z - centerPos.z) * scale

            if x1 >= p0.x and x1 <= p1.x and y1 >= p0.y and y1 <= p1.y then
                ui.drawLine(vec2(x1, y1), vec2(x2, y2), rgbm(1, 0.85, 0, 0.8), 2.0)
            end
        end

        ui.drawCircleFilled(vec2(midX, midY), 4.5, rgbm(0, 0.8, 1, 1), 12)
    end

    ui.separator()

    if ui.checkbox("1. Запись точек с движения", state.isRecording) then
        state.isRecording = not state.isRecording
    end

    if ui.button("2. Запустить ботов на трассу", vec2(-1, 30)) then
        releaseBots()
    end

    if ui.checkbox("Физика ИИ активна", state.isActive) then
        state.isActive = not state.isActive
    end

    ui.separator()
    state.maxSpeedKmh = ui.slider("Макс. скорость", state.maxSpeedKmh, 60, 280, "%.0f км/ч")
    state.rubberBand = ui.slider("Резиновая лента", state.rubberBand, 0.0, 1.0, "%.2f")

    ui.separator()
    if ui.button("Сохранить", vec2(90, 26)) then savePath() end
    ui.sameLine()
    if ui.button("Загрузить", vec2(90, 26)) then loadPath() end
    ui.sameLine()
    if ui.button("Очистить", vec2(-1, 26)) then
        nodes = {}
        state.isActive = false
    end
end

loadPath()