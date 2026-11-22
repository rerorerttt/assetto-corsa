local function forceFullGas()
  local car = ac.getCar(0)
  if not car then return end

  -- Только когда включён автопилот (Ctrl+C)
  if not car.isAIControlled then return end

  -- 1. Пытаемся перехватить педали (работает и на AI, газ берётся как max)
  local controls = ac.overrideCarControls(0)
  if controls then
    controls.combineAxis = false
    controls.gas = 1
    controls.brake = 0
  end

  -- 2. Снимаем любой лимит газа у AI (на всякий случай)
  if physics and physics.setAIThrottleLimit then
    physics.setAIThrottleLimit(0, 1)
  end
end

function script.update(dt)
  forceFullGas()
end