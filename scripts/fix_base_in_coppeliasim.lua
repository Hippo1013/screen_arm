function sysCall_init()
    local logPath = 'D:/face_screen_arm_build/fix_base.log'
    local log = io.open(logPath, 'w')
    local function write(message)
        log:write(message .. '\n')
        log:flush()
    end

    local ok, err = pcall(function()
        local sim = require('sim')
        local scenePath = 'D:/face_screen_arm_build/face_screen_support_arm_scene.ttt'
        local modelPath = 'D:/CoppeliaSim_Edu_V4_10_0_rev0/models/robots/non-mobile/face_screen_support_arm.ttm'

        local loaded = sim.loadScene(scenePath)
        write('loadedScene=' .. scenePath .. '\tresult=' .. tostring(loaded))

        local all = sim.getObjectsInTree(sim.handle_scene)
        local baseObjects = {}
        local modelBase = -1
        for i = 1, #all do
            local alias = sim.getObjectAlias(all[i])
            write(tostring(all[i]) .. '\t' .. tostring(alias) .. '\t' .. tostring(sim.getObjectType(all[i])))
            if alias == 'base_link' or string.find(alias or '', 'base_link', 1, true) then
                baseObjects[#baseObjects + 1] = all[i]
            end
            local isModel = false
            pcall(function()
                isModel = sim.getBoolProperty(all[i], 'modelBase')
            end)
            if isModel and modelBase == -1 then
                modelBase = all[i]
            end
        end

        if #baseObjects == 0 then
            error('base_link was not found')
        end

        for i = 1, #baseObjects do
            local base = baseObjects[i]
            local alias = sim.getObjectAlias(base)
            local isRespondableShape = string.find(alias or '', '_respondable', 1, true) ~= nil
            pcall(function() sim.setBoolProperty(base, 'dynamic', false) end)
            pcall(function() sim.setBoolProperty(base, 'respondable', isRespondableShape) end)
            pcall(function() sim.setBoolProperty(base, 'setToDynamicWithParent', false) end)
            pcall(function() sim.setObjectInt32Param(base, sim.shapeintparam_static, 1) end)
            pcall(function() sim.setObjectInt32Param(base, sim.shapeintparam_respondable, isRespondableShape and 1 or 0) end)
            write('fixedBaseHandle=' .. tostring(base) .. '\t' .. tostring(alias) .. '\trespondable=' .. tostring(isRespondableShape))
        end
        write('modelBase=' .. tostring(modelBase))

        if modelBase ~= -1 then
            sim.saveModel(modelBase, modelPath)
            write('savedModel=' .. modelPath)
        end
        sim.saveScene(scenePath)
        write('savedScene=' .. scenePath)
        write('DONE')
    end)

    if not ok then
        write('ERROR: ' .. tostring(err))
    end
    log:close()
    local sim = require('sim')
    sim.quitSimulator()
end
