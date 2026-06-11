function sysCall_init()
    local logPath = 'D:/face_screen_arm_build/disable_screen_collision.log'
    local log = io.open(logPath, 'w')
    local function write(message)
        log:write(message .. '\n')
        log:flush()
    end

    local ok, err = pcall(function()
        local sim = require('sim')
        local scenePath = 'D:/face_screen_arm_build/face_screen_support_arm_scene.ttt'
        local modelPath = 'D:/CoppeliaSim_Edu_V4_10_0_rev0/models/robots/non-mobile/face_screen_support_arm.ttm'

        write('loadedScene=' .. scenePath .. '\tresult=' .. tostring(sim.loadScene(scenePath)))

        local modelBase = sim.getObject('/face_screen_support_arm', {noError = true})
        local all = sim.getObjectsInTree(sim.handle_scene)
        for i = 1, #all do
            local alias = sim.getObjectAlias(all[i])
            if string.find(alias or '', 'screen_pitch_link', 1, true) then
                pcall(function() sim.setBoolProperty(all[i], 'respondable', false) end)
                pcall(function() sim.setObjectInt32Param(all[i], sim.shapeintparam_respondable, 0) end)
                write('screenCollisionOff=' .. tostring(all[i]) .. '\t' .. tostring(alias))
            end
        end

        if modelBase and modelBase >= 0 then
            sim.saveModel(modelBase, modelPath)
            write('savedModel=' .. modelPath)
        end
        sim.saveScene(scenePath)
        write('savedScene=' .. scenePath)
        write('DONE')
    end)

    if not ok then write('ERROR: ' .. tostring(err)) end
    log:close()
    local sim = require('sim')
    sim.quitSimulator()
end
