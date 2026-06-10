function sysCall_init()
    local logPath = 'D:/face_screen_arm_build/import_face_screen_arm.log'
    local log = io.open(logPath, 'w')
    local function write(message)
        log:write(message .. '\n')
        log:flush()
    end

    local ok, err = pcall(function()
        local sim = require('sim')
        local simURDF = require('simURDF')

        local urdfPath = 'D:/face_screen_arm_build/generated/urdf/face_screen_support_arm.urdf'
        local modelPath = 'D:/CoppeliaSim_Edu_V4_10_0_rev0/models/robots/non-mobile/face_screen_support_arm.ttm'
        local scenePath = 'D:/face_screen_arm_build/face_screen_support_arm_scene.ttt'

        write('Importing URDF: ' .. urdfPath)
        local opts = 2 + 8 + 128
        local robotName = simURDF.import(urdfPath, opts)
        write('robotName=' .. tostring(robotName))

        local all = sim.getObjectsInTree(sim.handle_scene)
        local jointCount = 0
        local shapeCount = 0
        local modelBase = -1
        for i = 1, #all do
            local typ = sim.getObjectType(all[i])
            if typ == sim.sceneobject_joint then jointCount = jointCount + 1 end
            if typ == sim.sceneobject_shape then shapeCount = shapeCount + 1 end
            local isModel = false
            pcall(function()
                isModel = sim.getBoolProperty(all[i], 'modelBase')
            end)
            if isModel and modelBase == -1 then modelBase = all[i] end
        end

        write('objectCount=' .. tostring(#all))
        write('jointCount=' .. tostring(jointCount))
        write('shapeCount=' .. tostring(shapeCount))
        write('modelBase=' .. tostring(modelBase))

        if modelBase ~= -1 then
            sim.setObjectAlias(modelBase, 'face_screen_support_arm')
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
