function sysCall_init()
    local logPath = 'D:/face_screen_arm_build/set_pose_120_120_30_and_fix_mount.log'
    local log = io.open(logPath, 'w')
    local function write(message)
        log:write(message .. '\n')
        log:flush()
    end

    local ok, err = pcall(function()
        local sim = require('sim')
        local scenePath = 'D:/face_screen_arm_build/face_screen_support_arm_scene.ttt'
        write('loadedScene=' .. scenePath .. '\tresult=' .. tostring(sim.loadScene(scenePath)))

        local function findByAlias(aliasWanted)
            local h = sim.getObject('/' .. aliasWanted, {noError = true})
            if h >= 0 then return h end
            local all = sim.getObjectsInTree(sim.handle_scene)
            for i = 1, #all do
                if sim.getObjectAlias(all[i]) == aliasWanted then return all[i] end
            end
            return -1
        end

        local function worldBounds(handle)
            local minx = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_min_x)
            local miny = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_min_y)
            local minz = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_min_z)
            local maxx = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_max_x)
            local maxy = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_max_y)
            local maxz = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_max_z)
            local m = sim.getObjectMatrix(handle, sim.handle_world)
            local b = {min = {math.huge, math.huge, math.huge}, max = {-math.huge, -math.huge, -math.huge}}
            for _, x in ipairs({minx, maxx}) do
                for _, y in ipairs({miny, maxy}) do
                    for _, z in ipairs({minz, maxz}) do
                        local p = sim.multiplyVector(m, {x, y, z})
                        for k = 1, 3 do
                            if p[k] < b.min[k] then b.min[k] = p[k] end
                            if p[k] > b.max[k] then b.max[k] = p[k] end
                        end
                    end
                end
            end
            return b
        end

        local function setJoint(alias, displayValue, prismatic)
            local h = findByAlias(alias)
            if h < 0 then error(alias .. ' was not found') end
            local v = prismatic and displayValue / 1000.0 or displayValue * math.pi / 180.0
            sim.setJointPosition(h, v)
            sim.setJointTargetPosition(h, v)
            write(string.format('setJoint=%s displayValue=%.3f', alias, displayValue))
        end

        local desk = findByAlias('office_desk_top')
        local base = findByAlias('base_link_respondable')
        local model = findByAlias('face_screen_support_arm')
        if desk < 0 then error('office_desk_top was not found') end
        if base < 0 then error('base_link_respondable was not found') end

        -- Set the requested computer-stand initial posture.
        setJoint('joint1_base_yaw', 0, false)
        setJoint('joint2_shoulder_pitch', -120, false)
        setJoint('joint3_elbow_pitch', 120, false)
        setJoint('joint4_telescopic', 30, true)
        setJoint('joint5_screen_pan', 0, false)
        setJoint('joint6_screen_pitch', 0, false)

        -- Put the real base object, not only the imported model marker, on the desktop.
        local deskTopZ = worldBounds(desk).max[3]
        local baseBounds = worldBounds(base)
        local baseCenterX = (baseBounds.min[1] + baseBounds.max[1]) / 2
        local baseCenterY = (baseBounds.min[2] + baseBounds.max[2]) / 2
        local targetX, targetY = 0.30, 0.00
        local dp = {targetX - baseCenterX, targetY - baseCenterY, deskTopZ - baseBounds.min[3]}

        local p = sim.getObjectPosition(base, sim.handle_world)
        p[1] = p[1] + dp[1]
        p[2] = p[2] + dp[2]
        p[3] = p[3] + dp[3]
        sim.setObjectPosition(base, sim.handle_world, p)
        sim.setObjectOrientation(base, sim.handle_world, {0, 0, 0})

        if model >= 0 then
            sim.setObjectPosition(model, sim.handle_world, {targetX, targetY, deskTopZ})
            sim.setObjectOrientation(model, sim.handle_world, {0, 0, 0})
        end

        local newBounds = worldBounds(base)
        write(string.format('deskTopZ=%.6f', deskTopZ))
        write(string.format('baseMoveDelta=%.6f %.6f %.6f', dp[1], dp[2], dp[3]))
        write(string.format('newBaseMinZ=%.6f', newBounds.min[3]))
        write(string.format('newBaseCenter=%.6f %.6f', (newBounds.min[1] + newBounds.max[1]) / 2, (newBounds.min[2] + newBounds.max[2]) / 2))

        sim.saveScene(scenePath)
        write('savedScene=' .. scenePath)
        write('DONE')
    end)

    if not ok then write('ERROR: ' .. tostring(err)) end
    log:close()
    local sim = require('sim')
    sim.quitSimulator()
end
